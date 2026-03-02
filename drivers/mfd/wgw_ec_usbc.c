// SPDX-License-Identifier: GPL-2.0
/*
 * USB Type-C support driver for the Wifx board EC
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>

#include <linux/mfd/wgw-ec/core.h>
#include <linux/mfd/wgw-ec/reg.h>
#include <linux/mfd/wgw-ec/usbc.h>

#define DRV_NAME "wgw-ec-usbc"

#define REG_INTERRUPT_DATA_MODE_CHANGE 0x01
#define REG_INTERRUPT_POWER_MODE_CHANGE 0x02

enum wgw_ec_usb_power_mode {
	WGW_EC_USB_POWER_MODE_DETACHED,
	WGW_EC_USB_POWER_MODE_SOURCE,
	WGW_EC_USB_POWER_MODE_SINK,
	WGW_EC_USB_POWER_MODE_ERROR,
};

enum wgw_ec_usb_data_mode {
	WGW_EC_USB_DATA_MODE_DEVICE,
	WGW_EC_USB_DATA_MODE_HOST,
	WGW_EC_USB_DATA_MODE_ERROR,
};

struct wgw_ec_usbc_dev {
	struct device *dev;
	struct wgw_ec_core *core;

	/* notification from wgw-ec-dev */
	struct notifier_block notifier;

	struct typec_port *port;
	struct typec_partner *partner;
	struct typec_capability typec_cap;
	struct blocking_notifier_head notifier_list;

	/* cache */
	spinlock_t lock;
	enum typec_data_role data_mode;
	enum typec_role pwr_role;
	bool attached;
};

static const struct of_device_id wgw_ec_usbc_of_match[] = {
	{
		.compatible = "wifx,wgw-ec-usbc",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, wgw_ec_usbc_of_match);

static const char *power_state_to_str(bool attached, enum typec_role pwr_role)
{
	if (!attached)
		return "detached";
	if (pwr_role == TYPEC_SOURCE)
		return "source";
	if (pwr_role == TYPEC_SINK)
		return "sink";
	return "error";
}

static const char *data_mode_to_str(enum typec_data_role mode)
{
	switch (mode) {
	case TYPEC_HOST:
		return "host";
	case TYPEC_DEVICE:
		return "device";
	default:
		return "error";
	}
}

static const char *const typec_roles[] = {
	[TYPEC_SINK] = "sink",
	[TYPEC_SOURCE] = "source",
};

static const char *typec_role_string(enum typec_role role)
{
	if (role < 0 || role >= ARRAY_SIZE(typec_roles))
		return "unknown";

	return typec_roles[role];
}

static const char *const typec_data_roles[] = {
	[TYPEC_DEVICE] = "device",
	[TYPEC_HOST] = "host",
};

static const char *typec_data_role_string(enum typec_data_role data_role)
{
	if (data_role < 0 || data_role >= ARRAY_SIZE(typec_data_roles))
		return "unknown";

	return typec_data_roles[data_role];
}

static int power_mode_get(struct wgw_ec_usbc_dev *usbc,
			  enum typec_role *pwr_role, bool *attached)
{
	u8 reg;
	struct wgw_ec_transport *transport = usbc->core->transport;

	int ret = transport->read_byte(transport, WGW_EC_REG_USB_MODE_POWER,
				       &reg);
	if (ret < 0) {
		dev_err(usbc->dev,
			"failed to read USB-C power mode register (%d)\n", ret);
		return ret;
	}

	switch (reg) {
	case WGW_EC_USB_POWER_MODE_DETACHED:
		*attached = false;
		*pwr_role = TYPEC_SINK;
		return 0;
	case WGW_EC_USB_POWER_MODE_SOURCE:
		*attached = true;
		*pwr_role = TYPEC_SOURCE;
		return 0;
	case WGW_EC_USB_POWER_MODE_SINK:
		*attached = true;
		*pwr_role = TYPEC_SINK;
		return 0;
	default:
		return -EIO;
	}
}

static int data_mode_get(struct wgw_ec_usbc_dev *usbc,
			 enum typec_data_role *data_role)
{
	u8 reg;
	struct wgw_ec_transport *transport = usbc->core->transport;

	int ret =
		transport->read_byte(transport, WGW_EC_REG_USB_MODE_DATA, &reg);
	if (ret < 0) {
		dev_err(usbc->dev,
			"failed to read USB-C data mode register (%d)\n", ret);
		return ret;
	}

	switch (reg) {
	case WGW_EC_USB_DATA_MODE_DEVICE:
		*data_role = TYPEC_DEVICE;
		return 0;
	case WGW_EC_USB_DATA_MODE_HOST:
		*data_role = TYPEC_HOST;
		return 0;
	default:
		return -EIO;
	}
}

static int data_mode_set(struct wgw_ec_usbc_dev *usbc,
			 enum typec_data_role data_role)
{
	u8 reg;
	struct wgw_ec_transport *transport = usbc->core->transport;

	switch (data_role) {
	case TYPEC_DEVICE:
		reg = WGW_EC_USB_DATA_MODE_DEVICE;
		break;
	case TYPEC_HOST:
		reg = WGW_EC_USB_DATA_MODE_HOST;
		break;
	default:
		return -EINVAL;
	}

	return transport->write_byte(transport, WGW_EC_REG_USB_MODE_DATA, reg);
}

static int wgw_ec_usbc_connect(struct wgw_ec_usbc_dev *usbc, bool attached,
			       enum typec_role pwr_role)
{
	struct typec_partner_desc desc;

	if (!attached)
		return 0;

	/* Update power role even if a partner is already registered. */
	usbc->pwr_role = pwr_role;
	typec_set_pwr_role(usbc->port, pwr_role);

	if (usbc->partner)
		return 0;

	desc.usb_pd = false;
	desc.accessory = TYPEC_ACCESSORY_NONE;
	desc.identity = NULL;

	usbc->partner = typec_register_partner(usbc->port, &desc);
	if (IS_ERR(usbc->partner))
		return PTR_ERR(usbc->partner);

	return 0;
}

static void wgw_ec_usbc_disconnect(struct wgw_ec_usbc_dev *usbc, bool attached)
{
	if (attached)
		return;

	if (!IS_ERR(usbc->partner) && usbc->partner)
		typec_unregister_partner(usbc->partner);
	usbc->partner = NULL;

	usbc->pwr_role = TYPEC_SINK;
	typec_set_pwr_role(usbc->port, TYPEC_SINK);
}

static int wgw_ec_usbc_dr_set(struct typec_port *port,
			      enum typec_data_role role)
{
	struct wgw_ec_usbc_dev *usbc = typec_get_drvdata(port);
	int status;

	dev_dbg(usbc->dev, "wgw_ec_usbc_dr_set: %d\n", role);
	status = data_mode_set(usbc, role);
	if (status < 0)
		return status;

	typec_set_data_role(usbc->port, role);
	return 0;
}

static int wgw_ec_usbc_trig_notify(struct notifier_block *nb, unsigned long evt,
				   void *dv)
{
	struct wgw_ec_usbc_dev *usbc =
		container_of(nb, struct wgw_ec_usbc_dev, notifier);
	struct wgw_ec_irq_event *irq_evt = dv;
	struct wgw_ec_transport *transport;
	u8 irq_status;
	struct wgw_ec_usbc_notification notif;
	enum typec_data_role new_data_mode = usbc->data_mode;
	enum typec_role new_pwr_role = usbc->pwr_role;
	bool new_attached = usbc->attached;
	int ret;
	bool data_role_changed = false;
	bool pwr_role_changed = false;
	bool attached_changed = false;

	dev_dbg(usbc->dev, "notified by wgw-ec-core\n");

	if (!irq_evt || !irq_evt->transport)
		return NOTIFY_DONE;

	transport = irq_evt->transport;
	irq_status = irq_evt->irq_status;

	if (!(irq_status & (REG_INTERRUPT_DATA_MODE_CHANGE |
			    REG_INTERRUPT_POWER_MODE_CHANGE))) {
		return NOTIFY_DONE;
	}

	if (irq_status & REG_INTERRUPT_DATA_MODE_CHANGE) {
		/* Clear first the ISR bit to ensure we get the latest value */
		ret = transport->write_byte(transport, WGW_EC_REG_INTERRUPT,
					    REG_INTERRUPT_DATA_MODE_CHANGE);
		if (ret < 0) {
			dev_err(usbc->dev,
				"failed to write register ISR register (data)\n");
			goto failure;
		}

		/* EC signals modification on USB-C data mode */
		ret = data_mode_get(usbc, &new_data_mode);
		if (ret < 0)
			goto failure;
	}

	if (irq_status & REG_INTERRUPT_POWER_MODE_CHANGE) {
		/* Clear first the ISR bit to ensure we get the latest value */
		ret = transport->write_byte(transport, WGW_EC_REG_INTERRUPT,
					    REG_INTERRUPT_POWER_MODE_CHANGE);
		if (ret < 0) {
			dev_err(usbc->dev,
				"failed to write register ISR register (power)\n");
			goto failure;
		}

		/* EC signals modification on USB-C power mode */
		ret = power_mode_get(usbc, &new_pwr_role, &new_attached);
		if (ret < 0)
			goto failure;
	}

	dev_dbg(usbc->dev,
		"USB-C regs: isr=0x%02x, attached=%s, pwr=%s, data=%s\n",
		irq_status, new_attached ? "yes" : "no",
		typec_role_string(new_pwr_role),
		typec_data_role_string(new_data_mode));

	/* init notification object */
	notif.dev = usbc->dev;
	notif.data_role = new_data_mode;
	notif.pwr_role = new_pwr_role;
	notif.attached = new_attached;

	spin_lock_bh(&usbc->lock);
	if (usbc->data_mode != new_data_mode) {
		usbc->data_mode = new_data_mode;
		data_role_changed = true;
	}
	if (usbc->pwr_role != new_pwr_role) {
		usbc->pwr_role = new_pwr_role;
		pwr_role_changed = true;
	}
	attached_changed = (usbc->attached != new_attached);
	if (attached_changed)
		usbc->attached = new_attached;
	spin_unlock_bh(&usbc->lock);

	if (!new_attached) {
		dev_dbg(usbc->dev, "USB-C detach event\n");
		wgw_ec_usbc_disconnect(usbc, new_attached);
	} else {
		/* We consider here that link is attached, we need to 
			* determine if link is new (attach) or already existing
			* but data role changed */

		if (attached_changed) {
			dev_dbg(usbc->dev, "USB-C attach event\n");
			ret = wgw_ec_usbc_connect(usbc, new_attached,
						  new_pwr_role);
			if (ret)
				dev_err(usbc->dev,
					"failed to register partner\n");
		} else {
			dev_dbg(usbc->dev, "USB-C modification event\n");
		}

		if (pwr_role_changed)
			typec_set_pwr_role(usbc->port, new_pwr_role);
		if (data_role_changed)
			typec_set_data_role(usbc->port, new_data_mode);
	}

	if (pwr_role_changed || attached_changed)
		blocking_notifier_call_chain(&usbc->notifier_list,
					     WGW_USBC_POWER_MODE_CHANGE,
					     &notif);
	if (data_role_changed)
		blocking_notifier_call_chain(&usbc->notifier_list,
					     WGW_USBC_DATA_MODE_CHANGE, &notif);

	return NOTIFY_OK;

failure:
	/* One last chance to clear the interrupt flags */
	dev_warn(
		usbc->dev,
		"clearing USB-C IRQ bits after error; latest event may be lost\n");
	ret = transport->write_byte(transport, WGW_EC_REG_INTERRUPT,
				    irq_status &
					    (REG_INTERRUPT_DATA_MODE_CHANGE |
					     REG_INTERRUPT_POWER_MODE_CHANGE));
	if (ret < 0)
		dev_err(usbc->dev,
			"failed to write register ISR register (failure)\n");
	return NOTIFY_OK;
}

static const struct typec_operations wgw_ec_usbc_ops = {
	.dr_set = wgw_ec_usbc_dr_set
};

static int wgw_ec_usbc_probe(struct platform_device *pdev)
{
	struct wgw_ec_core *core = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct wgw_ec_usbc_dev *usbc;
	struct wgw_ec_usbc_notification notif;
	struct device_node *np;
	int ret;

	dev_dbg(dev, "wgw-ec-usbc probe\n");

	if (!core)
		return dev_err_probe(dev, -ENODEV, "no parent EC device\n");

	usbc = devm_kzalloc(dev, sizeof(*usbc), GFP_KERNEL);
	if (!usbc)
		return -ENOMEM;

	usbc->core = core;
	usbc->dev = dev;

	/* OF node should already be bound by the MFD core via .of_compatible */
	np = dev_of_node(dev);
	if (!np)
		return dev_err_probe(dev, -ENOENT,
				     "no device tree node bound\n");

	/* retrieve data and power mode */
	ret = data_mode_get(usbc, &usbc->data_mode);
	if (ret < 0) {
		dev_err(dev, "failed to read USB-C data mode register\n");
		return ret;
	}
	ret = power_mode_get(usbc, &usbc->pwr_role, &usbc->attached);
	if (ret < 0) {
		dev_err(dev, "failed to read USB-C power mode register\n");
		return ret;
	}

	dev_info(dev,
		 "USB-C controller detected: power mode=%s, data mode=%s\n",
		 power_state_to_str(usbc->attached, usbc->pwr_role),
		 data_mode_to_str(usbc->data_mode));

	spin_lock_init(&usbc->lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&usbc->notifier_list);
	usbc->notifier.priority = 10;
	usbc->notifier.notifier_call = wgw_ec_usbc_trig_notify;
	ret = blocking_notifier_chain_register(&core->transport->notifier_list,
					       &usbc->notifier);
	if (ret < 0) {
		dev_err(dev, "failed to register to wgw-ec notifier list\n");
		return ret;
	}

	usbc->typec_cap.revision = USB_TYPEC_REV_1_1;
	usbc->typec_cap.prefer_role = TYPEC_SINK;
	usbc->typec_cap.type = TYPEC_PORT_DRP;
	usbc->typec_cap.data = TYPEC_PORT_DRD;
	usbc->typec_cap.ops = &wgw_ec_usbc_ops;
	usbc->typec_cap.driver_data = usbc;
	usbc->port = typec_register_port(dev, &usbc->typec_cap);
	if (IS_ERR(usbc->port)) {
		ret = PTR_ERR(usbc->port);
		usbc->port = NULL;
		goto unregister_ec_notifier;
	}

	/* Memorize data role */
	typec_set_data_role(usbc->port, usbc->data_mode);
	/* Memorize power role and apply it */
	ret = wgw_ec_usbc_connect(usbc, usbc->attached, usbc->pwr_role);
	if (ret < 0) {
		dev_err(dev, "failed to initialize Type-C partner: %d\n", ret);
		goto unregister_typec_port;
	}

	platform_set_drvdata(pdev, usbc);

	notif.dev = usbc->dev;
	notif.data_role = usbc->data_mode;
	notif.pwr_role = usbc->pwr_role;
	notif.attached = usbc->attached;
	blocking_notifier_call_chain(&usbc->notifier_list,
				     WGW_USBC_DEVICE_PROBE, &notif);

	dev_info(dev, "Wifx EC USB-C controller ready\n");
	dev_dbg(dev, "of_node=%pOF fwnode=%pfw\n", dev->of_node,
		dev_fwnode(dev));

	return 0;

unregister_typec_port:
	if (usbc->port) {
		typec_unregister_port(usbc->port);
		usbc->port = NULL;
	}

unregister_ec_notifier:
	blocking_notifier_chain_unregister(&core->transport->notifier_list,
					   &usbc->notifier);
	return ret;
}

static void wgw_ec_usbc_remove(struct platform_device *pdev)
{
	struct wgw_ec_usbc_dev *usbc = platform_get_drvdata(pdev);

	if (!usbc)
		return;

	blocking_notifier_chain_unregister(
		&usbc->core->transport->notifier_list, &usbc->notifier);

	/* Disconnect and free Type-C/role switch resources */
	wgw_ec_usbc_disconnect(usbc, false);

	if (usbc->port) {
		typec_unregister_port(usbc->port);
		usbc->port = NULL;
	}

	platform_set_drvdata(pdev, NULL);
}

#ifdef CONFIG_PM_SLEEP
static int wgw_ec_usbc_suspend(struct device *dev)
{
	return 0;
}

static int wgw_ec_usbc_resume(struct device *dev)
{
	struct wgw_ec_usbc_notification notif;
	struct wgw_ec_usbc_dev *usbc = dev_get_drvdata(dev);
	enum typec_data_role data_mode;
	enum typec_role pwr_role;
	bool attached;
	int ret;
	bool data_role_changed;
	bool pwr_role_changed;
	bool attached_changed;

	if (!usbc)
		return 0;

	ret = data_mode_get(usbc, &data_mode);
	if (ret < 0) {
		dev_warn(dev,
			 "failed to refresh USB-C data role on resume: %d\n",
			 ret);
		return 0;
	}

	ret = power_mode_get(usbc, &pwr_role, &attached);
	if (ret < 0) {
		dev_warn(dev,
			 "failed to refresh USB-C power role on resume: %d\n",
			 ret);
		return 0;
	}

	spin_lock_bh(&usbc->lock);
	data_role_changed = (usbc->data_mode != data_mode);
	pwr_role_changed = (usbc->pwr_role != pwr_role);
	attached_changed = (usbc->attached != attached);
	usbc->data_mode = data_mode;
	usbc->pwr_role = pwr_role;
	usbc->attached = attached;
	spin_unlock_bh(&usbc->lock);

	if (!attached)
		wgw_ec_usbc_disconnect(usbc, false);
	else {
		ret = wgw_ec_usbc_connect(usbc, true, pwr_role);
		if (ret < 0)
			dev_warn(
				dev,
				"failed to refresh Type-C partner on resume: %d\n",
				ret);
		typec_set_data_role(usbc->port, data_mode);
	}

	notif.dev = usbc->dev;
	notif.data_role = data_mode;
	notif.pwr_role = pwr_role;
	notif.attached = attached;

	if (data_role_changed || pwr_role_changed || attached_changed)
		blocking_notifier_call_chain(&usbc->notifier_list,
					     WGW_USBC_NOTIFIER_UPDATE, &notif);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(wgw_ec_usbc_pm_ops, wgw_ec_usbc_suspend,
			 wgw_ec_usbc_resume);

static struct platform_driver wgw_ec_usbc_driver = {
	.driver = {
		.name = "wgw-ec-usbc",
		.of_match_table = of_match_ptr(wgw_ec_usbc_of_match),
		.pm = &wgw_ec_usbc_pm_ops,
	},
	.probe = wgw_ec_usbc_probe,
	.remove = wgw_ec_usbc_remove,
};

int wgw_ec_usbc_register_notifier(struct device *dev, struct notifier_block *nb)
{
	struct wgw_ec_usbc_notification notif;
	struct wgw_ec_usbc_dev *usbc;
	int ret;

	if (!dev || !nb)
		return -ENODEV;

	usbc = dev_get_drvdata(dev);
	if (!usbc)
		return -EPROBE_DEFER;

	pr_debug("wgw_ec_usbc register a new notifier\n");

	ret = blocking_notifier_chain_register(&usbc->notifier_list, nb);
	if (ret < 0) {
		pr_err("wgw_ec_usbc: failed to register a new notifier: %d\n",
		       ret);
		return ret;
	}

	spin_lock_bh(&usbc->lock);
	notif.dev = usbc->dev;
	notif.data_role = usbc->data_mode;
	notif.pwr_role = usbc->pwr_role;
	notif.attached = usbc->attached;
	spin_unlock_bh(&usbc->lock);

	blocking_notifier_call_chain(&usbc->notifier_list,
				     WGW_USBC_NOTIFIER_UPDATE, &notif);
	return 0;
}
EXPORT_SYMBOL_GPL(wgw_ec_usbc_register_notifier);

int wgw_ec_usbc_unregister_notifier(struct device *dev,
				    struct notifier_block *nb)
{
	struct wgw_ec_usbc_dev *usbc;
	int ret;

	if (!dev || !nb)
		return -ENODEV;

	usbc = dev_get_drvdata(dev);
	if (!usbc)
		return -ENODEV;

	pr_debug("wgw_ec_usbc unregister a notifier\n");
	ret = blocking_notifier_chain_unregister(&usbc->notifier_list, nb);
	if (ret < 0)
		pr_err("wgw_ec_usbc: failed to unregister a notifier: %d\n",
		       ret);
	return ret;
}
EXPORT_SYMBOL_GPL(wgw_ec_usbc_unregister_notifier);

module_platform_driver(wgw_ec_usbc_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_DESCRIPTION("USB Type-C support for the Wifx board EC");
MODULE_LICENSE("GPL v2");
