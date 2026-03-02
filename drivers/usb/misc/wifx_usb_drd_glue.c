// SPDX-License-Identifier: GPL-2.0
/*
 * Wifx DRD glue for USB role switching
 *
 * Bridges Type-C role notifications from the EC to a per-device notifier chain,
 * exposes a usb_role_switch, and optionally uses ID/VBUS GPIOs for role sensing.
 *
 * Copyright (C) 2026 Wifx SA,
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>
#include <linux/notifier.h>
#include <linux/usb/role.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/device/bus.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/wgw-ec/usbc.h>
#include <linux/usb/wifx_usb_drd_glue.h>

struct wifx_drd_glue {
	struct device *dev;
	struct device_node *udc_np;
	struct device_node *usbc_np;

	/* Output switch consumed by the UDC side */
	struct usb_role_switch *role_sw_out;

	/* EC USB-C device */
	struct device *usbc_dev;

	enum usb_role current_role;
	struct mutex role_lock; /* protects current_role and transitions */
	enum typec_data_role data_role;
	bool attached;

	/* Optional GPIOs + IRQs */
	struct gpio_desc *id_gpiod; /* active-low: 0 => A-cable/host */
	int id_irq;
	struct gpio_desc *vbus_valid_gpiod; /* 1 => valid VBUS present */
	int vbus_irq;

	struct notifier_block usbc_nb;
	struct blocking_notifier_head notifier_list;
	struct wifx_drd_glue_notification last_notif;
	struct regulator *vbus_supply;
	bool have_last_notif;
	bool usbc_registered;
	bool removing;
	bool role_sw_synced;
	bool use_typec;
	bool vbus_on;
	bool suspended;
	struct delayed_work gpio_debounce_work;
	struct notifier_block bus_nb;
};

static int wifx_drd_apply_role_locked(struct wifx_drd_glue *glue,
				      enum typec_data_role new_data_role,
				      bool attached);

static struct usb_role_switch *
wifx_drd_get_output_switch(struct wifx_drd_glue *glue);

static int wifx_drd_init_gpio_inputs(struct wifx_drd_glue *glue);

static enum typec_data_role
wifx_drd_detect_role_gpio(struct wifx_drd_glue *glue, bool *attached);

static enum usb_role wifx_drd_map_role_to_udc(enum typec_data_role data_role,
					      bool attached)
{
	switch (data_role) {
	case TYPEC_DEVICE:
		return attached ? USB_ROLE_DEVICE : USB_ROLE_NONE;
	case TYPEC_HOST:
		return USB_ROLE_HOST;
	default:
		return USB_ROLE_NONE;
	}
}

static void wifx_drd_set_vbus_supply_locked(struct wifx_drd_glue *glue,
					    enum usb_role udc_role)
{
	int ret;
	bool want_on;

	if (!glue->vbus_supply) {
		dev_dbg(glue->dev,
			"vbus-supply: no regulator bound, skip (role=%s)\n",
			usb_role_string(udc_role));
		return;
	}

	want_on = (udc_role == USB_ROLE_HOST);
	if (glue->vbus_on == want_on) {
		dev_dbg(glue->dev,
			"vbus-supply: unchanged (role=%s, state=%s)\n",
			usb_role_string(udc_role),
			glue->vbus_on ? "on" : "off");
		return;
	}

	dev_dbg(glue->dev, "vbus-supply: request %s (role=%s, prev=%s)\n",
		want_on ? "enable" : "disable", usb_role_string(udc_role),
		glue->vbus_on ? "on" : "off");

	if (want_on)
		ret = regulator_enable(glue->vbus_supply);
	else
		ret = regulator_disable(glue->vbus_supply);

	if (ret) {
		dev_warn(glue->dev,
			 "failed to %s vbus-supply for role %s: %d\n",
			 want_on ? "enable" : "disable",
			 usb_role_string(udc_role), ret);
		return;
	}

	glue->vbus_on = want_on;
	dev_dbg(glue->dev, "vbus-supply: now %s\n",
		glue->vbus_on ? "on" : "off");
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

/* Debounce policy for GPIO threaded handlers */
#define DRD_GPIO_ATTACH_DEBOUNCE_MS 50
#define DRD_GPIO_DETACH_DEBOUNCE_MS 0
#define DRD_GPIO_HW_DEBOUNCE_US 500

static void wifx_drd_gpio_debounce_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct wifx_drd_glue *glue =
		container_of(dwork, struct wifx_drd_glue, gpio_debounce_work);
	enum typec_data_role sensed;
	bool attached = false;

	mutex_lock(&glue->role_lock);
	if (glue->removing || glue->suspended) {
		mutex_unlock(&glue->role_lock);
		return;
	}

	sensed = wifx_drd_detect_role_gpio(glue, &attached);
	wifx_drd_apply_role_locked(glue, sensed, attached);
	mutex_unlock(&glue->role_lock);

	dev_dbg(glue->dev, "GPIO debounce apply: role=%s attached=%s\n",
		typec_data_role_string(sensed), attached ? "yes" : "no");
}

static const char *wifx_drd_gpio_name(struct wifx_drd_glue *glue,
				      struct gpio_desc *gpiod)
{
	if (gpiod == glue->id_gpiod)
		return "id";

	if (gpiod == glue->vbus_valid_gpiod)
		return "vbus";

	return "unknown";
}

static void wifx_drd_schedule_gpio_debounce(struct wifx_drd_glue *glue,
					    struct gpio_desc *gpiod)
{
	enum typec_data_role sensed_role;
	enum typec_data_role current_role;
	bool sensed_attached = false;
	bool current_attached;
	unsigned long delay;

	mutex_lock(&glue->role_lock);
	if (glue->removing) {
		mutex_unlock(&glue->role_lock);
		return;
	}

	current_attached = glue->attached;
	current_role = glue->data_role;
	sensed_role = wifx_drd_detect_role_gpio(glue, &sensed_attached);

	/*
	 * When leaving host mode on ID IRQ, VBUS can still read high briefly
	 * because it is locally sourced. Treat this as detach immediately to
	 * avoid emitting a transient host->device state.
	 */
	if (current_attached && current_role == TYPEC_HOST &&
	    gpiod == glue->id_gpiod && sensed_attached &&
	    sensed_role == TYPEC_DEVICE && glue->vbus_on) {
		sensed_attached = false;
		dev_dbg(glue->dev,
			"GPIO IRQ (%s): ignore self-powered VBUS during host exit\n",
			wifx_drd_gpio_name(glue, gpiod));
	}

	if (current_attached && !sensed_attached)
		delay = msecs_to_jiffies(DRD_GPIO_DETACH_DEBOUNCE_MS);
	else
		delay = msecs_to_jiffies(DRD_GPIO_ATTACH_DEBOUNCE_MS);

	mutex_unlock(&glue->role_lock);

	mod_delayed_work(system_wq, &glue->gpio_debounce_work, delay);
	dev_dbg(glue->dev,
		"GPIO IRQ (%s): debounce scheduled (%u ms), state %s/%s -> %s/%s (sensed role=%s)\n",
		wifx_drd_gpio_name(glue, gpiod), jiffies_to_msecs(delay),
		typec_data_role_string(current_role),
		current_attached ? "attached" : "detached",
		typec_data_role_string(sensed_role),
		sensed_attached ? "attached" : "detached",
		typec_data_role_string(sensed_role));
}

static int wifx_drd_glue_usbc_notify(struct notifier_block *nb,
				     unsigned long evt, void *dv)
{
	struct wifx_drd_glue *glue =
		container_of(nb, struct wifx_drd_glue, usbc_nb);
	struct wgw_ec_usbc_notification *notif = dv;
	struct wifx_drd_glue_notification gnotif;

	if (!notif || READ_ONCE(glue->removing) || READ_ONCE(glue->suspended))
		return NOTIFY_DONE;

	dev_dbg(glue->dev,
		"notif from wgw-ec-usbc: evt=%lu, attached=%s, pwr role=%s, data role=%s\n",
		evt, notif->attached ? "yes" : "no",
		typec_role_string(notif->pwr_role),
		typec_data_role_string(notif->data_role));

	gnotif.dev = glue->dev;
	gnotif.data_role = notif->data_role;
	gnotif.pwr_role = notif->pwr_role;
	gnotif.attached = notif->attached;

	mutex_lock(&glue->role_lock);
	glue->last_notif = gnotif;
	glue->have_last_notif = true;
	/* Apply EC-reported role immediately to the UDC switch. */
	wifx_drd_apply_role_locked(glue, gnotif.data_role, gnotif.attached);
	mutex_unlock(&glue->role_lock);

	blocking_notifier_call_chain(&glue->notifier_list, evt, &gnotif);
	return NOTIFY_OK;
}

static int wifx_drd_glue_usbc_register(struct wifx_drd_glue *glue)
{
	int ret;

	if (glue->usbc_registered)
		return 0;

	glue->usbc_nb.notifier_call = wifx_drd_glue_usbc_notify;
	glue->usbc_nb.priority = 10;

	if (!glue->usbc_dev)
		return -ENODEV;

	ret = wgw_ec_usbc_register_notifier(glue->usbc_dev, &glue->usbc_nb);
	if (!ret)
		glue->usbc_registered = true;

	return ret;
}

static struct device *wifx_drd_get_usbc_dev(struct device *dev)
{
	struct device_node *np;
	struct platform_device *pdev;

	if (!dev || !dev->of_node)
		return NULL;

	np = of_parse_phandle(dev->of_node, "wifx,wgw-ec-usbc", 0);
	if (!np)
		return NULL;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return NULL;

	return &pdev->dev;
}

int wifx_usb_drd_glue_register_notifier(struct device *dev,
					struct notifier_block *nb)
{
	struct wifx_drd_glue *glue;
	struct wifx_drd_glue_notification snap;
	bool have_snap = false;
	int ret;

	if (!dev)
		return -ENODEV;

	glue = dev_get_drvdata(dev);
	if (!glue)
		return -ENODEV;

	ret = wifx_drd_glue_usbc_register(glue);
	if (ret && ret != -ENODEV)
		return ret;

	ret = blocking_notifier_chain_register(&glue->notifier_list, nb);
	if (ret)
		return ret;

	mutex_lock(&glue->role_lock);
	if (glue->have_last_notif) {
		snap = glue->last_notif;
		have_snap = true;
	}
	mutex_unlock(&glue->role_lock);

	if (have_snap)
		blocking_notifier_call_chain(&glue->notifier_list,
					     WIFX_DRD_GLUE_NOTIFIER_UPDATE,
					     &snap);

	return 0;
}
EXPORT_SYMBOL_GPL(wifx_usb_drd_glue_register_notifier);

int wifx_usb_drd_glue_unregister_notifier(struct device *dev,
					  struct notifier_block *nb)
{
	struct wifx_drd_glue *glue;

	if (!dev)
		return -ENODEV;

	glue = dev_get_drvdata(dev);
	if (!glue)
		return -ENODEV;

	return blocking_notifier_chain_unregister(&glue->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(wifx_usb_drd_glue_unregister_notifier);

static enum typec_data_role
wifx_drd_detect_role_gpio(struct wifx_drd_glue *glue, bool *attached)
{
	int id = -1, vbus = -1;

	if (glue->id_gpiod)
		id = gpiod_get_value_cansleep(glue->id_gpiod);
	if (glue->vbus_valid_gpiod)
		vbus = gpiod_get_value_cansleep(glue->vbus_valid_gpiod);

	*attached = true;
	/* Simple policy:
	 * - if id == 0 (active low, A-cable), then HOST
	 * - else if vbus == 1, then DEVICE
	 * - else NONE
	 */
	if (glue->id_gpiod && id == 0)
		return TYPEC_HOST;

	if (glue->vbus_valid_gpiod && vbus > 0)
		return TYPEC_DEVICE;

	*attached = false;
	return TYPEC_DEVICE;
}

static int wifx_drd_init_gpio_inputs(struct wifx_drd_glue *glue)
{
	struct device_node *connector_np = NULL;
	struct fwnode_handle *connector_fwnode = NULL;
	int ret;

	if (!glue->dev->of_node)
		return 0;

	connector_np = of_get_child_by_name(glue->dev->of_node, "connector");
	if (connector_np)
		connector_fwnode = of_fwnode_handle(connector_np);

	if (connector_fwnode) {
		glue->id_gpiod = devm_fwnode_gpiod_get_index(glue->dev,
							     connector_fwnode,
							     "id", 0, GPIOD_IN,
							     "wifx_drd_id");
		if (IS_ERR(glue->id_gpiod)) {
			if (PTR_ERR(glue->id_gpiod) == -ENOENT)
				glue->id_gpiod = NULL;
			else {
				of_node_put(connector_np);
				return PTR_ERR(glue->id_gpiod);
			}
		}

		glue->vbus_valid_gpiod = devm_fwnode_gpiod_get_index(
			glue->dev, connector_fwnode, "vbus", 0, GPIOD_IN,
			"wifx_drd_vbus");
		if (IS_ERR(glue->vbus_valid_gpiod)) {
			if (PTR_ERR(glue->vbus_valid_gpiod) == -ENOENT)
				glue->vbus_valid_gpiod = NULL;
			else {
				of_node_put(connector_np);
				return PTR_ERR(glue->vbus_valid_gpiod);
			}
		}
	}

	of_node_put(connector_np);

	if (!glue->id_gpiod)
		glue->id_gpiod =
			devm_gpiod_get_optional(glue->dev, "id", GPIOD_IN);
	if (IS_ERR(glue->id_gpiod))
		return PTR_ERR(glue->id_gpiod);

	if (!glue->vbus_valid_gpiod)
		glue->vbus_valid_gpiod =
			devm_gpiod_get_optional(glue->dev, "vbus", GPIOD_IN);
	if (IS_ERR(glue->vbus_valid_gpiod))
		return PTR_ERR(glue->vbus_valid_gpiod);

	if (glue->id_gpiod) {
		ret = gpiod_set_debounce(glue->id_gpiod,
					 DRD_GPIO_HW_DEBOUNCE_US);
		if (ret && ret != -ENOTSUPP)
			dev_dbg(glue->dev,
				"failed to set ID GPIO debounce: %d\n", ret);
	}

	if (glue->vbus_valid_gpiod) {
		ret = gpiod_set_debounce(glue->vbus_valid_gpiod,
					 DRD_GPIO_HW_DEBOUNCE_US);
		if (ret && ret != -ENOTSUPP)
			dev_dbg(glue->dev,
				"failed to set VBUS GPIO debounce: %d\n", ret);
	}

	return 0;
}

/**
 * wifx_drd_get_output_switch - Find the remote UDC role switch
 * @glue: pointer to the glue instance
 *
 * Prefer the UDC phandle when available, then fall back to the legacy
 * endpoint-based lookup.
 *
 * Return: a pointer to the struct usb_role_switch if found, NULL otherwise.
 */
static struct usb_role_switch *
wifx_drd_get_output_switch(struct wifx_drd_glue *glue)
{
	struct fwnode_handle *endpoint = NULL;
	struct fwnode_handle *remote_node = NULL;
	struct usb_role_switch *sw = NULL;

	if (glue->udc_np) {
		sw = usb_role_switch_find_by_fwnode(
			of_fwnode_handle(glue->udc_np));
		if (sw)
			return sw;
	}

	/* Legacy path: endpoints with role-switch-out tag. */
	while ((endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(glue->dev),
							  endpoint))) {
		bool keep_endpoint = false;

		/* Check if this endpoint points to the output (UDC) */
		if (!fwnode_property_present(endpoint, "role-switch-out"))
			goto next_endpoint;

		remote_node = fwnode_graph_get_remote_port_parent(endpoint);
		if (!remote_node)
			goto next_endpoint;

		/* Attempt to resolve the switch from the remote fwnode */
		sw = usb_role_switch_find_by_fwnode(remote_node);

		/* Release the remote node reference immediately after lookup */
		fwnode_handle_put(remote_node);

		if (sw) {
			/* Found the switch, cleanup and return */
			keep_endpoint = true;
			return sw;
		}

next_endpoint:
		if (!keep_endpoint)
			fwnode_handle_put(endpoint);
	}

	return NULL;
}

static int wifx_drd_apply_role_locked(struct wifx_drd_glue *glue,
				      enum typec_data_role new_data_role,
				      bool attached)
{
	enum usb_role udc_role;
	bool role_changed = (glue->data_role != new_data_role) ||
			    (glue->attached != attached);
	bool switch_available_before = !!glue->role_sw_out;
	bool switch_available_after;
	int ret = 0;

	udc_role = wifx_drd_map_role_to_udc(new_data_role, attached);
	wifx_drd_set_vbus_supply_locked(glue, udc_role);

	/* Lazy discovery: try to bind to the UDC if not already done */
	if (!switch_available_before)
		glue->role_sw_out = wifx_drd_get_output_switch(glue);

	switch_available_after = !!glue->role_sw_out;

	if (role_changed) {
		dev_dbg(glue->dev,
			"state changed, role: %s -> %s, attached: %s -> %s\n",
			typec_data_role_string(glue->data_role),
			typec_data_role_string(new_data_role),
			glue->attached ? "yes" : "no", attached ? "yes" : "no");
		glue->data_role = new_data_role;
		glue->attached = attached;
	}

	if (!switch_available_after) {
		glue->role_sw_synced = false;
		dev_dbg(glue->dev,
			"UDC role switch not yet available (waiting for bus notification)\n");
	} else if (role_changed || !glue->role_sw_synced ||
		   (!switch_available_before && switch_available_after)) {
		ret = usb_role_switch_set_role(glue->role_sw_out, udc_role);
		if (ret) {
			dev_warn(
				glue->dev,
				"failed to set UDC role %s from data=%d attached=%d: %d\n",
				usb_role_string(udc_role), new_data_role,
				attached, ret);
			return ret;
		}

		glue->current_role = udc_role;
		glue->role_sw_synced = true;
	} else {
		dev_dbg(glue->dev,
			"UDC role switch already bound, role unchanged (data=%d attached=%d)\n",
			new_data_role, attached);
	}
	return 0;
}

static int wifx_drd_glue_bus_notify(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	struct wifx_drd_glue *glue =
		container_of(nb, struct wifx_drd_glue, bus_nb);
	struct device *dev = data;
	int ret;

	if (READ_ONCE(glue->removing) || READ_ONCE(glue->suspended))
		return NOTIFY_DONE;

	if (action != BUS_NOTIFY_ADD_DEVICE &&
	    action != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;

	if (!dev || !glue->dev)
		return NOTIFY_DONE;

	if (glue->usbc_np && dev->of_node == glue->usbc_np &&
	    !glue->usbc_registered && !glue->usbc_dev) {
		get_device(dev);
		glue->usbc_dev = dev;

		ret = wifx_drd_glue_usbc_register(glue);
		if (ret) {
			if (ret != -ENODEV && ret != -EPROBE_DEFER)
				dev_warn(
					glue->dev,
					"failed to register usbc notifier on bus action=%lu: %d\n",
					action, ret);
			put_device(glue->usbc_dev);
			glue->usbc_dev = NULL;
		} else {
			dev_dbg(
				glue->dev,
				"USBC device is available, notifier registered\n");
		}
	}

	if (!glue->udc_np)
		return NOTIFY_DONE;

	if (glue->role_sw_out)
		return NOTIFY_DONE;

	if (dev->of_node != glue->udc_np)
		return NOTIFY_DONE;

	mutex_lock(&glue->role_lock);
	if (!glue->role_sw_out) {
		glue->role_sw_out = wifx_drd_get_output_switch(glue);
		if (glue->role_sw_out) {
			dev_dbg(glue->dev,
				"UDC device is available, binding role switch now\n");
			wifx_drd_apply_role_locked(glue, glue->data_role,
						   glue->attached);
		}
	}
	mutex_unlock(&glue->role_lock);

	return NOTIFY_DONE;
}

/* Threaded IRQ handlers */
static irqreturn_t wifx_drd_id_irq_thread(int irq, void *data)
{
	struct wifx_drd_glue *glue = data;

	wifx_drd_schedule_gpio_debounce(glue, glue->id_gpiod);

	return IRQ_HANDLED;
}

static irqreturn_t wifx_drd_vbus_irq_thread(int irq, void *data)
{
	struct wifx_drd_glue *glue = data;

	wifx_drd_schedule_gpio_debounce(glue, glue->vbus_valid_gpiod);

	return IRQ_HANDLED;
}

static int wifx_drd_request_gpio_irq(struct wifx_drd_glue *glue,
				     struct gpio_desc *gpiod,
				     irqreturn_t (*threadfn)(int, void *),
				     const char *name)
{
	int irq, ret, flags;

	irq = gpiod_to_irq(gpiod);
	if (irq < 0)
		return irq;

	/* Trigger both edges to track state flips; threaded handler, no hard-IRQ fn */
	flags = IRQF_ONESHOT | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	/* Start disabled; enabled after successful request */
	irq_set_status_flags(irq, IRQ_NOAUTOEN);

	ret = devm_request_threaded_irq(glue->dev, irq, NULL, threadfn, flags,
					name, glue);
	if (ret)
		return ret;

	enable_irq(irq);
	return irq;
}

/**
 * wifx_drd_glue_probe - Platform driver probe function
 * @pdev: platform device pointer
 *
 * Return: 0 on success, negative error code on failure.
 */
static int wifx_drd_glue_probe(struct platform_device *pdev)
{
	struct wifx_drd_glue *glue;
	int ret = 0;
	enum typec_data_role data_role = TYPEC_DEVICE;
	bool attached = false;
	bool has_vbus_supply_prop = false;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	glue->dev = &pdev->dev;
	mutex_init(&glue->role_lock);
	BLOCKING_INIT_NOTIFIER_HEAD(&glue->notifier_list);
	glue->have_last_notif = false;
	glue->usbc_registered = false;
	glue->removing = false;
	glue->role_sw_synced = false;
	glue->use_typec = false;
	glue->vbus_on = false;
	glue->udc_np = NULL;
	glue->usbc_np = NULL;
	glue->vbus_supply = NULL;
	INIT_DELAYED_WORK(&glue->gpio_debounce_work,
			  wifx_drd_gpio_debounce_work);
	glue->bus_nb.notifier_call = wifx_drd_glue_bus_notify;

	/* Store driver data for remove() and exported API lookups */
	platform_set_drvdata(pdev, glue);

	if (glue->dev->of_node)
		glue->usbc_np = of_parse_phandle(glue->dev->of_node,
						 "wifx,wgw-ec-usbc", 0);

	if (glue->dev->of_node)
		has_vbus_supply_prop = of_property_read_bool(glue->dev->of_node,
							     "vbus-supply");

	dev_dbg(&pdev->dev, "vbus-supply DT property: %s\n",
		has_vbus_supply_prop ? "present" : "absent");

	glue->use_typec = !!glue->usbc_np;

	if (glue->use_typec)
		glue->usbc_dev = wifx_drd_get_usbc_dev(&pdev->dev);

	glue->vbus_supply = devm_regulator_get_optional(&pdev->dev, "vbus");
	if (IS_ERR(glue->vbus_supply)) {
		if (PTR_ERR(glue->vbus_supply) == -ENODEV) {
			dev_dbg(&pdev->dev,
				"vbus-supply: regulator not available in current config/topology\n");
			glue->vbus_supply = NULL;
		} else {
			ret = PTR_ERR(glue->vbus_supply);
			dev_err(&pdev->dev, "failed to get vbus-supply: %d\n",
				ret);
			return ret;
		}
	} else {
		dev_dbg(&pdev->dev, "vbus-supply: regulator acquired\n");
	}

	if (glue->usbc_dev) {
		ret = wifx_drd_glue_usbc_register(glue);
		if (ret == -EPROBE_DEFER) {
			put_device(glue->usbc_dev);
			glue->usbc_dev = NULL;
			return ret;
		}
		if (ret) {
			dev_dbg(&pdev->dev,
				"failed to register usbc notifier: %d\n", ret);
			return ret;
		}

		/* Registration above may immediately trigger a state notification. */
		data_role = glue->last_notif.data_role;
		attached = glue->last_notif.attached;

		dev_dbg(&pdev->dev, "wgw-ec-usbc notifier registered\n");
	} else if (!glue->use_typec) {
		ret = wifx_drd_init_gpio_inputs(glue);
		if (ret)
			return ret;

		data_role = wifx_drd_detect_role_gpio(glue, &attached);
	}
	mutex_lock(&glue->role_lock);
	glue->data_role = data_role;
	glue->attached = attached;
	mutex_unlock(&glue->role_lock);

	if (glue->dev->of_node)
		glue->udc_np = of_parse_phandle(glue->dev->of_node,
						"wifx,udc-controller", 0);

	/*
	 * Try to bind immediately if the UDC role switch is already available.
	 * If not, rely on bus notifications to bind later without probe deferral.
	 */
	glue->role_sw_out = wifx_drd_get_output_switch(glue);
	if (glue->role_sw_out) {
		dev_dbg(&pdev->dev, "bound to UDC role switch\n");
		mutex_lock(&glue->role_lock);
		wifx_drd_apply_role_locked(glue, glue->data_role,
					   glue->attached);
		mutex_unlock(&glue->role_lock);
	} else {
		dev_dbg(&pdev->dev,
			"UDC role switch not found yet, waiting for bus event\n");
	}

	ret = bus_register_notifier(glue->dev->bus, &glue->bus_nb);
	if (ret)
		dev_warn(&pdev->dev, "failed to register bus notifier: %d\n",
			 ret);

	/* Request GPIO IRQs if present */
	glue->id_irq = 0;
	glue->vbus_irq = 0;

	if (!glue->use_typec) {
		if (glue->id_gpiod) {
			ret = wifx_drd_request_gpio_irq(glue, glue->id_gpiod,
							wifx_drd_id_irq_thread,
							"wifx_drd_id");
			if (ret < 0) {
				dev_warn(&pdev->dev,
					 "failed to request ID IRQ: %d\n", ret);
			} else {
				glue->id_irq = ret;
			}
		}

		if (glue->vbus_valid_gpiod) {
			ret = wifx_drd_request_gpio_irq(
				glue, glue->vbus_valid_gpiod,
				wifx_drd_vbus_irq_thread, "wifx_drd_vbus");
			if (ret < 0) {
				dev_warn(&pdev->dev,
					 "failed to request VBUS IRQ: %d\n",
					 ret);
			} else {
				glue->vbus_irq = ret;
			}
		}
	}

	dev_info(&pdev->dev,
		 "USB DRD glue ready, initial role: %s, attached: %s\n",
		 typec_data_role_string(data_role), attached ? "yes" : "no");
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wifx_drd_glue_suspend(struct device *dev)
{
	struct wifx_drd_glue *glue = dev_get_drvdata(dev);

	if (!glue)
		return 0;

	mutex_lock(&glue->role_lock);
	glue->suspended = true;
	mutex_unlock(&glue->role_lock);

	cancel_delayed_work_sync(&glue->gpio_debounce_work);

	return 0;
}

static int wifx_drd_glue_resume(struct device *dev)
{
	struct wifx_drd_glue *glue = dev_get_drvdata(dev);
	enum typec_data_role data_role;
	bool attached;

	if (!glue)
		return 0;

	mutex_lock(&glue->role_lock);
	glue->suspended = false;

	if (glue->use_typec && glue->have_last_notif) {
		data_role = glue->last_notif.data_role;
		attached = glue->last_notif.attached;
	} else if (!glue->use_typec) {
		data_role = wifx_drd_detect_role_gpio(glue, &attached);
	} else {
		data_role = glue->data_role;
		attached = glue->attached;
	}

	wifx_drd_apply_role_locked(glue, data_role, attached);
	mutex_unlock(&glue->role_lock);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(wifx_drd_glue_pm_ops, wifx_drd_glue_suspend,
			 wifx_drd_glue_resume);

static void wifx_drd_glue_remove(struct platform_device *pdev)
{
	struct wifx_drd_glue *glue = platform_get_drvdata(pdev);

	if (!glue)
		return;

	mutex_lock(&glue->role_lock);
	glue->removing = true;
	mutex_unlock(&glue->role_lock);

	if (glue->usbc_registered)
		wgw_ec_usbc_unregister_notifier(glue->usbc_dev, &glue->usbc_nb);

	bus_unregister_notifier(glue->dev->bus, &glue->bus_nb);

	cancel_delayed_work_sync(&glue->gpio_debounce_work);

	/* devm_* automatically releases IRQ and GPIO resources. */

	if (glue->role_sw_out && !IS_ERR(glue->role_sw_out)) {
		usb_role_switch_put(glue->role_sw_out);
		glue->role_sw_out = NULL;
	}
	glue->role_sw_synced = false;

	if (glue->udc_np) {
		of_node_put(glue->udc_np);
		glue->udc_np = NULL;
	}

	if (glue->usbc_np) {
		of_node_put(glue->usbc_np);
		glue->usbc_np = NULL;
	}

	if (glue->usbc_dev)
		put_device(glue->usbc_dev);

	/* Optional: clear drvdata */
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id wifx_drd_glue_of_match[] = {
	{ .compatible = "wifx,drd-glue" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wifx_drd_glue_of_match);

static struct platform_driver wifx_drd_glue_driver = {
	.probe = wifx_drd_glue_probe,
	.remove_new = wifx_drd_glue_remove,
	.driver = {
		.name = "wifx_drd_glue",
		.of_match_table = wifx_drd_glue_of_match,
		.pm = &wifx_drd_glue_pm_ops,
	},
};
module_platform_driver(wifx_drd_glue_driver);

MODULE_DESCRIPTION(
	"Wifx DRD glue with usb_role_switch and per-instance notifier chain (GPIO IRQ-based)");
MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_LICENSE("GPL v2");
