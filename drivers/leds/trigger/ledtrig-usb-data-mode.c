// SPDX-License-Identifier: GPL-2.0
/*
 * Wifx USB-C LED trigger (data role)
 *
 * Drives an LED based on the Type-C data role reported by the Wifx DRD glue,
 * using a notifier to track role changes and reflect them on the LED.
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/leds.h>
#include <linux/device.h>
#include <linux/device/bus.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/usb/wifx_usb_drd_glue.h>

struct ledtrig_usb_data_mode_data {
	/* Protect fast fields (enabled/led_on). */
	spinlock_t lock;

	struct led_classdev *led_cdev;

	/* Provider node and device (wifx,drd-glue). */
	struct device_node
		*provider_np; /* ref held between activate and deactivate */
	struct platform_device *provider_pdev; /* device ref held when non-NULL */

	/* Async binding and updates. */
	struct delayed_work work; /* LED updates + bind attempts */
	struct notifier_block notifier;
	struct notifier_block bus_nb;

	bool led_on;
	bool enabled;
	bool bound;
	bool removing;
	bool bus_notifier_registered;
};

static void
ledtrig_usb_data_mode_apply_led(struct ledtrig_usb_data_mode_data *td);

static void
ledtrig_usb_data_mode_release_provider(struct ledtrig_usb_data_mode_data *td)
{
	if (!td->provider_pdev)
		return;

	put_device(&td->provider_pdev->dev);
	td->provider_pdev = NULL;
}

static int
ledtrig_usb_data_mode_bind_provider(struct ledtrig_usb_data_mode_data *td)
{
	struct device *led_dev = td->led_cdev ? td->led_cdev->dev : NULL;
	int ret;

	if (!led_dev || !td->provider_np)
		return -ENODEV;

	if (READ_ONCE(td->bound))
		return 0;

	if (!td->provider_pdev) {
		struct platform_device *pdev;

		pdev = of_find_device_by_node(td->provider_np);
		if (!pdev)
			return -EAGAIN;

		get_device(&pdev->dev);

		td->provider_pdev = pdev;
	}

	ret = wifx_usb_drd_glue_register_notifier(&td->provider_pdev->dev,
						  &td->notifier);
	if (!ret) {
		unsigned long flags;

		spin_lock_irqsave(&td->lock, flags);
		td->bound = true;
		spin_unlock_irqrestore(&td->lock, flags);

		dev_info(led_dev, "ledtrig-usb-data-mode: bound to %s\n",
			dev_name(&td->provider_pdev->dev));
		return 0;
	}

	if (ret == -ENODEV || ret == -EPROBE_DEFER)
		return -EAGAIN;

	dev_dbg(led_dev, "ledtrig-usb-data-mode: notifier reg failed: %d\n",
		ret);

	ledtrig_usb_data_mode_release_provider(td);

	return ret;
}

/* sysfs "enable" */
static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ledtrig_usb_data_mode_data *td = led_trigger_get_drvdata(dev);
	unsigned long flags;
	bool enabled;

	if (!td)
		return -ENODEV;

	spin_lock_irqsave(&td->lock, flags);
	enabled = td->enabled;
	spin_unlock_irqrestore(&td->lock, flags);

	return scnprintf(buf, PAGE_SIZE, "%u\n", enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	struct ledtrig_usb_data_mode_data *td = led_trigger_get_drvdata(dev);
	unsigned long flags;
	unsigned int val;
	int ret;

	if (!td)
		return -ENODEV;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 1)
		return -EINVAL;

	/* Avoid waiting on a pending work item in sysfs context. */
	cancel_delayed_work(&td->work);

	spin_lock_irqsave(&td->lock, flags);
	td->enabled = val != 0;
	spin_unlock_irqrestore(&td->lock, flags);

	/* Work is only used when registered to trigger source. */
	if (!READ_ONCE(td->removing) && READ_ONCE(td->bound))
		schedule_delayed_work(&td->work, 0);
	else
		ledtrig_usb_data_mode_apply_led(td);

	return size;
}

static DEVICE_ATTR_RW(enable);
static struct attribute *ledtrig_usb_data_mode_attrs[] = {
	&dev_attr_enable.attr, NULL
};
ATTRIBUTE_GROUPS(ledtrig_usb_data_mode);

/* Update LED based on current state. */
static void
ledtrig_usb_data_mode_apply_led(struct ledtrig_usb_data_mode_data *td)
{
	struct led_classdev *cdev = td->led_cdev;
	unsigned long flags;
	bool enabled;
	bool bound;
	bool led_on;

	if (!cdev)
		return;

	spin_lock_irqsave(&td->lock, flags);
	enabled = td->enabled;
	bound = td->bound;
	led_on = td->led_on;
	spin_unlock_irqrestore(&td->lock, flags);

	if (!enabled)
		led_set_brightness(cdev, LED_OFF);
	else
		led_set_brightness(cdev,
				   (bound && led_on) ? LED_FULL : LED_OFF);
}

/* Notifier callback from provider. */
static int ledtrig_usb_data_mode_notify(struct notifier_block *notifier,
					unsigned long evt, void *dv)
{
	struct wifx_drd_glue_notification *notif = dv;
	struct ledtrig_usb_data_mode_data *td = container_of(
		notifier, struct ledtrig_usb_data_mode_data, notifier);
	struct device *dev = td->led_cdev ? td->led_cdev->dev : NULL;
	unsigned long flags;

	if (!td || !td->led_cdev || !notif || READ_ONCE(td->removing))
		return NOTIFY_DONE;

	dev_dbg(dev, "ledtrig-usb-data-mode: notifier event %lu\n", evt);

	/* If bound to a specific provider, ignore others. */
	if (td->provider_pdev && notif->dev != &td->provider_pdev->dev) {
		dev_dbg(dev,
			"ledtrig-usb-data-mode: event from unrelated provider, ignored\n");
		return NOTIFY_DONE;
	}

	if (evt == WIFX_DRD_GLUE_DATA_MODE_CHANGE ||
	    evt == WIFX_DRD_GLUE_DEVICE_PROBE ||
	    evt == WIFX_DRD_GLUE_NOTIFIER_UPDATE) {
		spin_lock_irqsave(&td->lock, flags);
		td->led_on = (notif->data_role == TYPEC_DEVICE);
		spin_unlock_irqrestore(&td->lock, flags);

		if (!READ_ONCE(td->removing) && READ_ONCE(td->bound))
			schedule_delayed_work(&td->work, 0);

		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

/* Try to bind to provider and/or update LED state. */
static void ledtrig_usb_data_mode_work(struct work_struct *work)
{
	struct ledtrig_usb_data_mode_data *td = container_of(
		work, struct ledtrig_usb_data_mode_data, work.work);

	if (READ_ONCE(td->removing) || !READ_ONCE(td->bound))
		return;

	/*
	 * Do not unregister bus notifier from bus callback context; doing it here
	 * avoids deadlocks on notifier chain locking.
	 */
	if (td->bus_notifier_registered) {
		bus_unregister_notifier(&platform_bus_type, &td->bus_nb);
		td->bus_notifier_registered = false;
	}

	/* Apply current LED state. */
	ledtrig_usb_data_mode_apply_led(td);
}

static int ledtrig_usb_data_mode_bus_notify(struct notifier_block *nb,
					    unsigned long action, void *data)
{
	struct ledtrig_usb_data_mode_data *td =
		container_of(nb, struct ledtrig_usb_data_mode_data, bus_nb);
	struct device *dev = data;

	if (READ_ONCE(td->removing))
		return NOTIFY_DONE;

	if (action != BUS_NOTIFY_ADD_DEVICE &&
	    action != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;

	if (!dev || dev->of_node != td->provider_np || READ_ONCE(td->bound))
		return NOTIFY_DONE;

	if (ledtrig_usb_data_mode_bind_provider(td) == 0)
		schedule_delayed_work(&td->work, 0);

	return NOTIFY_DONE;
}

static int ledtrig_usb_data_mode_activate(struct led_classdev *led_cdev)
{
	struct device *dev = led_cdev->dev;
	struct device_node *led_np;
	struct of_phandle_args of_trigger_handle;
	struct ledtrig_usb_data_mode_data *td;
	int ret, count;

	dev_dbg(dev, "ledtrig-usb-data-mode activate for %s\n", led_cdev->name);

	if (!dev || !dev->fwnode) {
		dev_err(dev, "ledtrig-usb-data-mode: no LED device/fwnode\n");
		return -ENODEV;
	}

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td)
		return -ENOMEM;

	td->led_cdev = led_cdev;
	td->enabled = false;
	td->led_on = false;
	td->bound = false;
	td->removing = false;
	td->bus_notifier_registered = false;
	spin_lock_init(&td->lock);
	INIT_DELAYED_WORK(&td->work, ledtrig_usb_data_mode_work);
	td->notifier.notifier_call = ledtrig_usb_data_mode_notify;
	td->notifier.priority = 10;
	td->bus_nb.notifier_call = ledtrig_usb_data_mode_bus_notify;

	led_np = to_of_node(dev->fwnode);
	if (!led_np) {
		dev_err(dev,
			"ledtrig-usb-data-mode: LED fwnode is not a DT node\n");
		ret = -ENODEV;
		goto err_free;
	}

	/* Find trigger source (provider) from LED node. */
	count = of_count_phandle_with_args(led_np, "trigger-sources",
					   "#trigger-source-cells");
	if (count == -ENOENT) {
		dev_err(dev,
			"ledtrig-usb-data-mode: no trigger phandle found\n");
		ret = -ENODEV;
		goto err_free;
	} else if (count < 0) {
		dev_err(dev,
			"ledtrig-usb-data-mode: failed to get trigger sources for %pOF\n",
			led_np);
		ret = count;
		goto err_free;
	} else if (count != 1) {
		dev_err(dev,
			"ledtrig-usb-data-mode: too many trigger sources (%d), max is 1\n",
			count);
		ret = -EINVAL;
		goto err_free;
	}

	ret = of_parse_phandle_with_args(led_np, "trigger-sources",
					 "#trigger-source-cells", 0,
					 &of_trigger_handle);
	if (ret) {
		dev_err(dev,
			"ledtrig-usb-data-mode: failed to parse trigger-source: %d\n",
			ret);
		goto err_free;
	}

	/* Only accept wifx drd-glue as provider. */
	if (!of_device_is_compatible(of_trigger_handle.np, "wifx,drd-glue")) {
		dev_err(dev,
			"ledtrig-usb-data-mode: %pOF is not a compatible provider\n",
			of_trigger_handle.np);
		ret = -ENODEV;
		goto err_put_np;
	}

	/* Store provider node for later retries; keep ref from parse. */
	td->provider_np = of_trigger_handle.np;

	/* 1) Try to register directly with trigger source. */
	ret = ledtrig_usb_data_mode_bind_provider(td);
	if (ret == 0)
		goto ready;
	if (ret != -EAGAIN) {
		dev_err(dev,
			"ledtrig-usb-data-mode: failed to register to trigger source: %d\n",
			ret);
		goto err_put_np;
	}

	/* 2) Fallback: wait for source instantiation/binding via bus events. */
	ret = bus_register_notifier(&platform_bus_type, &td->bus_nb);
	if (ret) {
		dev_err(dev,
			"ledtrig-usb-data-mode: failed to register bus notifier: %d\n",
			ret);
		goto err_put_np;
	}
	td->bus_notifier_registered = true;

ready:
	/* Expose drvdata to sysfs before scheduling work. */
	led_set_trigger_data(led_cdev, td);

	if (READ_ONCE(td->bound))
		schedule_delayed_work(&td->work, 0);

	pr_info("ledtrig-usb-data-mode: registered to indicate USB-C data role\n");

	return 0;

err_put_np:
	ledtrig_usb_data_mode_release_provider(td);
	of_node_put(of_trigger_handle.np);
err_free:
	kfree(td);
	return ret;
}

static void ledtrig_usb_data_mode_deactivate(struct led_classdev *led_cdev)
{
	struct ledtrig_usb_data_mode_data *td = led_get_trigger_data(led_cdev);

	pr_debug("ledtrig-usb-data-mode deactivate\n");

	if (!td)
		return;

	/* Drop drvdata early to avoid sysfs races. */
	led_set_trigger_data(led_cdev, NULL);

	/* Block all new work scheduling paths. */
	WRITE_ONCE(td->removing, true);

	/* Unregister notifier first to stop callbacks. */
	if (READ_ONCE(td->bound) && td->provider_pdev) {
		unsigned long flags;

		wifx_usb_drd_glue_unregister_notifier(&td->provider_pdev->dev,
						      &td->notifier);
		spin_lock_irqsave(&td->lock, flags);
		td->bound = false;
		spin_unlock_irqrestore(&td->lock, flags);
	}

	/* Stop work after callback source is detached. */
	cancel_delayed_work_sync(&td->work);

	if (td->bus_notifier_registered) {
		bus_unregister_notifier(&platform_bus_type, &td->bus_nb);
		td->bus_notifier_registered = false;
	}

	/* Release provider device ref if held. */
	ledtrig_usb_data_mode_release_provider(td);

	/* Release provider OF ref. */
	if (td->provider_np) {
		of_node_put(td->provider_np);
		td->provider_np = NULL;
	}

	kfree(td);
}

static struct led_trigger ledtrig_usb_data_mode = {
	.name = "usb-data-mode",
	.activate = ledtrig_usb_data_mode_activate,
	.deactivate = ledtrig_usb_data_mode_deactivate,
	.groups = ledtrig_usb_data_mode_groups,
};

static int __init ledtrig_usb_data_mode_init(void)
{
	pr_debug("ledtrig-usb-data-mode init\n");
	return led_trigger_register(&ledtrig_usb_data_mode);
}

static void __exit ledtrig_usb_data_mode_exit(void)
{
	pr_debug("ledtrig-usb-data-mode exit\n");
	led_trigger_unregister(&ledtrig_usb_data_mode);
}

module_init(ledtrig_usb_data_mode_init);
module_exit(ledtrig_usb_data_mode_exit);

MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_DESCRIPTION("USB DRD data mode trigger for the Wifx board EC LED");
MODULE_LICENSE("GPL v2");
