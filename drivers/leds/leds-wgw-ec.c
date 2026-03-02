// SPDX-License-Identifier: GPL-2.0
/*
 * LED driver for the Wifx board EC MFD driver
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#include <linux/mfd/wgw-ec/core.h>
#include <linux/mfd/wgw-ec/reg.h>

#define DRV_NAME "wgw-ec-leds"

static const struct of_device_id led_wgw_ec_of_match[] = {
	{
		.compatible = "wifx,wgw-ec-leds",
	},
	{},
};
MODULE_DEVICE_TABLE(of, led_wgw_ec_of_match);

struct led_wgw_ec {
	const char *name;
	const char *default_trigger;
	s32 reg;
	u8 active_low;
	unsigned max_brightness;
	struct device_node *of_node;
};

struct led_wgw_ec_data {
	struct led_classdev cdev;
	struct wgw_ec_core *core;
	u8 id;
	unsigned int active_low;
	/* delayed work */
	struct work_struct work;
};

struct led_wgw_ec_priv {
	int num_leds;
	struct wgw_ec_core *core;
	struct led_wgw_ec_data leds[];
};

// private prototypes
static void led_wgw_ec_remove(struct platform_device *pdev);

static void led_wgw_ec_set_work(struct work_struct *work)
{
	struct led_wgw_ec_data *led_data =
		container_of(work, struct led_wgw_ec_data, work);
	struct wgw_ec_transport *transport = led_data->core->transport;

	u16 brightness = led_data->cdev.brightness;
	if (led_data->active_low) {
		brightness = led_data->cdev.max_brightness - brightness;
	}
	transport->write_word(transport, WGW_EC_REG_LED_START + led_data->id,
			      brightness);
}

static void led_wgw_ec_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct led_wgw_ec_data *led_data =
		container_of(led_cdev, struct led_wgw_ec_data, cdev);

	led_cdev->brightness = value;
	schedule_work(&led_data->work);
}

static int led_wgw_ec_add(struct device *dev, struct led_wgw_ec_priv *priv,
			  struct led_wgw_ec *led)
{
	struct led_wgw_ec_data *led_data = &priv->leds[priv->num_leds];
	struct led_init_data init_data = {};
	int ret;

	led_data->core = priv->core;
	if (led->reg < 0) {
		led_data->id = (u8)priv->num_leds;
	} else {
		led_data->id = (u8)led->reg;
	}
	led_data->active_low = led->active_low;
	led_data->cdev.name = led->name;
	led_data->cdev.default_trigger = led->default_trigger;
	led_data->cdev.max_brightness = led->max_brightness;
	led_data->cdev.brightness = LED_OFF;
	if (led->active_low) {
		led_data->cdev.brightness = led->max_brightness;
	}
	led_data->cdev.brightness_set = led_wgw_ec_set;

	INIT_WORK(&led_data->work, led_wgw_ec_set_work);

	init_data.fwnode = of_fwnode_handle(led->of_node);
	ret = led_classdev_register_ext(dev, &led_data->cdev, &init_data);
	if (!ret) {
		priv->num_leds++;
	} else {
		dev_err(dev, "failed to register wgw-ec led for %s: %d\n",
			led->name, ret);
	}
	dev_info(dev, "registered led (name=%s, trigger=%s)\n",
		 led_data->cdev.name, led_data->cdev.default_trigger);
	return ret;
}

static int led_wgw_ec_probe_dt(struct device *dev, struct led_wgw_ec_priv *priv)
{
	struct device_node *np = dev_of_node(dev);
	struct device_node *child;
	struct led_wgw_ec led;
	int ret = 0;

	if (!np)
		return dev_err_probe(dev, -EINVAL, "no DT node (of_node)\n");

	memset(&led, 0, sizeof(led));

	/* Iterate only over available children (status != "disabled") */
	for_each_available_child_of_node(np, child) {
		memset(&led, 0, sizeof(led));

		ret = of_property_read_string(child, "label", &led.name);
		if (ret) {
			dev_err(dev, "%pOF: missing 'label' property\n", child);
			goto err_child;
		}

		ret = of_property_read_u32(child, "reg", &led.reg);
		if (ret) {
			dev_err(dev, "%pOF: missing 'reg' property\n", child);
			goto err_child;
		}

		ret = of_property_read_string(child, "linux,default-trigger",
					      &led.default_trigger);
		if (ret) {
			dev_dbg(dev,
				"%pOF: no 'linux,default-trigger', using none\n",
				child);
			led.default_trigger = NULL;
		}

		ret = of_property_read_u32(child, "max-brightness",
					   &led.max_brightness);
		if (ret)
			led.max_brightness = LED_FULL;

		led.active_low = of_property_read_bool(child, "active-low");

		/* Pass the child LED node to the registration helper */
		led.of_node = child;

		ret = led_wgw_ec_add(dev, priv, &led);
		if (ret) {
			dev_err(dev, "%pOF: failed to add LED (reg=%u): %d\n",
				child, led.reg, ret);
			goto err_child;
		}
	}

	return 0;

err_child:
	/* for_each_available_child_of_node handles of_node_put(child) at loop end.
	 * We return directly here, so no manual of_node_put(child) is needed.
	 */
	return ret;
}

static int led_wgw_ec_probe(struct platform_device *pdev)
{
	struct wgw_ec_core *core = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct device_node *np = dev_of_node(dev);
	struct led_wgw_ec_priv *priv;
	int count, ret;

	if (!core)
		return dev_err_probe(dev, -ENODEV, "no parent EC device\n");

	if (!np)
		return dev_err_probe(dev, -EINVAL, "no DT node (of_node)\n");

	if (!of_device_is_available(np))
		return dev_err_probe(dev, -ENODEV, "%pOF is disabled\n", np);

	/* Count available children under the 'leds' node */
	count = of_get_available_child_count(np);
	dev_dbg(dev, "detected %d LED node(s) under %pOF\n", count, np);

	if (count <= 0)
		return dev_err_probe(dev, -ENODEV, "no LED subnodes found\n");

	priv = devm_kzalloc(dev, struct_size(priv, leds, count), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->core = core;
	priv->num_leds = 0; /* Incremented in led_wgw_ec_add */
	platform_set_drvdata(pdev, priv);

	ret = led_wgw_ec_probe_dt(dev, priv);
	if (ret) {
		dev_dbg(dev, "create from DT failed, cleanup\n");
		led_wgw_ec_remove(pdev);
		return ret;
	}

	return 0;
}

static void led_wgw_ec_remove(struct platform_device *pdev)
{
	struct led_wgw_ec_priv *priv = platform_get_drvdata(pdev);
	struct wgw_ec_transport *transport;
	struct led_wgw_ec_data *led_data;
	int i;
	u16 brightness;

	if (!priv)
		return;

	/* Always stop pending LED work before unregister/teardown */
	for (i = 0; i < priv->num_leds; i++) {
		led_data = &priv->leds[i];
		cancel_work_sync(&led_data->work);
	}

	if (!priv->core || !priv->core->transport)
		goto unregister_only;

	transport = priv->core->transport;

	/* Turn LEDs off (optional safety state) */
	for (i = 0; i < priv->num_leds; i++) {
		led_data = &priv->leds[i];

		/* Set LED to 0 (or any other preferred safe state) */
		brightness = 0;
		if (led_data->active_low)
			brightness = led_data->cdev.max_brightness - brightness;

		/* Ignore errors here: driver is being removed */
		transport->write_word(transport,
				      WGW_EC_REG_LED_START + led_data->id,
				      brightness);
	}

unregister_only:
	/* Unregister class devices last */
	for (i = 0; i < priv->num_leds; i++) {
		led_data = &priv->leds[i];
		led_classdev_unregister(&led_data->cdev);
	}
}

static struct platform_driver led_wgw_ec_driver = {
	.driver = {
		.name	= "wgw-ec-leds",
		.of_match_table = of_match_ptr(led_wgw_ec_of_match),
	},
	.probe = led_wgw_ec_probe,
	.remove   = led_wgw_ec_remove,
};
module_platform_driver(led_wgw_ec_driver);

MODULE_ALIAS("platform:" DRV_NAME);
MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_DESCRIPTION("LED support for the Wifx board EC");
MODULE_LICENSE("GPL v2");
