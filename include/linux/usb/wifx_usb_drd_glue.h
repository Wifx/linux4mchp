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
#ifndef _LINUX_USB_WIFX_DRD_GLUE_H
#define _LINUX_USB_WIFX_DRD_GLUE_H

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>

#define WIFX_DRD_GLUE_DEVICE_PROBE 0x0001
#define WIFX_DRD_GLUE_NOTIFIER_UPDATE 0x0002
#define WIFX_DRD_GLUE_DATA_MODE_CHANGE 0x0003
#define WIFX_DRD_GLUE_POWER_MODE_CHANGE 0x0004

struct wifx_drd_glue_notification {
	struct device *dev;
	enum typec_data_role data_role;
	enum typec_role pwr_role;
	bool attached;
};

int wifx_usb_drd_glue_register_notifier(struct device *dev,
					struct notifier_block *nb);
int wifx_usb_drd_glue_unregister_notifier(struct device *dev,
					  struct notifier_block *nb);

#endif /* _LINUX_USB_WIFX_DRD_GLUE_H */
