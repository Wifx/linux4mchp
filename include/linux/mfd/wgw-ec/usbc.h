// SPDX-License-Identifier: GPL-2.0
/*
 * USB Type-C support driver for the Wifx board EC
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#ifndef __LINUX_MFD_WGW_EC_USBC_H
#define __LINUX_MFD_WGW_EC_USBC_H

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>

/* Events and notification from the usb core */
#define WGW_USBC_DEVICE_PROBE 0x0001
#define WGW_USBC_NOTIFIER_UPDATE 0x0002
#define WGW_USBC_DATA_MODE_CHANGE 0x0003
#define WGW_USBC_POWER_MODE_CHANGE 0x0004

struct wgw_ec_usbc_notification {
	struct device *dev;
	enum typec_data_role data_role;
	enum typec_role pwr_role;
	bool attached;
};

#if IS_REACHABLE(CONFIG_MFD_WGW_EC_USBC)
int wgw_ec_usbc_register_notifier(struct device *dev,
				  struct notifier_block *nb);
int wgw_ec_usbc_unregister_notifier(struct device *dev,
				    struct notifier_block *nb);
#else
static inline int wgw_ec_usbc_register_notifier(struct device *dev,
						struct notifier_block *nb)
{
	return -ENODEV;
}

static inline int wgw_ec_usbc_unregister_notifier(struct device *dev,
						  struct notifier_block *nb)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_MFD_WGW_EC_USBC_H */
