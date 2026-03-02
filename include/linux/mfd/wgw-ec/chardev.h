// SPDX-License-Identifier: GPL-2.0
/*
 * Miscellaneous character driver for Wifx board EC
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 */
#ifndef __LINUX_MFD_WGW_EC_CHARDEV_H
#define __LINUX_MFD_WGW_EC_CHARDEV_H

#include <linux/i2c-dev.h>

#define WGW_EC_DEV_IOC 0xEC
#define WGW_EC_DEV_IOC_PKT_CMD \
	_IOW(WGW_EC_DEV_IOC, 1, struct i2c_smbus_ioctl_data)
#define WGW_EC_DEV_IOC_PKT_CMD_LTR \
	_IOW(WGW_EC_DEV_IOC, 2, struct i2c_smbus_ioctl_data)

#endif /* __LINUX_MFD_WGW_EC_CHARDEV_H */
