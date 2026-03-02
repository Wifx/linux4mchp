// SPDX-License-Identifier: GPL-2.0
/*
 * Core header for the Wifx Embedded Controller (EC)
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#ifndef __LINUX_MFD_WGW_EC_CORE_H
#define __LINUX_MFD_WGW_EC_CORE_H

#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/mfd/wgw-ec/reg.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define WGW_EC_APP_COMMIT_HASH_SIZE 16
#define WGW_EC_APP_COMMIT_DATE_SIZE 32
#define WGW_EC_HW_VERSION_SIZE 32
#define WGW_EC_FW_VERSION_SIZE 32
#define WGW_EC_HW_SN_SIZE 16

#define WGW_EC_MEM_SLOT_COUNT 4
#define WGW_EC_MEM_SLOT_DATA_SIZE 32
#define WGW_EC_MEM_SLOT_STR_SIZE 31

#define WGW_EC_MEM_SLOT_STATE_ERROR (-1)
#define WGW_EC_MEM_SLOT_STATE_SET 0x01
#define WGW_EC_MEM_SLOT_STATE_OTP 0x02
#define WGW_EC_MEM_SLOT_STATE_SET_OTP \
	(WGW_EC_MEM_SLOT_STATE_SET | WGW_EC_MEM_SLOT_STATE_OTP)

struct wgw_ec_core;
struct wgw_ec_transport;

struct wgw_ec_slot_str {
	char data[WGW_EC_MEM_SLOT_STR_SIZE + 1]; /* include '\0' */
	u8 state;
};

struct wgw_ec_slot_u8 {
	u8 value;
	u8 state;
};

enum wgw_ec_mainboard_ref {
	// LORIX One's PCB
	WGW_EC_MB_WGW_L01_BASE = 0,
	// Wifx L1's PCB (or w/o model in slot[1])
	WGW_EC_MB_WGW_L02_BASE_L1 = 1,
	// Wifx Y1 (Y-Linx)'s PCB
	WGW_EC_MB_WGW_L02_BASE_Y1 = 2,
	// Wifx L1's PCB overriden with Wifx L1 4G product model
	WGW_EC_MB_WGW_L02_BASE_L1_4G = 3,
};

enum wgw_ec_mainboard_variant {
	WGW_EC_MB_VAR_8XX = 0,
	WGW_EC_MB_VAR_9XX = 1,
};

enum wgw_ec_model {
	WGW_EC_M_LORIX_ONE = 0,
	WGW_EC_M_WIFX_L1 = 1,
	WGW_EC_M_WIFX_Y1 = 2,
	WGW_EC_M_WIFX_L1_4G = 3,
};

/* Product model variants */
#define WGW_EC_MODEL_VAR_UNDEFINED (-1)
#define WGW_EC_MODEL_VAR_UNKNOWN (-2)
#define WGW_EC_MODEL_VAR_ERROR (-3)

enum wgw_ec_wifx_l1_variant {
	WGW_EC_WIFX_L1_VAR_8XX = 0,
	WGW_EC_WIFX_L1_VAR_9XX = 1,
	WGW_EC_WIFX_L1_VAR_MAX = 2,
};

enum wgw_ec_wifx_l1_4g_variant {
	WGW_EC_WIFX_L1_4G_VAR_8XX_EU = 0,
	WGW_EC_WIFX_L1_4G_VAR_9XX_AU = 1,
	WGW_EC_WIFX_L1_4G_VAR_9XX_US = 2,
	WGW_EC_WIFX_L1_4G_VAR_MAX = 3,
};

struct wgw_ec_version {
	u16 major;
	u16 minor;
	u16 revision;
};

struct wgw_ec_hw_tuple_info {
	s8 id;
	const char *str;
};

struct wgw_ec_fw_info {
	struct wgw_ec_version version;
	char version_str[WGW_EC_FW_VERSION_SIZE];
	char commit_hash[WGW_EC_APP_COMMIT_HASH_SIZE];
	char commit_date[WGW_EC_APP_COMMIT_DATE_SIZE];
};

struct wgw_ec_mainboard_info {
	char version_str[WGW_EC_HW_VERSION_SIZE];
	struct wgw_ec_hw_tuple_info model;
	struct wgw_ec_hw_tuple_info base_model;
	struct wgw_ec_hw_tuple_info variant;
	struct wgw_ec_fw_info fw;
};

struct wgw_ec_product_info {
	struct wgw_ec_slot_str serial;
	struct wgw_ec_hw_tuple_info model;
	struct wgw_ec_hw_tuple_info variant;
	char version_str[WGW_EC_HW_VERSION_SIZE];
};

struct wgw_ec_info {
	u8 protoc;
	u8 boot_state;
	struct wgw_ec_mainboard_info mainboard;
	struct wgw_ec_product_info product;
};

#define WGW_EC_MEM_SLOT_SIZE 32
struct wgw_ec_memory_slot {
	u8 data[WGW_EC_MEM_SLOT_SIZE];
	u8 length;
	u8 flags;
};

#define CMD_LTR_STATUS_SUCCESS 0
#define CMD_LTR_STATUS_BUSY 1
#define CMD_LTR_STATUS_INVALID_ARG 2
#define CMD_LTR_STATUS_FAILURE 3
#define CMD_LTR_STATUS_BAD_CRC 4
#define CMD_LTR_STATUS_NOT_WRITABLE 5
#define CMD_LTR_STATUS_PAGE_NOT_ALIGNED 6

/*
 * Transport-facing handle (allocated by transport driver, e.g. I2C).
 * Provides bus access ops and the transport device.
 */
struct wgw_ec_transport {
	const char *phys_name;
	struct device *dev; /* transport parent device (I2C/SPI client) */
	struct wgw_ec_core *core; /* core device (ec-core platform) */
	void *priv;

	/* GPIO/IRQ managed by core (set at runtime by core, or left 0/NULL) */
	int irq;
	struct gpio_desc *cpu_state_pin;

	struct mutex lock_ltr;
	struct blocking_notifier_head notifier_list;
	struct delayed_work irq_work;

	/* Bus ops: return length (>=0) or error (<0) */
	int (*read_byte)(struct wgw_ec_transport *mcu, char command, u8 *data);
	int (*read_word)(struct wgw_ec_transport *mcu, char command, u16 *data);
	int (*read_block)(struct wgw_ec_transport *mcu, char command, u8 *data);

	int (*write_byte)(struct wgw_ec_transport *mcu, char command, u8 data);
	int (*write_word)(struct wgw_ec_transport *mcu, char command, u16 data);
	int (*write_block)(struct wgw_ec_transport *mcu, char command,
			   const u8 *data, u8 len);
};

/*
 * IRQ contract: each consumer clears the IRQ bits it handles. If multiple
 * consumers ever share a bit, use a dedicated aggregator to coordinate
 * clearing for that bit.
 */
struct wgw_ec_irq_event {
	struct wgw_ec_transport *transport;
	u8 irq_status;
};

/*
 * Core EC instance (platform child bound to ec-core DT node).
 */
struct wgw_ec_core {
	struct device class_dev; /* optional class device */
	struct wgw_ec_transport *transport; /* link to transport instance */
	struct device *dev; /* core device (platform device) */

	struct wgw_ec_info cache_info;
	struct mutex cache_lock;
};

/* Helpers exported for subdrivers */
int wgw_ec_get_ltr_status(struct wgw_ec_core *core);
int wgw_ec_wait_ready(struct wgw_ec_core *core);

int wgw_ec_boot_state_get(struct wgw_ec_core *core, u8 *boot_state);
int wgw_ec_boot_state_clr_update(struct wgw_ec_core *core, u8 *boot_state);

#define to_wgw_ec_core(dev) container_of(dev, struct wgw_ec_core, class_dev)

#endif /* __LINUX_MFD_WGW_EC_CORE_H */