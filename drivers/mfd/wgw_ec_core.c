// SPDX-License-Identifier: GPL-2.0
/*
 * Core platform driver for the Wifx Embedded Controller (EC)
 *
 * This “core” driver binds to the DT child node (ec-core), owns the IRQ and
 * cpu-state GPIO, performs EC detection and caching, and registers MFD children
 * (leds, usbc, etc.). Bus access is provided by the transport driver (I2C/SPI)
 * via struct wgw_ec_transport attached to the parent device.
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#include <linux/mfd/wgw-ec/core.h>
#include <linux/mfd/wgw-ec/reg.h>

#define DRV_NAME "wgw-ec-core"

/*
 * Architecture summary (transport <-> core <-> children):
 *
 *   board-ec@2a (I2C/SPI transport)
 *     -> wgw-ec-i2c: allocates struct wgw_ec_transport
 *        | dev_set_drvdata(parent, transport)
 *        v
 *   ec-core (platform child)
 *     -> wgw-ec-core: allocates struct wgw_ec_core
 *        | core->transport = transport
 *        | transport->core = core
 *        v
 *   MFD children (usbc, leds, sysfs, chardev)
 *     -> use core->transport for bus ops
 */
#ifndef to_wgw_ec_core
static inline struct wgw_ec_core *to_wgw_ec_core(const struct device *d)
{
	return container_of(d, struct wgw_ec_core, class_dev);
}
#endif

/* ------------------- IRQ handling (notifier fanout) ------------------- */

static void wgw_ec_irq_work(struct work_struct *work)
{
	struct wgw_ec_transport *transport =
		container_of(work, struct wgw_ec_transport, irq_work.work);
	struct wgw_ec_core *core = READ_ONCE(transport->core);
	struct device *dev = core ? core->dev : transport->dev;
	struct wgw_ec_irq_event event;
	int ret;
	u8 status = 0;

	if (!core)
		return;

	/* Read the interrupt status from the EC and pass it to the notifier chain */
	ret = transport->read_byte(transport, WGW_EC_REG_INTERRUPT, &status);
	if (ret < 0) {
		dev_warn(dev, "failed to read EC interrupt status: %d\n", ret);
		return;
	}
	dev_dbg(dev, "EC interrupt status: 0x%02x\n", status);

	event.transport = transport;
	event.irq_status = status;
	blocking_notifier_call_chain(&transport->notifier_list, 0, &event);
}

static irqreturn_t wgw_ec_irq_thread(int irq, void *data)
{
	struct wgw_ec_transport *transport = data;
	struct wgw_ec_core *core = READ_ONCE(transport->core);

	if (!core)
		return IRQ_HANDLED;

	dev_dbg(core->dev, "wgw-ec-core: irq thread run\n");
	mod_delayed_work(system_wq, &transport->irq_work, msecs_to_jiffies(10));
	return IRQ_HANDLED;
}

/* ------------------- Strings / helpers copied from previous code ------------------- */

#define WGW_EC_MEM_SLOT_EMPTY 0x00
#define WGW_EC_MEM_SLOT_SET 0x01
#define WGW_EC_MEM_SLOT_OTP 0x02
#define WGW_EC_MEM_SLOT_SET_OTP (WGW_EC_MEM_SLOT_SET | WGW_EC_MEM_SLOT_OTP)
#define WGW_EC_MEM_SLOT_STATE_Msk 0x03

static const char *unknown_str = "unknown";
static const char *undefined_str = "undefined";
static const char *error_str = "error";

static const char *mainboard_model_strs[] = { "wgw-l01-base", "wgw-l02-base",
					      "wgw-l02-base-y1",
					      "wgw-l02-base-4g" };

static int mainboard_ref_index(enum wgw_ec_mainboard_ref mb_ref)
{
	switch (mb_ref) {
	case WGW_EC_MB_WGW_L01_BASE:
	case WGW_EC_MB_WGW_L02_BASE_L1:
	case WGW_EC_MB_WGW_L02_BASE_Y1:
	case WGW_EC_MB_WGW_L02_BASE_L1_4G:
		return (int)mb_ref;
	default:
		return WGW_EC_MODEL_VAR_UNKNOWN;
	}
}

static const char *mainboard_ref_str(enum wgw_ec_mainboard_ref mb_ref)
{
	int index = mainboard_ref_index(mb_ref);
	if (index < 0)
		return unknown_str;
	return mainboard_model_strs[index];
}

static const char *mainboard_variant_strs[] = { "8XX", "9XX" };

static int mainboard_variant_index(enum wgw_ec_mainboard_variant mb_variant)
{
	switch (mb_variant) {
	case WGW_EC_MB_VAR_8XX:
	case WGW_EC_MB_VAR_9XX:
		return (int)mb_variant;
	default:
		return WGW_EC_MODEL_VAR_UNKNOWN;
	}
}

static const char *
mainboard_variant_str(enum wgw_ec_mainboard_variant mb_variant)
{
	int index = mainboard_variant_index(mb_variant);
	if (index < 0)
		return unknown_str;
	return mainboard_variant_strs[index];
}

static const char *model_strs[] = { "lorix-one", "wifx-l1", "wifx-y1",
				    "wifx-l1-4g" };
static const char *model_pretty_strs[] = { "LORIX One", "Wifx L1", "Wifx Y1",
					   "Wifx L1 4G" };

static int model_index(enum wgw_ec_model model)
{
	switch (model) {
	case WGW_EC_M_LORIX_ONE:
	case WGW_EC_M_WIFX_L1:
	case WGW_EC_M_WIFX_Y1:
	case WGW_EC_M_WIFX_L1_4G:
		return (int)model;
	default:
		return -1;
	}
}

static const char *model_str(enum wgw_ec_model model)
{
	int index = model_index(model);
	return (index < 0) ? unknown_str : model_strs[index];
}

static const char *model_pretty_str(enum wgw_ec_model model)
{
	int index = model_index(model);
	return (index < 0) ? unknown_str : model_pretty_strs[index];
}

static int model_variant_index(enum wgw_ec_model product_model,
			       s8 product_variant)
{
	int min_level = min((int)product_model, (int)product_variant);

	if (min_level < 0)
		return min_level;

	switch (product_model) {
	case WGW_EC_M_WIFX_L1:
		if (product_variant < WGW_EC_WIFX_L1_VAR_MAX)
			return product_variant;
		break;
	case WGW_EC_M_WIFX_L1_4G:
		if (product_variant < WGW_EC_WIFX_L1_4G_VAR_MAX)
			return product_variant;
		break;
	case WGW_EC_M_LORIX_ONE:
	case WGW_EC_M_WIFX_Y1:
	default:
		break;
	}
	return WGW_EC_MODEL_VAR_UNKNOWN;
}

static const char *model_variant_str(enum wgw_ec_model product_model,
				     s8 product_variant)
{
	static const char *model_l1_variant_strs[] = { "8XX", "9XX" };
	static const char *model_l1_4g_variant_strs[] = { "8XX-EU", "9XX-AU",
							  "9XX-US" };

	int index = model_variant_index(product_model, product_variant);
	if (index < 0) {
		switch (index) {
		case WGW_EC_MODEL_VAR_UNDEFINED:
			return undefined_str;
		case WGW_EC_MODEL_VAR_UNKNOWN:
			return unknown_str;
		case WGW_EC_MODEL_VAR_ERROR:
		default:
			return error_str;
		}
	}
	switch (product_model) {
	case WGW_EC_M_WIFX_L1:
		return model_l1_variant_strs[product_variant];
	case WGW_EC_M_WIFX_L1_4G:
		return model_l1_4g_variant_strs[product_variant];
	default:
		return unknown_str;
	}
}

static int hw_version_str(char *version_str,
			  const struct wgw_ec_version *version, int max_len)
{
	if (version->revision == 0)
		return snprintf(version_str, max_len, "%d.%d", version->major,
				version->minor);
	return snprintf(version_str, max_len, "%d.%d%c", version->major,
			version->minor, (char)(version->revision + 'A'));
}

static int fw_version_str(char *version_str,
			  const struct wgw_ec_version *version, int max_len)
{
	return snprintf(version_str, max_len, "%d.%d.%d", version->major,
			version->minor, version->revision);
}

/* ------------------- EC register helpers (public for children) ------------------- */

int wgw_ec_get_ltr_status(struct wgw_ec_core *core)
{
	struct wgw_ec_transport *transport = core->transport;
	u8 status;
	int ret = transport->read_byte(transport, WGW_EC_REG_CMD_LTR_STATUS,
				       &status);
	if (ret < 0) {
		dev_err(core->dev,
			"failed to read LTR command status from device\n");
		return ret;
	}
	return (int)status;
}
EXPORT_SYMBOL_GPL(wgw_ec_get_ltr_status);

int wgw_ec_wait_ready(struct wgw_ec_core *core)
{
	int i, status;
	for (i = 0; i < 10; i++) {
		status = wgw_ec_get_ltr_status(core);
		if (status < 0)
			return status;
		if (status == CMD_LTR_STATUS_SUCCESS)
			return 0;

		if (status == CMD_LTR_STATUS_BUSY) {
			msleep(5);
			continue;
		}
		dev_err(core->dev, "LTR command error status: %d\n", status);
		return -EIO;
	}
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(wgw_ec_wait_ready);

/* ------------------- EC info/cache (copied from previous wgw_ec_core.c) ------------------- */

struct hw_info {
	struct wgw_ec_version version;
	u8 model;
	u8 variant;
	u8 frequency;
};

struct wgw_ec_reg {
	union {
		u8 data[32];
		struct wgw_ec_version fw_info_version;
		const char *fw_info_commit_hash;
		const char *fw_info_commit_date;
		struct hw_info hw_info;
	};
};

static int mem_slot_get(struct wgw_ec_core *core, u8 slot_index,
			struct wgw_ec_memory_slot *slot)
{
	struct wgw_ec_transport *transport = core->transport;
	char buffer[32];
	int ret;

	if (slot_index > 3) {
		dev_err(core->dev, "slot[%d] doesn't exist\n", slot_index);
		return -EINVAL;
	}

	ret = transport->read_block(
		transport, WGW_EC_REG_MEM_SLOT0_CTRL + slot_index, buffer);
	if (ret < 0) {
		dev_err(core->dev,
			"failed to read memory slot[%d] ctrl register\n",
			slot_index);
		return ret;
	}
	slot->flags = buffer[0] & WGW_EC_MEM_SLOT_STATE_Msk;
	slot->length = buffer[1];

	if (!(slot->flags & WGW_EC_MEM_SLOT_SET)) {
		slot->length = 0;
	} else {
		ret = transport->read_block(
			transport, WGW_EC_REG_MEM_SLOT0 + slot_index, buffer);
		if (ret < 0) {
			dev_err(core->dev,
				"failed to read memory slot[%d] data register\n",
				slot_index);
			return ret;
		}
		if (ret != slot->length) {
			dev_err(core->dev,
				"failed to read memory slot[%d] data register, data length error\n",
				slot_index);
			return ret;
		}
		memcpy(slot->data, buffer, slot->length);
	}
	return slot->length;
}

static int mem_slot_get_str(struct wgw_ec_core *core, u8 slot_index,
			    struct wgw_ec_slot_str *slot_str)
{
	int32_t ret;
	struct wgw_ec_memory_slot slot;

	ret = mem_slot_get(core, slot_index, &slot);
	if (ret < 0) {
		slot_str->state = WGW_EC_MEM_SLOT_STATE_ERROR;
		strcpy(slot_str->data, error_str);
		return ret;
	}
	if (ret > WGW_EC_MEM_SLOT_STR_SIZE) {
		slot_str->state = WGW_EC_MEM_SLOT_STATE_ERROR;
		strcpy(slot_str->data, error_str);
		dev_err(core->dev,
			"string in slot[%d] read from device is too long (%d)\n",
			slot_index, ret);
		return -EIO;
	}
	slot_str->state = 0;
	if (slot.flags & WGW_EC_MEM_SLOT_OTP)
		slot_str->state |= WGW_EC_MEM_SLOT_STATE_OTP;

	if (!(slot.flags & WGW_EC_MEM_SLOT_SET)) {
		strcpy(slot_str->data, undefined_str);
	} else {
		memcpy(slot_str->data, slot.data, slot.length);
		slot_str->data[slot.length] = '\0';
		slot_str->state |= WGW_EC_MEM_SLOT_STATE_SET;
	}
	return 0;
}

static int mem_slot_get_u8(struct wgw_ec_core *core, u8 slot_index,
			   struct wgw_ec_slot_u8 *slot_u8)
{
	int ret;
	struct wgw_ec_memory_slot slot;

	ret = mem_slot_get(core, slot_index, &slot);
	if (ret < 0) {
		slot_u8->state = WGW_EC_MEM_SLOT_STATE_ERROR;
		return ret;
	}
	if (ret > 1) {
		dev_err(core->dev,
			"data in slot[%d] is too big to fit in a u8 (%d)\n",
			slot_index, ret);
		return -EIO;
	}

	slot_u8->state = slot.flags;
	slot_u8->value = slot.data[0];
	return 0;
}

static int product_serial_get(struct wgw_ec_core *core,
			      struct wgw_ec_slot_str *serial)
{
	int ret = mem_slot_get_str(core, 0, serial);
	if (ret < 0)
		dev_err(core->dev, "error retrieving product serial (%d)\n",
			ret);
	return ret;
}

static int product_model_get(struct wgw_ec_core *core,
			     struct wgw_ec_slot_u8 *model)
{
	int ret = mem_slot_get_u8(core, 1, model);
	if (ret < 0)
		dev_err(core->dev, "error retrieving product model (%d)\n",
			ret);
	return ret;
}

static int product_version_get(struct wgw_ec_core *core,
			       struct wgw_ec_slot_str *version)
{
	int ret = mem_slot_get_str(core, 2, version);
	if (ret < 0)
		dev_err(core->dev, "error retrieving product version (%d)\n",
			ret);
	return ret;
}

static int product_variant_get(struct wgw_ec_core *core,
			       struct wgw_ec_slot_u8 *variant)
{
	int ret = mem_slot_get_u8(core, 3, variant);
	if (ret < 0)
		dev_err(core->dev, "error retrieving product variant (%d)\n",
			ret);
	return ret;
}

static int fw_info_get(struct wgw_ec_core *core, struct wgw_ec_fw_info *fw_info)
{
	struct wgw_ec_transport *transport = core->transport;
	struct wgw_ec_reg reg;
	int32_t ret;

	ret = transport->read_block(transport, WGW_EC_REG_FW_INFO1, reg.data);
	if (ret < 0) {
		dev_err(core->dev, "failed to read firmware version (%d)\n",
			ret);
		return ret;
	}
	if (ret != sizeof(reg.fw_info_version)) {
		dev_err(core->dev,
			"failed to read fw version cause of wrong returned size (%d)\n",
			ret);
		return -EIO;
	}
	fw_info->version = reg.fw_info_version;
	fw_version_str(fw_info->version_str, &reg.fw_info_version,
		       WGW_EC_FW_VERSION_SIZE);
	dev_dbg(core->dev, "mainboard fw version: %s\n", fw_info->version_str);

	ret = transport->read_block(transport, WGW_EC_REG_FW_INFO2, reg.data);
	if (ret < 0) {
		dev_err(core->dev, "failed to read firmware commit hash (%d)\n",
			ret);
		return ret;
	}
	if (ret >= WGW_EC_APP_COMMIT_HASH_SIZE) {
		dev_err(core->dev,
			"firmware commit hash string is too long (%d)\n", ret);
		return -EIO;
	}
	memcpy(&fw_info->commit_hash[0], reg.data, ret);
	fw_info->commit_hash[ret] = '\0';
	dev_dbg(core->dev, "mainboard fw version hash: %s\n",
		fw_info->commit_hash);

	ret = transport->read_block(transport, WGW_EC_REG_FW_INFO3, reg.data);
	if (ret < 0) {
		dev_err(core->dev, "failed to read firmware commit date (%d)\n",
			ret);
		return ret;
	}
	if (ret >= WGW_EC_APP_COMMIT_DATE_SIZE) {
		dev_err(core->dev,
			"firmware commit date string is too long (%d)\n", ret);
		return -EIO;
	}
	memcpy(&fw_info->commit_date[0], reg.data, ret);
	fw_info->commit_date[ret] = '\0';
	dev_dbg(core->dev, "mainboard fw version date: %s\n",
		fw_info->commit_date);

	return 0;
}

static int mainboard_info_get(struct wgw_ec_core *core,
			      struct wgw_ec_mainboard_info *mb_info)
{
	struct wgw_ec_transport *transport = core->transport;
	struct device *dev = core->dev;
	struct wgw_ec_reg reg;
	int32_t ret;

	ret = transport->read_block(transport, WGW_EC_REG_HW_INFO, reg.data);
	if (ret < 0) {
		dev_err(dev, "failed to read hw info (%d)\n", ret);
		return ret;
	}
	if (ret != sizeof(reg.hw_info)) {
		dev_err(dev,
			"failed to read hw info register (wrong returned size)\n");
		return -EIO;
	}

	hw_version_str(mb_info->version_str, &reg.hw_info.version,
		       WGW_EC_HW_VERSION_SIZE);
	dev_dbg(dev, "mainboard HW version: %s\n", mb_info->version_str);

	mb_info->model.id = mainboard_ref_index(reg.hw_info.model);
	mb_info->model.str = mainboard_ref_str(reg.hw_info.model);

	switch (mb_info->model.id) {
	case WGW_EC_MB_WGW_L02_BASE_L1:
	case WGW_EC_MB_WGW_L02_BASE_L1_4G:
		mb_info->base_model.id = WGW_EC_MB_WGW_L02_BASE_L1;
		mb_info->base_model.str =
			mainboard_ref_str(mb_info->base_model.id);
		break;
	default:
		dev_err(dev, "mainboard model '%s' (%d) is not supported\n",
			mainboard_ref_str(mb_info->model.id),
			mb_info->model.id);
		return -EIO;
	}

	mb_info->variant.id = mainboard_variant_index(reg.hw_info.frequency);
	mb_info->variant.str = mainboard_variant_str(reg.hw_info.frequency);
	dev_dbg(dev, "mainboard variant: %s (%d)\n", mb_info->variant.str,
		mb_info->variant.id);

	ret = fw_info_get(core, &mb_info->fw);
	if (ret < 0)
		dev_err(dev, "failed to read firmware info (%d)\n", ret);

	return ret;
}

int wgw_ec_boot_state_get(struct wgw_ec_core *core, u8 *boot_state)
{
	struct wgw_ec_transport *transport = core->transport;
	int ret = transport->read_byte(transport, WGW_EC_REG_LAST_RESET_STATE,
				       boot_state);
	if (ret < 0) {
		dev_err(core->dev, "failed to read from device\n");
		*boot_state = 0xFF;
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(wgw_ec_boot_state_get);

int wgw_ec_boot_state_clr_update(struct wgw_ec_core *core, u8 *boot_state)
{
	struct wgw_ec_transport *transport = core->transport;
	int ret = transport->write_byte(transport, WGW_EC_REG_LAST_RESET_STATE,
					0xFF);
	if (ret < 0) {
		dev_err(core->dev, "failed to write to device\n");
		return ret;
	}
	return wgw_ec_boot_state_get(core, boot_state);
}
EXPORT_SYMBOL_GPL(wgw_ec_boot_state_clr_update);

static int detect_wgw_ec(struct wgw_ec_core *core)
{
	struct wgw_ec_info *cache = &core->cache_info;
	struct wgw_ec_transport *transport = core->transport;
	struct device *dev = core->dev;
	u8 protoc;
	int ret;

	mutex_lock(&core->cache_lock);

	ret = transport->read_byte(transport, WGW_EC_REG_PROTOC_VER, &protoc);
	if (ret < 0) {
		dev_err(dev, "failed to read protocole version (%d)\n", ret);
		goto out;
	}
	dev_info(dev, "detected wgw-ec, protocol version=%d\n", protoc);

	if (protoc != 2) {
		dev_err(dev, "protocol version %d not supported\n", protoc);
		ret = -EPROTO;
		goto out;
	}

	cache->protoc = protoc;
	ret = 0;
out:
	mutex_unlock(&core->cache_lock);
	return ret;
}

static int fetch_cache_info(struct wgw_ec_core *core)
{
	struct wgw_ec_info *cache = &core->cache_info;
	struct wgw_ec_slot_str slot_str;
	struct wgw_ec_slot_u8 slot_u8;
	int ret;

	mutex_lock(&core->cache_lock);

	ret = mainboard_info_get(core, &cache->mainboard);
	if (ret < 0) {
		dev_err(core->dev, "failed to read mainboard info (%d)\n", ret);
		goto out;
	}

	ret = product_model_get(core, &slot_u8);
	if (ret < 0) {
		dev_err(core->dev,
			"could not determine product model, possibly corrupted (mb id: %d)\n",
			cache->mainboard.model.id);
		cache->product.model.id = WGW_EC_MODEL_VAR_ERROR;
	} else if (slot_u8.state < WGW_EC_MEM_SLOT_STATE_SET) {
		dev_dbg(core->dev, "product model slot isn't set, state: %d\n",
			slot_u8.state);
		if (cache->mainboard.model.id != WGW_EC_MB_WGW_L02_BASE_L1) {
			dev_err(core->dev,
				"could not determine product model, undefined (mb id: %d)\n",
				cache->mainboard.model.id);
		}
		cache->product.model.id = WGW_EC_MODEL_VAR_UNDEFINED;
	} else {
		cache->product.model.id = slot_u8.value;
	}

	ret = product_variant_get(core, &slot_u8);
	if (ret < 0) {
		dev_err(core->dev,
			"could not determine product variant, possibly corrupted: %d\n",
			ret);
		cache->product.variant.id = WGW_EC_MODEL_VAR_ERROR;
	} else if (slot_u8.state < WGW_EC_MEM_SLOT_STATE_SET) {
		dev_dbg(core->dev,
			"product variant slot isn't set, state: %d\n",
			slot_u8.state);
		if (cache->product.model.id == WGW_EC_MODEL_VAR_UNDEFINED &&
		    cache->mainboard.model.id == WGW_EC_MB_WGW_L02_BASE_L1) {
			cache->product.variant.id = cache->mainboard.variant.id;
		} else {
			dev_err(core->dev,
				"could not determine product variant, undefined\n");
			cache->product.variant.id = WGW_EC_MODEL_VAR_UNDEFINED;
		}
	} else {
		cache->product.variant.id = slot_u8.value;
	}

	ret = product_version_get(core, &slot_str);
	if (ret < 0) {
		dev_err(core->dev,
			"could not determine product version, possibly corrupted: %d\n",
			ret);
		strcpy(cache->product.version_str, error_str);
	} else if (slot_str.state < WGW_EC_MEM_SLOT_STATE_SET) {
		if (cache->product.model.id == WGW_EC_MODEL_VAR_UNDEFINED &&
		    cache->mainboard.model.id == WGW_EC_MB_WGW_L02_BASE_L1) {
			strcpy(cache->product.version_str,
			       cache->mainboard.version_str);
		} else {
			dev_err(core->dev,
				"could not determine product version, undefined\n");
			strcpy(cache->product.version_str, undefined_str);
		}
	} else {
		strcpy(cache->product.version_str, slot_str.data);
	}

	if (cache->product.model.id == WGW_EC_MODEL_VAR_UNDEFINED &&
	    cache->mainboard.model.id == WGW_EC_MB_WGW_L02_BASE_L1)
		cache->product.model.id = WGW_EC_M_WIFX_L1;

	cache->product.model.str = model_str(cache->product.model.id);
	cache->product.variant.str = model_variant_str(
		cache->product.model.id, cache->product.variant.id);

	ret = product_serial_get(core, &cache->product.serial);
	if (ret < 0)
		dev_err(core->dev,
			"could not determine product serial, possibly corrupted: %d\n",
			ret);

	ret = wgw_ec_boot_state_get(core, &cache->boot_state);
	if (ret < 0) {
		dev_err(core->dev, "failed to read boot state (%d)\n", ret);
		goto out;
	}

	ret = 0;
out:
	mutex_unlock(&core->cache_lock);
	return ret;
}

static int display_cache_info(struct wgw_ec_core *core)
{
	struct wgw_ec_info *cache = &core->cache_info;
	int ret = -ENODEV;

	mutex_lock(&core->cache_lock);

	switch (cache->product.model.id) {
	case WGW_EC_M_WIFX_L1:
	case WGW_EC_M_WIFX_L1_4G:
		break;
	default:
		dev_err(core->dev, "Unknown product detected (id=%d)\n",
			cache->mainboard.model.id);
		goto out;
	}

	dev_info(core->dev, "Found Wifx product, model: %s, variant: %s\n",
		 model_pretty_str((enum wgw_ec_model)cache->product.model.id),
		 cache->product.variant.str);
	if (cache->product.variant.id < 0) {
		dev_warn(
			core->dev,
			"failed to determine correctly product variant, support could be limited.\n");
	}

	if (!cache->product.serial.state) {
		dev_warn(core->dev, "Serial: %s\n", cache->product.serial.data);
	} else if (cache->product.serial.state < 0) {
		dev_err(core->dev, " Serial: %s\n", cache->product.serial.data);
	} else {
		dev_info(core->dev, "Serial: %s\n", cache->product.serial.data);
		if (!(cache->product.serial.state &
		      WGW_EC_MEM_SLOT_STATE_OTP)) {
			dev_warn(core->dev, "serial is not locked\n");
		} else if (!(cache->product.serial.state &
			     WGW_EC_MEM_SLOT_STATE_SET)) {
			dev_err(core->dev,
				"serial is locked with null value\n");
		}
	}
	dev_info(core->dev, "Product version: %s\n",
		 cache->product.version_str);
	dev_info(core->dev, "Firmware version: %s (%s) [%s]\n",
		 cache->mainboard.fw.version_str,
		 cache->mainboard.fw.commit_hash,
		 cache->mainboard.fw.commit_date);

	switch (cache->boot_state) {
	case 0x00:
		dev_info(core->dev, "Boot: 0x00 (normal mode)\n");
		break;
	case 0x01:
		dev_info(core->dev, "Boot: 0x01 (factory reset mode)\n");
		break;
	default:
		dev_info(core->dev,
			 "Boot: 0x%02X (unknown mode), clearing boot state\n",
			 cache->boot_state);
		ret = wgw_ec_boot_state_clr_update(core, &cache->boot_state);
		if (ret < 0) {
			mutex_unlock(&core->cache_lock);
			return ret;
		}
		break;
	}

	ret = 0;
out:
	mutex_unlock(&core->cache_lock);
	return ret;
}

/* ------------------- MFD cells ------------------- */

static struct class wifx_class = {
	.name = "wifx",
};

static struct mfd_cell wgw_ec_mfd_cells[] = {
	{
		.name = "wgw-ec-leds",
		.of_compatible = "wifx,wgw-ec-leds",
	},
	{
		.name = "wgw-ec-usbc",
		.of_compatible = "wifx,wgw-ec-usbc",
	},
};

static const struct mfd_cell wgw_ec_platform_cells[] = {
	{
		.name = "wgw-ec-chardev",
	},
	{
		.name = "wgw-ec-sysfs",
	},
};

/* ------------------- Probe / Remove ------------------- */

static void wgw_ec_class_release(struct device *dev)
{
	dev_dbg(dev, "wgw-ec-class release\n");
	kfree(to_wgw_ec_core(dev));
}

static int wgw_ec_core_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	struct device *dev = &pdev->dev;
	struct wgw_ec_core *core = kzalloc(sizeof(*core), GFP_KERNEL);
	struct wgw_ec_transport *transport;

	dev_dbg(dev, "wgw-ec-core probe\n");

	if (!core)
		return -ENOMEM;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		kfree(core);
		return ret;
	}

	mutex_init(&core->cache_lock);
	dev_set_drvdata(dev, core);

	/* Link to transport instance from parent (I2C/SPI) */
	transport = dev_get_drvdata(dev->parent);
	if (!transport) {
		dev_err(dev, "no transport instance from parent\n");
		ret = -ENODEV;
		goto err_free_ec;
	}
	core->transport = transport;
	core->dev = dev;
	transport->core = core;

	/* Init core-owned infra on the transport handle (notifier/mutex) */
	BLOCKING_INIT_NOTIFIER_HEAD(&transport->notifier_list);
	mutex_init(&transport->lock_ltr);
	INIT_DELAYED_WORK(&transport->irq_work, wgw_ec_irq_work);

	/* Request IRQ from this core's DT node (child of the transport) */
	irq = platform_get_irq(pdev, 0);
	if (irq > 0) {
		int wret;

		transport->irq = irq;
		ret = devm_request_threaded_irq(
			dev, irq, NULL, wgw_ec_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "wgw-ec-irq",
			transport);
		if (ret) {
			dev_err(dev, "Failed to request IRQ %d: %d\n", irq,
				ret);
			goto err_free_ec;
		}
		/* Best effort: clear the interrupt latch in EC */
		wret = transport->write_byte(transport, WGW_EC_REG_INTERRUPT,
					     0xFF);
		if (wret < 0)
			dev_warn(dev,
				 "failed to clear EC interrupt latch: %d\n",
				 wret);
	} else if (irq == -ENXIO) {
		/* No IRQ resource found */
		dev_warn(dev, "no IRQ provided to EC core\n");
	} else if (irq < 0) {
		dev_err(dev, "failed to get IRQ: %d\n", irq);
		goto err_free_ec;
	}

	/* cpu-state GPIO from this core node */
	transport->cpu_state_pin =
		devm_gpiod_get_optional(dev, "cpu-state", GPIOD_OUT_HIGH);
	if (IS_ERR(transport->cpu_state_pin)) {
		ret = PTR_ERR(transport->cpu_state_pin);
		dev_err(dev, "Failed to request cpu-state gpio: %d\n", ret);
		goto err_free_ec;
	} else if (!transport->cpu_state_pin) {
		dev_warn(
			dev,
			"No cpu-state gpio provided, functionalities will be limited\n");
	}

	/* Detect EC and fetch/cache info */
	ret = detect_wgw_ec(core);
	if (ret) {
		dev_err(dev, "failed to detect embedded controller\n");
		goto err_free_ec;
	}

	ret = fetch_cache_info(core);
	if (ret) {
		dev_err(dev, "failed to fetch machine information\n");
		goto err_free_ec;
	}
	display_cache_info(core);

	/* Optional class device (unchanged behaviour) */
	device_initialize(&core->class_dev);
	core->class_dev.class = &wifx_class;
	core->class_dev.parent = dev;
	core->class_dev.release = wgw_ec_class_release;

	ret = dev_set_name(&core->class_dev, "%s", "wgw-ec");
	if (ret) {
		dev_err(dev, "dev_set_name failed => %d\n", ret);
		goto err_put_class_dev_no_unregister;
	}

	ret = device_add(&core->class_dev);
	if (ret)
		goto err_put_class_dev_no_unregister;

	/* Add implicit platform subdevices (chardev/sysfs) */
	ret = mfd_add_hotplug_devices(dev, wgw_ec_platform_cells,
				      ARRAY_SIZE(wgw_ec_platform_cells));
	if (ret) {
		dev_warn(dev, "failed to add wgw-ec platform devices: %d\n",
			 ret);
		goto err_unregister_class_dev;
	}

	/* Add MFD children (leds/usbc) that match DT subnodes under ec-core */
	dev_dbg(dev, "adding MFD cells with parent of_node=%pOF fwnode=%pfw\n",
		dev->of_node, dev_fwnode(dev));
	ret = mfd_add_devices(dev, PLATFORM_DEVID_AUTO, wgw_ec_mfd_cells,
			      ARRAY_SIZE(wgw_ec_mfd_cells), NULL, 0, NULL);
	if (ret) {
		dev_warn(dev, "failed to add wgw-ec subdevices: %d\n", ret);
		goto err_remove_mfd;
	}

	dev_info(dev, "Wifx EC Core ready\n");
	dev_dbg(dev, "of_node=%pOF fwnode=%pfw\n", dev->of_node,
		dev_fwnode(dev));

	return 0;

err_remove_mfd:
	mfd_remove_devices(dev);

err_unregister_class_dev:
	device_unregister(&core->class_dev);
	return ret;

err_put_class_dev_no_unregister:
	put_device(&core->class_dev);
err_free_ec:
	kfree(core);
	return ret;
}

static void wgw_ec_core_remove(struct platform_device *pdev)
{
	struct wgw_ec_core *core = dev_get_drvdata(&pdev->dev);

	dev_dbg(&pdev->dev, "wgw-ec-core remove\n");
	if (core && core->transport) {
		WRITE_ONCE(core->transport->core, NULL);
		cancel_delayed_work_sync(&core->transport->irq_work);
	}

	mfd_remove_devices(&pdev->dev);
	if (core) {
		/* device_unregister will trigger class_dev->release to kfree(ec) */
		device_unregister(&core->class_dev);
	}
}

#ifdef CONFIG_PM_SLEEP
static int wgw_ec_core_suspend(struct device *dev)
{
	struct wgw_ec_core *core = dev_get_drvdata(dev);

	if (!core || !core->transport)
		return 0;

	cancel_delayed_work_sync(&core->transport->irq_work);

	return 0;
}

static int wgw_ec_core_resume(struct device *dev)
{
	struct wgw_ec_core *core = dev_get_drvdata(dev);

	if (!core || !core->transport)
		return 0;

	/*
	 * Resync IRQ-driven consumers after system resume in case an EC status
	 * bit changed while the workqueue path was quiesced.
	 */
	mod_delayed_work(system_wq, &core->transport->irq_work, 0);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(wgw_ec_core_pm_ops, wgw_ec_core_suspend,
			 wgw_ec_core_resume);

/* ------------------- OF/ID tables and module boilerplate ------------------- */

static const struct of_device_id wgw_ec_core_of_match[] = {
	{ .compatible = "wifx,wgw-ec-core" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wgw_ec_core_of_match);

static struct platform_driver wgw_ec_core_driver = {
	.probe  = wgw_ec_core_probe,
	.remove = wgw_ec_core_remove,
	.driver = {
		.name           = DRV_NAME,
		.of_match_table = wgw_ec_core_of_match,
		.pm             = &wgw_ec_core_pm_ops,
	},
};

static int __init wgw_ec_core_init(void)
{
	int ret;

	ret = class_register(&wifx_class);
	if (ret) {
		pr_err("wgw-ec-core: failed to register device class\n");
		return ret;
	}

	ret = platform_driver_register(&wgw_ec_core_driver);
	if (ret) {
		pr_err("wgw-ec-core: can't register driver: %d\n", ret);
		class_unregister(&wifx_class);
		return ret;
	}
	return 0;
}

static void __exit wgw_ec_core_exit(void)
{
	platform_driver_unregister(&wgw_ec_core_driver);
	class_unregister(&wifx_class);
}

module_init(wgw_ec_core_init);
module_exit(wgw_ec_core_exit);

MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_DESCRIPTION(
	"Core platform driver for the Wifx board Embedded Controller");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
