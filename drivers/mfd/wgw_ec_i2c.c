// SPDX-License-Identifier: GPL-2.0
/*
 * I2C driver for the Wifx board EC (transport layer)
 *
 * Copyright (C) 2021 Wifx SA,
 *               2021 Yannick Lanz <yannick.lanz@wifx.net>
 *               2026 Yannick Serafini <yannick.serafini@wifx.net>
 */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#include <linux/mfd/wgw-ec/core.h>
#include <linux/mfd/wgw-ec/reg.h>

#define WGW_EC_DUMP_MAX 32
#ifdef DEBUG

static void wgw_ec_i2c_dbg_dump(struct device *dev, const char *tag,
				const u8 *buf, size_t len)
{
	char line[256];
	size_t pos = 0;
	size_t i;
	size_t max = min(len, (size_t)WGW_EC_DUMP_MAX);

	pos += scnprintf(line + pos, sizeof(line) - pos,
			 "wgw-ec-i2c: %s, data=", tag);
	for (i = 0; i < max && pos < sizeof(line); i++)
		pos += scnprintf(line + pos, sizeof(line) - pos, "0x%02x%s",
				 buf[i], (i + 1 < max) ? " " : "");
	if (len > WGW_EC_DUMP_MAX)
		scnprintf(line + pos, sizeof(line) - pos,
			  " ... (%zu remaining bytes)", len - WGW_EC_DUMP_MAX);

	dev_dbg(dev, "%s\n", line);
}
#else
static inline void wgw_ec_i2c_dbg_dump(struct device *dev, const char *tag,
				       const u8 *buf, size_t len)
{
}
#endif

/* Returns length read (>= 0) or error (< 0) */
static int read_byte(struct wgw_ec_transport *wgw_dev, char command, u8 *data)
{
	struct i2c_client *client = wgw_dev->priv;
	s32 result = i2c_smbus_read_byte_data(client, command);
	u8 tx_buf[1] = { (u8)command };

	wgw_ec_i2c_dbg_dump(&client->dev, "tx byte", tx_buf, sizeof(tx_buf));
	if (result < 0)
		return result;

	*data = (u8)result;
	wgw_ec_i2c_dbg_dump(&client->dev, "rx byte", data, 1);
	return 1;
}

static int read_word(struct wgw_ec_transport *wgw_dev, char command, u16 *data)
{
	struct i2c_client *client = wgw_dev->priv;
	s32 result = i2c_smbus_read_word_data(client, command);
	u8 tx_buf[1] = { (u8)command };
	u8 rx_buf[2];

	wgw_ec_i2c_dbg_dump(&client->dev, "tx word", tx_buf, sizeof(tx_buf));
	if (result < 0)
		return result;

	*data = (u16)result;
	rx_buf[0] = (u8)(*data & 0xff);
	rx_buf[1] = (u8)((*data >> 8) & 0xff);
	wgw_ec_i2c_dbg_dump(&client->dev, "rx word", rx_buf, sizeof(rx_buf));
	return 2;
}

static int read_block(struct wgw_ec_transport *wgw_dev, char command, u8 *data)
{
	struct i2c_client *client = wgw_dev->priv;
	int ret;
	u8 tx_buf[1] = { (u8)command };

	/* Returns number of bytes read or negative error */
	wgw_ec_i2c_dbg_dump(&client->dev, "tx block", tx_buf, sizeof(tx_buf));
	ret = (int)i2c_smbus_read_block_data(client, command, data);
	if (ret >= 0)
		wgw_ec_i2c_dbg_dump(&client->dev, "rx block", data,
				    (size_t)ret);
	return ret;
}

/* Returns length written (>= 0) or error (< 0) */
static int write_byte(struct wgw_ec_transport *wgw_dev, char command, u8 data)
{
	struct i2c_client *client = wgw_dev->priv;
	s32 result = i2c_smbus_write_byte_data(client, command, data);
	u8 tx_buf[2] = { (u8)command, data };

	wgw_ec_i2c_dbg_dump(&client->dev, "tx byte", tx_buf, sizeof(tx_buf));
	if (result < 0)
		return result;

	return 1;
}

static int write_word(struct wgw_ec_transport *wgw_dev, char command, u16 data)
{
	struct i2c_client *client = wgw_dev->priv;
	s32 result = i2c_smbus_write_word_data(client, command, data);
	u8 tx_buf[3] = { (u8)command, (u8)(data & 0xff),
			 (u8)((data >> 8) & 0xff) };

	wgw_ec_i2c_dbg_dump(&client->dev, "tx word", tx_buf, sizeof(tx_buf));
	if (result < 0)
		return result;

	return 2;
}

static int write_block(struct wgw_ec_transport *wgw_dev, char command,
		       const u8 *data, u8 len)
{
	struct i2c_client *client = wgw_dev->priv;
	u8 tx_buf[WGW_EC_DUMP_MAX + 1];
	size_t copy_len = min((size_t)len, (size_t)WGW_EC_DUMP_MAX);

	/* Returns number of bytes written or negative error */
	tx_buf[0] = (u8)command;
	if (copy_len)
		memcpy(&tx_buf[1], data, copy_len);
	wgw_ec_i2c_dbg_dump(&client->dev, "tx block", tx_buf, copy_len + 1);
	return (int)i2c_smbus_write_block_data(client, command, len, data);
}

static int wgw_ec_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct wgw_ec_transport *wgw_dev;
	int err;

	dev_dbg(dev, "wgw-ec-i2c probe\n");

	wgw_dev = devm_kzalloc(dev, sizeof(*wgw_dev), GFP_KERNEL);
	if (!wgw_dev)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "SMBus Byte Data not supported\n");
		return -EIO;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(dev, "SMBus Word Data not supported\n");
		return -EIO;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BLOCK_DATA)) {
		dev_err(dev, "SMBus Block Data not supported\n");
		return -EIO;
	}

	if (i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_PEC))
		client->flags |= I2C_CLIENT_PEC;
	else
		dev_warn(dev, "SMBus PEC not supported, running without PEC\n");

	/* Publish transport ops to the core */
	i2c_set_clientdata(client, wgw_dev);
	wgw_dev->dev = dev;
	wgw_dev->priv = client;
	wgw_dev->read_byte = read_byte;
	wgw_dev->read_word = read_word;
	wgw_dev->read_block = read_block;
	wgw_dev->write_byte = write_byte;
	wgw_dev->write_word = write_word;
	wgw_dev->write_block = write_block;
	wgw_dev->phys_name = client->adapter->name;

	/*
	 * Make the transport-visible from the parent device so that
	 * the core (child PlatformDevice bound to DT node "wifx,wgw-ec-core")
	 * can retrieve it via dev_get_drvdata(dev->parent).
	 */
	dev_set_drvdata(dev, wgw_dev);

	dev_info(dev, "Wifx EC I2C transport ready\n");
	dev_dbg(dev, "of_node=%pOF, fwnode=%pfw\n", dev->of_node,
		dev_fwnode(dev));

	/*
	 * Populate DT children (e.g. "wifx,wgw-ec-core") under this I2C node.
	 * Children will use &client->dev as their parent.
	 */
	err = of_platform_populate(dev_of_node(dev), NULL, NULL, dev);
	if (err) {
		dev_err(dev, "failed to populate DT children: %d\n", err);
		return dev_err_probe(dev, err, "populate DT children failed\n");
	}

	return 0;
}

static void wgw_ec_i2c_remove(struct i2c_client *client)
{
	struct wgw_ec_transport *wgw_dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "wgw-ec-i2c remove\n");

	/* Remove DT-populated children */
	of_platform_depopulate(&client->dev);

	if (wgw_dev)
		dev_set_drvdata(&client->dev, NULL);
}

static const struct i2c_device_id wgw_ec_i2c_id[] = { { "wgw-ec-i2c", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, wgw_ec_i2c_id);

static const struct of_device_id wgw_ec_of_match[] = {
	{ .compatible = "wifx,wgw-ec-i2c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wgw_ec_of_match);

static struct i2c_driver wgw_ec_i2c_driver = {
	.driver = {
		.name = "wgw-ec-i2c",
		.of_match_table = of_match_ptr(wgw_ec_of_match),
	},
	.probe = wgw_ec_i2c_probe,
	.remove = wgw_ec_i2c_remove,
	.id_table = wgw_ec_i2c_id,
};

module_i2c_driver(wgw_ec_i2c_driver);

MODULE_AUTHOR("Yannick Serafini <yannick.serafini@wifx.net>");
MODULE_DESCRIPTION("I2C transport driver for the Wifx Embedded Controller");
MODULE_LICENSE("GPL v2");
