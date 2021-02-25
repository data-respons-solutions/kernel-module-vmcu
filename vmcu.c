/*
 * Copyright (c) 2021  Data Respons
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

static const struct regmap_config vmcu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

struct vmcu {
	struct i2c_client		*client;
	struct regmap			*regmap;
};

static int probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vmcu *vmcu = NULL;
	uint reg = 0;
	int r = 0;

	dev_err(&client->dev, "We're here: %s\n", __func__);

	vmcu = devm_kzalloc(&client->dev, sizeof(struct vmcu), GFP_KERNEL);
	if (!vmcu) {
		return -ENOMEM;
	}

	vmcu->client = client;
	vmcu->regmap = devm_regmap_init_i2c(client, &vmcu_regmap_config);
	if (IS_ERR(vmcu->regmap)) {
		dev_err(&client->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}
	i2c_set_clientdata(client, vmcu);

	r = regmap_read(vmcu->regmap, 0x00, &reg);
	if (r < 0) {
		dev_err(&client->dev, "Failed reading regmap: %d\n", r);
		return r;
	}
	dev_err(&client->dev, "Reg 0x00: %x\n", reg);

	return 0;
}

static const struct of_device_id of_vmcu_match[] = { { .compatible =
	"datarespons,vec6200-mcu" }, { /* Sentinel */} };

static struct i2c_device_id vmcu_id[] = { { "vec6200-mcu", 0 }, { } };
MODULE_DEVICE_TABLE(i2c, vmcu_id);

static struct i2c_driver vmcu_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "vmcu",
		.of_match_table = of_vmcu_match,
		//.pm = &vmcu_pm_ops,
	},
	.id_table = vmcu_id,
	.probe = probe,
	//.remove = vmcu_remove,
	//.shutdown = vmcu_shutdown,
};
module_i2c_driver(vmcu_driver);

MODULE_AUTHOR("Mikko Salomäki <ms@datarespons.se>");
MODULE_DESCRIPTION("Vehicle MCU driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
