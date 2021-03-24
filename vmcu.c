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
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/mtd/mtd.h>

#define MAGIC_REG_OFFSET		0x0
#define MAGIC_REG_VALUE			0x0ec70add
#define VERSION_REG_OFFSET		0x1
#define RTC_TIME_REG_OFFSET		0x2
#define RTC_TIME_HOUR_SHIFT		0
#define RTC_TIME_MIN_SHIFT		8
#define RTC_TIME_SEC_SHIFT		16
#define RTC_DATE_REG_OFFSET		0x3
#define RTC_DATE_YEAR_SHIFT		0
#define RTC_DATE_MONTH_SHIFT	8
#define RTC_DATE_DAY_SHIFT		16
#define RTC_DATE_WDAY_SHIFT		24
#define FSTATUS_REG_OFFSET		0x20
#define FSTATUS_BUSY_MASK		(1 << 0)
#define FSTATUS_ERASE_MASK		(1 << 2)
#define FSIZE_REG_OFFSET		0x21
#define FPTR_REG_OFFSET			0x22
#define FREAD_REG_OFFSET		0x23
#define FWRITE_REG_OFFSET		0x24

static const struct regmap_range volatile_ranges[] = {
	regmap_reg_range(RTC_TIME_REG_OFFSET, RTC_DATE_REG_OFFSET),
	regmap_reg_range(FSTATUS_REG_OFFSET, FSTATUS_REG_OFFSET),
	regmap_reg_range(FPTR_REG_OFFSET, FWRITE_REG_OFFSET),
};

static const struct regmap_access_table volatile_access_table = {
	.yes_ranges = volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(volatile_ranges),
};

static const struct regmap_config vmcu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.volatile_table = &volatile_access_table,
};

struct vmcu {
	struct mutex			mtx;
	struct regmap			*regmap;
	struct i2c_client		*client;
	struct rtc_device 		*rtc;
	struct mtd_info			mtd;
};

static int rtc_set(struct device *dev, struct rtc_time *rtctime)
{
	struct vmcu *vmcu = dev_get_drvdata(dev);
	int r = 0;

	uint32_t date = bin2bcd(rtctime->tm_year - 100) << RTC_DATE_YEAR_SHIFT
			| bin2bcd(rtctime->tm_mon + 1) << RTC_DATE_MONTH_SHIFT
			| bin2bcd(rtctime->tm_mday) << RTC_DATE_DAY_SHIFT
			| bin2bcd(rtctime->tm_wday + 1) << RTC_DATE_WDAY_SHIFT;

	uint32_t time = bin2bcd(rtctime->tm_hour) << RTC_TIME_HOUR_SHIFT
			| bin2bcd(rtctime->tm_min) << RTC_TIME_MIN_SHIFT
			| bin2bcd(rtctime->tm_sec) << RTC_TIME_SEC_SHIFT;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;

	r = regmap_write(vmcu->regmap, RTC_DATE_REG_OFFSET, date);
	if (r == 0)
		r = regmap_write(vmcu->regmap, RTC_TIME_REG_OFFSET, time);

	mutex_unlock(&vmcu->mtx);

	if (r < 0)
		return r;

	dev_dbg(dev, "RTC time write: %d:%d:%d - %d/%d/%d, wd=%d\n",
		rtctime->tm_hour, rtctime->tm_min, rtctime->tm_sec,
		rtctime->tm_year, rtctime->tm_mon, rtctime->tm_mday,
		rtctime->tm_wday);

	return 0;
}

static int rtc_read(struct device *dev, struct rtc_time *rtctime)
{
	struct vmcu *vmcu = dev_get_drvdata(dev);
	uint32_t time = 0;
	uint32_t date = 0;
	int r = 0;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;

	r = regmap_read(vmcu->regmap, RTC_TIME_REG_OFFSET, &time);
	if (r == 0)
		r = regmap_read(vmcu->regmap, RTC_DATE_REG_OFFSET, &date);

	mutex_unlock(&vmcu->mtx);

	if (r < 0)
		return r;

	rtctime->tm_sec = bcd2bin(time >> RTC_TIME_SEC_SHIFT & 0xff);
	rtctime->tm_min = bcd2bin(time >> RTC_TIME_MIN_SHIFT & 0xff);
	rtctime->tm_hour = bcd2bin(time >> RTC_TIME_HOUR_SHIFT & 0xff);
	rtctime->tm_mday = bcd2bin(date >> RTC_DATE_DAY_SHIFT);
	rtctime->tm_mon = bcd2bin(date >> RTC_DATE_MONTH_SHIFT) - 1;
	rtctime->tm_year = bcd2bin(date >> RTC_DATE_YEAR_SHIFT) + 100;
	rtctime->tm_wday = bcd2bin(date >> RTC_DATE_WDAY_SHIFT) - 1;
	rtctime->tm_yday = 0;
	rtctime->tm_isdst = 0;

	dev_dbg(dev, "RTC time read: %d:%d:%d - %d/%d/%d, wd=%d\n",
		rtctime->tm_hour, rtctime->tm_min, rtctime->tm_sec,
		rtctime->tm_year, rtctime->tm_mon, rtctime->tm_mday,
		rtctime->tm_wday);

	return 0;
}

static struct rtc_class_ops vmcu_rtc_ops = {
	.read_time = rtc_read, .set_time = rtc_set, };

static int flash_busy(struct regmap* regmap)
{
	u32 val = 0;
	int r = regmap_read(regmap, FSTATUS_REG_OFFSET, &val);
	if (r)
		return r;
	if ((val & FSTATUS_BUSY_MASK) == FSTATUS_BUSY_MASK)
		return -EAGAIN;
	return 0;
}

static int flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct regmap *regmap = mtd->priv;
	u32 val = 0;
	int r = 0;

	if (instr->addr != 0 || instr->len != mtd->size)
		return -EINVAL;

	r = flash_busy(regmap);
	if (r)
		return r;

	r = regmap_update_bits(regmap, FSTATUS_REG_OFFSET, FSTATUS_ERASE_MASK, FSTATUS_ERASE_MASK);
	if (r)
		return r;

	// FIXME: wait for interrupt?
	do {
		r = regmap_read(regmap, FSTATUS_REG_OFFSET, &val);
		if (r)
			return r;
	} while ((val & FSTATUS_BUSY_MASK) == FSTATUS_BUSY_MASK);

	return 0;
}

static int flash_read(struct mtd_info* mtd, loff_t from, size_t len,
						size_t* retlen, u_char* buf)
{
	struct regmap *regmap = mtd->priv;
	int r = 0;
	size_t transfer = 0;
	const size_t read_size = 8;

	dev_err(regmap_get_device(regmap), "read: %lld: %zu\n", from, len);
	r = flash_busy(regmap);
	if (r)
		return r;

	r = regmap_write(regmap, FPTR_REG_OFFSET, from);
	if (r)
		return r;

	*retlen = 0;
	while (*retlen < len) {
		transfer = len - *retlen;
		if (transfer > read_size)
			transfer = read_size;

		r = regmap_raw_read(regmap, FREAD_REG_OFFSET,  buf + *retlen, transfer);
		if (r) {
			dev_err(regmap_get_device(regmap), "read error: %d: %u\n", r, *retlen);
			return r;
		}
		*retlen += transfer;
	}

	return 0;
}

static int flash_write(struct mtd_info *mtd, loff_t to, size_t len,
						size_t* retlen, const u_char* buf)
{
	struct regmap *regmap = mtd->priv;
	int r = 0;
	size_t transfer = 0;
	const size_t max_write_size = 8;

	r = flash_busy(regmap);
	if (r)
		return r;

	r = regmap_write(regmap, FPTR_REG_OFFSET, to);
	if (r)
		return r;

	*retlen = 0;
	while (*retlen < len) {
		transfer = len - *retlen;
		if (transfer > max_write_size)
			transfer = max_write_size;

		r = regmap_raw_write(regmap, FWRITE_REG_OFFSET,  buf + *retlen, transfer);
		if (r)
			return r;
		*retlen += transfer;
	}

	return 0;
}
static int flash_register(struct regmap* regmap, struct mtd_info* mtd)
{
	int r = 0;
	u32 val = 0;

	mtd->type = MTD_NORFLASH;
	mtd->flags = MTD_CAP_NORFLASH;

	r = regmap_read(regmap, FSIZE_REG_OFFSET, &val);
	if (r < 0)
		return r;
	mtd->size = val;
	mtd->erasesize = mtd->size;
	mtd->writesize = 1;
	mtd->writebufsize = 1;
	mtd->name = "vmcu";
	mtd->priv = regmap;
	mtd->_erase = flash_erase;
	mtd->_read = flash_read;
	mtd->_write = flash_write;

	r = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (r < 0) {
		dev_err(regmap_get_device(regmap), "failed mtd register: %d\n", r);
		return r;
	}

	dev_err(regmap_get_device(regmap), "fffsize: %llu\n", mtd->size);

	return 0;
}

static int probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct vmcu *vmcu = NULL;
	u32 val = 0;
	int r = 0;

	vmcu = devm_kzalloc(&client->dev, sizeof(struct vmcu), GFP_KERNEL);
	if (!vmcu) {
		return -ENOMEM;
	}

	vmcu->client = client;
	vmcu->regmap = devm_regmap_init_i2c(client, &vmcu_regmap_config);
	if (IS_ERR(vmcu->regmap))
		return -EINVAL;
	i2c_set_clientdata(client, vmcu);
	mutex_init(&vmcu->mtx);

	// check magic number
	r = regmap_read(vmcu->regmap, MAGIC_REG_OFFSET, &val);
	if (r < 0)
		return r;
	if (val != MAGIC_REG_VALUE) {
		dev_err(&client->dev, "Wrong magic number 0x%x\n", val);
		return -EINVAL;
	}

	// check version
	r = regmap_read(vmcu->regmap, VERSION_REG_OFFSET, &val);
	if (r < 0)
		return r;
	dev_info(&client->dev, "Version: %u.%u.%u\n", val >> 16 & 0xff, val >> 8 & 0xff, val & 0xff);

	// rtc
	vmcu->rtc = devm_rtc_device_register(&client->dev, "vmcu", &vmcu_rtc_ops, THIS_MODULE);
	if (IS_ERR(vmcu->rtc))
		return PTR_ERR(vmcu->rtc);

	r = flash_register(vmcu->regmap, &vmcu->mtd);
	if (r < 0)
		return r;
	// Get flash size

	return 0;
}

static int remove(struct i2c_client* client)
{
	struct vmcu *vmcu = dev_get_drvdata(&client->dev);
	int r = 0;

	r = mtd_device_unregister(&vmcu->mtd);
	if (r < 0)
		return r;

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
	},
	.id_table = vmcu_id,
	.probe = probe,
	.remove = remove,
};
module_i2c_driver(vmcu_driver);

MODULE_AUTHOR("Mikko Salomäki <ms@datarespons.se>");
MODULE_DESCRIPTION("Vehicle MCU driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
