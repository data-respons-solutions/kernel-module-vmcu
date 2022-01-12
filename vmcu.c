/*
 * Copyright (c) 2021  Data Respons Solutions
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
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/led-class-multicolor.h>
#include <linux/iio/iio.h>

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
#define REG_LED0				0x4
#define REG_LED0_GREEN			BIT(0)
#define REG_LED0_RED			BIT(1)
#define REG_LED0_BLINK			BIT(2)
#define REG_SENSOR				0x6
#define REG_SENSOR_ITEMP		GENMASK(10, 0)
#define REG_SENSOR_ITEMP_SIGN	BIT(11)
#define REG_ADC_MASK			GENMASK(11, 0)
#define REG_ADC_VBAT			0x11
#define REG_ADC0				0x12

static const struct regmap_range volatile_ranges[] = {
	regmap_reg_range(RTC_TIME_REG_OFFSET, RTC_DATE_REG_OFFSET),
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
	.disable_locking = 1,
};

#define LED0_NUM_COLORS	2
#define ADC_REF_mV 		3300
#define ADC_BITS 		4096

struct vmcu {
	struct mutex			mtx;
	struct regmap			*regmap;
	struct i2c_client		*client;
	struct rtc_device 		*rtc;
	struct led_classdev_mc	led0;
	struct mc_subled 		led0_subled[LED0_NUM_COLORS];
};

static int rtc_set(struct device *dev, struct rtc_time *rtctime)
{
	struct vmcu *vmcu = dev_get_drvdata(dev);
	int r = 0;

	const uint32_t date = bin2bcd(rtctime->tm_year - 100) << RTC_DATE_YEAR_SHIFT
			| bin2bcd(rtctime->tm_mon + 1) << RTC_DATE_MONTH_SHIFT
			| bin2bcd(rtctime->tm_mday) << RTC_DATE_DAY_SHIFT
			| bin2bcd(rtctime->tm_wday + 1) << RTC_DATE_WDAY_SHIFT;

	const uint32_t time = bin2bcd(rtctime->tm_hour) << RTC_TIME_HOUR_SHIFT
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
	uint32_t time;
	uint32_t date;
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

static int led0_blink(struct led_classdev *cdev, unsigned long *delay_on, unsigned long *delay_off)
{
	struct vmcu* vmcu = dev_get_drvdata(cdev->dev->parent);
	int r = 0;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;
	r = regmap_update_bits(vmcu->regmap, REG_LED0, REG_LED0_BLINK, REG_LED0_BLINK);
	mutex_unlock(&vmcu->mtx);
	if (r)
		return r;

	*delay_on = 1000;
	*delay_off = 1000;

	return 0;
}

static int led0_set_blocking(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct led_classdev_mc *led0 = lcdev_to_mccdev(cdev);
	struct vmcu* vmcu = dev_get_drvdata(cdev->dev->parent);
	uint32_t mask = REG_LED0_GREEN | REG_LED0_RED;
	uint32_t val = 0;
	int r = 0;

	led_mc_calc_color_components(led0, brightness);

	if (led0->subled_info[0].brightness == 1)
		val |= REG_LED0_RED;
	if (led0->subled_info[1].brightness == 1)
		val |= REG_LED0_GREEN;
	if (val == 0)
		mask |= REG_LED0_BLINK;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;
	r = regmap_update_bits(vmcu->regmap, REG_LED0, mask, val);
	mutex_unlock(&vmcu->mtx);
	if (r)
		return r;

	return 0;
}

static int led0_register(struct vmcu* vmcu)
{
	struct led_init_data init_data = {};
	int r = 0;
	uint32_t val = 0;

	r = regmap_read(vmcu->regmap, REG_LED0, &val);
	if (r) {
		dev_err(&vmcu->client->dev, "Failed reading led0 state\n");
		return r;
	}

	vmcu->led0_subled[0].color_index = LED_COLOR_ID_RED;
	vmcu->led0_subled[0].channel = 0;
	vmcu->led0_subled[0].brightness = (val & REG_LED0_RED) == REG_LED0_RED ? 1 : 0;
	vmcu->led0_subled[1].color_index = LED_COLOR_ID_GREEN;
	vmcu->led0_subled[1].channel = 1;
	vmcu->led0_subled[1].brightness = (val & REG_LED0_GREEN) == REG_LED0_GREEN ? 1 : 0;
	vmcu->led0.subled_info = &vmcu->led0_subled[0];
	vmcu->led0.num_colors = LED0_NUM_COLORS;
	init_data.default_label = ":led0";
	init_data.devicename = "vmcu";
	vmcu->led0.led_cdev.max_brightness = 1;
	vmcu->led0.led_cdev.brightness_set_blocking = led0_set_blocking;
	vmcu->led0.led_cdev.blink_set = led0_blink;

	r = devm_led_classdev_multicolor_register_ext(&vmcu->client->dev, &vmcu->led0, &init_data);
	if (r < 0) {
		dev_err(&vmcu->client->dev, "Failed registering led0\n");
		return r;
	}

	return 0;
}

static int iio_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct vmcu *vmcu = dev_get_drvdata(indio_dev->dev.parent);
	uint32_t data = 0;
	int r = 0;
	int sign_bit = 0;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		r = mutex_lock_interruptible(&vmcu->mtx);
		if (r)
			return r;
		r = regmap_read(vmcu->regmap, chan->address, &data);
		mutex_unlock(&vmcu->mtx);
		if (r)
			return r;
		*val = data & REG_ADC_MASK;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		r = mutex_lock_interruptible(&vmcu->mtx);
		if (r)
			return r;
		r = regmap_read(vmcu->regmap, chan->address, &data);
		mutex_unlock(&vmcu->mtx);
		if (r)
			return r;
		if ((data & REG_SENSOR_ITEMP_SIGN) == REG_SENSOR_ITEMP_SIGN)
			sign_bit = 1;
		data &= REG_SENSOR_ITEMP;
		if (sign_bit)
			data = -data;
		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = ADC_REF_mV;
		*val2 = 4096;
		return IIO_VAL_FRACTIONAL;
	}
	return -EINVAL;
}

static const struct iio_chan_spec vmcu_iio_channels[] = {
	{ /* [0] */
		.channel = 0,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = REG_ADC0,
		.extend_name = "adc0",
	},
	{ /* [1] */
		.channel = 1,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = REG_ADC_VBAT,
		.extend_name = "vbat",
	},
	{ /* [2] */
		.channel = 2,
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = REG_SENSOR,
		.extend_name = "itemp",
	},
};

static const struct iio_info vmcu_iio_info = {
	.read_raw = iio_read_raw,
};

static int adc_register(struct vmcu *vmcu)
{
	struct iio_dev *indio_dev;
	int r = 0;

	indio_dev = devm_iio_device_alloc(&vmcu->client->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name = "vmcu";
	indio_dev->info = &vmcu_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = vmcu_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(vmcu_iio_channels);

	r = devm_iio_device_register(&vmcu->client->dev, indio_dev);
	if (r < 0) {
		dev_err(&vmcu->client->dev, "Failed registering adc\n");
		return r;
	}

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
	if (r < 0) {
		dev_err(&client->dev, "failed reg [%x] read: %d\n", MAGIC_REG_OFFSET, r);
		return r;
	}
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

	// led0
	r = led0_register(vmcu);
	if (r < 0)
		return r;

	// adc
	r = adc_register(vmcu);
	if (r < 0)
		return r;

	return 0;
}

static const struct of_device_id of_vmcu_match[] = { { .compatible =
	"drs,vmcu" }, { /* Sentinel */} };

static struct i2c_device_id vmcu_id[] = { { "vmcu", 0 }, { } };
MODULE_DEVICE_TABLE(i2c, vmcu_id);

static struct i2c_driver vmcu_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "vmcu",
		.of_match_table = of_vmcu_match,
	},
	.id_table = vmcu_id,
	.probe = probe,
};
module_i2c_driver(vmcu_driver);

MODULE_AUTHOR("Mikko Salomäki <ms@datarespons.se>");
MODULE_DESCRIPTION("Vehicle MCU driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
