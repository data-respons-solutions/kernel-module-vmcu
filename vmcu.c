/*
 * Copyright (c) 2021  Data Respons Solutions AB
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
#include <linux/gpio/driver.h>
#include <linux/firmware.h>

#define MAGIC_REG				0x0
#define MAGIC_VALUE				0x0ec70add
#define VERSION_REG				0x1
#define VERSION_PATCH_MASK		GENMASK(7, 0)
#define VERSION_PATCH_SHIFT		0
#define VERSION_MINOR_MASK		GENMASK(15, 8)
#define VERSION_MINOR_SHIFT		8
#define VERSION_MAJOR_MASK		GENMASK(23, 16)
#define VERSION_MAJOR_SHIFT		16
#define RTC_TIME_REG			0x2
#define RTC_TIME_HOUR_MASK		GENMASK(7, 0)
#define RTC_TIME_HOUR_SHIFT		0
#define RTC_TIME_MIN_MASK		GENMASK(15, 8)
#define RTC_TIME_MIN_SHIFT		8
#define RTC_TIME_SEC_MASK		GENMASK(23, 16)
#define RTC_TIME_SEC_SHIFT		16
#define RTC_DATE_REG			0x3
#define RTC_DATE_YEAR_MASK		GENMASK(7, 0)
#define RTC_DATE_YEAR_SHIFT		0
#define RTC_DATE_MON_MASK		GENMASK(15, 8)
#define RTC_DATE_MON_SHIFT		8
#define RTC_DATE_DAY_MASK		GENMASK(23, 16)
#define RTC_DATE_DAY_SHIFT		16
#define RTC_DATE_WDAY_MASK		GENMASK(31, 24)
#define RTC_DATE_WDAY_SHIFT		24
#define LED0_REG				0x4
#define LED0_GREEN_MASK			BIT(0)
#define LED0_RED_MASK			BIT(1)
#define LED0_BLINK_MASK			BIT(2)
#define SENSOR_REG				0x6
#define SENSOR_ITEMP_MASK		GENMASK(10, 0)
#define SENSOR_ITEMP_SIGN_MASK	BIT(11)
#define ADC_VALUE_MASK			GENMASK(11, 0)
#define ADC_VBAT_REG			0x11
#define ADC0_REG				0x12
#define ADC1_REG				0x13
#define ADC2_REG				0x14
#define ADC3_REG				0x15
#define ADC4_REG				0x16
#define STATUS_REG				0x20
#define STATUS_IGNITION1_MASK	BIT(0)
#define STATUS_IGNITION2_MASK	BIT(0)
/*
 * WAKECTRL0 -> not volatile
 * WAKECTRL1 -> not volatile
 */
#define GPIO0_REG				0x30
#define GPIO0_RESET0_MASK		BIT(0)
#define GPIO0_RESET1_MASK		BIT(1)
#define GPIO0_RESET2_MASK		BIT(2)
#define GPIO0_RESET3_MASK		BIT(3)
#define GPIO0_SET0_MASK			BIT(8)
#define GPIO0_SET1_MASK			BIT(9)
#define GPIO0_SET2_MASK			BIT(10)
#define GPIO0_SET3_MASK			BIT(11)
#define GPIO0_STATE0_MASK		BIT(16)
#define GPIO0_STATE1_MASK		BIT(17)
#define GPIO0_STATE2_MASK		BIT(18)
#define GPIO0_STATE3_MASK		BIT(19)
#define GPIO0_STATESETE_MASK	BIT(31)
/*
 * GPIOCTRL0 -> not volatile
 */
#define APPCTRL_REG				0x70
#define APPCTRL_PARTITION_MASK	BIT(0)
#define APPCTRL_A_VALID_MASK	BIT(1)
#define APPCTRL_A_BLANK_MASK	BIT(2)
#define APPCTRL_B_VALID_MASK	BIT(3)
#define APPCTRL_B_BLANK_MASK	BIT(4)
#define APPCTRL_ERASE_MASK		BIT(5)
#define APPCTRL_SWAP_MASK		BIT(6)
#define APPCTRL_SIZE_MASK		GENMASK(15, 8)
#define APPCTRL_SIZE_SHIFT		8
#define APPCTRL_UNIT_MASK		GENMASK(23, 16)
#define APPCTRL_UNIT_SHIFT		16
#define APPCTRL_ALIGNMENT_MASK	GENMASK(31, 24)
#define APPCTRL_ALIGNMENT_SHIFT	24
#define APPWRITE_REG			0x75
#define APPREAD_REG				0x76

static const struct regmap_range volatile_ranges[] = {
	regmap_reg_range(RTC_TIME_REG, RTC_DATE_REG),
	regmap_reg_range(SENSOR_REG, SENSOR_REG),
	regmap_reg_range(ADC_VBAT_REG, ADC4_REG),
	regmap_reg_range(STATUS_REG, STATUS_REG),
	regmap_reg_range(GPIO0_REG, GPIO0_REG),
	regmap_reg_range(APPCTRL_REG, APPCTRL_REG),
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

enum vmcu_fw_part {
	PART_A,
	PART_B,
};

struct vmcu_fw {
	enum vmcu_fw_part part;
	u32 app_size;
	u32 write_unit;
	u32 write_align;
	u32 file_offset;
	u32 file_size;
};

struct vmcu {
	struct mutex			mtx;
	struct regmap			*regmap;
	struct i2c_client		*client;
	struct rtc_device 		*rtc;
	struct led_classdev_mc	led0;
	struct mc_subled 		led0_subled[LED0_NUM_COLORS];
	struct gpio_chip		gpio;
	struct fw_upload		*fw_upload;
	struct vmcu_fw			fw;
};

static int rtc_set(struct device *dev, struct rtc_time *rtctime)
{
	struct vmcu *vmcu = dev_get_drvdata(dev);
	int r = 0;

	const uint32_t date =
			(((rtctime->tm_year - 100) << RTC_DATE_YEAR_SHIFT) & RTC_DATE_YEAR_MASK)
			| ((rtctime->tm_mon  << RTC_DATE_MON_SHIFT) & RTC_DATE_MON_MASK)
			| ((rtctime->tm_mday << RTC_DATE_DAY_SHIFT) & RTC_DATE_DAY_MASK)
			| ((rtctime->tm_wday  << RTC_DATE_WDAY_SHIFT) & RTC_DATE_WDAY_MASK);

	const uint32_t time =
			((rtctime->tm_hour << RTC_TIME_HOUR_SHIFT) & RTC_TIME_HOUR_MASK)
			| ((rtctime->tm_min << RTC_TIME_MIN_SHIFT) & RTC_TIME_MIN_MASK)
			| ((rtctime->tm_sec << RTC_TIME_SEC_SHIFT) & RTC_TIME_SEC_MASK);

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;

	r = regmap_write(vmcu->regmap, RTC_DATE_REG, date);
	if (r == 0)
		r = regmap_write(vmcu->regmap, RTC_TIME_REG, time);

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

	r = regmap_read(vmcu->regmap, RTC_TIME_REG, &time);
	if (r == 0)
		r = regmap_read(vmcu->regmap, RTC_DATE_REG, &date);

	mutex_unlock(&vmcu->mtx);

	if (r < 0)
		return r;

	rtctime->tm_sec = (time & RTC_TIME_SEC_MASK) >> RTC_TIME_SEC_SHIFT;
	rtctime->tm_min = (time & RTC_TIME_MIN_MASK) >> RTC_TIME_MIN_SHIFT;
	rtctime->tm_hour = (time & RTC_TIME_HOUR_MASK) >> RTC_TIME_HOUR_SHIFT;
	rtctime->tm_mon = (date & RTC_DATE_MON_MASK) >> RTC_DATE_MON_SHIFT;
	rtctime->tm_mday = (date & RTC_DATE_DAY_MASK) >> RTC_DATE_DAY_SHIFT;
	rtctime->tm_year = ((date & RTC_DATE_YEAR_MASK) >> RTC_DATE_YEAR_SHIFT) + 100;
	rtctime->tm_wday = (date & RTC_DATE_WDAY_MASK) >> RTC_DATE_WDAY_SHIFT;
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
	r = regmap_update_bits(vmcu->regmap, LED0_REG, LED0_BLINK_MASK, LED0_BLINK_MASK);
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
	uint32_t mask = LED0_GREEN_MASK | LED0_RED_MASK;
	uint32_t val = 0;
	int r = 0;

	led_mc_calc_color_components(led0, brightness);

	if (led0->subled_info[0].brightness == 1)
		val |= LED0_RED_MASK;
	if (led0->subled_info[1].brightness == 1)
		val |= LED0_GREEN_MASK;
	if (val == 0)
		mask |= LED0_BLINK_MASK;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;
	r = regmap_update_bits(vmcu->regmap, LED0_REG, mask, val);
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

	r = regmap_read(vmcu->regmap, LED0_REG, &val);
	if (r) {
		dev_err(&vmcu->client->dev, "Failed reading led0 state\n");
		return r;
	}

	vmcu->led0_subled[0].color_index = LED_COLOR_ID_RED;
	vmcu->led0_subled[0].channel = 0;
	vmcu->led0_subled[0].brightness = (val & LED0_RED_MASK) == LED0_RED_MASK ? 1 : 0;
	vmcu->led0_subled[1].color_index = LED_COLOR_ID_GREEN;
	vmcu->led0_subled[1].channel = 1;
	vmcu->led0_subled[1].brightness = (val & LED0_GREEN_MASK) == LED0_GREEN_MASK ? 1 : 0;
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
		*val = data & ADC_VALUE_MASK;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		r = mutex_lock_interruptible(&vmcu->mtx);
		if (r)
			return r;
		r = regmap_read(vmcu->regmap, chan->address, &data);
		mutex_unlock(&vmcu->mtx);
		if (r)
			return r;
		if ((data & SENSOR_ITEMP_SIGN_MASK) == SENSOR_ITEMP_SIGN_MASK)
			sign_bit = 1;
		data &= SENSOR_ITEMP_MASK;
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
	{
		.channel = 0,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC0_REG,
		.extend_name = "adc0",
	},
	{
		.channel = 1,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC1_REG,
		.extend_name = "adc1",
	},
	{
		.channel = 2,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC2_REG,
		.extend_name = "adc2",
	},
	{
		.channel = 3,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC3_REG,
		.extend_name = "adc3",
	},
	{
		.channel = 4,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC4_REG,
		.extend_name = "adc4",
	},
	{
		.channel = 5,
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.address = ADC_VBAT_REG,
		.extend_name = "vbat",
	},
	{
		.channel = 6,
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.address = SENSOR_REG,
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

static int vmcu_gpio_get_direction(struct gpio_chip* chip, unsigned int offset)
{
	switch (offset) {
	case 0:
	case 1:
		return GPIO_LINE_DIRECTION_IN;
	case 2:
	case 3:
	case 4:
	case 5:
		return GPIO_LINE_DIRECTION_OUT;
	default:
		return -ENODEV;
	}
}

static int vmcu_gpio_get_multiple(struct gpio_chip* chip, unsigned long* mask, unsigned long* bits)
{
	struct vmcu *vmcu = gpiochip_get_data(chip);
	int r = 0;
	uint32_t data = 0;
	const uint32_t status_mask = GENMASK(1, 0);
	const uint32_t gpio_mask = GENMASK(5, 2);

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return r;

	if ((*mask & status_mask) != 0) {
		r = regmap_read(vmcu->regmap, STATUS_REG, &data);
		if (r)
			goto exit;
		if ((data & STATUS_IGNITION1_MASK) == STATUS_IGNITION1_MASK)
			*bits |= BIT(0);
		if ((data & STATUS_IGNITION1_MASK) == STATUS_IGNITION2_MASK)
			*bits |= BIT(1);
	}

	if ((*mask & gpio_mask) != 0) {
		r = regmap_read(vmcu->regmap, GPIO0_REG, &data);
		if (r)
			goto exit;
		if ((data & GPIO0_STATE0_MASK) == GPIO0_STATE0_MASK)
			*bits |= BIT(2);
		if ((data & GPIO0_STATE1_MASK) == GPIO0_STATE1_MASK)
			*bits |= BIT(3);
		if ((data & GPIO0_STATE2_MASK) == GPIO0_STATE2_MASK)
			*bits |= BIT(4);
		if ((data & GPIO0_STATE3_MASK) == GPIO0_STATE3_MASK)
			*bits |= BIT(5);
	}

exit:
	mutex_unlock(&vmcu->mtx);

	return r;
}

static int vmcu_gpio_get(struct gpio_chip* chip, unsigned int offset)
{
	int r = 0;
	unsigned long mask = 1 << offset;
	unsigned long bits = 0;

	r = vmcu_gpio_get_multiple(chip, &mask, &bits);
	if (!r)
		r = bits ? 1 : 0;
	return r;
}

static void vmcu_gpio_set_multiple(struct gpio_chip* chip, unsigned long* mask, unsigned long* bits)
{
	struct vmcu *vmcu = gpiochip_get_data(chip);
	int r = 0;
	uint32_t data = 0;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return;

	if ((*mask & BIT(2)) == BIT(2))
		data |=	(*bits & BIT(2)) == BIT(2) ? GPIO0_SET0_MASK : GPIO0_RESET0_MASK;
	if ((*mask & BIT(3)) == BIT(3))
		data |=	(*bits & BIT(3)) == BIT(3) ? GPIO0_SET1_MASK : GPIO0_RESET1_MASK;
	if ((*mask & BIT(4)) == BIT(4))
		data |=	(*bits & BIT(4)) == BIT(4) ? GPIO0_SET2_MASK : GPIO0_RESET2_MASK;
	if ((*mask & BIT(5)) == BIT(5))
		data |=	(*bits & BIT(5)) == BIT(5) ? GPIO0_SET3_MASK : GPIO0_RESET3_MASK;

	if (data != 0)
		regmap_write(vmcu->regmap, GPIO0_REG, data);

	mutex_unlock(&vmcu->mtx);
}

static void vmcu_gpio_set(struct gpio_chip* chip, unsigned int offset, int value)
{
	unsigned long mask = 1 << offset;
	unsigned long bits = value ? mask : 0;
	vmcu_gpio_set_multiple(chip, &mask, &bits);
}

static int gpio_register(struct vmcu *vmcu)
{
	int r = 0;

	vmcu->gpio.label = "vmcu";
	vmcu->gpio.parent = &vmcu->client->dev;
	vmcu->gpio.owner = THIS_MODULE;
	vmcu->gpio.get_direction = vmcu_gpio_get_direction;
	vmcu->gpio.get_multiple = vmcu_gpio_get_multiple;
	vmcu->gpio.get = vmcu_gpio_get;
	vmcu->gpio.set_multiple = vmcu_gpio_set_multiple;
	vmcu->gpio.set = vmcu_gpio_set;
	vmcu->gpio.ngpio = 6;
	vmcu->gpio.base = -1;
	vmcu->gpio.can_sleep = true;

	r = devm_gpiochip_add_data(&vmcu->client->dev, &vmcu->gpio, vmcu);
	if (r < 0)
		dev_err(&vmcu->client->dev, "Failed registering gpio\n");
	return r;
}

/*
 * Firmware update
 *
 * The MCU supports application updates by an A/B partition scheme.
 * Relocation is not supported and the provided binaries
 * must be compiled for either side A or B.
 * Provided firmware image is expected to be a concatenated blob
 * of A-header, A-binary, B-header, B-binary.
 *
 * Application partitions are 28KiB in size and consist of 14 pages
 * of 2KiB each. Erase time per page is max 40ms which leads to an
 * erase time of 14 * 40 = 560ms.
 * Code runs from flash and during erase any operations reading from
 * flash are blocking.
 * We simply wait for 1000ms after an erase operation is started.
 *
 * Flashing procedure:
 *  - Check A or B partition (current = running partition, next = update target)
 *  - Find corresponding hdr/binary from firmware blob
 *  - Erase next if not blank
 *  - Write hdr/binary to next
 *  - Check hdr/binary in next validated by MCU
 *  - Swap to next, invalidating current
 */

static enum fw_upload_err vmcu_fw_prepare(struct fw_upload* fw_upload, const u8* data, u32 size)
{
	struct vmcu *vmcu = fw_upload->dd_handle;
	uint32_t val = 0;
	int blank = 0;
	int r = 0;

	(void) data;

	/* Input file should consist of partition A and B of equal size */
	if (size % 2 != 0)
		return FW_UPLOAD_ERR_INVALID_SIZE;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return FW_UPLOAD_ERR_HW_ERROR;

	r = regmap_read(vmcu->regmap, APPCTRL_REG, &val);
	if (r) {
		dev_err(&vmcu->client->dev, "Failed reading appctrl [0x%x]: %d\n", APPCTRL_REG, r);
		r = FW_UPLOAD_ERR_RW_ERROR;
		goto exit;
	}

	dev_info(&vmcu->client->dev, "val: 0x%x\n", val);

	vmcu->fw.write_unit = (val & APPCTRL_UNIT_MASK) >> APPCTRL_UNIT_SHIFT;
	vmcu->fw.app_size = ((val & APPCTRL_SIZE_MASK) >> APPCTRL_SIZE_SHIFT) * 1024;
	vmcu->fw.write_align = (val & APPCTRL_ALIGNMENT_MASK) >> APPCTRL_ALIGNMENT_SHIFT;
	dev_info(&vmcu->client->dev, "App area size: %u: unit: %u: align: %u\n",
			vmcu->fw.app_size, vmcu->fw.write_unit, vmcu->fw.write_align);

	vmcu->fw.file_size = size / 2;
	vmcu->fw.file_offset = vmcu->fw.part == PART_A ? 0 : vmcu->fw.file_size;
	dev_info(&vmcu->client->dev, "File size: %u: offset: %u\n", vmcu->fw.file_size, vmcu->fw.file_offset);
	if (vmcu->fw.file_size > vmcu->fw.app_size) {
		r = FW_UPLOAD_ERR_INVALID_SIZE;
		goto exit;
	}

	vmcu->fw.part = PART_A;
	if ((val & APPCTRL_PARTITION_MASK) == APPCTRL_PARTITION_MASK)
		vmcu->fw.part = PART_B;
	dev_info(&vmcu->client->dev, "Current partition: %s\n", vmcu->fw.part == PART_A ? "A" : "B");

	if ((vmcu->fw.part == PART_A && (val & APPCTRL_B_BLANK_MASK) == APPCTRL_B_BLANK_MASK)
		|| (vmcu->fw.part == PART_B && (val & APPCTRL_A_BLANK_MASK) == APPCTRL_A_BLANK_MASK))
		blank = 1;

	if (blank) {
		dev_info(&vmcu->client->dev, "next partition blank\n");
	}
	else {
		dev_info(&vmcu->client->dev, "erasing next\n");
		r = regmap_write_bits(vmcu->regmap, APPCTRL_REG, APPCTRL_ERASE_MASK, APPCTRL_ERASE_MASK);
		if (r) {
			dev_err(&vmcu->client->dev, "Failed writing appctrl [0x%x]: %d\n", APPCTRL_REG, r);
			r = FW_UPLOAD_ERR_RW_ERROR;
			goto exit;
		}
		/* Should be erased well within 10000ms */
		for (int i = 0; i < 20; ++i) {
			r = regmap_read(vmcu->regmap, APPCTRL_REG, &val);
			/* Ignore errors as MCU unresponsive during update */
			if (r == 0) {
				if ((vmcu->fw.part == PART_A && (val & APPCTRL_B_BLANK_MASK) == APPCTRL_B_BLANK_MASK)
					|| (vmcu->fw.part == PART_B && (val & APPCTRL_A_BLANK_MASK) == APPCTRL_A_BLANK_MASK)) {
					blank = 1;
					break;
				}
			}
			msleep(500);
		}
		if (!blank) {
			dev_err(&vmcu->client->dev, "Failed erasing -- timeout\n");
			r = FW_UPLOAD_ERR_TIMEOUT;
			goto exit;
		}
	}

	r = FW_UPLOAD_ERR_NONE;

exit:
	mutex_unlock(&vmcu->mtx);
	return r;
}

static enum fw_upload_err vmcu_fw_write(struct fw_upload* fw_upload, const u8* data, u32 offset, u32 size, u32* written)
{
	struct vmcu *vmcu = fw_upload->dd_handle;
	u8 buf[36];
	const u32 bytes = size < 32 ? size : 32;
	int r = 0;

	/* We only use half of the provided firmware file */
	if (offset >= vmcu->fw.file_size) {
		*written = vmcu->fw.file_size;
		return FW_UPLOAD_ERR_NONE;
	}

	/* Prepare buffer */
	buf[0] = offset & 0xff;
	buf[1] = (offset >> 8) & 0xff;
	buf[2] = bytes & 0xff;
	buf[3] = (bytes >> 8) & 0xff;
	memcpy(buf, data + offset + vmcu->fw.file_offset, bytes);

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return FW_UPLOAD_ERR_HW_ERROR;

	/* Write */
	r = i2c_master_send(vmcu->client, buf, bytes + 4);
	mutex_unlock(&vmcu->mtx);
	if (r == bytes + 4) {
		r = FW_UPLOAD_ERR_NONE;
		*written = bytes;
	}
	else {
		r = FW_UPLOAD_ERR_RW_ERROR;
		dev_err(&vmcu->client->dev, "failed writing %u bytes to offset %u [%d]\n", bytes, offset, r);
	}

	return r;
}

static enum fw_upload_err vmcu_fw_poll_complete(struct fw_upload* fw_upload)
{
	struct vmcu *vmcu = fw_upload->dd_handle;
	u32 val = 0;
	int r = 0;

	r = mutex_lock_interruptible(&vmcu->mtx);
	if (r)
		return FW_UPLOAD_ERR_HW_ERROR;

	r = regmap_read(vmcu->regmap, APPCTRL_REG, &val);
	if (r) {
		dev_err(&vmcu->client->dev, "Failed reading appctrl [0x%x]: %d\n", APPCTRL_REG, r);
		r = FW_UPLOAD_ERR_RW_ERROR;
		goto exit;
	}

	if ((vmcu->fw.part == PART_A && (val & APPCTRL_B_VALID_MASK) == APPCTRL_B_VALID_MASK)
		|| (vmcu->fw.part == PART_B && (val & APPCTRL_A_VALID_MASK) == APPCTRL_A_VALID_MASK)) {
		dev_info(&vmcu->client->dev, "Next partition valid\n");
	}
	else {
		dev_err(&vmcu->client->dev, "Next partition invalid");
		r = FW_UPLOAD_ERR_HW_ERROR;
		goto exit;
	}

	r = regmap_write_bits(vmcu->regmap, APPCTRL_REG, APPCTRL_SWAP_MASK, APPCTRL_SWAP_MASK);
	if (r) {
		dev_err(&vmcu->client->dev, "Failed writing appctrl [0x%x]: %d\n", APPCTRL_REG, r);
		r = FW_UPLOAD_ERR_RW_ERROR;
		goto exit;
	}

	r = FW_UPLOAD_ERR_NONE;
exit:
	mutex_unlock(&vmcu->mtx);
	return r;
}

void vmcu_fw_cancel(struct fw_upload* fw_upload)
{
	struct vmcu *vmcu = fw_upload->dd_handle;
	dev_err(&vmcu->client->dev, "cancel\n");
}

void vmcu_fw_cleanup(struct fw_upload* fw_upload)
{
	struct vmcu *vmcu = fw_upload->dd_handle;
	dev_err(&vmcu->client->dev, "cleanup\n");
}

static const struct fw_upload_ops vmcu_fw_ops = {
        .prepare = vmcu_fw_prepare,
        .write = vmcu_fw_write,
        .poll_complete = vmcu_fw_poll_complete,
        .cancel = vmcu_fw_cancel,
        .cleanup = vmcu_fw_cleanup,
};

static int firmware_register(struct vmcu* vmcu)
{
	struct fw_upload *fwl = NULL;

	fwl = firmware_upload_register(THIS_MODULE, &vmcu->client->dev,
					"vmcu", &vmcu_fw_ops, vmcu);
	if (IS_ERR(fwl)) {
		dev_err(&vmcu->client->dev, "Failed registering firmware upload [%ld]\n", PTR_ERR(fwl));
		return PTR_ERR(fwl);
	}

	vmcu->fw_upload = fwl;

	return 0;
}

static int vmcu_probe(struct i2c_client* client, const struct i2c_device_id* id)
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
	r = regmap_read(vmcu->regmap, MAGIC_REG, &val);
	if (r < 0) {
		dev_err(&client->dev, "failed reg [%x] read: %d\n", MAGIC_REG, r);
		return r;
	}
	if (val != MAGIC_VALUE) {
		dev_err(&client->dev, "Wrong magic number 0x%x\n", val);
		return -EINVAL;
	}

	// check version
	r = regmap_read(vmcu->regmap, VERSION_REG, &val);
	if (r < 0)
		return r;
	dev_info(&client->dev, "Version: %u.%u.%u\n",
			(u32) (val & VERSION_MAJOR_MASK) >> VERSION_MAJOR_SHIFT,
			(u32) (val & VERSION_MINOR_MASK) >> VERSION_MINOR_SHIFT,
			(u32) (val & VERSION_PATCH_MASK) >> VERSION_PATCH_SHIFT);

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

	// gpio
	r = gpio_register(vmcu);
	if (r < 0)
		return r;

	// firmware
	r = firmware_register(vmcu);
	if (r < 0)
		return r;

	return 0;
}

static int vmcu_remove(struct i2c_client* client)
{
	struct vmcu *vmcu = i2c_get_clientdata(client);

	if (vmcu->fw_upload)
		firmware_upload_unregister(vmcu->fw_upload);

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
	.probe = vmcu_probe,
	.remove = vmcu_remove,
};
module_i2c_driver(vmcu_driver);

MODULE_AUTHOR("Mikko Salomäki <ms@datarespons.se>");
MODULE_DESCRIPTION("Vehicle MCU driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
