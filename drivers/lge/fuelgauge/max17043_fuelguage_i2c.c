/*
 *  MAX17043_fuelgauge.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009, 2011, 2012 LG Electronics
 *  Dajin Kim <dajin.kim@lge.com>
 *
 *  porting by <jongho3.lee@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <mach/gpio.h>
#include <linux/slab.h>
#include <linux/charger_rt9524.h>
#include <linux/muic/muic.h>
#include <linux/fuel_gauge_max17043.h>

//#define D printk
static DEFINE_MUTEX(fuel_update_lock);

/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add gpio_alert */
struct max17043_chip {
	struct i2c_client *client;
	struct delayed_work gauge_work;
	struct delayed_work alert_work;
	struct power_supply battery;

	/* Max17043 Registers.(Raw Data) */
	int vcell;		// VCELL Register vaule
	int soc;		// SOC Register value
	int version;		// Max17043 Chip version
	int config;		// RCOMP, Sleep, ALRT, ATHD

	/* Interface with Android */
	int voltage;		// Battery Voltage   (Calculated from vcell)
	int capacity;		// Battery Capacity  (Calculated from soc)
	int fg_enable;		// Battery Capacity  (Calculated from soc)
	max17043_status status;	// State Of max17043
	int gpio_alert;
	int default_rcomp;
};

/*
 * Voltage Calibration Data
 *   voltage = (capacity * gradient) + intercept
 *   voltage must be in +-15%
 */
struct max17043_calibration_data {
	int voltage;		/* voltage in mA */
	int capacity;		/* capacity in % */
	int gradient;		/* gradient * 1000 */
	int intercept;		/* intercept * 1000 */
};

/* 180mA Load for Battery */
static struct max17043_calibration_data without_charger[] = {
	{3953, 81,  9, 3242},
	{3800, 58,  7, 3403},
	{3740, 40,  3, 3611},
	{3695, 20,  2, 3650},
	{3601,  4,  6, 3574},
	{3300,  0, 55, 3548},
	{  -1, -1, -1,   -1},	// End of Data
};

/* 770mA Charging Battery */
static struct max17043_calibration_data with_charger[] = {
	{3865,   2, 66, 3709},
	{3956,  19,  5, 3851},
	{4021,  46,  2, 3912},
	{4088,  61,  5, 3813},
	{4158,  71,  7, 3689},
	{4200, 100,  2, 4042},
	{-1,    -1, -1,   -1},	// End of Data
};

static struct max17043_chip *reference = NULL;

int need_to_quickstart = 0;
EXPORT_SYMBOL(need_to_quickstart);

#define SOC_TIMES 				1000
#define FUEL_GAUGE_VOLT_ERROR_RANGE		53	// 43 * (250/200)

battery_graph_prop max17043_battery_soc_graph[] = {
	{4100, 100 * SOC_TIMES},
	{3893,  75 * SOC_TIMES},
	{3783,  57 * SOC_TIMES},
	{3721,  36 * SOC_TIMES},
	{3686,  20 * SOC_TIMES},
	{3575,   4 * SOC_TIMES},
	{3487,   2 * SOC_TIMES},
	{3300,   0 * SOC_TIMES},
};

static int max17043_reference_graph(int __x,
		battery_graph_prop * ref_battery_graph,
		int arraysize, int *error_range)
{
	int i = 1;
	int __y = 0;
	int slope, const_term;
	int delta_y, delta_x;

	D(" battery graph array size = %d", arraysize);

	while (__x < ref_battery_graph[i].x && i < (arraysize - 1))
		i++;

	delta_x = ref_battery_graph[i - 1].x - ref_battery_graph[i].x;
	delta_y = (ref_battery_graph[i - 1].y - ref_battery_graph[i].y);

	slope = delta_y / delta_x;

	const_term =
	    (ref_battery_graph[i].y) - (ref_battery_graph[i].x * slope);

	__y = (__x * slope + const_term);

	/* [jongho3.lee@lge.com] Soc error range should be allowd
	 * as much as fuel gauge volt error range.
	 */
	if (error_range) {
		*error_range =
		    slope * (FUEL_GAUGE_VOLT_ERROR_RANGE) / (*error_range);

		if (*error_range < 0) {
			*error_range *= -1;
		}

		if (*error_range < 5) {
			*error_range = 5;
		}
	}

	D(" ##### SLOPE = %d, CONST_TERM = %d ##########", slope, const_term);
	D(" ##### CALCULATED __y = %d ##########", __y);

	if (ref_battery_graph[i - 1].y > ref_battery_graph[i].y) {
		if (__y > ref_battery_graph[i - 1].y) {
			__y = ref_battery_graph[i - 1].y;
			D(" ##### fixing __y = %d ##########", __y);
		} else if (__y < ref_battery_graph[i].y) {
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	} else {
		if (__y < ref_battery_graph[i - 1].y) {
			__y = ref_battery_graph[i - 1].y;
			D(" ##### fixing __y = %d ##########", __y);
		} else if (__y > ref_battery_graph[i].y) {
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	}

	return __y;
}

static int max17043_validate_gauge_value(int voltage, int capacity)
{
	int calculated_soc, error_range = SOC_TIMES;

	calculated_soc =
	    max17043_reference_graph(voltage, max17043_battery_soc_graph,
				     ARRAY_SIZE(max17043_battery_soc_graph),
				     &error_range) / SOC_TIMES;

	D(" ##### SOC = %d,  CALCULATED SOC = %d, error_range = %d  ########",
	  capacity, calculated_soc, error_range);

	if ((capacity < calculated_soc + error_range)
	    && (capacity > calculated_soc - error_range)) {
		D(" ##### SOC & CALCULATED SOC is met  ##########");
		/* [jongho3.lee@lge.com] capacity validation should be
		 * performed just once.
		 */
		return 1;
	}

	D(" ##### SOC & CALCULATED SOC is NOT met 1!!!  ##########");
	/* [jongho3.lee@lge.com] capacity validation should be performed just once. */
	return 0;
}

int set_fg_enable(int en)
{
	if (!reference)
		return 0;

	reference->fg_enable = en;

	return 0;
}
EXPORT_SYMBOL(set_fg_enable);

int get_fg_enable(void)
{
	if (!reference)
		return 0;

	return reference->fg_enable;
}
EXPORT_SYMBOL(get_fg_enable);

static int max17043_write_reg(struct i2c_client *client, int reg, u16 value)
{
	int ret = 0;
	int retry = 1;

	mutex_lock(&fuel_update_lock);
	if (!get_fg_enable()) {
		D("MAX17043 CAMERAIS ON! DONt try to use I2C....., %d,",
		  !get_fg_enable());
		ret = -1;
	} else {
		DGAU("reg: 0x%x, value: 0x%04x", reg, value);
		/* FIXME: <jongho3.lee> */
		value = ((value & 0xFF00) >> 8) | ((value & 0xFF) << 8);

		while (retry--) {
			ret = i2c_smbus_write_word_data(client, reg, value);

			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__,
					ret);
				D("MAX17043 I2C WRITE ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				/* <jongho3.lee@lge.com> sleep before next try */
				msleep(1);
			} else
				break;
		}
	}
	mutex_unlock(&fuel_update_lock);

	return ret;
}

static int max17043_read_reg(struct i2c_client *client, int reg)
{
	int ret = 0;
	int retry = 1;

	mutex_lock(&fuel_update_lock);
	/* LGE_CHANGE [bk.shin@lge.com] 2012-02-01, LGE_P940, add from P940 GB */
	if (!get_fg_enable()) {
		D("MAX17043 CAMERAIS ON! DONt try to use I2C....., %d,",
		  !get_fg_enable());
		ret = -1;
	} else {
		while (retry--) {
			D("MAX17043 I2C READ REG@@@@@@@@@@@@@@");
			ret = i2c_smbus_read_word_data(client, reg);
			//max17043_write_reg(client, MAX17043_CONFIG_REG, reference->config);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__,
					ret);
				D("MAX17043 I2C READ ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
				//<jongho3.lee@lge.com> sleep before next try
				msleep(1);
			} else
				break;
		}
	}
	mutex_unlock(&fuel_update_lock);

	if (ret < 0)
		return ret;

#ifdef __DEBUG_FUELGAUGE
	u16 val = (((ret & 0xFF00) >> 8) | ((ret & 0xFF) << 8));
	u16 athd = (val & 0x1f);

	DGAU("reg:0x%x, value: 0x%04x", reg, val);
	if (reg == 0xc) {
		DGAU("ALRT bit: %d", ((val & 0x20) >> 5));
	}
#endif

	return ((ret & 0xFF00) >> 8) | ((ret & 0xFF) << 8);
}

static int max17043_reset(struct max17043_chip *chip)
{
	struct i2c_client *client = chip->client;

	if (!chip->fg_enable)
		return 0;

	max17043_write_reg(client, MAX17043_CMD_REG, 0x5400);

	chip->status = MAX17043_RESET;
	dev_info(&client->dev, "MAX17043 Fuel-Gauge Reset\n");
	return 0;
}

int max17043_quickstart(void)
{
	struct i2c_client *client = NULL;

	if (!reference)
		return 0;

	client = reference->client;

	if (!reference->fg_enable)
		return 0;

	max17043_write_reg(client, MAX17043_MODE_REG, 0x4000);

	reference->status = MAX17043_QUICKSTART;
	dev_info(&client->dev, "MAX17043 Fuel-Gauge Quick-Start\n");

	cancel_delayed_work(&reference->gauge_work);
	schedule_delayed_work(&reference->gauge_work, HZ / 4);

	return 0;
}
EXPORT_SYMBOL(max17043_quickstart);

static int max17043_read_vcell(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	D("!get_fg_enable() %d,", !get_fg_enable());
	if (!chip->fg_enable)
		return 0;

	value = max17043_read_reg(client, MAX17043_VCELL_REG);
	D("max17043_read_vcell = %d", value);

	if (value < 0)
		return value;

	chip->vcell = value >> 4;

	return 0;
}

static int max17043_read_soc(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	D("!get_fg_enable() %d,", !get_fg_enable());

	if (!chip->fg_enable)
		return 0;

	value = max17043_read_reg(client, MAX17043_SOC_REG);
	D("max17043_read_soc= %d", value);

	if (value < 0)
		return value;

	chip->soc = value;

	return 0;
}

static int max17043_read_version(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	if (!chip->fg_enable)
		return 0;

	value = max17043_read_reg(client, MAX17043_VER_REG);

	chip->version = value;

	dev_info(&client->dev, "MAX17043 Fuel-Gauge Ver %d\n", value);

	return 0;
}

static int max17043_read_config(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	u16 value;

	if (!chip->fg_enable)
		return 0;

	value = max17043_read_reg(client, MAX17043_CONFIG_REG);

	if (value < 0)
		return value;

	chip->config = value;

	return 0;
}

static int max17043_write_config(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	if (!chip->fg_enable)
		return 0;

	max17043_write_reg(client, MAX17043_CONFIG_REG, chip->config);

	return 0;
}

/* LGE_SJIT 2012-02-08 [dojip.kim@lge.com] fix the warning */
#if 0
static int max17043_need_quickstart(int charging)
{
	struct max17043_calibration_data *data;
	int i = 0;
	int expected;
	int diff;
	int vol;
	int level;

	if (reference == NULL)
		return 0;

	/* Get Current Data */
	vol = reference->voltage;
	level = reference->soc >> 8;
	if (level > 100)
		level = 100;
	else if (level < 0)
		level = 0;

	/* choose data to use */
	if (charging) {
		data = with_charger;
		while (data[i].voltage != -1) {
			if (vol <= data[i].voltage)
				break;
			i++;
		}
	} else {
		data = without_charger;
		while (data[i].voltage != -1) {
			if (vol >= data[i].voltage)
				break;
			i++;
		}
	}

	/* absense of data */
	if (data[i].voltage == -1) {
		if (charging) {
			if (level == 100)
				return 0;
			else
				return 1;
		} else {
			if (level == 0)
				return 0;
			else
				return 1;
		}
	}
	/* calculate diff */
	expected = (vol - data[i].intercept) / data[i].gradient;
	if (expected > 100)
		expected = 100;
	else if (expected < 0)
		expected = 0;
	diff = expected - level;

	dev_info(&reference->client->dev,
			"max17043 : expected=(%d-%d)/%d",
			vol, data[i].intercept, data[i].gradient);
	dev_info(&reference->client->dev,
			"max17043: diff:%d-%d=%d",
			expected, level, expected - level);
	/* judge */
	if (diff < -MAX17043_TOLERANCE || diff > MAX17043_TOLERANCE)
		need_to_quickstart += 1;
	else
		need_to_quickstart = 0;

	/* Maximum continuous reset time is 2.
	 * If reset over 2 times, discard it.
	 */
	if (need_to_quickstart > 2)
		need_to_quickstart = 0;

	return need_to_quickstart;
}
#endif

static int max17043_next_alert_level(int level)
{
	int next_level;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, add a definition of Battery Alert Threshold.
	 */
	if (level > ATHD_LEVEL)
		next_level = max17043_reverse_get_ui_capacity(ATHD_LEVEL);
	else if (level < 5)
		next_level = max17043_reverse_get_ui_capacity(level - 1);
	else
		next_level = max17043_reverse_get_ui_capacity(3);

	DGAU("level: %d, next_level: %d", level, next_level);
	return next_level;
}

static int max17043_set_rcomp(int rcomp)
{
	if (reference == NULL)
		return -ENODEV;

	dev_info(&reference->client->dev,
			"max17043: %s: new rcomp=0x%x\n", __func__, rcomp);
	rcomp &= 0xff;
	reference->config = ((reference->config & 0x00ff) | (rcomp << 8));

	max17043_write_config(reference->client);

	return 0;
}

int max17043_set_alert_level(int level)
{
	if (reference == NULL)
		return -1;

	if (level > 32)
		level = 32;
	else if (level < 1)
		level = 1;

	dev_info(&reference->client->dev,
			"max17043: %s: 32-%d=0x%x, Next_Alert_Level:%d(%%)\n",
			__func__, level, 32 - level, level);
	level = 32 - level;
	if (level == (reference->config & 0x1F))
		return level;

	reference->config = ((reference->config & 0xffe0) | level);
	max17043_write_config(reference->client);

	return level;
}
EXPORT_SYMBOL(max17043_set_alert_level);

static int max17043_clear_interrupt(struct max17043_chip *chip)
{
	DGAU("---------------");
	if (chip->config & 0x20) {
		chip->config &= 0xffdf;
		max17043_write_config(chip->client);
	}

	return 0;
}

static int max17043_update(struct max17043_chip *chip)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = max17043_read_soc(client);
	if (ret < 0)
		return ret;

	/* <jongho3.lee@lge.com> sleep before next try */
	ret = max17043_read_vcell(client);
	if (ret < 0)
		return ret;

	/* convert raw data to usable data */
	/* vcell * 1.25 mV */
	chip->voltage = (chip->vcell * 5) >> 2;
	chip->capacity = chip->soc >> 8;

	if (chip->capacity > 100)
		chip->capacity = 100;
	else if (chip->capacity < 0)
		chip->capacity = 0;

	chip->status = MAX17043_WORKING;

	return 0;
}

static void max17043_update_work(struct work_struct *work)
{
	struct max17043_chip *chip =
		container_of(work, struct max17043_chip, gauge_work.work);

	max17043_update(chip);
	schedule_delayed_work(&chip->gauge_work,
				      MAX17043_WORK_DELAY);
}

int max17043_update_by_other(void)
{
	if (reference == NULL)
		return -1;

	/* FIXME  --> work */
	schedule_delayed_work(&reference->gauge_work, 0);
	return 0;
}
EXPORT_SYMBOL(max17043_update_by_other);

static void max17043_alert_work(struct work_struct *work)
{
	struct max17043_chip *chip =
		container_of(work, struct max17043_chip, alert_work.work);

	max17043_update(chip);
	max17043_clear_interrupt(chip);
}

static irqreturn_t max17043_interrupt_handler(int irq, void *data)
{
	struct i2c_client *client = data;
	struct max17043_chip *chip = i2c_get_clientdata(client);

	dev_info(&chip->client->dev,
			"max17043: interrupt_handler: GAUGE_INT(%d):%d\n",
			chip->gpio_alert, gpio_get_value(chip->gpio_alert));

#if !defined(DEBUG_MODE)
	schedule_delayed_work(&chip->alert_work, 0);
#endif
	return IRQ_HANDLED;
}

/* sysfs interface : for AT Commands [START] */
ssize_t max17043_show_soc(struct device * dev,
			  struct device_attribute * attr, char *buf)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);
	int level;

	max17043_read_soc(chip->client);

	level = ((chip->soc) >> 8);
	if (level > 100)
		level = 100;
	else if (level < 0)
		level = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", level);
}
DEVICE_ATTR(soc, 0444, max17043_show_soc, NULL);

ssize_t max17043_show_status(struct device * dev,
			     struct device_attribute * attr, char *buf)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);

	max17043_read_vcell(chip->client);

	switch (chip->status) {
	case MAX17043_RESET:
		return snprintf(buf, PAGE_SIZE, "reset\n");
	case MAX17043_QUICKSTART:
		return snprintf(buf, PAGE_SIZE, "quickstart\n");
	case MAX17043_WORKING:
		return snprintf(buf, PAGE_SIZE, "working\n");
	default:
		return snprintf(buf, PAGE_SIZE, "ERROR\n");
	}
}

ssize_t max17043_store_status(struct device * dev,
			      struct device_attribute * attr,
			      const char *buf, size_t count)
{
	struct max17043_chip *chip = dev_get_drvdata(dev);

	if (strncmp(buf, "reset", 5) == 0) {
		max17043_reset(chip);
	} else if (strncmp(buf, "quickstart", 10) == 0) {
		max17043_quickstart();
	} else if (strncmp(buf, "working", 7) == 0) {
		// do nothing
	} else {
		return -1;
	}

	return count;
}

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-25,[P940] restore max17043 'state' attribute */
DEVICE_ATTR(state, 0666, max17043_show_status, max17043_store_status);
/* sysfs interface : for AT Commands [END] */

/* <jongho3.lee@lge.com> get capacity.. */
int max17043_get_ui_capacity(void)
{
	int ui_cap;

	if (reference == NULL)
		return 1;

	ui_cap = reference->soc - (1 << 8);
	ui_cap = (ui_cap * 100) / (RECHARGING_BAT_SOC_CON - SHUTDOWN_SOC_CON);
	/* LGE_CHANGE [bk.shin@lge.com] 2012-02-01, LGE_P940, add from P940 GB */
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-10-27, apply new half up algorithm */
#if 1
	ui_cap += 128;		// half up
	ui_cap >>= 8;
#else
	ui_cap >>= 8;
	if (reference->soc & 0x80)	// half up
		ui_cap++;
#endif

	if (ui_cap > 100)
		ui_cap = 100;
	else if (ui_cap < 0)
		ui_cap = 0;

	D("*** ui_cap = %d", ui_cap);
	return ui_cap;
}
EXPORT_SYMBOL(max17043_get_ui_capacity);

int max17043_reverse_get_ui_capacity(int ui_cap)
{
	int real_cap;

	if (reference == NULL)
		return 1;

	ui_cap <<= 8;
	ui_cap = (ui_cap * (RECHARGING_BAT_SOC_CON - SHUTDOWN_SOC_CON)) / 100;

	real_cap = ui_cap + (1 << 8);

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	if (real_cap & 0x80)	// half up
		real_cap += (1 << 8);

	real_cap >>= 8;

	D("*** reversed real cap = %d", real_cap);
	return real_cap;
}
EXPORT_SYMBOL(max17043_reverse_get_ui_capacity);

/* SYMBOLS to use outside of this module */
int max17043_get_capacity(void)
{
	if (reference == NULL)	// if fuel gauge is not initialized,
		return 1;	// return Dummy Value
	return reference->capacity;
}
EXPORT_SYMBOL(max17043_get_capacity);

int max17043_get_voltage(void)
{
	if (reference == NULL)	// if fuel gauge is not initialized,
		return 4200;	// return Dummy Value
	return reference->voltage;
}
EXPORT_SYMBOL(max17043_get_voltage);

int max17043_do_calibrate(void)
{
	if (reference == NULL)
		return -1;

	max17043_update(reference);
	if (max17043_validate_gauge_value
	    (reference->voltage, reference->capacity)) {
		return 0;
	}
	max17043_quickstart();
	return 0;
}
EXPORT_SYMBOL(max17043_do_calibrate);

int max17043_set_rcomp_by_temperature(int temp)
{
	int rcomp;

	if (reference == NULL)
		return -1;	// MAX17043 not initialized

	rcomp = reference->default_rcomp;

	temp /= 10;

	if (temp < 20)
		rcomp += 5 * (20 - temp);
	else if (temp > 20)
		rcomp -= (22 * (temp - 20)) / 10;

	if (rcomp < 0x00)
		rcomp = 0x00;
	else if (rcomp > 0xff)
		rcomp = 0xff;

	max17043_set_rcomp(rcomp);

	return 0;
}
EXPORT_SYMBOL(max17043_set_rcomp_by_temperature);
/* End SYMBOLS */

static int __devinit max17043_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17043_chip *chip;
	int ret = 0;
	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add platform_data */
	struct max17043_platform_data *pdata = client->dev.platform_data;

	D("max17043_probe I2C...............................................");

	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] check the pdata */
	if (pdata == NULL)
		return -EINVAL;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	reference = chip;
	chip->client = client;

	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] get the values from pdata */
	chip->default_rcomp = pdata->rcomp;
	chip->gpio_alert = pdata->gpio_alert;

	ret = gpio_request(chip->gpio_alert, "max17043_alert");
	if (ret < 0) {
		dev_err(&client->dev, "%s: gpio %d request Failed\n",
				__func__, chip->gpio_alert);
		goto err_gpio_request_failed;
	}
	gpio_direction_input(chip->gpio_alert);

	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] pass the client */
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-20,[P940] modify */
	ret = request_irq(gpio_to_irq(chip->gpio_alert),
			  max17043_interrupt_handler,
			  IRQF_TRIGGER_FALLING, MAX17043_I2C_NAME, client);
	if (ret < 0) {
		dev_err(&client->dev, "%s: IRQ Request Failed\n", __func__);
		goto err_request_irq_failed;
	}

	ret = enable_irq_wake(gpio_to_irq(chip->gpio_alert));
	if (ret < 0) {
		dev_err(&client->dev, "%s: set GAUGE_INT to wakeup source failed.\n", __func__);
		goto err_request_wakeup_irq_failed;
	}
	//enable_irq(gpio_to_irq(chip->gpio_alert));

	i2c_set_clientdata(client, chip);

	/* sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/soc */
	ret = device_create_file(&client->dev, &dev_attr_soc);
	if (ret < 0) {
		dev_err(&client->dev, "%s:File device creation failed: %d\n", __func__, ret);
		goto err_create_file_soc_failed;
	}
	/* sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/state */
	ret = device_create_file(&client->dev, &dev_attr_state);
	if (ret < 0) {
		dev_err(&client->dev, "%s:File device creation failed: %d\n", __func__, ret);
		goto err_create_file_state_failed;
	}

	chip->vcell = 3360;
	chip->soc = 100 << 8;
	chip->voltage = 4200;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 *  [P940] Imported from Cosmopolitan : 0->100
	 */
	chip->capacity = 100;
	chip->config = 0x971C;
	chip->fg_enable = 1;

	INIT_DELAYED_WORK_DEFERRABLE(&chip->alert_work, max17043_alert_work);
	INIT_DELAYED_WORK_DEFERRABLE(&chip->gauge_work, max17043_update_work);

	max17043_read_version(client);
	max17043_read_config(client);
	/* <jongho3.lee@lge.com> rcom value should be set with temp */
#if 0
	max17043_set_rcomp(chip->default_rcomp);
	if (need_to_quickstart == -1) {
		max17043_quickstart();
		need_to_quickstart = 0;
		return 0;
	} else
#endif
	schedule_delayed_work(&chip->gauge_work, 0);
	max17043_clear_interrupt(chip);

	dev_info(&client->dev, "max17043 probed\n");

	return 0;

err_create_file_state_failed:
	device_remove_file(&client->dev, &dev_attr_soc);
err_create_file_soc_failed:
	disable_irq_wake(gpio_to_irq(chip->gpio_alert));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(chip->gpio_alert), client);
err_request_irq_failed:
	gpio_free(chip->gpio_alert);
err_gpio_request_failed:
	kfree(chip);
	chip = NULL;
	reference = NULL;

	return ret;
}

static int __devexit max17043_remove(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_state);
	device_remove_file(&client->dev, &dev_attr_soc);

	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] free irq and gpio */
	free_irq(gpio_to_irq(chip->gpio_alert), client);
	gpio_free(chip->gpio_alert);

	cancel_delayed_work_sync(&chip->alert_work);
	cancel_delayed_work_sync(&chip->gauge_work);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	chip = NULL;
	reference = NULL;
	return 0;
}

#ifdef CONFIG_PM
static int max17043_suspend(struct i2c_client *client, pm_message_t state)
{
	int alert_level;
	struct max17043_chip *chip = i2c_get_clientdata(client);

	max17043_read_config(client);
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-20,
	 * [P940] Alert Threshold bug fix
	 */
	/* LGE_CHANGE [bk.shin@lge.com] 2012-02-01, LGE_P940, add from P940 GB
	 * euiseop.shin 2011-12-22
	 */
	alert_level = max17043_next_alert_level(max17043_get_ui_capacity());
	max17043_set_alert_level(alert_level);

	cancel_delayed_work(&chip->gauge_work);
	flush_delayed_work(&chip->alert_work);

	return 0;
}

static int max17043_resume(struct i2c_client *client)
{
	schedule_delayed_work(&reference->gauge_work, 0);

	return 0;
}
#else
#define max17043_suspend NULL
#define max17043_resume NULL
#endif /* CONFIG_PM */

/* euiseop.shin 20110526 */
static const struct i2c_device_id max17043_id[] = {
	{MAX17043_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, max17043_id);

static struct i2c_driver max17043_i2c_driver = {
	.driver = {
		.name = MAX17043_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = max17043_probe,
	.remove = __devexit_p(max17043_remove),
	.suspend = max17043_suspend,
	.resume = max17043_resume,
	.id_table = max17043_id,
};

/* boot argument from boot loader */
static s32 __init max17043_state(char *str)
{
	switch (str[0]) {
	case 'g':		// fuel gauge value is good
	case 'q':		// did quikcstart.
		need_to_quickstart = 0;
		break;
	case 'b':		// battery not connected when booting
		need_to_quickstart = 1;
		break;
	case 'e':		// quickstart needed. but error occured.
		need_to_quickstart = -1;
		break;
	default: 		// can not enter here
		break;
	}
	return 0;
}
__setup("fuelgauge=", max17043_state);

static int __init max17043_init(void)
{
	return i2c_add_driver(&max17043_i2c_driver);
}

module_init(max17043_init);

static void __exit max17043_exit(void)
{
	i2c_del_driver(&max17043_i2c_driver);
}

module_exit(max17043_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("MAX17043 Fuel Gauge");
MODULE_LICENSE("GPL");
