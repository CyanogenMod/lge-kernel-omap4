/*
 * Copyright 2011 LG Electronic Inc. All Rights Reserved.

 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTIES OR REPRESENTATIONS; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.See the GNU General Public License
 * for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/power_supply.h>

//#include <linux/cosmo/fuel_gauge_max17043.h>
#include <linux/fuel_gauge_max17043.h>
//#include <linux/cosmo/charger_rt9524.h>
#include <linux/charger_rt9524.h>
//#include <linux/cosmo/cosmo_muic.h>
#include <linux/muic/muic.h>

static DEFINE_MUTEX(fuel_update_lock);

struct max17043_dev {
	u16	is_active;		/* Being used as I2C? */
	struct	i2c_adapter adapter;
	struct	i2c_algo_bit_data algo;
	/* Spinlock for access to device registers.  Not yet globally used. */
	spinlock_t	reg_lock;
	struct platform_device *pdev;
	struct max17043_platform_data *pdata;

	struct delayed_work		gauge_work;
	struct delayed_work		alert_work;

#if defined(FG_WAKE_LCOK_TIMEOUT)
	struct wake_lock		fg_timed_wake_lock;
#endif

	/* Max17043 Registers.(Raw Data) */
	int vcell;				// VCELL Register vaule
	int soc;				// SOC Register value
	int version;			// Max17043 Chip version
	int config;				// RCOMP, Sleep, ALRT, ATHD

	/* Interface with Android */
	int voltage;			// Battery Voltage   (Calculated from vcell)
	int capacity;			// Battery Capacity  (Calculated from soc)
	int fg_enable;			// Battery Capacity  (Calculated from soc)
	max17043_status status;	// State Of max17043
};

struct max17043_dev *i2c_max17043_dev = NULL;


struct max17043_calibration_data {
	int voltage;	/* voltage in mA */
	int capacity;	/* capacity in % */
	int gradient;	/* gradient * 1000 */
	int intercept;	/* intercept * 1000 */
};

// 180mA Load for Battery
static struct max17043_calibration_data without_charger[] = {
	{3953,		81,		9,		3242},
	{3800,		58,		7,		3403},
	{3740,		40,		3,		3611},
	{3695,		20,		2,		3650},
	{3601,		4,		6,		3574},
	{3300,		0,		55,		3548},
	{ -1, -1, -1, -1},	// End of Data
};
// 770mA Charging Battery
static struct max17043_calibration_data with_charger[] = {
	{3865,		2,		66,		3709},
	{3956,		19,		5,		3851},
	{4021,		46,		2,		3912},
	{4088,		61,		5,		3813},
	{4158,		71,		7,		3689},
	{4200,		100,		2,		4042},
	{ -1, -1, -1, -1},	// End of Data
};

#define SOC_TIMES 1000
#define FUEL_GAUGE_VOLT_ERROR_RANGE		53       // 43 * (250/200)
battery_graph_prop max17043_battery_soc_graph[] =
{
	{4100, 100 * SOC_TIMES },
	{3893, 75 * SOC_TIMES },
	{3783, 57 * SOC_TIMES },
	{3721, 36 * SOC_TIMES },
	{3686, 20 * SOC_TIMES },
	{3575, 4 * SOC_TIMES },
	{3487, 2 * SOC_TIMES },
	{3300, 0 * SOC_TIMES },
};

int max17043_reference_graph(int __x, battery_graph_prop* ref_battery_graph, int arraysize, int* error_range)
{
	int i = 1;
	int __y = 0;
	int slope, const_term;
	int delta_y, delta_x;

	D(" battery graph array size = %d", arraysize );
	D(" battery graph array size = %d", arraysize );

	while( __x < ref_battery_graph[i].x \
			&& i < (arraysize - 1) )
	{
		i++;
	}

	delta_x = ref_battery_graph[i-1].x - ref_battery_graph[i].x;
	delta_y = (ref_battery_graph[i-1].y - ref_battery_graph[i].y);

	slope = delta_y  / delta_x;

	const_term = (ref_battery_graph[i].y) - (ref_battery_graph[i].x * slope);

	__y = (__x* slope + const_term);

	//[jongho3.lee@lge.com] Soc error range should be allowd as much as fuel gauge volt error range.
	if(error_range)
	{
		*error_range = slope * (FUEL_GAUGE_VOLT_ERROR_RANGE) / (*error_range);

		if(*error_range < 0)
		{
			*error_range *= -1;
		}

		if(*error_range < 5)
		{
			*error_range = 5;
		}
	}


	D(" ####### array_size = %d ##########", arraysize);
	D(" ##### SLOPE = %d, CONST_TERM = %d ##########", slope, const_term);
	D(" ##### CALCULATED __y = %d ##########", __y);

	if(ref_battery_graph[i-1].y > ref_battery_graph[i].y)
	{
		if(__y > ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y < ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	}
	else
	{
		if(__y < ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y > ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			D(" ##### fixing __y = %d ##########", __y);
		}
	}

	return __y;
}

int max17043_validate_gauge_value(int voltage, int capacity)
{
	int calculated_soc, error_range = SOC_TIMES;

	calculated_soc = max17043_reference_graph(voltage, max17043_battery_soc_graph, ARRAY_SIZE(max17043_battery_soc_graph), &error_range) / SOC_TIMES;

	D(" ##### SOC = 0x%x,  CALCULATED SOC = %d, error_range = %d  ########", capacity, calculated_soc, error_range);

	if( (capacity < calculated_soc + error_range) && (capacity > calculated_soc - error_range) )
	{
		D(" ##### SOC & CALCULATED SOC is met  ##########");
		//[jongho3.lee@lge.com] capacity validation should be performed just once.
		return 1;
	}

	D(" ##### SOC & CALCULATED SOC is NOT met 1!!!  ##########");
	//[jongho3.lee@lge.com] capacity validation should be performed just once.
	return 0;

}

int need_to_quickstart = 0;
EXPORT_SYMBOL(need_to_quickstart);

int set_fg_enable(int en)
{
	if(!i2c_max17043_dev)

	{
		return 0;
	}

	i2c_max17043_dev->fg_enable = en;
	return 0;
}
EXPORT_SYMBOL(set_fg_enable);

int get_fg_enable(void)
{
	if(!i2c_max17043_dev)
	{
		return 0;
	}

	return i2c_max17043_dev->fg_enable;
}
EXPORT_SYMBOL(get_fg_enable);

int is_cam_on(void)
{
	if(!i2c_max17043_dev)
	{
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(is_cam_on);

static void max17043_i2c_setsda(void *data, int state)
{
	struct max17043_dev *max_dev = data;
	unsigned long flags;

	spin_lock_irqsave(&i2c_max17043_dev->reg_lock, flags);
  	gpio_direction_output(max_dev->pdata->gpio_sda, 1);
	gpio_set_value(max_dev->pdata->gpio_sda, state);
	spin_unlock_irqrestore(&i2c_max17043_dev->reg_lock, flags);
}

static void max17043_i2c_setsda_dir(void *data, int state)
{
	struct max17043_dev *max_dev = data;
	unsigned long flags;

	spin_lock_irqsave(&i2c_max17043_dev->reg_lock, flags);
	if (state)
		gpio_direction_input(max_dev->pdata->gpio_sda);
	else
		gpio_direction_output(max_dev->pdata->gpio_sda, 0);
	spin_unlock_irqrestore(&i2c_max17043_dev->reg_lock, flags);
}

static void max17043_i2c_setscl(void *data, int state)
{
	struct max17043_dev *max_dev = data;
	unsigned long flags;

	spin_lock_irqsave(&i2c_max17043_dev->reg_lock, flags);
    gpio_set_value(max_dev->pdata->gpio_scl, state);
	spin_unlock_irqrestore(&i2c_max17043_dev->reg_lock, flags);
}

static int max17043_i2c_getsda(void *data)
{
	struct max17043_dev *max_dev = data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&i2c_max17043_dev->reg_lock, flags);
	gpio_direction_input(max_dev->pdata->gpio_sda);
	ret = gpio_get_value(max_dev->pdata->gpio_sda);
	spin_unlock_irqrestore(&i2c_max17043_dev->reg_lock, flags);
	return ret;
}

static int max17043_i2c_getscl(void *data)
{
	struct max17043_dev *max_dev = data;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&i2c_max17043_dev->reg_lock, flags);
	ret = gpio_get_value(max_dev->pdata->gpio_scl);
	spin_unlock_irqrestore(&i2c_max17043_dev->reg_lock, flags);
	return ret;
}

int max17043_i2c_readbyte(u8 slave_addr, u8 index, u8 *pdata)
{
	struct i2c_msg msgs[2];
	u8 mm1[] = {0x00};
	int ret = 0;

	if (!i2c_max17043_dev->is_active)
		return -ENODEV;

	mutex_lock(&fuel_update_lock);

	mm1[0] = index;
	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = mm1;

	msgs[1].addr = slave_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = pdata;

	ret = i2c_transfer(&i2c_max17043_dev->adapter, msgs, 2);

	mutex_unlock(&fuel_update_lock);

	return ret;
}

int max17043_i2c_read_word(u8 slave_addr, u8 index, u8 *buf)
{
	struct i2c_msg msgs[2];
	u8 mm1[] = {0x00};
	int ret = 0;

	if (!i2c_max17043_dev->is_active)
		return -ENODEV;

	mutex_lock(&fuel_update_lock);

	mm1[0] = index;
	msgs[0].addr = slave_addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = mm1;

	msgs[1].addr = slave_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 2;
	msgs[1].buf = buf;

	ret = i2c_transfer(&i2c_max17043_dev->adapter, msgs, 2);
 	if ( ret < 0) {
		printk(KERN_ERR "max17043_i2c_readbytes: transfer error\n");
		ret = -EIO;
	}

	mutex_unlock(&fuel_update_lock);

	return ret;
}

int max17043_i2c_writebyte(u8 slave_addr, u8 index, u8 data)
{
	u8 msg[2] = {index, data};
	struct i2c_msg msgs;
	int ret = 0;

	if (!i2c_max17043_dev->is_active)
		return -ENODEV;

	mutex_lock(&fuel_update_lock);

	msgs.flags = 0;
	msgs.addr = slave_addr;
	msgs.len = 2;
	msgs.buf = msg;

	ret = i2c_transfer(&i2c_max17043_dev->adapter, &msgs, 1);

	mutex_unlock(&fuel_update_lock);

	return ret;
}

int max17043_i2c_write_word(u8 slave_addr, u8 index, u8 *buf)
{
	struct i2c_msg msg;
	u8 write_buff[4];
	int ret = 0;

	if (!i2c_max17043_dev->is_active)
		return -ENODEV;

	mutex_lock(&fuel_update_lock);

	write_buff[0] = index;
	write_buff[1] = buf[0];
	write_buff[2] = buf[1];

	msg.addr = slave_addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = write_buff;


	if (i2c_transfer(&i2c_max17043_dev->adapter, &msg, 1) < 0) {
		printk(KERN_ERR "max17043_i2c_writebytes: transfer error\n");
		ret = -EIO;
	}

	mutex_unlock(&fuel_update_lock);

	return ret;

}

#if 0
void max17043_test(void)
{
	u8 buff_r[2];
	u8 buff_w[2];
	u32 i, j;

	D("Start of max17043_test\n");
	for(j = 0; j < 1000; j++) {
		D("#%d\n", j);
		for(i = 0; i < 16;) {
		#if 0
		//read version register
		max17043_i2c_readbytes(FUEL_GAUGE_MAX17043_SLAVE_ID, 0x6, buff_r, 2);
		D("version register 0x%04x 0x%04x \n", buff_r[0], buff_r[1]);
		#endif

		//read  config register
		//max17043_i2c_readbytes(FUEL_GAUGE_MAX17043_SLAVE_ID, 0xc, buff_r, 2);
		//D("before config register 0x%04x 0x%04x \n", buff_r[0], buff_r[1]);

		//write config register
		buff_w[0] = i++;
		buff_w[1] = i++;
		max17043_i2c_write_word(FUEL_GAUGE_MAX17043_SLAVE_ID, 0xc, buff_w);
		//D("write config register \n");

		//read config register
		max17043_i2c_read_word(FUEL_GAUGE_MAX17043_SLAVE_ID, 0xc, buff_r);
		//D("after config register 0x%04x 0x%04x \n", buff_r[0], buff_r[1]);
		D("buff_w[0]= %x, buff_w[1]= %x, buff_r[0]=%x, buff_r[1]=%x",
		buff_w[0], buff_w[1], buff_r[0], buff_r[1]);

		if ((buff_w[0] != buff_r[0]) || (buff_w[1] != buff_r[1]))
			D("I2C read write ERROR!!!!!!!i=%d, buff_r[0]=%d, buff_r[1]=%d\n",
				i, buff_r[0], buff_r[1]);

		}
	}
	D("End of max17043_test\n");
}
#endif

static int max17043_write_reg(struct max17043_dev *max_dev, int reg, u16 value)
{
	int ret = 0;
	int retry = 1;
	u8	data[2];

	if(!get_fg_enable())
	{
		D("MAX17043 CAMERAIS ON! DONt try to use I2C.....,%d,",get_fg_enable());
		return -1;
	}

	data[0] = ((value & 0xFF00) >> 8);
	data[1] = (value & 0xFF);

	while(retry--) {
		max17043_i2c_write_word(max_dev->pdata->slave_addr, reg, data);

		if (ret < 0)
		{
			dev_err(&max_dev->pdev->dev, "%s: err %d\n", __func__, ret);
			D("MAX17043 I2C WRITE ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			//<jongho3.lee@lge.com> sleep before next try
			msleep(1);
		}
		else
			break;
	}

	return ret;
}



static int max17043_read_reg(struct max17043_dev *max_dev, int reg)
{
	int ret = 0;
	int retry = 1;
	u8	data[2];

	if(!get_fg_enable())
	{
		D("MAX17043 CAMERAIS ON! DONt try to use I2C....., %d,",!get_fg_enable());
		return -1;
	}

	while(retry--) {
		D("MAX17043 I2C READ REG@@@@@@@@@@@@@@");

		ret = max17043_i2c_read_word(max_dev->pdata->slave_addr, reg, data);
		if (ret < 0)
		{
			dev_err(&max_dev->pdev->dev, "%s: err %d\n", __func__, ret);
			D("MAX17043 I2C READ ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			//<jongho3.lee@lge.com> sleep before next try
			msleep(1);
		}
		else
			break;
	}
	if(ret < 0)
		return ret;

	return (((u16)data[0]) << 8) | ((u16)data[1]);
}

static int max17043_reset(struct max17043_dev *max_dev)
{
	if(!i2c_max17043_dev)
	{
		return 0;
	}

	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	max17043_write_reg(max_dev, MAX17043_CMD_REG, 0x5400);

	max_dev->status = MAX17043_RESET;
	dev_info(&max_dev->pdev->dev, "MAX17043 Fuel-Gauge Reset\n");
	return 0;
}

int max17043_quickstart(void)
{

	if(!i2c_max17043_dev)
	{
		return 0;
	}


	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	max17043_write_reg(i2c_max17043_dev, MAX17043_MODE_REG, 0x4000);

	i2c_max17043_dev->status = MAX17043_QUICKSTART;
	dev_info(&i2c_max17043_dev->pdev->dev, "MAX17043 Fuel-Gauge Quick-Start\n");

	cancel_delayed_work(&i2c_max17043_dev->gauge_work);
	charger_schedule_delayed_work(&i2c_max17043_dev->gauge_work, HZ/4);

	return 0;
}
EXPORT_SYMBOL(max17043_quickstart);

static int max17043_read_vcell(struct max17043_dev *max_dev)
{
	u16 value;

	if(!i2c_max17043_dev)
	{
		return 0;
	}


	D("!get_fg_enable() %d,",!get_fg_enable());
	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	value = max17043_read_reg(max_dev, MAX17043_VCELL_REG);
	D("max17043_read_vcell = %d", value);

	if(value < 0)
		return value;

	max_dev->vcell = value >> 4;

	return 0;
}
static int max17043_read_soc(struct max17043_dev *max_dev)
{
	u16 value;

	if(!i2c_max17043_dev)
	{
		return 0;
	}


	D("!get_fg_enable() %d,",!get_fg_enable());
	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	value = max17043_read_reg(max_dev, MAX17043_SOC_REG);
	D("max17043_read_soc= 0x%x", value);

	if(value < 0)
		return value;

	max_dev->soc = value;

	return 0;
}
static int max17043_read_version(struct max17043_dev *max_dev)
{
	u16 value;

	if(!i2c_max17043_dev)
	{
		return 0;
	}


	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	value = max17043_read_reg(max_dev, MAX17043_VER_REG);

	max_dev->version = value;

	dev_info(&max_dev->pdev->dev, "MAX17043 Fuel-Gauge Ver %d\n", value);

	return 0;
}
static int max17043_read_config(struct max17043_dev *max_dev)
{
	u16 value;

	if(!i2c_max17043_dev)
	{
		return 0;
	}

	if(!(i2c_max17043_dev->fg_enable))
	{
		return 0;
	}

	value = max17043_read_reg(max_dev, MAX17043_CONFIG_REG);

	if(value < 0)
		return value;

	max_dev->config = value;

	return 0;
}
static int max17043_write_config(struct max17043_dev *max_dev)
{

	if(!i2c_max17043_dev)
	{
		D("if(!i2c_max17043_dev)");
		return 0;
	}


	if(!(i2c_max17043_dev->fg_enable))
	{
		D("if(!(i2c_max17043_dev->fg_enable))");
		return 0;
	}

	max17043_write_reg(max_dev, MAX17043_CONFIG_REG, max_dev->config);
	D("max17043_write_reg(max_dev=0x%x, MAX17043_CONFIG_REG=0x%x, max_dev->config=0x%x)",max_dev,MAX17043_CONFIG_REG,max_dev->config);

	return 0;
}

static int max17043_need_quickstart(int charging)
{
	struct max17043_calibration_data* data;
	int i = 0;
	int expected;
	int diff;
	int vol;
	int level;

	if(i2c_max17043_dev == NULL)
		return 0;

	// Get Current Data
	vol = i2c_max17043_dev->voltage;
	level = i2c_max17043_dev->soc >> 8;
	if(level > 100)
		level = 100;
	else if(level < 0)
		level = 0;

	// choose data to use
	if(charging) {
		data = with_charger;
		while(data[i].voltage != -1) {
			if(vol <= data[i].voltage)
				break;
			i++;
		}
	} else {
		data = without_charger;
		while(data[i].voltage != -1) {
			if(vol >= data[i].voltage)
				break;
			i++;
		}
	}

	// absense of data
	if(data[i].voltage == -1) {
		if(charging) {
			if(level == 100)
				return 0;
			else
				return 1;
		}
		else {
			if(level == 0)
				return 0;
			else
				return 1;
		}
	}

	// calculate diff
	expected = (vol - data[i].intercept) / data[i].gradient;
	if(expected > 100)
		expected = 100;
	else if(expected < 0)
		expected = 0;
	diff = expected - level;

	// judge
	if(diff < -MAX17043_TOLERANCE || diff > MAX17043_TOLERANCE) {
		//printk(KERN_DEBUG "[BATTERY] real : %d%% , expected : %d%%\n", level, expected);
		need_to_quickstart += 1;
	} else {
		need_to_quickstart = 0;
	}

	// Maximum continuous reset time is 2. If reset over 2 times, discard it.
	if(need_to_quickstart > 2)
		need_to_quickstart = 0;

	return need_to_quickstart;
}

static int max17043_next_alert_level(int level)
{
	int next_level;
	if(level > 15)
		next_level = max17043_reverse_get_ui_capacity(15);
	else if(level < 5)
		next_level = max17043_reverse_get_ui_capacity(level-1);
	else
		next_level = max17043_reverse_get_ui_capacity(3);

	return next_level;
}

static int max17043_set_rcomp(int rcomp)
{
	if(i2c_max17043_dev == NULL)
		return -1;

	rcomp &= 0xff;
	i2c_max17043_dev->config = ((i2c_max17043_dev->config & 0x00ff) | (rcomp << 8));

	max17043_write_config(i2c_max17043_dev);

	return 0;
}

static int max17043_set_athd(int level)
{
	if(i2c_max17043_dev == NULL)
		return -1;

	D("level = %d,",level);

	if(level > 32)
		level = 32;
	else if(level < 1)
		level = 1;

	if( (level << 8) >=  i2c_max17043_dev->soc)
	{
		level--;
		D("level = %d,",level);
	}
	D("next wake up soc = 0x%x , soc_now = 0x%x", (level << 8), i2c_max17043_dev->soc);

	level = 32 - level;
	if(level == (i2c_max17043_dev->config & 0x1F))
		return level;

	D("level = %d,",level);


	i2c_max17043_dev->config = ((i2c_max17043_dev->config & 0xff00) | level);
	D("i2c_max17043_dev->config = 0x%x,",i2c_max17043_dev->config);
	max17043_write_config(i2c_max17043_dev);

	return level;
}
static int max17043_clear_interrupt(struct max17043_dev *max_dev)
{

	if(max_dev->config & 0x20) {
		max_dev->config &= 0xffdf;
		max17043_write_config(max_dev);
	}

	return 0;
}


static int max17043_update(void)
{
	int ret;


	if(i2c_max17043_dev == NULL)
	{
		return -1;
	}

	ret = max17043_read_soc(i2c_max17043_dev);
	if(ret < 0)
		return ret;

	//<jongho3.lee@lge.com> sleep before next try
	ret = max17043_read_vcell(i2c_max17043_dev);
	if(ret < 0)
		return ret;

	D("MAX17043 fuel-gauge i2c_max17043_dev->voltage= %d", i2c_max17043_dev->voltage );
	D("MAX17043 fuel-gauge i2c_max17043_dev->soc = 0x%x", i2c_max17043_dev->soc);

	/* convert raw data to usable data */
	i2c_max17043_dev->voltage = (i2c_max17043_dev->vcell * 5) >> 2;	// vcell * 1.25 mV
	i2c_max17043_dev->capacity = i2c_max17043_dev->soc >> 8;
	// adjust full condition...just hack...
/*
	if(i2c_max17043_dev->soc & 0x80)	// half up
		i2c_max17043_dev->capacity++;
*/

	if(i2c_max17043_dev->capacity > 100)
		i2c_max17043_dev->capacity = 100;
	else if(i2c_max17043_dev->capacity < 0)
		i2c_max17043_dev->capacity = 0;

	i2c_max17043_dev->status = MAX17043_WORKING;
	D("MAX17043 fuel-gauge i2c_max17043_dev->capacity= %d", i2c_max17043_dev->capacity);

	return 0;
}

static void max17043_update_work(struct work_struct *work)
{
	max17043_update();
	charger_schedule_delayed_work(&i2c_max17043_dev->gauge_work, MAX17043_WORK_DELAY);

	return;
}


int max17043_update_by_other(void)
{
	if(i2c_max17043_dev == NULL)
	{
		return -1;
	}

	charger_schedule_delayed_work(&i2c_max17043_dev->gauge_work, 0);

	return 0;
}
EXPORT_SYMBOL(max17043_update_by_other);


static void max17043_alert_work(struct work_struct *work)
{
	if(i2c_max17043_dev == NULL)
		return;

	D("max17043_alert_work, soc_now = 0x%x", i2c_max17043_dev->soc);

	max17043_update();
#if defined(FG_WAKE_LCOK_TIMEOUT)
	wake_lock_timeout(&(i2c_max17043_dev->fg_timed_wake_lock), 2*HZ);
#endif

	max17043_clear_interrupt(i2c_max17043_dev);

}

static irqreturn_t max17043_interrupt_handler(int irq, void *data)
{
	if(i2c_max17043_dev == NULL) {
		return IRQ_HANDLED;
	}
#if defined(DEBUG_MODE)
#else
	charger_schedule_delayed_work(&(i2c_max17043_dev->alert_work), 0);
#endif
	return IRQ_HANDLED;
}
#if 0	// B-Project Does not use fuel gauge as a battery driver
/* sysfs(power_supply) interface : for Android Battery Service [START] */
static enum power_supply_property max17043_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};
static int max17043_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17043_dev *max_dev = container_of(psy,
				struct max17043_dev, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max_dev->voltage;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max_dev->capacity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/* sysfs interface : for Android Battery Service [END] */
#endif
/* sysfs interface : for AT Commands [START] */
ssize_t max17043_show_soc(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	int level;

	if(i2c_max17043_dev == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");

	max17043_read_soc(i2c_max17043_dev);

	level = ((i2c_max17043_dev->soc) >> 8);
	if(level > 100)
		level = 100;
	else if(level < 0)
		level = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", level);
}
DEVICE_ATTR(soc, 0444, max17043_show_soc, NULL);
ssize_t max17043_show_status(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{

	max17043_read_vcell(i2c_max17043_dev);
	if(i2c_max17043_dev == NULL)
		return snprintf(buf, PAGE_SIZE, "ERROR\n");
	switch(i2c_max17043_dev->status) {
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
ssize_t max17043_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	if(i2c_max17043_dev == NULL)
		return -1;

	if(strncmp(buf,"reset",5) == 0) {
		max17043_reset(i2c_max17043_dev);
	} else if(strncmp(buf,"quickstart",10) == 0) {
		max17043_quickstart();
	} else if(strncmp(buf,"working",7) == 0) {
		// do nothing
	} else {
		return -1;
	}
	return count;
}
DEVICE_ATTR(state, 0664, max17043_show_status, max17043_store_status);
/* sysfs interface : for AT Commands [END] */

//<jongho3.lee@lge.com> get capacity..
int max17043_get_ui_capacity(void)
{
	int ui_cap;

	if(i2c_max17043_dev == NULL)
		return 1;
	ui_cap = i2c_max17043_dev->soc - (SHUTDOWN_SOC_CON << 8);
	ui_cap = (ui_cap * 100) / (RECHARGING_BAT_SOC_CON - SHUTDOWN_SOC_CON);
	ui_cap >>= 8;
	if(i2c_max17043_dev->soc & 0x80)	// half up
		ui_cap++;

	if(ui_cap > 100)
	{
		ui_cap = 100;
	}
	else if(ui_cap < 0)
	{
		ui_cap = 0;
	}
	D("*** ui_cap = %d", ui_cap);
	return ui_cap;
}
EXPORT_SYMBOL(max17043_get_ui_capacity);

int max17043_reverse_get_ui_capacity(int ui_cap)
{
	int real_cap;

	if(i2c_max17043_dev == NULL)
		return 1;

	ui_cap <<= 8;
	ui_cap = (ui_cap * (RECHARGING_BAT_SOC_CON - SHUTDOWN_SOC_CON)) / 100;

	real_cap = ui_cap + (SHUTDOWN_SOC_CON << 8);

    if(real_cap  & 0x80) // half up
		real_cap += (1 << 8);

	real_cap >>= 8;

	D("*** reversed real cap = %d", real_cap);
	return real_cap;
}
EXPORT_SYMBOL(max17043_reverse_get_ui_capacity);


/* SYMBOLS to use outside of this module */
int max17043_get_capacity(void)
{
	if(i2c_max17043_dev == NULL)	// if fuel gauge is not initialized,
		return 1;			// return Dummy Value

	D("***  cap = %d", i2c_max17043_dev->capacity);
	return i2c_max17043_dev->capacity;
}
EXPORT_SYMBOL(max17043_get_capacity);
int max17043_get_voltage(void)
{
	if(i2c_max17043_dev == NULL)	// if fuel gauge is not initialized,
		return 4200;		// return Dummy Value
	return i2c_max17043_dev->voltage;
}
EXPORT_SYMBOL(max17043_get_voltage);
int max17043_do_calibrate(void)
{
	if(i2c_max17043_dev == NULL)
		return -1;

	max17043_update();

	if(max17043_validate_gauge_value(i2c_max17043_dev->voltage, i2c_max17043_dev->capacity))
	{
		return 0;
	}
	max17043_quickstart();

	return 0;
}
EXPORT_SYMBOL(max17043_do_calibrate);

int max17043_set_rcomp_by_temperature(int temp)
{
	int rcomp;
	if(i2c_max17043_dev == NULL)
		return -1;	// MAX17043 not initialized

	rcomp = RCOMP_BL48LN;

	temp /= 10;

	if(temp < 20)
	{
		rcomp += 5*(20-temp);
	}
	else if(temp>20)
	{
		rcomp -= (22*(temp-20))/10;
	}

	if(rcomp < 0x00)
		rcomp = 0x00;
	else if(rcomp > 0xff)
		rcomp = 0xff;

	max17043_set_rcomp(rcomp);
	return 0;
}
EXPORT_SYMBOL(max17043_set_rcomp_by_temperature);
int max17043_set_alert_level(int alert_level)
{
	return max17043_set_athd(alert_level);
}
EXPORT_SYMBOL(max17043_set_alert_level);
/* End SYMBOLS */


static int max17043_probe(struct platform_device *pdev)
{
	struct max17043_dev *max_dev;
	struct i2c_adapter *adapter;
	struct i2c_algo_bit_data *algo;
	int ret;

	D("########## max17043_probe GPIO....................................................... \n");

	max_dev = kzalloc(sizeof(struct max17043_dev), GFP_KERNEL);
	if (!max_dev)
	{
		printk(KERN_ERR "out of memory\n");
		return -ENOMEM;
	}

	max_dev->pdata = pdev->dev.platform_data;
	max_dev->pdev = pdev;

	algo = &max_dev->algo;
	adapter = &max_dev->adapter;

	max_dev->is_active = 0;

	algo->setsda = max17043_i2c_setsda_dir;
	algo->setscl = max17043_i2c_setscl;
	algo->getsda = max17043_i2c_getsda;
	algo->getscl = max17043_i2c_getscl;
	algo->udelay = max_dev->pdata->udelay;
	algo->timeout = max_dev->pdata->timeout;
	algo->data = max_dev;

	sprintf(adapter->name, "max17043_i2c");
	adapter->owner = THIS_MODULE;
	adapter->id = 0x01FFFF;
	adapter->class = I2C_CLASS_DDC;
	adapter->algo_data = algo;
	if (pdev)
		adapter->dev.parent = &pdev->dev;
	else
		adapter->dev.parent = NULL;

//	printk("########## max17043_probe GPIO REQUEST START................................ \n");
//	printk("########## max17043_probe GPIO REQUEST SDA................................ \n");

	ret = gpio_request(max_dev->pdata->gpio_sda, "fuel_gauge_sda");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO_%d for fuel_gauge_sda\n",
				max_dev->pdata->gpio_sda);
		ret = -ENOSYS;
		goto err_gpio_request_failed;
	}
	gpio_direction_output(max_dev->pdata->gpio_sda, 1);

//	printk("########## max17043_probe GPIO REQUEST SCL................................ \n");
	ret = gpio_request(max_dev->pdata->gpio_scl, "fuel_gauge_scl");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO_%d for fuel_gauge_scl\n",
				max_dev->pdata->gpio_scl);
		ret = -ENOSYS;
		goto err_gpio_request_failed;
	}
	gpio_direction_output(max_dev->pdata->gpio_scl, 1);

//	printk("########## max17043_probe GPIO REQUEST ALERT................................ \n");
	ret = gpio_request(max_dev->pdata->gpio_alert, "fuel_gauge_alert");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO_%d for fuel_gauge_alert\n",
				max_dev->pdata->gpio_alert);
		ret = -ENOSYS;
		goto err_gpio_request_failed;
	}
	gpio_direction_input(max_dev->pdata->gpio_alert);

//	printk("########## max17043_probe GPIO REQUEST ALERT IRQ................................ \n");
	ret = request_irq(gpio_to_irq(max_dev->pdata->gpio_alert),
				max17043_interrupt_handler,
				IRQF_TRIGGER_FALLING,
				"fuel_gauge_driver",
				max_dev);
	if (ret < 0) {
		printk(KERN_ERR "Failed to request IRQ for fuel_gauge_low\n");
		goto err_request_irq_failed;
	}

//	printk("########## max17043_probe GPIO REQUEST ALERT ENABLE_IRQ_WAKE................................ \n");
	ret = enable_irq_wake(gpio_to_irq(max_dev->pdata->gpio_alert));
	if (ret < 0) {
		printk(KERN_DEBUG "[MAX17043] set GAUGE_INT to wakeup source failed.\n");
		goto err_request_wakeup_irq_failed;
	}

	max_dev->vcell = 3360;
	max_dev->soc = 100 << 8;
	max_dev->voltage = 4200;
	max_dev->capacity = 100;
	max_dev->config = 0xB000;
	max_dev->fg_enable = 1;

//	printk("########## max17043_probe GPIO REQUEST PLATFORM_SET_DRVDATA................................ \n");
    platform_set_drvdata(pdev, max_dev);

	i2c_max17043_dev = max_dev;

	/* Raise SCL and SDA */
//	printk("########## max17043_probe GPIO REQUEST I2C_SETSDA................................ \n");
	max17043_i2c_setsda(max_dev, 1);
	udelay(20);
//	printk("########## max17043_probe GPIO REQUEST I2C_SETSCL................................ \n");
	max17043_i2c_setscl(max_dev, 1);
	udelay(20);

//	printk("########## max17043_probe GPIO REQUEST i2c_bit_add_bus................................ \n");
	ret = i2c_bit_add_bus(adapter);

	if (ret < 0) {
		printk(KERN_ERR "max17043: cannot create i2c bus %d\n", ret);
	}
	max_dev->is_active = 1;

//	printk("########## max17043_probe GPIO REQUEST max17043_read_version................................ \n");
	max17043_read_version(max_dev);
//	printk("########## max17043_probe GPIO REQUEST max17043_read_config................................ \n");
	max17043_read_config(max_dev);

	//max17043_test();



	INIT_DELAYED_WORK_DEFERRABLE(&i2c_max17043_dev->alert_work, max17043_alert_work);
	INIT_DELAYED_WORK_DEFERRABLE(&i2c_max17043_dev->gauge_work, max17043_update_work);

#if defined(FG_WAKE_LCOK_TIMEOUT)
	wake_lock_init(&i2c_max17043_dev->fg_timed_wake_lock, WAKE_LOCK_SUSPEND, "fg_timed_wakelock");
//	printk("########## max17043_probe GPIO REQUEST wake_lock_init................................ \n");
#endif

	{
//		printk("########## max17043_probe GPIO REQUEST charger_schedule_delayed_work................................ \n");
		charger_schedule_delayed_work(&max_dev->gauge_work, 0);
//		printk("########## max17043_probe GPIO REQUEST max17043_clear_interrupt................................ \n");
		max17043_clear_interrupt(max_dev);
	}

	// sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/soc
	ret = device_create_file(&max_dev->pdev->dev, &dev_attr_soc);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_soc_failed;
	}
	// sysfs path : /sys/devices/platform/i2c_omap.2/i2c-2/2-0036/state
	ret = device_create_file(&max_dev->pdev->dev, &dev_attr_state);
	if (ret < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, ret);
		ret = -ENODEV;
		goto err_create_file_state_failed;
	}

	return 0;


err_create_file_state_failed:
	device_remove_file(&max_dev->pdev->dev, &dev_attr_soc);
err_create_file_soc_failed:
	disable_irq_wake(gpio_to_irq(GAUGE_INT));
err_request_wakeup_irq_failed:
	free_irq(gpio_to_irq(max_dev->pdata->gpio_alert), NULL);
err_request_irq_failed:
	gpio_free(max_dev->pdata->gpio_sda);
	gpio_free(max_dev->pdata->gpio_scl);
	gpio_free(max_dev->pdata->gpio_alert);
err_gpio_request_failed:
	kfree(max_dev);

	return ret;
}

static int max17043_remove(struct platform_device *pdev)
{
	struct max17043_dev *max_dev = platform_get_drvdata(pdev);

	/*
	 * Only remove those entries in the array that we've
	 * actually used (and thus initialized algo_data)
	 */
	if (max_dev->is_active)
		i2c_del_adapter(&max_dev->adapter);
	return 0;
}

#ifdef CONFIG_PM
static int max17043_suspend(struct platform_device *pdev, pm_message_t state)
{
	int alert_level;

	struct max17043_dev *max_dev = platform_get_drvdata(pdev);

	max17043_update();
	max17043_read_config(i2c_max17043_dev);
	alert_level = max17043_next_alert_level(max17043_get_ui_capacity());
	max17043_set_athd(alert_level);
	D("%d i2c_max17043_dev->config = 0x%x,",max17043_read_config(i2c_max17043_dev),i2c_max17043_dev->config);

	cancel_delayed_work(&max_dev->gauge_work);
	flush_delayed_work(&max_dev->alert_work);

	pdev->dev.power.power_state = state;

	return 0;
}

static int max17043_resume(struct platform_device *pdev)
{

	charger_schedule_delayed_work(&i2c_max17043_dev->gauge_work, 0);
	pdev->dev.power.power_state = PMSG_ON;

	return 0;
}
#else
#define max17043_suspend NULL
#define max17043_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver max17043_driver = {
	.driver = {
		.name = "max17043_gpio",
	},
	.probe = max17043_probe,
	.remove = max17043_remove,
	.resume		= max17043_resume,
	.suspend	= max17043_suspend,
};

static int __init fuel_gauge_max17043_init(void)
{
	return platform_driver_register(&max17043_driver);
}

static void __exit fuel_gauge_max17043_exit(void)
{
	platform_driver_unregister(&max17043_driver);
}

module_init(fuel_gauge_max17043_init);
module_exit(fuel_gauge_max17043_exit);
