/*
 *  apds9900.c - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2011, 2012 LGE Inc.
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <mach/gpio.h>
#include <linux/lge/apds9900.h>

#define APDS9900_DRV_NAME	"apds9900"
#define DRIVER_VERSION		"1.0.0"

/*
 * Defines
 */

#define APDS9900_ENABLE_REG	0x00
#define APDS9900_ATIME_REG	0x01
#define APDS9900_PTIME_REG	0x02
#define APDS9900_WTIME_REG	0x03
#define APDS9900_AILTL_REG	0x04
#define APDS9900_AILTH_REG	0x05
#define APDS9900_AIHTL_REG	0x06
#define APDS9900_AIHTH_REG	0x07
#define APDS9900_PILTL_REG	0x08
#define APDS9900_PILTH_REG	0x09
#define APDS9900_PIHTL_REG	0x0A
#define APDS9900_PIHTH_REG	0x0B
#define APDS9900_PERS_REG	0x0C
#define APDS9900_CONFIG_REG	0x0D
#define APDS9900_PPCOUNT_REG	0x0E
#define APDS9900_CONTROL_REG	0x0F
#define APDS9900_REV_REG	0x11
#define APDS9900_ID_REG		0x12
#define APDS9900_STATUS_REG	0x13
#define APDS9900_CDATAL_REG	0x14
#define APDS9900_CDATAH_REG	0x15
#define APDS9900_IRDATAL_REG	0x16
#define APDS9900_IRDATAH_REG	0x17
#define APDS9900_PDATAL_REG	0x18
#define APDS9900_PDATAH_REG	0x19

#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

#define APDS_9900_IO		0x99
#define APDS_9900_IOCTL_ACTIVE           _IOW(APDS_9900_IO, 0x01, int)

#define APDS9900_ENABLE_PIEN 	0x20
#define APDS9900_ENABLE_AIEN 	0x10
#define APDS9900_ENABLE_WEN 	0x08
#define APDS9900_ENABLE_PEN 	0x04
#define APDS9900_ENABLE_AEN 	0x02
#define APDS9900_ENABLE_PON 	0x01

#define APDS9900_STATUS_PINT	0x20
#define APDS9900_STATUS_AINT	0x10


#define APDS900_SENSOR_DEBUG 0
 #if APDS900_SENSOR_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif

#define APDS_WATCHDOG_DEBUG 0

/*
 * Structs
 */

struct apds9900_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work dwork;
	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	unsigned int prox_lth;
	unsigned int prox_hth;
	unsigned int prox_stat; // 0: NEAR, 1: FAR
	unsigned int prox_raw;

	unsigned int lux;

	unsigned int GA;
	unsigned int DF;
	unsigned int LPC;

	unsigned int pdrive;
	unsigned int pdiode;
	unsigned int pgain;
	unsigned int again;
	unsigned int coeff_b;
	unsigned int coeff_c;
	unsigned int coeff_d;
	
	int irq;
	int irq_gpio;
	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	//struct input_dev *input_dev;
	struct input_dev *input_dev_prox;
	struct input_dev *input_dev_als;

	struct wake_lock wakelock;
};

/*
 * Global data
 */
static int apds_9900_initialized = 0;

/*
 * Management functions
 */

static int apds9900_set_command(struct i2c_client *client, int command)
{
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	ret = i2c_smbus_write_byte(client, clearInt);

	return ret;
}

static int apds9900_set_enable(struct i2c_client *client, int enable)
{
	int ret = 0;
	
	if (APDS_WATCHDOG_DEBUG)
		printk("apds9900: %s,%d -->>\n", __FUNCTION__, __LINE__);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_ENABLE_REG, enable);
	if (APDS_WATCHDOG_DEBUG)
		printk("apds9900: %s,%d --<<\n", __FUNCTION__, __LINE__);

	DEBUG_MSG("apds9900_set_enable = [%x] \n",enable);

	return ret;
}

static int apds9900_set_atime(struct i2c_client *client, int atime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_ATIME_REG, atime);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->atime = atime;

	return ret;
}
static int apds9900_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_PTIME_REG, ptime);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->ptime = ptime;

	return ret;
}

static int apds9900_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_WTIME_REG, wtime);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->wtime = wtime;

	return ret;
}

static int apds9900_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS9900_AILTL_REG, threshold);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}
	
	data->ailt = threshold;

	return ret;
}

static int apds9900_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS9900_AIHTL_REG, threshold);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}
	
	data->aiht = threshold;

	return ret;
}

static int apds9900_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS9900_PILTL_REG, threshold);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}
	
	data->pilt = threshold;

	return ret;
}

static int apds9900_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS9900_PIHTL_REG, threshold);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}
	
	data->piht = threshold;

	return ret;
}

static int apds9900_set_pers(struct i2c_client *client, int pers)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_PERS_REG, pers);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->pers = pers;

	return ret;
}

static int apds9900_set_config(struct i2c_client *client, int config)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_CONFIG_REG, config);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->config = config;

	return ret;
}

static int apds9900_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_PPCOUNT_REG, ppcount);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->ppcount = ppcount;

	return ret;
}

static int apds9900_set_control(struct i2c_client *client, int control)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;
	
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS9900_CONTROL_REG, control);
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error\n", __func__);
		return ret;
	}

	data->control = control;

	return ret;
}

unsigned int apds9900_get_LPC(struct i2c_client *client)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned int ALSIT;
	unsigned int LPC;


	if (data == NULL)
		return 0;

	/* Assume
	GA 	= 1.8395 // Glass Attenuation Factor (Open air - 0.46)
	DF	= 52	 // Device Factor (Open air - 52)
	ALSIT 	= 27.2 	 // ALS Integration Time  ALSIT = (256-ATIME)*2.72
	AGAIN 	= 1	 // ALS Gain
	LPC 	= GA * DF / (ALSIT * AGAIN) // Lux per count
		= 1.8395*52 / (27.2*8)
		= 0.4395
	*/
	ALSIT = (256-data->atime)*272; // org *100;
	LPC = (data->GA * data->DF * 100) / ALSIT; // AGAIN = 1 , LPC = org *100

	return LPC;
}

/* Set some defalut configuration to APDS-9900 */
static int apds_9900_init(struct i2c_client *client)
{
	int ret = 0;
	struct apds9900_data *data = i2c_get_clientdata(client);

	/*
	ATIME = 0xFF; // 27.2ms . minimum ALS integration time
	WTIME = 0xf6; // 27.2ms . minimum Wait time
	PTIME = 0xff; // 2.72ms . minimum Prox integration time
	*/
	ret = apds9900_set_atime(client, data->atime);
	ret |= apds9900_set_wtime(client, data->wtime);
	ret |= apds9900_set_ptime(client, data->ptime);

	//Interrupt persistence = 0x33 -> Proximity(4) | ALS(4)
	ret |= apds9900_set_pers(client, data->pers);

	// Wait long timer <- no needs so set 0
	ret |= apds9900_set_config(client, 0x00);
		
	// Pulse count for proximity
	ret |= apds9900_set_ppcount(client, data->ppcount);
	
	ret |= apds9900_set_control(client, 0x20 /*PDRIVE | PDIODE | PGAIN | AGAIN*/);

	// init threshold for proximity
	ret |= apds9900_set_pilt(client, 1023);
	ret |= apds9900_set_piht(client, 0);

	// init threshold for als
	ret |= apds9900_set_ailt(client, 0xFFFF);
	ret |= apds9900_set_aiht(client, 0);

	data->prox_stat = 1; /* FAR */

	data->LPC = apds9900_get_LPC(client);

	return ret;
}

/*
 * SysFS support
 */
static ssize_t apds9900_show_atime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->atime);
}

static ssize_t apds9900_store_atime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	apds9900_set_atime(client, rdata);
	data->LPC = apds9900_get_LPC(client);

	return count;
}

static DEVICE_ATTR(atime, 0660, apds9900_show_atime, apds9900_store_atime);	

static ssize_t apds9900_show_ptime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->ptime);
}

static ssize_t apds9900_store_ptime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	apds9900_set_ptime(client, rdata);

	return count;
}

static DEVICE_ATTR(ptime, 0660, apds9900_show_ptime, apds9900_store_ptime);	

static ssize_t apds9900_show_wtime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->wtime);
}

static ssize_t apds9900_store_wtime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	apds9900_set_wtime(client, rdata);

	return count;
}

static DEVICE_ATTR(wtime, 0660, apds9900_show_wtime, apds9900_store_wtime);	

static ssize_t apds9900_show_ppcount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->ppcount);
}

static ssize_t apds9900_store_ppcount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	apds9900_set_ppcount(client, rdata);

	return count;
}

static DEVICE_ATTR(ppcount, 0660, apds9900_show_ppcount, apds9900_store_ppcount);	

static ssize_t apds9900_show_pers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->pers);
}

static ssize_t apds9900_store_pers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	apds9900_set_pers(client, rdata);

	return count;
}

static DEVICE_ATTR(pers, 0660, apds9900_show_pers, apds9900_store_pers);

static ssize_t apds9900_show_pilt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->prox_lth);
}

static ssize_t apds9900_store_pilt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	data->prox_lth = rdata;

	return count;
}

static DEVICE_ATTR(pilt, 0660, apds9900_show_pilt, apds9900_store_pilt);	


static ssize_t apds9900_show_piht(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->prox_hth);
}

static ssize_t apds9900_store_piht(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	data->prox_hth = rdata;

	return count;
}

static DEVICE_ATTR(piht, 0660, apds9900_show_piht, apds9900_store_piht);	

static ssize_t apds9900_show_pdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int pdata;

	pdata = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS9900_PDATAL_REG);		
	if (pdata < 0) {
		dev_err(dev, "%s: i2c_error\n", __func__);
		pdata = -1;
	}

	return sprintf(buf, "%d\n",pdata);
}

static DEVICE_ATTR(pdata, 0440, apds9900_show_pdata, NULL);	

static ssize_t apds9900_show_GA(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->GA);
}

static ssize_t apds9900_store_GA(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	data->GA = rdata;
	data->LPC = apds9900_get_LPC(client);

	return count;
}

static DEVICE_ATTR(GA, 0660, apds9900_show_GA, apds9900_store_GA);	

static ssize_t apds9900_show_DF(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n",data->DF);
}

static ssize_t apds9900_store_DF(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	data->DF = rdata;
	data->LPC = apds9900_get_LPC(client);

	return count;
}

static DEVICE_ATTR(DF, 0660, apds9900_show_DF, apds9900_store_DF);	

static ssize_t apds9900_show_interrupt_prox(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable);
}
static ssize_t apds9900_show_interrupt_als(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable);
}

static ssize_t apds9900_store_interrupt_prox(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
// this value should be same with the value in sensors.cpp
#define STORE_INTERUPT_SELECT_PROXIMITY		0x02

	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	
	int enable = (int)rdata;
	//int val = 0;
	int ret;

	DEBUG_MSG("apds9900_store_interrupt = [%d] apds_9900_initialized [%d] \n",rdata, apds_9900_initialized);
	// if first enter , initializing
	if (!apds_9900_initialized) {
		ret = apds_9900_init(client);
		if (ret < 0) {
			dev_err(dev, "%s: apds_9900_init failed\n", __func__);
			return ret;
		}
		apds_9900_initialized = 1;
		enable_irq(data->irq);
	}

	disable_irq(data->irq);

	// check proximity enable flag
	if (enable & STORE_INTERUPT_SELECT_PROXIMITY) {	
		if(enable & 0x01) { // enable
			data->enable_ps_sensor = 1;
			data->enable |= (APDS9900_ENABLE_PIEN|APDS9900_ENABLE_PEN|APDS9900_ENABLE_PON);
		}
		else {	//disable
			data->enable_ps_sensor = 0;
			if (data->enable_als_sensor)
				 data->enable &= ~(APDS9900_ENABLE_PIEN|APDS9900_ENABLE_PEN);
			else
			    data->enable = 0;
		}
	}

	ret = apds9900_set_enable(client, data->enable);

	enable_irq(data->irq);

	DEBUG_MSG("apds9900_store_interrupt data->enable [%x] \n", data->enable);

	if (ret < 0)
		return ret;
	
	dev_info(dev, "apds9900: proximity enabled\n");
	return count;
}

static ssize_t apds9900_store_interrupt_als(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
// this value should be same with the value in sensors.cpp
#define STORE_INTERUPT_SELECT_ALS			0x04

	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	
	int enable = (int)rdata;
	int ret;

	DEBUG_MSG("apds9900_store_interrupt = [%d] apds_9900_initialized [%d] \n",rdata, apds_9900_initialized);
	// if first enter , initializing

	if (!apds_9900_initialized) {
		ret = apds_9900_init(client);
		if (ret < 0) {
			dev_err(dev, "%s: apds_9900_init failed\n", __func__);
			return ret;
		}
		apds_9900_initialized = 1;
		enable_irq(data->irq);
	}

	disable_irq(data->irq);

	// check ALS enable flag
	if (enable & STORE_INTERUPT_SELECT_ALS) {

		if(enable & 0x01) { // enable
			data->enable_als_sensor = 1;
			data->enable |= (APDS9900_ENABLE_AIEN|APDS9900_ENABLE_AEN|APDS9900_ENABLE_PON);
		}
		else {
			data->enable_als_sensor = 0;
			if(data->enable_ps_sensor)
				data->enable &= ~(APDS9900_ENABLE_AIEN | APDS9900_ENABLE_AEN);
			else
				data->enable = 0;
		}
	}

	ret = apds9900_set_enable(client, data->enable);

	enable_irq(data->irq);

	DEBUG_MSG("apds9900_store_interrupt data->enable [%x] \n", data->enable);

	if (ret < 0)
		return ret;
	
	dev_info(dev, "apds9900: light sensor enabled\n");
	return count;
}

static DEVICE_ATTR(interrupt_prox, 0660,
		   apds9900_show_interrupt_prox, apds9900_store_interrupt_prox);		   
static DEVICE_ATTR(interrupt_als, 0660,
		   apds9900_show_interrupt_als, apds9900_store_interrupt_als);		   
static ssize_t apds9900_show_proxidata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->prox_raw);
}

static DEVICE_ATTR(proxidata, 0660 , apds9900_show_proxidata, NULL);

static ssize_t apds9900_show_luxdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->lux);
}

static DEVICE_ATTR(luxdata, 0660 , apds9900_show_luxdata, NULL);

static ssize_t apds9900_show_irdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", i2c_smbus_read_word_data(client, CMD_WORD|APDS9900_IRDATAL_REG));
}

static DEVICE_ATTR(irdata, 0660, apds9900_show_irdata, NULL);

static ssize_t apds9900_show_cdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9900_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", i2c_smbus_read_word_data(client, CMD_WORD|APDS9900_CDATAL_REG));
}

static DEVICE_ATTR(cdata, 0660, apds9900_show_cdata, NULL);

static struct attribute *apds9900_attributes[] = {
	&dev_attr_atime.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_pers.attr,
	&dev_attr_pilt.attr,
	&dev_attr_piht.attr,
	&dev_attr_GA.attr,
	&dev_attr_DF.attr,
	&dev_attr_pdata.attr,
	&dev_attr_interrupt_prox.attr,
	&dev_attr_interrupt_als.attr,
	&dev_attr_proxidata.attr,
	&dev_attr_luxdata.attr,
	&dev_attr_irdata.attr,
	&dev_attr_cdata.attr,
	NULL
};

static const struct attribute_group apds9900_attr_group = {
	.attrs = apds9900_attributes,
};

/* LGE_SJIT_S 2011-12-05 [dojip.kim@lge.com] sysfs for new ICS Sensor HAL
 *  enable in sysfs
 *  poll_delay in sysfs
 */

static ssize_t poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t poll_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

static ssize_t light_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9900_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t light_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct apds9900_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	bool enable;
	int ret;

	if (sysfs_streq(buf, "1")) {
		enable = true;
	}
	else if (sysfs_streq(buf, "0")) {
		enable = false;
	}
	else {
		dev_err(dev, "apds9900: %s: invalid value %s\n", __func__, buf);
		return -EINVAL;
	}

	dev_info(dev, "apds9900: try to enable light %d\n", enable);

	if (!apds_9900_initialized) {
		ret = apds_9900_init(client);
		if (ret < 0) {
			dev_err(dev, "%s: apds_9900_init failed\n", __func__);
			return ret;
		}
		apds_9900_initialized = 1;
		enable_irq(data->irq);
	}

	disable_irq(data->irq);

	if(enable) { // enable
		data->enable_als_sensor = 1;
		data->enable |= (APDS9900_ENABLE_AIEN|APDS9900_ENABLE_AEN|APDS9900_ENABLE_PON);
	}
	else {
		data->enable_als_sensor = 0;
		if(data->enable_ps_sensor)
			data->enable &= ~(APDS9900_ENABLE_AIEN | APDS9900_ENABLE_AEN);
		else
			data->enable = 0;
	}

	ret = apds9900_set_enable(client, data->enable);

	enable_irq(data->irq);

	dev_dbg(dev, "apds9900: %s: data->enable 0x%x\n",
			__func__, data->enable);
	if (ret < 0)
		return ret;

	dev_info(dev, "apds9900: light sensor enabled\n");

	return size;
}

static ssize_t proximity_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9900_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t proximity_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct apds9900_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	bool enable;
	int ret;

	if (sysfs_streq(buf, "1")) {
		enable = true;
	}
	else if (sysfs_streq(buf, "0")) {
		enable = false;
	}
	else {
		dev_err(dev, "apds9900: %s: invalid value %s\n", __func__, buf);
		return -EINVAL;
	}

	dev_info(dev, "apds9900: try to enable proximity %d\n", enable);

	// if first enter , initializing
	if (!apds_9900_initialized) {
		ret = apds_9900_init(client);
		if (ret < 0) {
			dev_err(dev, "%s: apds_9900_init failed\n", __func__);
			return ret;
		}
		apds_9900_initialized = 1;
		enable_irq(data->irq);
	}

	disable_irq(data->irq);

	if(enable) { // enable
		data->enable_ps_sensor = 1;
		data->enable |= (APDS9900_ENABLE_PIEN|APDS9900_ENABLE_PEN|APDS9900_ENABLE_PON);
	}
	else {	//disable
		data->enable_ps_sensor = 0;
		if (data->enable_als_sensor)
			 data->enable &= ~(APDS9900_ENABLE_PIEN|APDS9900_ENABLE_PEN);
		else
		    data->enable = 0;
	}

	ret = apds9900_set_enable(data->client, data->enable);

	enable_irq(data->irq);

	dev_dbg(dev, "apds9900: %s: data->enable [%x] \n",
			__func__, data->enable);

	if (ret < 0)
		return ret;

	dev_info(dev, "apds9900: proximity enabled\n");

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		light_enable_show, light_enable_store);

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		proximity_enable_show, proximity_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};
/* LGE_SJIT_E 2011-12-05 [dojip.kim@lge.com] sysfs for new ICS Sensor HAL */

/*
 * Initialization function
 */
static int apds9900_init_client(struct i2c_client *client)
{
	struct apds9900_data *data = i2c_get_clientdata(client);
	int ret;

	apds9900_set_enable(client, 0);

	mdelay(100);

	ret = i2c_smbus_read_byte_data(client, APDS9900_ENABLE_REG);
	if (ret < 0)
		return ret;

	DEBUG_MSG("apds9900_init_client\n");

	data->enable = 0;

	return 0;
}

static void apds_9900_proximity_handler(struct apds9900_data *data, int cdata)
{
	struct i2c_client *client = data->client;
	int rdata;
	
	rdata = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS9900_PDATAL_REG);		
	data->prox_raw = rdata;
		
	DEBUG_MSG("prox sensor report data = %d\n",rdata);

	if (wake_lock_active(&data->wakelock))
		wake_unlock(&data->wakelock);
   	
	wake_lock_timeout(&data->wakelock, 2*HZ);

	if (rdata > data->prox_hth &&
	    (cdata < (75*(1024*(256-data->atime)))/100)) {
		apds9900_set_enable(client, 0);
		
		input_report_abs(data->input_dev_prox, ABS_DISTANCE, 0);/* NEAR */
		input_sync(data->input_dev_prox);
		apds9900_set_pilt(client, data->prox_lth);
		apds9900_set_piht(client, 1023);

		data->prox_stat = 0; /* NEAR */

		dev_info(&client->dev, "apds9900: proximity: NEAR\n");	
	}
	else if (rdata < data->prox_lth) {
		apds9900_set_enable(client,0);

		input_report_abs(data->input_dev_prox, ABS_DISTANCE, 1);/* FAR */
		input_sync(data->input_dev_prox);
		apds9900_set_pilt(client, 0);
		apds9900_set_piht(client, data->prox_hth);

		data->prox_stat = 1; /* FAR */

		dev_info(&client->dev, "apds9900: proximity: FAR 1\n");
	}
}

static void apds_9900_als_handler(struct apds9900_data *data)
{		
	struct i2c_client *client = data->client;
	int LTH = 0;
	int HTH = 0;
	int TH_GAP = 0;

	int APDS_IAC1 = 0;
	int APDS_IAC2 = 0;
	int APDS_IAC = 0;	
	int LUX;	
	int cdata;
	int irdata;

	// read reg for als
	cdata = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS9900_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS9900_IRDATAL_REG);

	// calculation LUX , please check datasheet from AVAGO
	APDS_IAC1 = cdata - (data->coeff_b*irdata)/100;
	APDS_IAC2 = (data->coeff_c*cdata - data->coeff_d*irdata)/100;
	APDS_IAC = APDS_IAC1>APDS_IAC2?APDS_IAC1:APDS_IAC2;
	LUX = (APDS_IAC*data->LPC)/100;	

	/* check prox under sunlight */
	if ((data->prox_stat == 0) &&
	    (cdata > (75*(1024*(256-data->atime)))/100)) {
		data->prox_raw = data->prox_hth;

		input_report_abs(data->input_dev_prox, ABS_DISTANCE, 1);/* FAR */
		input_sync(data->input_dev_prox);
		apds9900_set_pilt(client, 0);
		apds9900_set_piht(client, data->prox_hth);

		data->prox_stat = 1; /* FAR */

		dev_info(&client->dev,
				"apds9900: proximity: FAR 2\n");
	}

	//report vlaue to sensors.cpp

	//set min and max value
	LUX = LUX>5 ? LUX : 0;
	LUX = LUX<20000 ? LUX : 20000; //10240, modified by SH Kim Avago

	data->lux = LUX;

	/* x-axis raw acceleration */
	input_report_abs(data->input_dev_als, ABS_MISC, LUX);
 
	input_sync(data->input_dev_als);
	apds9900_set_enable(client, 0);

	// set threshold for als
	TH_GAP = cdata / 10;
	LTH = cdata  - TH_GAP;
	LTH = LTH > 0 ? LTH : 0;
	HTH = cdata + TH_GAP;
	HTH = HTH < 0xFFFF ? HTH : 0xFFFF;

	apds9900_set_ailt(client, LTH);
	apds9900_set_aiht(client, HTH);

	DEBUG_MSG("light sensor report lux =%d cdata = %d irdata = %d\n",LUX,cdata,irdata);
}

static void apds_9900_irq_work_func(struct work_struct *work)
{
   	struct apds9900_data *data =
		container_of(work, struct apds9900_data, dwork.work);
	struct i2c_client *client = data->client;

	int status;

	dev_dbg(&client->dev, "%s\n", __func__);

	status = i2c_smbus_read_byte_data(client,
			CMD_BYTE|APDS9900_STATUS_REG);

	if(status & APDS9900_STATUS_PINT) {
		int cdata = i2c_smbus_read_word_data(client,
				CMD_WORD|APDS9900_CDATAL_REG);
		apds_9900_proximity_handler(data, cdata);
	}

	if(status & APDS9900_STATUS_AINT) {
		apds_9900_als_handler(data);			
	}

	//DEBUG_MSG("apds_9900_irq_work_func status = %d\n",status);
	
	// ACK about interupt handling
	if(status & APDS9900_STATUS_PINT) {
		if(status & APDS9900_STATUS_AINT)
			apds9900_set_command(client,2);
		else
			apds9900_set_command(client,0);
	}
	else if(status & APDS9900_STATUS_AINT) {
		apds9900_set_command(client,1);
	}
	
	apds9900_set_control(client, 0x20/*PDRIVE | PDIODE | PGAIN | AGAIN*/);
	apds9900_set_enable(client, data->enable);

	if(data->enable_ps_sensor==0 && data->enable_als_sensor==0)
		 apds9900_set_enable(client, 0);
}

static void apds9900_reschedule_work(struct apds9900_data *data,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, delay);

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);
}

static irqreturn_t apds_9900_irq_handler(int irq, void *dev_id)						   
{
 	struct apds9900_data *data = dev_id;

	apds9900_reschedule_work(data, 0);
	return IRQ_HANDLED;
}	

/*
 * I2C init/probing/exit functions
 */
static struct i2c_driver apds9900_driver;

static int __devinit apds9900_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9900_platform_data *pdata;
	struct apds9900_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		err = -EINVAL;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9900_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->atime = pdata->atime;
	data->ptime = pdata->ptime;
	data->wtime = pdata->wtime;
	data->pers  = pdata->pers;
	data->ppcount = pdata->ppcount;
	data->pdrive = pdata->pdrive;
	data->pdiode = pdata->pdiode;
	data->pgain  = pdata->pgain;
	data->again  = pdata->again;
	data->prox_hth = pdata->threshhold_high;
	data->prox_lth = pdata->threshhold_low;
	data->coeff_b = pdata->coeff_b;
	data->coeff_c = pdata->coeff_c;
	data->coeff_d = pdata->coeff_d;
	data->GA = pdata->apds_ga;
	data->DF = pdata->apds_df;
	data->irq_gpio = pdata->irq_gpio;

	gpio_request(pdata->ldo_gpio,"apds9900_ldo");
	gpio_direction_output(pdata->ldo_gpio, 1);	// OUTPUT 
	gpio_set_value(pdata->ldo_gpio, 1);		// ON 
	
	data->client = client;
	i2c_set_clientdata(client, data);
	
	//apds_9900_i2c_client = client;

	data->enable = 0;	/* default mode is standard */
	dev_info(&client->dev, "enable = %s\n", data->enable ? "1" : "0");

	mutex_init(&data->update_lock);

	/* Initialize the APDS9900 chip */
	err = apds9900_init_client(client);
	if (err < 0) {
		dev_err(&client->dev, "%s: apds9900_init_client faied\n",
				__func__);
		goto exit_kfree;
	}

	err = gpio_request(pdata->irq_gpio,"apds9900_irq");
	if (err < 0) {
		dev_err(&client->dev, "%s(): gpio_request failed\n", __func__);
		 goto exit_kfree;
	}
	gpio_direction_input(pdata->irq_gpio);
	data->irq = OMAP_GPIO_IRQ(pdata->irq_gpio);
				
	err = request_irq(data->irq, apds_9900_irq_handler,
			IRQF_TRIGGER_FALLING,"apds9900", data);
	if (err < 0) {
		goto exit_request_irq_failed;
	}
	disable_irq(data->irq);

	INIT_DELAYED_WORK(&data->dwork, apds_9900_irq_work_func);
	data->input_dev_prox = input_allocate_device();
	if (!data->input_dev_prox) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s(): Failed to allocate input device for prox\n", __func__);
		goto exit_prox_input_dev_alloc_failed;
	}
	/* LGE_SJIT 2011-12-05 [dojip.kim@lge.com]
	 *  sysfs for new ICS Sensor HAL
	 */
	input_set_drvdata(data->input_dev_prox, data);
	set_bit(EV_ABS, data->input_dev_prox->evbit);
	input_set_abs_params(data->input_dev_prox, ABS_DISTANCE, 0, 360, 0, 0);

	data->input_dev_prox->name = "proximity";

	err = input_register_device(data->input_dev_prox);
	if (err) {
		dev_err(&client->dev, "%s(): Unable to register input device for proximity: %s\n", __func__, data->input_dev_prox->name);
		goto exit_prox_input_register_device_failed;
	}
	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als) {
 		err = -ENOMEM;
		dev_err(&client->dev, "%s(): Failed to allocate input device for als\n", __func__);
		goto exit_als_input_dev_alloc_failed;
 	}
	/* LGE_SJIT 2011-12-05 [dojip.kim@lge.com]
	 *  sysfs for new ICS Sensor HAL
	 */
	input_set_drvdata(data->input_dev_als, data);
	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_LED, data->input_dev_als->evbit);
	input_set_abs_params(data->input_dev_als, ABS_MISC, 0, 10240, 0, 0);
	input_set_abs_params(data->input_dev_als, LED_MISC, 0, 10240, 0, 0);
 
	data->input_dev_als->name = "als";
 
	err = input_register_device(data->input_dev_als);
 	if (err) {
		dev_err(&client->dev, "%s(): Unable to register input device for als: %s\n", __func__, data->input_dev_als->name);
		goto exit_als_input_register_device_failed;
 	}

	/* LGE_SJIT_S 2011-12-05 [dojip.kim@lge.com]
	 *  sysfs for new ICS Sensor HAL
	 */
	err = sysfs_create_group(&data->input_dev_als->dev.kobj,
			&light_attribute_group);
	if (err)
		goto exit_als_sysfs_create_group;

	err = sysfs_create_group(&data->input_dev_prox->dev.kobj,
			&proximity_attribute_group);
	if (err)
		goto exit_prox_sysfs_create_group;
	/* LGE_SJIT_E 2011-12-05 [dojip.kim@lge.com] */

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9900_attr_group);
	if (err)
		goto exit_sysfs_create_group;

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	wake_lock_init(&data->wakelock, WAKE_LOCK_SUSPEND, data->input_dev_prox->name);

	return 0;

exit_sysfs_create_group:
	sysfs_remove_group(&data->input_dev_prox->dev.kobj,
			&light_attribute_group);
exit_prox_sysfs_create_group:
	sysfs_remove_group(&data->input_dev_als->dev.kobj,
			&light_attribute_group);
exit_als_sysfs_create_group:
exit_als_input_register_device_failed:
	input_unregister_device(data->input_dev_als);
exit_als_input_dev_alloc_failed:
exit_prox_input_register_device_failed:
	input_unregister_device(data->input_dev_prox);
exit_prox_input_dev_alloc_failed:
	free_irq(data->irq, data);
exit_request_irq_failed:
	gpio_free(pdata->irq_gpio);
exit_kfree:
	kfree(data);
	data = NULL;
exit:
	dev_info(&client->dev, "probe error\n");
	return err;
}

static int __devexit apds9900_remove(struct i2c_client *client)
{
	struct apds9900_data *data = i2c_get_clientdata(client);

	DEBUG_MSG("apds9900_remove\n");

	/* Power down the device */
	apds9900_set_enable(client, 0);

	wake_lock_destroy(&data->wakelock);
	sysfs_remove_group(&client->dev.kobj, &apds9900_attr_group);

	/* LGE_SJIT_S 2011-12-05 [dojip.kim@lge.com]
	 * sysfs for new ICS Sensor HAL
	 */
	sysfs_remove_group(&data->input_dev_als->dev.kobj,
			&light_attribute_group);
	sysfs_remove_group(&data->input_dev_prox->dev.kobj,
			&light_attribute_group);
	/* LGE_SJIT_E 2011-12-05 [dojip.kim@lge.com] */

	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_prox);
	free_irq(data->irq, data);
	gpio_free(data->irq_gpio);

	kfree(data);
	data = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int apds9900_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9900_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "apds9900_suspend [%x]\n", data->enable);

	/* If irq occurs during suspend, abnormal work operation
	 * may cuase the kernel panic
	 */
	if (apds_9900_initialized) {
		disable_irq(data->irq);
		enable_irq_wake(data->irq);
	}

    flush_delayed_work_sync(&data->dwork);

	if (data->enable & APDS9900_ENABLE_PIEN) {
		int enable = APDS9900_ENABLE_PIEN | APDS9900_ENABLE_PEN | APDS9900_ENABLE_PON;
		apds9900_set_enable(client, enable);
	}
	else {
		apds9900_set_enable(client, 0);
	}

	return 0;
}

static int apds9900_resume(struct i2c_client *client)
{
	struct apds9900_data *data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "apds9900_resume [%x]\n", data->enable);

	if (apds_9900_initialized)
		enable_irq(data->irq);

	apds9900_reschedule_work(data, 0);

	return 0;
}

#else

#define apds9900_suspend	NULL
#define apds9900_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds9900_id[] = {
	{ "apds9900", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9900_id);

static struct i2c_driver apds9900_driver = {
	.driver = {
		.name	= APDS9900_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds9900_suspend,
	.resume	= apds9900_resume,
	.probe	= apds9900_probe,
	.remove	= __devexit_p(apds9900_remove),
	.id_table = apds9900_id,
};

static int __init apds9900_init(void)
{
	return i2c_add_driver(&apds9900_driver);
}

static void __exit apds9900_exit(void)
{
	i2c_del_driver(&apds9900_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com> / LGE Inc.");
MODULE_DESCRIPTION("APDS9900 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9900_init);
module_exit(apds9900_exit);
