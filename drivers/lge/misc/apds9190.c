/*
 *  apds9190.c - Linux kernel modules for proximity sensor
 *
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#include <mach/gpio.h>
#include <linux/lge/apds9190.h>

#define APDS9190_DRV_NAME	"apds9190"
#define DRIVER_VERSION		"1.0.5"

/* Change History 
 *
 * 1.0.1	Functions apds9190_show_rev(), apds9190_show_id() and apds9190_show_status()
 *			have missing CMD_BYTE in the i2c_smbus_read_byte_data(). APDS-9190 needs
 *			CMD_BYTE for i2c write/read byte transaction.
 *
 *
 * 1.0.2	Include PS switching threshold level when interrupt occurred
 *
 *
 * 1.0.3	Implemented ISR and delay_work, correct PS threshold storing
 *
 * 1.0.4	Added Input Report Event
 */

/*
 * Defines
 */

#define APDS9190_ENABLE_REG	0x00
#define APDS9190_ATIME_REG	0x01
#define APDS9190_PTIME_REG	0x02
#define APDS9190_WTIME_REG	0x03
#define APDS9190_AILTL_REG	0x04
#define APDS9190_AILTH_REG	0x05
#define APDS9190_AIHTL_REG	0x06
#define APDS9190_AIHTH_REG	0x07
#define APDS9190_PILTL_REG	0x08
#define APDS9190_PILTH_REG	0x09
#define APDS9190_PIHTL_REG	0x0A
#define APDS9190_PIHTH_REG	0x0B
#define APDS9190_PERS_REG	0x0C
#define APDS9190_CONFIG_REG	0x0D
#define APDS9190_PPCOUNT_REG	0x0E
#define APDS9190_CONTROL_REG	0x0F
#define APDS9190_REV_REG	0x11
#define APDS9190_ID_REG		0x12
#define APDS9190_STATUS_REG	0x13
#define APDS9190_CDATAL_REG	0x14
#define APDS9190_CDATAH_REG	0x15
#define APDS9190_IRDATAL_REG	0x16
#define APDS9190_IRDATAH_REG	0x17
#define APDS9190_PDATAL_REG	0x18
#define APDS9190_PDATAH_REG	0x19

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

#define APDS9190_ENABLE_PIEN    0x20
#define APDS9190_ENABLE_PEN     0x04
#define APDS9190_ENABLE_PON     0x01

// LGE_CHANGE_S [younglae.kim@lge.com] , add for Proximity sensor calibration
#define DEFAULT_CROSS_TALK		50
#if defined(CONFIG_MACH_LGE_U2_P760)
#define ADD_TO_CROSS_TALK       250
#else
#define ADD_TO_CROSS_TALK		300
#endif
#define SUB_FROM_PS_THRESHOLD	100

static int stored_cross_talk = 0;
// LGE_CHANGE_E [younglae.kim@lge.com]

#define APDS9190_SENSOR_DEBUG 0

/*
 * Structs
 */

struct apds9190_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work	dwork;	/* for PS interrupt */
	struct input_dev *input_dev_ps;

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

	int irq;
	int irq_gpio;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; /* always lower than ps_threshold */
	unsigned int ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */

// LGE_CHANGE_S [younglae.kim@lge.com] , add for Proximity sensor calibration
	unsigned int cross_talk;
    unsigned int avg_cross_talk;
// LGE_CHANGE_E [younglae.kim@lge.com]

    struct wake_lock wakelock;
};

/*
 * Global data
 */
static int apds9190_initialized = 0;

/*
 * Management functions
 */


// LGE_CHANGE_S [younglae.kim@lge.com] , add for Proximity sensor calibration
static int __init prox_cal_data(char *str)
{
	s32 value = simple_strtol(str, NULL, 10);
	stored_cross_talk = value;
	printk("ProximitySensor : cal_data = %d\n", stored_cross_talk);

	return 1;
}
__setup("prox_cal_data=", prox_cal_data);
// LGE_CHANGE_E [younglae.kim@lge.com]

static int apds9190_set_command(struct i2c_client *client, int command)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds9190_set_enable(struct i2c_client *client, int enable)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int apds9190_set_atime(struct i2c_client *client, int atime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds9190_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds9190_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds9190_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds9190_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds9190_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->pilt = threshold;

	return ret;
}

static int apds9190_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->piht = threshold;

	return ret;
}

static int apds9190_set_pers(struct i2c_client *client, int pers)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds9190_set_config(struct i2c_client *client, int config)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds9190_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds9190_set_control(struct i2c_client *client, int control)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	return ret;
}

static int apds_9190_init(struct i2c_client *client)
{
    int ret = 0;
    struct apds9190_data *data = i2c_get_clientdata(client);

    ret = apds9190_set_atime(client, data->atime);	// 100.64ms ALS integration time
	ret |= apds9190_set_ptime(client, data->ptime);	// 2.72ms Prox integration time
	ret |= apds9190_set_wtime(client, data->wtime);	// 2.72ms Wait time

	ret |= apds9190_set_ppcount(client, data->ppcount);	// 8-Pulse for proximity
	ret |= apds9190_set_config(client, 0);		// no long wait
	ret |= apds9190_set_control(client, 0x20);	// 100mA, IR-diode, 1X PGAIN, 1X AGAIN

	ret |= apds9190_set_pilt(client, 1023);	// to force first Near-to-Far interrupt
	ret |= apds9190_set_piht(client, 0);

	ret |= apds9190_set_ailt(client, 0);		// init threshold for als
	ret |= apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);

	ret |= apds9190_set_pers(client, data->pers);	// 2 consecutive Interrupt persistence

	if(ret < 0) {
		printk("%s: configuration set failed");
		return ret;
	}
	data->ps_detection = 1; // we are forcing Near-to-Far interrupt, so this is defaulted to 1

	printk("%s: configurations are set\n", __func__);

	return ret;
}

static void apds9190_change_ps_threshold(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	data->ps_data =	i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
	int irdata=0;
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);

    if (wake_lock_active(&data->wakelock))
        wake_unlock(&data->wakelock);

    wake_lock_timeout(&data->wakelock, 2*HZ);

	if ( (data->ps_data > data->pilt) && (data->ps_data >= data->piht) && (irdata != (100*(1024*(256-data->atime)))/100)) {
		/* far-to-NEAR */
		data->ps_detection = 1;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 0);/* FAR-to-NEAR detection */	
		input_sync(data->input_dev_ps);

		apds9190_set_pilt(client, data->ps_hysteresis_threshold);
		apds9190_set_piht(client, 1023);

/*		i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PILTL_REG, data->ps_hysteresis_threshold);
		i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PIHTL_REG, 1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;
*/
		printk("\n[ProximitySensor] far-to-NEAR\n");
	}
	else if ( (data->ps_data <= data->pilt) && (data->ps_data < data->piht) ) {
		/* near-to-FAR */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		apds9190_set_pilt(client, 0);
		apds9190_set_piht(client, data->ps_threshold);

		printk("\n[ProximitySensor] near-to-FAR\n");
	}
	else if ( (irdata == (100*(1024*(256-data->atime)))/100) && (data->ps_detection == 1) ) {
		/* under strong ambient light condition*/
		/* near-to-FAR */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		/*Keep the threshold NEAR condition to prevent the frequent Interrup under strong ambient light*/

		apds9190_set_pilt(client, data->ps_hysteresis_threshold);
		apds9190_set_piht(client, 1023);

		printk("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");		
		printk("\n[ProximitySensor] near-to-FAR\n");
	}
	else if ( (data->pilt == 1023) && (data->piht == 0) )
	{
		/* this is the first near-to-far forced interrupt */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, 1);/* NEAR-to-FAR detection */	
		input_sync(data->input_dev_ps);

		apds9190_set_pilt(client, 0);
		apds9190_set_piht(client, data->ps_threshold);

		printk("[ProximitySensor] near-to-FAR\n");
	}
}

static void apds9190_reschedule_work(struct apds9190_data *data,
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

/* PS interrupt routine */
static void apds9190_work_handler(struct work_struct *work)
{
	struct apds9190_data *data = container_of(work, struct apds9190_data, dwork.work);
	struct i2c_client *client=data->client;
	int	status=0;
	int cdata=0;
	int irdata=0;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_STATUS_REG);

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, 1);	/* disable 9190's ADC first */

	printk("status = %x\n", status);

	if((status & data->enable & 0x30) == 0x30) {
		/* both ALS and PS are interrupted */

		/* Change ALS threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
		if(cdata == 100*(1024*(256-data->atime))/100) {
			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);		
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);		
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold for normal mode\n");
		}

		/* check if this is triggered by strong ambient light  */		
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if(irdata != (100*(1024*(256-data->atime)))/100)
			apds9190_change_ps_threshold(client);
		else {
			if(data->ps_detection == 1) {
				apds9190_change_ps_threshold(client);			
				
				printk("* Triggered by background ambient noise\n");	
				printk("\n[ProximitySensor] near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				printk("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				printk("* Triggered by background ambient noise\n");
				printk("\n[ProximitySensor] maintain FAR \n");
			}
		}
		apds9190_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */
	}
	else if((status & data->enable & 0x20) == 0x20) {
		/* only PS is interrupted */

		/* Change ALS threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
		if(cdata == 100*(1024*(256-data->atime))/100) {
			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);		
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);		
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold for normal mode\n");
		}
		
		/* check if this is triggered by strong ambient light */
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if(irdata != (100*(1024*(256-data->atime)))/100)
			apds9190_change_ps_threshold(client);
		else {
			if(data->ps_detection == 1) {
				apds9190_change_ps_threshold(client);			
		        printk("* Triggered by background ambient noise\n");	
				printk("\n[ProximitySensor] near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				printk("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				printk("* Triggered by background ambient noise\n");
				printk("\n[ProximitySensor] maintain FAR \n");
			}
		}
		apds9190_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	}
	else if((status & data->enable & 0x10) == 0x10) {
		/* only ALS is interrupted */

		/* Change ALS Threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);		
		if(cdata == 100*(1024*(256-data->atime))/100) {
			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);		
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);		
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			printk("* Set ALS Theshold for normal mode\n");
		}
		
		/* check if this is triggered by the strong ambient light */
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if(irdata != (100*(1024*(256-data->atime)))/100)
			apds9190_change_ps_threshold(client);
		else {
			if(data->ps_detection == 1) {
				apds9190_change_ps_threshold(client);			
				printk("* Triggered by background ambient noise\n");
				printk("\n[ProximitySensor] near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				printk("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				printk("* Triggered by background ambient noise\n");
				printk("\n[ProximitySensor] maintain FAR \n");
			}
		}
		apds9190_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	}

#if APDS9190_SENSOR_DEBUG
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
	cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
	ailt = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_AILTL_REG);
	aiht = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_AIHTL_REG);
	pilt = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PILTL_REG);
	piht = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PIHTL_REG);

	printk("pdata = %d\n", pdata);
	printk("cdata = %d\n", cdata);
	printk("irdata = %d\n", irdata);
	
	printk("ailt = %d\n", ailt);
	printk("aiht = %d\n", aiht);
	printk("pilt = %d\n", pilt);
	printk("piht = %d\n", piht);

	printk("--------------------\n");
#endif

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, data->enable);	
}

/* assume this is ISR */
static irqreturn_t apds9190_interrupt(int vec, void *info)
{
	struct apds9190_data *data = info;

	apds9190_reschedule_work(data, 0);	

	return IRQ_HANDLED;
}

/*
 * SysFS support
 */

static ssize_t apds9190_show_atime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->atime);
}

static ssize_t apds9190_store_atime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_atime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(atime,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_atime, apds9190_store_atime);

static ssize_t apds9190_show_ptime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ptime);
}

static ssize_t apds9190_store_ptime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_ptime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ptime,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ptime, apds9190_store_ptime);

static ssize_t apds9190_show_wtime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->wtime);
}

static ssize_t apds9190_store_wtime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_wtime(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(wtime,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_wtime, apds9190_store_wtime);


static ssize_t apds9190_show_pers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->pers);
}

static ssize_t apds9190_store_pers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_pers(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(pers,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_pers, apds9190_store_pers);


static ssize_t apds9190_show_ppcount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ppcount);
}

static ssize_t apds9190_store_ppcount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_ppcount(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ppcount,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ppcount, apds9190_store_ppcount);

static ssize_t apds9190_show_control(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->control);
}

static ssize_t apds9190_store_control(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	ret = apds9190_set_control(client, val);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(control,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_control, apds9190_store_control);

static ssize_t apds9190_show_ps_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_threshold);
}

static ssize_t apds9190_store_ps_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_th)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_threshold = val;

	return ps_th;
}

static DEVICE_ATTR(ps_threshold,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ps_threshold, apds9190_store_ps_threshold);

static ssize_t apds9190_show_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_hysteresis_threshold);
}

static ssize_t apds9190_store_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_hy_th)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_hysteresis_threshold = val;

	return ps_hy_th;
}

static DEVICE_ATTR(ps_hysteresis,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ps_hysteresis_threshold, apds9190_store_ps_hysteresis_threshold);

static ssize_t apds9190_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9190_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	static bool first = true;
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
    struct apds9190_platform_data *pdata;
	unsigned long val = simple_strtoul(buf, NULL, 10);

    pdata = client->dev.platform_data;
    if(pdata == NULL) {
        return -EINVAL;
    }

	printk("%s: enable proximity sensor (%ld)\n", __func__, val);
	
	if ((val != 0) && (val != 1)) {
		printk("%s: store unvalid value=%ld\n", __func__, val);
		return count;
	}

	if( !apds9190_initialized  ) {
        ret = apds_9190_init(client);
        if(ret < 0) {
            printk("%s: apds_9190_init failed\n", __func__);
            return ret;
        }
        apds9190_initialized = 1;
        enable_irq(data->irq);
	}

	disable_irq(data->irq);
	
	if(val == 1) {
		//turn on p sensor
		if (data->enable_ps_sensor==0) {
			data->enable_ps_sensor= 1;
			apds9190_set_enable(client, 0x3F);	 /* enable PS and ALS interrupt and Wait */
		}
	}
	else {
		//turn off p sensor - kk 25 Apr 2011 we can't turn off the entire sensor, the light sensor may be needed by HAL
		data->enable_ps_sensor = 0;
		apds9190_set_enable(client, 0);
	}

	enable_irq(data->irq);
	
	return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IRUGO | S_IWUSR | S_IWGRP,
				   apds9190_show_enable_ps_sensor, apds9190_store_enable_ps_sensor);


static ssize_t apds9190_show_id(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	int id;

	mutex_lock(&data->update_lock);
	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_ID_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", id);
}
static DEVICE_ATTR(id, S_IWUSR | S_IRUGO, apds9190_show_id, NULL);

static ssize_t apds9190_show_pdata(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	int pdata;

	mutex_lock(&data->update_lock);
	pdata = i2c_smbus_read_word_data(client, CMD_BYTE|APDS9190_PDATAL_REG);
	mutex_unlock(&data->update_lock);

	if(pdata < 0) {
		printk("%s: i2c_error\n", __func__);
		pdata = -1;
	}

	return sprintf(buf, "%d\n", pdata);
}
static DEVICE_ATTR(pdata, S_IRUGO, apds9190_show_pdata, NULL);

static ssize_t apds9190_show_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	int status;

	mutex_lock(&data->update_lock);
	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_STATUS_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", status);
}
static DEVICE_ATTR(status, S_IRUSR | S_IRGRP, apds9190_show_status, NULL);


// LGE_CHANGE_S [younglae.kim@lge.com] , add for Proximity sensor calibration
static int apds9190_Run_Cross_talk_Calibration(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int pdata[20], total_pdata = 0;
	int i, j, temp;
	bool isCal = false;

	printk("===================================================\n%s: START proximity sensor calibration\n", __func__);

	// proximity enable & wait
	apds9190_set_enable(client, 0x0D);

RE_CALIBRATION:

	// read 20 data to calculate the average pdata at no-object state
	for(i=0; i<20; i++) {
		pdata[i] = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
		mdelay(5);
	}

	// pdata sorting
	for(i=0; i<19; i++) {
		for(j=i+1; j<20; j++) {
			if(pdata[i] > pdata[j]) {
				temp = pdata[i];
				pdata[i] = pdata[j];
				pdata[j] = temp;
			}
		}
	}

	// calculate the cross-talk using central 10 data
	for(i=5; i<15; i++) {
        printk("pdata = %d\n", pdata[i]);
		total_pdata += pdata[i];
    }

	// store cross_talk value into the apds9190_data structure
	data->cross_talk = total_pdata / 10;

    /* LGE_CHANGE [younglae.kim@lge.com] 2012-07-05, original average cross-talk
     * this value is used at Hidden Menu to check if the calibration is pass or fail
    */
    data->avg_cross_talk = data->cross_talk;

	// check if the calibrated cross_talk data is valid or not
	if(data->cross_talk > 720) {
		printk("%s: invalid calibrated data\n", __func__);

		if(!isCal) {
			printk("%s: RE_CALIBRATION start\n", __func__);
			isCal = true;
			total_pdata = 0;
			goto RE_CALIBRATION;
		} else {
			printk("%s: CALIBRATION FAIL -> cross_talk is set to DEFAULT\n", __func__);
			data->cross_talk = DEFAULT_CROSS_TALK;
		}
	}

	// proximity disable
	apds9190_set_enable(client, 0x00);

	printk("%s: total_pdata = %d & cross_talk = %d\n%s: FINISH proximity sensor calibration\n===================================================\n",
            __func__, total_pdata, data->cross_talk, __func__);

	return data->cross_talk;
}

static ssize_t apds9190_show_run_calibration(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->avg_cross_talk);
}

static ssize_t apds9190_store_run_calibration(struct device *dev,
			struct device_attribute *attr, char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	// start calibration
	apds9190_Run_Cross_talk_Calibration(client);

	// set threshold for near/far status
	data->ps_threshold = data->cross_talk + ADD_TO_CROSS_TALK;
	data->ps_hysteresis_threshold = data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	printk("%s: [piht][pilt][c_t] = [%d][%d][%d]\n",
				__func__, data->ps_threshold, data->ps_hysteresis_threshold, data->cross_talk);

	return count;
}
static DEVICE_ATTR(run_calibration,  S_IRUGO | S_IWUSR| S_IWGRP,
		   apds9190_show_run_calibration, apds9190_store_run_calibration);
// LGE_CHANGE_E [younglae.kim@lge.com]

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-07-18, add for ALC/Proximity Test
static ssize_t apds9190_show_default_crosstalk(struct device *dev,
            struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", DEFAULT_CROSS_TALK);
}

static ssize_t apds9190_store_default_crosstalk(struct device *dev,
			struct device_attribute *attr, char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

    data->ps_threshold = DEFAULT_CROSS_TALK + ADD_TO_CROSS_TALK;
	data->ps_hysteresis_threshold = data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	printk("%s: [piht][pilt][c_t] = [%d][%d][%d]\n",
				__func__, data->ps_threshold, data->ps_hysteresis_threshold, data->cross_talk);

    return count;
}
static DEVICE_ATTR(default_crosstalk, S_IRUGO | S_IWUSR | S_IWGRP,
            apds9190_show_default_crosstalk, apds9190_store_default_crosstalk);
// LGE_CHANGE_E [younglae.kim@lge.com]

static struct attribute *apds9190_attributes[] = {
	
	&dev_attr_atime.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_pers.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_control.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_ps_hysteresis.attr,
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_id.attr,
	&dev_attr_pdata.attr,
	&dev_attr_status.attr,
	&dev_attr_run_calibration.attr,
    &dev_attr_default_crosstalk.attr,
	NULL
};

static const struct attribute_group apds9190_attr_group = {
	.attrs = apds9190_attributes,
};

/*
 * Initialization function
 */
static int apds9190_init_client(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds9190_set_enable(client, 0);

	if (err < 0)
		return err;
	
	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_ID_REG);
	if (id == 0x29) {
		printk("initialize APDS-9190 client\n");
	}
	else {
		printk("NOT APDS-9190\n");
		return -EIO;
	}

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds9190_driver;
static int __devinit apds9190_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct apds9190_platform_data *pdata;
	struct apds9190_data *data;
	int err = 0, rdata = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}
    
    pdata = client->dev.platform_data;
    if(pdata == NULL) {
        err = -EINVAL;
        goto exit;
    }

	data = kzalloc(sizeof(struct apds9190_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

    data->atime = pdata->atime;
    data->ptime = pdata->ptime;
    data->wtime = pdata->wtime;
    data->pers = pdata->pers;
    data->ppcount = 0x0F;

// LGE_CHANGE_S [younglae.kim@lge.com] , add for Proximity sensor calibration
	if(stored_cross_talk >= 0 && stored_cross_talk <= 720)
		data->cross_talk = stored_cross_talk;
	else
		data->cross_talk = DEFAULT_CROSS_TALK;

    data->avg_cross_talk = stored_cross_talk;

	data->ps_threshold = data->cross_talk + ADD_TO_CROSS_TALK;
	data->ps_hysteresis_threshold = data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	printk("%s(): stored_cross_talk = %d, data->cross_talk = %d\n%s(): %d < ps_data < %d\n",
						__func__, stored_cross_talk, data->cross_talk,
						__func__, data->ps_hysteresis_threshold, data->ps_threshold);

// LGE_CHANGE_E [younglae.kim@lge.com]
    data->irq_gpio = pdata->irq_gpio;

    gpio_request(pdata->ldo_gpio, "apds9190_ldo");
    gpio_direction_output(pdata->ldo_gpio, 1);
    gpio_set_value(pdata->ldo_gpio, 1);

// LGE_CHANGE_ need delay >4.5ms between VDD up & I2C init [jongho3.lee@lge.com]
	mdelay(5);

	data->client = client;
	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_detection = 0;	/* default to no detection */
	data->enable_ps_sensor = 0;	// default to 0
	
	rdata = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_ID_REG);
	printk("%s(): apds9190 ID = 0x%2x\n", __func__, rdata);
	printk("%s(): enable = %x\n", __func__, data->enable);

	mutex_init(&data->update_lock);

	/* Initialize the APDS9190 chip */
	err = apds9190_init_client(client);
	if (err) {
        printk("%s(): apds9190_init_client failed\n", __func__);
		goto exit_kfree;
    }

    err = gpio_request(pdata->irq_gpio, "apds9190_irq");
    if (err) {
        printk("%s(): gpio_request_failed\n", __func__);

        goto exit_kfree;
    }
    gpio_direction_input(pdata->irq_gpio);
    data->irq = OMAP_GPIO_IRQ(pdata->irq_gpio);

    err = request_irq(data->irq, apds9190_interrupt,
            IRQF_TRIGGER_FALLING, APDS9190_DRV_NAME, data);
    if (err) {
        goto exit_request_irq_failed;
    }
    disable_irq(data->irq);

	INIT_DELAYED_WORK(&data->dwork, apds9190_work_handler);
	/* Register to Input Device */
	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device proximity\n", __func__);
		goto exit_input_dev_ps_alloc_failed;
	}
	
    input_set_drvdata(data->input_dev_ps, data);
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_ps->name = "proximity";

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("Unable to register input device proximity: %s\n",
		       data->input_dev_ps->name);
		goto exit_input_dev_ps_register_device_failed;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9190_attr_group);
	if (err)
		goto exit_sysfs_create_group;

	printk("%s support ver. %s enabled\n", __func__, DRIVER_VERSION);

    wake_lock_init(&data->wakelock, WAKE_LOCK_SUSPEND, data->input_dev_ps->name);

	printk("-----------------------------\n");
	printk("apds9190 driver initialized!!\n");
	printk("-----------------------------\n");
	return 0;

exit_sysfs_create_group:
	input_unregister_device(data->input_dev_ps);
exit_input_dev_ps_register_device_failed:
	input_free_device(data->input_dev_ps);
exit_input_dev_ps_alloc_failed:
	free_irq(data->irq, data);
exit_request_irq_failed:
	gpio_free(pdata->irq_gpio);
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit apds9190_remove(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	
	input_unregister_device(data->input_dev_ps);
	
	input_free_device(data->input_dev_ps);

	free_irq(data->irq, client);

	sysfs_remove_group(&client->dev.kobj, &apds9190_attr_group);

	/* Power down the device */
	apds9190_set_enable(client, 0);

    wake_lock_destroy(&data->wakelock);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM

static int apds9190_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct apds9190_data *data = i2c_get_clientdata(client);

    printk("%s [%x]\n", __func__, data->enable);

    if (apds9190_initialized) {
        disable_irq(data->irq);
        enable_irq_wake(data->irq);
    }

    flush_delayed_work_sync(&data->dwork);

    if (data->enable & APDS9190_ENABLE_PIEN) {
        int enable = APDS9190_ENABLE_PIEN | APDS9190_ENABLE_PEN | APDS9190_ENABLE_PON;
        apds9190_set_enable(client, enable);
    } else {
        apds9190_set_enable(client, 0);
    }

	return 0;
}

static int apds9190_resume(struct i2c_client *client)
{
    struct apds9190_data *data = i2c_get_clientdata(client);

    printk("%s [%x]\n", __func__, data->enable);

    if (apds9190_initialized)
        enable_irq(data->irq);

    apds9190_reschedule_work(data, 0);

	return 0;
}

#else

#define apds9190_suspend	NULL
#define apds9190_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds9190_id[] = {
	{ "apds9190", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9190_id);

static struct i2c_driver apds9190_driver = {
	.driver = {
		.name	= APDS9190_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds9190_suspend,
	.resume	= apds9190_resume,
	.probe	= apds9190_probe,
	.remove	= __devexit_p(apds9190_remove),
	.id_table = apds9190_id,
};

static int __init apds9190_init(void)
{
	printk("--------------------\n");
	printk("apds9190 driver init\n");
	printk("--------------------\n");
	return i2c_add_driver(&apds9190_driver);
}

static void __exit apds9190_exit(void)
{
	i2c_del_driver(&apds9190_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9190 proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9190_init);
module_exit(apds9190_exit);

