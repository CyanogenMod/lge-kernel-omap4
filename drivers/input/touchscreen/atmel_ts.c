/*
 * AT42QT602240/ATMXT224 Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * 2010 Modified by LG Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>

#ifndef QT602240_USE_FIRMWARE_CLASS
#include "atmel_firmware.h"
#endif

int last_x;
int last_y;

#define QT_EDGE_COMPENSATION_Y 25
enum {
	QT_TOUCHKEY_MENU = 0,
	QT_TOUCHKEY_HOME,
	QT_TOUCHKEY_BACK,
	QT_TOUCHKEY_SEARCH,
	QT_TOUCHKEY_MAX
};

typedef struct {
	int16_t x1;         /* x start        */
	int16_t y1;         /* y start        */
	int16_t x2;         /* x end          */
	int16_t y2;         /* y end          */
	int16_t amp_thrs;   /* amp threshold  */
	int16_t size_thrs;  /* size threshold */
	int16_t key;        /* linux key type */
	int16_t id;         /* pressed id     */
} touchkey_area_info_t;

static touchkey_area_info_t touchkey_area[QT_TOUCHKEY_MAX] = {
/*     x1,   y1,   x2,   y2, amp, size,        key,   id */
	{  50,  810,   60,  850, 500,   50,   KEY_MENU, 0xFF },
	{ 150, 8100,  196,  850, 500,   50,   KEY_HOME, 0xFF },
	{ 280,  810,  330,  850, 500,   50,   KEY_BACK, 0xFF },
	{ 420,  810,  426,  850, 500,   50, KEY_SEARCH, 0xFF }
};

#define PRESSED     1
#define RELEASED    0

enum {
	NO_KEY_TOUCHED,
	KEY1_TOUCHED,
	KEY2_TOUCHED,
	KEY3_TOUCHED,
	KEY4_TOUCHED,
	MAX_KEY_TOUCH
};

static u8 firmware_status;
static u8 esd_check;
static uint8_t ts_cal_check;

struct ts_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct ts_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct ts_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct ts_finger {
	int status;
	int x;
	int y;
	int area;
};

/* Each client has this additional data */
struct ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct qt602240_platform_data *pdata;
	struct ts_object *object_table;
	struct ts_info info;
	struct ts_finger finger[MAX_FINGER];
	unsigned int irq;
	struct work_struct work;
	struct early_suspend	early_suspend;
};

static struct workqueue_struct *atmel_wq;

static void ts_power_up(struct ts_data *data);
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *);
static void ts_late_resume(struct early_suspend *);
#endif

static bool ts_object_readable(unsigned int type)
{
	switch (type) {
	case GEN_MESSAGE:
	case GEN_COMMAND:
	case GEN_POWER:
	case GEN_ACQUIRE:
	case TOUCH_MULTI:
	case TOUCH_KEYARRAY:
	case TOUCH_PROXIMITY:
	case PROCI_GRIPFACE:
	case PROCG_NOISE:
	case PROCI_ONETOUCH:
	case PROCI_TWOTOUCH:
	case SPT_COMMSCONFIG:
	case SPT_GPIOPWM:
	case SPT_SELFTEST:
	case SPT_CTECONFIG:
	case SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool ts_object_writable(unsigned int type)
{
	switch (type) {
	case GEN_COMMAND:
	case GEN_POWER:
	case GEN_ACQUIRE:
	case TOUCH_MULTI:
	case TOUCH_KEYARRAY:
	case TOUCH_PROXIMITY:
	case PROCI_GRIPFACE:
	case PROCG_NOISE:
	case PROCI_ONETOUCH:
	case SPT_GPIOPWM:
	case SPT_SELFTEST:
	case SPT_CTECONFIG:
		return true;
	default:
		return false;
	}
}

static void ts_dump_message(struct device *dev,
				  struct ts_message *message)
{
	printk(KERN_INFO"reportid:\t0x%x\n", message->reportid);
	printk(KERN_INFO"message1:\t0x%x\n", message->message[0]);
	printk(KERN_INFO"message2:\t0x%x\n", message->message[1]);
	printk(KERN_INFO"message3:\t0x%x\n", message->message[2]);
	printk(KERN_INFO"message4:\t0x%x\n", message->message[3]);
	printk(KERN_INFO"message5:\t0x%x\n", message->message[4]);
	printk(KERN_INFO"message6:\t0x%x\n", message->message[5]);
	printk(KERN_INFO"message7:\t0x%x\n", message->message[6]);
	printk(KERN_INFO"checksum:\t0x%x\n", message->checksum);
}

static int ts_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = UNLOCK_CMD_LSB;
	buf[1] = UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ts_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __ts_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ts_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __ts_read_reg(client, reg, 1, val);
}

static int ts_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ts_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __ts_read_reg(client, reg, OBJECT_SIZE,
				   object_buf);
}

static struct ts_object *ts_get_object(struct ts_data *data, u8 type)
{
	struct ts_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type (%d)\n", type);
	return NULL;
}

static int ts_read_message(struct ts_data *data, struct ts_message *message)
{
	struct ts_object *object;

	u16 reg;

	object = ts_get_object(data, GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __ts_read_reg(data->client, reg,
			sizeof(struct ts_message), message);
}

static int ts_read_object(struct ts_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct ts_object *object;
	u16 reg;

	object = ts_get_object(data, type);

	if (!object)
		return -EINVAL;

	reg = object->start_address;

	return __ts_read_reg(data->client, reg + offset, 1, val);
}

static int ts_read_diagnostic_object(struct ts_data *data,
				u8 type, u16 len, u8 *val)
{
	struct ts_object *object;
	u16 reg;

	object = ts_get_object(data, type);

	if (!object)
		return -EINVAL;

	reg = object->start_address;

	return __ts_read_reg(data->client, reg, len, val);
}

static int ts_write_object(struct ts_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct ts_object *object;
	u16 reg;

	object = ts_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;

	return ts_write_reg(data->client, reg + offset, val);
}

static void reset_chip(struct ts_data *data)
{
	ts_write_object(data, GEN_COMMAND,
			COMMAND_RESET, 1);

	msleep(RESET_TIME);
}

static void ts_calibrate_chip(struct ts_data *data)
{
	printk(KERN_INFO"qt602240_calibrate_chip");
	if (ts_cal_check == 0) {
		/* change settings to zero until calibration good */
		ts_write_object(data, GEN_ACQUIRE, ACQUIRE_ATCHCALST, 0);
		ts_write_object(data, GEN_ACQUIRE, ACQUIRE_ATCHCALSTHR, 0);
	}

	/* send calibration command to the chip */
	ts_write_object(data, GEN_COMMAND, COMMAND_CALIBRATE, 1);
	ts_cal_check = 1;
}

static unsigned int qt_time_point;
static unsigned int qt_time_diff;
static unsigned int qt_timer_state;
static int ts_check_abs_time(void)
{
	qt_time_diff = 0;

	if (!qt_time_point)
		return 0;

	qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
	if (qt_time_diff > 0)
		return 1;
	else
		return 0;
}

void ts_check_chip_calibration(struct ts_data *data)
{
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint8_t tch_ch = 0, atch_ch = 0;
	uint8_t check_mask;
	uint8_t i;
	uint8_t j;
	uint8_t x_line_limit;

	/* we have had the first touchscreen or face suppression message
	 * after a calibration - check the sensor state and try to confirm if
	 * cal was good or bad */

	/* get touch flags from the chip using the diagnostic object */
	/* write command to command processor to get touch flags
	 *  - 0xF3 Command required to do this */

	ts_write_object(data, GEN_COMMAND, COMMAND_DIAGNOSTIC, data_byte);

	msleep(10);

	/* read touch flags from the diagnostic object
	 *  - clear buffer so the while loop can run first time */
	memset(data_buffer , 0xFF, sizeof(data_buffer));

	/* wait for diagnostic object to update */
	while (!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))) {
		/* wait for data to be valid  */
		if (try_ctr > 10) {
			/* Failed! */
			printk(KERN_INFO"[QT602240] Diagnostic Data did not update!!\n");
			qt_timer_state = 0;
			break;
		}
		msleep(2);
		try_ctr++; /* timeout counter */
		ts_read_diagnostic_object(data, DEBUG_DIAGNOSTIC, 2, data_buffer);
	}

	/* data is ready - read the detection flags */
	/* data array is 20 x 16 bits for each set of flags, 2 byte header,
	 * 40 bytes for touch flags 40 bytes for antitouch flags*/
	ts_read_diagnostic_object(data, DEBUG_DIAGNOSTIC, 82, data_buffer);

	/* count up the channels/bits if we recived the data properly */
	if ((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)) {
		/* mode 0 : 16 x line, mode 1 : 17 etc etc upto mode 4.*/
		/* Victor(Mode 3) : 19(x) * 11(y) */
		x_line_limit = data->pdata->x_line;

		if (x_line_limit > 20) {
			/* hard limit at 20 so we don't over-index the array */
			x_line_limit = 20;
		}

		/* double the limit as the array is in bytes not words */
		x_line_limit = x_line_limit << 1;

		/* count the channels and print the flags to the log */
		/* check X lines - data is in words so increment 2 at a time */
		for (i = 0; i < x_line_limit; i += 2) {
			/* print the flags to the log
			 *  - only really needed for debugging */

			/* count how many bits set for this row */
			for (j = 0; j < 8; j++) {
				/* create a bit mask to check against */
				check_mask = 1 << j;

				/* check detect flags */
				if (data_buffer[2+i] & check_mask)
					tch_ch++;
				if (data_buffer[3+i] & check_mask)
					tch_ch++;

				/* check anti-detect flags */
				if (data_buffer[42+i] & check_mask)
					atch_ch++;
				if (data_buffer[43+i] & check_mask)
					atch_ch++;
			}
		}


		/* print how many channels we counted */
		/*
			printk(KERN_INFO"[QT602240] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);
		*/

		/* send page up command so we can detect when data updates next time,
		 * page byte will sit at 1 until we next send F3 command */
		data_byte = 0x01;
		ts_write_object(data, GEN_COMMAND, COMMAND_DIAGNOSTIC, data_byte);

		/* process counters and decide if we must re-calibrate or if cal was good */
		if ((tch_ch > 0) && (atch_ch == 0)) {
			/* cal was good - don't need to check any more */
			if (!ts_check_abs_time())
				qt_time_diff = 301;

			if (qt_timer_state == 1) {
				if (qt_time_diff > 300) {
					printk(KERN_INFO"[QT602240] calibration was good\n");
					ts_cal_check = 0;
					qt_timer_state = 0;
					qt_time_point = jiffies_to_msecs(jiffies);

					/* Write normal acquisition config back to the chip. */
					ts_write_object(data, GEN_ACQUIRE, ACQUIRE_ATCHCALST, 0x09);
					ts_write_object(data, GEN_ACQUIRE, ACQUIRE_ATCHCALSTHR, 0x23);
			    } else
				   ts_cal_check = 1;
			} else {
				qt_timer_state = 1;
				qt_time_point = jiffies_to_msecs(jiffies);
				ts_cal_check = 1;
			}
		} else if (atch_ch >= 6) {
			printk(KERN_DEBUG "[TSP] calibration was bad\n");
			/* cal was bad - must recalibrate and check afterwards */
			ts_calibrate_chip(data);
			qt_timer_state = 0;
			qt_time_point = jiffies_to_msecs(jiffies);
		} else {
			/*
				printk(KERN_INFO"[QT602240] calibration was not decided yet\n");
			*/
			/* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
			ts_cal_check = 1;
			/* Reset the 100ms timer */
			qt_timer_state = 0;
			qt_time_point = jiffies_to_msecs(jiffies);
		}
	}
}

static void ts_input_report(struct ts_data *data, int single_id)
{
	struct ts_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;

	int finger_num;
	int id, i;

	finger_num = 0;


    for (id = 0; id < MAX_FINGER; id++) {
		if ((finger[id].y > touchkey_area[0].y1) && (finger[id].y < touchkey_area[0].y2)) {
			for (i = 0; i < QT_TOUCHKEY_MAX; i++) {
				if ((finger[id].x > touchkey_area[i].x1) && (finger[id].x < touchkey_area[i].x2)) {
					if (finger[id].status != RELEASE)
						input_report_key(input_dev, touchkey_area[i].key, 1);
					else
						input_report_key(input_dev, touchkey_area[i].key, 0);
					input_sync(input_dev);
				}
			}
		}
	}

	if (finger[single_id].status == RELEASE) {
		for (id = 0; id < MAX_FINGER; id++) {
			if (id != single_id && finger[id].status != 0)
				finger_num++;
		}

		if (finger_num == 0) {
			finger[single_id].status = 0;
			input_mt_sync(input_dev);
			input_sync(input_dev);
			return;
		}
	}

	for (id = 0; id < MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;
		if (finger[id].status == RELEASE) {
			finger[id].status = 0;
			continue;
		}

		if (finger[id].area > MAX_WIDTH)
			finger[id].area = MAX_WIDTH;

		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, MAX_WIDTH);
		input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);

		input_mt_sync(input_dev);
	}
	input_sync(input_dev);
}

static void ts_input_touchevent(struct ts_data *data,
				      struct ts_message *message, int id)
{
	struct ts_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;

	/* Check the touch is present on the screen */
	if (!(status & DETECT)) {
		if (status & RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);
			printk("[%d] released\n", id);
			finger[id].status = RELEASE;
			ts_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (PRESS | MOVE)))
		return;

	x = (message->message[1] << 2) | ((message->message[3] & ~0x3f) >> 6);
	y = (message->message[2] << 2) | ((message->message[3] & ~0xf3) >> 2);
	area = message->message[4];

	finger[id].status = status & MOVE ?
				MOVE : PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;

	last_x = finger[id].x;
	last_y = finger[id].y;

	ts_input_report(data, id);
}

static void ts_work_func(struct work_struct *work)
{
	struct ts_data *data = container_of(work, struct ts_data, work);
	struct ts_message message;
	struct ts_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	uint8_t touch_message_flag = 0;

	if (ts_read_message(data, &message)) {
		dev_err(dev, "Failed to read message\n");
		enable_irq(data->client->irq);
		return;
	}

	reportid = message.reportid;

	/* whether reportid is thing of QT602240_TOUCH_MULTI */
	object = ts_get_object(data, TOUCH_MULTI);
	if (!object)
		goto end;

	max_reportid = object->max_reportid;
	min_reportid = max_reportid - object->num_report_ids + 1;
	id = reportid - min_reportid;

	if (reportid != 0xff)
		esd_check = 0;

	if (reportid >= min_reportid && reportid <= max_reportid) {
		ts_input_touchevent(data, &message, id);

		/* PRESS or MOVE */
		if ((message.message[0] & DETECT))
			touch_message_flag = 1;
	} else if (reportid != 0xff) {
		ts_dump_message(dev, &message);
	}

	esd_check++;
end:
	if (esd_check >= 5) {
		printk(KERN_INFO"ESD!! Touch Reset!!!\n");
		reset_chip(data);
	}

	if (touch_message_flag && (ts_cal_check))
	    ts_check_chip_calibration(data);
	enable_irq(data->client->irq);
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	struct ts_data *data = dev_id;

	disable_irq_nosync(data->client->irq);
	queue_work(atmel_wq, &data->work);

	return IRQ_HANDLED;
}

static int ts_check_reg_init(struct ts_data *data)
{
	struct ts_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j;
	u8 version = data->info.version;
	u8 *init_vals;

	switch (version) {
	case VER_16:
		init_vals = (u8 *)init_vals_ver_16;
		break;
	default:
		dev_err(dev, "Firmware version %d doesn't support\n", version);
		return -EINVAL;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!ts_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++)
			ts_write_object(data, object->type, j,
					init_vals[index + j]);

		index += object->size + 1;
	}

	return 0;
}

static void ts_handle_pdata(struct ts_data *data)
{
	const struct qt602240_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen voltage */
	if (pdata->voltage) {
		if (pdata->voltage < VOLTAGE_DEFAULT) {
			voltage = (VOLTAGE_DEFAULT - pdata->voltage) /
				VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - VOLTAGE_DEFAULT) /
				VOLTAGE_STEP;

		ts_write_object(data, SPT_CTECONFIG,
				CTE_VOLTAGE, voltage);
	}
}

static int ts_get_info(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	struct ts_info *info = &data->info;
	int error;
	u8 val;

	error = ts_read_reg(client, FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = ts_read_reg(client, VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = ts_read_reg(client, VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = ts_read_reg(client, BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = ts_read_reg(client, OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int ts_get_object_table(struct ts_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct ts_object *object = data->object_table + i;

		reg = OBJECT_START + OBJECT_SIZE * i;
		error = ts_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
	}

	return 0;
}

static int ts_initialize(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	struct ts_info *info = &data->info;
	int error;
	u8 val;

	error = ts_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct ts_data),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = ts_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = ts_check_reg_init(data);
	if (error)
		return error;

	ts_handle_pdata(data);

	/* Backup to memory */
	ts_write_object(data, GEN_COMMAND,
			COMMAND_BACKUPNV,
			BACKUP_VALUE);
	msleep(BACKUP_TIME);

	/* Soft reset */
	ts_write_object(data, GEN_COMMAND,
			COMMAND_RESET, 1);
	msleep(RESET_TIME);

	/* Update matrix size at info struct */
	error = ts_read_reg(client, MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = ts_read_reg(client, MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;
}

static ssize_t ts_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	struct ts_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"Object Table Element %d(Type %d)\n",
				i + 1, object->type);

		if (!ts_object_readable(object->type)) {
			count += snprintf(buf + count, PAGE_SIZE - count, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			error = ts_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += snprintf(buf + count, PAGE_SIZE - count,
					"  Byte %d: 0x%x (%d)\n", j, val, val);
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	return count;
}

int ts_boot_read_mem(struct i2c_client *client, unsigned char *mem)
{
	struct i2c_msg rmsg;
	int ret;

	rmsg.addr = client->addr;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 1;
	rmsg.buf = mem;
	ret = i2c_transfer(client->adapter, &rmsg, 1);

	return ret;
}

static int ts_load_fw(struct device *dev, const char *fn)
{
	struct ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
#ifdef USE_FIRMWARE_CLASS
	const struct firmware *fw = NULL;
#endif
	unsigned int frame_size = 0;
	unsigned int pos = 0;
	int ret;

#ifdef USE_FIRMWARE_CLASS
	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}
#else
	unsigned char  *firmware_data;
	unsigned long int fw_size = 0;

	firmware_data = firmware;
	fw_size = sizeof(firmware);
#endif

	/* Change to the bootloader mode */
	ts_write_object(data, GEN_COMMAND,
			COMMAND_RESET, BOOT_VALUE);
	msleep(RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == APP_LOW)
		client->addr = BOOT_LOW;
	else
		client->addr = BOOT_HIGH;

	{
		unsigned char boot_status;
		unsigned char boot_ver;
		unsigned int  crc_error_count;
		unsigned int next_frame;
		unsigned int j, read_status_flag;

		crc_error_count = 0;
		next_frame = 0;

		if ((ts_boot_read_mem(client, &boot_status) == 1) &&
			(boot_status & WAITING_BOOTLOAD_CMD) == WAITING_BOOTLOAD_CMD) {

			boot_ver = boot_status & BOOT_STATUS_MASK;
			crc_error_count = 0;
			next_frame = 0;

#ifdef USE_FIRMWARE_CLASS
				while (pos < fw->size) {
#else
				while (pos < fw_size) {
#endif
				for (j = 0; j < 5; j++) {
					if (ts_boot_read_mem(client, &boot_status) == 1)	{
						read_status_flag = 1;
						break;
					} else {
						mdelay(60);
						read_status_flag = 0;
					}
				}

				if (read_status_flag == 1) {
					if ((boot_status & WAITING_BOOTLOAD_CMD) == WAITING_BOOTLOAD_CMD) {

						if (ts_unlock_bootloader(client) == 0) {
							mdelay(10);
							printk(KERN_INFO"Unlock OK\n");
						} else {
							printk(KERN_INFO"Unlock fail\n");
						}
					} else if ((boot_status & WAITING_BOOTLOAD_CMD) == WAITING_FRAME_DATA) {

						#ifdef USE_FIRMWARE_CLASS
							frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));
						#else
							frame_size = ((*(firmware_data + pos) << 8) | *(firmware_data + pos + 1));
						#endif

						/* We should add 2 at frame size as the the firmware data is not
						 * included the CRC bytes.
						 */
						frame_size += 2;

						/* Exit if frame data size is zero */
						if (0 == frame_size) {
							printk(KERN_INFO"0 == frame_size\n");
							ret = 0;
							goto out;
						}
						next_frame = 1;
						/* Write one frame to device */
						#ifdef USE_FIRMWARE_CLASS
							ts_fw_write(client, fw->data + pos, frame_size);
						#else
							ts_fw_write(client, firmware_data + pos, frame_size);
						#endif

						mdelay(10);
					} else if (boot_status == FRAME_CRC_CHECK) {
						#ifdef TOUCH_LOG_ENABLE
							printk(KERN_INFO"CRC Check\n");
						#endif
					} else if (boot_status == FRAME_CRC_PASS) {
						if (next_frame == 1) {
							#ifdef TOUCH_LOG_ENABLE
								printk(KERN_INFO"CRC Ok\n");
							#endif
							pos += frame_size;
							next_frame = 0;

							#ifdef USE_FIRMWARE_CLASS
								printk(KERN_INFO"Updated %d bytes / %zd bytes\n", pos, fw->size);
							#else
								printk(KERN_INFO"Updated %d bytes / %lu bytes\n", pos, fw_size);
							#endif
						} else
							printk(KERN_INFO"next_frame != 1\n");
					} else if (boot_status	== FRAME_CRC_FAIL) {
						printk(KERN_INFO"CRC Fail\n");
						crc_error_count++;
					}
					if (crc_error_count > 10) {
						ret = 1;
						goto out;
					}
				} else {
					ret = 1;
					goto out;
				}
			}
		} else {
			printk(KERN_INFO"[TSP] read_boot_state() or (boot_status & 0xC0) == 0xC0) is fail!!!\n");
			ret = 1;
		}
	}

out:
#ifdef USE_FIRMWARE_CLASS
	release_firmware(fw);
#endif

	/* Change to slave address of application */
	if (client->addr == BOOT_LOW)
		client->addr = APP_LOW;
	else
		client->addr = APP_HIGH;

	return ret;
}

static ssize_t ts_update_fw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char a, b;
	struct ts_data *data = dev_get_drvdata(dev);

	a = data->info.version & 0xf0;
	a = a >> 4;
	b = data->info.version & 0x0f;
	return snprintf(buf, PAGE_SIZE, "%d%d\n", a, b);
}

static ssize_t ts_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	unsigned int version;
	int error;

	if (sscanf(buf, "%u", &version) != 1) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	if (data->info.version < VER_32 || version < VER_32) {
		dev_err(dev, "FW update supported starting with version 21\n");
		return -EINVAL;
	}

	printk(KERN_INFO"The firmware update Start!!\n");
	firmware_status = UPDATE_FIRM_UP;

	disable_irq(data->client->irq);

	error = ts_load_fw(dev, FW_NAME);
	if (error) {
		firmware_status = FAIL_FIRM_UP;
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		firmware_status = SUCCESS_FIRM_UP;

		printk(KERN_INFO"The firmware update succeeded!!\n");

		/* Wait for reset */
		msleep(FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		ts_initialize(data);
	}

	enable_irq(data->irq);

	return count;
}

static ssize_t ts_firm_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", firmware_status);
}

static DEVICE_ATTR(object, 0444, ts_object_show, NULL);
static DEVICE_ATTR(firm_status, S_IRUGO | S_IWUSR | S_IXOTH,
				   ts_firm_status_show, NULL);
static DEVICE_ATTR(update_fw, 0664, ts_update_fw_show, ts_update_fw_store);

static struct attribute *ts_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_firm_status.attr,
	NULL
};

static const struct attribute_group ts_attr_group = {
	.attrs = ts_attrs,
};

static void ts_start(struct ts_data *data)
{
	/* Touch enable */
	ts_write_object(data,
			TOUCH_MULTI, TOUCH_CTRL, 0x8F);

	ts_write_object(data,
			GEN_POWER, POWER_IDLEACQINT, 0x40);

	ts_write_object(data,
			GEN_POWER, POWER_ACTVACQINT, 0xFF);

	ts_write_object(data,
			GEN_POWER, POWER_ACTV2IDLETO, 0x32);
}

static void ts_stop(struct ts_data *data)
{
	ts_write_object(data,
			GEN_POWER, POWER_IDLEACQINT, 0);

	ts_write_object(data,
			GEN_POWER, POWER_ACTVACQINT, 0);

	ts_write_object(data,
			GEN_POWER, POWER_ACTV2IDLETO, 0);

	ts_write_object(data,
			TOUCH_MULTI, TOUCH_CTRL, 0);
}

static int ts_input_open(struct input_dev *dev)
{
	struct ts_data *data = input_get_drvdata(dev);

	ts_start(data);

	return 0;
}

static void ts_input_close(struct input_dev *dev)
{
	struct ts_data *data = input_get_drvdata(dev);

	ts_stop(data);
}

static int __devinit ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct ts_data *data;
	struct input_dev *input_dev;
	int error;

	if (!client->dev.platform_data)
		return -EINVAL;

	data = kzalloc(sizeof(struct ts_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	qt_time_point = 0;

	INIT_WORK(&data->work, ts_work_func);

	input_dev->name = "AT42QT602240/ATMXT224 Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = ts_input_open;
	input_dev->close = ts_input_close;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_SEARCH, input_dev->keybit);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, MAX_YC, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
			MAX_FINGER - 1, 0, 0);

	input_set_drvdata(input_dev, data);

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = client->dev.platform_data;
	data->irq = client->irq;

	ts_power_up(data);

	msleep(100);

	i2c_set_clientdata(client, data);

	error = ts_initialize(data);
	if (error)
		goto err_free_object;


	/* LGE_SJIT 2011-11-17 [dojip.kim@lge.com] add trigger low */
	error = request_irq(client->irq, ts_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW,
			client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &ts_attr_group);
	if (error)
		goto err_unregister_device;

	firmware_status = NO_FIRM_UP;
	esd_check = 0;
	ts_cal_check = 0;
	qt_time_point = jiffies_to_msecs(jiffies);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ts_early_suspend;
	data->early_suspend.resume = ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit ts_remove(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);

	unregister_early_suspend(&data->early_suspend);

	sysfs_remove_group(&client->dev.kobj, &ts_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

static void ts_power_up(struct ts_data *data)
{
	data->pdata->power_enable(1, true);

	msleep(80);
	enable_irq(data->client->irq);
	msleep(20);
}

#ifdef CONFIG_PM
static int ts_suspend(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		ts_stop(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int ts_resume(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	struct ts_finger *finger = data->finger;
	int id;

	for (id = 0; id < MAX_FINGER; id++) {
		if (finger[id].status != 0)
			finger[id].status = 0;
	}

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		ts_start(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else
#define ts_suspend	NULL
#define ts_resume		NULL
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *h)
{
	struct ts_data *data = container_of(h, struct ts_data, early_suspend);

	cancel_work_sync(&data->work);
	disable_irq(data->client->irq);

	if (firmware_status != UPDATE_FIRM_UP) {
		ts_suspend(data->client);
		qt_timer_state = 0;
	}
}

static void ts_late_resume(struct early_suspend *h)
{
	struct ts_data *data = container_of(h, struct ts_data, early_suspend);

	if (firmware_status != UPDATE_FIRM_UP) {
		ts_cal_check = 1;
		ts_resume(data->client);
		enable_irq(data->client->irq);
	}
}
#endif

/* LGE_SJIT 2011-11-17, [dojip.kim@lge.com] add 'atmel_mxt_ts' as id */
static const struct i2c_device_id ts_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts_id);

static struct i2c_driver ts_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
	},
	.probe		= ts_probe,
	.remove		= __devexit_p(ts_remove),
	.id_table	= ts_id,
};

static int __init ts_init(void)
{
	atmel_wq = create_singlethread_workqueue("atmel_wq");
	if (!atmel_wq)
		return -ENOMEM;
	return i2c_add_driver(&ts_driver);
}

static void __exit ts_exit(void)
{
	i2c_del_driver(&ts_driver);
	if (atmel_wq)
		destroy_workqueue(atmel_wq);
}

module_init(ts_init);
module_exit(ts_exit);

/* Module information */
MODULE_AUTHOR("fred.cho");
MODULE_DESCRIPTION("AT42QT602240/ATMXT224 Touchscreen driver");
MODULE_LICENSE("GPL");
