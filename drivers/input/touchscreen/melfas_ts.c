/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c/melfas_ts.h>
#include <linux/gpio.h>

#ifdef CONFIG_TS_INFO_CLASS
#include "ts_class.h"
#endif

#define MODE_CONTROL                    0x01
#define TS_READ_START_ADDR              0x10

#define TS_READ_START_ADDR		0x10
#define TS_READ_VERSION_ADDR		0xF0
#define TS_HW_REVISION_ADDR             0xF1
#define TS_CORE_VERSION_ADDR            0xF3
#define TS_PRIVATE_CUSTOM_VERSION_ADDR  0xF4
#define TS_PUBLIC_CUSTOM_VERSION_ADDR   0xF5

#define TS_READ_REGS_LEN		100
#define TS_READ_VERSION_INFO_LEN	6

#define MELFAS_MAX_TOUCH		10  /* ts->pdata->num_of_finger */

#define I2C_RETRY_CNT			10

#define PRESS_KEY			1
#define RELEASE_KEY			0

#define DEBUG_PRINT			0

#define	SET_DOWNLOAD_BY_GPIO		1

#if SET_DOWNLOAD_BY_GPIO
/*#include "mms136_download.h"*/
#include "mms136_ISP_download.h"
#endif

static int irq_flag;
static int is_reflashing;
static int fw_no_sleep;

enum {
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info {
	int strength;
	int width;
	int posX;
	int posY;
};

struct melfas_ts_data {
#ifdef CONFIG_TS_INFO_CLASS
	struct ts_info_classdev cdev;
#endif
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;
	struct delayed_work work;
	uint32_t flags;
	int version;
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

int (*g_power_enable) (int en, bool log_en);
static char tmp_flag[10];
static struct workqueue_struct *melfas_wq;

/* LGE_SJIT 2011-12-09 [dojip.kim@lge.com] helper for irq */
static inline void melfas_enable_irq(int irq)
{
#if DEBUG_PRINT
	printk(KERN_INFO "%s\n", __func__);
#endif
	enable_irq(irq);
}


static inline void melfas_disable_irq_nosync(int irq)
{
#if DEBUG_PRINT
	printk(KERN_INFO "%s\n", __func__);
#endif
	disable_irq_nosync(irq);
}

static void melfas_ts_work_func(struct work_struct *work)
{
	struct melfas_ts_data *ts =
		container_of(to_delayed_work(work), struct melfas_ts_data, work);
	struct i2c_client *client;
	int ret = 0, i, count = 0;
	uint8_t buf[TS_READ_REGS_LEN];
	int touchType = 0, touchState = 0, touchID = 0;
	int posX = 0, posY = 0, width = 0, strength = 10;
	int keyID = 0, reportID = 0;
	uint8_t read_num = 0;


#if DEBUG_PRINT
	printk(KERN_ERR "%s\n", __func__);
#endif
	if (ts == NULL) {
		printk(KERN_ERR "%s: TS NULL\n", __func__);
		return;
	}
	client = ts->client;

#define MIP_INPUT_EVENT_PACKET_SIZE	0x0F
#define MIP_INPUT_EVENT_INFORMATION	0x10

	buf[0] = MIP_INPUT_EVENT_PACKET_SIZE;
	ret = i2c_master_send(client, buf, 1);
	ret |= i2c_master_recv(client, &read_num, 1);
	/* LGE_SJIT 2011-12-09 [dojip.kim@lge.com] reset on i2c err */
	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c error %d\n", __func__, ret);
		g_power_enable(0, true);
		g_power_enable(1, true);
	}

	if (read_num == 0) {
		printk(KERN_ERR "read number 0 error (irq_flag %d)!\n",
				irq_flag);

		if (irq_flag == 0) {
			irq_flag++;
			melfas_enable_irq(ts->client->irq);
		}

		return;
	}

	buf[0] = MIP_INPUT_EVENT_INFORMATION;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &buf[0], read_num);

	for (i = 0; i < read_num; i = i + 6) {
		if (ret < 0) {
			printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
			if (irq_flag == 0) {
				irq_flag++;
				melfas_enable_irq(ts->client->irq);
			}
			return ;
		} else {
			touchType  =  ((buf[i] & 0x60) >> 5);				/* Touch Screen, Touch Key */
			touchState = ((buf[i] & 0x80) == 0x80);				/* touchAction = (buf[0]>>7)&&0x01;*/
			reportID = (buf[i] & 0x0F);					/* Touch Screen -> n.th finger input
											Touch Key -> n.th touch key area. */
			posX = (uint16_t) (buf[i + 1] & 0x0F) << 8 | buf[i + 2];	/* X position (0 ~ 4096, 12 bit) */
			posY = (uint16_t) (buf[i + 1] & 0xF0) << 4 | buf[i + 3];	/* Y position (0 ~ 4096, 12 bit) */
			width = buf[i + 4];

			if (touchType == TOUCH_KEY)
				keyID = reportID;
			else if (touchType == TOUCH_SCREEN)
				touchID = reportID-1;

			if (touchID > ts->pdata->num_of_finger-1) {
				if (irq_flag == 0) {
					irq_flag++;
					melfas_enable_irq(ts->client->irq);
				}
			    return;
			}

			if (touchType == TOUCH_SCREEN) {
				g_Mtouch_info[touchID].posX = posX;
				g_Mtouch_info[touchID].posY = posY;
				g_Mtouch_info[touchID].width = width;

				if (touchState)
					g_Mtouch_info[touchID].strength = strength;
				else {
					g_Mtouch_info[touchID].strength = 0;
					tmp_flag[touchID] = 1;
				}
			} else if (touchType == TOUCH_KEY) {
				if (keyID > ts->pdata->num_of_button || keyID == 0)
					printk(KERN_ERR "Touchkey ID error \n");
				else
					input_report_key(ts->input_dev, ts->pdata->button[keyID-1], touchState ? PRESS_KEY : RELEASE_KEY);
#if DEBUG_PRINT
				printk(KERN_ERR "melfas_ts_work_func: keyID : %d, touchState: %d\n", keyID, touchState);
#endif
				break;
			}

		}
	}

	if (touchType == TOUCH_SCREEN) {
		for (i = 0; i < ts->pdata->num_of_finger; i++) {
			if (g_Mtouch_info[i].strength <= 0) {
				if (count <  ts->pdata->num_of_finger-1) {
					count++;
					continue;
				} else {
					input_mt_sync(ts->input_dev);
					break;
				}
			}

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
			if (g_Mtouch_info[i].strength <= 0) {
				input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
			}
			else {
				input_report_abs(ts->input_dev, ABS_PRESSURE, 255);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
			}

			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_mt_sync(ts->input_dev);
#if DEBUG_PRINT
			if ((touchState == 1 && tmp_flag[touchID] == 1) || (touchState == 0 && tmp_flag[touchID] == 1)) {
				printk(KERN_ERR "Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
					   i, touchState, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
				if (touchState == 1)
					tmp_flag[touchID] = 0;
			}
#endif
			if (g_Mtouch_info[i].strength == 0)
				g_Mtouch_info[i].strength = -1;
		}
	}

	input_sync(ts->input_dev);

	if (irq_flag == 0) {
		irq_flag++;
		melfas_enable_irq(ts->client->irq);
	}
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_irq_handler\n");
#endif
	if (irq_flag == 1) {
		irq_flag--;
		melfas_disable_irq_nosync(ts->client->irq);
    }

	queue_delayed_work(melfas_wq, &ts->work, 0);

	return IRQ_HANDLED;
}

static ssize_t
mms136_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int len;

	len = snprintf(buf, PAGE_SIZE, "%d\n", ts->version);
	return len;
}

static ssize_t
mms136_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int len;
	len = snprintf(buf, PAGE_SIZE, "\nMMS-136 Device Status\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "irq num       is %d\n", ts->client->irq);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_irq num  is %d(level=%d)\n", ts->pdata->i2c_int_gpio, gpio_get_value(ts->pdata->i2c_int_gpio));
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_scl num  is %d\n", ts->pdata->gpio_scl);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_sda num  is %d\n", ts->pdata->gpio_sda);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_en  num  is %d\n", ts->pdata->gpio_ldo);
	len += snprintf(buf + len, PAGE_SIZE - len, "irq is %s\n", irq_flag ? "enabled" : "disabled");
	len += snprintf(buf + len, PAGE_SIZE - len, "mode is %s\n", is_reflashing ? "reflash" : "working");
	len += snprintf(buf + len, PAGE_SIZE - len, "Power status  is %s\n", gpio_get_value(ts->pdata->gpio_ldo) ? "on" : "off");
	return len;
}

static ssize_t
mms136_fw_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;
	switch (cmd) {
	case 1:
		ts->pdata->power_enable(1, true);
		mdelay(50);
		if (irq_flag == 1) {
			irq_flag--;
			melfas_disable_irq_nosync(ts->client->irq);
		}
		cancel_delayed_work_sync(&ts->work);
		is_reflashing = 1;
		do {
			mdelay(100);
		} while (!gpio_get_value(ts->pdata->i2c_int_gpio));

		mms100_download(0);

		if (irq_flag == 0) {
			irq_flag++;
			melfas_enable_irq(ts->client->irq);
		}
		is_reflashing = 0;
		ts->pdata->power_enable(0, true);
		ts->pdata->power_enable(1, true);
		break;
	default:
		printk(KERN_INFO "usage: echo [1] > fw_upgrade\n");
		break;
	}
	return count;
}

static ssize_t
mms136_power_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 1: /* touch power on */
		ts->pdata->power_enable(1, true);
		break;
	case 2: /*touch power off */
		ts->pdata->power_enable(0, true);
		break;
	case 3:
		ts->pdata->power_enable(0, true);
		ts->pdata->power_enable(1, true);
		break;
	default:
		printk(KERN_INFO "usage: echo [1|2|3] > control\n");
		printk(KERN_INFO "  - 1: power on\n");
		printk(KERN_INFO "  - 2: power off\n");
		printk(KERN_INFO "  - 3: power reset\n");
		break;
	}
	return count;
}

static ssize_t
mms136_irq_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 1: /* interrupt pin high */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			printk(KERN_ERR "%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 1);
		printk(KERN_INFO "MMS-136 INTR GPIO pin high\n");
		break;
	case 2: /* interrupt pin LOW */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			printk(KERN_ERR "%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 0);
		printk(KERN_INFO "MMS-136 INTR GPIO pin low\n");
		break;
	case 3:
		if (irq_flag == 0) {
			irq_flag++;
			melfas_enable_irq(ts->client->irq);
		} else
			printk(KERN_INFO "Already Irq Enabled\n");
		break;
	case 4:
		if (irq_flag == 1) {
			irq_flag--;
			melfas_disable_irq_nosync(ts->client->irq);
		} else
			printk(KERN_INFO "Already Irq Disabled\n");
		break;
	default:
		printk(KERN_INFO "usage: echo [1|2|3|4] > control\n");
		printk(KERN_INFO "  - 1: interrupt pin high\n");
		printk(KERN_INFO "  - 2: interrupt pin low\n");
		printk(KERN_INFO "  - 3: enable_irq\n");
		printk(KERN_INFO "  - 4: disable_irq_nosync\n");
		break;
	}
	return count;
}

static ssize_t
mms136_reg_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd, ret, reg_addr, length, i;
	uint8_t reg_buf[TS_READ_REGS_LEN];
	if (sscanf(buf, "%d, 0x%x, %d", &cmd, &reg_addr, &length) != 3)
		return -EINVAL;
	switch (cmd) {
	case 1:
		reg_buf[0] = reg_addr;
		ret = i2c_master_send(ts->client, reg_buf, 1);
		if (ret < 0) {
			printk(KERN_ERR "i2c master send fail\n");
			break;
		}
		ret = i2c_master_recv(ts->client, reg_buf, length);
		if (ret < 0) {
			printk(KERN_ERR "i2c master recv fail\n");
			break;
		}
		for (i = 0; i < length; i++) {
			printk(KERN_INFO "0x%x", reg_buf[i]);
		}
		printk(KERN_INFO "\n 0x%x register read done\n", reg_addr);
		break;
	case 2:
		reg_buf[0] = reg_addr;
		reg_buf[1] = length;
		ret = i2c_master_send(ts->client, reg_buf, 2);
		if (ret < 0) {
			printk(KERN_ERR "i2c master send fail\n");
			break;
		}
		printk(KERN_INFO "\n 0x%x register write done\n", reg_addr);
		break;
	default:
		printk(KERN_INFO "usage: echo [1(read)|2(write)], [reg address], [length|value] > reg_control\n");
		printk(KERN_INFO "  - Register Set or Read\n");
		break;
	}
	return count;
}

static ssize_t
mms136_inspection_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len, i, j;
	uint8_t write_buf[5];
	uint8_t read_buf[500];
	int flag = 0;
	write_buf[0] = 0xA0;
	write_buf[1] = 0x40;
	i2c_master_send(ts->client, write_buf, 2);
	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 3) {
			flag = 0;
			break;
		}
		mdelay(100);
	}
	write_buf[0] = 0xA0;
	write_buf[1] = 0x41;
	i2c_master_send(ts->client, write_buf, 2);
	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 3) {
			flag = 0;
			break;
		}
		mdelay(100);
	}
	len = snprintf(buf, PAGE_SIZE, "Touch Firmware Version is %d\n", ts->version);
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < 22; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < 12 ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < 22; j++) {
			write_buf[0] = 0xA0;
			write_buf[1] = 0x42;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);
			write_buf[0] = 0xAE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_buf[0], 1);

			write_buf[0] = 0xAF;
			i2c_master_send(ts->client, write_buf, 2);
			i2c_master_recv(ts->client, read_buf, read_buf[0]);
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", read_buf[i * 22 + j]);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	write_buf[0] = 0xA0;
	write_buf[1] = 0x4F;

	i2c_master_send(ts->client, write_buf, 2);
	return len;
}

static struct device_attribute mms136_device_attrs[] = {
	__ATTR(status,  S_IRUGO | S_IWUSR, mms136_status_show, NULL),
	__ATTR(version, S_IRUGO | S_IWUSR, mms136_version_show, NULL),
	__ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, mms136_fw_upgrade_store),
	__ATTR(power_control, S_IRUGO | S_IWUSR, NULL, mms136_power_control_store),
	__ATTR(irq_control, S_IRUGO | S_IWUSR, NULL, mms136_irq_control_store),
	__ATTR(reg_control, S_IRUGO | S_IWUSR, NULL, mms136_reg_control_store),
	__ATTR(inspection, S_IRUGO | S_IWUSR, mms136_inspection_show, NULL),
};

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0, i;

	uint8_t buf[TS_READ_VERSION_INFO_LEN];
	irq_flag = 1;
	is_reflashing = 0;
	fw_no_sleep = 0;
#if DEBUG_PRINT
	dev_info(&client->dev, "%s: melfas_ts_probe Start!!!\n", __func__);
#endif

	memset(&tmp_flag[0], 0x01, sizeof(tmp_flag));

	melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!melfas_wq) {
		printk(KERN_ERR "[TOUCH]failed to create singlethread workqueue\n");
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->pdata = client->dev.platform_data;

	ret = ts->pdata->power_enable(0, true);
	ret = ts->pdata->power_enable(1, true);

	g_power_enable = ts->pdata->power_enable;

	INIT_DELAYED_WORK(&ts->work, melfas_ts_work_func);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		ret = i2c_master_send(ts->client, &buf[0], 1);
		if (ret >= 0) {
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() ok [%d]\n", ret);
			break;
		} else {
			printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed[%d]\n", ret);
			if (i == I2C_RETRY_CNT-1) {
				printk(KERN_ERR "melfas_ts_probe : no touch panel \n");
				ret = mms100_download(1);
				if (ret == 0)
					printk(KERN_ERR "melfas_ts_probe : Touch FW update Success \n");
				else
					return ret ;
			}
		}
	}

	buf[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf[0], 1);
	ret = i2c_master_recv(ts->client, &buf[0], TS_READ_VERSION_INFO_LEN);

	printk(KERN_INFO "= Melfas Version Info =\n");
	printk(KERN_INFO "Panel Version :: %d, HW Revision :: %d, HW Compatibility GR :: %d\n", buf[0], buf[1], buf[2]);
	printk(KERN_INFO "Core Version :: %d, Private Custom Version :: %d, Public Custom Version :: %d\n", buf[3], buf[4], buf[5]);

	ts->version = buf[3];

#if DEBUG_PRINT
	printk(KERN_ERR "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif

#if SET_DOWNLOAD_BY_GPIO
	if (buf[3] < ts->pdata->fw_ver || buf[3] == 0xFF) {
		printk(KERN_ERR "melfas_probe : download start \n");
		/*mcsdl_download_binary_data();*/
		mms100_download(0);

		buf[0] = TS_READ_VERSION_ADDR;
		ret = i2c_master_send(ts->client, &buf[0], 1);
		ret = i2c_master_recv(ts->client, &buf[0], TS_READ_VERSION_INFO_LEN);

		ts->version = buf[3];
	}

	/*ret = ts->pdata->power_enable(0, true);
	mdelay(50);
	ret = ts->pdata->power_enable(1, true);*/

#endif /* SET_DOWNLOAD_BY_GPIO end */

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "melfas-ts" ;

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	for (i = 0; i < ts->pdata->num_of_button; i++)
		ts->input_dev->keybit[BIT_WORD(ts->pdata->button[i])] |= BIT_MASK(ts->pdata->button[i]);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  ts->pdata->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,  ts->pdata->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "melfas_ts_probe: Failed to register device\n");
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}


	if (ts->client->irq) {
#if DEBUG_PRINT
		printk(KERN_ERR "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif
		ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);

		if (ret > 0) {
			printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	for (i = 0; i < ts->pdata->num_of_finger ; i++)
		g_Mtouch_info[i].strength = -1;

#if DEBUG_PRINT
	/* SJIT 2011-12-09 [dojip.kim@lge.com] read MODE_CONTROL */
	ret = i2c_smbus_read_byte_data(client, MODE_CONTROL);
	if (ret < 0) {
		printk(KERN_ERR "%s: i2c read error\n", __func__);
	}
	printk(KERN_INFO "%s: mode control 0x%x\n", __func__, ret);
	printk(KERN_INFO "melfas_ts_probe: succeed to register input device\n");
#endif

	for (i = 0; i < ARRAY_SIZE(mms136_device_attrs); i++) {
		ret = device_create_file(&client->dev, &mms136_device_attrs[i]);
		if (ret) {
			goto err_request_irq;
		}
	}

#ifdef CONFIG_TS_INFO_CLASS
	ts->cdev.name = "version";
	ts->cdev.version = ts->version;
	ts->cdev.flags = ts->flags;

	ts_info_classdev_register(&client->dev, &ts->cdev);
#endif

#if CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#if DEBUG_PRINT
	printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif
	return 0;

err_request_irq:
	printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");
err_check_functionality_failed:
	printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");

	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
#ifdef CONFIG_TS_INFO_CLASS
	ts_info_classdev_unregister(&ts->cdev);
#endif
	kfree(ts);
	return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;
	for (i = 0; i < ts->pdata->num_of_finger; i++) {
		if (-1 == g_Mtouch_info[i].strength) {
			g_Mtouch_info[i].posX = 0;
			g_Mtouch_info[i].posY = 0;
			continue;
		}

		g_Mtouch_info[i].strength = 0;

		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_mt_sync(ts->input_dev);

		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;

		if (0 == g_Mtouch_info[i].strength)
			g_Mtouch_info[i].strength = -1;
	}
}

static void melfas_ts_suspend_func(struct melfas_ts_data *ts)
{
	int ret;

	printk(KERN_ERR "melfas_ts_suspend start \n");

	if (irq_flag == 1) {
		irq_flag--;
		melfas_disable_irq_nosync(ts->client->irq);
	}

	ret = cancel_delayed_work_sync(&ts->work);
	/* sleep */
	ret = i2c_smbus_write_byte_data(ts->client, MODE_CONTROL, 0x00);

	release_all_fingers(ts);

	ret = ts->pdata->power_enable(0, true);
	if (ret < 0)
		printk(KERN_ERR "melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");

	printk(KERN_ERR "melfas_ts_suspend end \n");

}

static void melfas_ts_resume_func(struct melfas_ts_data *ts)
{
	int ret = 0;
	printk(KERN_ERR "melfas_ts_resume start \n");

	ret = ts->pdata->power_enable(1, true);

	if (irq_flag == 0) {
		irq_flag++;
		melfas_enable_irq(ts->client->irq);
	}

	printk(KERN_ERR "melfas_ts_resume end \n");

}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	if (is_reflashing == 0) {
		melfas_ts_suspend_func(ts);
		fw_no_sleep = 0;
	} else
		fw_no_sleep = 1;
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	if (is_reflashing == 0 && fw_no_sleep == 0)
		melfas_ts_resume_func(ts);
}
#endif

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] FIXME */
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EALRYSUSPEND)
static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);


	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);


	return 0;
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_TS_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= MELFAS_TS_NAME,
	},
	.id_table	= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= __devexit_p (melfas_ts_remove),
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&melfas_ts_driver);
	if (ret < 0) {
		printk(KERN_ERR "[TOUCH]failed to i2c_add_driver\n");
		destroy_workqueue(melfas_wq);
	}
	return ret;
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);

	if (melfas_wq)
		destroy_workqueue(melfas_wq);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
