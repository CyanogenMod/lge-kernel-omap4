/* drivers/misc/vib-omap-pwm.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
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
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <plat/dmtimer.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/lge/pwm-vibrator.h>
#include "../../staging/android/timed_output.h"

struct regulator *regulator_vib;

struct pwm_vib_data {
	struct timed_output_dev dev;
	struct work_struct vibrator_work;
	struct hrtimer vibe_timer;
	spinlock_t vibe_lock;
	struct omap_dm_timer *pwm_timer;
	atomic_t vibe_gain;
	u8 vibe_state;

	u32	freq;
	u32 duty;
	u32 load;
	u32 match;
	u32	gpio_enable;
	int (*power)(bool on);	
	int port; // LGE_SJIT 2011-09-01 [jongrak.kwon@lge.com] pwm port info
};

struct workqueue_struct *vibrator_wq;

#define CLK_HZ	38400000

static void enable_vibrator(struct pwm_vib_data *data, int on)
{
	unsigned long   flags;

	spin_lock_irqsave(&data->vibe_lock, flags);
	if (on){
		if(!data->vibe_state){
			data->vibe_state = 1;
			omap_dm_timer_set_load_start(data->pwm_timer, 1, 0xffffffff - data->load);
			gpio_set_value(data->gpio_enable, 1);
		}
	}
	else{
		if(data->vibe_state){
			data->vibe_state = 0;
			gpio_set_value(data->gpio_enable, 0);
			omap_dm_timer_stop(data->pwm_timer);
		}
	}	
	spin_unlock_irqrestore(&data->vibe_lock, flags);
}

static void work_func_vibrator(struct work_struct *vibrator_work)
{
	struct pwm_vib_data *data = 
			container_of(vibrator_work, struct pwm_vib_data, vibrator_work);
	
	enable_vibrator(data, atomic_read(&data->vibe_gain));
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct pwm_vib_data *data =
	    container_of(dev, struct pwm_vib_data, dev);
	unsigned long	flags;

	spin_lock_irqsave(&data->vibe_lock, flags);
	hrtimer_cancel(&data->vibe_timer);
	
	if (value == 0)
		atomic_set(&data->vibe_gain, 0);
	else{
		atomic_set(&data->vibe_gain, 1);
		value = (value > 15000 ? 15000 : value);
		hrtimer_start(&data->vibe_timer,
				ktime_set(value / 1000, (value % 1000) * 1000000),
				HRTIMER_MODE_REL);
	}

	queue_work(vibrator_wq, &data->vibrator_work);
	spin_unlock_irqrestore(&data->vibe_lock, flags);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct pwm_vib_data *data =
	    container_of(dev, struct pwm_vib_data, dev);
	if (hrtimer_active(&data->vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&data->vibe_timer);
		return ktime_to_ms(r); // LGE_SJIT 2011-09-01 [jongrak.kwon@lge.com] use API (=> OK if CONFIG_KTIME_SCALAR is changed) 
	} else{
		return 0;
	}
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *vibe_timer)
{
	struct pwm_vib_data *data =
			container_of(vibe_timer, struct pwm_vib_data, vibe_timer);

	atomic_set(&data->vibe_gain, 0);
	queue_work(vibrator_wq, &data->vibrator_work);
	
	return HRTIMER_NORESTART;
}

static ssize_t store_vib_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = 
			container_of(dev, struct platform_device, dev);
	struct pwm_vib_data *data = platform_get_drvdata(pdev);
	int time=0;

	if(data->vibe_state)
		return 0;
	
	sscanf(buf, "%d %d %d", &data->freq, &data->duty, &time);

	if(data->freq > 50000)	data->freq = 50000;
	if(data->freq < 10000)	data->freq = 10000;
	if(data->duty > 99)		data->duty = 99;
	if(data->duty < 1)		data->duty = 1;
	
	data->load = CLK_HZ / data->freq;
	data->match = (data->load * (100 - data->duty)) / 100;

	omap_dm_timer_enable(data->pwm_timer);
	omap_dm_timer_set_match(data->pwm_timer, 1, 0xffffffff - data->match);
	omap_dm_timer_set_pwm(data->pwm_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_stop(data->pwm_timer);
	
	vibrator_enable(&data->dev, time);
	
	return count;
}

static ssize_t show_vib_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = 
			container_of(dev, struct platform_device, dev);
	struct pwm_vib_data *data = platform_get_drvdata(pdev);
	int ret = 0;
	
	ret = sprintf(buf, "====== Vibrator Info ======\n");
	ret += sprintf(buf+ret, "freq = %d, duty = %d\n", data->freq, data->duty);
	ret += sprintf(buf+ret, "load = 0x%x, match = 0x%x\n", data->load, data->match);
	return ret;
}

static DEVICE_ATTR(vib_mode, 0666, show_vib_mode, store_vib_mode);

static int vibrator_probe(struct platform_device *pdev)
{
	struct pwm_vibrator_platform_data *pdata = pdev->dev.platform_data;
	struct pwm_vib_data *data;
	int ret = 0;

	data = kzalloc(sizeof(struct pwm_vib_data), GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "[VIBRATOR] %s[%u]: kzalloc", __FUNCTION__, __LINE__);
		goto	err_vibrator_probe_alloc;
	}
	
	if (pdata){
		data->freq = pdata->freq;
		data->duty = pdata->duty;
		data->gpio_enable = pdata->gpio_enable;
		data->power = pdata->power;
		/* LGE_SJIT 2011-09-01 [jongrak.kwon@lge.com] pwm port info */
		data->port = pdata->port;
	}

	if (!data->freq)
		data->freq = 1;

	data->load = CLK_HZ / data->freq;
	data->match = (data->load * (100 - data->duty)) / 100;
	
	if(data->power)
		data->power(1);

	mdelay(100);

	atomic_set(&data->vibe_gain, 0);
	spin_lock_init(&data->vibe_lock);
	
	gpio_request(data->gpio_enable, "vib_en_gpio");
	gpio_direction_output(data->gpio_enable, 0);

	// LGE_SJIT 2011-09-01 [jongrak.kwon@lge.com] use data port info
	data->pwm_timer	=	omap_dm_timer_request_specific(data->port);
	if (data->pwm_timer == NULL) {
		printk(KERN_ERR "[VIBRATOR] %s[%u]: timer_request", __FUNCTION__, __LINE__);
		goto	err_vibrator_probe_timer_request;
	}
	
	omap_dm_timer_set_source(data->pwm_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_match(data->pwm_timer, 1, 0xffffffff - data->match);
	omap_dm_timer_set_pwm(data->pwm_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	INIT_WORK(&data->vibrator_work, work_func_vibrator);

	hrtimer_init(&data->vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->vibe_timer.function = vibrator_timer_func;
	
	data->dev.name		=	"vibrator";
	data->dev.get_time	=	vibrator_get_time;
	data->dev.enable	=	vibrator_enable;

	ret	=	timed_output_dev_register(&data->dev);
	if (ret < 0){
		printk(KERN_ERR "[VIBRATOR] %s[%u]: dev_register", __FUNCTION__, __LINE__);
		goto	err_vibrator_probe_dev_register;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_vib_mode);
	if (ret < 0) {
		printk(KERN_INFO "[VIBRATOR] sysfs register failed\n");
		goto err_sysfs_create_group_failed;
	}

	platform_set_drvdata(pdev, data);
	
	printk(KERN_INFO "[VIBRATOR] initialized\n");
	return	0;

err_sysfs_create_group_failed:
	device_remove_file(&pdev->dev, &dev_attr_vib_mode);
err_vibrator_probe_dev_register:
	timed_output_dev_unregister(&data->dev);
err_vibrator_probe_timer_request:
	omap_dm_timer_free(data->pwm_timer);
err_vibrator_probe_alloc:
	kfree(data);
	return	-1;
}

static int vibrator_remove(struct platform_device *pdev)
{
	struct pwm_vib_data *data = platform_get_drvdata(pdev);

	if(data->power)
		data->power(0);

	omap_dm_timer_free(data->pwm_timer);
	timed_output_dev_unregister(&data->dev);

	device_remove_file(&pdev->dev, &dev_attr_vib_mode);
	kfree(data);

	return	0;
}

int vibrator_suspend(struct platform_device *dev, pm_message_t state)
{
	struct pwm_vib_data *data = platform_get_drvdata(dev);

	if(data->power)
		data->power(0);
	
	return 0;
}

int vibrator_resume(struct platform_device *dev)
{
	struct pwm_vib_data *data = platform_get_drvdata(dev);
	
	if(data->power)
		data->power(1);
	mdelay(100);

	omap_dm_timer_set_source(data->pwm_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_match(data->pwm_timer, 1, 0xffffffff - data->match);
	omap_dm_timer_set_pwm(data->pwm_timer, 0, 1, OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_stop(data->pwm_timer);
	
	return 0;
}

/* TO DO: Need to make this drivers own platform data entries */
static struct platform_driver vibrator_omap_pwm_driver = {
	.probe	=	vibrator_probe,
	.remove	=	vibrator_remove,
	.suspend = 	vibrator_suspend,
	.resume =	vibrator_resume,
	.driver	=	{
		.name	=	VIB_PWM_NAME,
		.owner	=	THIS_MODULE,
	},
};

static int __init vibrator_omap_pwm_init(void)
{
	vibrator_wq = create_singlethread_workqueue("vibrator_wq");
	if (!vibrator_wq) {
		printk(KERN_WARNING "vibrator: failed to create a thread\n");
	}
	return	platform_driver_register(&vibrator_omap_pwm_driver);
}

static void __exit vibrator_omap_pwm_exit(void)
{
	platform_driver_unregister(&vibrator_omap_pwm_driver);
	if (vibrator_wq)
		destroy_workqueue(vibrator_wq);
}

module_init(vibrator_omap_pwm_init);
module_exit(vibrator_omap_pwm_exit);

MODULE_DESCRIPTION("timed output gptimer pwm vibrator device");
MODULE_LICENSE("GPL");

