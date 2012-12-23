/*
 * Keyboard/Button LED Driver controlled by GPIO
 *
 * Copyright (C) 2011 LGE Inc.
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Dan Murphy <DMurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/lge/leds_keypad.h>
#ifdef CONFIG_LGE_HANDLE_PANIC
//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
//mo2haewoon.you@lge.com <= [END]
#endif

static int keypad_gpio;
static int use_hold_key = 0;
static int hold_key_gpio;

#ifdef CONFIG_LGE_HANDLE_PANIC
//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET	
extern int lge_hidden_reset_mode;
int hidden_check=1;
//mo2haewoon.you@lge.com <= [END]
#endif

struct keypad_led_data {
	struct led_classdev keypad_led_class_dev;
#ifdef CONFIG_LGE_HANDLE_PANIC
//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET	
struct delayed_work hidden_reset_led_delayed_work;
//mo2haewoon.you@lge.com <= [END]	
#endif
};
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
/* LGE_CHANGE_S, dukwung.kim, 2012-03-20, TEST CODE LED ON/OFF */
int pw_led_on_off=1;
int cause_of_pw_pressed=0;
void set_pw_led_on_off(int value)
{
	printk(KERN_ERR ">>>>>>> set_pw_led_on_off PW_LED: %d, pw_led_on_off: %d>>>>>>>>>>>>\n", value, pw_led_on_off);
	if(value == PW_LED_ON && pw_led_on_off == 0)
	{
		//printk(KERN_ERR ">>>>>>> SYSFS_LED ON!>>>>>>>>>>>>\n");
		gpio_set_value(hold_key_gpio, 1);
		pw_led_on_off = 1;
		return ;
	}
	else if(value == PW_LED_OFF && pw_led_on_off == 1 && cause_of_pw_pressed!=1)
	{
	//	printk(KERN_ERR " SYSFS_LED OFF!\n");
		gpio_set_value(hold_key_gpio, 0);
		pw_led_on_off = 0;
		return ;
	}
	printk(KERN_ERR ">>>>>>>: set_pw_led_on_off PW_LED: %d, pw_led_on_off: %d>>>>>>>>>>>>\n", value, pw_led_on_off);
	return ;
}
EXPORT_SYMBOL(set_pw_led_on_off);

/* LGE_CHANGE_S, dukwung.kim, 2012-03-20, TEST CODE LED ON/OFF */
#endif
static void keypad_led_store(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	/* 20120224 sangjae.han@lge.com Add sysfile to maintain the backlight on[LGE_START]*/
	if(led_cdev->br_maintain_trigger == 1){
		printk(KERN_ERR "[pwr_led]: br_maintain_on trigger is on!\n");
		return;
		}
	/* 20120224 sangjae.han@lge.com Add sysfile to maintain the backlight on[LGE_END]*/

#ifdef CONFIG_LGE_HANDLE_PANIC
//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET
	if( (lge_hidden_reset_mode == 1) && ( hidden_check == 1) )
	{
		// LED off
	}
	else		
	{
		if (value != 0 && value < 255) {
			//printk(KERN_INFO "FRONT_LED: SYSFS_LED On!\n");
			gpio_set_value(keypad_gpio, 1);

		} else if(value == 255){
		//	printk(KERN_INFO "ALL_LED: SYSFS_LED On!\n");
			gpio_set_value(keypad_gpio, 1);
			if(use_hold_key)
				gpio_set_value(hold_key_gpio, 1);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
			pw_led_on_off = 1;
			cause_of_pw_pressed = 1;
#endif
		} else {
			//printk(KERN_INFO "ALL_LED: SYSFS_LED Off!\n");
			gpio_set_value(keypad_gpio, 0);
			if(use_hold_key)
				gpio_set_value(hold_key_gpio, 0);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
			pw_led_on_off = 0;
			cause_of_pw_pressed = 0;
#endif
	         }
         }
//mo2haewoon.you@lge.com <= [END]
#endif
}

#ifdef CONFIG_LGE_HANDLE_PANIC
//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET
static void hidden_reset_check_delayed_work(struct work_struct *work)
{
    hidden_check = 0;
}
//mo2haewoon.you@lge.com <= [END]
#endif

static int __devinit keypad_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct keypad_led_data *info;
	struct leds_keypad_platform_data *pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "%s(): platform_data NULL\n", __func__);
		return -EINVAL;
	}

	keypad_gpio   = pdata->keypad_gpio;
	hold_key_gpio = pdata->hold_key_gpio;
	use_hold_key  = pdata->use_hold_key;
	
	ret = gpio_request(keypad_gpio, "kp_leds_gpio"); 
	if(ret){
		dev_err(&pdev->dev, "request gpio %d failed!\n", keypad_gpio);
		return ret;
	}
	gpio_direction_output(keypad_gpio, 0); 

	/* 20110418 kyungtae.oh@lge.com for Power LED [LGE_START] */
	if (use_hold_key) {
		ret = gpio_request(hold_key_gpio, "pwr_leds_gpio");
		if(ret){
			dev_err(&pdev->dev, "request gpio %d failed!\n", hold_key_gpio);
			return ret;
		}
		gpio_direction_output(hold_key_gpio, 0);
	}
	/* 20110418 kyungtae.oh@lge.com for Power LED [LGE_END] */

	info = kzalloc(sizeof(struct keypad_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

#ifdef CONFIG_LGE_HANDLE_PANIC
	//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET
	INIT_DELAYED_WORK(&info->hidden_reset_led_delayed_work, hidden_reset_check_delayed_work);
        schedule_delayed_work(&info->hidden_reset_led_delayed_work, msecs_to_jiffies(45000));	
	//mo2haewoon.you@lge.com <= [END]
#endif

	platform_set_drvdata(pdev, info);

	/* LGE_SJIT 2011-12-05 [dojip.kim@lge.com] set by platform_data */
	if (pdata->name)
		info->keypad_led_class_dev.name = pdata->name;
	else
		info->keypad_led_class_dev.name = "keyboard-backlight";
	info->keypad_led_class_dev.brightness_set = keypad_led_store;
	info->keypad_led_class_dev.max_brightness = LED_FULL;

	ret = led_classdev_register(&pdev->dev, &info->keypad_led_class_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: Register led class failed\n", __func__);
		kfree(info);
		return ret;
	}

	return ret;
}

static int keypad_led_remove(struct platform_device *pdev)
{
	struct keypad_led_data *info = platform_get_drvdata(pdev);

	led_classdev_unregister(&info->keypad_led_class_dev);

	return 0;
}

static struct platform_driver keypad_led_driver = {
	.probe  = keypad_led_probe,
	.remove = keypad_led_remove,
	.driver = {
		.name = "keypad_led",
		.owner = THIS_MODULE,
	},
};

static int __init keypad_led_init(void)
{
	return platform_driver_register(&keypad_led_driver);
}

static void __exit keypad_led_exit(void)
{
	platform_driver_unregister(&keypad_led_driver);
}

module_init(keypad_led_init);
module_exit(keypad_led_exit);

MODULE_DESCRIPTION("Keyboard/Button LEDS driver");
MODULE_LICENSE("GPL");
