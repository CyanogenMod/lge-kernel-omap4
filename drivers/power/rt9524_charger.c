/*
 * Charging IC driver (rt9524)
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>		// might need to get fuel gauge info
#include <linux/power_supply.h>		// might need to get fuel gauge info
#include <linux/i2c/twl.h>
//#include "../staging/android//timed_output.h"
#include <linux/charger_rt9524.h>
#include <linux/muic/muic.h>

#define DRIVER_NAME				"BQ25040"

#define CHR_IC_DEALY				200	/* 200 us */
#define CHR_IC_SET_DEALY			2000	// LGE_CHANGE [byoungcheol.lee@lge.com]  2011-06-30, changed delay time to 2000 from 1500 for bq25040 in Rev. B.
#define CHR_TIMER_SECS				3600 /* 7200 secs*/

// LGE_CHANGE [byoungcheol.lee@lge.com]  2011-08-30, Add to reset Safety timer for test
#define CHR_RESET_SAFETY_TIMER_SECS		10 /* Add to reset Safety timer. Charger will be reconfigurated on every reset time  */
//#define CHR_IC_REINIT_DELAY			32000	// LGE_CHANGE [byoungcheol.lee@lge.com]  2011-09-03, add to reinitial wait time for bq25040.
#define CHR_IC_REINIT_MS_DELAY			32	// LGE_CHANGE [byoungcheol.lee@lge.com]  2011-09-03, add to reinitial wait time for bq25040.

static DEFINE_MUTEX(charging_lock);

enum power_supply_type charging_ic_status;
//static recharging_state_t recharging_status;
static int bat_soc;
struct delayed_work	charger_timer_work;

struct timer_list charging_timer;

void charging_timer_func(unsigned long try)
{
	u32 wait;

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();

	if(charging_timer.data > 3 || bat_soc > 99)
	{
		schedule_delayed_work(&charger_timer_work, 0);
		return;
	}
	else
	{
		//wait = (HZ*CHR_TIMER_SECS * (100 - bat_soc)) / 100;
		wait = HZ*CHR_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-7, changed charging timer
		//wait = HZ*CHR_RESET_SAFETY_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-08-30, changed charging timer for test
		charging_timer.expires += wait;
		charging_timer.data += 1;

		add_timer(&charging_timer);
		
		printk("[%s] charging_timer.data = %lx\n", DRIVER_NAME, charging_timer.data);
	}
}


/*
void set_charging_ic_bat_soc(int soc)
{
	bat_soc = soc;
	return ;
}
EXPORT_SYMBOL(set_charging_ic_bat_soc);
*/

enum power_supply_type get_charging_ic_status(void)
{
	return charging_ic_status;
}
EXPORT_SYMBOL(get_charging_ic_status);

void charging_ic_active_default()
{
	u32 wait;

	if(charging_ic_status == POWER_SUPPLY_TYPE_USB)
	{
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}

	if( charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		charging_ic_deactive();
	}

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	//udelay(CHR_IC_DEALY);
	udelay(CHR_IC_SET_DEALY); // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-06-30, changed delay time

	charging_ic_status = POWER_SUPPLY_TYPE_USB;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();
	
// [jongho3.lee@lge.com] charging timer setting
	//wait = (HZ*CHR_TIMER_SECS * (100 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-7, changed charging timer
	//wait = HZ*CHR_RESET_SAFETY_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-08-30, changed charging timer for test

	init_timer(&charging_timer);
	charging_timer.data = 0;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting

	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
}
EXPORT_SYMBOL(charging_ic_active_default);

void charging_ic_set_ta_mode()
{
	u32 wait;

	if(charging_ic_status == POWER_SUPPLY_TYPE_MAINS)
	{
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}

	if( charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		charging_ic_deactive();
	}

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	
	msleep(CHR_IC_REINIT_MS_DELAY); // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-09-03, Add delay time cause recharging fail. ( > 32mSec in reconfiguration )
	
	//udelay(CHR_IC_DEALY);
	udelay(CHR_IC_SET_DEALY); // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-06-30, changed delay time

	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gague
	bat_soc = get_bat_soc();
	
// [jongho3.lee@lge.com] charging timer setting
	//wait = (HZ*CHR_TIMER_SECS * (100 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-7, changed charging timer
	//wait = HZ*CHR_RESET_SAFETY_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-08-30, changed charging timer for test

	init_timer(&charging_timer);
	charging_timer.data = 0;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting
	
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
}
EXPORT_SYMBOL(charging_ic_set_ta_mode);

void charging_ic_set_usb_mode()
{
	charging_ic_active_default();
}
EXPORT_SYMBOL(charging_ic_set_usb_mode);

void charging_ic_set_factory_mode()
{
	u32 wait;

	if(charging_ic_status == POWER_SUPPLY_TYPE_FACTORY)
	{
		D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
		return;
	}

	if( charging_ic_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		charging_ic_deactive();
	}

	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	//udelay(CHR_IC_DEALY);
	udelay(CHR_IC_SET_DEALY); // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-06-30, changed delay time
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 1);
	udelay(CHR_IC_DEALY);
	gpio_set_value(CHG_EN_SET_N_OMAP, 0);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_FACTORY;

	mutex_unlock(&charging_lock);

	// [jongho3.lee@lge.ocm] FIXME : get battery SOC from fuel gauge
	bat_soc = get_bat_soc();
	
// [jongho3.lee@lge.com] charging timer setting
	//wait = (HZ*CHR_TIMER_SECS * (100 - bat_soc)) / 100;
	wait = HZ*CHR_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-7, changed charging timer
	//wait = HZ*CHR_RESET_SAFETY_TIMER_SECS; // LGE_CHANGE [byoungcheol.lee@lge.com]  2011-08-30, changed charging timer for test

	init_timer(&charging_timer);
	charging_timer.data = 0;
	charging_timer.expires = jiffies + wait;
	charging_timer.function = charging_timer_func;
	add_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting
	
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
}
EXPORT_SYMBOL(charging_ic_set_factory_mode);

// LGE_CHANGE [byoungcheol.lee@lge.com] 2011-11-22, Add battery present for Factory mode.
extern int get_bat_present(void);

void charging_ic_deactive()
{
	static int temp_deactive_cnt = 1;

	printk("[CHG] charging_ic_status = %d\n", charging_ic_status);

	if(temp_deactive_cnt == 1)
	{
		temp_deactive_cnt = 0;
	}
	else
	{
		if((charging_ic_status == POWER_SUPPLY_TYPE_BATTERY) ||
		  ((charging_ic_status == POWER_SUPPLY_TYPE_FACTORY)
		    && !get_bat_present()))
		{
			D("[charger_rt9524]:: it's already  %s mode!! \n", __func__);
			return;
		}
	}
	
	mutex_lock(&charging_lock);

	gpio_set_value(CHG_EN_SET_N_OMAP, 1);

	udelay(CHR_IC_SET_DEALY);

	charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;

	mutex_unlock(&charging_lock);

// [jongho3.lee@lge.com] charging timer setting
	del_timer(&charging_timer);
// [jongho3.lee@lge.com] charging timer setting

	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
}
EXPORT_SYMBOL(charging_ic_deactive);

#if 0
static irqreturn_t charging_ic_interrupt_handler(int irq, void *data)
{
	struct delayed_work* charger_work;


	charger_work = get_charger_work();

	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);

	schedule_delayed_work(charger_work, 0);


	return IRQ_HANDLED;
}
#endif

static void charging_timer_work(struct work_struct *work)
{
	charger_fsm(CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED);
}

// LGE_CHANGE [euiseop.shin@lge.com] 2011-04-07,P940 Battery Charger Mode [START_LGE]
void set_boot_charging_mode(int charging_mode)
{
	charging_ic_status = charging_mode;
}
EXPORT_SYMBOL(set_boot_charging_mode);
// LGE_CHANGE [euiseop.shin@lge.com] 2011-04-07,P940 Battery Charger Mode [END_LGE]

static int charging_ic_probe(struct platform_device *dev)
{
	int ret = 0;

	ret = gpio_request(CHG_EN_SET_N_OMAP, "charging_ic_en");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for charging_ic\n", __func__, CHG_EN_SET_N_OMAP);
		return -ENOSYS;
	}
	gpio_direction_output(CHG_EN_SET_N_OMAP, 0);

	ret = gpio_request(CHG_STATUS_N_OMAP, "charging_ic_status");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for charging_ic_status\n", __func__, CHG_STATUS_N_OMAP);
		goto err_gpio_request_failed;
	}
	gpio_direction_input(CHG_STATUS_N_OMAP);

	INIT_DELAYED_WORK_DEFERRABLE(&charger_timer_work,
				charging_timer_work);

	printk("[%s]:: charging_ic Initialization is done\n", DRIVER_NAME);

	return 0;

#if 0
err_request_irq_failed:
	gpio_free(CHG_STATUS_N_OMAP);
#endif
err_gpio_request_failed:
	gpio_free(CHG_EN_SET_N_OMAP);

	printk("[%s]:: Charging IC probe failed\n", DRIVER_NAME);
	return ret;
}

static int charging_ic_remove(struct platform_device *dev)
{
	charging_ic_deactive();

	free_irq(CHG_STATUS_N_OMAP, NULL);

	gpio_free(CHG_EN_SET_N_OMAP);
	gpio_free(CHG_STATUS_N_OMAP);

	return 0;
}

static int charging_ic_suspend(struct platform_device *dev, pm_message_t state)
{
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
	dev->dev.power.power_state = state;
	return 0;
}

static int charging_ic_resume(struct platform_device *dev)
{
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
	dev->dev.power.power_state = PMSG_ON;
	return 0;
}

static struct platform_driver charging_ic_driver = {
	.probe = charging_ic_probe,
	.remove = charging_ic_remove,
	.suspend = charging_ic_suspend,
	.resume= charging_ic_resume,
	.driver = {
		.name = "cosmo_charger",
	},
};

static int __init charging_ic_init(void)
{
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
	return platform_driver_register(&charging_ic_driver);
}

static void __exit charging_ic_exit(void)
{
	printk("[%s]:: %s: \n", DRIVER_NAME, __func__);
	platform_driver_unregister(&charging_ic_driver);
}

module_init(charging_ic_init);
module_exit(charging_ic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("charging_ic Driver");
MODULE_LICENSE("GPL");
