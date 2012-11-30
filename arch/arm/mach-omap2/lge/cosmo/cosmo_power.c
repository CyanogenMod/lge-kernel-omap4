/*
 * Power restart function for COSMO 
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * Author: Do-Yeob Kim <doyeob.kim@lge.com>
 *         Jugwan Eom <jugwan.eom@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <lge/common.h>
#include <lge/board.h>
#include <plat/lge_nvdata_handler.h>
#include <linux/i2c/twl.h>
extern void lge_user_reset();

static int cosmo_reboot_notify(struct notifier_block *nb,
		unsigned long code, void *data)
{
	if (code == SYS_RESTART) {
		/* To clear display during reboot */
		gpio_set_value(GPIO_LCD_RESET, 0);
		mdelay(5);
		gpio_set_value(GPIO_LCD_POWER_EN, 0);

        	/* We still need this with the old bl... */
		lge_user_reset();
		twl_i2c_write_u8(0x14, 0x47, 0x06);
	} else {
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER, 0x07, 0x06);
        }
	return NOTIFY_DONE;
}

static struct notifier_block cosmo_reboot_notifier = {
	.notifier_call = cosmo_reboot_notify,
};

#if 0
static void cosmo_pm_restart(char str, const char *cmd)
{
	/* Do nothing */
}
#endif

int __init pm_restart_init(void)
{
#if 0
	lge_set_pm_restart(cosmo_pm_restart);
#else
	 
	return register_reboot_notifier(&cosmo_reboot_notifier);
#endif
}

lge_machine_initcall(pm_restart_init);
