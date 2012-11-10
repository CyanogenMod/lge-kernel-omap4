/*
 * Power restart function for U2
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

static int u2_reboot_notify(struct notifier_block *nb,
		unsigned long code, void *data)
{
	if (code == SYS_RESTART) {

		/* To clear display during reboot */
		gpio_set_value(GPIO_LCD_RESET, 0);
		mdelay(5);
		gpio_set_value(GPIO_LCD_POWER_EN, 0);
	}

	return NOTIFY_DONE;
}

static struct notifier_block u2_reboot_notifier = {
	.notifier_call = u2_reboot_notify,
};

#if 0
static void u2_pm_restart(char str, const char *cmd)
{
	/* Do nothing */
}
#endif

int __init pm_restart_init(void)
{
#if 0
	lge_set_pm_restart(u2_pm_restart);
#else

	return register_reboot_notifier(&u2_reboot_notifier);
#endif
}

lge_machine_initcall(pm_restart_init);
