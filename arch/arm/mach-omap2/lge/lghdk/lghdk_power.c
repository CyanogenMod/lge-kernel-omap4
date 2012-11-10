/*
 * Power restart function for Cosmopolitan
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c/twl.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/syscalls.h>
#include <lge/common.h>


//#define USE_HW_RESET
static int lghdk_reboot_notify(struct notifier_block *nb,
		unsigned long code, void *data)
{
#ifdef USE_HW_RESET
	if (code == SYS_RESTART) {
		/* LGE_CHANGE_S [Darren.Kang@lge.com] 2011-01-06,
		  Cosmopolitan: Changed for HW reset[ST] */
		static const char NVDATA_PARTITION[]  = "/dev/block/platform/mmci-omap-hs.1/by-name/nv";
		static const int  NVDATA_RESET_OFFSET  = 100;

		char nvdata_reset_buffer[2] = { 0xDA };

		int h_file = 0;

		set_fs(KERNEL_DS);
		h_file = sys_open(NVDATA_PARTITION, O_RDWR,0);

		if (h_file >= 0) {
			sys_lseek(h_file, NVDATA_RESET_OFFSET, 0 );

			if (sys_write(h_file, nvdata_reset_buffer, 1) != 1) {
				printk("Can't write Reset Bit.\n");
			}
			sys_close(h_file);
		} else {
			printk("Can't open NVDATA partition ret = %d.\n",h_file);
		}
		/* LGE_CHANGE_E [Darren.Kang@lge.com] 2011-01-06 */
	}
#endif
	return NOTIFY_DONE;
}

static struct notifier_block lghdk_reboot_notifier = {
	.notifier_call = lghdk_reboot_notify,
};

static void lghdk_pm_restart(char str, const char *cmd)
{
	/* LGE_CHANGE [Darren.Kang@lge.com] 2011-01-06,
	  Cosmopolitan: Changed for HW reset[ST] */
	twl_i2c_write_u8(TWL_MODULE_PM_MASTER, 0x47, 0x06);
}

static int lghdk_panic_notify(struct notifier_block *n, unsigned long val, void *v)
{
//	resume_console();
	machine_restart("lge.kpanic");
	return 0;
}

static struct notifier_block lghdk_panic_notifier = {
	.notifier_call = lghdk_panic_notify,
};

int __init pm_restart_init(void)
{
#ifdef USE_HW_RESET
	lge_set_pm_restart(lghdk_pm_restart);
#endif
	atomic_notifier_chain_register(&panic_notifier_list, &lghdk_panic_notifier);
	return register_reboot_notifier(&lghdk_reboot_notifier);
}

lge_machine_initcall(pm_restart_init);
