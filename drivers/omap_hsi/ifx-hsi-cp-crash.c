
/* ifx-hsi-cp-crash.c
 *
 * Copyright (C) 2010 LGE. All rights reserved.
 * Author: Jaesung.woo <jaesung.woo@lge.com>
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

#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <mach/gpio.h>
#include <plat/lge_err_handler.h>
#include <plat/lge_nvdata_handler.h>

#include "hsi_driver.h"

#include "xmd-ch.h"

#define CP_CRASH_INT_N  			26

static struct delayed_work	cp_crash_int_delayed_wq;


#ifndef ENABLE_CP_CRASH_RESET
#define EVENT_KEY 				KEY_F24
static struct input_dev *in_dev = NULL;
#endif

static void CP_CRASH_wq_func(struct work_struct *cp_crash_wq);
static irqreturn_t CP_CRASH_interrupt_handler(s32 irq, void *data);


int IFX_CP_CRASH_DUMP_INIT(void)
{
	int ret = 0;
	
	/* Assign GPIO	*/
	ret = gpio_request(CP_CRASH_INT_N, "CP CRASH IRQ GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d is already occupied by other driver!\n", CP_CRASH_INT_N);
		return -ENOSYS;
	}

	/* Initializes GPIO direction before use or IRQ setting */
	ret = gpio_direction_input(CP_CRASH_INT_N);
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d direction initialization failed!\n", CP_CRASH_INT_N);
		return -ENOSYS;
	}

	/* Registers MUIC work queue function */
	INIT_DELAYED_WORK(&cp_crash_int_delayed_wq, CP_CRASH_wq_func);

	/* 
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
	ret = request_irq(gpio_to_irq(CP_CRASH_INT_N), CP_CRASH_interrupt_handler, IRQF_TRIGGER_RISING, "cp_crash_irq", NULL);
	if (ret < 0) {
		printk(KERN_INFO "[CP CRASH IRQ] GPIO#%03d IRQ line set up failed!\n", CP_CRASH_INT_N);
		free_irq(gpio_to_irq(CP_CRASH_INT_N), NULL);
		return -ENOSYS;
	}

#ifndef ENABLE_CP_CRASH_RESET
	in_dev = input_allocate_device();
	if (!in_dev) {
		printk("Can't allocate power button\n");
		return -ENOMEM;
	}

	in_dev->evbit[0] = BIT_MASK(EV_KEY);
	in_dev->keybit[BIT_WORD(EVENT_KEY)] = BIT_MASK(EVENT_KEY);
	in_dev->name = "hsi";
	in_dev->phys = "hsi/input0";
	in_dev->dev.parent = NULL;

	ret = input_register_device(in_dev);
	if (ret) {
		printk("Can't register EVENT_KEY button: %d\n", ret);
	}
#endif //ENABLE_CP_CRASH_RESET
	
	return ret;

}


static void CP_CRASH_wq_func(struct work_struct *cp_crash_wq)
{
	int ret;
	unsigned char data;
	struct timespec ts;
	struct rtc_time tm;

	char* argv[] = {"/system/bin/ifx_coredump", "CP_CRASH_IRQ", NULL};
	char *envp[] = { "HOME=/",	"PATH=/sbin:/bin:/system/bin",	NULL };	

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	printk(KERN_INFO "[CP CRASH IRQ] CP_CRASH_wq_func()\n");	
	printk(KERN_INFO "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);	
	
	if(gpio_get_value(CP_CRASH_INT_N))
	{
		lge_store_ciq_reset(0, LGE_NVDATA_IQ_RESET_EXCEPTION);
		
		// UPDATE CP_CRASH_COUNT 
		lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CP_CRASH_COUNT_OFFSET, &data, 1);
		data++;
		lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_CP_CRASH_COUNT_OFFSET, &data, 1);

		// CHECK CP_CRASH_DUMP OPTION
		if (lge_is_crash_dump_enabled() != 1)
		{	
#ifndef ENABLE_CP_CRASH_RESET	// LGE_RIL_RECOVERY
			printk(" CP CRASH! immediate RIL/CP reset");
			input_report_key(in_dev, EVENT_KEY, 1);
			input_report_key(in_dev, EVENT_KEY, 0);
			input_sync(in_dev);
			printk("[CPW] input_report_key(): %d\n", EVENT_KEY);
#endif
			return;
		}

		ret = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_EXEC);

		printk(KERN_INFO "[CP CRASH IRQ] launch ifx_coredump process ret:%d\n", ret);

		gpio_set_value(82, 1);
	}
	else
	{
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]	
#if 0	
		/*****************************************************************************
			1. Case of HSI_LL_MSG_BREAK in hsi_ll_read_complete_cb
			2. Case of invalid packet in Rmnet from CP (becasue of very low battery and etc)
		*****************************************************************************/
		printk(KERN_INFO "[CP CRASH IRQ] CP_CRASH_wq_func() - CP_CRASH_INT_N - Invaild\n");

#ifndef ENABLE_CP_CRASH_RESET	// LGE_RIL_RECOVERY
		printk(" CP CRASH! immediate RIL/CP reset");
		input_report_key(in_dev, EVENT_KEY, 1);
		input_report_key(in_dev, EVENT_KEY, 0);
		input_sync(in_dev);
		printk("[CPW] input_report_key(): %d\n", EVENT_KEY);
#endif
#endif		
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
	}
}

void ifx_schedule_cp_dump_or_reset(void)
{
	/* Make the interrupt on CP CRASH INT wake up OMAP which is in suspend mode */
	schedule_delayed_work( &cp_crash_int_delayed_wq, msecs_to_jiffies(500));
}

static irqreturn_t CP_CRASH_interrupt_handler(s32 irq, void *data)
{
	ifx_schedule_cp_dump_or_reset();
	return IRQ_HANDLED;
}

