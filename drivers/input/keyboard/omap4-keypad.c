/*
 * OMAP4 Keypad Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Author: Abraham Arce <x0066660@ti.com>
 * Initial Code: Syed Rafiuddin <rafiuddin.syed@ti.com>
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <plat/omap4-keypad.h>
/* LGE_SJIT 2011-12-06 [dojip.kim@lge.com] export input handle */
#include <linux/lge/lge_input.h>
/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] add wakelock */
#include <linux/wakelock.h>

/* OMAP4 registers */
#define OMAP4_KBD_REVISION		0x00
#define OMAP4_KBD_SYSCONFIG		0x10
#define OMAP4_KBD_SYSSTATUS		0x14
#define OMAP4_KBD_IRQSTATUS		0x18
#define OMAP4_KBD_IRQENABLE		0x1C
#define OMAP4_KBD_WAKEUPENABLE		0x20
#define OMAP4_KBD_PENDING		0x24
#define OMAP4_KBD_CTRL			0x28
#define OMAP4_KBD_DEBOUNCINGTIME	0x2C
#define OMAP4_KBD_LONGKEYTIME		0x30
#define OMAP4_KBD_TIMEOUT		0x34
#define OMAP4_KBD_STATEMACHINE		0x38
#define OMAP4_KBD_ROWINPUTS		0x3C
#define OMAP4_KBD_COLUMNOUTPUTS		0x40
#define OMAP4_KBD_FULLCODE31_0		0x44
#define OMAP4_KBD_FULLCODE63_32		0x48

/* OMAP4 bit definitions */
#define OMAP4_DEF_IRQENABLE_EVENTEN	(1 << 0)
#define OMAP4_DEF_IRQENABLE_LONGKEY	(1 << 1)
#define OMAP4_DEF_IRQENABLE_TIMEOUTEN	(1 << 2)
#define OMAP4_DEF_WUP_EVENT_ENA		(1 << 0)
#define OMAP4_DEF_WUP_LONG_KEY_ENA	(1 << 1)
#define OMAP4_DEF_WUP_TIMEOUTEN_ENA      (1 << 2)
#define OMAP4_DEF_CTRL_NOSOFTMODE	(1 << 1)
#define OMAP4_DEF_CTRL_PTV		(1 << 2)

#define OMAP4_DEF_REPEAT_MODE		(1 << 8)
#define OMAP4_DEF_TIMEOUT_LONG_KEY      (1 << 7)
#define OMAP4_DEF_TIMEOUT_EMPTY		(1 << 6)
#define OMAP4_DEF_LONG_KEY		(1 << 5)

/* OMAP4 values */
#define OMAP4_VAL_IRQDISABLE		0x00
/* DEBOUNCE TIME VALUE = 0x2 PVT = 0x6  Tperiod = 12ms*/
#define OMAP4_VAL_DEBOUNCINGTIME	0x2
#define OMAP4_VAL_PVT			0x6


#define OMAP4_MASK_IRQSTATUSDISABLE	0xFFFF

/* LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add debug_mask to check the H/W status for keypad immediately
 * echo 1 > sys/devices/platform/omap/omap4-keypad/keypad_debug
 */
u32 debug_mask = 0;

#ifdef CONFIG_MACH_LGE_COSMO
//mo2haewoon.you@lge.com => [START] keylock command
int atcmd_keylock=0;
//mo2haewoon.you@lge.com => [END]
#endif

struct omap4_keypad {
	struct input_dev *input;

	void __iomem *base;
	int irq;

	unsigned int rows;
	unsigned int cols;
	unsigned int row_shift;
	unsigned char key_state[8];
	void (*keypad_pad_wkup)(int enable);
	/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] Add wakelock */
	struct wake_lock wlock;
	unsigned short keymap[];
};

/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] for Android SafeMode
 * [yehan.ahn@lge.com] 2011-06-10, [P940] for enable the saving-mode
 */
#ifdef CONFIG_KEYBOARD_OMAP4_SAFEMODE
static int safemode_key = 0;

static ssize_t show_safemode_key(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", safemode_key);
}
DEVICE_ATTR(key_saving, 0660, show_safemode_key, NULL);
#endif

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add debug_mask to check the H/W status for keypad immediately
static ssize_t show_keypad_debug_mask(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_mask);
}

static ssize_t store_keypad_debug_mask(struct device *dev,
		struct device_attribute *attr, char *buf, size_t count)
{
	debug_mask = simple_strtoul(buf, NULL, 10);
	printk("%s: debug_mask is set to %d\n", __func__, debug_mask);

	return count;
}

static DEVICE_ATTR(keypad_debug, S_IWUSR | S_IRUGO | S_IRGRP | S_IROTH,
			show_keypad_debug_mask, store_keypad_debug_mask);
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-06

//mo2haewoon.you@lge.com => [START] keylock command
#ifdef CONFIG_MACH_LGE_COSMO
//LGE_S 11.12.13 <ntdeaewan.choi@lge.com> AT%KEYLOCK touch,keytouch,key disable
static ssize_t keypad_keylock_show(struct device *dev,  struct device_attribute *attr,  char *buf)
{
	int r = 0;

	r += sprintf(buf+r, "%d", atcmd_keylock);

	return r;
}

static ssize_t keypad_keylock_store(struct device *dev,  struct device_attribute *attr,  const char *buf, size_t count)
{
    int ret;

    ret = sscanf(buf, "%d", &atcmd_keylock);

	printk("keypad_keylock_store = [%d]\n", atcmd_keylock);

	if(atcmd_keylock == 1)
	{
		printk("[KEYPAD] AT%KEYLOCK ON.\n");
	}
	else if(atcmd_keylock == 0)
	{
		printk("[KEYPAD] AT%KEYLOCK OFF.\n");
	}

	return ret;
}
static DEVICE_ATTR(keylock, 0665, keypad_keylock_show, keypad_keylock_store);
//mo2haewoon.you@lge.com <= [END] 
#endif

/* Interrupt handler */
static irqreturn_t omap4_keypad_interrupt(int irq, void *dev_id)
{
	struct omap4_keypad *keypad_data = dev_id;
	struct input_dev *input_dev = keypad_data->input;
	unsigned char key_state[ARRAY_SIZE(keypad_data->key_state)];
	unsigned int col, row, code, changed;
	u32 *new_state = (u32 *) key_state;

	/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] wake lock from P940 GB
	 * 2011.12.07 jaekyung.oh@lge.com For Volume Control.
	 */
	wake_lock_timeout(&keypad_data->wlock, 1 * HZ);

	*new_state = __raw_readl(keypad_data->base + OMAP4_KBD_FULLCODE31_0);
	*(new_state + 1) = __raw_readl(keypad_data->base
						+ OMAP4_KBD_FULLCODE63_32);

	// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add to check H/W status
	if(debug_mask) {
		printk("========================================================\n");
		printk("%s: [%#x][%#x]\n", __func__, *new_state, *(new_state+1));
		printk("========================================================\n");
	}
	// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-06

	for (col = 0; col < keypad_data->cols; col++) {
		changed = key_state[col] ^ keypad_data->key_state[col];

		if (!changed)
			continue;
		for (row = 0; row < keypad_data->rows; row++) {
			if (changed & (1 << row)) {
				code = MATRIX_SCAN_CODE(row, col,
						keypad_data->row_shift);

				// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add to check H/W status
				if(debug_mask) {
					printk("%s: [changed][col][row][code] = [%#x][%d][%d][%d]\n", __func__, changed, col, row, code);
					printk("========================================================\n");
				}
				// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-06

                                //mo2haewoon.you@lge.com => [START]  keylock command
#ifdef CONFIG_MACH_LGE_COSMO
				if( keypad_data->keymap[code] && !atcmd_keylock) {
#else
				if( keypad_data->keymap[code] ) {
#endif
                                //mo2haewoon.you@lge.com <= [END]
				    input_event(input_dev, EV_MSC, MSC_SCAN, code);
				    input_report_key(input_dev,
                            keypad_data->keymap[code],
                            (bool)(key_state[col] & (1 << row)));

#ifdef CONFIG_MACH_LGE_U2	/* seungbum.park@lge.com - 2012/05/21 - the HOME_key is added */
                    printk("[omap4-keypad] %s KEY %s\n",
                                                (keypad_data->keymap[code] == KEY_VOLUMEUP) ? "Vol_UP" : ((keypad_data->keymap[code] == KEY_VOLUMEDOWN) ? "Vol_DOWN" : "HOME"),
                                                (key_state[col] & (1 << row)) ? "PRESS" : "RELEASE" );
#else
                    printk("[omap4-keypad] %s KEY %s\n",
						(keypad_data->keymap[code] == KEY_VOLUMEUP) ? "Vol_UP" : ((keypad_data->keymap[code] == KEY_VOLUMEDOWN) ? "Vol_DOWN" : "CAPTURE"),
						(key_state[col] & (1 << row)) ? "PRESS" : "RELEASE" );
#endif

#ifdef CONFIG_INPUT_LGE_GKPD
                    gkpd_report_key(keypad_data->keymap[code], (bool)(key_state[col] & (1 << row)));
#endif

                    break;
				}

				/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com]
				 * for Android SafeMode
				 * [yehan.ahn@lge.com] 2011-06-10,
				 * [P940] for enable the saving-mode
				 */
#ifdef CONFIG_KEYBOARD_OMAP4_SAFEMODE
				if (keypad_data->keymap[code] == KEY_VOLUMEUP) {
					safemode_key = !!(key_state[col] & (1 << row));
				}
#endif
			}
		}
	}

	input_sync(input_dev);

	memcpy(keypad_data->key_state, key_state,
		sizeof(keypad_data->key_state));

	/* clear pending interrupts */
	__raw_writel(__raw_readl(keypad_data->base + OMAP4_KBD_IRQSTATUS),
			keypad_data->base + OMAP4_KBD_IRQSTATUS);


	printk("#################################### %s is finished!!!!!\n", __func__);
	return IRQ_HANDLED;
}

static int omap4_keypad_open(struct input_dev *input)
{
	struct omap4_keypad *keypad_data = input_get_drvdata(input);

#ifdef KBD_DEBUG
	printk("omap4-keypad: omap4_keypad_open \n");
#endif

	pm_runtime_get_sync(input->dev.parent);

	disable_irq(keypad_data->irq);

	__raw_writel(OMAP4_DEF_CTRL_NOSOFTMODE |
			(OMAP4_VAL_PVT << OMAP4_DEF_CTRL_PTV),
			keypad_data->base + OMAP4_KBD_CTRL);

	__raw_writel(OMAP4_VAL_DEBOUNCINGTIME,
			keypad_data->base + OMAP4_KBD_DEBOUNCINGTIME);

	/* Enable event IRQ*/
	__raw_writel(OMAP4_DEF_IRQENABLE_EVENTEN,
			keypad_data->base + OMAP4_KBD_IRQENABLE);

	/* Enable event wkup*/
	__raw_writel(OMAP4_DEF_WUP_EVENT_ENA,
			keypad_data->base + OMAP4_KBD_WAKEUPENABLE);

	/* clear pending interrupts */
	__raw_writel(__raw_readl(keypad_data->base + OMAP4_KBD_IRQSTATUS),
			keypad_data->base + OMAP4_KBD_IRQSTATUS);
	enable_irq(keypad_data->irq);

	return 0;
}

static void omap4_keypad_close(struct input_dev *input)
{
	struct omap4_keypad *keypad_data = input_get_drvdata(input);

	disable_irq(keypad_data->irq);

	/* Disable interrupts */
	__raw_writel(OMAP4_VAL_IRQDISABLE,
		     keypad_data->base + OMAP4_KBD_IRQENABLE);

	/* clear pending interrupts */
	__raw_writel(__raw_readl(keypad_data->base + OMAP4_KBD_IRQSTATUS),
			keypad_data->base + OMAP4_KBD_IRQSTATUS);

	enable_irq(keypad_data->irq);

#ifdef KBD_DEBUG
	printk("omap4-keypad: omap4_keypad_close \n");
#endif

	pm_runtime_put_sync(input->dev.parent);
}

static int __devinit omap4_keypad_probe(struct platform_device *pdev)
{
	const struct omap4_keypad_platform_data *pdata;
	struct omap4_keypad *keypad_data;
	struct input_dev *input_dev;
	struct resource *res;
	resource_size_t size;
	unsigned int row_shift, max_keys;
	int irq;
	int error;

	/* platform data */
	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no base address specified\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no keyboard irq assigned\n");
		return -EINVAL;
	}

	if (!pdata->keymap_data) {
		dev_err(&pdev->dev, "no keymap data defined\n");
		return -EINVAL;
	}

	row_shift = get_count_order(pdata->cols);
	max_keys = pdata->rows << row_shift;

	keypad_data = kzalloc(sizeof(struct omap4_keypad) +
				max_keys * sizeof(keypad_data->keymap[0]),
			      GFP_KERNEL);
	if (!keypad_data) {
		dev_err(&pdev->dev, "keypad_data memory allocation failed\n");
		return -ENOMEM;
	}

	size = resource_size(res);

	res = request_mem_region(res->start, size, pdev->name);
	if (!res) {
		dev_err(&pdev->dev, "can't request mem region\n");
		error = -EBUSY;
		goto err_free_keypad;
	}

	keypad_data->base = ioremap(res->start, resource_size(res));
	if (!keypad_data->base) {
		dev_err(&pdev->dev, "can't ioremap mem resource\n");
		error = -ENOMEM;
		goto err_release_mem;
	}

	keypad_data->irq = irq;
	keypad_data->row_shift = row_shift;
	keypad_data->rows = pdata->rows;
	keypad_data->cols = pdata->cols;
	keypad_data->keypad_pad_wkup = pdata->keypad_pad_wkup;

	/* input device allocation */
	keypad_data->input = input_dev = input_allocate_device();
	if (!input_dev) {
		error = -ENOMEM;
		goto err_unmap;
	}

	input_dev->name = pdev->name;
	input_dev->dev.parent = &pdev->dev;
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;

	input_dev->open = omap4_keypad_open;
	input_dev->close = omap4_keypad_close;

	input_dev->keycode	= keypad_data->keymap;
	input_dev->keycodesize	= sizeof(keypad_data->keymap[0]);
	input_dev->keycodemax	= max_keys;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_REP, input_dev->evbit);

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_set_drvdata(input_dev, keypad_data);

	matrix_keypad_build_keymap(pdata->keymap_data, row_shift,
			input_dev->keycode, input_dev->keybit);

	/* LGE_SJIT 2011-12-06 [dojip.kim@lge.com] MHL keybits
	 * jk.koo kibu.lee 20110810 MHL RCP codes into media keys and
	 * transfer these to the input manager
	 */
#if defined(CONFIG_MHL_INPUT_RCP)
	hdmi_common_register_keys(input_dev);
#endif
	/* LGE_SJIT 2011-12-09 [dojip.kim@lge.com] for hook key */
#if defined(CONFIG_SND_OMAP_SOC_LGE_JACK)
	__set_bit(KEY_HOOK, input_dev->keybit);
#endif
	/* LGE_SJIT 2011-01-05 [dojip.kim@lge.com] Add wake lock */
	wake_lock_init(&keypad_data->wlock, WAKE_LOCK_SUSPEND, "omap4-keypad");

	/*
	 * Set irq level detection for mpu. Edge event are missed
	 * in gic if the mpu is in low power and keypad event
	 * is a wakeup.
	 */
	error = request_irq(keypad_data->irq, omap4_keypad_interrupt,
			     IRQF_TRIGGER_HIGH,
			     "omap4-keypad", keypad_data);
	if (error) {
		dev_err(&pdev->dev, "failed to register interrupt\n");
		goto err_free_input;
	}
	enable_irq_wake(OMAP44XX_IRQ_KBD_CTL);

	pm_runtime_enable(&pdev->dev);

	error = input_register_device(keypad_data->input);
	if (error < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_pm_disable;
	}

	platform_set_drvdata(pdev, keypad_data);

	/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] for Android SafeMode
	 * [yehan.ahn@lge.com] 2011-06-10, [P940] for enable the saving-mode
	 */
#ifdef CONFIG_KEYBOARD_OMAP4_SAFEMODE
	error = device_create_file(&pdev->dev, &dev_attr_key_saving);
	if (error < 0) {
		dev_warn(&pdev->dev, "failed to create sysfs for key_saving\n");
	}
#endif

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add debug_mask to check the H/W status for keypad immediately
	error = device_create_file(&pdev->dev, &dev_attr_keypad_debug);
	if (error < 0) {
		dev_warn(&pdev->dev, "failed to create sysfs for keypad_debug\n");
	}
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-06

//mo2haewoon.you@lge.com => [START]  keylock command
#ifdef CONFIG_MACH_LGE_COSMO
	error = device_create_file(&pdev->dev, &dev_attr_keylock);
	if (error) {
		printk( "keypad: keylock create file: Fail\n");
		device_remove_file(&pdev->dev, &dev_attr_keylock);
	}
#endif
//mo2haewoon.you@lge.com <= [END]

	/* LGE_SJIT 2011-12-06 [dojip.kim@lge.com] export input handle */
#ifdef CONFIG_MACH_LGE
	lge_input_set(input_dev);
#endif

	return 0;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	free_irq(keypad_data->irq, keypad_data);
	/* LGE_SJIT 2011-01-05 [dojip.kim@lge.com] Add wake lock */
	wake_lock_destroy(&keypad_data->wlock);
err_free_input:
	input_free_device(input_dev);
err_unmap:
	iounmap(keypad_data->base);
err_release_mem:
	release_mem_region(res->start, size);
err_free_keypad:
	kfree(keypad_data);
	return error;
}

static int __devexit omap4_keypad_remove(struct platform_device *pdev)
{
	struct omap4_keypad *keypad_data = platform_get_drvdata(pdev);
	struct resource *res;

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-06 , add debug_mask to check the H/W status for keypad immediately
	device_remove_file(&pdev->dev, &dev_attr_keypad_debug);
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-06

//mo2haewoon.you@lge.com => [START]  keylock command
#ifdef CONFIG_MACH_LGE_COSMO
	device_remove_file(&pdev->dev, &dev_attr_keylock);
#endif
//mo2haewoon.you@lge.com <= [END]

	/* LGE_SJIT 2012-01-05 [dojip.kim@lge.com] for Android SafeMode
	 * [yehan.ahn@lge.com] 2011-06-10, [P940] for enable the saving-mode
	 */
#ifdef CONFIG_KEYBOARD_OMAP4_SAFEMODE
	device_remove_file(&pdev->dev, &dev_attr_key_saving);
#endif

	free_irq(keypad_data->irq, keypad_data);

	/* LGE_SJIT 2011-01-05 [dojip.kim@lge.com] Add wake lock */
	wake_lock_destroy(&keypad_data->wlock);

	pm_runtime_disable(&pdev->dev);

	input_unregister_device(keypad_data->input);

	iounmap(keypad_data->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(keypad_data);
	platform_set_drvdata(pdev, NULL);

	return 0;
}
static int omap4_keypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap4_keypad *keypad_data = platform_get_drvdata(pdev);

	if (keypad_data->keypad_pad_wkup)
		keypad_data->keypad_pad_wkup(1);

	return 0;
}
static int omap4_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap4_keypad *keypad_data = platform_get_drvdata(pdev);

	if (keypad_data->keypad_pad_wkup)
		keypad_data->keypad_pad_wkup(0);

	return 0;
}
static const struct dev_pm_ops omap4_keypad_pm_ops = {
	.suspend = omap4_keypad_suspend,
	.resume = omap4_keypad_resume,
};

static struct platform_driver omap4_keypad_driver = {
	.probe		= omap4_keypad_probe,
	.remove		= __devexit_p(omap4_keypad_remove),
	.driver		= {
		.name	= "omap4-keypad",
		.owner	= THIS_MODULE,
		.pm	= &omap4_keypad_pm_ops,
	},
};

static int __init omap4_keypad_init(void)
{
	return platform_driver_register(&omap4_keypad_driver);
}
module_init(omap4_keypad_init);

static void __exit omap4_keypad_exit(void)
{
	platform_driver_unregister(&omap4_keypad_driver);
}
module_exit(omap4_keypad_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP4 Keypad Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:omap4-keypad");
