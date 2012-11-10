/*
 * Board support file for containing WiFi specific details for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Pradeep Gurumath <pradeepgurumath@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* linux/arch/arm/mach-omap2/board-4430sdp-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#include <mach-omap2/mux.h> 
#include <lge/board.h>

static int lg_wifi_cd;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;


int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;

	wifi_status_cb_devid = dev_id;

	return 0;
}

int omap_wifi_status(struct device *dev, int slot)
{
	return lg_wifi_cd;
}

int lg_wifi_set_carddetect(int val)
{
	printk(KERN_WARNING"%s: %d\n", __func__, val);
	lg_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(lg_wifi_set_carddetect);
#endif

static int lg_wifi_power_state;

int lg_wifi_power(int on)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);
	gpio_set_value(CONFIG_BCMDHD_GPIO_WL_RESET, on);
	lg_wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(lg_wifi_power);
#endif

static int lg_wifi_reset_state;
int lg_wifi_reset(int on)
{
	printk(KERN_WARNING"%s: %d\n", __func__, on);
	lg_wifi_reset_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(lg_wifi_reset);
#endif

#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BCMDHD_USE_STATIC_BUF)
extern void* mem_prealloc( int section, unsigned long size);
#endif

struct wifi_platform_data lg_wifi_control = {
	.set_power	= lg_wifi_power,
	.set_reset	= lg_wifi_reset,
	.set_carddetect	= lg_wifi_set_carddetect,
#if defined(CONFIG_LGE_BCM432X_PATCH) && defined(CONFIG_BCMDHD_USE_STATIC_BUF)
	.mem_prealloc   = mem_prealloc,
#endif
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource lg_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= OMAP_GPIO_IRQ(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP),
		.end		= OMAP_GPIO_IRQ(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP),
#if defined(CONFIG_LGE_BCM432X_PATCH)
                .flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
#else
                .flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
#endif

	},
};

static struct platform_device lg_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(lg_wifi_resources),
	.resource       = lg_wifi_resources,
	.dev            = {
		.platform_data = &lg_wifi_control,
	},
};
#endif

static int __init lg_wifi_init(void)
{
	int ret;

	printk(KERN_WARNING"%s: start\n", __func__);

	ret = gpio_request(CONFIG_BCMDHD_GPIO_WL_RESET, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			CONFIG_BCMDHD_GPIO_WL_RESET);
		goto out;
	}
	gpio_direction_output(CONFIG_BCMDHD_GPIO_WL_RESET, 0);

	ret = gpio_request(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP);
		goto out;
	}
	gpio_direction_input(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP);
#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&lg_wifi_device);
#endif
out:
	return ret;
}

device_initcall(lg_wifi_init);
