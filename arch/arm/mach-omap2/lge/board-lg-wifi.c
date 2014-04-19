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
#include <linux/random.h> //for test . Get Random MAC address when booting time.

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#endif

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


// For broadcom
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;
static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc (65536, GFP_KERNEL);
	if(!wlan_static_scan_buf1)
		goto err_mem_alloc;

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

static int bcm_wifi_get_mac_addr(unsigned char* buf)
{
    uint rand_mac;
    static unsigned char mymac[6] = {0,};
    const unsigned char nullmac[6] = {0,};
    pr_debug("%s: %p\n", __func__, buf);

    printk("[%s] Entering...in Board-l1-mmc.c\n", __func__  );

    if( buf == NULL ) return -EAGAIN;

    if( memcmp( mymac, nullmac, 6 ) != 0 )
    {
        /* Mac displayed from UI are never updated..
           So, mac obtained on initial time is used */
        memcpy( buf, mymac, 6 );
        return 0;
    }


    srandom32((uint)jiffies);
    rand_mac = random32();
    buf[0] = 0x00;
    buf[1] = 0x90;
    buf[2] = 0x4c;
    buf[3] = (unsigned char)rand_mac;
    buf[4] = (unsigned char)(rand_mac >> 8);
    buf[5] = (unsigned char)(rand_mac >> 16);

    memcpy( mymac, buf, 6 );

    printk("[%s] Exiting. MyMac :  %x : %x : %x : %x : %x : %x \n",__func__ , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5] );

    return 0;
}

struct wifi_platform_data lg_wifi_control = {
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= brcm_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
	.set_power	= lg_wifi_power,
	.set_reset	= lg_wifi_reset,
	.set_carddetect	= lg_wifi_set_carddetect,
    .get_mac_addr   = bcm_wifi_get_mac_addr, // Get custom MAC address
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource lg_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= OMAP_GPIO_IRQ(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP),
		.end		= OMAP_GPIO_IRQ(CONFIG_BCMDHD_GPIO_WL_HOSTWAKEUP),
#if defined(CONFIG_LGE_BCM433X_PATCH)
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
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&lg_wifi_device);
#endif
out:
	return ret;
}

device_initcall(lg_wifi_init);
