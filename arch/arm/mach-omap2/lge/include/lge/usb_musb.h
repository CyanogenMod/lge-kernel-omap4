/*
 * LGE common header file
 *
 * Copyright (C) 2010 LG Electronics Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LGE_USB_MUSB_COMMON_H
#define __LGE_USB_MUSB_COMMON_H

#ifdef CONFIG_ANDROID
#define MAX_USB_SERIAL_NUM		17
#define OMAP_VENDOR_ID			0x0451
#define OMAP_UMS_PRODUCT_ID		0xD100
#define OMAP_ADB_PRODUCT_ID		0xD101
#define OMAP_UMS_ADB_PRODUCT_ID		0xD102
#define OMAP_RNDIS_PRODUCT_ID		0xD103
#define OMAP_RNDIS_ADB_PRODUCT_ID	0xD104
#define OMAP_ACM_PRODUCT_ID		0xD105
#define OMAP_ACM_ADB_PRODUCT_ID		0xD106
#define OMAP_ACM_UMS_ADB_PRODUCT_ID	0xD107

#define OMAP_LGE_USB_PDEV_ANDROID		0
#define OMAP_LGE_USB_PDEV_MASS_STORAGE	1
#define OMAP_LGE_USB_PDEV_RNDIS			2

extern char device_serial[MAX_USB_SERIAL_NUM];
#endif

#endif
