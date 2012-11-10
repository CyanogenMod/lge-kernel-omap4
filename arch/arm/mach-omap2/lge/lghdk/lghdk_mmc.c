/*
 * MMC controller  initializing.
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/mmc/host.h>
#include <lge/common.h>

struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | 
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable   = true,
		.no_off_init	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA | /* MMC_CAP_8_BIT_DATA */
					MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,	// LGE_SJIT 2011-08-31 [jongrak.kwon@lge.com] Remove unused CD. CD will be handled by TWL6030
		.gpio_wp	= -EINVAL,
		.nonremovable	= false,
		.ocr_mask	= MMC_VDD_32_33,
		.no_off_init	= true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= CONFIG_TIWLAN_MMC_CONTROLLER,
		.caps		= MMC_CAP_4_BIT_DATA, // | MMC_CAP_8_BIT_DATA, yonghoon.lee modified
		.gpio_cd	= -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.no_off_init	= true,
		/* LGE_SJIT 2012-01-09 [dojip.kim@lge.com]
		 * no suspend. WiFi doesn't use pm on MMC
		 */
		.no_suspend	= true,
	},
	{}	/* Terminator */
};

int __init lghdk_mmc_init(void)
{
	return lge_set_hsmmc_info(&mmc[0]);
}

lge_machine_initcall(lghdk_mmc_init);
