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
		.mmc		=	2,
		.caps		=	MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA |
					MMC_CAP_1_8V_DDR,
		.gpio_cd	=	-EINVAL,
		.gpio_wp	=	-EINVAL,
		.ocr_mask	=	MMC_VDD_165_195,
		.nonremovable	=	true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	=	true,
#endif
	},
	{
		.mmc		=	1,
		.caps		=	MMC_CAP_4_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	=	40,
		.gpio_wp	=	-EINVAL,
#ifdef CONFIG_MACH_LGE_MMC_COVER
					.sd_cover	=	42, 	
#endif	
		.nonremovable	=	false,
#ifdef CONFIG_MACH_LGE_COSMO //nthyuinjin.yang 120428 temp modification for SD card probe.
		.ocr_mask	=	MMC_VDD_29_30,
#else
		.ocr_mask	=	MMC_VDD_32_33,
#endif
#ifdef CONFIG_PM_RUNTIME
		.power_saving	=	true,
#endif
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable	= true,
		.no_suspend	=	true,
	},
	{}	/* Terminator */
};

int __init cosmo_mmc_init(void)
{
	return lge_set_hsmmc_info(&mmc[0]);
}

lge_machine_initcall(cosmo_mmc_init);
