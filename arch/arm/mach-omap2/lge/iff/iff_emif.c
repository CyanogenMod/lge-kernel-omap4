/*
 * emif initializing code.
 *
 * Copyright (C) 2010 LG Electronic Inc.
 *
 * Author: Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/lpddr2-samsung.h>
#include <lge/common.h>

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :

 * 	EMIF1 - CS0 -	4 Gb
 *	EMIF2 - CS0 -	4 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 * Same devices installed on EMIF1 and EMIF2
 */
static struct lge_emif_devices emif_devices __initdata = {
	.emif1_devices = {
		.cs0_device = &samsung_4G_S4,
		.cs1_device = NULL
	},
	.emif2_devices = {
		.cs0_device = &samsung_4G_S4,
		.cs1_device = NULL
	}
}; 

int __init lge_emif_init(void)
{
	return lge_set_emif_devices(&emif_devices);
}

lge_machine_initcall(lge_emif_init);
