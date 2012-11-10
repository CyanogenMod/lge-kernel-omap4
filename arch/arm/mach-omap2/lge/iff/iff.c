/*
 * Machine descript file for LGE IFF Board.
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <lge/common.h>
#include <mach/omap4-common.h>

MACHINE_START(LGE_IFF, "LGE IFF board")
	.boot_params	= 0x80000100,
	.reserve	= lge_common_reserve,
	.map_io		= lge_common_map_io,
	.init_early	= lge_common_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= lge_common_init,
	.timer		= &lge_common_timer,
MACHINE_END
