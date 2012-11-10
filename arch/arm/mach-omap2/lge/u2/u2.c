/*
 * Machine descript file for LGE U2 Board.
 *
 * Copyright (C) 2011, 2012 LG Electronics, Inc.
 *
 * Author: Jugwan Eom <jugwan.eom@lge.com>
 *
 * Based on mach-omap2/lge/cosmopolitan/cosmopolitan.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <lge/common.h>
#include <mach/omap4-common.h>

#ifdef CONFIG_MACH_LGE_U2_P760
MACHINE_START(LGE_U2, "LGE P760 board")
#elif defined(CONFIG_MACH_LGE_U2_P769)
MACHINE_START(LGE_U2, "LGE P769 board")

#elif defined(CONFIG_MACH_LGE_U2_P768)
MACHINE_START(LGE_U2, "LGE P768 board")

#else
#error "Unknown U2 board"
#endif
	.boot_params	= 0x80000100,
	.reserve	= lge_common_reserve,
	.map_io		= lge_common_map_io,
	.init_early	= lge_common_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= lge_common_init,
	.timer		= &lge_common_timer,
MACHINE_END
