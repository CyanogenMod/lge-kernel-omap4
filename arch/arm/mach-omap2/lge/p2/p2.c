/*
 * Machine descript file for LGE P2 Board.
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

#ifdef CONFIG_MACH_LGE_P2_P940
MACHINE_START(LGE_P2, "LGE P940 board")
#elif defined(CONFIG_MACH_LGE_P2_SU540)
MACHINE_START(LGE_P2, "LGE SU540 board")
#elif defined(CONFIG_MACH_LGE_P2_KU5400)
MACHINE_START(LGE_P2, "LGE KU5400 board")
#elif defined(CONFIG_MACH_LGE_P2_LU5400)
MACHINE_START(LGE_P2, "LGE LU5400 board")
#elif defined(CONFIG_MACH_LGE_P2_DCM)
MACHINE_START(LGE_P2, "LGE DCM board")
#else
#error "Unknown P2 board"
#endif
	.boot_params	= 0x80000100,
	.reserve	= lge_common_reserve,
	.map_io		= lge_common_map_io,
	.init_early	= lge_common_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= lge_common_init,
	.timer		= &lge_common_timer,
MACHINE_END
