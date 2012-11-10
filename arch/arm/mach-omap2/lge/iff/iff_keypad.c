/*
 * OMAP keypad device initializing code.
 *
 * Copyright (C) 2011 LG Electronic Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/input.h>
#include <plat/omap4-keypad.h>
#include <lge/common.h>
#include <lge/board_rev.h>

/* LGE_SJIT 2011-11-17 [dojip.kim@lge.com] modified for omap4-keypad */
/* LGE_SJIT_S 2011-10-12 [choongryeol.lee@lge.com] */
static const int iff_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
};

static struct matrix_keymap_data iff_keymap_data = {
	.keymap			= iff_keymap,
	.keymap_size		= ARRAY_SIZE(iff_keymap),
};

static struct omap4_keypad_platform_data iff_keypad_data = {
	.keymap_data		= &iff_keymap_data,
	.rows			= 2,
	.cols			= 1,
};

/* LGE_SJIT_E 2011-10-12 [choongryeol.lee@lge.com]  */

int __init iff_keypad_init(void)
{
	/* LGE_SJIT 2012-01-13 [dojip.kim@lge.com]
	 * Not use omap keypad driver due to LG HW bug
	 * rev_c uses gpio-keys which is defined in iff_pdev.c
	 */
	if (system_rev == LGE_PCB_C)
		return 0;

	return lge_set_keypad_info(&iff_keypad_data);
}

lge_machine_initcall(iff_keypad_init);
