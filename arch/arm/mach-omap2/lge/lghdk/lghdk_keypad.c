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
#include <linux/input/matrix_keypad.h>
#include <lge/common.h>

/* LGE_SJIT_S 2011-10-12 [choongryeol.lee@lge.com] FIXME - move to lghdk_xxx file */
static const int lghdk_keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(1, 0, KEY_VOLUMEDOWN),
	KEY(0, 1, KEY_HOME),
	KEY(1, 1, KEY_HOME),
};

static struct matrix_keymap_data lghdk_keymap_data = {
	.keymap			= lghdk_keymap,
	.keymap_size		= ARRAY_SIZE(lghdk_keymap),
};

static struct omap4_keypad_platform_data lghdk_keypad_data = {
	.keymap_data		= &lghdk_keymap_data,
	.rows			= 2,
	.cols			= 2,
};

/* LGE_SJIT_E 2011-10-12 [choongryeol.lee@lge.com] FIXME - move to lghdk_xxx file */

int __init lghdk_keypad_init(void)
{
	return lge_set_keypad_info(&lghdk_keypad_data);
}

lge_machine_initcall(lghdk_keypad_init);
