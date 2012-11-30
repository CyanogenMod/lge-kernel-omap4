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

#if defined(CONFIG_MACH_LGE_COSMO)
static const int keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),	
	KEY(1, 0, KEY_HOME),
	KEY(1, 1, KEY_3D),
	KEY(2, 1, KEY_TESTMODE_UNLOCK), //Pattern_Unlock_AT_Command
};

static struct matrix_keymap_data keymap_data = {
	.keymap			= keymap,
	.keymap_size		= ARRAY_SIZE(keymap),
};

static struct omap4_keypad_platform_data keypad_data = {
	.keymap_data		= &keymap_data,
	.rows			= 4,
	.cols			= 4,
};

#else
static const int keymap[] = {
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
	KEY(0, 2, 0),
	KEY(1, 1, KEY_TESTMODE_UNLOCK),
	KEY(1, 2, KEY_CAPTURE),
};

static struct matrix_keymap_data keymap_data = {
	.keymap			= keymap,
	.keymap_size		= ARRAY_SIZE(keymap),
};

static struct omap4_keypad_platform_data keypad_data = {
	.keymap_data		= &keymap_data,
	.rows			= 2,
	.cols			= 3,
};
#endif //##

static int __init keypad_init(void)
{
	return lge_set_keypad_info(&keypad_data);
}

lge_machine_initcall(keypad_init);
