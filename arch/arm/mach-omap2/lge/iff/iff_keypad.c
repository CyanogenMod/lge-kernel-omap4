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

/*                                                                   */
/*                                                 */
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

/*                                                  */

int __init iff_keypad_init(void)
{
	/*                                        
                                               
                                                       
  */
	if (system_rev == LGE_PCB_C)
		return 0;

	return lge_set_keypad_info(&iff_keypad_data);
}

lge_machine_initcall(iff_keypad_init);
