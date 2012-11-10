/* 
 * arch/arm/mach-omap2/lge/lge_cmdline.c
 *
 * Copyright (C) 2011 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/setup.h>
#include <lge/board.h>

/* It will be redefined in board specific file if you need */
void __init __attribute__((weak)) lge_manipulate_cmdline(char *default_command_line)
{
}

void __init manipulate_cmdline(char *default_command_line,
		const char *tag_command_line, size_t size)
{
	strlcpy(default_command_line, tag_command_line, size);
	lge_manipulate_cmdline(default_command_line);
	pr_info("bootloader  cmdline: %s\n", tag_command_line);
	pr_info("manipulated cmdline: %s\n", default_command_line);
}
