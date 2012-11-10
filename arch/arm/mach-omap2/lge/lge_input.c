/* drivers/input/misc/lge_input.c
 *
 * Copyright (C) 2012 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/lge/lge_input.h>

static struct input_dev *input_private = NULL;
static struct input_dev *input_private_touch = NULL;

int lge_input_set(void *dev)
{
	if (input_private) {
		pr_info("lge_input: already set input_device\n");
		return -EBUSY;
	}
	input_private = dev;

	return 0;
}
EXPORT_SYMBOL(lge_input_set);

struct input_dev *lge_input_get(void)
{
	return input_private;
}
EXPORT_SYMBOL(lge_input_get);

int lge_input_set_touch(void *dev)
{
	if (input_private_touch) {
		pr_info("lge_input: already set input_device_touch\n");
		return -EBUSY;
	}
	input_private_touch = dev;

	return 0;
}
EXPORT_SYMBOL(lge_input_set_touch);

struct input_dev *lge_input_get_touch(void)
{
	return input_private_touch;
}
EXPORT_SYMBOL(lge_input_get_touch);
