/*
 * Miscellaneous initializing code.
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <lge/common.h>
#include <lge/board.h>

/* LGE_SJIT 2012-01-18 [dojip.kim@lge.com]
 * Ignore gpios as these are not intened to wakeup the system
 */
unsigned long lge_get_no_pad_wakeup_gpios(int bank_id)
{
	unsigned long v = 0;

	switch(bank_id) {
		case 0:
			break;
		case 1:
			/* GPIO 63 as it is used for HDMI HPD */
			v = BIT(31);
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		default:
			break;
	}
	return v;
}
EXPORT_SYMBOL(lge_get_no_pad_wakeup_gpios);

int __init iff_misc_init(void)
{
	return 0;
}

lge_machine_initcall(iff_misc_init);
