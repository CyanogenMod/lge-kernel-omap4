/* include/linux/isa1000-vib.h
 *
 * Copyright (C) 2011, LG Electronics Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_ISA1000_VIB_H
#define __LINUX_ISA1000_VIB_H

#ifdef CONFIG_ANDROID_TIMED_OUTPUT

#define ISA1000_VIB_NAME "isa1000-vib"

struct isa1000_vib_platform_data {
    int max_timeout;
    int enable_gpio;

    int (*poweron) (void);
    int (*poweroff) (void);
    int (*pwm_init) (struct platform_device *pdev);
	int (*pwm_exit) (void);
    int (*pwm_start) (void);
    int (*pwm_stop) (void);
};

#endif	/* CONFIG_ANDROID_TIMED_OUTPUT */

#endif	/* __LINUX_ISA1000_VIB_H */
