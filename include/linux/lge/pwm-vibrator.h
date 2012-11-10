/*
 * The header file for vibrator 
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __PWM_VIBRATOR_H
#define __PWM_VIBRATOR_H

#define VIB_PWM_NAME "vib-pwm"

struct pwm_vibrator_platform_data {
	u32 freq;
	u32 duty;
	u32 gpio_enable;
	int (*power)(bool bool);
	int port;	//LGE_SJIT 2011-09-01 [jongrak.kwon@lge.com] pwm port info
};

#endif /* __PWM_VIBRATOR_H */
