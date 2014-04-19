/*
 * The header file for Cosmopolitan GPS
 *
 * Copyright (C) 2011 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __GPS_GPIO_H
#define __GPS_GPIO_H

struct gps_gpio_platform_data {
	unsigned pwron;		/* PWR_ON GPIO */
	unsigned reset_n;	/* RESET_N GPIO */
#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	unsigned lna_sd;	/* LNA_SD GPIO */
#endif
//                                                                       
#define UART_DEV_NAME_LEN 32
        unsigned char uart_dev_name[UART_DEV_NAME_LEN]; /* gps uart name */
        int (*uart_enable) (void);
        int (*uart_disable) (void);
//                                                                       
};

#endif /* __GPS_GPIO_H */
