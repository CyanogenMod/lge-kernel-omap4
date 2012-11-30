/*
 * Copyright 2011 LG Electronic Inc. All Rights Reserved.

 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTIES OR REPRESENTATIONS; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.See the GNU General Public License
 * for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
//#define FUEL_GPIO_TEST_BOARD 1

#define FUEL_GAUGE_MAX17043_SLAVE_ID	0x36	/* 0x6C(Write) 0x6D(Read)*/
#if defined(FUEL_GPIO_TEST_BOARD)
#define FUEL_GAUGE_MAX17043_GPIO_ALERT	157		/* 0x6C(Write) 0x6D(Read)*/
#define FUEL_GAUGE_MAX17043_GPIO_SCL	13		/* 0x6C(Write) 0x6D(Read)*/
#define FUEL_GAUGE_MAX17043_GPIO_SDA	100		/* 0x6C(Write) 0x6D(Read)*/
#else
#define FUEL_GAUGE_MAX17043_GPIO_ALERT	21		/* 0x6C(Write) 0x6D(Read)*/
#define FUEL_GAUGE_MAX17043_GPIO_SCL	157		/* 0x6C(Write) 0x6D(Read)*/
#define FUEL_GAUGE_MAX17043_GPIO_SDA	163		/* 0x6C(Write) 0x6D(Read)*/
#endif
#define FUEL_GAUGE_MAX17043_UDELAY		5		/* 0x6C(Write) 0x6D(Read)*/
#define FUEL_GAUGE_MAX17043_TIMEOUT		20		/* 0x6C(Write) 0x6D(Read)*/


// Below code should be moved to inclue/linux/fuel_gauge_max17043.h
struct max17043_platform_data {
	u8  slave_addr;
	u16	gpio_alert;	/* active low */
	u16	gpio_scl;
	u16	gpio_sda;
	int	udelay;
	int	timeout;	
};

typedef struct __battery_graph_prop
{
	s32 x;
	s32 y;
} battery_graph_prop;

#ifndef __MAX17043_FUELGAUGE_H_
#define __MAX17043_FUELGAUGE_H_
#endif

#define GAUGE_INT	21	
#define RCOMP_BL44JN	(0xB0)	/* Default Value for cosmo Battery */


#define FG_CONTROL_ENABLE	1704301
#define FG_CONTROL_DISABLE	1704300


#define MAX17043_VCELL_REG	0x02
#define MAX17043_SOC_REG	0x04
#define MAX17043_MODE_REG	0x06
#define MAX17043_VER_REG	0x08
#define MAX17043_CONFIG_REG	0x0C
#define MAX17043_CMD_REG	0xFE

#define MAX17043_VCELL_MSB	0x02
#define MAX17043_VCELL_LSB	0x03
#define MAX17043_SOC_MSB	0x04
#define MAX17043_SOC_LSB	0x05
#define MAX17043_MODE_MSB	0x06
#define MAX17043_MODE_LSB	0x07
#define MAX17043_VER_MSB	0x08
#define MAX17043_VER_LSB	0x09
#define MAX17043_CONFIG_MSB	0x0C
#define MAX17043_CONFIG_LSB	0x0D
#define MAX17043_CMD_MSB	0xFE
#define MAX17043_CMD_LSB	0xFF

//#define UPDATE_SOC 1
//#define DEBUG_MODE
#define MAX17043_WORK_DELAY	(20 * HZ)	// 10s
#define MAX17043_BATTERY_FULL	97		// Tuning Value
#define MAX17043_TOLERANCE	20		// Tuning Value


typedef enum {
	MAX17403_UNKNOWN,
	MAX17043_RESET,
	MAX17043_QUICKSTART,
	MAX17043_WORKING,
	MAX17043_STATE_MAX
} max17043_status;

extern int max17043_get_capacity(void);
extern int max17043_get_voltage(void);
extern int max17043_do_calibrate(void);
extern int max17043_set_rcomp_by_temperature(int temp);
extern int max17043_set_alert_level(int alert_level);
extern int max17043_update_by_other(void);
int max17043_get_ui_capacity(void);
int max17043_reverse_get_ui_capacity(int level);
int max17043_quickstart(void);

int set_fg_enable(int en);
int get_fg_enable(void);
int is_cam_on(void);

