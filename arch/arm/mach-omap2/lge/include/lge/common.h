/*
 * LGE common header file
 *
 * Copyright (C) 2010 LG Electronics Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LGE_PLAT_COMMON_H
#define __LGE_PLAT_COMMON_H

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/omapfb.h>
#include <asm/types.h>
#include <mach/emif.h>

#include <plat/common.h>
#include <plat/omap4-keypad.h>
//#include <plat/display.h>
//#include <plat/voltage.h>

#include "../../../hsmmc.h"
#include "../../../mux.h"

struct lge_emif_devices {
	struct emif_device_details emif1_devices;
	struct emif_device_details emif2_devices;
};

struct lge_i2c_config {
	int bus_id;
	u32 clkrate;
	struct i2c_board_info *info;
	unsigned len;
};

#define LGE_MUX_GPIO_TERMINATOR	0xffff

struct lge_mux_gpio_info {
	unsigned gpio;
	unsigned val;
};

struct lge_machine_data {
	struct lge_emif_devices *emif_devices;		/* EMIF */
	struct omapfb_platform_data *fb_pdata;		/* FB pdata */
	struct omap_dss_board_info *dss_board;		/* DSS */
	struct lge_i2c_config *i2c_cfg[4];		/* I2C 1 */
	struct spi_board_info *spi_board;		/* SPI */
	unsigned spi_len;
	struct omap2_hsmmc_info *hsmmc_info;		/* HS-MMC */
	struct lge_wlan_config *wlan_cfg;		/* WLAN */
	struct platform_device **pdevs;			/* platform devices */
	unsigned pdev_len;
	struct omap_board_mux *board_mux;			/* MUX info */
	struct lge_mux_gpio_info *mux_gpios;		/* GPIOs */
	struct omap4_keypad_platform_data *keypad_data; /* platform data for OMAP keypad controler */ 
	struct platform_device *usb_pdev[3];
	void (*cp_init) (void);				/* CP init function */
	void (*power_off) (void);			/* power off function */
	void (*pm_restart) (char str, const char *cmd);	/* power restart function */
};

extern struct lge_machine_data lge_machine_data __initdata;

struct lge_init_func {
	void (*func) (void);
	struct list_head node;
};

typedef void (*initfn_t) (void);
extern void __init lge_init_func_add(initfn_t func);

#define LGE_MUX_GPIO(gpio_num, mux_value) \
{ \
	.gpio	= (gpio_num), \
	.val	= (mux_value), \
}

#define lge_machine_initcall(fn)	postcore_initcall_sync(fn)

#define LGE_LATE_INIT(fn)	\
int __init lge_##fn##_initcall(void) \
{ \
	lge_init_func_add(fn); \
	return 0; \
} \
lge_machine_initcall(lge_##fn##_initcall)

#define lge_common_timer	omap_timer

extern struct twl4030_usb_data omap4_usbphy_data;

extern void __init lge_common_map_io(void);
//extern void __init lge_common_init_irq(void);
extern void __init lge_common_init(void);
extern void __init lge_common_reserve(void);
extern void __init lge_common_init_early(void);

int __init lge_set_i2c_bus_info(struct lge_i2c_config *);
int __init lge_set_hsmmc_info(struct omap2_hsmmc_info *hsmmc_info);
int __init lge_set_pm_restart(void (*fp_pm_restart)(char, const char *) );
int __init lge_set_mux_gpios(struct lge_mux_gpio_info *gpio_infos);
int __init lge_set_fb_info(struct omapfb_platform_data *fb_pdata);
int __init lge_set_dss_info(struct omap_dss_board_info *dss_board_info);
int __init lge_set_spi_board(struct spi_board_info *spi_board_info, unsigned spi_board_array_size);
int __init lge_set_cp_init(void (*cp_init)(void));
int __init lge_set_emif_devices(struct lge_emif_devices *emif_devices);
int __init lge_set_pdevs(struct platform_device **pdevs, unsigned pdevs_array_size);
int __init lge_set_mux_info(struct omap_board_mux *board_mux);
int __init lge_set_keypad_info(struct omap4_keypad_platform_data *keypad_data);

void omap4_create_board_props(void);
void omap_ion_init(void);
void omap4_register_ion(void);

extern unsigned long lge_get_no_pad_wakeup_gpios(int bank_id);

#endif /* __LGE_PLAT_COMMON_H */
