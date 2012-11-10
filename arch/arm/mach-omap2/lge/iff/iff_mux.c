/*
 * Board specific MUX configuration file.
 *
 * Copyright (C) 2011 LG Electronics Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/gpio.h>
#include <mach-omap2/control.h>
#include <lge/common.h>
#include <lge/board.h>

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(KPD_COL3, OMAP_MUX_MODE1 | OMAP_WAKEUP_EN),
	OMAP4_MUX(KPD_COL4, OMAP_MUX_MODE1 | OMAP_WAKEUP_EN),
	OMAP4_MUX(KPD_ROW3, OMAP_MUX_MODE1 | OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_INPUT_EN),
	OMAP4_MUX(KPD_ROW4, OMAP_MUX_MODE1 | OMAP_PULL_ENA | OMAP_PULL_UP |
			OMAP_WAKEUP_EN | OMAP_INPUT_EN),

#if 0
	/* wifi pads setting */ 
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct lge_mux_gpio_info mux_gpios[] __initdata = {
#if 0
	LGE_MUX_GPIO(LGE_WIFI_IRQ_GPIO, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE), 
	LGE_MUX_GPIO(LGE_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT), 
	LGE_MUX_GPIO(CONFIG_BCM4330_GPIO_BT_RESET, OMAP_PIN_OUTPUT),
    LGE_MUX_GPIO(CONFIG_BCM4330_GPIO_HOST_WAKEUP, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
    LGE_MUX_GPIO(CONFIG_BCM4330_GPIO_BT_ENABLE, OMAP_PIN_OUTPUT ), 

#endif	
	
	{ .gpio = LGE_MUX_GPIO_TERMINATOR },
};

int __init iff_mux_init(void)
{
	lge_set_mux_gpios(mux_gpios);
	lge_set_mux_info(board_mux);
	return 0;
}

lge_machine_initcall(iff_mux_init);


