/*
 * Display sub-system initializing code.
 *
 * Copyright (C) 2010 LG Electronic Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * Modeified by Choongryeol Lee <choongryeol.lee@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c/twl.h>
#include <linux/gpio.h>
#include <lge/common.h>
#include <lge/board.h>
#include <mach-omap2/control.h>
#include <video/lge-dsi-panel.h>

enum {
	DSS_IX_DSI1=0,
	DSS_IX_DSI2
};

static int panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_request(GPIO_LCD_POWER_EN, "dsi1_lcd_en");
	gpio_direction_output(GPIO_LCD_POWER_EN, 1);
	gpio_set_value(GPIO_LCD_POWER_EN, 1);
	mdelay(5);
	return 0;
}

static void panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_POWER_EN, 0);
}

static struct lge_dsi_panel_data dsi1_panel = {
	.name		= "dx11d100vm_panel",
	.reset_gpio		= GPIO_LCD_RESET,
	.use_ext_te		= false,
 	.ext_te_gpio		= false,
	.esd_interval		= false,
	.set_backlight		= NULL
};

static struct omap_dss_device lcd1_device = 
{
	.name	= "lcd",
	.driver_name = "dx11d100vm_panel",
	
	.type = OMAP_DISPLAY_TYPE_DSI,
	.data = &dsi1_panel,
	
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.data3_lane	= 4,
		.data3_pol	= 0,
		.data4_lane	= 5,
		.data4_pol	= 0,
		
		.type = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
		.module = DSS_IX_DSI1,
	},

	.clocks = {
		.dispc = {
			.channel = {
				.lck_div	= 1, /* Logic Clock = 160.0 MHz */
				.pck_div	= 2, /* Pixel Clock = 80.0 MHz */
				.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},

		.dsi = {
			.regn		= 19,  /* Fint = 1.92 MHz */
			.regm		= 250, /* CLKIN4DDR = 960.0 MHz */
			.regm_dispc	= 5,   /* PLL1_CLK1 = 160.0 MHz */
			.regm_dsi	= 6,   /* PLL1_CLK2 = 137.1 MHz */

			.lp_clk_div	= 12,  /* LP Clock =  MHz */
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
		},
	},
	
	.platform_enable	=	panel_enable_lcd,
	.platform_disable	=	panel_disable_lcd,
	.channel		= OMAP_DSS_CHANNEL_LCD,
	.skip_init = false,
};
static struct omap_dss_device hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.hpd_gpio = GPIO_HDMI_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *dss_devices[] = {
	&lcd1_device,
	&hdmi_device,
};

static struct omap_dss_board_info dss_board_info = {
	.num_devices	= ARRAY_SIZE(dss_devices),
	.devices		= dss_devices,
	.default_device	= &lcd1_device,
};


int __init iff_dss_init(void)
{
	return lge_set_dss_info(&dss_board_info);
}

lge_machine_initcall(iff_dss_init);

static void iff_lcd_dsi_phy_init(void)
{
	u32 reg;

	/* Enable 5 lanes (0 ~ 4) in DSI1 */
	/* Disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI1_LANEENABLE_MASK;
	reg |= 0x1F << OMAP4_DSI1_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI1_PIPD_MASK;
	reg |= 0x1F << OMAP4_DSI1_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
}

static void iff_hdmi_pad_init(void)
{
	u32 r;

	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd.hdmi_hpd",
				OMAP_PIN_INPUT_PULLDOWN);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl.hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda.hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);

	/* strong pullup on DDC lines using unpublished register */
	r = ((1 << 24) | (1 << 28)) ;
	omap4_ctrl_pad_writel(r, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_1);

	gpio_request(GPIO_HDMI_HPD, NULL);
	omap_mux_init_gpio(GPIO_HDMI_HPD, OMAP_PIN_INPUT | OMAP_PULL_ENA);
	gpio_direction_input(GPIO_HDMI_HPD);

}

LGE_LATE_INIT(iff_lcd_dsi_phy_init);
LGE_LATE_INIT(iff_hdmi_pad_init);
