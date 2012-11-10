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
#include <linux/omapfb.h>
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
	gpio_request(GPIO_LCD_POWER_EN, "dsi2_lcd_en");
	gpio_direction_output(GPIO_LCD_POWER_EN, 1);
	gpio_set_value(GPIO_LCD_POWER_EN, 1);
	mdelay(10);

	return 0;
}

static void panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_POWER_EN, 0);
}

static struct lge_dsi_panel_data dsi2_lgd_panel = {
	.name		= "lh430wv4_panel",
	.reset_gpio		= GPIO_LCD_RESET,
	.use_ext_te		= false,
 	.ext_te_gpio		= GPIO_LCD_EXT_TE,
	.esd_interval		= false,
	.set_backlight		= NULL
};

static struct omap_dss_device lgd_device = 
{
	.name	= "2lcd",
	.driver_name	= "lh430wv4_panel",
	.type	= OMAP_DISPLAY_TYPE_DSI,

	.data = &dsi2_lgd_panel,
	
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 3,
		.data1_pol	= 0,
		.data2_lane	= 2,
		.data2_pol	= 0,
	#if defined(CONFIG_DSI_VIDEO_MODE) || defined (CONFIG_DSI_VIDEO_CMD_TRANSITION)
		.type = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
	#else
		.type = OMAP_DSS_DSI_TYPE_CMD_MODE,
	#endif
		.module = DSS_IX_DSI2,
	},

	.clocks = {
		.dispc = {
			.channel = {
				.lck_div	= 1,	 /* Logic Clock = 153.6 MHz */
				.pck_div	= 5,	 /* Pixel Clock = 30.7 MHz */
				.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},

		.dsi = {
			.regn		= 19,  /* Fint = 1.92 MHz */
			.regm		= 180,  /* CLKIN4DDR = 768.0 MHz */
			.regm_dispc	= 4,  /* PLL2_CLK1 = 153.6 MHz */
			.regm_dsi	= 6,  /* PLL2_CLK2 = 109.7 MHz */

			.lp_clk_div	= 12, 	/* LP Clock */
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DSI,
		},
	},

	.platform_enable	=	panel_enable_lcd,
	.platform_disable	=	panel_disable_lcd,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	.skip_init = true,
#else
	.skip_init = false,
#endif
};

static struct lge_dsi_panel_data dsi2_hitachi_panel = {
	.name			= "tx11d101vm_panel",
	.reset_gpio		= GPIO_LCD_RESET,
	.use_ext_te 		= false,
	.ext_te_gpio		= GPIO_LCD_EXT_TE,
	.esd_interval		= false,
	.set_backlight		= NULL
};

static struct omap_dss_device hitachi_device = 
{
	.name	= "2lcd",
	.driver_name	= "tx11d101vm_panel",
	.type	= OMAP_DISPLAY_TYPE_DSI,

	.data = &dsi2_hitachi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 3,
		.data1_pol	= 0,
		.data2_lane	= 2,
		.data2_pol	= 0,
	#if defined(CONFIG_DSI_VIDEO_MODE) || defined (CONFIG_DSI_VIDEO_CMD_TRANSITION)
		.type = OMAP_DSS_DSI_TYPE_VIDEO_MODE,
	#else
		.type = OMAP_DSS_DSI_TYPE_CMD_MODE,
	#endif
		.module = DSS_IX_DSI2,
	},
	
	.clocks = {
		.dispc = {
			.channel = {
					.lck_div	= 1,	 /* Logic Clock = 153.6 MHz */
					.pck_div	= 5,	 /* Pixel Clock = 30.7 MHz */
					.lcd_clk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DISPC,
			},
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_FCK,
		},
				
		.dsi = {
			.regn		= 19,  /* Fint = 1.92 MHz */
			.regm		= 180,	/* CLKIN4DDR = 768.0 MHz */
			.regm_dispc	= 4,  /* PLL2_CLK1 = 153.6 MHz */
			.regm_dsi	= 6,  /* PLL2_CLK2 = 109.7 MHz */

			.lp_clk_div = 12,	/* LP Clock */
			.dsi_fclk_src	= OMAP_DSS_CLK_SRC_DSI2_PLL_HSDIV_DSI,
		},
	},

	.platform_enable	=	panel_enable_lcd,
	.platform_disable	=	panel_disable_lcd,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	.skip_init = true,
#else
	.skip_init = false,
#endif
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
// LGE_CHANGE_S [sungho.jung@lge.com] 2012-02-21, HDMI PLL supports maximum pixel clock of 148.5MHz
#if 1
			.max_pixclk_khz = 75000, // Max is 75 Mhz
#else
			.max_pixclk_khz = 148500,// Max is 148.5 Mhz
#endif
// LGE_CHANGE_E [sungho.jung@lge.com] 2012-02-21
		},
	},
	.hpd_gpio = GPIO_HDMI_HPD,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device lcd2_info;
static struct omap_dss_device *dss_devices[] = {
	&lcd2_info,
#ifdef CONFIG_OMAP4_DSS_HDMI		
	&hdmi_device,
#endif
};

static struct omap_dss_board_info dss_board_info = {
	.num_devices	= ARRAY_SIZE(dss_devices),
	.devices		= dss_devices,
	.default_device	= &lcd2_info,
};

int __init p940_dss_init(void)
{
	unsigned char maker_id;

	gpio_request(GPIO_LCD_MAKER_ID,"LCD Maker ID");
	gpio_request(GPIO_LCD_POWER_EN, "dsi2_lcd_en");
	gpio_direction_output(GPIO_LCD_POWER_EN, 1);
	gpio_direction_input(GPIO_LCD_MAKER_ID);
	
	gpio_set_value(GPIO_LCD_POWER_EN, 1);	//turn on lcd ldo to check lcd id
	mdelay(1);
	
	maker_id = gpio_get_value(GPIO_LCD_MAKER_ID);
	
	if(maker_id == 1)
		lcd2_info = lgd_device;
	else
		lcd2_info = hitachi_device;

	printk("[LCD] %s is detected\n", lcd2_info.driver_name);
	
	return lge_set_dss_info(&dss_board_info);
}

lge_machine_initcall(p940_dss_init);

/* LGE_SJIT 2012-1-13 [jongrak.kwon@lge.com]
 * Set the FB size to 10MB for upto three buffers for P2
 * Set the FB size to 8MB for upto two buffers for P2
 * Align_2M((Align_60K(800*4096) = 3317760) * 2) = 8MB
 * => Three buffer requires framework support (current two buffer support)
 */
#define FB_RAM_SIZE	(8*1024*1024) /* Align_60K (800 * 4096) * 2 */

static struct omapfb_platform_data fb_pdata = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			[0] = {
				.size = FB_RAM_SIZE,
			},
		},
	},
};

int __init p940_fb_init(void)
{
	return lge_set_fb_info(&fb_pdata);
}

lge_machine_initcall(p940_fb_init);

static void p940_dsi_phy_init(void)
{
	u32 reg;

	/* Enable 3 lanes (0 ~ 2) in DSI2 */
	/* Disable pull down */
	reg = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);
	reg &= ~OMAP4_DSI2_LANEENABLE_MASK;
	reg |= 0x1F << OMAP4_DSI2_LANEENABLE_SHIFT;
	reg &= ~OMAP4_DSI2_PIPD_MASK;
	reg |= 0x1F << OMAP4_DSI2_PIPD_SHIFT;
	omap4_ctrl_pad_writel(reg, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_DSIPHY);

#if defined(CONFIG_DSI_CMD_MODE)
	omap_mux_init_signal("gpmc_ncs6.dsi2_te0", OMAP_PIN_INPUT);
#endif
}

static void p940_hdmi_pad_init(void)
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

LGE_LATE_INIT(p940_dsi_phy_init);
LGE_LATE_INIT(p940_hdmi_pad_init);
