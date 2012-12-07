/*
 * Board support file for LGE OMAP4 Board.
 *
 * Copyright (C) 2011 LG Electronics Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-yeob Kim <doyeob.kim@lge.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/i2c/twl.h>
#include <linux/pm.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/reboot.h>
#include <linux/omapfb.h>
#include <linux/memblock.h>

#include <plat/i2c.h>
#include <plat/irqs.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap-serial.h>
#include <plat/usb.h>
#include <plat/vram.h>
#include <plat/remoteproc.h>
#include <plat/omap-pm.h>


#include <mach/omap4-common.h>
#include <mach/dmm.h>

#include <mach-omap2/pm.h>
#include <mach-omap2/mux.h>
#include <mach-omap2/timer-gp.h>
#include <mach-omap2/prm-regbits-44xx.h>
#include <mach-omap2/prm44xx.h>
#include <mach-omap2/control.h>
#include <mach-omap2/omap4_ion.h>
#include <mach-omap2/omap_ram_console.h>

#include <video/omapdss.h>

#include <lge/board.h>
#include <lge/common.h>

//nthyunjin.yang 120518 sdcard cover start
//#define CONFIG_MACH_LGE_MMC_ENHANCED_COVER 1
//#define CONFIG_MACH_LGE_MMC_COVER 1
//nthyunjin.yang 120518 sdcard cover end

#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#define GPIO_WIFI_PMENA		168
#define GPIO_WIFI_IRQ		167
/* lge_machine_data */
struct lge_machine_data lge_machine_data __initdata;

/* init-function list head */
static struct list_head init_func_list __initdata = {
	.next = &init_func_list,
	.prev = &init_func_list
};

void __init lge_init_func_add(initfn_t func)
{
	struct lge_init_func *init_func;

	if (func == NULL) {
		pr_err("%s: func is NULL!\n", __func__);
		return;
	}

	init_func = kmalloc(sizeof(struct lge_init_func), GFP_ATOMIC);
	if (init_func == NULL) {
		pr_err("%s: failed to allocate memory!\n", __func__);
		return;
	}

	init_func->func = func;
	list_add_tail(&init_func->node, &init_func_list);
}


static void lge_common_pm_poweroff(void)
{
	/* LGE_CHANGE [dongjin73.kim@lge.com] 2010-?-?,
	  COMMON: POWER OFF (PHOENIX_DEV_ON - 25h) */
	twl_i2c_write_u8(TWL_MODULE_PM_MASTER, 0x07, 0x06);
}

static void __init lge_mux_gpio_config(struct lge_mux_gpio_info *mux_gpio_info)
{
	if (!mux_gpio_info)
		return;
	
	while (mux_gpio_info->gpio != LGE_MUX_GPIO_TERMINATOR) {
        omap_mux_init_gpio(mux_gpio_info->gpio,mux_gpio_info->val);
		mux_gpio_info++;
	}

	return;
}

/* LGE_SJIT 2012-01-12 [dojip.kim@lge.com] rtc enable (P940 GB) */
static void omap4_twl6030_rtc_enable(void) {
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
#define GPIO_TWL6030_RTC	6

	omap_mux_init_signal("fref_clk0_out.gpio_wk6", OMAP_PIN_OUTPUT);

	gpio_request(GPIO_TWL6030_RTC, "sys_drm_msecure");
	gpio_direction_output(GPIO_TWL6030_RTC, 1);
}

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
#ifndef CONFIG_MACH_LGE_MMC_COVER //nthyunjin.yang 120517 for sd card
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
#else
#if defined(CONFIG_MACH_LGE_MMC_ENHANCED_COVER)
	  	pdata->slots[0].card_detect_irq_by_data3pin = TWL6030_IRQ_BASE + MMCDETECT_INTR_OFFSET;	  
#endif
#endif
	}
	/* Setting MMC5 SDIO card .built-in variable
	 * This is to make sure that if WiFi driver is not loaded
	 * at all, then the MMC/SD/SDIO driver does not keep
	 * turning on/off the voltage to the SDIO card
	 */
	if (pdev->id == 4) {
		ret = 0;
		pdata->slots[0].mmc_data.built_in = 1;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static void omap_4460hsi_pad_conf(void)
{
        /*
         * HSI pad conf: hsi1_ca/ac_wake/flag/data/ready
         */

        /* hsi1_cawake */
        omap_mux_init_signal("usbb1_ulpitll_clk.hsi1_cawake", \
                OMAP_PIN_INPUT_PULLDOWN | \
                OMAP_PIN_OFF_NONE | \
                OMAP_PIN_OFF_WAKEUPENABLE);
        /* hsi1_caflag */
        omap_mux_init_signal("usbb1_ulpitll_dir.hsi1_caflag", \
                OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE);
        /* hsi1_cadata */
        omap_mux_init_signal("usbb1_ulpitll_stp.hsi1_cadata", \
                OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE);
        /* hsi1_acready */
        omap_mux_init_signal("usbb1_ulpitll_nxt.hsi1_acready", \
                OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_OUTPUT_LOW);
        /* hsi1_acwake */
        omap_mux_init_signal("usbb1_ulpitll_dat0.hsi1_acwake", \
                OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE);
        /* hsi1_acdata */
        omap_mux_init_signal("usbb1_ulpitll_dat1.hsi1_acdata", \
                OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE);
        /* hsi1_acflag */
        omap_mux_init_signal("usbb1_ulpitll_dat2.hsi1_acflag", \
                OMAP_PIN_OUTPUT | \
                OMAP_PIN_OFF_NONE);
        /* hsi1_caready */
        omap_mux_init_signal("usbb1_ulpitll_dat3.hsi1_caready", \
                OMAP_PIN_INPUT | \
                OMAP_PIN_OFF_NONE);

//mo2haewoon.you@lge.com => [START]
		/*
		* Set CP crash notification instead of 
		 * IFX SPI configuration for upgrade
		*/

		/* IPC_SRDY gpio_119 */
		omap_mux_init_signal("abe_dmic_clk1.gpio_119", \
					  OMAP_PIN_INPUT_PULLDOWN | \
					  OMAP_PIN_OFF_NONE | \
					  OMAP_PIN_OFF_WAKEUPENABLE);

		/* IPC_MRDY gpio_120 */
		omap_mux_init_signal("abe_dmic_din1.gpio_120", \
					  OMAP_PIN_INPUT_PULLDOWN | \
					  OMAP_PIN_OFF_NONE | \
					  OMAP_PIN_OFF_WAKEUPENABLE);

//mo2haewoon.you@lge.com [START]
#if defined(CONFIG_MACH_LGE_COSMO_SU760) || defined(CONFIG_MACH_LGE_CX2_SU870)
	/* hsi1_omap_send */
	omap_mux_init_signal("abe_dmic_din3.gpio_122", \
		OMAP_PIN_INPUT_PULLDOWN | \
		OMAP_PIN_OFF_NONE | \
		OMAP_PIN_OFF_WAKEUPENABLE);
#elif defined(CONFIG_MACH_LGE_COSMO) || defined(CONFIG_MACH_LGE_CX2)
		omap_mux_init_signal("abe_dmic_din2.gpio_121", \
					  OMAP_PIN_INPUT_PULLDOWN | \
					  OMAP_PIN_OFF_NONE | \
					  OMAP_PIN_OFF_WAKEUPENABLE);
#endif
//mo2haewoon.you@lge.com [END]
}

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
};

void __init lge_common_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);


#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif

}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static void omap4_audio_conf(void)
{
	/* LGE_SJIT 2012-01-12 [dojip.kim@lge.com] wakeup enable (P940 GB) */
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE);
}

static void __init pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata lge_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata lge_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata lge_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata lge_i2c_4_bus_pdata;

static void __init omap4_i2c_init(void)
{
	int i;
	struct lge_i2c_config *cfg;

	omap_i2c_hwspinlock_init(1, 0, &lge_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &lge_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &lge_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &lge_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &lge_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &lge_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &lge_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &lge_i2c_4_bus_pdata);

	for(i = 0; i < 4; i++) {
		if (lge_machine_data.i2c_cfg[i] != NULL) {
			cfg = lge_machine_data.i2c_cfg[i];
			omap_register_i2c_bus(cfg->bus_id, cfg->clkrate,
					cfg->info, cfg->len);
		}
	}
	//omap4_pmic_init("twl6030", &lghdk_twldata);
}

int __init lge_set_i2c_bus_info(struct lge_i2c_config *cfg)
{
	BUG_ON(cfg->bus_id < 1 || cfg->bus_id > 4);
	
	if(lge_machine_data.i2c_cfg[cfg->bus_id-1] != NULL) {
		printk("i2c bus %d is already set.\n", cfg->bus_id);
		return -EINVAL;
	}

	lge_machine_data.i2c_cfg[cfg->bus_id-1] = cfg;

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#define FB_RAM_SIZE                20*1024*1024 /* Align_60K(800 * 4096) * 3 */

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

/* LGE_SJIT 2012-01-16 [dojip.kim@lge.com]
 * fb_size might be different on board.
 */
int __init lge_set_fb_info(struct omapfb_platform_data *fb_pdata)
{
	if (lge_machine_data.fb_pdata != NULL) {
		printk("fb pdata is already set.\n");
		return -EBUSY;
	}
	lge_machine_data.fb_pdata = fb_pdata;
	return 0;
}

static void lge_common_display_init(void)
{
	struct omapfb_platform_data *fb = &fb_pdata;

	if (lge_machine_data.fb_pdata)
		fb = lge_machine_data.fb_pdata;
	omap_vram_set_sdram_vram(fb->mem_desc.region[0].size, 0);
	omapfb_set_platform_data(fb);

	if (lge_machine_data.dss_board != NULL)
		omap_display_init(lge_machine_data.dss_board);
}

static struct omap_device_pad uart1_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE1,
	},
	{
		.name	= "mcspi1_cs1.uart1_rx",
                .flags  = OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
                .enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
                .idle   = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE1,
	},
};

// +s LGBT_COMMON_UART_SLEEP hyuntae0.kim@lge.com 120802
//TIK Brandon 20120801 : Power consumption - for wakeup after software reset.
static struct omap_device_pad uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_WAKEUP_EN | OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
		.flags  = OMAP_DEVICE_PAD_REMUX,
		.idle   = OMAP_PIN_OFF_INPUT_PULLUP | OMAP_MUX_MODE7,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
//		.flags = OMAP_DEVICE_PAD_WAKEUP,
//		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.enable        = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};
//TIK Brandon 20120801 : Power consumption - for wakeup after software reset.
// +e LGBT_COMMON_UART_SLEEP

/* LGE_SJIT 2011-11-03 [jongrak.kwon@lge.com] UART3 mux name correction */
static struct omap_device_pad uart3_pads[] __initdata = {
	{
		.name	= "dpm_emu6.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE2,
	},
	{
		.name	= "dpm_emu7.uart3_rx_irrx",
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE2,
	},
	{
		.name	= "dpm_emu8.uart3_rts_sd",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE2,
	},
	{
		.name	= "dpm_emu9.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLDOWN | OMAP_MUX_MODE2,
	},
};

static struct omap_device_pad uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		/* LGE_SJIT 2012-01-04 [dojip.kim@lge.com]
		 * Don't use pad wakeup if you use uart4 as console
		 * It causes abnormal wakeup when suspending
		 */
//mo2haewoon.you@lge.com [START]		 
#if 1
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
#else
		.flags	= OMAP_DEVICE_PAD_REMUX, // | OMAP_DEVICE_PAD_WAKEUP,
		.enable = OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
#endif
//mo2haewoon.you@lge.com [END]		 
	},
};

static struct omap_uart_port_info uart2_info __initdata = {
	.use_dma	= 1,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.rts_mux_driver_control = 1,
};

static struct omap_uart_port_info uart3_info __initdata = {
	.use_dma	= 0,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = 0,
};

static inline void __init board_serial_init(void)
{
	omap_serial_init_port_pads(0, uart1_pads,
		ARRAY_SIZE(uart1_pads), NULL);
	omap_serial_init_port_pads(1, uart2_pads,
		ARRAY_SIZE(uart2_pads), &uart2_info);
	omap_serial_init_port_pads(2, uart3_pads,
		ARRAY_SIZE(uart3_pads), &uart3_info);
	omap_serial_init_port_pads(3, uart4_pads,
		ARRAY_SIZE(uart4_pads), NULL);
}

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_OHCI_PORT_MODE_PHY_6PIN_DATSE0,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_ohci_init(void)
{

	omap_mux_init_signal("usbb2_ulpitll_clk.gpio_157", \
		OMAP_PIN_OUTPUT | \
		OMAP_PIN_OFF_NONE);

	/* Power on the ULPI PHY */
	if (gpio_is_valid(OMAP4SDP_MDM_PWR_EN_GPIO)) {
		gpio_request(OMAP4SDP_MDM_PWR_EN_GPIO, "USBB1 PHY VMDM_3V3");
		gpio_direction_output(OMAP4SDP_MDM_PWR_EN_GPIO, 1);
	}

	usbhs_init(&usbhs_bdata);

	return;

}
#else
static void __init omap4_ehci_ohci_init(void){}
#endif

#ifdef CONFIG_MACH_LGE_COSMO
static void omap4_sdp4430_wifi_mux_init(void)
{
	omap_mux_init_gpio(GPIO_WIFI_IRQ, OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(GPIO_WIFI_PMENA, OMAP_PIN_OUTPUT);
}

static struct wl12xx_platform_data omap4_sdp4430_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4,
//	.platform_quirks = 1,
};

static struct regulator_consumer_supply omap4_sdp4430_vmmc5_supply = {
	.supply = "vmmc",
	.dev_name = "omap_hsmmc.4",
};
static struct regulator_init_data sdp4430_vmmc5 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap4_sdp4430_vmmc5_supply,
};
static struct fixed_voltage_config sdp4430_vwlan = {
	.supply_name = "vwl1271",
	.microvolts = 1800000, /* 1.8V */
	.gpio = GPIO_WIFI_PMENA,
	.startup_delay = 70000, /* 70msec */
	.enable_high = 1,
	.enabled_at_boot = 0,
	.init_data = &sdp4430_vmmc5,
};
static struct platform_device omap_vwlan_device = {
	.name	= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &sdp4430_vwlan,
	}
};

void omap4_sdp4430_wifi_init(void)
{
	omap4_sdp4430_wifi_mux_init();
	if (wl12xx_set_platform_data(&omap4_sdp4430_wlan_data))
		pr_err("Error setting wl12xx data\n");
	platform_device_register(&omap_vwlan_device);
}
#endif

void __init lge_common_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;
	struct lge_init_func *init_func, *tmp;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;

	omap4_mux_init(lge_machine_data.board_mux, NULL, package);


	if (lge_machine_data.power_off != NULL)
		pm_power_off = lge_machine_data.power_off;
	else
		pm_power_off = lge_common_pm_poweroff;

	if (lge_machine_data.pm_restart != NULL)
		arm_pm_restart = lge_machine_data.pm_restart;

	if (lge_machine_data.emif_devices != NULL) {
		struct lge_emif_devices *emif_devs = lge_machine_data.emif_devices;
		omap_emif_setup_device_details(&emif_devs->emif1_devices, &emif_devs->emif2_devices);
	}

	omap_board_config = sdp4430_config;
	omap_board_config_size = ARRAY_SIZE(sdp4430_config);

	//omap_init_board_version(0);

	/* LGE_SJIT 2012-01-12 [dojip.kim@lge.com] rtc enable (P940 GB) */
	omap4_twl6030_rtc_enable();
	omap4_audio_conf();
	omap4_create_board_props();
	pmic_mux_init();
	omap4_i2c_init();
	//blaze_sensor_init();
	//blaze_touch_init();
	omap4_register_ion();
	
	if (lge_machine_data.pdevs != NULL)
		platform_add_devices(lge_machine_data.pdevs,
				lge_machine_data.pdev_len);

	board_serial_init();

#ifdef CONFIG_MACH_LGE_COSMO
	omap4_sdp4430_wifi_init();
#endif
	if (lge_machine_data.hsmmc_info != NULL)
		omap4_twl6030_hsmmc_init(lge_machine_data.hsmmc_info);

	omap4_ehci_ohci_init();

	usb_musb_init(&musb_board_data);

	if(lge_machine_data.keypad_data != NULL) {
		status = omap4_keyboard_init(lge_machine_data.keypad_data);
	
		if (status)
			pr_err("Keypad initialization failed: %d\n", status);
	}

	omap_dmm_init();
	lge_common_display_init();
	//blaze_panel_init();
	//blaze_keypad_init();

	lge_mux_gpio_config(lge_machine_data.mux_gpios);

#if defined(CONFIG_OMAP_HSI)
	pr_info("Modem MIPI-HSI detected");
	omap_4460hsi_pad_conf();
#endif

#ifdef CONFIG_LGE_BROADCAST_TDMB
#ifdef CONFIG_SPI
	if (lge_machine_data.spi_board != NULL)
		spi_register_board_info(lge_machine_data.spi_board,
				lge_machine_data.spi_len);
#endif /* CONFIG_SPI */
#endif /* CONFIG_LGE_BROADCAST_TDMB */

	if (cpu_is_omap446x()) {
		/* Vsel0 = gpio, vsel1 = gnd */
		status = omap_tps6236x_board_setup(true, TPS62361_GPIO, -1,
					OMAP_PIN_OFF_OUTPUT_HIGH, -1);
		if (status)
			pr_err("TPS62361 initialization failed: %d\n", status);
	}

	omap_enable_smartreflex_on_init();
	if (enable_suspend_off)
		omap_pm_enable_off_mode();
	/* call functions that defined LGE_LATE_INIT(function)*/
	list_for_each_entry_safe(init_func, tmp, &init_func_list, node) {
		init_func->func();
		list_del(&init_func->node);
		kfree(init_func);
	}
}

#if 0
void __init lge_common_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}
#endif 

void __init lge_common_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

void __init lge_common_reserve(void)
{
	/* LGE_SJIT 2012-02-06 [dojip.kim@lge.com]
	 * need to align 1M for lge panic handler
	 */
	omap_ram_console_init(LGE_RAM_CONSOLE_START_DEFAULT,
			ALIGN(LGE_RAM_CONSOLE_SIZE_DEFAULT, SZ_1M));

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
#if defined(CONFIG_MACH_LGE_COSMO)	
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE);
#else
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					SZ_1M*90);
#endif
#ifdef CONFIG_OMAP_REMOTE_PROC_DSP
	memblock_remove(PHYS_ADDR_TESLA_MEM, PHYS_ADDR_TESLA_SIZE);
	omap_dsp_set_static_mempool(PHYS_ADDR_TESLA_MEM,
					PHYS_ADDR_TESLA_SIZE);
#endif

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}

int __init lge_set_hsmmc_info(struct omap2_hsmmc_info *hsmmc_info)
{
	if(hsmmc_info == NULL) {
		pr_err("the provided argument 'hsmmc_info' is null");
		return -EINVAL;
	}

	if(lge_machine_data.hsmmc_info != NULL) {
		pr_err("hsmmc_info is already initialized");
		return -EPERM;
	}

	lge_machine_data.hsmmc_info = hsmmc_info;

	return 0;
}

int __init lge_set_pm_restart(void (*fp_pm_restart)(char, const char *) )
{
	if(fp_pm_restart == NULL) {
		pr_err("the provided argument 'fp_pm_restart' is null");
		return -EINVAL;
	}

	if(lge_machine_data.pm_restart != NULL) {
		pr_err("pm_restart is already initialized");
		return -EPERM;
	}

	lge_machine_data.pm_restart = fp_pm_restart;

	return 0;
}

int __init lge_set_mux_gpios(struct lge_mux_gpio_info *mux_gpio_infos)
{
	if(mux_gpio_infos == NULL) {
		pr_err("the provided argument 'gpio_infos' is empty");
		return -EINVAL;
	}

	if(lge_machine_data.mux_gpios != NULL) {
		pr_err("gpios is already initialized");
		return -EPERM;
	}

	lge_machine_data.mux_gpios = mux_gpio_infos;

	return 0;
}

int __init lge_set_dss_info(struct omap_dss_board_info *dss_board_info)
{
	if(dss_board_info == NULL) {
		pr_err("the provided argument 'dss_board_info' is null");
		return -EINVAL;
	}

	if(lge_machine_data.dss_board != NULL) {
		pr_err("dss_board is already initialized");
		return -EPERM;
	}

	lge_machine_data.dss_board = dss_board_info;

	return 0;
}

int __init lge_set_spi_board(struct spi_board_info *spi_board_info, unsigned spi_board_array_size)
{
	if(spi_board_info == NULL || spi_board_array_size == 0) {
		pr_err("the provided argument 'spi_board_info' is empty");
		return -EINVAL;
	}

	if(lge_machine_data.spi_board != NULL) {
		pr_err("spi_board is already initialized");
		return -EPERM;
	}

	lge_machine_data.spi_board = spi_board_info;
	lge_machine_data.spi_len = spi_board_array_size;

	return 0;
}

int __init lge_set_cp_init(void (*cp_init)(void))
{
	if(cp_init == NULL) {
		pr_err("the provided argument 'cp_init' is null");
		return -EINVAL;
	}

	if(lge_machine_data.cp_init != NULL) {
		pr_err("cp_init is already initialized");
		return -EPERM;
	}

	lge_machine_data.cp_init = cp_init;

	return 0;
}

int __init lge_set_emif_devices(struct lge_emif_devices *emif_devices)
{
	if(emif_devices == NULL) {
		pr_err("the provided argument 'emif_devices' is null");
		return -EINVAL;
	}

	if(lge_machine_data.emif_devices != NULL) {
		pr_err("emif_devices is already initialized");
		return -EPERM;
	}

	lge_machine_data.emif_devices = emif_devices;

	return 0;
}

int __init lge_set_pdevs(struct platform_device **pdevs, unsigned pdevs_array_size)
{	
	if(pdevs == NULL || pdevs_array_size == 0) {
		pr_err("the provided argument 'pdevs' is empty");
		return -EINVAL;
	}

	if(lge_machine_data.pdevs != NULL) {
		pr_err("pdevs is already initialized");
		return -EPERM;
	}

	lge_machine_data.pdevs = pdevs;
	lge_machine_data.pdev_len = pdevs_array_size;

	return 0;
}


int __init lge_set_mux_info(struct omap_board_mux *board_mux)
{
	if(lge_machine_data.board_mux != NULL) {
		pr_err("board_mux is already initialized");
		return -EPERM;
	}

	lge_machine_data.board_mux = board_mux;
	return 0;
}

int __init lge_set_keypad_info(struct omap4_keypad_platform_data *keypad_data)
{
	if(lge_machine_data.keypad_data != NULL) {
		pr_err("keypad_data is already initialized");
		return -EPERM;
	}

	lge_machine_data.keypad_data = keypad_data;
	return 0;

}

/* LGE_SJIT 2012-01-18 [dojip.kim@lge.com]
 * Ignore gpios as these are not intended to wakeup the system
 */
unsigned long __attribute__((weak)) lge_get_no_pad_wakeup_gpios(int bank_id)
{
	return 0;
}
