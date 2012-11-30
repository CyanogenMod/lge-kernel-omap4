/*
 * I2C devices intializing code.
 *
 * Copyright (C) 2010,2011,2012 LG Electronic Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cdc_tcxo.h>
#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6130x.h>
#include <mach/irqs.h>
#include <plat/gpio.h>
#include <lge/common.h>
#include <lge/board.h>
#include <lge/board_rev.h>
#include <linux/err.h>
#if defined(CONFIG_TOUCHSCREEN_COMMON_SYNAPTICS)
#include <linux/input/lge_touch_core.h>
#endif
#include <linux/mpu.h>
#include <linux/lge/apds9900.h>
#include <linux/lge/lp8720.h>
#include <linux/lge/lm3528.h>
#include <linux/muic/muic.h>

/* LGE_CHANGE_S [yehan.ahn@lge.com] 2011-04-07,
 * [P940] general power control for device driver
 */
int device_power_control(char *reg_id, int on)
{
	struct regulator *device_regulator = NULL;
	int regulator_status = 0;
	int regulator_status_prev = 0;

	device_regulator = regulator_get_exclusive(NULL, reg_id);
	printk(KERN_INFO "power_control: device_regulator: %p",
	       device_regulator);

	if (IS_ERR(device_regulator)) {
		printk(KERN_ERR
		       "power_control: it couldn't be initialized reg_id=%s\n",
		       reg_id);
		return -1;
	}
	regulator_status_prev = regulator_is_enabled(device_regulator);
	if (regulator_status_prev < 0) {
		printk(KERN_ERR
		       "power_control: device_regulator_prev %s errno=%d\n",
		       reg_id, regulator_status_prev);
		regulator_put(device_regulator);
		return -1;
	}

	if (on) {
		if (regulator_status_prev == 0)
			regulator_enable(device_regulator);
	} else {
		if (regulator_status_prev > 0)
			regulator_disable(device_regulator);
	}

	regulator_status = regulator_is_enabled(device_regulator);
	if (regulator_status < 0) {
		printk(KERN_ERR "power_control: device_regulator %s errno=%d\n",
		       reg_id, regulator_status);
		regulator_put(device_regulator);
		return -1;
	}

	regulator_put(device_regulator);

	printk(KERN_INFO "power_control: %s : %s -> %s\n", reg_id,
	       regulator_status_prev > 0 ? "ON" : "OFF",
	       regulator_status > 0 ? "ON" : "OFF");
	return 0;
}

int vibrator_power_control(int on)
{
	return device_power_control("vib_power", on);
}

int touch_power_control(int on){
	return device_power_control("touch_power", on);
}

/* LGE_CHANGE_S [yehan.ahn@lge.com] 2011-04-16
 * [P940] Regulator initilization
 */
/* LGE_SJIT 2011-10-18 [jongrak.kwon@lge.com] Change for Kernel 3.0 */
static struct regulator_consumer_supply twl6030_vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

static struct regulator_consumer_supply twl6030_vpp_supply[] = {
	REGULATOR_SUPPLY("vpp", NULL),
};

static struct regulator_consumer_supply twl6030_vusim_supply[] = {
	REGULATOR_SUPPLY("vib_power", NULL),
};

static struct regulator_consumer_supply twl6030_vana_supply[] = {
	REGULATOR_SUPPLY("vana", NULL),
};

static struct regulator_consumer_supply twl6030_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),	//for DSS
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi2"),	// for DSI2
};

static struct regulator_consumer_supply twl6030_vdac_supply[] = {
	REGULATOR_SUPPLY("hdmi_vref", NULL),	// hdmi phy power
};

static struct regulator_consumer_supply twl6030_vusb_supply[] = {
	REGULATOR_SUPPLY("vusb", "twl6030_usb"),
};

static struct regulator_consumer_supply twl6030_vaux1_supply[] = {
	REGULATOR_SUPPLY("vaux1", NULL),
};

static struct regulator_consumer_supply twl6030_vaux2_supply[] = {
	REGULATOR_SUPPLY("mhl_1v8", NULL),
};

/* P940 !EVB only */
static struct regulator_consumer_supply twl6030_regen2_supply[] = {
	REGULATOR_SUPPLY("touch_power", NULL),
};

static struct regulator_consumer_supply twl6030_vaux3_supply[] = {
	REGULATOR_SUPPLY("cam2pwr", NULL),
};

#define TWL6030_REGULATOR_DEVICE(_id, _minmv, _maxmv, _always_on, _boot_on) \
	static struct regulator_init_data twl6030_##_id##_data = {	\
		.constraints = {					\
			.min_uV = (_minmv),				\
			.max_uV = (_maxmv),				\
			.apply_uV = 1,					\
			.always_on = (_always_on),			\
			.boot_on = (_boot_on),				\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL	\
					| REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask	 = (REGULATOR_CHANGE_VOLTAGE	\
					| REGULATOR_CHANGE_MODE		\
					| REGULATOR_CHANGE_STATUS),	\
			.state_mem = {					\
				.enabled = false,			\
				.disabled = true,			\
			},						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(twl6030_##_id##_supply),\
		.consumer_supplies = twl6030_##_id##_supply,		\
	};

// LGE_SJIT 2011-10-18 [jongrak.kwon@lge.com] Match the constraint
#ifdef CONFIG_MACH_LGE_COSMO //nthyunjin.yang 120412
TWL6030_REGULATOR_DEVICE(vmmc,  3000000, 3000000, 0, 1);	// VPMIC_VMMC
TWL6030_REGULATOR_DEVICE(vpp,   1800000, 1800000, 0, 0);	// VPMIC_OMAP_VPP_CUST
TWL6030_REGULATOR_DEVICE(vusim, 3000000, 3000000, 0, 0);	// TWL_LDO_3.0V
/* The Vusb is defined directly instead of below def() for 172777*/
/* TWL6030_REGULATOR_DEVICE(vusb, 	3300000, 3300000, 0,0);	// USB */
TWL6030_REGULATOR_DEVICE(vaux1, 2800000, 2800000, 0, 1);	// VPMIC_VAUX1
TWL6030_REGULATOR_DEVICE(vaux2, 2800000, 2800000, 1, 1);	// TWL_LDO_2.8V
TWL6030_REGULATOR_DEVICE(vaux3, 1800000, 1800000, 1, 1);	// TWL_DO_1.8V
#else
TWL6030_REGULATOR_DEVICE(vmmc,  1200000, 3300000, 0, 1);	// SD
TWL6030_REGULATOR_DEVICE(vpp,   1800000, 1800000, 0, 0);	// OMAP_VPP_CUST
TWL6030_REGULATOR_DEVICE(vusim, 3200000, 3200000, 0, 0);	// Vibrator
/* The Vusb is defined directly instead of below def() for 172777*/
/* TWL6030_REGULATOR_DEVICE(vusb, 	3300000, 3300000, 0,0);	// USB */
TWL6030_REGULATOR_DEVICE(vaux1, 2800000, 2800000, 0, 1);	// eMMC
TWL6030_REGULATOR_DEVICE(vaux2, 1800000, 1800000, 0, 0);	// MHL 1.8V
TWL6030_REGULATOR_DEVICE(vaux3, 1800000, 1800000, 0, 0);	// Cam
#endif

static struct regulator_init_data twl6030_vana_data = {
	.constraints = {
		.min_uV = 2100000,
		.max_uV = 2100000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
		.boot_on = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6030_vana_supply),
	.consumer_supplies = twl6030_vana_supply,
};

static struct regulator_init_data twl6030_vcxio_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
		.boot_on = true,
		/* modified to add VCXIO during suspend (17267) */
		.state_mem = {
			.disabled = true,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6030_vcxio_supply),
	.consumer_supplies = twl6030_vcxio_supply,
};

static struct regulator_init_data twl6030_vdac_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
		.boot_on = true,
		/* modified to add VDAC during suspend (17267) */
		.state_mem = {
			.disabled = true,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6030_vdac_supply),
	.consumer_supplies = twl6030_vdac_supply,
};

/*
	Trying match the TI Source based on the patch 17277
	adding inital_state only.
*/

static struct regulator_init_data twl6030_vusb_data = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.apply_uV = false,//true,	// LGE_BSP 2012.03.13 [myeonggyu.son@lge.com] 4AI.1.2 patch - failed on setting VUSB regulator node
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
		.state_mem = {
			.enabled = false,
			.disabled = true,
		},
		.initial_state = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6030_vusb_supply),
	.consumer_supplies = twl6030_vusb_supply,
};

static struct regulator_init_data twl6030_regen2_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6030_regen2_supply),
	.consumer_supplies = twl6030_regen2_supply,
};

static struct twl4030_madc_platform_data cosmo_gpadc_data = {
	.irq_line = 1,
};

static int cosmo_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925,		/* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880,	/* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823,	/* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755,	/* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679,	/* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599,	/* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519,	/* 50 - 59 */
	511, 504, 496		/* 60 - 62 */
};

static struct twl4030_bci_platform_data cosmo_bci_data = {
	.monitoring_interval = 10,
	.max_charger_currentmA = 1500,
	.max_charger_voltagemV = 4560,
	.max_bat_voltagemV = 4200,
	.low_bat_voltagemV = 3300,
	.battery_tmp_tbl = cosmo_batt_table,
	.tblsize = ARRAY_SIZE(cosmo_batt_table),
};

static int tps6130x_enable(int on)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &val, TWL6040_REG_GPOCTL);
	if (ret < 0) {
		pr_err("%s: failed to read GPOCTL %d\n", __func__, ret);
		return ret;
	}

	/* TWL6040 GPO2 connected to TPS6130X NRESET */
	if (on)
		val |= TWL6040_GPO2;
	else
		val &= ~TWL6040_GPO2;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, val, TWL6040_REG_GPOCTL);
	if (ret < 0)
		pr_err("%s: failed to write GPOCTL %d\n", __func__, ret);

	return ret;
}

/*
  Implementation of PDM UL errata, when twl6040 enter into sleep
  mode, the value of the PDUMP could be at logic 1 (VDDVIO) state. In this
  state, the pull down resistor on omap side results in current drain.
  Since the logic 1 reset value cannot be modified on the TWL6040, the solution
  to stop the current drain is to disable the pulldown resistor on the input of
  the PDM buffer of OMAP. (Refer omap gerrit ID 17638 for further details)
*/

static int twl6040_init(void)
{
	u8 rev = 0;
	int ret;

	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE,
			      &rev, TWL6040_REG_ASICREV);
	if (ret)
		return ret;

	/*
	 * ERRATA: Reset value of PDM_UL buffer logic is 1 (VDDVIO)
	 * when AUDPWRON = 0, which causes current drain on this pin's
	 * pull-down on OMAP side. The workaround consists of disabling
	 * pull-down resistor of ABE_PDM_UL_DATA pin
	 * Impacted revisions: ES1.1 and ES1.2 (both share same ASICREV value)
	 */
	if (rev == TWL6040_REV_1_1)
		omap_mux_init_signal("abe_pdm_ul_data.abe_pdm_ul_data",
				     OMAP_PIN_INPUT);

	return 0;
}

struct tps6130x_platform_data tps6130x_pdata = {
	.chip_enable = tps6130x_enable,
};

static struct regulator_consumer_supply twl6040_vddhf_supply[] = {
	REGULATOR_SUPPLY("vddhf", "twl6040-codec"),
};

static struct regulator_init_data twl6040_vddhf = {
	.constraints = {
		.min_uV = 4075000,
		.max_uV = 4950000,
		.apply_uV = true,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(twl6040_vddhf_supply),
	.consumer_supplies = twl6040_vddhf_supply,
	.driver_data = &tps6130x_pdata,
};

static struct twl4030_codec_audio_data twl6040_audio = {
	.vddhf_uV = 4075000,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout = 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio = &twl6040_audio,
	.vibra = &twl6040_vibra,
	.audpwron_gpio = 127,
	.naudint_irq = OMAP44XX_IRQ_SYS_2N,
	.irq_base = TWL6040_CODEC_IRQ_BASE,
	.init = twl6040_init,
};

static struct regulator_init_data cosmo_clk32kg = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		/* LGE_SJIT_S 11/18/2011 [mohamed.khadri@lge.com]
		 * CLK32K enabled by default for WLAN SDIO
		 */
		.always_on = true,
	},
};

static struct twl4030_platform_data cosmo_twldata = {
	.irq_base = TWL6030_IRQ_BASE,
	.irq_end = TWL6030_IRQ_END,

	/* Regulators */
	.vmmc = &twl6030_vmmc_data,
	.vpp = &twl6030_vpp_data,
	.vusim = &twl6030_vusim_data,
	.vana = &twl6030_vana_data,
	.vcxio = &twl6030_vcxio_data,
	.vdac = &twl6030_vdac_data,
	.vusb = &twl6030_vusb_data,
	.vaux1 = &twl6030_vaux1_data,
	.vaux2 = &twl6030_vaux2_data,
	.vaux3 = &twl6030_vaux3_data,
	.usb = &omap4_usbphy_data,
	.clk32kg = &cosmo_clk32kg,
	.madc = &cosmo_gpadc_data,
	.bci = &cosmo_bci_data,

	/* children */
	.codec = &twl6040_codec,
};

/* LGE_CHANGE_E [yehan.ahn@lge.com] */

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data sdp4430_cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

		CDC_TCXO_LDOEN1, 0
	},
};

#if defined(CONFIG_TOUCHSCREEN_COMMON_SYNAPTICS)
static struct touch_device_caps touch_caps = {
	.button_support = 1,
#if defined(CONFIG_INPUT_LGE_ANDROID_3KEYS)	// S, K, L
	.number_of_button = 3,
	.button_name = {KEY_MENU, KEY_HOME, KEY_BACK},
#else // CONFIG_INPUT_LGE_ANDROID_3KEYS
	.number_of_button = 4,
	.button_name = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH},
#endif // CONFIG_INPUT_LGE_ANDROID_3KEYS
	.button_margin = 10,
	.is_width_supported = 1,
	.is_pressure_supported = 1,
	.is_id_supported = 1,
	.max_width = 15,
	.max_pressure = 0xFF,
	.max_id = 10,
	.lcd_x = 480,
	.lcd_y = 800,
	.x_max = 1124,
	.y_max = 2040,
};

static struct touch_operation_role touch_role = {
	.operation_mode = INTERRUPT_MODE,
	.key_type = TOUCH_SOFT_KEY,
	.report_mode = 0,
	.delta_pos_threshold = 0,
	.orientation = 0,
	.booting_delay = 400,
	.reset_delay = 10,
	.report_period = 12500000,
	.suspend_pwr = POWER_SLEEP,
	.resume_pwr = POWER_WAKE,
	.irqflags = IRQF_TRIGGER_FALLING,
	.jitter_filter_enable = 1,
	.jitter_curr_ratio = 28,
	.accuracy_filter_enable = 1,	
};

static struct touch_power_module touch_pwr = {
	.use_regulator = 0,
	.power = touch_power_control,
};

static struct touch_platform_data synaptics_pdata = {
	.int_pin = GPIO_TOUCH_IRQ,
	.reset_pin = GPIO_TOUCH_RESET,
	.maker = "Synaptics",
	.caps = &touch_caps,
	.role = &touch_role,
	.pwr = &touch_pwr,
};
#endif

/* Sub-PMIC I2C platform_data */
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num = GPIO_LP8720,
};

/* Backlight I2C platform_data */
#if defined(CONFIG_BACKLIGHT_LM3528)
static struct lm3528_platform_data lm3528_pdata = {
	.gpio_hwen = GPIO_LCD_CP_EN,
};
#endif

/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add rotcpy */
static void rotcpy(s8 dst[3 * 3], const s8 src[3 * 3])
{
	memcpy(dst, src, 3 * 3);
}

/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] mpu orientation for HW A */
static s8 orientation_a[] = {
	-1, 0, 0,
	0, -1, 0,
	0, 0, 1
};

/* Sensor I2C platform_data */
static struct mpu3050_platform_data mpu3050_data = {
	.int_config = 0x10,
	.orientation = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	},
	.level_shifter = 0,
	.accel = {
		.irq		= OMAP_GPIO_IRQ(GPIO_MOTION_INT),
		.adapt_num 	= 4,
		.bus		= EXT_SLAVE_BUS_SECONDARY,
		.address	= 0x0F,
		.orientation = {
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		},
	},
	.compass = {
//		.irq		= OMAP_GPIO_IRQ(GPIO_COMPASS_INT),
		.adapt_num	= 4,
		.bus		= EXT_SLAVE_BUS_PRIMARY,
		.address	= 0x0E,
#if defined(CONFIG_MACH_LGE_COSMO_P920)
		.orientation = {
			0, 1, 0,
			1, 0, 0,
			0, 0, 1
		},
#elif defined(CONFIG_MACH_LGE_COSMO_SU760)
		.orientation = {
			0, 1, 0,
			1, 0, 0,
			0, 0, -1
		},
#else
		.orientation = {
			0, 1, 0,
			1, 0, 0,
			0, 0, 1
		},
#endif
	},
};

/* LGE_SJIT 2011-12-02 [dojip.kim@lge.com]
 * define the platform_data for APDS9900
 */
static struct apds9900_platform_data apds9900_pdata = {
	.atime = 0xf6,		// 27.2ms . minimum ALS integration time
	.wtime = 0xf6,		// 27.2ms . minimum Wait time
	.ptime = 0xff,		// 2.72ms . minimum Prox integration time
	.pers = 0x33,
	.ppcount = 0x08,
	.pdrive = 0,		// 10mA of LED Power
	.pdiode = 0x20,		// IR Diode
	.pgain = 0x00,		// 1x Prox gain
	.again = 0x00,		// 1x ALS gain
	.threshhold_high = 600,
	.threshhold_low = 500,
	.coeff_b = 210,
	.coeff_c = 40,
	.coeff_d = 78,
	.apds_ga = 227,
	.apds_df = 52,
	.irq_gpio = GPIO_APDS_IRQ,
	.ldo_gpio = GPIO_APDS_LDO,
};

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] Add muic platform data */
static struct muic_platform_data muic_pdata = {
	.gpio_int = GPIO_MUIC_INT,
#if defined(CONFIG_MHL)
	.gpio_mhl = GPIO_MHL_SEL,
	.gpio_ifx_vbus = GPIO_IFX_USB_VBUS_EN,
#endif
};

/* i2c_1_info */
static struct i2c_board_info i2c_1_info[] __initdata = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &cosmo_twldata,
	},
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &sdp4430_cdc_data,
	},
	{
		I2C_BOARD_INFO("tps6130x", 0x33),
		.platform_data = &twl6040_vddhf,
	},
};

/* i2c_2_info */
static struct i2c_board_info i2c_2_info[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_COMMON_SYNAPTICS)
	{
		I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x20),
		.irq = OMAP_GPIO_IRQ(52),
		.platform_data = &synaptics_pdata,
	},
#endif
	{
		I2C_BOARD_INFO(LM3528_I2C_NAME, LM3528_I2C_ADDR),
		.platform_data = &lm3528_pdata,
	},
};

/* i2c_3_info */
static struct i2c_board_info i2c_3_info[] __initdata = {
#if defined(CONFIG_MUIC_TSU5611)
	{
		I2C_BOARD_INFO("tsu5611", 0x44),
		.irq = OMAP_GPIO_IRQ(GPIO_MUIC_INT),
		.platform_data = &muic_pdata,
	},
#elif defined(CONFIG_MUIC_TS5USBA33402)
	{
		I2C_BOARD_INFO("ts5usba33402", 0x44),
		.irq = OMAP_GPIO_IRQ(GPIO_MUIC_INT),
		.platform_data = &muic_pdata,
	},
#elif defined(CONFIG_MUIC_MAX14526)
	{
		I2C_BOARD_INFO("max14526", 0x44),
		.irq = OMAP_GPIO_IRQ(GPIO_MUIC_INT),
		.platform_data = &muic_pdata,
	},
#endif
	{
		I2C_BOARD_INFO("apds9900", 0x39),
		.platform_data = &apds9900_pdata,
	},
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME, LP8720_I2C_ADDR),
		.platform_data = &lp8720_pdata,
	},
};

/* i2c_4_info */
static struct i2c_board_info i2c_4_info[] __initdata = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq = OMAP_GPIO_IRQ(GPIO_GYRO_INT),
		.platform_data = &mpu3050_data,
	},
#if 0 //mo2hongkeon.kim 2012-0829 block duplicated registration
	/* LGE_SJIT 2011-11-24 [dojip.kim@lge.com] 2011-11-24
	 * add kxtf9, ami306
	 */
	{
		I2C_BOARD_INFO("kxtf9", 0x0f),
		.irq = OMAP_GPIO_IRQ(GPIO_MOTION_INT),
		.platform_data = &mpu3050_data.accel,
	},
	{
		I2C_BOARD_INFO("ami306", 0x0e),
		.irq = OMAP_GPIO_IRQ(GPIO_COMPASS_INT),
		.platform_data = &mpu3050_data.compass,
	},
#endif
};

/* i2c_1_config */
static struct lge_i2c_config i2c_1_config __initdata = {
	.bus_id = 1,
	.clkrate = 400,
	.info = i2c_1_info,
	.len = ARRAY_SIZE(i2c_1_info)
};

/* i2c_2_config */
static struct lge_i2c_config i2c_2_config __initdata = {
	.bus_id = 2,
	.clkrate = 400,
	.info = i2c_2_info,
	.len = ARRAY_SIZE(i2c_2_info)
};

/* i2c_3_config */
static struct lge_i2c_config i2c_3_config __initdata = {
	.bus_id = 3,
	.clkrate = 400,
	.info = i2c_3_info,
	.len = ARRAY_SIZE(i2c_3_info)
};

/* i2c_4_config */
static struct lge_i2c_config i2c_4_config __initdata = {
	.bus_id = 4,
	.clkrate = 400,
	.info = i2c_4_info,
	.len = ARRAY_SIZE(i2c_4_info)
};

int __init lge_i2c_init(void)
{
	int fixed_rev = 0;

	cosmo_twldata.regen2 = &twl6030_regen2_data;

	lge_set_i2c_bus_info(&i2c_1_config);
	lge_set_i2c_bus_info(&i2c_2_config);
	lge_set_i2c_bus_info(&i2c_3_config);
	lge_set_i2c_bus_info(&i2c_4_config);

	return 0;
}

lge_machine_initcall(lge_i2c_init);
