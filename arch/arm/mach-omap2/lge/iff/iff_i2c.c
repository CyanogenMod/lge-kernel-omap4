/*
 * I2C devices intializing code.
 *
 * Copyright (C) 2010, 2011 LG Electronic Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <mach/irqs.h>
#include <plat/gpio.h>
#include <lge/common.h>
#include <lge/board.h>
/* LGE_CHANGE_S [sungchull.kim@lge.com] 2011-06-13,
  added cdc_tcxo to i2c */
#include <linux/cdc_tcxo.h>
/* LGE_CHANGE_E [sungchull.kim@lge.com] 2011-06-13 */
/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] melfas touch mms-136 */
#include <linux/i2c/melfas_ts.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/lge/apds9900.h>
#include <linux/fuel_gauge_max17043.h>
#include <linux/muic/muic.h>
#include <lge/board_rev.h>

// LGE_CHANGE_S [seungmoon.lee@lge.com] 2011-07-19, general power control for device driver
/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] fix the sync error
 * remove 'static' in definition of regulator
 * [P940] regulator: remove sync error
 * author: yehan.ahn@lge.com
 */
int device_power_control(char* reg_id, int on){
	struct regulator *device_regulator = NULL;
	int regulator_status = 0;
	int regulator_status_prev = 0;

	device_regulator = regulator_get_exclusive(NULL, reg_id);
	if (IS_ERR(device_regulator)) {
		printk(KERN_ERR "[POWER_CONTROL] it couldn't be initialized");
		return -1;
	}

	regulator_status_prev = regulator_is_enabled(device_regulator);
	if (regulator_status_prev < 0) {
		printk(KERN_ERR "[POWER_CONTROL] device_regulator_prev %s errno=%d\n", reg_id, regulator_status_prev);
		regulator_put(device_regulator);
		return -1;
	}

	if (on) {
		if (regulator_status_prev == 0) {
			regulator_enable(device_regulator);
		}
	}
	else {
		if (regulator_status_prev > 0) {
			regulator_disable(device_regulator);
		}
	}

	regulator_status = regulator_is_enabled(device_regulator);
	if (regulator_status < 0) {
		printk(KERN_ERR "[POWER_CONTROL] device_regulator %s errno=%d\n", reg_id, regulator_status);
		regulator_put(device_regulator);
		return -1;
	}

	regulator_put(device_regulator);
	printk(KERN_INFO "[POWER_CONTROL] %s: %s -> %s\n", reg_id, regulator_status_prev > 0? "ON" : "OFF", regulator_status > 0? "ON" : "OFF");
	return 0;
}

int vibrator_power_control(bool on){
	return device_power_control("vib_power", on);
}
// LGE_CHANGE_E [seungmoon.lee@lge.com] 2011-07-19

/* PMIC(TWL6030) I2C platform_data */
static struct regulator_consumer_supply iff_vmmc_supply[] = {
	REGULATOR_SUPPLY("vmmc","omap_hsmmc.0"),	// LGE_SJIT 2011-08-30 [jongrak.kwon@lge.com] Change for Kernel 3.0
};

static struct regulator_consumer_supply iff_vusim_supply[] = {
	REGULATOR_SUPPLY("vib_power", NULL),
};

static struct regulator_consumer_supply iff_vcxio_supply[] = {

        REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
        REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_consumer_supply iff_vdac_supply[] = {
	REGULATOR_SUPPLY("hdmi_vref", NULL),
};

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com]
 * touch LDO is controlled by TWL6030 REGEN2
 */
static struct regulator_consumer_supply iff_regen2_supply[] = {
	REGULATOR_SUPPLY("touch_en", NULL),
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data iff_vmmc = {   /* VPMIC_VMMC_S */
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3300000,	// LGE_SJIT 2011-08-30 [jongrak.kwon@lge.com] Match the constraint 
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem 		= {
						.enabled = false,
						.disabled = true,
					},
		.initial_state          = PM_SUSPEND_MEM,
		.boot_on 		= true,
	},
	.num_consumer_supplies  	= ARRAY_SIZE(iff_vmmc_supply),
	.consumer_supplies      	= iff_vmmc_supply,
};

static struct regulator_init_data iff_vpp = {   /* VPMIC_OMAP_VPP_CUST_S */
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,
	},
};

/* Vibrator */
static struct regulator_init_data iff_vusim = {  /* TWL_LDO_3.0V_S */
	.constraints = {
		.min_uV			= 3000000,
		.max_uV			= 3300000, // LGE_SJIT 02/01/2012 [mohamed.khadri@lge.com] Based on MOTOR operating range
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies 		= ARRAY_SIZE(iff_vusim_supply),
	.consumer_supplies 		= iff_vusim_supply,
};

static struct regulator_init_data iff_vana = {   /* VPMIC_VANA_S - NC*/
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data iff_vcxio = {   /* VPMIC_VCXIO_S */
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .boot_on                = true,
        /* modified to add VCXIO during suspend (17267) */
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
                .initial_state          = PM_SUSPEND_MEM,
	},
	.num_consumer_supplies		= ARRAY_SIZE(iff_vcxio_supply),
	.consumer_supplies		= iff_vcxio_supply,
};

static struct regulator_init_data iff_vdac = {  /* VPMIC_VDAC_S */
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .boot_on                = true,
        /* modified to add VDAC during suspend (17267) */
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
                .initial_state          = PM_SUSPEND_MEM,

	},
	.num_consumer_supplies  	= ARRAY_SIZE(iff_vdac_supply),
	.consumer_supplies      	= iff_vdac_supply,
};

static struct regulator_init_data iff_vusb = {   /* VPMIC_VUSB_S */
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data iff_vaux1 = {   /* VPMIC_VAUX1_S */
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.boot_on		= true,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,

	},
};


static struct regulator_init_data iff_vaux2 = { /* VPMIC_NTC_1V2 */
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 1200000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on		= true,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data iff_vaux3 = { /* TWL_LDO_2.8V_GPS - NC*/
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
                .state_mem              = {
                                            .enabled = false,
                                            .disabled = true,
                                        },
		.initial_state          = PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data iff_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com]
 * touch LDO is controlled by TWL6030 REGEN2
 */
static struct regulator_init_data iff_regen2 = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(iff_regen2_supply),
	.consumer_supplies      = iff_regen2_supply,
};

static struct twl4030_madc_platform_data iff_gpadc_data = {
	.irq_line	= 1,
};

#ifdef CONFIG_PN544_NFC
/*dongjoon.kim 2011.07.06 C2 Bring-up [START]*/
#include <linux/nfc/pn544.h>
static struct pn544_i2c_platform_data nfc_pdata = {
        .irq_gpio = NFC_GPIO_IRQ,
        .ven_gpio = NFC_GPIO_VEN,
        .firm_gpio = NFC_GPIO_FRIM,
};
/*dongjoon.kim 2011.07.06 C2 Bring-up [END]*/
#endif

static int iff_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};

static struct twl4030_bci_platform_data iff_bci_data = {
	//.monitoring_interval		= 10,
	//.max_charger_currentmA		= 1500,
	//.max_charger_voltagemV		= 4560,
	//.max_bat_voltagemV		= 4200,
	//.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= iff_batt_table,
	.tblsize			= ARRAY_SIZE(iff_batt_table),
};

static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	//.max_timeout	 = 15000,
	//.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base		= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_platform_data iff_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &iff_vmmc,
	.vpp		= &iff_vpp,
	.vusim		= &iff_vusim,
	.vana		= &iff_vana,
	.vcxio		= &iff_vcxio,
	.vdac		= &iff_vdac,
	.vusb		= &iff_vusb,
	.vaux1		= &iff_vaux1,
	.vaux2		= &iff_vaux2,
	.vaux3		= &iff_vaux3,
	.usb		= &omap4_usbphy_data,
	.madc		= &iff_gpadc_data,
	.bci		= &iff_bci_data,
	/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com]
	 * touch LDO is controlled by TWL6030 REGEN2
	 */
	.regen2		= &iff_regen2,

	/* children */
	.codec		= &twl6040_codec,
};

/*
 * The Clock Driver Chip (TCXO) on OMAP4 based SDP needs to
 * be programmed to output CLK1 based on REQ1 from OMAP.
 * By default CLK1 is driven based on an internal REQ1INT signal
 * which is always set to 1.
 * Doing this helps gate sysclk (from CLK1) to OMAP while OMAP
 * is in sleep states.
 */
static struct cdc_tcxo_platform_data iff_cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

		CDC_TCXO_LDOEN1,

		0 },
};

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] melfas touch mms-136 */
static void __init touch_gpio_init(void)
{
	if (gpio_request_one(GPIO_TOUCH_INT, GPIOF_IN, "touch_int") < 0) {
		printk(KERN_ERR "[Touch] Can't request TOUCH_INT GPIO\n");
	}
	if (gpio_request_one(GPIO_TOUCH_MAKER_ID, GPIOF_IN, "touch_maker_id") < 0) {
		printk(KERN_ERR "[Touch] Can't request TOUCH_MAKER_ID GPIO\n");
	}
}

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] melfas touch mms-136 */
static int touch_set_vreg(int on, bool log_en)
{
	int ret = 0;

	if (log_en) {
		printk(KERN_INFO "[Touch] %s: power %s\n",
				__func__, on ? "On" : "Off");
	}

	ret = device_power_control("touch_en", on);

	return ret;
}

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] melfas touch mms-136 */
static struct melfas_tsi_platform_data melfas_touch_pdata = {
	/* FIXME: */
#if 1
	.gpio_scl = 128,
	.gpio_sda = 129,
	.gpio_ldo = -1,
	.i2c_int_gpio = GPIO_TOUCH_INT,
#endif
	.power_enable = touch_set_vreg,
	.ic_booting_delay = 400,       /* ms */
	.report_period    = 12500000,  /* 12.5 msec */
	.num_of_finger    = 10,
	.num_of_button    = 4,
	.button[0]        = KEY_MENU,
	.button[1]        = KEY_HOME,
	.button[2]        = KEY_BACK,
	.button[3]        = KEY_SEARCH,
	.x_max            = TS_X_MAX,
	.y_max            = TS_Y_MAX,
	.fw_ver           = 1,
	.palm_threshold   = 0,
	.delta_pos_threshold = 0,
};


#if defined(CONFIG_SUBPMIC_LP8720)
#include <linux/lge/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num	=	60,
};
#endif

#if defined(CONFIG_SUBPMIC_RT8053)
#include <linux/lge/rt8053.h>
static struct rt8053_platform_data rt8053_pdata = {
	.en_gpio_num	=	99,
};
#endif

#if defined(CONFIG_BACKLIGHT_LM3528)
#include <linux/lge/lm3528.h>
static struct lm3528_platform_data	lm3528_pdata = {
	.gpio_hwen	=	98,
};
#endif

#if defined(CONFIG_BACKLIGHT_LM3530)
#include <linux/lge/lm3530.h>
static struct lm3530_platform_data	lm3530_pdata = {
	.gpio_hwen	=	GPIO_LCD_CP_EN,
};
#endif


#if defined(CONFIG_CAMERAFLASH_LM3559)
#include <linux/lge/lm3559.h>
static struct lm3559_platform_data	lm3559_pdata = {
	.gpio_hwen	=	191,
};
#endif

/* LGE_SJIT 2012-01-19 [dojip.kim@lge.com] add rotcpy */
static void rotcpy(s8 dst[3*3], const s8 src[3*3])
{
	memcpy(dst, src, 3*3);
}

static s8 mpu_orientation_c[] = {
	0, -1, 0,
	1,  0, 0,
	0,  0, 1
};

static s8 accel_orientation_c[] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

static s8 compass_orientation_c[] = {
	0, -1, 0,
	1,  0, 0,
	0,  0, 1
};

/* Sensor I2C platform_data */
static struct mpu_platform_data mpu_data = {
	.int_config = 0x10,
	.orientation = {
		-1, 0,  0,
		0,  1,  0,
		0,  0,  -1
	},
	.level_shifter = 0,
	.accel = {
		.irq		= OMAP_GPIO_IRQ(GPIO_MOTION_INT),
		.adapt_num 	= 4,
		.bus		= EXT_SLAVE_BUS_SECONDARY,
		.address	= 0x0F,
		.orientation = {
			1,  0,  0,
			0,  -1, 0,
			0,  0, -1
		},
	},
	.compass = {
		.irq		= OMAP_GPIO_IRQ(GPIO_COMPASS_INT),
		.adapt_num	= 4,
		.bus		= EXT_SLAVE_BUS_PRIMARY,
		.address	= 0x0E,
		.orientation = {
			0,  1,  0,
			1,  0,  0,
			0,  0, -1
		},
	},
};

/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-18, */
#if defined(CONFIG_CHARGER_MAX8971)
#include <linux/max8971.h>
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */

/* LGE_SJIT 2011-12-02 [dojip.kim@lge.com]
 * define the platform_data for APDS9900
 */
static struct apds9900_platform_data apds9900_pdata = {
	.atime    = 0xdb, // minimum ALS integration time
	.wtime    = 0xff, // minimum Wait time
	.ptime    = 0xff, // minimum Prox integration time
	.pers     = 0x33,
	.ppcount  = 0x08,
	.pdrive   = 0,    // 10mA of LED Power
	.pdiode   = 0x20, // IR Diode
	.pgain    = 0x00, // 1x Prox gain
	.again    = 0x00, // 1x ALS gain
	.threshhold_high = 600,
	.threshhold_low  = 500,
	.coeff_b = 223,
	.coeff_c = 74,
	.coeff_d = 142,
	.apds_ga = 48,
	.apds_df = 52,
	.irq_gpio = 16,
	.ldo_gpio = 104,
};

/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add max17043 platform_data */
static struct max17043_platform_data max17043_pdata = {
	.gpio_alert = GPIO_GAUGE_INT,
	.rcomp = RCOMP_BL44JN,
};

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] add muic platform data */
static struct muic_platform_data muic_pdata = {
	.gpio_int = GPIO_MUIC_INT,
	.gpio_mhl = GPIO_MHL_SEL,
	.gpio_ifx_vbus = GPIO_IFX_USB_VBUS_EN,
};

/* i2c_1_info */
static struct i2c_board_info i2c_1_info[] __initdata = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &iff_twldata,
	},
	{    
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &iff_cdc_data,
	},
};

/* i2c_2_info */
static struct i2c_board_info i2c_2_info[] __initdata = {
#if defined(CONFIG_BACKLIGHT_LM3530)
        /* Backlight */
	{
		I2C_BOARD_INFO(LM3530_I2C_NAME, LM3530_I2C_ADDR),
		.platform_data = &lm3530_pdata,
	},
#endif 
        /* MUIC */
	/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com]
	 * Add muic platform data
	 */
	{
		I2C_BOARD_INFO("ts5usba33402", 0x44),
		.irq = OMAP_GPIO_IRQ(GPIO_MUIC_INT),
		.platform_data = &muic_pdata,
	},
/* jaekyung.oh@lge.com 2011.02.12 End   <--] */
#ifdef CONFIG_PN544_NFC
/*dongjoon.kim@lge.com 2011.06.07 NFC C2 bring-up[START]*/
        {
                I2C_BOARD_INFO("pn544", NFC_I2C_SLAVE_ADDR),
                .type = "pn544",
                .irq = OMAP_GPIO_IRQ(NFC_GPIO_IRQ),
                .platform_data = &nfc_pdata,
        },
#endif
	/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] melfas touch mms-136 */
        {
                I2C_BOARD_INFO(MELFAS_TS_NAME, MELFAS_TOUCH_I2C_SLAVE_ADDR),
               .platform_data = &melfas_touch_pdata,
               .irq = OMAP_GPIO_IRQ(GPIO_TOUCH_INT),
        },
#if defined( CONFIG_SENSORS_APDS9900 )
	{
		I2C_BOARD_INFO("apds9900", 0x39),
                .platform_data = &apds9900_pdata,
	},
#endif

};

/* i2c_3_info - all Camera Related */
static struct i2c_board_info i2c_3_info[] __initdata = {
#if defined(CONFIG_SUBPMIC_LP8720)
        /* Sub PMIC-CAM */
        {
                I2C_BOARD_INFO(LP8720_I2C_NAME, LP8720_I2C_ADDR),
                .platform_data = &lp8720_pdata,
        },
#endif
#if defined(CONFIG_CAMERAFLASH_LM3559)
        /* Camera Flash LED - Connected throug Sub - FPCP */
        {
                I2C_BOARD_INFO(LM3559_I2C_NAME, LM3559_I2C_ADDR),
                .platform_data = &lm3559_pdata,
        },
#endif

/* TODO - Add 8M MIPI Camera */
/* TODO - Add VGA MIPI - Connected on Top FPCB*/
/* TODO - If all camera on I2C3 driven by ducati , then investigate if i2c3 info needs to be removed  */

};

/* i2c_4_info */
static struct i2c_board_info i2c_4_info[] __initdata = {
	{
		I2C_BOARD_INFO("mpu3050",0x68),
		.irq = OMAP_GPIO_IRQ(GPIO_GYRO_INT),
		.platform_data = &mpu_data,
	},
	/* LGE_SJIT 2011-11-24 [dojip.kim@lge.com] 2011-11-24
	 * add kxtf9, ami306
	 */
	{
		I2C_BOARD_INFO("kxtf9",0x0f),
		.irq = OMAP_GPIO_IRQ(GPIO_MOTION_INT),
		.platform_data = &mpu_data.accel,
	},
	{
		I2C_BOARD_INFO("ami306",0x0e),
		.irq = OMAP_GPIO_IRQ(GPIO_COMPASS_INT),
		.platform_data = &mpu_data.compass,
	},
	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add max17043 platform data */
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C)
        /* Fuel Gauge */
        {
                I2C_BOARD_INFO("max17043_i2c", 0x36),
		.platform_data = &max17043_pdata,
        },
#endif
#if defined(CONFIG_CHARGER_MAX8971)
	{
		I2C_BOARD_INFO(MAX8971_I2C_NAME,  0x35),
		.irq = GPIO_CHARGER_INT,
	},
#endif

};


/* i2c_1_config */
static struct lge_i2c_config i2c_1_config __initdata = {
	.bus_id	 = 1,
	.clkrate = 400,
	.info	 = i2c_1_info,
	.len	 = ARRAY_SIZE(i2c_1_info)
};

/* i2c_2_config */
static struct lge_i2c_config i2c_2_config __initdata = {
	.bus_id	 = 2,
	.clkrate = 400,
	.info	 = i2c_2_info,
	.len	 = ARRAY_SIZE(i2c_2_info)
};

/* i2c_3_config */
static struct lge_i2c_config i2c_3_config __initdata = {
	.bus_id	 = 3,
	.clkrate = 400,
	.info	 = i2c_3_info,
	.len	 = ARRAY_SIZE(i2c_3_info)
};

/* i2c_4_config */
static struct lge_i2c_config i2c_4_config __initdata = {
	.bus_id	 = 4,
	.clkrate = 400,
	.info	 = i2c_4_info,
	.len	 = ARRAY_SIZE(i2c_4_info)
};


int __init lge_i2c_init(void)
{
	/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com]
	 * call it for initialization of gpios...
	 */
	touch_gpio_init();

	/* LGE_SJIT 2012-01-19 [dojip.kim@lge.com]
	 * Fix sensor orientation
	 */
	if (system_rev == LGE_PCB_C) {
		rotcpy(mpu_data.orientation, mpu_orientation_c);
		rotcpy(mpu_data.accel.orientation, accel_orientation_c);
		rotcpy(mpu_data.compass.orientation, compass_orientation_c);
	}

	lge_set_i2c_bus_info(&i2c_1_config);
	lge_set_i2c_bus_info(&i2c_2_config);
	lge_set_i2c_bus_info(&i2c_3_config);
	lge_set_i2c_bus_info(&i2c_4_config);

	return 0;
}

lge_machine_initcall(lge_i2c_init);
