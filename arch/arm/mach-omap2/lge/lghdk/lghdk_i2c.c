/*
 * I2C devices intializing code.
 *
 * Copyright (C) 2010 LG Electronic Inc.
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

#include <linux/i2c/twl.h>
#include <linux/interrupt.h>
#include <linux/regulator/machine.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/irqs.h>
#include <plat/gpio.h>
#include <lge/common.h>
#include <lge/board.h>
#include <linux/input/touch_synaptics_rmi4_i2c.h>
/* LGE_CHANGE_S [sungchull.kim@lge.com] 2011-06-13,
  added cdc_tcxo to i2c */
#include <linux/cdc_tcxo.h>
/* LGE_CHANGE_E [sungchull.kim@lge.com] 2011-06-13 */
#include <linux/lge/apds9900.h>
#include <linux/fuel_gauge_max17043.h>

// LGE_CHANGE_S [seungmoon.lee@lge.com] 2011-07-19, general power control for device driver

int device_power_control(char* reg_id, int on){
	static struct regulator *device_regulator = NULL;

	device_regulator = regulator_get_exclusive(NULL, reg_id);
	if (!device_regulator) {
		printk(KERN_ERR "[POWER_CONTROL] it couldn't be initialized");
		return -1;
	}

	printk(KERN_INFO "[POWER_CONTROL] %s : %s", reg_id, on ? "ON" : "OFF");

	if(on)
		regulator_enable(device_regulator);
	else
		regulator_disable(device_regulator);

	regulator_put(device_regulator);
	return 0;
}

int vibrator_power_control(bool on){
	return device_power_control("vib_power", on);
}
// LGE_CHANGE_E [seungmoon.lee@lge.com] 2011-07-19

/* PMIC(TWL6030) I2C platform_data */
static struct regulator_consumer_supply lghdk_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0", // LGE_SJIT 2011-08-30 [jongrak.kwon@lge.com] Change for Kernel 3.0
	},
};

static struct regulator_consumer_supply lghdk_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};

static struct regulator_consumer_supply twl6030_vusim_supply[] = {
	REGULATOR_SUPPLY("vib_power", NULL),
};

static struct regulator_consumer_supply lghdk_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_consumer_supply lghdk_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data lghdk_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3300000,	// LGE_SJIT 2011-08-30 [jongrak.kwon@lge.com] Match the constraint 
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = lghdk_vmmc_supply,
};

static struct regulator_init_data lghdk_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data lghdk_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data lghdk_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data lghdk_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(lghdk_vcxio_supply),
	.consumer_supplies	= lghdk_vcxio_supply,
};

static struct regulator_init_data lghdk_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(lghdk_vdac_supply),
	.consumer_supplies      = lghdk_vdac_supply,

};

static struct regulator_init_data lghdk_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data lghdk_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,

		.always_on = true, /* LGE_SJIT 2011-09-28 [choongryeol.lee@lge.com] To prevent turn off during bootup */
	},
};

static struct regulator_init_data lghdk_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data lghdk_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = lghdk_cam2_supply,
};
// LGE_CHANGE_S [seungmoon.lee@lge.com] 2011-07-19, Regulator initilization

#define TWL6030_REGULATOR_DEVICE(_id, _minmv, _maxmv, _always_on)		\
	static struct regulator_init_data twl6030_##_id##_data = {		\
		.constraints = {						\
			.min_uV = (_minmv),					\
			.max_uV = (_maxmv),					\
			.apply_uV = 1,						\
			.always_on = (_always_on),				\
			.valid_modes_mask	= (REGULATOR_MODE_NORMAL	\
					| REGULATOR_MODE_STANDBY),		\
			.valid_ops_mask	 = (REGULATOR_CHANGE_VOLTAGE		\
					| REGULATOR_CHANGE_MODE			\
					| REGULATOR_CHANGE_STATUS),		\
		},								\
		.num_consumer_supplies = ARRAY_SIZE(twl6030_##_id##_supply),	\
		.consumer_supplies = twl6030_##_id##_supply,			\
	};
TWL6030_REGULATOR_DEVICE(vusim, 3000000, 3000000, 0);	// Vibrator
// LGE_CHANGE_E [seungmoon.lee@lge.com] 2011-07-19, Regulator initilization


static struct twl4030_madc_platform_data lghdk_gpadc_data = {
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

static int lghdk_batt_table[] = {
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

static struct twl4030_bci_platform_data lghdk_bci_data = {
	//.monitoring_interval		= 10,
	//.max_charger_currentmA		= 1500,
	//.max_charger_voltagemV		= 4560,
	//.max_bat_voltagemV		= 4200,
	//.low_bat_voltagemV		= 3300,
	.battery_tmp_tbl		= lghdk_batt_table,
	.tblsize			= ARRAY_SIZE(lghdk_batt_table),
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

// LGE_SJIT_S 11/18/2011 [mohamed.khadri@lge.com] CLK32K enabled by default for WLAN SDIO
static struct regulator_init_data p940_clk32kg = {
        .constraints = {
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
                .always_on              = true,
        },
};
// LGE_SJIT_E 11/18/2011 [mohamed.khadri@lge.com] CLK32K enabled by default for WLAN SDIO

static struct twl4030_platform_data lghdk_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &lghdk_vmmc,
	.vpp		= &lghdk_vpp,
	// LGE_CHANGE_S [seungmoon.lee@lge.com] 2011-07-19, change vibrator regulator
	.vusim		= &twl6030_vusim_data,
	// LGE_CHANGE_E [seungmoon.lee@lge.com] 2011-07-19, change vibrator regulator
	.vana		= &lghdk_vana,
	.vcxio		= &lghdk_vcxio,
	.vdac		= &lghdk_vdac,
	.vusb		= &lghdk_vusb,
	.vaux1		= &lghdk_vaux1,
	.vaux2		= &lghdk_vaux2,
	.vaux3		= &lghdk_vaux3,
// LGE_SJIT_S 11/18/2011 [mohamed.khadri@lge.com] CLK32K enabled by default for WLAN SDIO
	.clk32kg        = &p940_clk32kg,
// LGE_SJIT_E 11/18/2011 [mohamed.khadri@lge.com] CLK32K enabled by default for WLAN SDIO
	.usb		= &omap4_usbphy_data,
	.madc		= &lghdk_gpadc_data,
	.bci		= &lghdk_bci_data,

	/* children */
	.codec		= &twl6040_codec,
};


/* LGE_CHANGE_S [sungchull.kim@lge.com] 2011-06-13,
  added cdc_tcxo to i2c */
static struct cdc_tcxo_platform_data lghdk__cdc_data = {
	.buf = {
		CDC_TCXO_REQ4INT | CDC_TCXO_REQ1INT |
		CDC_TCXO_REQ4POL | CDC_TCXO_REQ3POL |
		CDC_TCXO_REQ2POL | CDC_TCXO_REQ1POL,

		CDC_TCXO_MREQ4 | CDC_TCXO_MREQ3 |
		CDC_TCXO_MREQ2 | CDC_TCXO_MREQ1,

		CDC_TCXO_LDOEN1,

		0 },
};
/* LGE_CHANGE_E [sungchull.kim@lge.com] 2011-06-13 */

static struct synaptics_ts_platform_data synaptics_t1320_ts_platform_data[] = {
	{
		.use_irq 	=	1,
		.irqflags 	= 	IRQF_TRIGGER_FALLING,
		.i2c_int_gpio = TS_I2C_INT_GPIO,
		.ic_booting_delay 	= 400,
		.report_period		= 12500000,
		.num_of_finger = 10,
		.num_of_button = 3,
		.button[0] = KEY_MENU,
		.button[1] = KEY_HOME,
		.button[2] = KEY_BACK,
		.x_max = 1110,
		.y_max = 1973,
		.fw_ver	= 1,
		.palm_threshold = 0,
		.delta_pos_threshold = 0,
	},
};


#if defined(CONFIG_SUBPMIC_LP8720)
#include <linux/lge/lp8720.h>
static struct lp8720_platform_data lp8720_pdata = {
	.en_gpio_num	=	99,
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
	//.gpio_pwm  = 144,	
};
#endif


#if defined(CONFIG_CAMERAFLASH_LM3559)
#include <linux/lge/lm3559.h>
static struct lm3559_platform_data	lm3559_pdata = {
	.gpio_hwen	=	191,
};
#endif

/* Sensor I2C platform_data */

/* jaekyung.oh@lge.com 2011.02.12 Start -->[
    MPU3050 Board configration
    if you wanna config ".orientation" value for your board, please mail to me.
*/
#if defined(CONFIG_MPU_SENSORS_MPU3050) || defined(CONFIG_SENSORS_MPU3050_MODULE)
#include <linux/mpu.h>
#define SENSOR_MPU_NAME "mpu3050"

static struct mpu3050_platform_data  mpu_pdata = {
    //.int_config  = 0x12,  //default=0x10   
	.int_config = 0x10,
    .orientation = {   0, -1,  0, 
                       1,  0,  0, 
                       0,  0,  1 },
    //.level_shifter = 0,
    .level_shifter = 0,
    .accel = {
	#ifdef CONFIG_SENSORS_MPU3050_MODULE
				 .get_slave_descr = NULL,
	#else
				 .get_slave_descr = get_accel_slave_descr,
	#endif
        .irq	     = OMAP_GPIO_IRQ(GPIO_MOTION_INT),
	.adapt_num   = 4,
        //.adapt_num   = 2,
        .bus         = EXT_SLAVE_BUS_SECONDARY,
        .address     = 0x0F,
        .orientation = {   1,  0,  0, 
                           0,  -1,  0, 
                           0,  0,  -1 },
    },
    .compass = {
	#ifdef CONFIG_SENSORS_MPU3050_MODULE
				 .get_slave_descr = NULL,
	#else
				 .get_slave_descr = get_compass_slave_descr,
	#endif
		.adapt_num	= 2,
		.bus         = EXT_SLAVE_BUS_PRIMARY,
        .address     = 0x0E,
        .orientation = {  0,  1,  0,
                         -1,  0,  0,
                          0,  0,  1 },
    },
};
#endif
/* jaekyung.oh@lge.com 2011.02.12 End <--]*/

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
	.coeff_c = 70,
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

/* i2c_1_info */
static struct i2c_board_info i2c_1_info[] __initdata = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &lghdk_twldata,
	},
/* LGE_CHANGE_S [sungchull.kim@lge.com] 2011-06-13,
  added cdc_tcxo to i2c */
	{
		I2C_BOARD_INFO("cdc_tcxo_driver", 0x6c),
		.platform_data = &lghdk__cdc_data,
	},
/* LGE_CHANGE_E [sungchull.kim@lge.com] 2011-06-13 */
};

/* i2c_2_info */
static struct i2c_board_info i2c_2_info[] __initdata = {
#if defined(CONFIG_LGE_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) 
	{	
		I2C_BOARD_INFO("synaptics_ts", 0x20),
	       .platform_data = &synaptics_t1320_ts_platform_data,
	       .irq = TS_I2C_INT_GPIO,
	},
#endif 
#if defined(CONFIG_BACKLIGHT_LM3528)
	{
		I2C_BOARD_INFO(LM3528_I2C_NAME,  LM3528_I2C_ADDR),
		.platform_data	=	&lm3528_pdata,
	},
#endif
//LGE_CHANGES_START_jaekyung.oh@lge.com 2011.06.20
#if defined(CONFIG_BACKLIGHT_LM3530)
	{
		I2C_BOARD_INFO(LM3530_I2C_NAME,  LM3530_I2C_ADDR),
		.platform_data	=	&lm3530_pdata,
	},
#endif
//LGE_CHANGES_END_jaekyung.oh@lge.com 2011.06.20
	{
		I2C_BOARD_INFO("lgdp4512_barrierA", 0x74),		
	},
	{
		I2C_BOARD_INFO("lgdp4512_barrierB", 0x75),
	},
#if defined(CONFIG_CAMERAFLASH_LM3559)
	{
		I2C_BOARD_INFO(LM3559_I2C_NAME,	LM3559_I2C_ADDR),
		.platform_data	=	&lm3559_pdata,
	},
#endif	
	/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] add max17043 platform_data */
	{
		I2C_BOARD_INFO("max17043_i2c",  0x36),
		.platform_data = &max17043_pdata,
	},

};

/* i2c_3_info */
static struct i2c_board_info i2c_3_info[] __initdata = {
/* MAX14526 (MUIC) */
#if defined(CONFIG_LGE_HDK_MUIC)
		{
			I2C_BOARD_INFO("max14526", 0x44),
		},
#endif	
#if defined(CONFIG_SUBPMIC_RT8053)
	{
		I2C_BOARD_INFO(RT8053_I2C_NAME, RT8053_I2C_ADDR),
		.platform_data	=	&rt8053_pdata,
	},
#endif
#if defined(CONFIG_SUBPMIC_LP8720)
	{
		I2C_BOARD_INFO(LP8720_I2C_NAME,  LP8720_I2C_ADDR),
		.platform_data =&lp8720_pdata,	
	},
#endif

/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21, */
#if defined(CONFIG_CHARGER_MAX8971)
	{
		I2C_BOARD_INFO(MAX8971_I2C_NAME,  0x35),
		.irq = GPIO_CHARGER_INT,
	},
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */

#if defined(CONFIG_LG_FW_STC3105_FUEL_GAUGE)
	{
		I2C_BOARD_INFO("stc3105",  0x70),
		.platform_data =&stc3105_pdata,
	},
#endif

};

/* i2c_4_info */
static struct i2c_board_info i2c_4_info[] __initdata = {
/* jaekyung.oh@lge.com 2011.02.12 Start -->[ */
#if defined(CONFIG_MPU_SENSORS_MPU3050) || defined(CONFIG_SENSORS_MPU3050_MODULE)
	{
		 I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = OMAP_GPIO_IRQ(GPIO_GYRO_INT),
		.platform_data = &mpu_pdata,
	},
#endif
#if defined( CONFIG_SENSORS_APDS9900 )
	{
		I2C_BOARD_INFO("apds9900", 0x39),
		.platform_data = &apds9900_pdata,
	},
#endif
/* jaekyung.oh@lge.com 2011.02.12 End   <--] */
#ifdef CONFIG_PN544_NFC
/*dongjoon.kim@lge.com 2011.06.07 NFC C2 bring-up[START]*/
        {
                I2C_BOARD_INFO("pn544", NFC_I2C_SLAVE_ADDR),
                .type = "pn544",
                .irq = OMAP_GPIO_IRQ(NFC_GPIO_IRQ),
                .platform_data = &nfc_pdata,
        },
/*dongjoon.kim@lge.com 2011.06.07 NFC C2 bring-up[END]*/
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
	lge_set_i2c_bus_info(&i2c_1_config);
	lge_set_i2c_bus_info(&i2c_2_config);
	lge_set_i2c_bus_info(&i2c_3_config);
	lge_set_i2c_bus_info(&i2c_4_config);

	return 0;
}

lge_machine_initcall(lge_i2c_init);
