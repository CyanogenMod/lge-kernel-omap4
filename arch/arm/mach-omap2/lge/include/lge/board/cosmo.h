/*
 * LGE board specific header file.
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LGE_BOARD_COSMO_H
#define __LGE_BOARD_COSMO_H

#define GPIO_LCD_POWER_EN	27	
#define GPIO_LCD_MAKER_ID	158
#define GPIO_LCD_RESET		30
#define GPIO_LCD_EXT_TE		103
#define GPIO_LCD_CP_EN		98
#define GPIO_HDMI_HPD		63

#define GPIO_3D_LCD_POWER_EN	144	
#define GPIO_3D_LCD_BANK_SEL	190

#define TPS62361_GPIO 		7

#define GPIO_MUIC_INT_A 	2 	/* for HW A */
#define GPIO_MUIC_INT 		7

/* LGE_SJIT 2011-12-07 [dojip.kim@lge.com] max17043 (fuelgauge) */
#define GPIO_GAUGE_INT_EVB	42 	/* for EVB */
#define GPIO_GAUGE_INT		21
#define RCOMP_BL44JN		(0xB8)
#define GPIO_GUAGE_SCL
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_GPIO)
#define GPIO_FUELGUAGE_I2C_SCL    157
#define GPIO_FUELGUAGE_I2C_SDA    163
#define GPIO_FUELGUAGE_I2C_UDELAY    5
#define GPIO_FUELGUAGE_I2C_TIMEOUT    20
#endif

#define GPIO_DP3T_IN_1		11 	/* OMAP_UART_SW */
#define GPIO_DP3T_IN_2		12 	/* IFX_UART_SW */
#define GPIO_USIF_IN_1		165 	/* USIF1_SW */
#define GPIO_IFX_USB_VBUS_EN 	55	/* IFX_USB_VBUS_EN */

#define GPIO_GYRO_INT		28
#define GPIO_MOTION_INT		29
#define GPIO_COMPASS_INT	13

#define GPIO_LP8720		99	/* Camera SubPMIC */
#define GPIO_LP8720_C		60	/* HW Rev.C or later */

#define GPIO_TOUCH_IRQ		52
#define GPIO_TOUCH_RESET	14

#define GPIO_APDS_IRQ		16	/* Proxi. + Light sensor */
#define GPIO_APDS_LDO		104

#if defined(CONFIG_INPUT_HALLEFFECT_BU52031NVX)
#define GPIO_HALL_INT		42
#endif

/* LGE_SJIT 2011-12-23 [jongrak.kwon@lge.com] gpios for reconfigure in kernel */
#define GPIO_FRONT_KEY_LED_EN	82
#define GPIO_CHG_STATUS_N	51
#ifdef CONFIG_MACH_LGE_COSMO //nthyunjin.yang 120426
#define GPIO_CAM_SUBPM_EN	99
#else
#define GPIO_CAM_SUBPM_EN	60
#endif
#define GPIO_RESET_PMU_N	164
#define GPIO_IFX_PWR_ON_SW	3
#define GPIO_PROXI_INT		16

#define GPIO_BT_RESET		166
#define GPIO_BT_WAKE		23
#define GPIO_BT_HOST_WAKE	160

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#if defined(CONFIG_LGE_HANDLE_PANIC)
//#define LGE_RAM_CONSOLE_START_DEFAULT (PLAT_PHYS_OFFSET + (388 * SZ_1M))
//#define LGE_RAM_CONSOLE_START_DEFAULT (0x98C00000) // 512MB
//#define LGE_RAM_CONSOLE_START_DEFAULT (0x99300000) // 512MB
#define LGE_RAM_CONSOLE_START_DEFAULT (0x9AA00000) // 512MB


#define LGE_RAM_CONSOLE_SIZE_DEFAULT  (SZ_1M - (4 * SZ_1K))

#define LGE_CRASH_LOG_START (LGE_RAM_CONSOLE_START_DEFAULT + LGE_RAM_CONSOLE_SIZE_DEFAULT)
#define LGE_CRASH_LOG_SIZE  (4 * SZ_1K)

void lge_set_reboot_reason(unsigned int reason);
#else
#define LGE_RAM_CONSOLE_START_DEFAULT (PLAT_PHYS_OFFSET + (455 * SZ_1M))
#define LGE_RAM_CONSOLE_SIZE_DEFAULT  (SZ_1M)
#endif /* CONFIG_LGE_HANDLE_PANIC */
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

#ifdef CONFIG_LGE_BROADCAST_TDMB
#define COSMO_TDMB_IRQ_GPIO		44
#endif /* CONFIG_LGE_BROADCAST_TDMB */

/* LGE_SJIT_S 2011-12-19 [mohamed.khadri@lge.com] gpios for GPS */
#if defined(CONFIG_GPS)
#define GPS_PWR_ON_GPIO		0
#define GPS_RESET_N_GPIO	1
#if defined(CONFIG_P940_GPS_LNA_SD_USE)
#define GPS_LNA_SD_GPIO		140
#endif
#endif
/* LGE_SJIT_E 2011-12-19 [mohamed.khadri@lge.com] gpios for GPS */

#endif /* __LGE_BOARD_COSMO_H */
