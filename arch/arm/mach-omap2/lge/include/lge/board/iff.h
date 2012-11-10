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
 */

#ifndef __LGE_BOARD_IFF_H
#define __LGE_BOARD_IFF_H

#define GPIO_LCD_POWER_EN	27	
#define GPIO_LCD_MAKER_ID	158
#define GPIO_LCD_RESET		30
#define GPIO_LCD_CP_EN		98
#define GPIO_HDMI_HPD		63

#define TPS62361_GPIO 		7

#define GPIO_GYRO_INT		28
#define GPIO_MOTION_INT		29
#define GPIO_COMPASS_INT	14

#define GPIO_CHARGER_INT	83

#define GPIO_GAUGE_INT		21
#define RCOMP_BL44JN		(0xB8)

#define GPIO_DP3T_IN_1		11	/* OMAP_UART_SW */
#define GPIO_DP3T_IN_2		12 	/* IFX_UART_SW */
#define GPIO_USIF_IN_1		165 	/* USIF1_SW */
#define GPIO_IFX_USB_VBUS_EN 	55	/* IFX_USB_VBUS_EN */
#define GPIO_MHL_SEL		177

#define GPIO_MUIC_INT		163

/* LGE_SJIT 2011-11-16 [dojip.kim@lge.com] definitions for touch */
#define GPIO_TOUCH_INT			52
#define GPIO_TOUCH_MAKER_ID		159
#define MELFAS_TOUCH_I2C_SLAVE_ADDR	0x48
#define ATMEL_TOUCH_I2C_SLAVE_ADDR	0x4A
#define TS_X_MIN			0
#define TS_X_MAX			720
#define TS_Y_MIN			0
#define TS_Y_MAX			1280

#define GPIO_BT_RESET		166
#define GPIO_BT_WAKE		160
#define GPIO_BT_HOST_WAKE	23

/* LGE_SJIT 2012-01-13 [dojip.kim@lge.com] only for rev_c */
#define GPIO_KEY_VOLUMEUP	175
#define GPIO_KEY_VOLUMEDOWN	171
#define GPIO_KEY_BASE		176

#if defined(CONFIG_INPUT_HALLEFFECT_BU52031NVX)
//#define GPIO_HALL_INT 42
#endif

/* LGE_SJIT_S 2011-12-14 [mohamed.khadri@lge.com] gpios for NFC - PN544 */
#ifdef CONFIG_PN544_NFC
#define NFC_GPIO_IRQ            4
#define NFC_GPIO_VEN            62
#define NFC_GPIO_FRIM           42
#define NFC_I2C_SLAVE_ADDR      0x28
#endif //CONFIG_PN544_NFC
/* LGE_SJIT_E 2011-12-14 [mohamed.khadri@lge.com] gpios for NFC - PN544 */

/* LGE_SJIT_S 2011-12-19 [mohamed.khadri@lge.com] gpios for GPS */
#if defined(CONFIG_GPS)
#define GPS_PWR_ON_GPIO         0
#define GPS_RESET_N_GPIO        1
#endif
/* LGE_SJIT_E 2011-12-19 [mohamed.khadri@lge.com] gpios for GPS */

/* XXX: REVISIT: find right ram_console region. the last 1MB of RAM? */
/* LGE_SJIT_E 2011-10-19 [jongrak.kwon@lge.com] Adjust the value referring to OMAP4_RAMCONSOLE value */
#ifdef CONFIG_ANDROID_RAM_CONSOLE
#ifdef CONFIG_LGE_HANDLE_PANIC
#define LGE_RAM_CONSOLE_START_DEFAULT (0xA0000000)
#define LGE_RAM_CONSOLE_SIZE_DEFAULT  (SZ_2M - (LGE_CRASH_LOG_SIZE))

#define LGE_CRASH_LOG_START (LGE_RAM_CONSOLE_START_DEFAULT + LGE_RAM_CONSOLE_SIZE_DEFAULT)
#define LGE_CRASH_LOG_SIZE  (4 * SZ_1K)

void lge_set_reboot_reason(unsigned int reason);
#else
#define LGE_RAM_CONSOLE_START_DEFAULT (0xA0000000)
#define LGE_RAM_CONSOLE_SIZE_DEFAULT  (SZ_2M)
#endif /* CONFIG_LGE_HANDLE_PANIC */
#endif /* CONFIG_ANDROID_RAM_CONSOLE */

#endif /* __LGE_BOARD_IFF_H */
