/*
 *  MUIC class driver
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 * Author: Seungho Park <seungho1.park@lge.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#ifndef __LINUX_MUIC_H__
#define __LINUX_MUIC_H__

#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/i2c.h>
#include <linux/muic/muic_client.h>

typedef enum {
	MUIC_UNINITED = -1,
	MUIC_UNKNOWN = 0,	// 0 - Error in detection or unsupported accessory.
	MUIC_NONE,		// 1 - No accessory is plugged in.
	MUIC_NA_TA,		// 2 - Not used actually. Special TA for North America.
	MUIC_LG_TA,		// 3
	MUIC_TA_1A,		// 4
	MUIC_INVALID_CHG,	// 5
	MUIC_AP_UART,		// 6
	MUIC_CP_UART,		// 7
	MUIC_AP_USB,		// 8
	MUIC_CP_USB,		// 9
	MUIC_TV_OUT_NO_LOAD,	// 10 - Not used.
	MUIC_EARMIC,		// 11
	MUIC_TV_OUT_LOAD,	// 12 - Not used.
	MUIC_OTG,		// 13 - Not used.
	//!![S] 2011-07-05 by pilsu.kim@lge.com : 
	MUIC_MHL,		// 14
	//!![E] 2011-07-05 by pilsu.kim@lge.com : 
	MUIC_RESERVE1,		// 15
	MUIC_RESERVE2,		// 16
	MUIC_RESERVE3,		// 17
	MUIC_MODE_NO,		// 18
} TYPE_MUIC_MODE;

typedef enum {
	CHARGING_UNKNOWN,
	CHARGING_NONE,
	CHARGING_NA_TA,
	CHARGING_LG_TA,		// MUIC_LG_TA
	CHARGING_TA_1A,
	CHARGING_INVALID_CHG,
#ifndef CONFIG_LGE_MACH_COSMO
	CHARGING_UART,                  // MUIC_AP_UART
	CHARGING_RESERVE1,              // dummy
#endif
	CHARGING_USB,		// MUIC_AP_USB
	CHARGING_FACTORY,
} TYPE_CHARGING_MODE;

typedef enum {
	NOT_UPON_IRQ,	// 0
	UPON_IRQ,	// 1
} TYPE_UPON_IRQ;

typedef enum {
	DEFAULT,	/* 0 - Just apply the default register settings. */
	RESET,		/* 1 - Fully reset the MUIC. It takes 250msec. */
	BOOTUP,		/* 2 - TSU5611 BUG fix */
} TYPE_RESET;

#if !defined(CONFIG_MUIC_MAX14526) && !defined(CONFIG_MACH_LGE_CX2)
typedef enum {
	NO_RETAIN,
	BOOT_CP_USB,
	BOOT_AP_USB,
	BOOT_RECOVERY,
#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)    
	BOOT_MHL,
#endif 
	RETAIN_MAX,
} TYPE_RETAIN_MODE;
#else
typedef enum {
	NO_RETAIN,
	BOOT_CP_USB = 1,
	BOOT_AP_USB = 2,
	BOOT_RECOVERY = 3,
#if defined(CONFIG_MHL)
	BOOT_MHL = 4,
#endif
} TYPE_RETAIN_MODE;
#endif

struct muic_device {
	const char	*name;
	struct device	*dev;
	void (*event_handler)(struct muic_device *);
	int (*read_int_state)(struct muic_device *, char *); 
	unsigned int mode;
#if defined(CONFIG_MAX8971_CHARGER)    
	unsigned int mode_in_retain;
#endif
	int		index;
};

struct muic_ops {
	void (*irq_handler)(struct muic_device *);
};

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] Add platform_data */
struct muic_platform_data {
	int gpio_int;
#if defined(CONFIG_MHL)
	int gpio_mhl;      /* MHL specific codes */
	int gpio_ifx_vbus; /* MHL specific codes */
#endif
};

extern int muic_device_register(struct muic_device *mdev, struct muic_ops *ops);
extern void muic_device_unregister(struct muic_device *mdev);

TYPE_MUIC_MODE muic_get_mode(void);
int muic_set_mode(TYPE_MUIC_MODE mode);
#if defined(CONFIG_MAX8971_CHARGER)
TYPE_MUIC_MODE muic_detect_cable(void);
TYPE_MUIC_MODE muic_get_mode_in_retain(void);
int muic_set_mode_in_retain(TYPE_MUIC_MODE mode);
#endif
int muic_register_client(struct muic_client_device *);
s32 muic_i2c_read_byte(struct i2c_client *client, u8 addr, u8 *value);
s32 muic_i2c_write_byte(struct i2c_client *client, u8 addr, u8 value);

#if defined(CONFIG_MUIC_MAX14526)
// hunsoo.lee cosmo gpio_wk2
#if defined(CONFIG_MACH_LGE_COSMO_REV_A)
#define MUIC_INT 2
#else
#define MUIC_INT 7
#endif

#define DP3T_IN_1_GPIO	11 		/* OMAP_UART_SW */
#define DP3T_IN_2_GPIO	12 		/* IFX_UART_SW */
#define USIF_IN_1_GPIO	165 	/* USIF1_SW */
#define IFX_USB_VBUS_EN_GPIO 55	/* IFX_USB_VBUS_EN */


#define TD_INT_STAT	70000	/* INT_STAT bits settle down time since MUIC INT falls on TI chip*/
#define TD_STATUS	250000	/* STATUS bits settle down time since MUIC INT falls on TI chip */
#define TD_DP_DM	1000	/* DP, DM path settle down time since SW_CONTROL writing on TI chip */

/* I2C addresses of MUIC internal registers */
#define	DEVICE_ID	(u8)0x00
#define	CONTROL_1	(u8)0x01
#define	CONTROL_2	(u8)0x02
#define	SW_CONTROL	(u8)0x03
#define	INT_STAT	(u8)0x04
/* LGE_CHANGE_S [sungchul.jung@lge.com] 2011-09-30 CX2  change register(TSU5611) */
#if defined (CONFIG_MACH_LGE_CX2)
#define	STATUS		(u8)0x06
#else
#define	STATUS		(u8)0x05
#endif
/* LGE_CHANGE_S [sungchul.jung@lge.com] 2011-09-30 CX2	change register(TSU5611) */



/* Masks for the each bit of CONTROL_1 register */
#define	ID_2P2		(u8)0x40
#define	ID_620		(u8)0x20
#define	ID_200		(u8)0x10

/* LGE_CHANGE_S [sungchul.jung@lge.com] 2011-09-30 CX2  change register(TSU5611) */
#if defined (CONFIG_MACH_LGE_CX2)
#define VLDO	(u8)0x00	/* Sets the voltage on the UID LDO. */
#else
#define VLDO	(u8)0x08	/* Sets the voltage on the UID LDO. */
#endif
/* LGE_CHANGE_S [sungchul.jung@lge.com] 2011-09-30 CX2  change register(TSU5611) */

#define	SEMREN		(u8)0x04
#define	ADC_EN		(u8)0x02
#define	CP_EN		(u8)0x01

/* Masks for the each bit of CONTROL_2 register */
#define	INTPOL		(u8)0x80
#define	INT_EN		(u8)0x40
#define	MIC_LP		(u8)0x20
#define	CP_AUD		(u8)0x10
#define	CHG_TYPE	(u8)0x02
#define	USB_DET_DIS	(u8)0x01

/* Masks for the each bit of SW_CONTROL register */
#define	MIC_ON		(u8)0x40
#define DP			(u8)0x38
#define DM			(u8)0x07

/* DP, DM settings */
#define DP_USB		(u8)0x00
#define	DP_UART		(u8)0x08
#define	DP_AUDIO	(u8)0x10
#define	DP_OPEN		(u8)0x38
#define DM_USB		(u8)0x00
#define	DM_UART		(u8)0x01
#define	DM_AUDIO	(u8)0x02
#define	DM_OPEN		(u8)0x07


/* Combined masks of SW_CONTROLl register */
#define USB		DP_USB   | DM_USB 	/* 0x00 */
#define UART 	DP_UART  | DM_UART 	/* 0x09 */
#define AUDIO	DP_AUDIO | DM_AUDIO /* 0x12 */
#define OPEN	DP_OPEN  | DM_OPEN 	/* 0x3f */


/* Masks for the each bit of INT_STATUS register */
#define	CHGDET		(u8)0x80
#define	MR_COMP		(u8)0x40
#define	SENDEND		(u8)0x20
#define	VBUS		(u8)0x10
#define	IDNO		(u8)0x0f

/* Masks for the each bit of STATUS register */
#define	DCPORT		(u8)0x80
#define	CHPORT		(u8)0x40
#define C1COMP      (u8)0x01	/* only on MAXIM */


//changseok.kim@lge.com
/* IDNO */
#define IDNO_0000     0x00
#define IDNO_0001     0x01
#define IDNO_0010     0x02
#define IDNO_0011     0x03
#define IDNO_0100     0x04
#define IDNO_0101     0x05
#define IDNO_0110     0x06
#define IDNO_0111     0x07
#define IDNO_1000     0x08
#define IDNO_1001     0x09
#define IDNO_1010     0x0A
#define IDNO_1011     0x0B

/* DP2 */
#define COMP2_TO_DP2  	0x00
#define COMP2_TO_U2   	0x08
#define COMP2_TO_AUD2 	0x10
#define COMP2_TO_HZ   	0x20

/* DM */
#define COMN1_TO_DN1  	0x00
#define COMN1_TO_U1   	0x01
#define COMN1_TO_AUD1 	0x02
#define COMN1_TO_C1COMP 0x03
#define COMN1_TO_HZ   	0x04

/* MUIC chip vendor */
#define TS5USBA33402 	0x10
#define MAX14526 		0x20
#define ANY_VENDOR 		0xff

typedef enum {
	USIF_AP,		// 0
	USIF_DP3T,		// 1
} TYPE_USIF_MODE;

typedef enum {
	DP3T_NC,		// 0
	DP3T_AP_UART,	// 1
	DP3T_CP_UART,	// 2
	DP3T_CP_USB,	// 3
} TYPE_DP3T_MODE;

extern struct i2c_client *muic_client;
extern const char *muic_path_str[];
extern const char *charging_mode_str[];

extern TYPE_MUIC_MODE muic_path;
extern TYPE_CHARGING_MODE charging_mode;
extern atomic_t muic_charger_detected;

extern void muic_mdelay(u32 microsec);
extern void check_charging_mode(void);
extern TYPE_MUIC_MODE get_muic_mode(void);
extern int get_muic_charger_detected(void);
extern void set_muic_charger_detected(void);
extern void dp3t_switch_ctrl(TYPE_DP3T_MODE mode);
extern void usif_switch_ctrl(TYPE_USIF_MODE mode);
#endif //#if defined(CONFIG_MUIC_MAX14526)

#endif /* __LINUX_SWITCH_H__ */
