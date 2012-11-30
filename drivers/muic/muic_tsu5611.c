 /*
  * TI MUIC TSU5611 driver
  *
  * Copyright (C) 2012 LGE Inc.
  *
  * Author: HunSoo Lee <hunsoo.lee@lge.com>
  *         Seungho Park <seungho1.park@lge.com>
  *         2011/11/22:change driver structure to use MUIC subsystem.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
  */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/muic/muic.h>
#if !defined(CONFIG_MAX8971_CHARGER)
#include <linux/charger_rt9524.h>
#else
#include <linux/max8971.h>
#endif

#include <asm/system.h>
#include <asm/gpio.h>

#include <lge/board.h>

#include <linux/muic/muic_tsu5611.h>

static int muic_retain_mode = NO_RETAIN;

static struct i2c_client *tsu5611 = NULL;

typedef enum int_{
	FIRST_INT = 0,
	SECONT_INT,
}INT;

static int Int_Status = FIRST_INT;

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com]
 * Add private device handle
 */
struct ts5usb_device {
	struct i2c_client *client;
	int gpio_int;
	int gpio_mhl;
	int gpio_ifx_vbus;
	int irq;
	struct delayed_work	muic_delayed_wq;
	struct wake_lock muic_wake_lock;
};

/*
 * Initialize MUIC, i.e., the CONTROL_1,2 and SW_CONTROL registers.
 * 1) Prepare to sense INT_STATUS1 and STATUS bits.
 * 2) Open MUIC paths. -> To keep the path from uboot setting, remove this stage.
 */
void muic_init_tsu5611(struct i2c_client *client, TYPE_RESET reset)
{
	dev_info(&client->dev, "%s: reset type %d\n", __func__, reset);

	muic_i2c_write_byte(client, SW_CONTROL, OPEN);
	muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);

	/* When boot up timing, CHG_TYP must be set within TSU5611.
	 * This is chip bug of TSU5611.
	 * Eg: SU540, KU5400, LU5400
	 * TODO: boot up init and reset init may be seperated,
	 *       cause those function implementation is ambigous
	 *       hunsoo.lee@lge.com
	 */
	if (BOOTUP == reset) { /* TSU5611 BUG fix */
		dev_info(&client->dev, "%s by BOOTUP\n", __func__);

		muic_i2c_write_byte(client, CONTROL_2, CHG_TYP);
		mdelay(250);
		muic_i2c_write_byte(client, CONTROL_2, INT1_EN | CHG_TYP);
	}else {
#if defined(CONFIG_MHL_TX_MUIC_BUG_FIX)
		muic_i2c_write_byte(client, CONTROL_2, INT1_EN);
#else
		muic_i2c_write_byte(client, CONTROL_2, INT1_EN | CHG_TYP);
#endif
	}
}
EXPORT_SYMBOL(muic_init_tsu5611);

#if 0
static int muic_set_mhl_mode(struct muic_client_device *mcdev)
{
	struct i2c_client *client = NULL;
	struct ts5usb_device *dev = dev_get_drvdata(&mcdev->dev);

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);		
		return -EINVAL;
	}

	client = dev->client;

	dev_info(&client->dev, "%s enter.\n", __func__);

	muic_i2c_write_byte(client, SW_CONTROL, OPEN);

	return 0;
}
#endif

/* MICRO_USB ->MUIC->DP3T->CP_USB */
/* DM,DP => UART_TX, RX => USB D+ IFX, USB D1 IFX */
static int muic_set_cp_usb_mode(struct muic_client_device *mcdev)
{
	struct i2c_client *client = NULL;
	struct ts5usb_device *dev = dev_get_drvdata(&mcdev->dev);

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);
		return -EINVAL;
	}

	client = dev->client;

	dev_info(&client->dev, "%s enter.\n", __func__);

	/* Enables 200K, SEMREN (0x01=0x14) : No need control CP_EN */
  	muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN /*|CP_EN*/);

  	/* Enables UART Path (0x03=0x09) */
  	muic_i2c_write_byte(client, SW_CONTROL, UART);

	return 0;
}

/* MICRO_USB ->MUIC->DP3T->AP_UART */
/* DM,DP => UART_TX, RX => UART4_TX_OMAP, RX_OMAP */
static int muic_set_ap_uart_mode(struct muic_client_device *mcdev)
{
	struct i2c_client *client = NULL;
	struct ts5usb_device *dev = dev_get_drvdata(&mcdev->dev);

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);
		return -EINVAL;
	}

	client = dev->client;

	dev_info(&client->dev, "%s enter.\n", __func__);

	/* Enables 200K, SEMREN (0x01=0x14) : No need control CP_EN */
  	muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN /*|CP_EN*/);

  	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(client, SW_CONTROL, UART);

	return 0;
}

/* MICRO_USB ->MUIC->DP3T->USIF1->CP_UART */
/* DM,DP => UART_TX, RX => UART_TX_SW, RX_SW => UART_TX_IFX, UART_RX_IFX */
static int muic_set_cp_uart_mode(struct muic_client_device *mcdev)
{
	struct i2c_client *client = NULL;
	struct ts5usb_device *dev = dev_get_drvdata(&mcdev->dev);

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);
		return -EINVAL;
	}

	client = dev->client;

	dev_info(&client->dev, "%s enter.\n", __func__);

	/* Enables 200K, SEMREN (0x01=0x14) : No need control CP_EN */
  	muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN /*|CP_EN*/);

	/* Enables UART Path (0x03=0x09) */
	muic_i2c_write_byte(client, SW_CONTROL, UART);

	return 0;
}

/* MICRO_USB->MUIC->AP_USB */
/* DP,DM->USB_DP_OMAP, DM_OMAP */
static int muic_set_ap_usb_mode(struct muic_client_device *mcdev)
{
	struct i2c_client *client = NULL;
	struct ts5usb_device *dev = dev_get_drvdata(&mcdev->dev);

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);
		return -EINVAL;
	}

	client = dev->client;

	dev_info(&client->dev, "%s enter.\n", __func__);

	/* In case of CP_USB TO AP_USB
	 * 1. Have delay for CP_USB communication completion
	 * 2. Set OPEN for DISCONNECT event TO UPPER LAYER
	 * 3. Have delay for VBUS ON
	 * 4. Set USB for CONNECT event TO UPPER LAYER */

	mdelay(200);

	muic_i2c_write_byte(client, SW_CONTROL, OPEN);

	mdelay(200);

	/* Enables USB Path (0x03=0x00) */
	muic_i2c_write_byte(client, SW_CONTROL, USB);

	/* Enables 200K, SEMREN (0x01=0x14) : No need control CP_EN */
  	muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN /*|CP_EN*/);

	return 0;
}

/*----------------------------------------------------------------------------
VBUS	CHGDET	IDNO (200k) 	DCPORT		Accessory
1			1	1011 (Open)		X		TA
1			1	0101 (180K)		X		LG_TA
1			1	0110 (240K)		X		1A_TA
1			1		X			1		MUIC_NA_TA (Special TA for North America)
1			1		X			0		AP USB
----------------------------------------------------------------------------*/
static void muic_set_charger_mode(struct i2c_client *client)
{
	u8 status_val = 0;
	u8 int_stat1_val = 0;
	u8 adc_value = 0;

	/* Double Check to ensure */
	muic_i2c_write_byte(client, CONTROL_2, CHG_TYP);			/* Enable Charger Type Detection and Disable Interrupt */
	mdelay(250);												/* Wait 250ms */
	muic_i2c_read_byte(client, STATUS, &status_val);			/* Read STATUS */
	muic_i2c_read_byte(client, INT_STATUS1, &int_stat1_val);	/* Read INT_STATUS1 */

	dev_info(&client->dev, "%s: (Double Check) STATUS: 0x%x, INT_STATUS1:0x%x\n",
								__func__, status_val, int_stat1_val);

	adc_value = int_stat1_val & IDNO; /* Get ADC_VALUE */

	if ((adc_value == IDNO_1011) || (adc_value == IDNO_0101)) { /* TA or LG_TA */
		muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);
		muic_i2c_write_byte(client, CONTROL_2, INT1_EN);
		muic_set_mode(MUIC_LG_TA);
	}
	else if (adc_value == IDNO_0110) { /* 1A Chareger */
		muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);
		muic_i2c_write_byte(client, CONTROL_2, INT1_EN);
		muic_set_mode(MUIC_TA_1A);
	}
	else {
		if (status_val & DCPORT) {
			dev_info(&client->dev, "%s: DCPORT\n", __func__);
			muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);
			muic_i2c_write_byte(client, CONTROL_2, INT1_EN);
			muic_set_mode(MUIC_NA_TA);
		} else {
			dev_info(&client->dev, "%s: AP_USB\n", __func__);
			muic_set_mode(MUIC_AP_USB);
		}
  	}
}

/*-------------------------------------------------------------------------------
VBUS	CHGDET	IDNO	 		Accessory
1			0	1011 (Open)		AP_USB
1			0	0101 (180K)		AP_USB
1			0	1010 (910K)		CP_USB for download and AT command
1			0	0100 (130K)		CP_UART
1			0	0010 (56K)		CP_USB
1			0	0000 (gnd)		MHL
-------------------------------------------------------------------------------*/
static TYPE_MUIC_MODE muic_get_no_charger_mode(u8 int_stat1_val)
{
	TYPE_MUIC_MODE mode = MUIC_NONE;
	u8 adc_value = int_stat1_val & IDNO;

	switch (adc_value) {
		case IDNO_0010 :	/* 56k */
		case IDNO_1010 :	/* 910k */
		case IDNO_1001 :	/* 620k */
			mode = MUIC_CP_USB;
			break;
	
		case IDNO_0100 :	/* 130k */
			mode = MUIC_CP_UART;
			break;

#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)	
		case IDNO_0000 :	/* gnd */
			mode = MUIC_MHL;
			break;
#endif

		case IDNO_1011 :	/* open */
		case IDNO_0101 :	/* 180k */
		default :
			mode = MUIC_AP_USB;
			break;
	}

	return mode;
}

/*-------------------------------------------------------------------------------
VBUS	CHGDET	IDNO	 		Accessory
0			0	0010 (56K)		AP_UART
0			0	0100 (130K)		CP_UART
-------------------------------------------------------------------------------*/
static void muic_no_vbus_set_mode(u8 int_stat1_val)
{
	u8 adc_value = int_stat1_val & IDNO;

	if(int_stat1_val & MR_COMP) {
		muic_set_mode(MUIC_NONE);
	}
	else {
		switch (adc_value) 
		{
			case IDNO_0010 :	/* 56k */
				muic_set_mode(MUIC_AP_UART);
				break;

			case IDNO_0100 :	/* 130k */
				muic_set_mode(MUIC_CP_UART);
				break;

			default:
				muic_set_mode(MUIC_NONE);
				break;
		}
	}
}

static int muic_is_retain_mode(void)
{
	if((muic_retain_mode < NO_RETAIN)||(muic_retain_mode >= RETAIN_MAX)) {
		pr_err("%s: muic_retain_mode:%d is wrong\n", __func__, muic_retain_mode);
		muic_retain_mode = NO_RETAIN;
	}

	return muic_retain_mode;
}

static int muic_process_retain_mode(void)
{
	if((muic_retain_mode < NO_RETAIN)||(muic_retain_mode >= RETAIN_MAX)) {
		pr_err("%s: muic_retain_mode:%d is wrong\n", __func__, muic_retain_mode);
		muic_retain_mode = NO_RETAIN;
	}

	switch (muic_retain_mode)
	{
		case BOOT_CP_USB :		muic_set_mode(MUIC_CP_USB);		break;
		case BOOT_AP_USB :		muic_set_mode(MUIC_AP_USB);		break;
#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)
		case BOOT_MHL :			muic_set_mode(MUIC_MHL);		break;
#endif
#if 0
		case BOOT_RECOVERY :	muic_set_mode(MUIC_CP_UART);	break;
#endif
		default:												break;
	}

	return muic_retain_mode;
}

#if defined(CONFIG_MHL_TX_MUIC_BUG_FIX)
static int tsu5611_process_reset_muic(struct i2c_client *client)
{
	if(Int_Status == FIRST_INT) {
		dev_info(&client->dev, "%s: Reset TSU5611 because of chipset bug\n", __func__);
		muic_i2c_write_byte(client, 0x09, 0x80); /* Open hidden register */
		muic_i2c_write_byte(client, 0x0b, 0xa8); /* MUIC reset */

		mdelay(300); /* delay is must 300ms ~ 600ms after reset */

		dev_info(&client->dev, "%s: Reset TSU5611 is done\n", __func__);
		Int_Status = SECONT_INT;
	}
	else {
		dev_info(&client->dev, "%s: << Second Interrupt >> \n", __func__);
		Int_Status = FIRST_INT;
	}

	return Int_Status;
}
#endif

/*-------------------------------------------------------------------------------
VBUS	CHGDET	IDNO	 		Accessory
1			0	1011 (Open)		AP_USB
1			0	0101 (180K)		AP_USB
1			0	1010 (910K)		CP_USB for download and AT command
1			0	0100 (130K)		CP_UART
1			0	0010 (56K)		CP_USB
1			0	0000 (gnd)		MHL
0			0	0100 (130K)		CP_UART
0			0	0010 (56K)		AP_UART
-------------------------------------------------------------------------------*/
static void muic_detect_device_set_mode(struct i2c_client *client, u8 int_status1_val)
{
	u8 status_val = 0;
	TYPE_MUIC_MODE real_mode = MUIC_UNKNOWN;

	dev_info(&client->dev, "%s: muic_retain_mode:%d, Int_Status:%d, INT_STATUS1:0x%2x\n",
							__func__, muic_retain_mode, Int_Status, int_status1_val);

	if(int_status1_val & VBUS) { /* VBUS  = HIGH */
		dev_info(&client->dev, "%s: VBUS is High.\n", __func__);

#if defined(CONFIG_MHL_TX_MUIC_BUG_FIX)
		if(tsu5611_process_reset_muic(client) == SECONT_INT)
			return;
#endif

		if(int_status1_val & CHGDET) {
			muic_set_charger_mode(client);
		}
		else {
			/* Read STATUS */
			muic_i2c_read_byte(client, STATUS, &status_val);
			
			dev_info(&client->dev,"%s: STATUS: 0x%x\n", __func__, status_val);

			/* Non-standard VBUS only Car charger detection */
			if(status_val & TIMEOUT_CD) {
				muic_set_charger_mode(client);
			}
			else {
				real_mode = muic_get_no_charger_mode(int_status1_val);

				/* retain mode only if no charger mode */
				if(muic_is_retain_mode()) {
#if defined(CONFIG_MAX8971_CHARGER)
					muic_set_mode_in_retain(real_mode);
#endif
					muic_process_retain_mode();
				}
				else {
					muic_set_mode(real_mode);
				}
			}
		}
	}
	else{ /* VBUS  = LOW */
		dev_info(&client->dev, "%s: VBUS is Low.\n", __func__);

		muic_no_vbus_set_mode(int_status1_val);
	}
}

#if defined(CONFIG_MACH_LGE)
extern void android_USB_disconnect(void);
#endif

static void muic_tsu5611_set_mode(struct i2c_client *client, u8 int_status1_val)
{
	TYPE_MUIC_MODE pre_mode = muic_get_mode();

	dev_info(&client->dev, "%s: INT_STATUS1:0x%x pre_mode:%d\n",
								__func__, int_status1_val, pre_mode);

	/* Branch according to the previous muic_mode */
	switch (pre_mode)
	{
		/* 
		 * MUIC_UNKNOWN is reached in two cases. Both do not have nothing to do with IRQ.
		 * First, at the initialization time where the muic_path is not available yet.
		 * Second, whenever the current muic_path detection is failed.
		 */
		case MUIC_UNKNOWN :
		/*
		 * If the previous muic_path was MUIC_NONE,
		 * the only possible condition for a MUIC IRQ is plugging in an accessory.
		 */
		case MUIC_NONE :
			muic_detect_device_set_mode(client, int_status1_val);
			break;

		/* 
		 * If the previous muic_path was MUIC_NA_TA, MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG,
		 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
		 * the only possible condition for a MUIC IRQ is plugging out the accessory.
		 * 
		 * In this case, initialize MUIC and wait an IRQ.
		 * We don't need to wait 250msec because this is not an erronous case
		 * (we need to reset the facility to set STATUS for an erronous case and
		 * have to wait 250msec) and, if this is not an erronous case, the facility
		 * was already initialized at the system booting.
		 */
		case MUIC_AP_UART :
		case MUIC_CP_UART :
			if((int_status1_val & VBUS) == 0) {
				dev_info(&client->dev, "%s: UART is removed\n", __func__);

				/* TODO: Why set OPEN???? */
				muic_i2c_write_byte(client, SW_CONTROL, OPEN);
#if defined(CONFIG_MAX8971_CHARGER)
				muic_set_mode_in_retain(MUIC_NONE);
#endif
				muic_set_mode(MUIC_NONE);
			}
			else {
/* TODO: Need to check if this case really happens???? */
#if 1
				dev_err(&client->dev, "%s: UART is not removed\n", __func__);
#else
				if((int_status1_val & IDNO) == IDNO_0010) { /* VBUS + 56K */
					muic_set_mode(MUIC_CP_USB);
				}
				else {
					dev_err(&client->dev, "%s: UART is not removed\n", __func__);
				}
#endif
			}
			break;

		case MUIC_NA_TA :
		case MUIC_LG_TA :
		case MUIC_TA_1A :
		case MUIC_INVALID_CHG :
			if (((int_status1_val & VBUS) == 0)||((int_status1_val & CHGDET) == 0)) {
				dev_info(&client->dev, "%s: TA is removed\n", __func__);
#if defined(CONFIG_MAX8971_CHARGER)
				muic_set_mode_in_retain(MUIC_NONE);
#endif
				muic_set_mode(MUIC_NONE);
			}
			else {
				dev_err(&client->dev, "%s: TA is not removed\n", __func__);
			}
			break;

		case MUIC_AP_USB :
		case MUIC_CP_USB :
			if((int_status1_val & VBUS) == 0) {
#if defined(CONFIG_MACH_LGE)
				/* 
				 * No USB disconnect irq(twl-usb) when USB is unpluged during USB mode change.
				 * USB mode change: Internal USB disconnect & connect operation.
				 * Send USB disconnect event from MUIC.
				 */
				android_USB_disconnect();
#endif
				dev_info(&client->dev, "%s: USB is removed\n", __func__);
#if defined(CONFIG_MAX8971_CHARGER)
				muic_set_mode_in_retain(MUIC_NONE);
#endif
				muic_set_mode(MUIC_NONE);
			}
			else {
				dev_err(&client->dev, "%s: USB is not removed\n", __func__);
			}
			break;

#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)
		case MUIC_MHL :
			if ((int_status1_val & VBUS) == 0) {
				dev_info(&client->dev, "%s: MHL is removed\n", __func__);
#if defined(CONFIG_MAX8971_CHARGER)
				muic_set_mode_in_retain(MUIC_NONE);
#endif
				muic_set_mode(MUIC_NONE);
			}
			else {
				dev_err(&client->dev, "%s: MHL is not removed\n", __func__);
			}
			break;
#endif
		default:
			dev_err(&client->dev, "%s: Nothing is removed\n", __func__);
#if defined(CONFIG_MAX8971_CHARGER)
			muic_set_mode_in_retain(MUIC_UNKNOWN);
#endif
			muic_set_mode(MUIC_UNKNOWN);
			break;
	}
}

static void muic_init_for_none_mode(struct i2c_client *client)
{
	TYPE_MUIC_MODE cur_mode = muic_get_mode();
	struct ts5usb_device *dev = i2c_get_clientdata(client);

	if (cur_mode == MUIC_UNKNOWN || cur_mode == MUIC_NONE) {
#if defined(CONFIG_MHL_TX_MUIC_BUG_FIX)
		/* SJIT 2012-01-27 [dojip.kim@lge.com] P940 GB
		 * tsu5611 300ms delay side effect bug fixed test code
		 */
		if (Int_Status == SECONT_INT) {
			if (!gpio_get_value(dev->gpio_mhl)) {
				dev_info(&client->dev, "%s: wait for mhl switch completed\n", __func__);
				muic_init_tsu5611(client, DEFAULT);
				gpio_set_value(dev->gpio_ifx_vbus, 0);
			}
			else {
				dev_info(&client->dev, "%s: muic: mhl switch not completed\n", __func__);

				while (!gpio_get_value(dev->gpio_mhl)) {
					udelay(500);
					muic_set_mode(MUIC_MHL);
				}

				dev_info(&client->dev, "%s: muic: mhl switch completed\n", __func__);
				muic_init_tsu5611(client, DEFAULT);
				gpio_set_value(dev->gpio_ifx_vbus, 0);
			}
		}
		else { /* FIRST_INT */
			muic_init_tsu5611(client, DEFAULT);
			gpio_set_value(dev->gpio_ifx_vbus, 0);
		}
#else
		dev_info(&client->dev, "%s: MUIC_UNKNOWN or MUIC_NONE mode\n", __func__);
		muic_init_tsu5611(client, DEFAULT);
		gpio_set_value(dev->gpio_ifx_vbus, 0);
#endif
	}
}

void muic_tsu5611_detect_accessory(struct i2c_client *client)
{
	u8 int_status1_val = 0;

	/* Read INT_STATUS1 */
	if (muic_i2c_read_byte(client, INT_STATUS1, &int_status1_val) < 0) {
		dev_err(&client->dev, "%s: INT_STATUS1 reading failed\n", __func__);
		muic_set_mode(MUIC_UNKNOWN);
		return;
	}

	/* Set mode according to the previous muic_mode */
	muic_tsu5611_set_mode(client, int_status1_val);

	/* Init muic for none mode */
	muic_init_for_none_mode(client);
}

static int __init muic_state(char *str)
{
	s32 muic_value = simple_strtol(str, NULL, 0);
	muic_retain_mode = muic_value;
	pr_info("%s: muic_value is %d\n", __func__, muic_value);
	return 1;
}
__setup("muic_state=", muic_state);

static void muic_tsu5611_wq_func(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct ts5usb_device *dev =
		container_of(dwork, struct ts5usb_device, muic_delayed_wq);

	struct i2c_client *client = NULL;

	if(!dev || !dev->client){
		pr_err("%s: dev or client is NULL.\n", __func__);
		return;
	}

	client = dev->client;

	wake_lock(&dev->muic_wake_lock);

	dev_info(&client->dev, "%s enter.\n", __func__);

	muic_tsu5611_detect_accessory(client);

	wake_unlock(&dev->muic_wake_lock);

	/* App needs some time to display charger disconnect notification */
	wake_lock_timeout(&dev->muic_wake_lock, HZ / 2);
}

static irqreturn_t tsu5611_interrupt_handler(s32 irq, void *data)
{
	struct ts5usb_device *dev = (struct ts5usb_device*)data;
	
	/* Make the interrupt on MUIC_INT_N
	  * wake up OMAP which is in suspend mode */

	/*
	 * Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STATUS1 and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STATUS1 and STATUS contents will be held).
	 *
	 * Do this only if tsu5611_interrupt_handler was called.
	 */
	  
	schedule_delayed_work( &dev->muic_delayed_wq, msecs_to_jiffies(70));

	return IRQ_HANDLED;
}

static int muic_int_stat_read(struct muic_device *mdev, char *buf)
{	
	u32 ret = 0;
	u8 value = 0;
	unsigned int len = 0;

	struct i2c_client *client = NULL;

	if(!tsu5611){
		pr_err("%s: tsu5611 is NULL.\n", __func__);
		return 0;
	}

	client = tsu5611;

	ret = muic_i2c_read_byte(client, INT_STATUS1, &value);

	if (ret < 0) {
		dev_err(&client->dev, "%s: INT_STATUS1 reading failed\n", __func__);
		return 0;
	}

	dev_info(&client->dev, "%s: INT_STATUS1 = 0x%02x\n", __func__, (0xff & value));
	
	len = sprintf(buf, "%02x\n", 0xff & value);

	return len;
}

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] define muic_device */
static struct muic_device muic_dev = {
	.name = "tsu5611",
	.read_int_state	= muic_int_stat_read,
};

static struct muic_client_ops tsu5611_ops = {
	.notifier_priority = MUIC_CLIENT_NOTI_MUIC,
	/* .on_none = Should not implement */		
	.on_ap_uart = muic_set_ap_uart_mode,
	.on_ap_usb 	= muic_set_ap_usb_mode,
	.on_cp_uart = muic_set_cp_uart_mode,
	.on_cp_usb 	= muic_set_cp_usb_mode,
#if 0
	.on_mhl = muic_set_mhl_mode,
#endif
};

static int __devinit muic_tsu5611_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct ts5usb_device *dev = NULL;
	struct muic_platform_data *pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	/* Allocate MUIC dev */
	dev = kzalloc(sizeof(struct ts5usb_device), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	dev->client = client;
	dev->gpio_int = pdata->gpio_int;
	dev->gpio_mhl = pdata->gpio_mhl;
	dev->gpio_ifx_vbus = pdata->gpio_ifx_vbus;
	dev->irq = client->irq;

	/*
	 * Initializes gpio_wk8 (MUIC_INT_N).
	 * Checks if other driver already occupied it.
	 */
	ret = gpio_request(dev->gpio_int, "MUIC IRQ GPIO");
	if (ret < 0) {
		dev_err(&client->dev, "%s: GPIO %d is already used\n", 
									__func__, dev->gpio_int);
		ret = -ENOSYS;
		goto err_gpio_request;
	}

	/* Initializes GPIO direction before use or IRQ setting */
	gpio_direction_input(dev->gpio_int);

	/* Registers MUIC work queue function */
	INIT_DELAYED_WORK(&dev->muic_delayed_wq, muic_tsu5611_wq_func);

	/*
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
	ret = request_irq(dev->irq, tsu5611_interrupt_handler,
			  IRQF_TRIGGER_FALLING, "muic_irq", dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s: request_irq failed!\n", __func__);
		goto err_request_irq;
	}

	/* Makes the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	ret = enable_irq_wake(dev->irq);
	if (ret < 0) {
		dev_err(&client->dev, "%s: GPIO %d wake up setting failed!\n", 
									__func__, dev->gpio_int);
		disable_irq_wake(dev->irq);
		goto err_irq_wake;
	}

	/* Initializes MUIC wake lock */
	wake_lock_init(&dev->muic_wake_lock, WAKE_LOCK_SUSPEND, "muic_wake_lock");

	/* Set dev data for i2c */
	i2c_set_clientdata(client, dev);

	/* Register MUIC device : muic_class */
	tsu5611 = client;
	ret = muic_device_register(&muic_dev, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "%s: muic_device_register failed\n", __func__);
		goto err_muic_device_register;
	}

	/* Register MUIC client device : muic_client_class */
	ret = muic_client_dev_register(dev->client->name, dev, &tsu5611_ops);
	if (ret < 0) {
		dev_err(&client->dev, "%s: muic_client_dev_register failed\n", __func__);
		goto err_muic_client_device_register;
	}

	/* Initializes MUIC - Finally MUIC INT becomes enabled */
	if (BOOT_RECOVERY == muic_retain_mode) { /* Recovery mode */
		muic_init_tsu5611(client, BOOTUP);
		muic_set_mode(MUIC_CP_UART);
		dev_info(&client->dev, "muic: %s: BOOT_RECOVERY!\n", __func__);
	}
	else {
		muic_init_tsu5611(client, BOOTUP);
		muic_tsu5611_detect_accessory(client);
	}

	return 0;

err_muic_client_device_register:
	/* Unregister MUIC device */
	muic_device_unregister(&muic_dev);
err_muic_device_register:
	/* Set tsu5611 NULL */
	tsu5611 = NULL;
	/* Unset client data for i2c */
	i2c_set_clientdata(client, NULL);
	/* Destroy MUIC wake lock */
	wake_lock_destroy(&dev->muic_wake_lock);
err_irq_wake:	
	/* Free MUIC GPIO irq */
	free_irq(dev->irq, dev);
err_request_irq:
	/* Cancel MUIC work queue function */
	cancel_delayed_work_sync(&dev->muic_delayed_wq);
	/* Free MUIC GPIO */
	gpio_free(dev->gpio_int);
err_gpio_request:
	/* Free MUIC dev */
	kfree(dev);

	return ret;
}

static int __devexit muic_tsu5611_remove(struct i2c_client *client)
{
	struct ts5usb_device *dev = i2c_get_clientdata(client);

	/* Unregister MUIC device */
	muic_device_unregister(&muic_dev);
	/* Set tsu5611 NULL */
	tsu5611 = NULL;
	/* Unset data for i2c */
	i2c_set_clientdata(client, NULL);
	/* Destroy MUIC wake lock */
	wake_lock_destroy(&dev->muic_wake_lock);
	/* Cancel MUIC work queue function */
	cancel_delayed_work_sync(&dev->muic_delayed_wq);	
	/* Free MUIC GPIO irq */
	free_irq(dev->irq, dev);
	/* Free MUIC GPIO */
	gpio_free(dev->gpio_int);
	/* Free MUIC dev */
	kfree(dev);

	return 0;
}

static const struct i2c_device_id tsu5611_ids[] = {
	{"tsu5611", 0},
	{/* end of list */},
};
MODULE_DEVICE_TABLE(i2c, tsu5611_ids);



static struct i2c_driver tsu5611_driver = {
	.probe	  = muic_tsu5611_probe,
	.remove	  = __devexit_p(muic_tsu5611_remove),
	.id_table = tsu5611_ids,
	.driver	  = {
		.name	= "tsu5611",
		.owner	= THIS_MODULE,
	},
};

static int __init tsu5611_init(void)
{
	return i2c_add_driver(&tsu5611_driver);
}

static void __exit tsu5611_exit(void)
{
	i2c_del_driver(&tsu5611_driver);
}

module_init(tsu5611_init);
module_exit(tsu5611_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("tsu5611 MUIC Driver");
MODULE_LICENSE("GPL");
