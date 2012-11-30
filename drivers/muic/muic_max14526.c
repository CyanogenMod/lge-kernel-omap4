#if 1 //mo2seongjae.jang 20120529 modified
 /*
  * TI MUIC MAX14526 driver
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
#include <linux/kernel.h>	/* printk() */
#include <linux/init.h>		/* __init, __exit */
#include <linux/uaccess.h>	/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>		/* set_irq_type() */
#include <linux/types.h>	/* kernel data types */
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>	/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/muic/muic.h>

//#include <linux/cosmo/cosmo_muic.h>
#include <linux/cosmo/charger_rt9524.h>


#include <asm/system.h>
#include <asm/gpio.h>

#include <lge/board.h>
#define DEBUGE_MAX14526
#define DEBUGE

static int muic_retain_mode;
static int muic_int_stat_mode;
int su760_factory_cable_detect;
extern atomic_t muic_charger_detected;

static struct i2c_client *max14526;
//static struct work_struct max14526_wq;
//static int Other_usb_count =0;
static int Int_Status = 0;

static TYPE_USIF_MODE usif_mode = USIF_AP;
static TYPE_DP3T_MODE dp3t_mode = DP3T_NC;

typedef enum int_{
	First_Int = 0,
	Second_Int,
}INT;

/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com]
 * Add private device handle
 */
struct ts5usb_device {
	struct i2c_client *client;
	int gpio_int;
#if defined(CONFIG_MHL)
	int gpio_mhl;
	int gpio_ifx_vbus;
#endif
	int irq;
	struct work_struct muic_wq;
	struct wake_lock muic_wake_lock;
};

/*
 * Initialize MUIC, i.e., the CONTROL_1,2 and SW_CONTROL registers.
 * 1) Prepare to sense INT_STATUS1 and STATUS bits.
 * 2) Open MUIC paths. -> To keep the path from uboot setting, remove this stage.
 */
void muic_mdelay(u32 microsec)
{
	do {
		udelay(1000);
	} while (microsec--);
}
void muic_init_max14526(struct i2c_client *client, TYPE_RESET reset)
{
	dev_info(&client->dev, "muic: %s\n", __func__);
	/*
  	 * Iniialize MAX14526 for Detection of Accessories
  	 * Enable 200K pull-up and ADC (0x01=0x12)
  	 * Enable Interrupt and set AUD Click/Pop resistor (0x02=0x50)
  	 */
	if (reset == RESET) {
		/* Clears default switch position (0x03=0x24) */
		muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
		muic_i2c_write_byte(client,CONTROL_1, ID_200 | ADC_EN);
//		mdelay(250);
		msleep(250);//mo2hongkeon.kim 2012-0816 for BT issue with TA charger
	}
	muic_i2c_write_byte(client,CONTROL_2, INT_EN | CP_AUD | CHG_TYPE);
}
EXPORT_SYMBOL(muic_init_max14526);

void usif_switch_ctrl(TYPE_USIF_MODE mode)
{
	if (mode == USIF_AP) {
		gpio_set_value(USIF_IN_1_GPIO, 0);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to AP\n");
	} else if (mode == USIF_DP3T) {
		gpio_set_value(USIF_IN_1_GPIO, 1);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to DP3T (then, MUIC)\n");
	} else {
		/* Just keep the current path */
	}

	usif_mode = mode;
	//printk(KERN_WARNING "[MUIC] usif_switch_ctrl(): usif_mode = %d\n", usif_mode);
}
EXPORT_SYMBOL(usif_switch_ctrl);

void dp3t_switch_ctrl(TYPE_DP3T_MODE mode)
{
	if (mode == DP3T_AP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_USB) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 1);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP USB is connected to MUIC UART\n");
	} else if (mode == DP3T_NC) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}

	dp3t_mode = mode;
	//printk(KERN_WARNING "[MUIC] dp3t_switch_ctrl(): dp3t_mode = %d\n", dp3t_mode);
}
EXPORT_SYMBOL(dp3t_switch_ctrl);

int muic_set_mhl_mode(struct muic_client_device *mcdev)// (struct i2c_client *client)
{
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev); 

	if(!client)
	{
		printk("muic %s, client : %d", __func__, client);
		return -1;
	}

	dev_info(&client->dev, "muic: %s entry.\n", __func__);
#if 0
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);
	/*
	 * AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART.
	 */
	dp3t_switch_ctrl(DP3T_NC);
#endif
	//muic_i2c_write_byte(SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);
	muic_i2c_write_byte(client, SW_CONTROL, DP_UART | DM_UART);

  	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	//muic_i2c_write_byte(CONTROL_1, ID_200 | SEMREN | CP_EN);
#if 0
	MHL_On(1);
#endif

//	muic_set_mode(MUIC_MHL);
}

int muic_set_cp_usb_mode(struct muic_client_device *mcdev)// (struct i2c_client *client)
{
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev);

	if(!client)
	{
		printk("muic %s, client : %ld", __func__, (long)client);
		return -1;
	}

	dev_info(&client->dev, "muic: %s entry.\n", __func__);
		
	muic_i2c_write_byte(client,SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Connect DP, DM to UART_TX, UART_RX */
	muic_i2c_write_byte(client,SW_CONTROL, DP_UART | DM_UART);

	//muic_path = MUIC_CP_USB;
	//charging_mode = CHARGING_USB;

	printk("[MUIC] muic_set_cp_usb_mode(): CP_USB\n");

	return 0;
}

int muic_set_ap_uart_mode(struct muic_client_device *mcdev)// (struct i2c_client *client)
{
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev);

	if(!client)
	{
		printk("muic %s, client : %d", __func__, client);
		return -1;
	}

	dev_info(&client->dev, "muic: %s\n", __func__);
	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	muic_i2c_write_byte(client,SW_CONTROL, DP_UART | DM_UART);

	//muic_path = MUIC_AP_UART;
	//charging_mode = CHARGING_UNKNOWN;

	printk("[MUIC] muic_set_ap_uart_mode(): AP_UART\n");

	return 0;
}

int muic_set_cp_uart_mode(struct muic_client_device *mcdev) //UART_MODE
{
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev);

	if(!client)
	{
		printk("muic %s, client : %ld", __func__, (long)client);
		return -1;
	}

	dev_info(&client->dev, "muic: %s\n", __func__);
	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);

	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	muic_i2c_write_byte(client,SW_CONTROL, DP_UART | DM_UART);

	//muic_path = MUIC_CP_UART;
	//charging_mode = CHARGING_UNKNOWN;

	printk("[MUIC] muic_set_cp_uart_mode(): CP_UART\n");

	return 0;
//	muic_set_mode(MUIC_CP_UART);  // CHECK
}

int muic_set_ap_usb_mode(struct muic_client_device *mcdev)// (struct i2c_client *client)
{
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev);

	if(!client)
	{
		printk("muic %s, client : %d", __func__, client);
		return -1;
	}

	dev_info(&client->dev, "muic: %s entry.\n", __func__);

	muic_i2c_write_byte(client,SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART.
	 */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Connect DP, DM to USB_DP, USB_DM */
	muic_i2c_write_byte(client,SW_CONTROL, DP_USB | DM_USB);

	//muic_path = MUIC_AP_USB;
	//charging_mode = CHARGING_USB;
	printk("[MUIC] muic_set_ap_usb_mode(): AP_USB\n");
	
	return 0;
}

void muic_set_charger_mode(struct i2c_client *client,unsigned char int_stat_value)
{
	s32 ret = 0;
  	unsigned char reg_value;

	dev_info(&client->dev, "muic: Charger ID = 0x%x\n", int_stat_value);

  	if (((int_stat_value & IDNO) == IDNO_0101) ||
	    ((int_stat_value & IDNO) == IDNO_1011)) {
		/*LG Proprietary TA Detected 180K ohm on ID */
		muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);
		muic_i2c_write_byte(client, CONTROL_2, INT_EN);
		muic_set_mode(MUIC_LG_TA);
	} else if ((int_stat_value & IDNO) == IDNO_0110) {
		/* 1A charger detected */
		muic_set_mode(MUIC_TA_1A);
	} else {
		dev_info(&client->dev, "muic: Charger ID11111\n");
		/* Enable interrpt and charger type detection (0x02=0x42) */
		muic_i2c_write_byte(client, CONTROL_2, INT_EN | CHG_TYPE);

		/* Read INT_STATUS1 */
		ret = muic_i2c_read_byte(client, INT_STAT, &reg_value);
		ret = muic_i2c_read_byte(client, STATUS, &reg_value);

		if (reg_value & DCPORT) {
			printk("Charger ID22222\n");
			muic_i2c_write_byte(client, CONTROL_1, ID_200 | SEMREN);
			muic_i2c_write_byte(client, CONTROL_2, INT_EN);
			muic_set_mode(MUIC_NA_TA);
		} else {
			printk("Charger ID33333\n");
			muic_set_mode(MUIC_AP_USB);
//			muic_set_usb_mode_detect(client);
		}
  	}
}

int muic_process_retention_mode(struct i2c_client *client)
{
	if(muic_retain_mode <0 ){
		printk(KERN_ERR "muic: %s, muic_retain_mode:%d is wrong state\n",
			__func__, muic_retain_mode);
		muic_retain_mode = 0;

		return -EINVAL;
	}

	if (BOOT_CP_USB == muic_retain_mode) {
		//dev_info(&client->dev, "muic: USB is CP retain\n");
		muic_set_mode(MUIC_CP_USB);
		//muic_set_develop_mode_detect(client);
	}
	else if (BOOT_AP_USB == muic_retain_mode) {
		//dev_info(&client->dev, "muic: USB is AP retain\n");
		muic_set_mode(MUIC_AP_USB);
		//muic_set_usb_mode_detect(client);
#if defined(CONFIG_MHL) //nthyunjin.yang 120511 temp add for build error.
	}else if (BOOT_MHL == muic_retain_mode){
		muic_set_mode(MUIC_MHL);
	}else if (BOOT_MHL == muic_retain_mode){
		muic_set_mode(MUIC_CP_UART);
#endif
	}

	return muic_retain_mode;
}

int max14526_process_reset_muic(struct i2c_client *client)
{
	if(Int_Status == First_Int) {
		dev_info(&client->dev, "muic: trying reset n redetect cause of MAX14526 chipset bug\n");
		/* Clears default switch position (0x03=0x24) */
		muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 

		mdelay(300); // delay is must 300ms ~ 600ms after reset

		dev_info(&client->dev, "muic:  device reset is done\n");
		Int_Status = Second_Int;
	}
	else {
		dev_info(&client->dev, "muic: << Second Interrupt\n");
		Int_Status = First_Int;
	}

	muic_i2c_write_byte(client,CONTROL_1, ID_200 | ADC_EN | CP_EN);
	muic_i2c_write_byte(client,CONTROL_2, INT_EN);

	return Int_Status;
}

#ifdef DEBUGE_MAX14526
int muic_set_device_none_detect(struct i2c_client *client,unsigned char int_stat_value)
{
//	s32 loop = 0;
	unsigned char charger_value = 0;

//	unsigned char stat_value = 0;
//	unsigned char Accessory_value = 0;
//	unsigned char orginal_value = 0;
	unsigned char id_register_value = 0;

//	int i;

	id_register_value = 0x0f & int_stat_value;

	dev_info(&client->dev, "muic: %s, muic_retain_mode:%d, Int_Status:%d, INT_STATUS:0x%2x\n",
		__func__,muic_retain_mode, Int_Status, int_stat_value);


	if(int_stat_value & VBUS) 
	{// VBUS  = High
		dev_info(&client->dev, "muic: %s: VBUS is High.... id_register_value:%d\n", __func__,id_register_value);

		if(muic_process_retention_mode(client))
			return 0;

		//if(Second_Int == max14526_process_reset_muic(client))
			//return 0;

		if(id_register_value == IDNO_0010 /*56k + VBUS*/)
		{
			muic_set_mode(MUIC_CP_USB);
		}
		else if(id_register_value == IDNO_1010 /*910k + VBUS*/)
		{
			muic_set_mode(MUIC_CP_USB);
		}
		else if(id_register_value == IDNO_1001 /*620k + VBUS*/)
		{
			muic_set_mode(MUIC_CP_USB);
		}
		else if(id_register_value == IDNO_0100)/*130k + VBUS*/
		{
			muic_set_mode(MUIC_CP_UART);
		}
#if defined(CONFIG_MHL)	
		else if(id_register_value==IDNO_0000)
		{
			muic_set_mode(MUIC_MHL);
		}
#endif
		else if(id_register_value == IDNO_0101)/*180k + VBUS LG proprietary TA Detected*/
		{
			//muic_set_mode(MUIC_LG_TA);
			muic_set_mode(MUIC_AP_USB);//lg usb calble의 int_stat_value =0x15 로 동작되어, 일반 usb cable로 수정 함.
		}
		else// if(charger_value & CHPORT)
		{
			dev_info(&client->dev, "muic: Charger detection....\n");
			
			muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_C1COMP);
			//muic_mdelay(2);//mdelay(250); // if charger type then waiting 250ms
			//muic_i2c_read_byte(client, INT_STAT, &stat_value);
			muic_i2c_read_byte(client,STATUS, &charger_value);
			dev_info(&client->dev, "muic: charger_value :0x%x\n",charger_value);
			//muic_set_charger_mode(client, charger_value);

			//else if( (stat_value & CHGDET)&&((id_register_value & IDNO) == IDNO_1011))//stat_value==0x9B 일반 충전기
			if(int_stat_value & CHGDET)
			{
				muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_set_mode(MUIC_LG_TA);
			}
			else if (charger_value & DCPORT)
			{
				/* Not used actually. Special TA for North America.*/
				muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_set_mode(MUIC_NA_TA);
			} 
			else if (charger_value & CHPORT)
			{
				//muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_set_mode(MUIC_AP_USB);
			}
			else if (charger_value & C1COMP) 
			{//MUIC_LG_TA , CHARGING_LG_TA
				muic_i2c_write_byte(client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_set_mode(MUIC_LG_TA);
				//muic_path = MUIC_LG_TA;
				//charging_mode = CHARGING_LG_TA;
			}
			else //stat_value==0x1B 일반 UBS케이블
			{
				//set_max14526_ap_usb_mode();
				muic_set_mode(MUIC_AP_USB);
				//muic_path = MUIC_AP_USB;
				//charging_mode = CHARGING_USB;
			}
		}
	}
	else
	{
		muic_set_mode(MUIC_NONE);
	}

	return 0;
}
#endif
extern void android_USB_disconnect(void);// for usb disconnect event
s32 muic_max14526_detect_accessory(struct i2c_client *client, s32 upon_irq)
{
	s32 ret = 0;
	s32 loop = 0;
	u8 int_stat_value=0;
	
	TYPE_MUIC_MODE muic_mode = muic_get_mode();
	struct ts5usb_device *dev = i2c_get_clientdata(client);
	/*
	 * Upon an MUIC IRQ (MUIC_INT_N falls),
	 * wait 70ms before reading INT_STATUS1 and STATUS.
	 * After the reads, MUIC_INT_N returns to high
	 * (but the INT_STATUS1 and STATUS contents will be held).
	 *
	 * Do this only if muic_max14526_detect_accessory() was called upon IRQ.
	 */
	if (upon_irq) {
		mdelay(70);
	}

	/* Read INT_STATUS1 */
	ret = muic_i2c_read_byte(client, INT_STAT, &int_stat_value);
	dev_info(&client->dev, "muic: %s: int_stat_value:0x%x :%d\n", __func__,	int_stat_value, muic_mode);

	if (ret < 0) {
		dev_info(&client->dev, "muic: INT_STATUS1 reading failed\n");
		muic_set_mode(MUIC_UNKNOWN);

		return ret;
	}

	switch (muic_mode) 
	{/* Branch according to the previous muic_mode */
		case MUIC_UNKNOWN :
		case MUIC_NONE :
			muic_set_device_none_detect(client, int_stat_value);
			break;

		case MUIC_AP_UART :
		case MUIC_CP_UART :
			dev_info(&client->dev, "muic: MUIC_AP_UART || MUIC_CP_UART\n");
			if(!(int_stat_value & VBUS)&&(IDNO_0010 == (int_stat_value & 0x0f)))
			{
				muic_set_mode(MUIC_CP_USB);
			} 
			else if (!(int_stat_value & VBUS))
			{
				muic_i2c_write_byte(client, SW_CONTROL, DP_UART | DM_UART);
				muic_set_mode(MUIC_NONE);
			}
			else
			{
				dev_info(&client->dev, "UART is not removed\n");
			}
			break;

		case MUIC_NA_TA :
		case MUIC_LG_TA :
		case MUIC_TA_1A :
		case MUIC_INVALID_CHG :
			dev_info(&client->dev, "muic: Detect step2\n");
			if (((int_stat_value & VBUS) == 0) ||((int_stat_value & CHGDET) == 0)) 
			{
				muic_set_mode(MUIC_NONE);
			}
			else
			{
				dev_info(&client->dev, "TA is not removed\n");
			}
			break;

		case MUIC_AP_USB :
			/* USB Host Removed */
			if((int_stat_value & VBUS) == 0)
			{
			android_USB_disconnect();// for usb disconnect event
			muic_set_mode(MUIC_NONE);
			}
			else
			{
				dev_info(&client->dev, "AP USB is not removed\n");
			}
			break;
			
		case MUIC_CP_USB :
			if((int_stat_value & VBUS) == 0)
			{
				muic_set_mode(MUIC_NONE);
			}
			else
			{
				dev_info(&client->dev, "CP USB is not removed\n");
			}
			break;
			
#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)
		case MUIC_MHL :
			dev_info(&client->dev, "muic: Detect step3  mhl \n");
			if ((int_stat_value & VBUS) == 0) {
				//MHL_On(0);
				muic_set_mode(MUIC_NONE);
			}
			break;
#endif
		default:
			dev_info(&client->dev, "muic: Failed to detect an accessory. Try again!");
			muic_set_mode(MUIC_UNKNOWN);
			ret = -1;
			break;
	}

	muic_mode = muic_get_mode();
	dev_info(&client->dev, "muic: muic_detection_mode=%d \n", muic_mode);

	if (muic_mode == MUIC_UNKNOWN || muic_mode == MUIC_NONE) {
#if defined(CONFIG_MHL) && defined(CONFIG_MHL_TX_MUIC_BUG_FIX)
		/* SJIT 2012-01-27 [dojip.kim@lge.com] P940 GB
		 * max14526 300ms delay side effect bug fixed test code
		 */
		if (Int_Status == Second_Int) {
			if (!gpio_get_value(dev->gpio_mhl)) {
				dev_info(&client->dev, "muic: wait for mhl switch completed\n");
				muic_init_max14526(client, DEFAULT);
				gpio_set_value(dev->gpio_ifx_vbus, 0);
			}
			else {
				dev_info(&client->dev, "muic: mhl switch not completed\n");
				while (!gpio_get_value(dev->gpio_mhl)) {
					udelay(500);
					muic_set_mode(MUIC_MHL);
//					MHL_On(0);
				}
				dev_info(&client->dev, "muic: mhl switch completed\n");
				muic_init_max14526(client, DEFAULT);
				gpio_set_value(dev->gpio_ifx_vbus, 0);
			}
		}
		else {
			muic_init_max14526(client, DEFAULT);
			gpio_set_value(dev->gpio_ifx_vbus, 0);
		}
#else
		muic_init_max14526(client, RESET);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		//charging_ic_deactive();
		dev_info(&client->dev, "muic: charging_ic_deactive()\n");
#endif
	}

// LGE_CHANGES_S [dukwung.kim@lge.com] 2012-03-19
#if defined(CONFIG_MAX8971_CHARGER)
	        if(muic_mode == MUIC_NA_TA || muic_mode == MUIC_LG_TA || muic_mode == MUIC_TA_1A)
		{
		        printk("[MUIC] max8971_start_charging TA\n");
		        max8971_start_charging(900);
									         }
		else if(muic_mode == MUIC_AP_USB)
		{
		        printk("[MUIC]max8971_start_charging USB\n");
		        max8971_start_charging(500);
										        }
		/* else if( muic_mode == MUIC_CP_USB || muic_mode ==  MUIC_CP_UART)
		{
		        printk("[MUIC] max8971_start_charging Factory\n");
		        max8971_start_charging(1550);
		}
		  */
		else if( muic_mode == MUIC_CP_USB )
		{
		 //printk("[MUIC] max8971_stop due to MUIC_CP_USB mode \n");
		        printk("[MUIC] max8971_start_charging Factory MUIC_CP_USB \n");
		        // max8971_start_charging(1550);
		        // max8971_stop_charging();
		        max8971_start_Factory_charging();
		}
		else if(  muic_mode ==  MUIC_CP_UART)
		{
		         printk("[MUIC] max8971_start_charging Factory MUIC_CP_UART \n");
	                //printk("[MUIC] max8971_stop due to MUIC_CP_UART mode \n");
	                // max8971_stop_charging();
	                // max8971_start_charging(1550);
	                 max8971_start_Factory_charging();
	        }
	        else if( muic_mode == MUIC_MHL)
	        {
	                printk("[MUIC] max8971_start_charging MHL\n");
	                max8971_start_charging(400);
	        }
	        else if( muic_mode == MUIC_NONE)
	        {
	                printk("[MUIC] max8971_stop_charging\n");
	                max8971_stop_charging();
		}
		else
		       printk("[MUIC] can not open muic mode\n");
		// LGE_CHANGES_E [dukwung.kim@lge.com] 2012-03-19
#endif
	return ret;
}

/* LGE_CHANGE_S [kenneth.kang@lge.com] 2011-07-26, CP retain mode */

/*mo2seongjae.jang@lge.com 20120813*/
int get_muic_retain_mode(void)
{
	return muic_retain_mode;
}
int get_muic_muic_int_stat_mode(void)
{
	return muic_int_stat_mode;
}
/*mo2seongjae.jang@lge.com 20120813*/

static int __init muic_state(char *str)
{
	s32 muic_value = simple_strtol(str, NULL, 0);
	muic_retain_mode = muic_value;
	printk(KERN_INFO "muic: Retain : %d\n", muic_value);

	return 1;
}
__setup("muic_state=", muic_state);/* LGE_CHANGE_E [kenneth.kang@lge.com] 2011-07-26, CP retain mode */

/*mo2seongjae.jang@lge.com 20120813
bootable\bootloader\lk\dev\muic\muic.c muic_init 함수에서 cmdline으로 처리된 함수.
*/
static int __init muic_int_stat(char *str)
{
	unsigned char id_register_value = 0;
	s32 muic_value = simple_strtol(str, NULL, 0);
	muic_int_stat_mode = muic_value;
	id_register_value = 0x0f & muic_int_stat_mode;
//IDNO_0010) // 56k + VBUS
//IDNO_1001)) // 620k + VBUS
//IDNO_1010) // 910k + VBUS
//IDNO_0100) //uart + VBUS
	if(muic_int_stat_mode & VBUS)
	{
		if((id_register_value == IDNO_0010) || \
		    (id_register_value == IDNO_1001) || \ 
		    (id_register_value == IDNO_1010) || \
		    (id_register_value == IDNO_0100))
		su760_factory_cable_detect = 1;
	}
	
	printk(KERN_INFO "muic_int_stat_mode : %d\n", muic_int_stat_mode);

	return 1;
}
__setup("muic_int_stat=", muic_int_stat);
/*mo2seongjae.jang@lge.com 20120813*/

static void max14526_wq_func(struct work_struct *work)
{
	s32 ret = 0;
	struct ts5usb_device *dev = container_of(work,
			struct ts5usb_device, muic_wq);
	struct i2c_client *client = dev->client;

	wake_lock(&dev->muic_wake_lock);

	dev_info(&client->dev, "muic: %s()\n", __func__);

	ret = muic_max14526_detect_accessory(client, UPON_IRQ);
//	dev_info(&client->dev, "muic: muic_detect_accessory(UPON_IRQ)detedted!!!!\n");
	wake_unlock(&dev->muic_wake_lock);
}

static irqreturn_t max14526_interrupt_handler(s32 irq, void *data)
{
	struct ts5usb_device *dev = data;
	/* Make the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	schedule_work(&dev->muic_wq);
	return IRQ_HANDLED;
}

static int muic_int_stat_read(struct muic_client_device *mcdev,
	char *buf)
{	
	struct i2c_client *client = max14526; //to_i2c_client(%mcdev->dev);// i2c_verify_client(&mcdev->dev);

	u32 ret;
	u32 value;
	unsigned int len;

	ret = muic_i2c_read_byte(client, INT_STAT, &value);
//	value = i2c_smbus_read_byte_data(muic_client, INT_STAT);
	dev_info(&client->dev, "[MUIC] INT_STAT = 0x%02x\n", (0xff & value));
	
	len = sprintf(buf, "%02x\n", 0xff & value);

	return len;
}



/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] define muic_device */
static struct muic_device muic_dev = {
	.name = "max14526",
	.read_int_state	= muic_int_stat_read,
};

static struct muic_client_ops max14526_ops = {
	.on_ap_uart = muic_set_ap_uart_mode,
	.on_ap_usb 	= muic_set_ap_usb_mode,
	.on_cp_uart = muic_set_cp_uart_mode,
	.on_cp_usb 	= muic_set_cp_usb_mode,
	//.on_mhl		= muic_set_mhl_mode,
};

static int __devinit max14526_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	TYPE_MUIC_MODE muic_mode;
	struct ts5usb_device *dev = NULL;
	struct muic_platform_data *pdata = client->dev.platform_data;

	dev_info(&client->dev, "muic: %s()\n", __func__);

	if (!pdata) {
		dev_err(&client->dev, "muic: %s: no platform data\n", __func__);
		return -EINVAL;
	}

	dev = kzalloc(sizeof(struct ts5usb_device), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "muic: %s: no memory\n", __func__);
		return -ENOMEM;
	}

	dev->client = client;
	dev->gpio_int = pdata->gpio_int;
#if defined(CONFIG_MHL)
	dev->gpio_mhl = pdata->gpio_mhl;
	dev->gpio_ifx_vbus = pdata->gpio_ifx_vbus;
#endif
	dev->irq = client->irq;

	max14526 = client;

	/* Initializes gpio_165 (USIF1_SW). */
	ret = gpio_request(USIF_IN_1_GPIO, "USIF switch control GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 165 USIF1_SW is already occupied by other driver!\n");
		/*
		 * We know board_cosmo.c:ifx_n721_configure_gpio() performs a gpio_request on this pin first.
		 * Because the prior gpio_request is also for the analog switch control, this is not a confliction.
		 */
		return -ENOSYS;
	}

	ret = gpio_direction_output(USIF_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_16 USIF_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	/*  Initializes gpio_11 (OMAP_UART_SW) and gpio_12 (IFX_UART_SW) */
	ret = gpio_request(DP3T_IN_1_GPIO, "DP3T switch control 1 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 11 DP3T_IN_1_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_11 DP3T_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	ret = gpio_request(DP3T_IN_2_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_12 DP3T_IN_2_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_2_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_12 DP3T_IN_2_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	ret = gpio_request(IFX_USB_VBUS_EN_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(IFX_USB_VBUS_EN_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	/*
	 * Initializes gpio_wk8 (MUIC_INT_N).
	 * Checks if other driver already occupied it.
	 */
	ret = gpio_request(dev->gpio_int, "MUIC IRQ GPIO");
	if (ret < 0) {
		dev_err(&client->dev, "muic: GPIO %d is already used\n",	dev->gpio_int);
		ret = -ENOSYS;
		goto err_gpio_request;
	}
	/* Initializes GPIO direction before use or IRQ setting */
	gpio_direction_input(dev->gpio_int);

	/* Registers MUIC work queue function */
	INIT_WORK(&dev->muic_wq, max14526_wq_func);

	/*
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
	ret = request_irq(dev->irq, max14526_interrupt_handler,IRQF_TRIGGER_FALLING, "muic_irq", dev);
	if (ret < 0) 
	{
		dev_err(&client->dev, "muic: %s: request_irq failed!\n",__func__);
		goto err_request_irq;
	}

	//disable_irq_wake(gpio_to_irq(MUIC_INT));

	/* Prepares a human accessible method to control MUIC */
	//create_cosmo_muic_proc_file();

	/* Selects one of the possible muic chips */
	//muic_detect_device();

	wake_lock_init(&dev->muic_wake_lock, WAKE_LOCK_SUSPEND,"muic_wake_lock");

	ret = muic_device_register(&muic_dev, NULL);
	if (ret < 0) {
		dev_err(&client->dev, "muic: %s: muic_device_register failed\n",__func__);
		goto err_muic_device_register;
	}

	i2c_set_clientdata(client, dev);

	// hunsoo.lee
	printk("%s, registering ops\n", __func__);
	muic_client_dev_register(dev->client->name, dev, &max14526_ops);

	/* Initializes MUIC - Finally MUIC INT becomes enabled */
	if (BOOT_RECOVERY == muic_retain_mode) 
	{ /* Recovery mode */
		muic_mode = MUIC_CP_UART;
		muic_init_max14526(client, BOOTUP);
		dev_info(&client->dev, "muic: %s: first boot\n", __func__);
	}
	else 
	{
		muic_init_max14526(client, RESET);
		muic_max14526_detect_accessory(client, NOT_UPON_IRQ);
		muic_mode = muic_get_mode();
	}
	muic_set_mode(muic_mode);

	/* Makes the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	ret = enable_irq_wake(dev->irq);
	if (ret < 0) {
		dev_err(&client->dev, "muic: GPIO %d wake up source setting failed!\n", dev->gpio_int);
		disable_irq_wake(dev->irq);
		goto err_irq_wake;
	}

	dev_info(&client->dev, "muic: muic_probe()\n");

	return ret;

err_irq_wake:
	muic_device_unregister(&muic_dev);
err_muic_device_register:
	wake_lock_destroy(&dev->muic_wake_lock);
	free_irq(dev->irq, dev);
err_request_irq:
	gpio_free(dev->gpio_int);
err_gpio_request:
	kfree(dev);

	return ret;
}

static int __devexit max14526_remove(struct i2c_client *client)
{
	struct ts5usb_device *dev = i2c_get_clientdata(client);

	/* LGE_SJIT 2012-01-27 [dojip.kim@lge.com] unregister muic device */
	muic_device_unregister(&muic_dev);
	cancel_work_sync(&dev->muic_wq);
	wake_lock_destroy(&dev->muic_wake_lock);
	free_irq(dev->irq, dev);
	gpio_free(dev->gpio_int);
	i2c_set_clientdata(client, NULL);
	kfree(dev);
//	remove_cosmo_muic_proc_file();
	return 0;
}

static s32 max14526_suspend(struct i2c_client *client, pm_message_t state)
{
	client->dev.power.power_state = state;

	return 0;
}

static s32 max14526_resume(struct i2c_client *client)
{
	client->dev.power.power_state = PMSG_ON;

	return 0;
}


static const struct i2c_device_id max14526_ids[] = {
	{"max14526", 0},
	{/* end of list */},
};
MODULE_DEVICE_TABLE(i2c, max14526_ids);



static struct i2c_driver max14526_driver = {
	.probe	  = max14526_probe,
	.remove	  = __devexit_p(max14526_remove),
	.suspend  = max14526_suspend,
	.resume   = max14526_resume,
	.id_table = max14526_ids,
	.driver	  = {
		.name	= "max14526",
		.owner	= THIS_MODULE,
	},
};

static int __init max14526_init(void)
{
	return i2c_add_driver(&max14526_driver);
}

static void __exit max14526_exit(void)
{
	i2c_del_driver(&max14526_driver);
}

module_init(max14526_init);
module_exit(max14526_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("max14526 MUIC Driver");
MODULE_LICENSE("GPL");

#else // ORG code mo2seongjae.jang 20120529 modified
/*
 * Cosmo MUIC MAX14526 driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
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
#include <asm/system.h>

/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>

#include <linux/muic/muic.h>
//#include <linux/cosmo/cosmo_muic.h>
#include <linux/cosmo/charger_rt9524.h>


void muic_init_max14526(TYPE_RESET reset)
{
	printk(KERN_WARNING "[MUIC] max14526_init()\n");

	//[jongho3.lee@lge.com]  muic detecting...
	atomic_set(&muic_charger_detected, 0); 
	
	if (reset == RESET) {
		/* Clears default switch position (0x03=0x24) */
		muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ); 
	}

	/*
  	 * Iniialize MAX14526 for Detection of Accessories
  	 * Enable 200K pull-up and ADC (0x01=0x12)
  	 * Enable Interrupt and set AUD Click/Pop resistor (0x02=0x50)
  	 */
	muic_i2c_write_byte(muic_client,CONTROL_1, ID_200 | ADC_EN | CP_EN);
	muic_i2c_write_byte(muic_client,CONTROL_2, INT_EN);
}
EXPORT_SYMBOL(muic_init_max14526);


void set_max14526_ap_uart_mode(void) //UART_MODE
{
	printk(KERN_WARNING "[MUIC] set_max14526_ap_uart_mode\n" );

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(muic_client,CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enables UART Path (0x03=0x09) */
	muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}


void set_max14526_ap_usb_mode(void)
{
	printk(KERN_WARNING "[MUIC] set_max14526_ap_usb_mode\n" );
	//KIMCS TEST	
	usif_switch_ctrl(USIF_AP);
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Enables USB Path (0x03=0x00) */
	muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_DP2 | COMN1_TO_DN1);

	/* Enables 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(muic_client,CONTROL_1, ID_200 | ADC_EN | CP_EN);
}


void set_max14526_cp_uart_mode(void) 
{
	printk(KERN_WARNING "[MUIC] set_max14526_cp_uart_mode\n" );

	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);
	
	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(muic_client,CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}


void set_max14526_cp_usb_mode(void) 
{
	printk(KERN_WARNING "[MUIC] set_max14526_cp_usb_mode\n" );

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);
	
	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Enable 200K, Charger Pump, and ADC (0x01=0x13) */
	muic_i2c_write_byte(muic_client,CONTROL_1, ID_200 | ADC_EN | CP_EN);

	/* Enable UART Path (0x03=0x09) */
	muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_U2 | COMN1_TO_U1);
}

void set_max14526_charger_mode(unsigned char int_stat_value)
{
	unsigned char reg_value;

	printk(KERN_WARNING "[MUIC] set_max14526_charger_mode, int_stat_value = 0x%02x\n", int_stat_value);

	if (((int_stat_value & IDNO) == IDNO_0101) || ((int_stat_value & IDNO) == IDNO_1011)) {
		/* LG Proprietary TA Detected 180K ohm on ID */
		muic_path = MUIC_LG_TA;
	} else if ((int_stat_value & IDNO) == IDNO_0110) {
		/* 1A charger detected */
		muic_path = MUIC_TA_1A;
	} else {
		/* Enable interrpt and charger type detection (0x02=0x42) */
		muic_i2c_write_byte(muic_client,CONTROL_2, INT_EN | CHG_TYPE);

		///////////////////////////////////////////////
		// This line should be modified by a customer.
		// 
		// Wait for Interrupt
		//
		////////////////////////////////////////////////

		/* Read INT_STAT */
		muic_i2c_read_byte(muic_client,INT_STAT, &reg_value);
		muic_i2c_read_byte(muic_client,STATUS, &reg_value);

		if (reg_value & DCPORT) {
			/* Dedicated charger(TA) detected */
			muic_path = MUIC_NA_TA;
		} else if (reg_value & CHPORT) {
			set_max14526_ap_usb_mode();
		}
	}
}

void set_max14526_muic_path(unsigned char int_stat_value)
{
	unsigned char reg_value;
	
	printk(KERN_WARNING "[MUIC] set_max14526_muic_path, int_stat_value = 0x%02x \n", int_stat_value);

	if (int_stat_value & VBUS) {
		if ((int_stat_value & IDNO) == IDNO_0010 || 
			(int_stat_value & IDNO) == IDNO_1001 ||
			(int_stat_value & IDNO) == IDNO_1010) {
			set_max14526_cp_usb_mode();
			muic_path = MUIC_CP_USB;
			charging_mode = CHARGING_FACTORY;
		} else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_max14526_cp_uart_mode();
			muic_path = MUIC_CP_UART;
			charging_mode = CHARGING_FACTORY;
		} else if (int_stat_value & CHGDET) {
			muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
			muic_path = MUIC_LG_TA;
			charging_mode = CHARGING_LG_TA;
		} else {
			muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_C1COMP);

			muic_mdelay(2);

			muic_i2c_read_byte(muic_client,STATUS, &reg_value);

			if (reg_value & C1COMP) {
				muic_i2c_write_byte(muic_client,SW_CONTROL, COMP2_TO_HZ | COMN1_TO_HZ);
				muic_path = MUIC_LG_TA;
				charging_mode = CHARGING_LG_TA;
			} else {
				set_max14526_ap_usb_mode();
				muic_path = MUIC_AP_USB;
				charging_mode = CHARGING_USB;
			}
		}
	} else {
		if ((int_stat_value & IDNO) == IDNO_0010) {
			set_max14526_ap_uart_mode();
			muic_path = MUIC_AP_UART;
			charging_mode = CHARGING_NONE;
		} else if ((int_stat_value & IDNO) == IDNO_0100) {
			set_max14526_cp_uart_mode();
			muic_path = MUIC_CP_UART;
			charging_mode = CHARGING_NONE;
		} else {
			muic_path = MUIC_UNKNOWN;
			charging_mode = CHARGING_NONE;
		}
	}
}


s32 muic_max14526_detect_accessory(s32 upon_irq)
{
	s32 ret = 0;

	u8 int_stat_value;

	/* For detecting time */
	muic_mdelay(250);
		
	/* Reads INT_STAT */
	ret = muic_i2c_read_byte(muic_client,INT_STAT, &int_stat_value);
	printk(KERN_WARNING "[MUIC] muic_max14526_detect_accessory, int_stat_value = 0x%02x \n", int_stat_value);
	
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] INT_STAT reading failed\n");
		muic_path = MUIC_UNKNOWN;
		charging_mode = CHARGING_UNKNOWN;
		//LGE_CHANGE_S [jongho3.lee@lge.com]  let charger know muic detected charger type..
		//set_muic_charger_detected();
		return ret;
	}
	
    /* Branches according to the previous muic_path */
	switch (muic_path) {

	/* MUIC_UNKNOWN is reached in two cases both do not have nothing to do with IRQ.
	 * First, at the initialization time where the muic_path is not available yet.
	 * Second, whenever the current muic_path detection is failed.
	 */
	case MUIC_UNKNOWN :

	/* If the previous muic_path was MUIC_NONE,
	 * the only possible condition for a MUIC IRQ is plugging in an accessory.
	 */
	case MUIC_NONE :
		set_max14526_muic_path(int_stat_value);           
		break;

	/* If the previous muic_path was MUIC_NA_TA, MUIC_LG_TA, MUIC_TA_1A, MUIC_INVALID_CHG,
	 * MUIC_AP_UART, MUIC_CP_UART, MUIC_AP_USB, MUIC_OTG, or MUIC_CP_USB,
	 * the only possible condition for a MUIC IRQ is plugging out the accessory.
	 * 
	 * In this case, initialize MUIC and wait an IRQ.
	 * We don't need to wait 250msec because this is not an erronous case
	 * (we need to reset the facility to set STATUS for an erronous case and
	 * have to wait 250msec) and, if this is not an erronous case, the facility
	 * was already initialized at the system booting.
	 */
	case MUIC_NA_TA:
	case MUIC_TA_1A:
	case MUIC_INVALID_CHG :
	case MUIC_LG_TA :
	case MUIC_AP_UART :
	case MUIC_CP_UART :
	case MUIC_AP_USB :
	case MUIC_CP_USB :
		if ((int_stat_value & VBUS) != 0) {
			set_max14526_muic_path(int_stat_value);
		} else if ((int_stat_value & IDNO) == IDNO_1011) {
			charging_mode = CHARGING_NONE;
			muic_path = MUIC_NONE;
		} else
			set_max14526_muic_path(int_stat_value);
		break;
		
	default :
		printk(KERN_WARNING "[MUIC] Failed to detect an accessory. Try again!");
		muic_path = MUIC_UNKNOWN;
		charging_mode = CHARGING_UNKNOWN;
		ret = -1;
	}	
	
	printk(KERN_WARNING "[MUIC] muic_max14526_detect_accessory, muic_path = %s, charing = %s\n", 
		   muic_path_str[muic_path], charging_mode_str[charging_mode]);

	if (muic_path == MUIC_UNKNOWN || muic_path == MUIC_NONE) {
		muic_init_max14526(RESET);
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
        printk(KERN_INFO "[MUIC] charging_ic_deactive()\n");
    }
	
	//LGE_CHANGE_S [jongho3.lee@lge.com]  let charger know muic detected charger type..
	//set_muic_charger_detected();
	return ret;
}
EXPORT_SYMBOL(muic_max14526_detect_accessory);
#endif
