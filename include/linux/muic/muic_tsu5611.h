/*
 *  tsu5611 muic driver
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 * Author: hunsoo.lee@lge.com
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

#ifndef _MUIC_TSU5611_H_
#define _MUIC_TSU5611_H_

#define TD_INT_STAT	70000	/* INT_STAT bits settle down time since MUIC INT falls on TI chip (us) */
#define TD_STATUS	250000	/* STATUS bits settle down time since MUIC INT falls on TI chip (us) */
#define TD_DP_DM	1000	/* DP, DM path settle down time since SW_CONTROL writing on TI chip (us) */

/* I2C addresses of MUIC internal registers */
#define	DEVICE_ID	(u8)0x00
#define	CONTROL_1	(u8)0x01
#define	CONTROL_2	(u8)0x02
#define	SW_CONTROL	(u8)0x03
#define	INT_STATUS1	(u8)0x04
#define	INT_STATUS2	(u8)0x05
#define	STATUS		(u8)0x06


/* Masks for the each bit of CONTROL_1 register */
#define	SEMREN2		(u8)0x80	/* Enables SEND/END2, Microphone Removal2 comparators and the UID LDO */
#define	ID_2P2		(u8)0x40	/* Connects an internal 2.21k external resistor between the internal UID LDO output to USB ID */
#define	ID_620		(u8)0x20	/* Connects an internal 620k internal resistor between the internal UID LDO output to USB ID */
#define	ID_200		(u8)0x10	/* Connects an internal 200k internal resistor between the internal UID LDO output to USB ID */
#define	VLDO		(u8)0x08	/* Sets the voltage on the UID LDO. */
#define	SEMREN		(u8)0x04	/* Enables SEND/END, Microphone Removal comparators and the UID LDO */
#define	ADC_EN		(u8)0x02	/* (No need control) Enables the internal ADC and UID LDO */
#define	CP_EN		(u8)0x01	/* (No need control) Enables the charger pump required for analog switch operation */


/* Masks for the each bit of CONTROL_2 register */
#define	INTPOL		(u8)0x80	/* Sets interrupt polarity */
#define	INT1_EN		(u8)0x40	/* Enable Interrupt generation on INT_STATUS1 */
#define	MIC_LP		(u8)0x20	/* Enable UID line pulsing for low power detection of the SEND/END key and microphone removal */
#define	CP_AUD		(u8)0x10	/* Sets position of the click/pop resistors on AUD */
#define	MB_200		(u8)0x08	/* Connect LDO to MIC through 200k resistor */
#define	INT2_EN		(u8)0x04	/* Enable Interrupt generation on INT_STATUS2 */
#define	CHG_TYP		(u8)0x02	/* Enable Charger Type Detection */
#define	USB_DET_DIS	(u8)0x01	/* Disable USB Charger Detection */


/* Masks for the each bit of SW_CONTROL register */
#define	MIC_ON		(u8)0x40
#define DP			(u8)0x38
#define DM			(u8)0x07


/* DP, DM settings */
#define DP_USB			(u8)0x00	/* DP connected to USB_DP */
#define	DP_UART			(u8)0x08	/* DP connected to UART_TX */
#define	DP_AUDIO		(u8)0x10	/* DP connected to AUDIO_R */
#define	DP_V_AUDIO		(u8)0x18	/* Future USe (DP connected to AUDIO_R for Video */
#define	DP_OPEN			(u8)0x38	/* DP switching path open */
#define DM_USB			(u8)0x00	/* DM connected to USB_DM */
#define	DM_UART			(u8)0x01	/* DM connected to UART_RX */
#define	DM_AUDIO		(u8)0x02	/* DM connected to AUDIO_L */
#define	DM_V_AUDIO		(u8)0x03	/* Future USe (DM connected to AUDIO_L for Video */
#define	DM_OPEN			(u8)0x07	/* DM switching path open */


/* Combined masks of SW_CONTROLl register */
#define USB			DP_USB   	| DM_USB 		/* 0x00 */
#define UART 		DP_UART  	| DM_UART 		/* 0x09 */
#define AUDIO		DP_AUDIO 	| DM_AUDIO		/* 0x12 */
#define V_AUDIO		DP_V_AUDIO 	| DM_V_AUDIO	/* 0x1b */
#define OPEN		DP_OPEN  	| DM_OPEN 		/* 0x3f */


/* Masks for the each bit of INT_STATUS1 register */
#define	CHGDET		(u8)0x80	/* USB Charger detection comparator */
#define	MR_COMP		(u8)0x40	/* MIC Removal comparator for SEMREN=1 */
#define	SENDEND		(u8)0x20	/* SEND/END comparator for SEMREN=1 */
#define	VBUS		(u8)0x10	/* VB comparator for ADC_EN=1 or SEMREN=1*/
#define	IDNO		(u8)0x0f	/* ADC Output for ADC_EN=1 */


/* Masks for the each bit of INT_STATUS2 register */
#define	MR_COMP2	(u8)0x02
#define SENDEND2	(u8)0x01


/* Masks for the each bit of STATUS register */
#define	DCPORT		(u8)0x80	/* Indicates if a Dedicated USB Charger(TA) is Connected */
#define	CHPORT		(u8)0x40	/* Indicates if a High Current Host/Hub is Connected */
#define TIMEOUT_CD 	(u8)0x01	/* Indicates if DP/DM contact detection */


/* IDNO */
#define IDNO_0000     0x00		/* gnd */
#define IDNO_0001     0x01		/* 24 */
#define IDNO_0010     0x02		/* 56 */
#define IDNO_0011     0x03		/* 100 */
#define IDNO_0100     0x04		/* 130 */
#define IDNO_0101     0x05		/* 180 */
#define IDNO_0110     0x06		/* 240 */
#define IDNO_0111     0x07		/* 330 */
#define IDNO_1000     0x08		/* 430 */
#define IDNO_1001     0x09		/* 620 */
#define IDNO_1010     0x0a		/* 910 */
#define IDNO_1011     0x0b		/* open */


/* MUIC chip vendor */
#define TI_TSXXX	 	0x10	/* TS5USBA33402 or TSU5611 */
#define MAX14526 		0x20
#define ANY_VENDOR 		0xff


#endif
