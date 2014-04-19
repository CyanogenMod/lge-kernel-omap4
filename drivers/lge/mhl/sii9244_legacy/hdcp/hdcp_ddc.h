/*
 * hdcp_ddc.c
 *
 * HDCP support functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author:	Sujeet Baranwal <s-baranwal@ti.com>	
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#define HDCPRX_SLV 0x74

#define MASTER_BASE		0xEC
#define MDDC_MANUAL_ADDR	0xEC        // Register Offsets
#define MDDC_SLAVE_ADDR		0xED
#define MDDC_SEGMENT_ADDR	0xEE
#define MDDC_OFFSET_ADDR	0xEF
#define MDDC_DIN_CNT_LSB_ADDR	0xF0
#define MDDC_DIN_CNT_MSB_ADDR	0xF1
#define MDDC_STATUS_ADDR	0xF2
#define MDDC_COMMAND_ADDR	0xF3
#define MDDC_FIFO_ADDR		0xF4
#define MDDC_FIFO_CNT_ADDR	0xF5

#define BIT_MDDC_ST_IN_PROGR    0x10
#define BIT_MDDC_ST_I2C_LOW     0x40
#define BIT_MDDC_ST_NO_ACK      0x20

/* DDC Command[3:0]:
 *
 * 1111 - Abort transaction
 * 1001 - Clear FIFO
 * 1010 - Clock SCL
 * 0000 - Current address read with no ACK on last byte
 * 0001 - Current address read with ACK on last byte
 * 0010 - Sequential read with no ACK on last byte
 * 0011 - Sequential read with ACK on last byte
 * 0100 - Enhanced DDC read with no ACK on last byte
 * 0101 - Enhanced DDC read with ACK on last byte
 * 0110 - Sequential write ignoring ACK on last byte
 * 0111 - Sequential write requiring ACK on last byte
 */

#define MASTER_CMD_ABORT        0x0f                    // Command Codes
#define MASTER_CMD_CLEAR_FIFO   0x09
#define MASTER_CMD_CLOCK        0x0a
#define MASTER_CMD_CUR_RD       0x00
#define MASTER_CMD_SEQ_RD       0x02
#define MASTER_CMD_ENH_RD       0x04
#define MASTER_CMD_SEQ_WR       0x06

#define MASTER_FIFO_WR_USE      0x01
#define MASTER_FIFO_RD_USE      0x02
#define MASTER_FIFO_EMPTY       0x04
#define MASTER_FIFO_FULL        0x08
#define MASTER_DDC_BUSY         0x10
#define MASTER_DDC_NOACK        0x20
#define MASTER_DDC_STUCK        0x40
#define MASTER_DDC_RSVD         0x80

/// OMAP 4 HDMI TRM:
#define HDMI_IP_CORE_SYSTEM__DDC_MAN		0x3B0
#define HDMI_IP_CORE_SYSTEM__DDC_ADDR		0x3B4 
#define HDMI_IP_CORE_SYSTEM__DDC_SEGM 		0x3B8 
#define HDMI_IP_CORE_SYSTEM__DDC_OFFSET		0x3BC 
#define HDMI_IP_CORE_SYSTEM__DDC_COUNT1		0x3C0
#define HDMI_IP_CORE_SYSTEM__DDC_COUNT2		0x3C4 
#define HDMI_IP_CORE_SYSTEM__DDC_STATUS		0x3C8 
#define HDMI_IP_CORE_SYSTEM__DDC_CMD		0x3CC 
#define HDMI_IP_CORE_SYSTEM__DDC_DATA		0x3D0 
#define HDMI_IP_CORE_SYSTEM__DDC_FIFOCNT	0x3D4

#define IIC_OK 0
#define _IIC_CAPTURED  1
#define _IIC_NOACK     2
#define _MDDC_CAPTURED 3
#define _MDDC_NOACK    4
#define _MDDC_FIFO_FULL  5

typedef struct {
	u8 slaveAddr;
	u8 offset; //suj: interpret "offset = DDC_SEGM register"
	u8 regAddr;
	u8 nbytes_lsb;	
	u8 nbytes_msb;
	u8 dummy;
	u8 cmd;
	u8 *pdata;
	u8 data[6];
} mddc_type;

enum ddc_operation {
	DDC_READ,
	DDC_WRITE
};

enum ri_suspend_resume {
	AUTO_RI_SUSPEND,
	AUTO_RI_RESUME
};



