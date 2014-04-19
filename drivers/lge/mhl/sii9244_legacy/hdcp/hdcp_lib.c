/*
 * hdcp_lib.c
 *
 * HDCP IP specific functions for TI OMAP processors.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Authors:	Sujeet Baranwal <s-baranwal@ti.com>
 *		Fabrice Olivero <f-olivero@ti.com>			
 *
 * Use of this software is controlled by the terms and conditions found
 * in the license agreement under which this software has been supplied.
 *
 */

#include <linux/delay.h>
#include "hdcp.h"

static void hdcp_lib_read_an(u8 *an);
static void hdcp_lib_read_aksv(u8 *ksv_data);
static void hdcp_lib_write_bksv(u8 *ksv_data);
static void hdcp_lib_generate_an(u8 *an);
static int hdcp_lib_r0_check(void);
static void hdcp_lib_get_m0(u8 mo_tx[8]);
static void hdcp_lib_set_repeater_bit_in_tx(enum hdcp_repeater rx_mode);
static void hdcp_lib_toggle_repeater_bit_in_tx(void);
static int hdcp_lib_initiate_step1(void);
static int hdcp_lib_check_ksv(uint8_t ksv[5]);
static int hdcp_lib_sha_bstatus_mx(struct hdcp_sha_in * sha, u8 m0[8]);
static int hdcp_lib_compare_vi(struct hdcp_sha_context *sha);

static u8 mo_tx[8]; /* Used to store M0 value for SHA-1 calculation */

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_read_an
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_read_an(u8 *an)
{
        u8 i;

        for(i = 0; i < 8; i++) {
                an[i] = (RD_REG_32(hdcp.hdmi_wp_base_addr +
			 HDMI_IP_CORE_SYSTEM,
			 HDMI_IP_CORE_SYSTEM__AN0 +
			 i * sizeof(uint32_t))) & 0xFF;
        }
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_read_aksv
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_read_aksv(u8 *ksv_data)
{
	u8 i;
	for(i = 0; i < 5; i++) {
		ksv_data[i] = RD_REG_32(hdcp.hdmi_wp_base_addr +
				   HDMI_IP_CORE_SYSTEM, HDMI_IP_CORE_SYSTEM__AKSV0 +
				   i * sizeof(uint32_t));
	
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_write_bksv
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_write_bksv(u8 *ksv_data)
{
	u8 i;
	for(i = 0; i < 5; i++) {
		WR_REG_32(hdcp.hdmi_wp_base_addr +
			HDMI_IP_CORE_SYSTEM, HDMI_IP_CORE_SYSTEM__BKSV0 +
			i * sizeof(uint32_t), ksv_data[i]);
	}
}
/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_generate_an
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_generate_an(u8 *an)
{
	/* Generate An using HDCP HW */
	/* TODO: confirm if we can keep this 10 ms delay */

	DBG("hdcp_lib_generate_an()\n");

	/* Start AN Gen */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 3, 3, 0);

	/* Delay of 10 ms */
	mdelay(10);

	/* Stop AN Gen */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 3, 3, 1);

	/* Must set 0x72:0x0F[3] twice to guarantee that takes effect */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 3, 3, 1);

	hdcp_lib_read_an(an);

	DBG("AN: %x %x %x %x %x %x %x %x\n", an[0],an[1], an[2], an[3], an[4],an[5], an[6], an[7]);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_r0_check
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_r0_check(void)
{
	u8 ro_rx[2],ro_tx[2];

	DBG("hdcp_lib_r0_check()\n");

	/* DDC: Read Ri' from RX */
	if (hdcp_ddc_read(DDC_Ri_LEN, DDC_Ri_ADDR , (u8 *)&ro_rx))
		return -HDCP_DDC_ERROR;

	/* Read Ri in HDCP IP */
	ro_tx[0] = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			     HDMI_IP_CORE_SYSTEM__R1) & 0xFF;

	ro_tx[1] = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			     HDMI_IP_CORE_SYSTEM__R2) & 0xFF;

	/* Compare values */
	DBG("ROTX: %x%x RORX:%x%x\n",ro_tx[0],ro_tx[1],ro_rx[0],ro_rx[1]);

    if ((ro_rx[0] == ro_tx[0]) && (ro_rx[1] == ro_tx[1]))
        return HDCP_OK;
    else
        return -HDCP_AUTH_FAILURE;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_get_m0
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_get_m0(u8 mo_tx[8])
{
	DBG("hdcp_lib_get_m0()\n");

	/* Enable M0 reading */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__SHA_CTRL, 3, 3, 1);

	/* M0 available in An register after 1st part authentication */
	hdcp_lib_read_an(mo_tx);

	/* Disable M0 reading */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__SHA_CTRL, 3, 3, 0);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_sha_bstatus_mx
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_sha_bstatus_mx(struct hdcp_sha_in *sha, u8 m0[8])
{
	u8 data[2],i;

	if (hdcp_ddc_read(DDC_BSTATUS_LEN, DDC_BSTATUS_ADDR, data))
		return -HDCP_DDC_ERROR;

	sha->data[sha->byte_counter++] = data[0];
	sha->data[sha->byte_counter++] = data[1];

	for(i = 0; i < 8; i++)
		sha->data[sha->byte_counter++] = m0[i];

	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_compare_vi
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_compare_vi(struct hdcp_sha_context *sha)
{
	u8 vprime[DDC_V_LEN];

	/* Read V' */
	if (hdcp_ddc_read(DDC_V_LEN, DDC_V_ADDR, vprime))
		return -HDCP_DDC_ERROR;

	if(!memcmp(vprime, sha->Message_Digest, 20)) {
		DBG("Vi Comparison PASSED\n");
		return HDCP_OK;
	}
	else {
		DBG_ERROR("Vi Comparison FAILED\n");
		return -HDCP_AUTH_FAILURE;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_set_repeater_bit_in_tx
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_set_repeater_bit_in_tx(enum hdcp_repeater rx_mode)
{
	DBG("hdcp_lib_set_repeater_bit_in_tx() value=%d\n", rx_mode);

	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 4, 4, rx_mode);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_toggle_repeater_bit_in_tx
 *-----------------------------------------------------------------------------
 */
static void hdcp_lib_toggle_repeater_bit_in_tx(void)
{
	if(hdcp_lib_check_repeater_bit_in_tx())
		hdcp_lib_set_repeater_bit_in_tx(HDCP_RECEIVER);
	else
		hdcp_lib_set_repeater_bit_in_tx(HDCP_REPEATER);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_initiate_step1
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_initiate_step1(void)
{
	/* HDCP authentication steps:
	 *   1) Read Bksv - check validity (is HDMI Rx supporting HDCP ?)
	 *   2) Initializes HDCP (CP reset release)
	 *   3) Read Bcaps - is HDMI Rx a repeater ?
	 *   *** First part authentication ***
	 *   4) Generates An
	 *   5) DDC: Writes An, Aksv
	 *   6) DDC: Read Bksv
	 */
	uint8_t an_ksv_data[8];
	uint8_t rx_type;
	
	DBG("hdcp_lib_initiate_step1()\n");

	/* DDC: Read BKSV from RX */
	if (hdcp_ddc_read(DDC_BKSV_LEN, DDC_BKSV_ADDR , an_ksv_data))
		return -HDCP_DDC_ERROR;

	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;
	DBG("BKSV: %02x %02x %02x %02x %02x\n", an_ksv_data[0], an_ksv_data[1],
					      an_ksv_data[2], an_ksv_data[3],
					      an_ksv_data[4]);

	if (hdcp_lib_check_ksv(an_ksv_data)) {
		DBG_ERROR("BKSV error (number of 0 and 1)\n");
		return -HDCP_AUTH_FAILURE;
	}

	/* TODO: Need to confirm it is required */
#ifndef _9032_AN_STOP_FIX_
	hdcp_lib_toggle_repeater_bit_in_tx();
#endif

	/* Release CP reset bit */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 2, 2, 1);

	/* Read BCAPS to determine if HDCP RX is a repeater */
	if (hdcp_ddc_read(DDC_BCAPS_LEN, DDC_BCAPS_ADDR, &rx_type))
		return -HDCP_DDC_ERROR;


	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;

	rx_type = FLD_GET(rx_type, DDC_BIT_REPEATER, DDC_BIT_REPEATER);

	/* Set repeater bit in HDCP CTRL */
	if (rx_type == 1) {
		hdcp_lib_set_repeater_bit_in_tx(HDCP_REPEATER);
		DBG("HDCP RX is a repeater\n");
	}
	else {
		hdcp_lib_set_repeater_bit_in_tx(HDCP_RECEIVER);
		DBG("HDCP RX is a receiver\n");
	}

	/* Generate An */
	hdcp_lib_generate_an(an_ksv_data);

	/* Authentication 1st step initiated HERE */
	
	/* DDC: Write An */
	if (hdcp_ddc_write(DDC_AN_LEN, DDC_AN_ADDR , an_ksv_data))
		return -HDCP_DDC_ERROR;


	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;

    
	hdcp_lib_read_aksv(an_ksv_data);

	DBG("AKSV: %02x %02x %02x %02x %02x\n", an_ksv_data[0], an_ksv_data[1],
					      an_ksv_data[2], an_ksv_data[3],
					      an_ksv_data[4]);

	if (hdcp_lib_check_ksv(an_ksv_data)) {
		DBG_ERROR("AKSV error (number of 0 and 1)\n");
		return -HDCP_AKSV_ERROR;
	}

	if (hdcp.pending_disable)
    {
        DBG_ERROR("HDCP_CANCELLED_AUTH\n");
		return -HDCP_CANCELLED_AUTH;
    }
	/* DDC: Write AKSV */
	if (hdcp_ddc_write(DDC_AKSV_LEN, DDC_AKSV_ADDR, an_ksv_data))
	{
	    DBG_ERROR("HDCP_DDC_ERROR\n");
	    return -HDCP_DDC_ERROR;
    }

	if (hdcp.pending_disable)
	{
	    DBG_ERROR("HDCP_CANCELLED_AUTH\n");
	    return -HDCP_CANCELLED_AUTH;
    }

	/* DDC: Read BKSV from RX */
	if (hdcp_ddc_read(DDC_BKSV_LEN, DDC_BKSV_ADDR , an_ksv_data))
	{
	    DBG_ERROR("HDCP_DDC_ERROR\n");
	    return -HDCP_DDC_ERROR;
    }

	/* Write Bksv to IP */
	hdcp_lib_write_bksv(an_ksv_data);

	/* Check IP BKSV error */
	if(RD_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 5, 5))
	{
	    DBG_ERROR("HDCP_AUTH_FAILURE\n");
	    return -HDCP_AUTH_FAILURE;
    }

	/* Here BSKV should be checked against revokation list */

	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_check_ksv
 *-----------------------------------------------------------------------------
 */
static int hdcp_lib_check_ksv(uint8_t ksv[5])
{
	int i, j;
	int zero = 0, one = 0;

	for(i = 0; i < 5; i++) {
		/* Count number of zero / one */
		for (j = 0; j < 8; j++) {
			if (ksv[i] & (0x01 << j))
				one++;
			else
				zero++;
		}
	}

	if (one == zero)
		return 0;
	else
		return -1;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_3des_load_key
 *-----------------------------------------------------------------------------
 */
int hdcp_3des_load_key(uint32_t *deshdcp_encrypted_key)
{
	int counter = 0, status = HDCP_OK;

	DBG("Loading HDCP keys...\n");

    if(!deshdcp_encrypted_key)
    {
        DBG_ERROR("deshdcp_encrypted_key is NULL");
        return -1;
    }
    
	/* Set decryption mode in DES control register */
  	WR_FIELD_32(hdcp.deshdcp_base_addr,
		    DESHDCP__DHDCP_CTRL,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		    0x0);

	/* Write encrypted data */
	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			  deshdcp_encrypted_key[counter]);
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			  deshdcp_encrypted_key[counter + 1]);

		/* Wait for output bit at '1' */
		while(
			RD_FIELD_32(hdcp.deshdcp_base_addr,
				    DESHDCP__DHDCP_CTRL,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L
			) != 0x1) {};

		/* Dummy read (indeed data are transfered directly into
		 * key memory)
		 */
		if (RD_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L) !=
									0x0) {
			status = -HDCP_3DES_ERROR;
			DBG_ERROR("DESHDCP dummy read error\n");
		}
		if (RD_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H) !=
									0x0) {
			status = -HDCP_3DES_ERROR;
			DBG_ERROR("DESHDCP dummy read error\n");
		}

		counter+=2;
	}

	return status;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_3des_encrypt_key
 *-----------------------------------------------------------------------------
 */
void hdcp_3des_encrypt_key(struct hdcp_encrypt_control *enc_ctrl,
			   uint32_t out_key[DESHDCP_KEY_SIZE])
{
	int counter = 0;

	DBG("Encrypting HDCP keys...\n");

	/* Reset encrypted key array */
	for (counter = 0; counter < DESHDCP_KEY_SIZE; counter++)
		out_key[counter] = 0;

	/* Set encryption mode in DES control register */
  	WR_FIELD_32(hdcp.deshdcp_base_addr,
		    DESHDCP__DHDCP_CTRL,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_F,
		    DESHDCP__DHDCP_CTRL__DIRECTION_POS_L,
		    0x1);

  	/* Write raw data and read encrypted data */
  	counter = 0;

	while (counter < DESHDCP_KEY_SIZE) {
		/* Fill Data registers */
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_L,
			  enc_ctrl->in_key[counter]);
		WR_REG_32(hdcp.deshdcp_base_addr, DESHDCP__DHDCP_DATA_H,
			  enc_ctrl->in_key[counter + 1]);

		/* Wait for output bit at '1' */
		while(
			RD_FIELD_32(hdcp.deshdcp_base_addr,
				    DESHDCP__DHDCP_CTRL,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_F,
				    DESHDCP__DHDCP_CTRL__OUTPUT_READY_POS_L
			) != 0x1) {};

    		/* Read enrypted data */
    		out_key[counter]     = RD_REG_32(hdcp.deshdcp_base_addr,
						 DESHDCP__DHDCP_DATA_L);
    		out_key[counter + 1] = RD_REG_32(hdcp.deshdcp_base_addr,
						 DESHDCP__DHDCP_DATA_H);

		counter+=2;
	}
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_disable
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_disable()
{
	DBG("hdcp_lib_disable() %u\n", jiffies_to_msecs(jiffies));
	
	/* CP reset */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 2, 2, 0);

	/* Clear AV mute in case it was set */
	hdcp_lib_set_av_mute(AV_MUTE_CLEAR);
	
	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_set_encryption
 *-----------------------------------------------------------------------------
 */
void hdcp_lib_set_encryption(enum encryption_state enc_state)
{
	unsigned long flags;
	DBG("Encryption state changed: %s\n", enc_state == HDCP_ENC_OFF ? "OFF" : "ON");

	spin_lock_irqsave(&hdcp.spinlock, flags);

	/* HDCP_CTRL::ENC_EN set/clear */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 0, 0, enc_state);

	/* Read to flush */
	RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__HDCP_CTRL);

	spin_unlock_irqrestore(&hdcp.spinlock, flags);

	DBG("Encryption state changed: %s hdcp_ctrl: %02x\n",
				enc_state == HDCP_ENC_OFF ? "OFF" : "ON",
				RD_REG_32(hdcp.hdmi_wp_base_addr +
					  HDMI_IP_CORE_SYSTEM,
					  HDMI_IP_CORE_SYSTEM__HDCP_CTRL));

}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_set_cp_packet
 *-----------------------------------------------------------------------------
 */
void hdcp_lib_set_av_mute(enum av_mute av_mute_state)
{
	unsigned long flags;
    u8 RegVal, TimeOutCount = 64;



	spin_lock_irqsave(&hdcp.spinlock, flags);

		

	DBG("hdcp_lib_set_cp_packet() av_mute=%d\n", av_mute_state);

    
    {
		RegVal = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
				   HDMI_CORE_AV_PB_CTRL2);

		/* PRguide-GPC: To change the content of the CP_BYTE1 register,
		 * CP_EN must be zero
		 * set PB_CTRL2 :: CP_RPT = 0
		 */
		WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
			    HDMI_CORE_AV_PB_CTRL2, 2, 2, 0);

		/* Set/clear AV mute state */
		WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
			  HDMI_CORE_AV_CP_BYTE1, av_mute_state);

		/* FIXME: This loop should be removed */
		while(TimeOutCount--) {
			/* continue in this loop till CP_EN becomes 0, prior to TimeOutCount becoming 0 */
			if( !RD_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
					 HDMI_CORE_AV_PB_CTRL2, 3, 3))
			break;
		}

		DBG("    timeoutcount=%d\n", TimeOutCount);

		/* FIXME: why is this if condition required?, according to prg, this shall be unconditioanlly */
		if(TimeOutCount) {
			/* set PB_CTRL2 :: CP_EN = 1 & CP_RPT = 1 */
			RegVal = FLD_MOD(RegVal, 0x3, 3, 2);

			WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
				  HDMI_CORE_AV_PB_CTRL2, RegVal);
		}
	}

	spin_unlock_irqrestore(&hdcp.spinlock, flags);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_check_repeater_bit_in_tx
 *-----------------------------------------------------------------------------
 */
u8 hdcp_lib_check_repeater_bit_in_tx(void)
{
        return RD_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			   HDMI_IP_CORE_SYSTEM__HDCP_CTRL, 4, 4);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_auto_ri_check
 *-----------------------------------------------------------------------------
 */
void hdcp_lib_auto_ri_check(bool state)
{
	u8 reg_val;
	unsigned long flags;

	DBG("hdcp_lib_auto_ri_check() state=%s\n",
		state == true ? "ON" : "OFF");

	spin_lock_irqsave(&hdcp.spinlock, flags);

	reg_val = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			    HDMI_IP_CORE_SYSTEM__INT_UNMASK3);

	reg_val = (state == true) ? (reg_val | 0xB0) : (reg_val & ~0xB0);

	/* Turn on/off the following Auto Ri interrupts */
	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__INT_UNMASK3, reg_val);

	/* Enable/Disable Ri */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__RI_CMD, 0, 0,
		    ((state == true) ? 1 : 0));

	/* Read to flush */
	RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__RI_CMD);

	spin_unlock_irqrestore(&hdcp.spinlock, flags);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_auto_bcaps_rdy_check
 *-----------------------------------------------------------------------------
 */
void hdcp_lib_auto_bcaps_rdy_check(bool state)
{
	u8 reg_val;
	unsigned long flags;

	DBG("hdcp_lib_auto_bcaps_rdy_check() state=%s\n",
		state == true ? "ON" : "OFF");

	spin_lock_irqsave(&hdcp.spinlock, flags);

	/* Enable KSV_READY / BACP_DONE interrupt */
	WR_FIELD_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		    HDMI_IP_CORE_SYSTEM__INT_UNMASK2, 7, 7, ((state == true) ? 1 : 0));

	/* Enable/Disable Ri  & Bcap */
	reg_val = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
			    HDMI_IP_CORE_SYSTEM__RI_CMD);

	/* Enable RI_EN & BCAP_EN OR disable BCAP_EN */
	reg_val = (state == true) ? (reg_val | 0x3) : (reg_val & ~0x2);

	WR_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__RI_CMD, reg_val);

	/* Read to flush */
	RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_IP_CORE_SYSTEM,
		  HDMI_IP_CORE_SYSTEM__RI_CMD);

	spin_unlock_irqrestore(&hdcp.spinlock, flags);
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step1_start
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step1_start(void)
{
	u8 hdmi_mode;
	int status;

	DBG("hdcp_lib_step1_start() %u\n", jiffies_to_msecs(jiffies));

	/* Check if mode is HDMI or DVI */
	hdmi_mode = RD_REG_32(hdcp.hdmi_wp_base_addr + HDMI_CORE_AV_BASE,
			      HDMI_CORE_AV_HDMI_CTRL) &
			      HDMI_CORE_AV_HDMI_CTRL__HDMI_MODE;

	DBG("RX mode: %s\n", hdmi_mode ? "HDMI" : "DVI");

	/* Set AV Mute in General Control Packet */
    hdcp_lib_set_av_mute(AV_MUTE_SET);

	/* Must turn encryption off when AVMUTE */
	hdcp_lib_set_encryption(HDCP_ENC_OFF);

	status = hdcp_lib_initiate_step1();

	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;
	else
		return status;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step1_r0_check
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step1_r0_check(void)
{
	int status = HDCP_OK;
	/* HDCP authentication steps:
	 *   1) DDC: Read M0'
	 *   2) Compare M0 and M0'
	 *   if Rx is a receiver: switch to authentication step 3
	 *   3) Enable encryption / auto Ri check / disable AV mute
	 *   if Rx is a repeater: switch to authentication step 2
	 *   3) Get M0 from HDMI IP and store it for further processing (V)
	 *   4) Enable encryption / auto Ri check / auto BCAPS RDY polling
	 *      Disable AV mute
	 */

	DBG("hdcp_lib_step1_r0_check() %u\n", jiffies_to_msecs(jiffies));

	status = hdcp_lib_r0_check();
	if (status < 0)
		return status;

	/* Authentication 1st step done */

	/* Now prepare 2nd step authentication in case of RX repeater and
	 * enable encryption / Ri check
	 */

 	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;

	if (hdcp_lib_check_repeater_bit_in_tx()) {
		/* Repeater: initiates 2nd part authentication */

		/* Read M0 from IP */
		hdcp_lib_get_m0(mo_tx);

		/* Enable encryption */
		hdcp_lib_set_encryption(HDCP_ENC_ON);

#ifdef _9032_AUTO_RI_
		/* Enable Auto Ri */
		hdcp_lib_auto_ri_check(true);
#endif

#ifdef _9032_BCAP_
		/* Enable automatic BCAPS polling */
		hdcp_lib_auto_bcaps_rdy_check(true);
#endif

		/* Now, IP waiting for BCAPS ready bit */
	}
	else {
		/* Receiver: enable encryption and auto Ri check */
		hdcp_lib_set_encryption(HDCP_ENC_ON);

#ifdef _9032_AUTO_RI_
		/* Enable Auto Ri */
		hdcp_lib_auto_ri_check(true);
#endif

	}

	/* Disable AV Mute in General Control Packet */
	/* Note: AV mute is required to have encryption enable to take effect */
	/* TODO: DVI mode ? Think about when to unmute for repeater */
	hdcp_lib_set_av_mute(AV_MUTE_CLEAR);

	return HDCP_OK;
}

/*-----------------------------------------------------------------------------
 * Function: hdcp_lib_step2
 *-----------------------------------------------------------------------------
 */
int hdcp_lib_step2(void)
{
	/* HDCP authentication steps:
	 *   1) Disable auto Ri check
	 *   2) DDC: read BStatus (nb of devices, MAX_DEV
	 */

	u8 bstatus[2];
	struct hdcp_sha_in sha_input;
	struct hdcp_sha_context sha_n;
	int status;

	DBG("hdcp_lib_step2() %u\n", jiffies_to_msecs(jiffies));

#ifdef _9032_AUTO_RI_
	/* Disable Auto Ri */
	hdcp_lib_auto_ri_check(false);
#endif

	/* DDC: Read Bstatus (1st byte) from Rx */
	if (hdcp_ddc_read(DDC_BSTATUS_LEN, DDC_BSTATUS_ADDR, bstatus))
		return -HDCP_DDC_ERROR;

	/* Get KSV list size */
	DBG("KSV list size: %d\n", bstatus[0] & DDC_BSTATUS0_DEV_COUNT);
	sha_input.byte_counter = (bstatus[0] & DDC_BSTATUS0_DEV_COUNT) * 5;

	/* Check BStatus topology errors */
	if (bstatus[0] & DDC_BSTATUS0_MAX_DEVS) {
		DBG("MAX_DEV_EXCEEDED set\n");
		return -HDCP_AUTH_FAILURE;
	}

	if (bstatus[1] & DDC_BSTATUS1_MAX_CASC) {
		DBG("MAX_CASCADE_EXCEEDED set\n");
		return -HDCP_AUTH_FAILURE;
	}

	DBG("Retrieving KSV list...\n");
	
	/* Set SHA input data: KSV list, Bstatus, M0 */
	memset(sha_input.data, 0, MAX_KSV_LIST_BST_M0);

	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;

	/* DDC: read KSV list */
	if (sha_input.byte_counter) {
		if (hdcp_ddc_read(sha_input.byte_counter, DDC_KSV_FIFO_ADDR,
				  (u8*)&sha_input.data))
			return -HDCP_DDC_ERROR;
	}

	/* Add Bstatus and M0 */
	if (hdcp_lib_sha_bstatus_mx(&sha_input, mo_tx))
		return -HDCP_DDC_ERROR;

	DBG("Processing V...\n");

	/* SHA1 processing */
	hdcp_sha1_reset(&sha_n);
	hdcp_sha1_input(&sha_n, (const unsigned char *) sha_input.data,
			sha_input.byte_counter);

	if (hdcp_sha1_result(&sha_n)) {
		DBG_ERROR("SHA-1: ERROR - Could not compute message digest\n");
		return -HDCP_SHA1_ERROR;
	}

	if (hdcp.pending_disable)
		return -HDCP_CANCELLED_AUTH;

	/* Compare V and V' */
	status = hdcp_lib_compare_vi(&sha_n);
	if (status == HDCP_OK) {
		/* Re-enable Ri check */
#ifdef _9032_AUTO_RI_
		hdcp_lib_auto_ri_check(true);
#endif
		return HDCP_OK;
	}
	else
		return status;

}
