/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Authors: Adam Hampson <ahampson@sta.samsung.com>
 *          Erik Gilling <konkers@android.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sii9244.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/usb/otg_id.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_MUIC)
#include <linux/muic/muic.h>
#include <linux/muic/muic_client.h>
#endif


#define T_SRC_VBUS_CBUS_TO_STABLE	200
#define T_SRC_WAKE_PULSE_WIDTH_1	19
#define T_SRC_WAKE_PULSE_WIDTH_2	60
#define T_SRC_WAKE_TO_DISCOVER		500
#define T_SRC_VBUS_CBUS_T0_STABLE	500

/* MHL TX Addr 0x72 Registers */
#define MHL_TX_IDL_REG			0x02
#define MHL_TX_IDH_REG			0x03
#define MHL_TX_REV_REG			0x04
#define MHL_TX_SRST			0x05
#define MHL_TX_INTR1_REG		0x71
#define MHL_TX_INTR2_REG		0x72	/* Not Documented */
#define MHL_TX_INTR3_REG		0x73	/* Not Documented */
#define MHL_TX_INTR4_REG		0x74
#define MHL_TX_INTR1_ENABLE_REG		0x75
#define MHL_TX_INTR2_ENABLE_REG		0x76	/* Not Documented */
#define MHL_TX_INTR3_ENABLE_REG		0x77	/* Not Documented */
#define MHL_TX_INTR4_ENABLE_REG		0x78
#define MHL_TX_INT_CTRL_REG		0x79
#define MHL_TX_TMDS_CCTRL		0x80

#define MHL_TX_DISC_CTRL1_REG		0x90
#define MHL_TX_DISC_CTRL2_REG		0x91
#define MHL_TX_DISC_CTRL3_REG		0x92
#define MHL_TX_DISC_CTRL4_REG		0x93	/* Not Documented */

/* There doesn't seem to be any documentation for CTRL5 but it looks like
 * it is some sort of pull up control register
 */
#define MHL_TX_DISC_CTRL5_REG		0x94
#define MHL_TX_DISC_CTRL6_REG		0x95
#define MHL_TX_DISC_CTRL7_REG		0x96
#define MHL_TX_DISC_CTRL8_REG		0x97	/* Not Documented */
#define MHL_TX_STAT1_REG		0x98	/* Not Documented */
#define MHL_TX_STAT2_REG		0x99

#define MHL_TX_MHLTX_CTL1_REG		0xA0
#define MHL_TX_MHLTX_CTL2_REG		0xA1
#define MHL_TX_MHLTX_CTL4_REG		0xA3
#define MHL_TX_MHLTX_CTL6_REG		0xA5
#define MHL_TX_MHLTX_CTL7_REG		0xA6

/* MHL TX SYS STAT Registers
 * Not Documented, mentioned only in reference of RSEN
 */
#define MHL_TX_SYSSTAT_REG		0x09

/* MHL TX SYS STAT Register Bits */
#define RSEN_STATUS			(1<<2)

/* MHL TX INTR4 Register Bits */
#define RGND_READY_INT			(1<<6)
#define VBUS_LOW_INT			(1<<5)
#define CBUS_LKOUT_INT			(1<<4)
#define MHL_DISC_FAIL_INT		(1<<3)
#define MHL_EST_INT			(1<<2)

/* MHL TX INTR4_ENABLE 0x78 Register Bits */
#define RGND_READY_MASK			(1<<6)
#define CBUS_LKOUT_MASK			(1<<4)
#define MHL_DISC_FAIL_MASK		(1<<3)
#define MHL_EST_MASK			(1<<2)

/* MHL TX INTR1 Register Bits*/
#define HPD_CHANGE_INT			(1<<6)
#define RSEN_CHANGE_INT			(1<<5)

/* MHL TX INTR1_ENABLE 0x75 Register Bits*/
#define HPD_CHANGE_INT_MASK		(1<<6)
#define RSEN_CHANGE_INT_MASK		(1<<5)

/* CBUS Interrupt Status Registers*/
#define CBUS_INT_STATUS_1_REG		0x08
#define CBUS_INT_STATUS_2_REG		0x1E

/* CBUS INTR1 STATUS Register bits */
#define MSC_RESP_ABORT			(1<<6)
#define MSC_REQ_ABORT			(1<<5)
#define MSC_REQ_DONE			(1<<4)
#define MSC_MSG_RECD			(1<<3)
#define CBUS_DDC_ABORT			(1<<2)

/* CBUS INTR1 STATUS 0x09 Enable Mask*/
#define MSC_RESP_ABORT_MASK		(1<<6)
#define MSC_REQ_ABORT_MASK		(1<<5)
#define MSC_REQ_DONE_MASK		(1<<4)
#define MSC_MSG_RECD_MASK		(1<<3)
#define CBUS_DDC_ABORT_MASK		(1<<2)

/* CBUS INTR2 STATUS Register bits */
#define WRT_STAT_RECD			(1<<3)
#define SET_INT_RECD			(1<<2)
#define WRT_BURST_RECD			(1<<0)

/* CBUS INTR2 STATUS 0x1F Enable Mask*/
#define WRT_STAT_RECD_MASK		(1<<3)
#define SET_INT_RECD_MASK		(1<<2)
#define WRT_BURST_RECD_MASK		(1<<0)

/* CBUS Control Registers*/
/* Retry count for all MSC commands*/
#define MSC_RETRY_FAIL_LIM_REG		0x1D

/* reason for MSC_REQ_ABORT interrupt on CBUS */
#define MSC_REQ_ABORT_REASON_REG	0x0D

#define MSC_RESP_ABORT_REASON_REG	0x0E

/* MSC Requestor Abort Reason Register bits*/
#define ABORT_BY_PEER			(1<<7)
#define UNDEF_CMD			(1<<3)
#define TIMEOUT				(1<<2)
#define PROTO_ERROR			(1<<1)
#define MAX_FAIL			(1<<0)

/* MSC Responder Abort Reason Register bits*/
#define ABORT_BY_PEER			(1<<7)
#define UNDEF_CMD			(1<<3)
#define TIMEOUT				(1<<2)

/* Set HPD came from Downstream, not documented */
#define SET_HPD_DOWNSTREAM		(1<<6)

/* MHL TX DISC1 Register Bits */
#define DISC_EN				(1<<0)

/* MHL TX DISC2 Register Bits */
#define SKIP_GND			(1<<6)
#define ATT_THRESH_SHIFT		0x04
#define ATT_THRESH_MASK			(0x03 << ATT_THRESH_SHIFT)
#define USB_D_OEN			(1<<3)
#define DEGLITCH_TIME_MASK		0x07
#define DEGLITCH_TIME_2MS		0
#define DEGLITCH_TIME_4MS		1
#define DEGLITCH_TIME_8MS		2
#define DEGLITCH_TIME_16MS		3
#define DEGLITCH_TIME_40MS		4
#define DEGLITCH_TIME_50MS		5
#define DEGLITCH_TIME_60MS		6
#define DEGLITCH_TIME_128MS		7

#define DISC_CTRL3_COMM_IMME		(1<<7)
#define DISC_CTRL3_FORCE_MHL		(1<<6)
#define DISC_CTRL3_FORCE_USB		(1<<4)
#define DISC_CTRL3_USB_EN		(1<<3)

/* MHL TX DISC4 0x93 Register Bits: undocumented */
#define CBUS_DISC_PUP_SEL_SHIFT		6
#define CBUS_DISC_PUP_SEL_MASK		(3<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_DISC_PUP_SEL_10K		(2<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_DISC_PUP_SEL_OPEN		(0<<CBUS_DISC_PUP_SEL_SHIFT)
#define CBUS_IDLE_PUP_SEL_SHIFT		4
#define CBUS_IDLE_PUP_SEL_MASK		(3<<CBUS_IDLE_PUP_SEL_SHIFT)
#define CBUS_IDLE_PUP_SEL_OPEN		(0<<CBUS_IDLE_PUP_SEL_SHIFT)

/* MHL TX DISC5 0x94 Register Bits */
#define CBUS_MHL_PUP_SEL_MASK		0x03	/* Not Documented */
#define CBUS_MHL_PUP_SEL_5K		0x01	/* Not Documented */
#define CBUS_MHL_PUP_SEL_OPEN		0x00

/* MHL TX DISC6 0x95 Register Bits */
#define USB_D_OVR			(1<<7)
#define USB_ID_OVR			(1<<6)
#define DVRFLT_SEL			(1<<5)
#define BLOCK_RGND_INT			(1<<4)
#define SKIP_DEG			(1<<3)
#define CI2CA_POL			(1<<2)
#define CI2CA_WKUP			(1<<1)
#define SINGLE_ATT			(1<<0)

/* MHL TX DISC7 0x96 Register Bits
 *
 * Bits 7 and 6 are labeled as reserved but seem to be related to toggling
 * the CBUS signal when generating the wake pulse sequence.
 */
#define USB_D_ODN			(1<<5)
#define VBUS_CHECK			(1<<2)
#define RGND_INTP_MASK			0x03
#define RGND_INTP_OPEN			0
#define RGND_INTP_2K			1
#define RGND_INTP_1K			2
#define RGND_INTP_SHORT			3

/* TPI Addr 0x7A Registers */
#define TPI_DPD_REG			0x3D

#define TPI_PD_TMDS			(1<<5)
#define TPI_PD_OSC_EN			(1<<4)
#define TPI_TCLK_PHASE			(1<<3)
#define TPI_PD_IDCK			(1<<2)
#define TPI_PD_OSC			(1<<1)
#define TPI_PD				(1<<0)

#define CBUS_CONFIG_REG			0x07
#define CBUS_INT_STATUS_1_REG		0x08
#define CBUS_INT_STATUS_2_REG		0x1E
#define CBUS_LINK_CONTROL_2_REG		0x31

/* HDMI RX Registers */
#define HDMI_RX_TMDS0_CCTRL1_REG	0x10
#define HDMI_RX_TMDS_CLK_EN_REG		0x11
#define HDMI_RX_TMDS_CH_EN_REG		0x12
#define HDMI_RX_PLL_CALREFSEL_REG	0x17
#define HDMI_RX_PLL_VCOCAL_REG		0x1A
#define HDMI_RX_EQ_DATA0_REG		0x22
#define HDMI_RX_EQ_DATA1_REG		0x23
#define HDMI_RX_EQ_DATA2_REG		0x24
#define HDMI_RX_EQ_DATA3_REG		0x25
#define HDMI_RX_EQ_DATA4_REG		0x26
#define HDMI_RX_TMDS_ZONE_CTRL_REG	0x4C
#define HDMI_RX_TMDS_MODE_CTRL_REG	0x4D

enum rgnd_state {
	RGND_UNKNOWN = 0,
	RGND_OPEN,
	RGND_1K,
	RGND_2K,
	RGND_SHORT
};

enum mhl_state {
	STATE_DISCONNECTED = 0,
	STATE_DISCOVERY_FAILED,
	STATE_CBUS_LOCKOUT,
	STATE_ESTABLISHED,
};

static inline bool mhl_state_is_error(enum mhl_state state)
{
	return state == STATE_DISCOVERY_FAILED ||
		state == STATE_CBUS_LOCKOUT;
}

struct sii9244_data {
	struct sii9244_platform_data	*pdata;
	struct otg_id_notifier_block	otg_id_nb;
	wait_queue_head_t		wq;

	bool				claimed;
	enum mhl_state			state;
	enum rgnd_state			rgnd;
	int				irq;
	bool				rsen;

	struct mutex			lock;
	struct regulator *mhl_reg;
};

static irqreturn_t sii9244_irq_thread(int irq, void *data);

static int mhl_tx_write_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9244->pdata->mhl_tx_client, offset,
			value);
}

static int mhl_tx_read_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9244->pdata->mhl_tx_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9244->pdata->mhl_tx_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int mhl_tx_set_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = mhl_tx_read_reg(sii9244, offset, &value);
	if (ret < 0)
		return ret;

	value |= mask;

	return mhl_tx_write_reg(sii9244, offset, value);
}

static int mhl_tx_clear_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = mhl_tx_read_reg(sii9244, offset, &value);
	if (ret < 0)
		return ret;

	value &= ~mask;

	return mhl_tx_write_reg(sii9244, offset, value);
}

static int tpi_write_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9244->pdata->tpi_client, offset,
			value);
}

static int tpi_read_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9244->pdata->tpi_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9244->pdata->tpi_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int hdmi_rx_write_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9244->pdata->hdmi_rx_client, offset,
			value);
}

static int cbus_write_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 value)
{
	return i2c_smbus_write_byte_data(sii9244->pdata->cbus_client, offset,
			value);
}

static int cbus_read_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 *value)
{
	int ret;

	if (!value)
		return -EINVAL;

	ret = i2c_smbus_write_byte(sii9244->pdata->cbus_client, offset);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_byte(sii9244->pdata->cbus_client);
	if (ret < 0)
		return ret;

	*value = ret & 0x000000FF;

	return 0;
}

static int cbus_set_reg(struct sii9244_data *sii9244, unsigned int offset,
		u8 mask)
{
	int ret;
	u8 value;

	ret = cbus_read_reg(sii9244, offset, &value);
	if (ret < 0)
		return ret;

	value |= mask;

	return cbus_write_reg(sii9244, offset, value);
}

static int mhl_wake_toggle(struct sii9244_data *sii9244,
		unsigned long high_period,
		unsigned long low_period)
{
	int ret;

	/* These bits are not documented. */
	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL7_REG, (1<<7) | (1<<6));
	if (ret < 0)
		return ret;

	usleep_range(high_period * USEC_PER_MSEC, high_period * USEC_PER_MSEC);

	ret = mhl_tx_clear_reg(sii9244, MHL_TX_DISC_CTRL7_REG, (1<<7) | (1<<6));
	if (ret < 0)
		return ret;

	usleep_range(low_period * USEC_PER_MSEC, low_period * USEC_PER_MSEC);

	return 0;
}

static int mhl_send_wake_pulses(struct sii9244_data *sii9244)
{
	int ret;

	ret = mhl_wake_toggle(sii9244, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_1);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9244, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_2);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9244, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_PULSE_WIDTH_1);
	if (ret < 0)
		return ret;

	ret = mhl_wake_toggle(sii9244, T_SRC_WAKE_PULSE_WIDTH_1,
			T_SRC_WAKE_TO_DISCOVER);
	if (ret < 0)
		return ret;

	return 0;
}

static int sii9244_cbus_init(struct sii9244_data *sii9244)
{
	u8 value;

	cbus_write_reg(sii9244, 0x07, 0x36);
	cbus_write_reg(sii9244, 0x40, 0x03);
	cbus_write_reg(sii9244, 0x42, 0x06);
	cbus_write_reg(sii9244, 0x36, 0x0C);

	cbus_write_reg(sii9244, 0x3D, 0xFD);
	cbus_write_reg(sii9244, 0x1C, 0x00);

	cbus_write_reg(sii9244, 0x44, 0x02);

	/* Setup our devcap*/
	cbus_write_reg(sii9244, 0x80, 0x04);
	cbus_write_reg(sii9244, 0x81, 0x10);
	cbus_write_reg(sii9244, 0x82, 0x02);
	cbus_write_reg(sii9244, 0x83, 0);
	cbus_write_reg(sii9244, 0x84, 0);
	cbus_write_reg(sii9244, 0x85, 0x01 | 0x02);
	cbus_write_reg(sii9244, 0x86, 0x01);
	cbus_write_reg(sii9244, 0x87, 0);
	cbus_write_reg(sii9244, 0x88, (1<<2) | (1<<1) | (1<<3) | (1<<7));
	cbus_write_reg(sii9244, 0x89, 0x0F);
	cbus_write_reg(sii9244, 0x8A, (1<<0) | (1<<1) | (1<<2));
	cbus_write_reg(sii9244, 0x8B, 0);
	cbus_write_reg(sii9244, 0x8C, 0);
	cbus_write_reg(sii9244, 0x8D, 16);
	cbus_write_reg(sii9244, 0x8E, 0x44);
	cbus_write_reg(sii9244, 0x8F, 0);

	cbus_read_reg(sii9244, 0x31, &value);
	value |= 0x0C;
	cbus_write_reg(sii9244, 0x31, value);

	cbus_read_reg(sii9244, 0x22, &value);
	value &= 0x0F;
	cbus_write_reg(sii9244, 0x22, value);

	cbus_write_reg(sii9244, 0x30, 0x01);

	return 0;
}

static int sii9244_power_init(struct sii9244_data *sii9244)
{
	int ret;

	/* Force the sii9244 into the D0 state. */
	ret = tpi_write_reg(sii9244, TPI_DPD_REG, 0x3F);
	if (ret < 0)
		return ret;

	/* Enable TxPLL Clock */
	ret = hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS_CLK_EN_REG, 0x01);
	if (ret < 0)
		return ret;

	/* Enable Tx Clock Path & Equalizer*/
	ret = hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS_CH_EN_REG, 0x15);
	if (ret < 0)
		return ret;

	/* Power Up TMDS*/
	ret = mhl_tx_write_reg(sii9244, 0x08, 0x35);
	if (ret < 0)
		return ret;

	return 0;
}

static void sii9244_hdmi_init(struct sii9244_data *sii9244)
{
	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
	hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);

	/* PLL Calrefsel */
	hdmi_rx_write_reg(sii9244, HDMI_RX_PLL_CALREFSEL_REG, 0x03);

	/* VCO Cal */
	hdmi_rx_write_reg(sii9244, HDMI_RX_PLL_VCOCAL_REG, 0x20);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9244, HDMI_RX_EQ_DATA0_REG, 0x8A);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9244, HDMI_RX_EQ_DATA1_REG, 0x6A);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9244, HDMI_RX_EQ_DATA2_REG, 0xAA);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9244, HDMI_RX_EQ_DATA3_REG, 0xCA);

	/* Auto EQ */
	hdmi_rx_write_reg(sii9244, HDMI_RX_EQ_DATA4_REG, 0xEA);

	/* Manual zone */
	hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS_ZONE_CTRL_REG, 0xA0);

	/* PLL Mode Value */
	hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS_MODE_CTRL_REG, 0x00);

	mhl_tx_write_reg(sii9244, MHL_TX_TMDS_CCTRL, 0x34);

	hdmi_rx_write_reg(sii9244, 0x45, 0x44);

	/* Rx PLL BW ~ 4MHz */
	hdmi_rx_write_reg(sii9244, 0x31, 0x0A);

	/* Analog PLL Control
	 * bits 5:4 = 2b00 as per characterization team.
	 */
	hdmi_rx_write_reg(sii9244, HDMI_RX_TMDS0_CCTRL1_REG, 0xC1);
}

static void sii9244_mhl_tx_ctl_int(struct sii9244_data *sii9244)
{
	mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL1_REG, 0xD0);
	mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL2_REG, 0xFC);
	mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL4_REG, 0xEB);
	mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL7_REG, 0x0C);
}

static void sii9244_power(struct sii9244_data *sii9244, bool  on)
{
	int ret;
	
	if (on)
	{
		gpio_set_value(sii9244->pdata->enable_gpio, 1);
		ret = regulator_enable(sii9244->mhl_reg);
		if (ret)
		{
			dev_err(&sii9244->pdata->mhl_tx_client->dev,
				"failed to enable mhl regulator\n");
		}
		msleep(5);
		gpio_set_value(sii9244->pdata->reset_gpio, 1);
		msleep(10);
		gpio_set_value(sii9244->pdata->reset_gpio, 0);
		msleep(10);
		gpio_set_value(sii9244->pdata->reset_gpio, 1);
		msleep(10);
		sii9244->pdata->hpd_mux_pull_up(1);
	} 
	else
	{
		sii9244->pdata->hpd_mux_pull_up(0);
		if(regulator_is_enabled(sii9244->mhl_reg) > 0)
		{
			ret = regulator_disable(sii9244->mhl_reg);
			if (ret)
			{
				dev_err(&sii9244->pdata->mhl_tx_client->dev,
					"failed to diable mhl regulator\n");
			}
		}
		gpio_set_value(sii9244->pdata->enable_gpio, 0);
		gpio_set_value(sii9244->pdata->reset_gpio, 0);
	}
}
 
	
static void sii9244_enable(struct sii9244_data *sii9244, bool enable)
{
	if(enable)
		gpio_set_value(sii9244->pdata->sel_gpio, 1);
	else
		gpio_set_value(sii9244->pdata->sel_gpio, 0);
}

static void sii9234_vbus_present(bool on)
{
}

static void sii9244_power_down(struct sii9244_data *sii9244)
{
	if (sii9244->claimed)
		sii9234_vbus_present(false);

	sii9244->state = STATE_DISCONNECTED;
	sii9244->claimed = false;

	tpi_write_reg(sii9244, TPI_DPD_REG, 0);

	sii9244_power(sii9244, 0);
	sii9244_enable(sii9244, 0);
}

#if !defined(CONFIG_MUIC)
static int sii9244_detection_callback(struct otg_id_notifier_block *nb)
{
	struct sii9244_data *sii9244 = container_of(nb, struct sii9244_data,
						otg_id_nb);
	int ret;
	u8 value;
	int handled = OTG_ID_UNHANDLED;

	pr_debug("sii9244: detection started\n");

	mutex_lock(&sii9244->lock);
	sii9244->rgnd = RGND_UNKNOWN;
	sii9244->state = STATE_DISCONNECTED;
	sii9244->rsen = false;

	/* Set the board configuration so the  sii9244 has access to the
	 * external connector.
	 */
	sii9244->pdata->enable(1);
	sii9244->pdata->power(1);

	ret = sii9244_power_init(sii9244);
	if (ret < 0)
		goto unhandled;

	sii9244_hdmi_init(sii9244);

	sii9244_mhl_tx_ctl_int(sii9244);

	/* Enable HDCP Compliance safety*/
	ret = mhl_tx_write_reg(sii9244, 0x2B, 0x01);
	if (ret < 0)
		goto unhandled;

	/* CBUS discovery cycle time for each drive and float = 150us*/
	ret = mhl_tx_read_reg(sii9244, MHL_TX_DISC_CTRL1_REG, &value);
	if (ret < 0)
		goto unhandled;

	value &= ~(1<<2);
	value |= (1<<3);

	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL1_REG, value);
	if (ret < 0)
		goto unhandled;

	/* Clear bit 6 (reg_skip_rgnd) */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL2_REG,
			(1<<7) /* Reserved Bit */ |
			2 << ATT_THRESH_SHIFT |
			DEGLITCH_TIME_128MS);
	if (ret < 0)
		goto unhandled;

	/* Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel */
	/* 1.8V CBUS VTH & GND threshold */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL5_REG, 0x75);
	if (ret < 0)
		goto unhandled;

	/* set bit 2 and 3, which is Initiator Timeout */
	ret = cbus_read_reg(sii9244, CBUS_LINK_CONTROL_2_REG, &value);
	if (ret < 0)
		goto unhandled;

	value |= 0x0C;

	ret = cbus_write_reg(sii9244, CBUS_LINK_CONTROL_2_REG, value);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL6_REG, 0xA0);
	if (ret < 0)
		goto unhandled;

	/* RGND & single discovery attempt (RGND blocking) */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL6_REG, BLOCK_RGND_INT |
			DVRFLT_SEL | SINGLE_ATT);
	if (ret < 0)
		goto unhandled;

	/* Use VBUS path of discovery state machine*/
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL8_REG, 0);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	/* To allow RGND engine to operate correctly.
	 * When moving the chip from D2 to D0 (power up, init regs)
	 * the values should be
	 * 94[1:0] = 01  reg_cbusmhl_pup_sel[1:0] should be set for 5k
	 * 93[7:6] = 10  reg_cbusdisc_pup_sel[1:0] should be
	 * set for 10k (default)
	 * 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
	 */
	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL3_REG, 0x86);
	if (ret < 0)
		goto unhandled;

	/* change from CC to 8C to match 5K*/
	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL4_REG, 0x8C);
	if (ret < 0)
		goto unhandled;

	/* Configure the interrupt as active high */
	ret = mhl_tx_clear_reg(sii9244, MHL_TX_INT_CTRL_REG, (1<<2) | (1<<1));
	if (ret < 0)
		goto unhandled;

	msleep(25);

	ret = mhl_tx_clear_reg(sii9244, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL1_REG, 0x27);
	if (ret < 0)
		goto unhandled;

	/* Reset CBUS */
	ret = mhl_tx_set_reg(sii9244, 0x05, 0x03);
	if (ret < 0)
		goto unhandled;

	usleep_range(2000, 3000);

	ret = mhl_tx_clear_reg(sii9244, 0x05, 0x03);
	if (ret < 0)
		goto unhandled;

	/* Adjust interrupt mask everytime reset is performed.*/
	ret = cbus_write_reg(sii9244, 0x09, 0);
	if (ret < 0)
		goto unhandled;

	ret = cbus_write_reg(sii9244, 0x1F, 0);
	if (ret < 0)
		goto unhandled;

	/* Enable Auto soft reset on SCDT = 0*/
	ret = mhl_tx_write_reg(sii9244, 0x05, 0x04);

	if (ret < 0)
		goto unhandled;

	/* HDMI Transcode mode enable*/
	ret = mhl_tx_write_reg(sii9244, 0x0D, 0x1C);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_INTR4_ENABLE_REG,
			RGND_READY_MASK | CBUS_LKOUT_MASK |
			MHL_DISC_FAIL_MASK | MHL_EST_MASK);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_INTR1_ENABLE_REG,
			       (1<<5) | (1<<6));
	if (ret < 0)
		goto unhandled;

	pr_debug("sii9244: waiting for RGND measurement\n");
	enable_irq(sii9244->irq);

	/* SiI9244 Programmer's Reference Section 2.4.3
	 * State : RGND Ready
	 */
	mutex_unlock(&sii9244->lock);
	ret = wait_event_timeout(sii9244->wq,
				 ((sii9244->rgnd != RGND_UNKNOWN) ||
				  mhl_state_is_error(sii9244->state)),
				 msecs_to_jiffies(2000));

	mutex_lock(&sii9244->lock);
	if (sii9244->rgnd == RGND_UNKNOWN || mhl_state_is_error(sii9244->state))
		goto unhandled;

	if (sii9244->rgnd != RGND_1K)
		goto unhandled;

	mutex_unlock(&sii9244->lock);

	pr_debug("sii9244: waiting for detection\n");
	ret = wait_event_timeout(sii9244->wq,
				 sii9244->state != STATE_DISCONNECTED,
				 msecs_to_jiffies(500));
	mutex_lock(&sii9244->lock);
	if (sii9244->state == STATE_DISCONNECTED)
		goto unhandled;

	if (sii9244->state == STATE_DISCOVERY_FAILED) {
		handled = OTG_ID_PROXY_WAIT;
		goto unhandled;
	}

	if (mhl_state_is_error(sii9244->state))
		goto unhandled;

	mutex_unlock(&sii9244->lock);
	wait_event_timeout(sii9244->wq, sii9244->rsen, msecs_to_jiffies(400));
	mutex_lock(&sii9244->lock);
	if (!sii9244->rsen)
		goto unhandled;

	pr_info("si9234: connection established\n");
	sii9244->claimed = true;
	sii9234_vbus_present(true);
	mutex_unlock(&sii9244->lock);

	return OTG_ID_HANDLED;

unhandled:
	pr_info("sii9244: Detection failed");
	if (sii9244->state == STATE_DISCONNECTED)
		pr_cont(" (timeout)");
	else if (sii9244->state == STATE_DISCOVERY_FAILED)
		pr_cont(" (discovery failed)");
	else if (sii9244->state == STATE_CBUS_LOCKOUT)
		pr_cont(" (cbus_lockout)");
	pr_cont("\n");

	disable_irq_nosync(sii9244->irq);

	sii9244_power_down(sii9244);

	mutex_unlock(&sii9244->lock);
	return handled;
}

static void sii9244_cancel_callback(struct otg_id_notifier_block *nb)
{
	struct sii9244_data *sii9244 = container_of(nb, struct sii9244_data,
						otg_id_nb);

	mutex_lock(&sii9244->lock);
	sii9244_power_down(sii9244);
	mutex_unlock(&sii9244->lock);
}
#endif //!CONFIG_MUIC

static irqreturn_t sii9244_irq_thread(int irq, void *data)
{
	struct sii9244_data *sii9244 = data;
	int ret;
	u8 intr1, intr4, value;
	u8 intr1_en, intr4_en;
	bool release_otg = false;

	mutex_lock(&sii9244->lock);
	mhl_tx_read_reg(sii9244, MHL_TX_INTR1_REG, &intr1);
	mhl_tx_read_reg(sii9244, MHL_TX_INTR4_REG, &intr4);

	mhl_tx_read_reg(sii9244, MHL_TX_INTR1_ENABLE_REG, &intr1_en);
	mhl_tx_read_reg(sii9244, MHL_TX_INTR4_ENABLE_REG, &intr4_en);
	pr_debug("sii9244: irq %02x/%02x %02x/%02x\n", intr1, intr1_en,
		 intr4, intr4_en);

	if (intr4 & RGND_READY_INT) {
		ret = mhl_tx_read_reg(sii9244, MHL_TX_STAT2_REG, &value);
		if (ret < 0) {
			dev_err(&sii9244->pdata->mhl_tx_client->dev,
					"STAT2 reg, err %d\n", ret);
			goto err_exit;
		}

		switch (value & RGND_INTP_MASK) {
		case RGND_INTP_OPEN:
			pr_debug("RGND Open\n");
			sii9244->rgnd = RGND_OPEN;
			break;
		case RGND_INTP_1K:
			pr_debug("RGND 1K\n");
			ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL1_REG,
					0x25);

			ret = mhl_send_wake_pulses(sii9244);
			sii9244->rgnd = RGND_1K;
			break;
		case RGND_INTP_2K:
			pr_debug("RGND 2K\n");
			ret = mhl_send_wake_pulses(sii9244);
			sii9244->rgnd = RGND_2K;
			break;
		case RGND_INTP_SHORT:
			pr_debug("RGND Short\n");
			sii9244->rgnd = RGND_SHORT;
			break;
		};
	}

	if (intr4 & CBUS_LKOUT_INT) {
		pr_debug("sii9244: CBUS Lockout Interrupt\n");
		sii9244->state = STATE_CBUS_LOCKOUT;
	}

	if (intr4 & MHL_DISC_FAIL_INT)
		sii9244->state = STATE_DISCOVERY_FAILED;

	if (intr4 & MHL_EST_INT) {
		/* discovery override */
		ret = mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL1_REG, 0x10);

		/* increase DDC translation layer timer (byte mode) */
		cbus_write_reg(sii9244, 0x07, 0x32);
		cbus_set_reg(sii9244, 0x44, 1<<1);

		/* Keep the discovery enabled. Need RGND interrupt */
		ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL1_REG, (1<<0));

		sii9244->state = STATE_ESTABLISHED;
	}

	if (intr1 & HPD_CHANGE_INT) {
		ret = cbus_read_reg(sii9244, MSC_REQ_ABORT_REASON_REG, &value);

		if (value & SET_HPD_DOWNSTREAM) {
			/* Downstream HPD Highi */

			/* Do we need to send HPD upstream using
			 * Register 0x79(page0)? Is HPD need to be overriden??
			 *      TODO: See if we need code for overriding HPD OUT
			 *      as per Page 0,0x79 Register
			 */

			/* Enable TMDS */
			ret = mhl_tx_set_reg(sii9244, MHL_TX_TMDS_CCTRL,
					     (1<<4));
			pr_debug("sii9244: MHL HPD High, enabled TMDS\n");

			ret = mhl_tx_set_reg(sii9244, MHL_TX_INT_CTRL_REG,
					     (1<<4) | (1<<5));
		} else {
			/*Downstream HPD Low*/

			/* Similar to above comments.
			 * TODO:Do we need to override HPD OUT value
			 * and do we need to disable TMDS here?
			 */

			/* Disable TMDS */
			ret = mhl_tx_clear_reg(sii9244, MHL_TX_TMDS_CCTRL,
					       (1<<4));
			pr_debug("sii9244 MHL HPD low, disabled TMDS\n");
			ret = mhl_tx_clear_reg(sii9244, MHL_TX_INT_CTRL_REG,
					       (1<<4) | (1<<5));
		}
	}

	if (intr1 & RSEN_CHANGE_INT) {
		ret = mhl_tx_read_reg(sii9244, MHL_TX_SYSSTAT_REG, &value);

		sii9244->rsen = value & RSEN_STATUS;

		if (value & RSEN_STATUS) {
			pr_info("sii9244: MHL cable connected.. RESN High\n");
		} else {
			pr_info("sii9244: RSEN lost\n");
			/* Once RSEN loss is confirmed,we need to check
			 * based on cable status and chip power status,whether
			 * it is SINK Loss(HDMI cable not connected, TV Off)
			 * or MHL cable disconnection
			 * TODO: Define the below mhl_disconnection()
			 */
			/* mhl_disconnection(); */
			/* Notify Disconnection to OTG */
			if (sii9244->claimed == true) {
				disable_irq_nosync(sii9244->irq);
				release_otg = true;
			}
			sii9244_power_down(sii9244);
		}
	}

err_exit:
	mhl_tx_write_reg(sii9244, MHL_TX_INTR1_REG, intr1);
	mhl_tx_write_reg(sii9244, MHL_TX_INTR4_REG, intr4);

	mutex_unlock(&sii9244->lock);

	pr_debug("sii9244: wake_up\n");
	wake_up(&sii9244->wq);

	if (release_otg)
		otg_id_notify();

	return IRQ_HANDLED;
}

#if defined(CONFIG_MUIC)
int sii9244_cancel(struct muic_client_device *mcdev)
{
	struct sii9244_data *sii9244 =dev_get_drvdata(&mcdev->dev);

	mutex_lock(&sii9244->lock);
	sii9244_power_down(sii9244);
	mutex_unlock(&sii9244->lock);
	return 0;
}

int sii9244_detection(struct muic_client_device *mcdev)
{
	struct sii9244_data *sii9244 =dev_get_drvdata(&mcdev->dev);
	int ret;
	u8 value;

	pr_debug("sii9244: detection started\n");

	mutex_lock(&sii9244->lock);
	sii9244->rgnd = RGND_UNKNOWN;
	sii9244->state = STATE_DISCONNECTED;
	sii9244->rsen = false;

	sii9244_power(sii9244, 1);
	sii9244_enable(sii9244, 1);

	ret = sii9244_power_init(sii9244);
	if (ret < 0)
		goto unhandled;

	sii9244_hdmi_init(sii9244);

	sii9244_mhl_tx_ctl_int(sii9244);

	/* Enable HDCP Compliance safety*/
	ret = mhl_tx_write_reg(sii9244, 0x2B, 0x01);
	if (ret < 0)
		goto unhandled;

	/* CBUS discovery cycle time for each drive and float = 150us*/
	ret = mhl_tx_read_reg(sii9244, MHL_TX_DISC_CTRL1_REG, &value);
	if (ret < 0)
		goto unhandled;

	value &= ~(1<<2);
	value |= (1<<3);

	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL1_REG, value);
	if (ret < 0)
		goto unhandled;

	/* Clear bit 6 (reg_skip_rgnd) */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL2_REG,
			(1<<7) /* Reserved Bit */ |
			2 << ATT_THRESH_SHIFT |
			DEGLITCH_TIME_128MS);
	if (ret < 0)
		goto unhandled;

	/* Changed from 66 to 65 for 94[1:0] = 01 = 5k reg_cbusmhl_pup_sel */
	/* 1.8V CBUS VTH & GND threshold */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL5_REG, 0x75);
	if (ret < 0)
		goto unhandled;

	/* set bit 2 and 3, which is Initiator Timeout */
	ret = cbus_read_reg(sii9244, CBUS_LINK_CONTROL_2_REG, &value);
	if (ret < 0)
		goto unhandled;

	value |= 0x0C;

	ret = cbus_write_reg(sii9244, CBUS_LINK_CONTROL_2_REG, value);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_MHLTX_CTL6_REG, 0xA0);
	if (ret < 0)
		goto unhandled;

	/* RGND & single discovery attempt (RGND blocking) */
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL6_REG, BLOCK_RGND_INT |
			DVRFLT_SEL | SINGLE_ATT);
	if (ret < 0)
		goto unhandled;

	/* Use VBUS path of discovery state machine*/
	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL8_REG, 0);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	/* To allow RGND engine to operate correctly.
	 * When moving the chip from D2 to D0 (power up, init regs)
	 * the values should be
	 * 94[1:0] = 01  reg_cbusmhl_pup_sel[1:0] should be set for 5k
	 * 93[7:6] = 10  reg_cbusdisc_pup_sel[1:0] should be
	 * set for 10k (default)
	 * 93[5:4] = 00  reg_cbusidle_pup_sel[1:0] = open (default)
	 */
	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL3_REG, 0x86);
	if (ret < 0)
		goto unhandled;

	/* change from CC to 8C to match 5K*/
	ret = mhl_tx_set_reg(sii9244, MHL_TX_DISC_CTRL4_REG, 0x8C);
	if (ret < 0)
		goto unhandled;

	/* Configure the interrupt as active high */
	ret = mhl_tx_clear_reg(sii9244, MHL_TX_INT_CTRL_REG, (1<<2) | (1<<1));
	if (ret < 0)
		goto unhandled;

	msleep(25);

	ret = mhl_tx_clear_reg(sii9244, MHL_TX_DISC_CTRL6_REG, USB_ID_OVR);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_DISC_CTRL1_REG, 0x27);
	if (ret < 0)
		goto unhandled;

	/* Reset CBUS */
	ret = mhl_tx_set_reg(sii9244, 0x05, 0x03);
	if (ret < 0)
		goto unhandled;

	usleep_range(2000, 3000);

	ret = mhl_tx_clear_reg(sii9244, 0x05, 0x03);
	if (ret < 0)
		goto unhandled;

	/* Adjust interrupt mask everytime reset is performed.*/
	ret = cbus_write_reg(sii9244, 0x09, 0);
	if (ret < 0)
		goto unhandled;

	ret = cbus_write_reg(sii9244, 0x1F, 0);
	if (ret < 0)
		goto unhandled;

	/* Enable Auto soft reset on SCDT = 0*/
	ret = mhl_tx_write_reg(sii9244, 0x05, 0x04);

	if (ret < 0)
		goto unhandled;

	/* HDMI Transcode mode enable*/
	ret = mhl_tx_write_reg(sii9244, 0x0D, 0x1C);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_INTR4_ENABLE_REG,
			RGND_READY_MASK | CBUS_LKOUT_MASK |
			MHL_DISC_FAIL_MASK | MHL_EST_MASK);
	if (ret < 0)
		goto unhandled;

	ret = mhl_tx_write_reg(sii9244, MHL_TX_INTR1_ENABLE_REG,
				   (1<<5) | (1<<6));
	if (ret < 0)
		goto unhandled;

	pr_debug("sii9244: waiting for RGND measurement\n");
	enable_irq(sii9244->irq);

	/* SiI9244 Programmer's Reference Section 2.4.3
	 * State : RGND Ready
	 */
	mutex_unlock(&sii9244->lock);
	ret = wait_event_timeout(sii9244->wq,
				 ((sii9244->rgnd != RGND_UNKNOWN) ||
				  mhl_state_is_error(sii9244->state)),
				 msecs_to_jiffies(2000));

	mutex_lock(&sii9244->lock);
	if (sii9244->rgnd == RGND_UNKNOWN || mhl_state_is_error(sii9244->state))
		goto unhandled;

	if (sii9244->rgnd != RGND_1K)
		goto unhandled;

	mutex_unlock(&sii9244->lock);

	pr_debug("sii9244: waiting for detection\n");
	ret = wait_event_timeout(sii9244->wq,
				 sii9244->state != STATE_DISCONNECTED,
				 msecs_to_jiffies(500));
	mutex_lock(&sii9244->lock);
	if (sii9244->state == STATE_DISCONNECTED)
		goto unhandled;

	if (sii9244->state == STATE_DISCOVERY_FAILED) {
		goto unhandled;
	}

	if (mhl_state_is_error(sii9244->state))
		goto unhandled;

	mutex_unlock(&sii9244->lock);
	wait_event_timeout(sii9244->wq, sii9244->rsen, msecs_to_jiffies(400));
	mutex_lock(&sii9244->lock);
	if (!sii9244->rsen)
		goto unhandled;

	pr_info("sii9244: connection established\n");
	sii9244->claimed = true;
	sii9234_vbus_present(true);
	mutex_unlock(&sii9244->lock);

	return 0;

unhandled:
	pr_info("sii9244: Detection failed");
	if (sii9244->state == STATE_DISCONNECTED)
		pr_cont(" (timeout)");
	else if (sii9244->state == STATE_DISCOVERY_FAILED)
		pr_cont(" (discovery failed)");
	else if (sii9244->state == STATE_CBUS_LOCKOUT)
		pr_cont(" (cbus_lockout)");
	pr_cont("\n");

	disable_irq_nosync(sii9244->irq);

	sii9244_power_down(sii9244);

	mutex_unlock(&sii9244->lock);
	return -1;

}

static struct muic_client_ops mhl_ops = {
	.on_none = sii9244_cancel,
	.on_mhl = sii9244_detection,
};
#endif //CONFIG_MUIC

static int sii9244_gpio_config(struct sii9244_data *sii9244)
{
	int ret;
	
	if( sii9244->pdata->int_gpio < 0 ||
		sii9244->pdata->sel_gpio < 0 ||
		sii9244->pdata->enable_gpio < 0  ||
		sii9244->pdata->reset_gpio < 0 ||
		sii9244->pdata->wakeup_gpio < 0 )
	{
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Unable to get gpio\n");
		goto  err_gpio;
	}

	ret = gpio_request(sii9244->pdata->int_gpio, "MHL_INT");
	if (ret) {
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Fail to request mhl_int_gpio PIN %d.\n", sii9244->pdata->int_gpio);
		goto err_gpio;
	}
	gpio_direction_input(sii9244->pdata->int_gpio);

	ret = gpio_request(sii9244->pdata->sel_gpio, "MUIC/MHL SEL");
	if (ret) {
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Fail to request mhl_sel_gpio PIN %d.\n", sii9244->pdata->sel_gpio);
		goto err_gpio;
	}
	gpio_direction_output(sii9244->pdata->sel_gpio, 0);

	ret = gpio_request(sii9244->pdata->enable_gpio, "MHL_EN");
	if (ret) {
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Fail to request mhl_enable_gpio PIN %d.\n", sii9244->pdata->enable_gpio);
		goto err_gpio;
	}
	gpio_direction_output(sii9244->pdata->enable_gpio, 0);

	ret = gpio_request(sii9244->pdata->reset_gpio, "MHL_RESET_N");
	if (ret) {
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Fail to request mhl_reset_gpio PIN %d.\n", sii9244->pdata->reset_gpio);
		goto err_gpio;
	}
	gpio_direction_output(sii9244->pdata->reset_gpio, 0);

	ret = gpio_request(sii9244->pdata->wakeup_gpio, "WAKEUP_MHL");
	if (ret) {
		dev_err(&sii9244->pdata->mhl_tx_client->dev,
			"Fail to request mhl_wakeup_gpio PIN %d.\n", sii9244->pdata->wakeup_gpio);
		goto err_gpio;
	}
	gpio_direction_input(sii9244->pdata->wakeup_gpio); 

	return 0;
	
err_gpio:
	return -EINVAL;
}
static int __devinit sii9244_mhl_tx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sii9244_data *sii9244;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	sii9244 = kzalloc(sizeof(struct sii9244_data), GFP_KERNEL);
	if (!sii9244) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	sii9244->pdata = client->dev.platform_data;
	sii9244->pdata->mhl_tx_client = client;
	if (!sii9244->pdata) {
		ret = -EINVAL;
		goto err_exit1;
	}

	i2c_set_clientdata(client, sii9244);

	sii9244->irq = client->irq;

	init_waitqueue_head(&sii9244->wq);
	mutex_init(&sii9244->lock);

	if (sii9244->mhl_reg == NULL) {
		struct regulator *mhl_reg;
		
		mhl_reg = regulator_get(&client->dev, "mhl_1v8");
		if (IS_ERR(mhl_reg)) {
			dev_err(&client->dev, "can't get MHL 1.8V regulator\n");
			return PTR_ERR(mhl_reg);
		}
		sii9244->mhl_reg = mhl_reg;
	}
	
	ret = request_threaded_irq(client->irq, NULL, sii9244_irq_thread,
				   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				   "sii9244", sii9244);
	if (ret < 0)
		goto err_exit2;

	disable_irq(client->irq);

	/* GPIO configuration */
	sii9244_gpio_config(sii9244);

#if defined(CONFIG_MUIC) 
	/* Register detection/cancel func to MUIC driver */
	ret = muic_client_dev_register(client->name, sii9244, &mhl_ops);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register mhl_ops\n");
		goto err_exit2;
	}
#else
	sii9244->otg_id_nb.detect = sii9244_detection_callback;
	sii9244->otg_id_nb.cancel = sii9244_cancel_callback;
	sii9244->otg_id_nb.priority = sii9244->pdata->prio;

	plist_node_init(&sii9244->otg_id_nb.p, sii9244->pdata->prio);

	ret = otg_id_register_notifier(&sii9244->otg_id_nb);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register notifier\n");
		goto err_exit2;
	}
#endif

	return 0;

err_exit2:
err_exit1:
	kfree(sii9244);
	return ret;
}

static int __devinit sii9244_tpi_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9244_platform_data *pdata = client->dev.platform_data;
	pdata->tpi_client = client;
	return 0;
}

static int __devinit sii9244_hdmi_rx_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9244_platform_data *pdata = client->dev.platform_data;
	pdata->hdmi_rx_client = client;
	return 0;
}

static int __devinit sii9244_cbus_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sii9244_platform_data *pdata = client->dev.platform_data;
	pdata->cbus_client = client;
	return 0;
}

static int __devexit sii9244_mhl_tx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9244_tpi_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9244_hdmi_rx_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit sii9244_cbus_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sii9244_mhl_tx_id[] = {
	{"sii9244_mhl_tx", 0},
	{}
};

static const struct i2c_device_id sii9244_tpi_id[] = {
	{"sii9244_tpi", 0},
	{}
};

static const struct i2c_device_id sii9244_hdmi_rx_id[] = {
	{"sii9244_hdmi_rx", 0},
	{}
};

static const struct i2c_device_id sii9244_cbus_id[] = {
	{"sii9244_cbus", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sii9244_mhl_tx_id);
MODULE_DEVICE_TABLE(i2c, sii9244_tpi_id);
MODULE_DEVICE_TABLE(i2c, sii9244_hdmi_rx_id);
MODULE_DEVICE_TABLE(i2c, sii9244_cbus_id);

static struct i2c_driver sii9244_mhl_tx_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9244_mhl_tx",
	},
	.id_table = sii9244_mhl_tx_id,
	.probe = sii9244_mhl_tx_i2c_probe,
	.remove = __devexit_p(sii9244_mhl_tx_remove),
	.command = NULL,
};

static struct i2c_driver sii9244_tpi_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9244_tpi",
	},
	.id_table = sii9244_tpi_id,
	.probe = sii9244_tpi_i2c_probe,
	.remove = __devexit_p(sii9244_tpi_remove),
};

static struct i2c_driver sii9244_hdmi_rx_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244_hdmi_rx",
	},
	.id_table	= sii9244_hdmi_rx_id,
	.probe	= sii9244_hdmi_rx_i2c_probe,
	.remove	= __devexit_p(sii9244_hdmi_rx_remove),
};

static struct i2c_driver sii9244_cbus_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii9244_cbus",
	},
	.id_table = sii9244_cbus_id,
	.probe = sii9244_cbus_i2c_probe,
	.remove = __devexit_p(sii9244_cbus_remove),
};

static int __init sii9244_init(void)
{
	int ret;

	ret = i2c_add_driver(&sii9244_mhl_tx_i2c_driver);
	if (ret < 0)
		return ret;

	ret = i2c_add_driver(&sii9244_tpi_i2c_driver);
	if (ret < 0)
		goto err_exit1;

	ret = i2c_add_driver(&sii9244_hdmi_rx_i2c_driver);
	if (ret < 0)
		goto err_exit2;

	ret = i2c_add_driver(&sii9244_cbus_i2c_driver);
	if (ret < 0)
		goto err_exit3;

	return 0;

err_exit3:
	i2c_del_driver(&sii9244_hdmi_rx_i2c_driver);
err_exit2:
	i2c_del_driver(&sii9244_tpi_i2c_driver);
err_exit1:
	i2c_del_driver(&sii9244_mhl_tx_i2c_driver);
	return ret;
}

static void __exit sii9244_exit(void)
{
	i2c_del_driver(&sii9244_cbus_i2c_driver);
	i2c_del_driver(&sii9244_hdmi_rx_i2c_driver);
	i2c_del_driver(&sii9244_tpi_i2c_driver);
	i2c_del_driver(&sii9244_mhl_tx_i2c_driver);
}

module_init(sii9244_init);
module_exit(sii9244_exit);
