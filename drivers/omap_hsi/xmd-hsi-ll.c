/*
 * xmd-hsi-ll.c
 *
 * HSI Link Layer
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: ThippaReddy <thippareddy.dammur@intel.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/hsi_driver_if.h>
#include "xmd-hsi-ll-if.h"
#include "xmd-hsi-ll-cfg.h"
#include "xmd-hsi-ll-internal.h"
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE_COSMOPOLITAN)
#include "xmd-ch.h"
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

// mo2haewoon.you@lge.com [START]
#if defined(HSI_LL_WAKE_LOCK)
#include <linux/wakelock.h>
 
static struct wake_lock hsi_acwake_lock;
static struct wake_lock hsi_cawake_lock;
#endif
// mo2haewoon.you@lge.com [END]

#if !defined(HSI_LL_DATA_MUL_OF_16)

#define HSI_LL_GET_SIZE_IN_WORDS(size)                       \
	if (size > 4)                                            \
		size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2); \
	else                                                     \
		size = 1;

#define HSI_LL_GET_SIZE_IN_BYTES(size)                       \
	if (size > 4)                                            \
		size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2); \
	else                                                     \
		size = 1;                                            \
	size<<=2;

#else /* HSI_LL_DATA_MUL_OF_16 */

#define HSI_LL_GET_SIZE_IN_WORDS(size)                        \
	if (size > 16)                                            \
		size = (size & 0x0F) ? ((size >> 4) + 1):(size >> 4); \
	else                                                      \
		size = 1;                                             \
	size<<=2;

#define HSI_LL_GET_SIZE_IN_BYTES(size)                        \
	if (size > 16)                                            \
		size = (size & 0x0F) ? ((size >> 4) + 1):(size >> 4); \
	else                                                      \
		size = 1;                                             \
	size<<=4;

#endif

DEFINE_MUTEX(hsi_ll_write_mutex);
DEFINE_MUTEX(hsi_ll_open_mutex);
DEFINE_MUTEX(hsi_ll_close_mutex);
DEFINE_MUTEX(hsi_ll_psv);

// LGE_ADD_START 20120605 seunghwan.jin@lge.com
extern char simple_hsi_log_debug_enable;
// LGE_ADD_END 20120605 seunghwan.jin@lge.com

static struct hsi_ll_data_struct hsi_ll_data;
static struct hsi_ll_if_struct   hsi_ll_if;

#if defined (HSI_LL_ENABLE_TIMERS)
static struct hsi_ll_timer_Q     hsi_ll_time_queue[HSI_LL_TIMER_Q_SIZE];
#endif

static struct hsi_device_driver  hsi_ll_iface =
{
	.ctrl_mask  = ANY_HSI_CONTROLLER,
	.ch_mask[0] = 0,
	.probe      = hsi_ll_probe_cb,
	.remove     = hsi_ll_remove_cb,
	.driver     =
	{
		.name = "HSI_LINK_LAYER",
	},
};

static hsi_ll_notify hsi_ll_cb;

#if defined (HSI_LL_DATA_MUL_OF_16)
static unsigned int hsi_ll_rx_cmd_data[4] = {
	0x00000000,
	0xFFFFFFFF,
	0xFFFFFFFF,
	0xFFFFFFFF
};
static unsigned int hsi_ll_tx_cmd_data[4] = {
	0x00000000,
	0xFFFFFFFF,
	0xFFFFFFFF,
	0xFFFFFFFF
};
#endif

static int hsi_ll_send_cmd_queue(
	unsigned int command,
	unsigned int channel,
	unsigned int phy_id)
{
	if (hsi_ll_data.tx_cmd.count >= HSI_LL_MAX_CMD_Q_SIZE) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: TX CMD Queue Overflow. %s %d",__func__,__LINE__);
#endif
		return HSI_LL_RESULT_QUEUE_FULL;
	} else {
		hsi_ll_data.tx_cmd.count++;
	}

	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.write_index].command = command;
	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.write_index].channel = channel;
	hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.write_index].phy_id  = phy_id;

	hsi_ll_data.tx_cmd.write_index++;

	if (hsi_ll_data.tx_cmd.write_index >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.write_index = 0;
	}

	hsi_ll_if.msg_avaliable_flag = 1;
	wake_up_interruptible(&hsi_ll_if.msg_avaliable);

	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_read_cmd_queue(
	unsigned int *command,
	unsigned int *channel,
	unsigned int *phy_id)
{
	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	if (hsi_ll_data.tx_cmd.count > 0) {
		hsi_ll_data.tx_cmd.count--;
	} else {
		spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
		return HSI_LL_RESULT_QUEUE_EMPTY;
	}

	*command = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.read_index].command;
	*channel = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.read_index].channel;
	*phy_id  = hsi_ll_data.tx_cmd.cmd_q[hsi_ll_data.tx_cmd.read_index].phy_id;

	hsi_ll_data.tx_cmd.read_index++;

	if (hsi_ll_data.tx_cmd.read_index >= HSI_LL_MAX_CMD_Q_SIZE) {
		hsi_ll_data.tx_cmd.read_index = 0;
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_command_decode(
	unsigned int* message,
	unsigned int* ll_msg_type,
	unsigned int* channel,
	unsigned int* param)
{
	int ret = 0;
	unsigned int msg = *message;

	*ll_msg_type = ((msg & 0xF0000000) >> 28);

	switch(*ll_msg_type) {
	case HSI_LL_MSG_BREAK:
		*channel = HSI_LL_INVALID_CHANNEL;
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		char lcrCal, lcrAct;
		char val1,val2,val3;

		*channel = ((msg & 0x0F000000) >> 24);
		*param   = ((msg & 0x00FFFF00) >> 8 );

		val1 = ((msg & 0xFF000000) >> 24);
		val2 = ((msg & 0x00FF0000) >> 16);
		val3 = ((msg & 0x0000FF00) >>  8);

		lcrAct = (msg & 0x000000FF);
		lcrCal = val1 ^ val2 ^ val3;

		if (lcrCal != lcrAct)
			ret = -1;
		}
		break;
	case HSI_LL_MSG_CONN_READY:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_ACK:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_NAK:
		*channel = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_CONF_RATE:
		*channel = ((msg & 0x0F000000) >> 24);
		*param   = ((msg & 0x0F000000) >> 24);
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET:
		*channel = ((msg & 0x0F000000) >> 24);
		*param   = ((msg & 0x00F00000) >> 20) ? (msg & 0x000FFFFF) : (msg & 0x00FFFFFF);
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
		if (*channel == 0) {
			printk("\nHSI_LL: Unexpected case. Received CMD = 0x%x. %s %d\n",
					  msg, __func__, __LINE__);
		}
#endif
		break;
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		*ll_msg_type = HSI_LL_MSG_INVALID;
		*channel     = HSI_LL_INVALID_CHANNEL;
		ret = -1;
		break;
	}

	return ret;
}

static int hsi_ll_send_command(
	int cmd_type,
	unsigned int channel,
	void* arg,
	unsigned int phy_id)
{
	unsigned int command = 0;
	int ret = 0;

	spin_lock_bh(&hsi_ll_if.wr_cmd_lock);

	switch(cmd_type) {
	case HSI_LL_MSG_BREAK:
		command = 0;
		break;
	case HSI_LL_MSG_OPEN_CONN:{
		unsigned int size = *(unsigned int*)arg;
		unsigned int lcr  = 0;

		if (size > 4) {
			size = (size & 0x3) ? ((size >> 2) + 1):(size >> 2);
		} else {
			size = 1;
		}

		command = ((HSI_LL_MSG_OPEN_CONN & 0x0000000F) << 28) |
				  ((channel              & 0x0000000F) << 24) |
				  ((size                 & 0x0000FFFF) << 8);

		lcr = ((command & 0xFF000000) >> 24) ^
			  ((command & 0x00FF0000) >> 16) ^
			  ((command & 0x0000FF00) >>  8);

		command = command | (lcr & 0x000000FF);
		}
		break;
	case HSI_LL_MSG_CONN_READY:
		command = ((HSI_LL_MSG_CONN_READY & 0x0000000F) << 28) |
				  ((channel               & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONN_CLOSED:
		command = ((HSI_LL_MSG_CONN_CLOSED & 0x0000000F) << 28) |
				  ((channel                & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CANCEL_CONN: {
		unsigned int role = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_CANCEL_CONN & 0x0000000F) << 28) |
				  ((channel                & 0x0000000F) << 24) |
				  ((role                   & 0x000000FF) << 16);
		}
		break;
	case HSI_LL_MSG_ACK: {
		unsigned int echo_params = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_ACK & 0x0000000F) << 28) |
				  ((channel        & 0x0000000F) << 24) |
				  ((echo_params    & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_NAK:
		command = ((HSI_LL_MSG_NAK & 0x0000000F) << 28) |
				  ((channel        & 0x0000000F) << 24);
		break;
	case HSI_LL_MSG_CONF_RATE: {
		unsigned int baud_rate = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_CONF_RATE & 0x0000000F) << 28) |
				  ((channel              & 0x0000000F) << 24) |
				  ((baud_rate            & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		unsigned int size = *(unsigned int*)arg;

		command = ((HSI_LL_MSG_OPEN_CONN_OCTET & 0x0000000F) << 28) |
				  ((channel                    & 0x0000000F) << 24) |
				  ((size                       & 0x00FFFFFF));
		}
		break;
	case HSI_LL_MSG_ECHO:
	case HSI_LL_MSG_INFO_REQ:
	case HSI_LL_MSG_INFO:
	case HSI_LL_MSG_CONFIGURE:
	case HSI_LL_MSG_ALLOCATE_CH:
	case HSI_LL_MSG_RELEASE_CH:
	case HSI_LL_MSG_INVALID:
	default:
		ret = -1;
		break;
	}

	if (ret == 0) {
		ret = hsi_ll_send_cmd_queue(command, channel, phy_id);
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if(ret != 0) {
			printk("\nHSI_LL:hsi_ll_send_cmd_queue fail for Channel %d.%s %d\n",
					  channel, __func__, __LINE__);
		}
	} else {
		printk("\nHSI_LL:Invalid command issued for Channel %d. %s %d\n",
				  channel, __func__, __LINE__);
#endif
	}
	spin_unlock_bh(&hsi_ll_if.wr_cmd_lock);
	return ret;
}

/* Read callback for channel 1 to 15 */
static void hsi_ll_read_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.rd_cb_lock);
	/* Data Rx callback*/
	if (hsi_ll_data.ch[dev->n_ch].rx.state != HSI_LL_RX_STATE_RX) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: Error. Invalid state for channel %d. %s %d",
				 dev->n_ch, __func__, __LINE__);
#endif
		spin_unlock_bh(&hsi_ll_if.rd_cb_lock);
		return;
	} else {
		hsi_ll_stop_rx_timer(dev->n_ch);

		if (hsi_ll_data.ch[dev->n_ch].rx.close_req == TRUE) {
			unsigned int role = HSI_LL_ROLE_RECEIVER;
			struct hsi_ll_rx_tx_data temp;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL:Read close requested for ch %d. %s %d\n",
					 dev->n_ch,__func__, __LINE__);
#endif
			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								dev->n_ch,
								&role,
								HSI_LL_PHY_ID_RX);

			temp.buffer = hsi_ll_data.ch[dev->n_ch].rx.buffer;
			temp.size   = hsi_ll_data.ch[dev->n_ch].rx.size;

			hsi_ll_cb(dev->n_ch,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_FREE_MEM,
					  &temp);

			hsi_ll_data.ch[dev->n_ch].rx.buffer    = NULL;
			hsi_ll_data.ch[dev->n_ch].rx.close_req = FALSE;
			hsi_ll_data.ch[dev->n_ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
		} else {
			struct hsi_ll_rx_tx_data temp_data;

			temp_data.buffer = hsi_ll_data.ch[dev->n_ch].rx.buffer;
			temp_data.size   = hsi_ll_data.ch[dev->n_ch].rx.size;

			hsi_ll_cb(dev->n_ch,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_READ_COMPLETE,
					  &temp_data);

			hsi_ll_data.ch[dev->n_ch].rx.buffer = NULL;
			hsi_ll_send_command(HSI_LL_MSG_CONN_CLOSED,
								dev->n_ch,
								NULL,
								HSI_LL_PHY_ID_RX);
			hsi_ll_data.ch[dev->n_ch].rx.state = HSI_LL_RX_STATE_IDLE;
		}
	}
	spin_unlock_bh(&hsi_ll_if.rd_cb_lock);
}

/* Read callback for control channel */
static void hsi_ll_read_complete_cb(struct hsi_device *dev, unsigned int size)
{
	int ret;
	unsigned int channel = 0, param = 0, ll_msg_type = 0;
#ifdef CONFIG_MACH_LGE_U2
	unsigned int ipc_temp = 0; // ipc temp.
#endif

	spin_lock_bh(&hsi_ll_if.rd_cmd_cb_lock);

#if defined (HSI_LL_DATA_MUL_OF_16)
	hsi_ll_data.rx_cmd = hsi_ll_rx_cmd_data[0];
#endif
	ret = hsi_ll_command_decode(&hsi_ll_data.rx_cmd,
								&ll_msg_type,
								&channel,
								&param);

// LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
    if(simple_hsi_log_debug_enable == '1')
        printk("HSI_LL: channel %d CP => AP CMD = 0x%x.\n", channel, hsi_ll_data.rx_cmd);
#ifdef CONFIG_MACH_LGE_U2
    // start ipc temp.
    else
    {
        ipc_temp = hsi_ll_data.rx_cmd;
        ipc_temp = ipc_temp >> 28;
        /*if(ipc_temp == 9 && channel < 11)
            printk("HSI_LL: CP closed ch[%d] successfully.\n", channel);*/
    }
    // end ipc start
#endif
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
    if(simple_hsi_log_debug_enable != '1')
        printk("\nHSI_LL: channel %d CP => AP CMD = 0x%x.\n", channel, hsi_ll_data.rx_cmd);
#endif
/* Original code is blocked. Maintain and unblock this code when you remove updated area.
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
	printk("\nHSI_LL: channel %d CP => AP CMD = 0x%x.\n", channel, hsi_ll_data.rx_cmd);
#endif
*/
// LGE_UPDATE_END 20120605 seunghwan.jin@lge.com

	if (hsi_ll_if.rd_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.rd_complete_flag = 1;
		wake_up_interruptible(&hsi_ll_if.rd_complete);
	}

	if (ret != 0) {
		hsi_ll_send_command(HSI_LL_MSG_NAK, channel, NULL, HSI_LL_PHY_ID_RX);
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: Received Invalid MSG %d from CP for channel %d. %s %d\n",
				 ll_msg_type, channel, __func__, __LINE__);
#endif
		goto quit_rd_cmd_cb;
	}

	switch(ll_msg_type) {
	case HSI_LL_MSG_BREAK: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Break received.%s %d\n", __func__, __LINE__);
#endif

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 0
		/* Start RIL recovery */
		ifx_schedule_cp_dump_or_reset();
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if 0 /*TODO: Enable this after glitch issue is identified/resolved. */
		unsigned int i;
		for(i=0; i < HSI_LL_MAX_CHANNELS; i++) {
			if (hsi_ll_data.ch[i].tx.state != HSI_LL_TX_STATE_CLOSED) {
				hsi_ll_stop_channel(i);
				hsi_ll_stop_rx_timer(i);
				hsi_ll_stop_tx_timer(i);
				hsi_ll_data.ch[i].tx.state = HSI_LL_TX_STATE_CLOSED;
				/* TBD. send break to other side */
				hsi_ll_data.ch[i].rx.state = HSI_LL_RX_STATE_CLOSED;
			}
		}
		hsi_ll_cb(HSI_LL_CTRL_CHANNEL,
				  HSI_LL_RESULT_IO_ERROR,
				  HSI_LL_EV_RESET_MEM,
				  NULL);
#endif
		}
		break;
	case HSI_LL_MSG_ECHO: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Error.Not supported.%s %d\n", __func__, __LINE__);
#endif
		}
		break;
	case HSI_LL_MSG_OPEN_CONN: {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:HSI_LL_MSG_OPEN_CONN Not supported.%s %d.\n",
					__func__, __LINE__);
#endif
		}
		break;
	case HSI_LL_MSG_CONN_CLOSED: {
		switch(hsi_ll_data.ch[channel].tx.state) {
		case HSI_LL_TX_STATE_TX:
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_WAIT_FOR_TX_COMPLETE;
			break;
		case HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED: {
			struct hsi_ll_rx_tx_data temp;
			hsi_ll_stop_tx_timer(channel);
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
			temp.buffer = hsi_ll_data.ch[channel].tx.buffer;
			temp.size   = hsi_ll_data.ch[channel].tx.size;
			hsi_ll_data.ch[channel].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL: Data transfer over channel %d completed. %s %d\n",
					  channel, __func__, __LINE__);
#endif
			hsi_ll_cb(channel,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
			}
			break;
		default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
#endif
			break;
		}
		}
		break;
	case HSI_LL_MSG_CANCEL_CONN:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Invalid State. %s %d\n", __func__, __LINE__);
#endif
		break;
	case HSI_LL_MSG_ACK: {
		switch(hsi_ll_data.ch[channel].tx.state) {
		case HSI_LL_TX_STATE_OPEN_CONN: {
			int ret;
			unsigned int size = hsi_ll_data.ch[channel].tx.size;

			hsi_ll_stop_tx_timer(channel);
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TX;

			HSI_LL_GET_SIZE_IN_WORDS(size);

			ret = hsi_write(hsi_ll_data.dev[channel],
							hsi_ll_data.ch[channel].tx.buffer,
							size);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			if (ret!=0) {
				printk("\nHSI_LL:hsi_write failed for channel %d with error %d. %s %d\n",
						 channel, ret, __func__, __LINE__);
			} else {
				printk("\nHSI_LL: Transferring data over channel %d. %s %d\n",
						 channel, __func__, __LINE__);
			}
#endif
			/*TODO: Calculate actual time based on the transfer size*/
			hsi_ll_start_tx_timer(channel, 10);
		}
		break;
		case HSI_LL_TX_STATE_CLOSED: { /* ACK as response to CANCEL_CONN */
			if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK) {
				hsi_ll_stop_tx_timer(channel);
				hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_CLOSED;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			} else {
				printk("\nHSI_LL: Error. %s %d\n", __func__, __LINE__);
#endif
			}
			}
			break;
		/* ACK as response to CONF_RATE */
		case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK: {
			hsi_ll_stop_tx_timer(channel);
			/* TODO:Complete this */

			}
			break;
		default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Error.Invalid state. Current channel %d state %d. %s %d\n",
				channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			break;
			}
		}
		break;
	case HSI_LL_MSG_NAK:{
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:NAK received for channel %d state %d. retry %d %s %d\n",
				 channel, hsi_ll_data.ch[channel].tx.state,
				 hsi_ll_data.ch[channel].tx.retry,  __func__, __LINE__);
#endif
		switch(hsi_ll_data.ch[channel].tx.state) {
			case HSI_LL_TX_STATE_OPEN_CONN: {
				hsi_ll_stop_tx_timer(channel);
				hsi_ll_data.ch[channel].tx.retry++;

				if (hsi_ll_data.ch[channel].tx.retry == HSI_LL_MAX_OPEN_CONN_RETRIES) {
					struct hsi_ll_rx_tx_data temp;
					temp.buffer = hsi_ll_data.ch[channel].tx.buffer;
					temp.size   = hsi_ll_data.ch[channel].tx.size;
					hsi_ll_data.ch[channel].tx.state  = HSI_LL_TX_STATE_IDLE;
					hsi_ll_data.ch[channel].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
					printk("\nHSI_LL: %d retry attempts failed, droping packet for channel %d\n ",
						hsi_ll_data.ch[channel].tx.retry, channel);
#endif
					hsi_ll_cb(channel,
							  HSI_LL_RESULT_ERROR,
							  HSI_LL_EV_WRITE_COMPLETE,
							 &temp);
				} else {
#if !defined (HSI_LL_ENABLE_TX_RETRY_WQ)
					hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
										channel,
									   &hsi_ll_data.ch[channel].tx.size,
										HSI_LL_PHY_ID_TX);
#else
					/* In NAK case Modem needs some delay  */
					PREPARE_WORK(&hsi_ll_data.ch[channel].tx.retry_work,
								  hsi_ll_retry_work);
					queue_work(hsi_ll_if.hsi_tx_retry_wq,
							  &hsi_ll_data.ch[channel].tx.retry_work);
#endif
					}
				}
				break;
			/* NAK as response to CONF_RATE */
			case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK: {
				hsi_ll_stop_tx_timer(channel);
				/* TODO:Complete this */
				/* hsi_ll_data.tx_if.cfg.new_data_rate_valid = FALSE; */
				hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
				}
				break;
			default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
				printk("\nHSI_LL:Error. Invalid state. Current channel %d state %d. %s %d\n",
				channel, hsi_ll_data.ch[channel].tx.state, __func__, __LINE__);
#endif
				break;
		}
		}
		break;
	case HSI_LL_MSG_CONF_RATE:
		/* TODO: Complete this */
		break;
	case HSI_LL_MSG_OPEN_CONN_OCTET: {
		switch(hsi_ll_data.ch[channel].rx.state) {
			case HSI_LL_RX_STATE_IDLE: {
				struct hsi_ll_rx_tx_data temp_data;
				temp_data.buffer  = NULL;
				temp_data.size    = hsi_ll_data.ch[channel].rx.size = param;

				/*Make sure requested memory size is multiple of word as read
				  is always in multiple of words. */
				if(temp_data.size & 0x03) {
					temp_data.size += (4 - (temp_data.size & 0x03));
				}

				hsi_ll_cb(channel,
						  HSI_LL_RESULT_SUCCESS,
						  HSI_LL_EV_ALLOC_MEM,
						  &temp_data);

				if (temp_data.buffer == NULL) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
					printk("\nHSI_LL: Could not get mem (size=%d)for channel %d, so currently blocking this ch\n",
							temp_data.size,channel);
#endif
					hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_BLOCKED;
					goto quit_rd_cmd_cb;
#else
					hsi_ll_send_command(HSI_LL_MSG_NAK,
										channel,
										NULL,
										HSI_LL_PHY_ID_RX);
#endif
				} else {
					unsigned int size = hsi_ll_data.ch[channel].rx.size;
					unsigned int *buf = temp_data.buffer;
					hsi_ll_data.ch[channel].rx.buffer = temp_data.buffer;
					hsi_ll_send_command(HSI_LL_MSG_ACK,
										channel,
										&size,
										HSI_LL_PHY_ID_RX);
					HSI_LL_GET_SIZE_IN_WORDS(size);
					/* TODO: Calculate actual time based on the amount of transfer*/
					hsi_ll_start_rx_timer(channel, 20);
					hsi_read(hsi_ll_data.dev[channel], buf, size);
					hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_RX;
				}
			}
			break;
		case HSI_LL_RX_STATE_BLOCKED: {
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* P2 issue : CP Open Conn Retry Workaround  : Sending NAK stops retry open conn timer */
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Error: Resend open_conn from CP. Current channel %d state %d. %s %d\n",
					channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			hsi_ll_send_command(HSI_LL_MSG_NAK, channel, NULL, HSI_LL_PHY_ID_RX);
			hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_ACK_NACK_MS);
			//hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_BLOCKED; /* HSI_LL_RX_STATE_SEND_NACK; */
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
			}
			break;
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* P2 issue : CP Open Conn Retry Workaround  : Sending NAK stops retry open conn timer */
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
		case HSI_LL_RX_STATE_RX:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Error: Resend open_conn from CP. Current channel %d state %d. %s %d\n",
					channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			hsi_ll_send_command(HSI_LL_MSG_NAK, channel, NULL, HSI_LL_PHY_ID_RX);
			hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_ACK_NACK_MS);
			//hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_RX;
			break;			
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

         default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Error.Invalid state. Current channel %d state %d. %s %d\n",
				channel,hsi_ll_data.ch[channel].rx.state, __func__, __LINE__);
#endif
			break;
		}
		}
		break;
	default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Error.Invalid message encountered for channel %d. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		break;
	}
quit_rd_cmd_cb:
	spin_unlock_bh(&hsi_ll_if.rd_cmd_cb_lock);
}

/* TX Retry Work Queue */
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
static void hsi_ll_retry_work(struct work_struct *work)
{
	struct hsi_ll_tx_ch *tx_ch= (struct hsi_ll_tx_ch*) container_of(work, struct hsi_ll_tx_ch,retry_work);
	unsigned int channel = tx_ch->channel;

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL: hsi_ll_retry_work - HSI_LL_IF_STATE_ERR_RECOVERY.\n");
#endif
		return;
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: retrying Open_connect for ch %d. Size %d. Retry count %d\n",
			channel, hsi_ll_data.ch[channel].tx.size,
			hsi_ll_data.ch[channel].tx.retry);
#endif
	udelay(100); /*If required Fine tune this delay*/
	hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
						channel,
					   &hsi_ll_data.ch[channel].tx.size,
						HSI_LL_PHY_ID_TX);
	return;
}
#endif

/* Write callback for channels 1 to 15 */
static void hsi_ll_write_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.wr_cb_lock);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL:Write complete for channel %d. %s %d\n",
			  dev->n_ch, __func__, __LINE__);
#endif
	switch(hsi_ll_data.ch[dev->n_ch].tx.state) {
	case HSI_LL_TX_STATE_TX:
		hsi_ll_data.ch[dev->n_ch].tx.state = HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_TX_COMPLETE: {
		struct hsi_ll_rx_tx_data temp;

		hsi_ll_stop_tx_timer(dev->n_ch);
		hsi_ll_data.ch[dev->n_ch].tx.state = HSI_LL_TX_STATE_IDLE;
		temp.buffer = hsi_ll_data.ch[dev->n_ch].tx.buffer;
		temp.size   = hsi_ll_data.ch[dev->n_ch].tx.size;
		hsi_ll_data.ch[dev->n_ch].tx.buffer = NULL;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL: Data transfer over channel %d completed. %s %d\n",
				  dev->n_ch, __func__, __LINE__);
#endif
		hsi_ll_cb(dev->n_ch, HSI_LL_RESULT_SUCCESS,
				  HSI_LL_EV_WRITE_COMPLETE,
				 &temp);
		}
		break;
	}

	spin_unlock_bh(&hsi_ll_if.wr_cb_lock);
}

/* Write callback for control channel */
static void hsi_ll_write_complete_cb(struct hsi_device *dev, unsigned int size)
{
	spin_lock_bh(&hsi_ll_if.wr_cmd_cb_lock);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL:Write Complete callback.%s %d\n", __func__, __LINE__);
#endif

	if (hsi_ll_if.wr_complete_flag != 1) {
		/* Raise an event */
		hsi_ll_if.wr_complete_flag = 1;
		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_IDLE;
		wake_up_interruptible(&hsi_ll_if.wr_complete);
	}

	spin_unlock_bh(&hsi_ll_if.wr_cmd_cb_lock);
}

static int hsi_ll_create_rx_thread(void)
{
	hsi_ll_if.rd_th = kthread_run(hsi_ll_rd_ctrl_ch_th,
								  NULL,
								 "hsi_ll_rd_ctrlch");

	if (IS_ERR(hsi_ll_if.rd_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Cannot create read ctrl Thread.%s %d\n",
				  __func__, __LINE__);
#endif
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return -1;
	}

	return 0;
}

static int hsi_ll_config_bus(void)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	hsi_ll_data.rx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.rx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.rx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.rx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.rx_cfg.ctx.counters   = HSI_LL_COUNTERS_VALUE;
	hsi_ll_data.rx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_RX,
					&hsi_ll_data.rx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:RX config error %d. %s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return HSI_LL_RESULT_ERROR;
	}

	hsi_ll_data.tx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.tx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.tx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.tx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.tx_cfg.ctx.arb_mode   = HSI_LL_ARBMODE_MODE;
	hsi_ll_data.tx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_TX,
					&hsi_ll_data.tx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return HSI_LL_RESULT_ERROR;
	}
	return ret;
}

static int hsi_ll_wr_ctrl_ch_th(void *data)
{
	int ret, i;
	unsigned int command, channel;
	unsigned int phy_id;
#ifdef CONFIG_MACH_LGE_U2
	unsigned int ipc_temp = 0; // ipc temp.
#endif

	wait_event_interruptible(hsi_ll_if.reg_complete,
							 hsi_ll_if.reg_complete_flag == 1);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL:Write thread Started.%s %d\n", __func__, __LINE__);
#endif
	ret = hsi_ll_open(HSI_LL_CTRL_CHANNEL);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Error while opening CTRL channel %d.%s %d",
				  ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.initialized = FALSE;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;

		return -1;
	}

	for(i=1; i < HSI_LL_MAX_CHANNELS; i++) {
		/*Note: This is just a WA to receive packets on channels that
				are not opened by application, with this WA it is possible
				to receive and drop broadcast packets on channels
				close/not opened by application */
		ret = hsi_ll_open(i);
		if (ret != 0)
			printk("HSI_LL: Error opening Channel %d,err=%d", i, ret);
	}
#if 0
	hsi_ll_data.rx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.rx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.rx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.rx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.rx_cfg.ctx.counters   = HSI_LL_COUNTERS_VALUE;
	hsi_ll_data.rx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_RX,
					&hsi_ll_data.rx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:RX config error %d. %s %d\n",
				ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return -1;
	}

	hsi_ll_data.tx_cfg.ctx.mode       = HSI_LL_INTERFACE_MODE;
	hsi_ll_data.tx_cfg.ctx.flow       = HSI_LL_RECEIVER_MODE;
	hsi_ll_data.tx_cfg.ctx.frame_size = HSI_LL_MAX_FRAME_SIZE;
	hsi_ll_data.tx_cfg.ctx.divisor    = HSI_LL_DIVISOR_VALUE;
	hsi_ll_data.tx_cfg.ctx.arb_mode   = HSI_LL_ARBMODE_MODE;
	hsi_ll_data.tx_cfg.ctx.channels   = HSI_LL_MAX_CHANNELS;

	ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					HSI_IOCTL_SET_TX,
					&hsi_ll_data.tx_cfg.ctx);

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:TX config error %d.%s %d\n", ret, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_INIT_ERROR;
		hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
		return -1;
	}
#endif
	ret = hsi_ll_config_bus();

	if(ret) {
		return -1;
	}

	/* LGE_SJIT 2012-01-04 [dojip.kim@lge.com]
	 * OMAP could not enter sleep mode until RIL is initializeda.
	 * So change the HSI wake line status to resolve this problem.
	 * HSI_LL_WAKE_LINE_HIGH -> HSI_LL_WAKE_LINE_LOW
	 *
	 * This method is suggested by jaesung.woo@lge.com
	 */
	hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);

	ret = hsi_ll_create_rx_thread();

	if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: Failed to create RX thread. Initiating HSI Link Layer Shutdown. %s %d\n",
				  __func__, __LINE__);
#endif
		hsi_ll_shutdown();
		return -1;
	}

	hsi_ll_if.msg_avaliable_flag = 0;
	wait_event_interruptible(hsi_ll_if.msg_avaliable,
							 hsi_ll_if.msg_avaliable_flag == 1);
	while(1) {

		hsi_ll_if.msg_avaliable_flag = 0;

		while(HSI_LL_RESULT_QUEUE_EMPTY == hsi_ll_read_cmd_queue(&command,
																 &channel,
																 &phy_id)) {
#if defined (HSI_LL_ENABLE_PM)
// mo2haewoon.you@lge.com [START]
/* Wait for some time(3HZ) and starts to suspend */
#if defined (HSI_LL_WAKE_LOCK)
				if(wake_lock_active(&hsi_acwake_lock))
					wake_unlock(&hsi_acwake_lock);
				wake_lock_timeout(&hsi_acwake_lock, HSI_LL_AC_WAKE_TIMEOUT);
#endif
// mo2haewoon.you@lge.com [END]

			wait_event_interruptible_timeout(hsi_ll_if.msg_avaliable,
											 hsi_ll_if.msg_avaliable_flag == 1,
											 HSI_LL_PV_READ_CMD_Q_TIMEOUT);
			if (hsi_ll_if.msg_avaliable_flag == 0) {
				hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_ENABLE;
				wake_up_interruptible(&hsi_ll_if.psv_event);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
				printk("\nHSI_LL: PSV enable requested.\n");
#endif
#endif
				wait_event_interruptible(hsi_ll_if.msg_avaliable,
										 hsi_ll_if.msg_avaliable_flag == 1);
#if defined (HSI_LL_ENABLE_PM)
			}
			hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
			hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_UNDEF;
#endif
			hsi_ll_if.msg_avaliable_flag = 0;
		}

		if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL: Stop CMD transfer till recovery completes.\n");
#endif
			wait_event_interruptible(hsi_ll_if.wr_complete,
									hsi_ll_if.wr_complete_flag == 2);
			hsi_ll_if.wr_complete_flag = 0;
			printk("HSI_LL: HSI LL recovery completed .Start CMD transfer\n");
		}

		/* Wakeup Other Side */
		if (hsi_ll_data.tx_cfg.ac_wake == HSI_LL_WAKE_LINE_LOW) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL: Requesting AC wake line High.\n");
#endif

// mo2haewoon.you@lge.com [START]
#if defined (HSI_LL_WAKE_LOCK)
			if(wake_lock_active(&hsi_acwake_lock))
				wake_unlock(&hsi_acwake_lock);
			wake_lock(&hsi_acwake_lock);
#endif
// mo2haewoon.you@lge.com [END]
			hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_HIGH);
		}
// LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
        if (simple_hsi_log_debug_enable == '1')
            printk("HSI_LL: channel %d : AP => CP CMD = 0x%x \n", channel, command);
#ifdef CONFIG_MACH_LGE_U2
        // start ipc temp.
        else
        {
            ipc_temp = command;
            ipc_temp = ipc_temp >> 28;
            /*if(ipc_temp == 9 && channel < 11)
                printk("HSI_LL: AP closed ch[%d] successfully.\n", channel);*/
        }
        // end ipc temp
#endif
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
        if (simple_hsi_log_debug_enable != '1')
            printk("\nHSI_LL: channel %d : AP => CP CMD = 0x%x \n", channel, command);
#endif
/* Original code is blocked. Maintain and unblock this code when you remove updated area.
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
		printk("\nHSI_LL: channel %d : AP => CP CMD = 0x%x \n", channel, command);
#endif
*/
// LGE_UPDATE_END 20120605 seunghwan.jin@lge.com
		hsi_ll_data.tx_cmd.channel = channel;
		hsi_ll_data.tx_cmd.phy_id  = phy_id;

		hsi_ll_data.ch[HSI_LL_CTRL_CHANNEL].tx.state = HSI_LL_TX_STATE_TX;

#if !defined(HSI_LL_DATA_MUL_OF_16)
		ret = hsi_write(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						&command,
						1);
#else
		hsi_ll_tx_cmd_data[0] = command;
		ret = hsi_write(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						hsi_ll_tx_cmd_data,
						4);
#endif

#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if (ret != 0) {
			printk("\nHSI_LL: hsi_write(...) failed for ctrl chanel, err=%d. %s %d",
					 ret, __func__, __LINE__);
		}
#endif
		wait_event_interruptible(hsi_ll_if.wr_complete,
								 hsi_ll_if.wr_complete_flag == 1);
		hsi_ll_if.wr_complete_flag = 0;
	}

	return 0;
}

static int hsi_ll_rd_ctrl_ch_th(void *data)
{
	int ret;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL:Read thread Started.%s %d\n", __func__, __LINE__);
#endif

	while(1) {
		if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL: Stop CMD reception till recovery completes.\n");
#endif
			wait_event_interruptible(hsi_ll_if.rd_complete,
						 hsi_ll_if.rd_complete_flag == 2);
			hsi_ll_if.rd_complete_flag = 0;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("HSI_LL: HSI LL recovery completed .Start CMD reception\n");
#endif
		}
#if !defined(HSI_LL_DATA_MUL_OF_16)
		ret = hsi_read(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					  &hsi_ll_data.rx_cmd,
					  1);
#else
		ret = hsi_read(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
					   hsi_ll_rx_cmd_data,
					   4);
#endif
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		if (ret) {
			printk("\nHSI_LL: hsi_read(...) failed for ctrl chanel, err=%d. %s %d",
					  ret, __func__, __LINE__);
		}
#endif
		wait_event_interruptible(hsi_ll_if.rd_complete,
								 hsi_ll_if.rd_complete_flag == 1);
		hsi_ll_if.rd_complete_flag = 0;
	}

	return 0;
}

/**
 * hsi_ll_write - HSI LL Write
 * @channel: Channel Number.
 * @char: pointer to buffer.
 * @size: Number of bytes to be transferred.
 */
int hsi_ll_write(int channel, unsigned char *buf, unsigned int size)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	if ((channel <= 0) || (channel >= HSI_LL_MAX_CHANNELS)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: write attempted for invalid channel %d. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		return HSI_LL_RESULT_INVALID_PARAM;
	}

	mutex_lock(&hsi_ll_write_mutex);

	if (hsi_ll_data.ch[channel].open == FALSE) {
		ret = HSI_LL_RESULT_ERROR;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: write failed as channel %d is not opened. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		goto quit_write;
	}

	if ((buf == NULL) || (size == 0)) {
		ret = HSI_LL_RESULT_INVALID_PARAM;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: write failed for channel %d due to invalid params. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		goto quit_write;
	}

	hsi_ll_data.ch[channel].tx.buffer = buf;
	hsi_ll_data.ch[channel].tx.size   = size;

	if ((hsi_ll_data.ch[channel].tx.state == HSI_LL_TX_STATE_IDLE)  &&
		(hsi_ll_data.state				!= HSI_LL_IF_STATE_CONFIG)) {

		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_OPEN_CONN;
		hsi_ll_data.ch[channel].tx.retry = 0;
		ret = hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET,
								  channel,
								  &size,
								  HSI_LL_PHY_ID_TX);

		if (ret != HSI_LL_RESULT_SUCCESS) {
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL: write failed for channel %d. %s %d\n",
					  channel, __func__, __LINE__);
#endif
			goto quit_write;
		}

		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_OPEN_CONN_MAX_MS);
	} else {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: write failed .channel %d busy. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
		goto quit_write;
	}
quit_write:
	mutex_unlock(&hsi_ll_write_mutex);
	return ret;
}

#if defined (HSI_LL_ENABLE_TIMERS)
static void hsi_ll_stop_channel(unsigned int channel)
{
	return;
}
#endif

/**
 * hsi_ll_open - HSI channel open.
 * @channel: Channel number.
 */
int hsi_ll_open(int channel)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	mutex_lock(& hsi_ll_open_mutex);

	if (hsi_ll_data.ch[channel].open == TRUE) {
		goto quit_open;
	}

	if (hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY ||
		hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR   ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: Invalid state for channel %d. %s %d\n",
				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
		goto quit_open;
	}

	hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_IDLE;
	hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_IDLE;

	if (0 > hsi_open(hsi_ll_data.dev[channel])) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Could not open channel %d.%s %d\n",
				  channel, __func__, __LINE__);
#endif
		ret = HSI_LL_RESULT_ERROR;
	} else {
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_data.ch[channel].tx.channel = channel;
		INIT_WORK(&hsi_ll_data.ch[channel].tx.retry_work, hsi_ll_retry_work);
#endif
		hsi_ll_data.ch[channel].open = TRUE;
	}

quit_open:
	mutex_unlock(&hsi_ll_open_mutex);
	return ret;
}

/**
 * hsi_ll_close - HSI channel close.
 * @channel: Channel number.
 */
int hsi_ll_close(int channel)
{
	int ret = HSI_LL_RESULT_SUCCESS;

	mutex_lock(& hsi_ll_close_mutex);

	if (hsi_ll_data.state == HSI_LL_IF_STATE_PERM_ERROR ||
		hsi_ll_data.state == HSI_LL_IF_STATE_UN_INIT) {
		ret = HSI_LL_RESULT_ERROR;
		goto quit_close;
	}
#if 0
	/*NOTE: Below code block is commented as a WA. This is done to ensure that
			all hsi channels are kept open all the time so that
			broadcast messages from CP are not blocked */
	if ((hsi_ll_data.ch[channel].rx.state != HSI_LL_RX_STATE_IDLE)		||
		(hsi_ll_data.ch[channel].rx.state != HSI_LL_RX_STATE_POWER_DOWN)) {
		if (channel == 0) {
			/* wait until end of transmission */
			hsi_ll_data.ch[channel].rx.close_req = TRUE;
		} else if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_RX) {
			int role = HSI_LL_ROLE_RECEIVER;
			hsi_ll_send_command(HSI_LL_MSG_CANCEL_CONN,
								channel,
								&role,
								HSI_LL_PHY_ID_RX);
			hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_CANCEL_CONN_MS);
			hsi_ll_cb(channel,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_FREE_MEM,
					  hsi_ll_data.ch[channel].rx.buffer);
			hsi_ll_data.ch[channel].rx.buffer    = NULL;
			hsi_ll_data.ch[channel].rx.close_req = FALSE;
			hsi_ll_data.ch[channel].rx.state     = HSI_LL_RX_STATE_SEND_CONN_CANCEL;
		} else if (hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_ACK   ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_NACK         ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_CONN_READY   ||
			hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_SEND_CONN_CLOSED) {
			hsi_ll_data.ch[channel].rx.close_req = TRUE;
		}
	} else {
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_CLOSED;
	}
	/* Upper layers should take care that noting is
		sent on this channel till it is opened again*/
	hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_CLOSED;
	hsi_close(hsi_ll_data.dev[channel]);
	hsi_ll_data.ch[channel].open = FALSE;
#endif

quit_close:
	mutex_unlock(&hsi_ll_close_mutex);
	return HSI_LL_RESULT_SUCCESS;
}

static void hsi_ll_wakeup_cp(unsigned int val)
{
	int ret = -1;

	mutex_lock(&hsi_ll_psv);

	if (val == HSI_LL_WAKE_LINE_HIGH) {
		hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_HIGH;
		ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
						HSI_IOCTL_ACWAKE_UP,
						NULL);
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("HSI_LL:Setting AC wake line to HIGH.\n");
#endif
	} else {
		if(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_DISABLE) {
			printk("\nHSI_LL: PSV revoked%s %d\n",
					 __func__, __LINE__);
			ret = 0;
		} else {
			hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_LOW;
			ret = hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
							HSI_IOCTL_ACWAKE_DOWN,
							NULL);
			hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("HSI_LL:Setting AC wake line to LOW .\n");
#endif
		}
	}
#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (ret) {
		printk("\nHSI_LL:Error setting AC_WAKE line err=%d. %s %d\n",
				  ret, __func__, __LINE__);
	}
#endif
	mutex_unlock(&hsi_ll_psv);

	return;
}

/*
 * Processes wakeup
 */
#if defined (HSI_LL_ENABLE_PM)

#define HSI_LL_PSV_TRY_COUNT_DEBUG_PRINT 30

static int hsi_ll_psv_th(void *data)
{
	unsigned int psv_done = 0;
	unsigned int channel = 0;

#if defined (HSI_LL_ENABLE_ERROR_LOG)
	unsigned int try_count = 0;
#endif

	while(1) {
		wait_event_interruptible(hsi_ll_if.psv_event,
					(hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE));
		psv_done = 0;
#if defined (HSI_LL_ENABLE_ERROR_LOG)		
		try_count = 0;
#endif

		do{
			if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_DISABLE) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
				printk("\nHSI_LL:PSV_Enable Revoked.\n");
#endif
				break;
			}

// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 0 /* Check TX state only */
			for(channel = 0; channel < HSI_LL_MAX_CHANNELS; channel++) {
				if (hsi_ll_data.ch[channel].tx.state != HSI_LL_TX_STATE_IDLE) {
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
					printk("\nHSI_LL:PSV_Enable - Channel[%d] is busy with status %d, retry after a little sleep.\n",
							channel,hsi_ll_data.ch[channel].tx.state);
#endif
					msleep_interruptible(HSI_LL_PV_THREAD_SLEEP_TIME);
					break;
				}
			}
#else /* PSV mode enhancement : check TX & RX state */

			while(hsi_ll_data.ch[0].tx.state != HSI_LL_TX_STATE_IDLE) {
				printk("HSI_LL: Ch 0 busy sending DLP CMDs, retry after short delay");
				msleep_interruptible(HSI_LL_PV_THREAD_SLEEP_TIME);
			}

			for(channel = 1; channel < HSI_LL_MAX_CHANNELS; channel++) {
				if ((hsi_ll_data.ch[channel].tx.state != HSI_LL_TX_STATE_IDLE) ||
					(hsi_ll_data.ch[channel].rx.state != HSI_LL_RX_STATE_IDLE)){
#if defined (HSI_LL_ENABLE_ERROR_LOG)
					if((try_count != 0) && (try_count % HSI_LL_PSV_TRY_COUNT_DEBUG_PRINT == 0)) {
						printk("\nHSI_LL:PSV_Enable - Channel[%d] is busy with TX status %d and RX status %d and retry after a little sleep with try count %d th.\n",
							channel, hsi_ll_data.ch[channel].tx.state, hsi_ll_data.ch[channel].rx.state, try_count);
					}

					try_count++;
#endif
					msleep_interruptible(HSI_LL_PV_THREAD_SLEEP_TIME);
					break;
				}
			}
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

			if (channel >= HSI_LL_MAX_CHANNELS) {
				/* All are idle. Check if PSV can be Enabled */
				if (hsi_ll_if.psv_event_flag == HSI_LL_PSV_EVENT_PSV_ENABLE) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
					printk("\nHSI_LL: Requesting to set AC wake line to low.\n");
#endif
					hsi_ll_wakeup_cp(HSI_LL_WAKE_LINE_LOW);
					psv_done = 1;
				}
			}
		} while(!psv_done);
	}

	return 1;
}
#endif

#if defined (HSI_LL_ENABLE_TIMERS)
static void hsi_ll_timer_bh(unsigned long data)
{
	unsigned int channel = 0;

	spin_lock_bh(&hsi_ll_if.timer_bh_lock);

	while(hsi_ll_if.timer_queue_cnt > 0) {
		if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_cmd == HIS_LL_TIMER_QUEUE_CMD_START) {
			if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_dir == HIS_LL_TIMER_DIR_TX) {
				channel = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].channel;

				hsi_ll_data.ch[channel].tx.timer_id.data     = channel;
				hsi_ll_data.ch[channel].tx.timer_id.expires  = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].time_out;
				hsi_ll_data.ch[channel].tx.timer_id.function = hsi_ll_tx_timer_cb;

				add_timer(&hsi_ll_data.ch[channel].tx.timer_id);
		} else if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_dir == HIS_LL_TIMER_DIR_RX) {
			channel = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].channel;

			hsi_ll_data.ch[channel].rx.timer_id.data     = channel;
			hsi_ll_data.ch[channel].rx.timer_id.expires  = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].time_out;
			hsi_ll_data.ch[channel].rx.timer_id.function = hsi_ll_rx_timer_cb;

			add_timer(&hsi_ll_data.ch[channel].rx.timer_id);
		}
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		else
			printk("\nHSI_LL: State Mis-Match.%s %d\n", __func__, __LINE__);
#endif
		} else if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_cmd == HIS_LL_TIMER_QUEUE_CMD_STOP) {
			if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_dir == HIS_LL_TIMER_DIR_TX) {
				channel = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].channel;
				del_timer(&hsi_ll_data.ch[channel].tx.timer_id);
			} else if (hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].timer_dir == HIS_LL_TIMER_DIR_RX) {
				channel = hsi_ll_time_queue[hsi_ll_if.timer_queue_rd].channel;
				del_timer(&hsi_ll_data.ch[channel].rx.timer_id);
			}
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		else
			printk("\nHSI_LL: State Mis-Match.%s %d\n", __func__, __LINE__);
#endif
		}
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		else
			printk("\nHSI_LL: State Mis-Match.%s %d.\n", __func__, __LINE__);
#endif
		hsi_ll_if.timer_queue_rd++;

		if (hsi_ll_if.timer_queue_rd >= HSI_LL_TIMER_Q_SIZE)
			hsi_ll_if.timer_queue_rd = 0;

		hsi_ll_if.timer_queue_cnt--;
	}

	spin_unlock_bh(&hsi_ll_if.start_tx_timer_lock);
}
#endif

static void hsi_ll_start_tx_timer(unsigned int channel, unsigned int time_out)
{
#if defined (HSI_LL_ENABLE_TIMERS)
	spin_lock_bh(&hsi_ll_if.start_tx_timer_lock);

	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_cmd = HIS_LL_TIMER_QUEUE_CMD_START;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_dir = HIS_LL_TIMER_DIR_TX;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].channel   = channel;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].time_out  = time_out;

	hsi_ll_if.timer_queue_wr++;

	if (hsi_ll_if.timer_queue_wr >= HSI_LL_TIMER_Q_SIZE)
		hsi_ll_if.timer_queue_wr = 0;

	hsi_ll_if.timer_queue_cnt++;

#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (hsi_ll_if.timer_queue_cnt > HSI_LL_TIMER_Q_SIZE)
		printk("\nHSI_LL: Timer Queue Full.%s %d.", __func__, __LINE__);
#endif
	tasklet_schedule(&hsi_ll_if.timer_tasklet);

	spin_unlock_bh(&hsi_ll_if.start_tx_timer_lock);
#endif
}

static void hsi_ll_start_rx_timer(unsigned int channel, unsigned int time_out)
{
#if defined (HSI_LL_ENABLE_TIMERS)
	spin_lock_bh(&hsi_ll_if.start_rx_timer_lock);

	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_cmd = HIS_LL_TIMER_QUEUE_CMD_START;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_dir = HIS_LL_TIMER_DIR_RX;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].channel   = channel;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].time_out  = time_out;

	hsi_ll_if.timer_queue_wr++;

	if (hsi_ll_if.timer_queue_wr >= HSI_LL_TIMER_Q_SIZE)
		hsi_ll_if.timer_queue_wr = 0;

	hsi_ll_if.timer_queue_cnt++;

#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (hsi_ll_if.timer_queue_cnt > HSI_LL_TIMER_Q_SIZE)
		printk("\nHSI_LL: Timer Queue Full.%s %d", __func__, __LINE__);
#endif
	tasklet_schedule(&hsi_ll_if.timer_tasklet);

	spin_unlock_bh(&hsi_ll_if.start_rx_timer_lock);
#endif
}

static void hsi_ll_stop_tx_timer(unsigned int channel)
{
#if defined (HSI_LL_ENABLE_TIMERS)
	spin_lock_bh(&hsi_ll_if.stop_tx_timer_lock);
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_cmd = HIS_LL_TIMER_QUEUE_CMD_STOP;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_dir = HIS_LL_TIMER_DIR_TX;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].channel   = channel;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].time_out  = 0;
	hsi_ll_if.timer_queue_wr++;
	if (hsi_ll_if.timer_queue_wr >= HSI_LL_TIMER_Q_SIZE)
		hsi_ll_if.timer_queue_wr = 0;
	hsi_ll_if.timer_queue_cnt++;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (hsi_ll_if.timer_queue_cnt > HSI_LL_TIMER_Q_SIZE)
		printk("\nHSI_LL: Timer Queue Full.%s %d", __func__, __LINE__);
#endif
	tasklet_schedule(&hsi_ll_if.timer_tasklet);
	spin_unlock_bh(&hsi_ll_if.stop_tx_timer_lock);
#endif
}

static void hsi_ll_stop_rx_timer(unsigned int channel)
{
#if defined (HSI_LL_ENABLE_TIMERS)
	spin_lock_bh(&hsi_ll_if.stop_rx_timer_lock);
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_cmd = HIS_LL_TIMER_QUEUE_CMD_STOP;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].timer_dir = HIS_LL_TIMER_DIR_RX;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].channel   = channel;
	hsi_ll_time_queue[hsi_ll_if.timer_queue_wr].time_out  = 0;
	hsi_ll_if.timer_queue_wr++;
	if (hsi_ll_if.timer_queue_wr >= HSI_LL_TIMER_Q_SIZE)
		hsi_ll_if.timer_queue_wr = 0;
	hsi_ll_if.timer_queue_cnt++;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
	if (hsi_ll_if.timer_queue_cnt > HSI_LL_TIMER_Q_SIZE)
		printk("\nHSI_LL: Timer Queue Full.%s %d", __func__, __LINE__);
#endif
	tasklet_schedule(&hsi_ll_if.timer_tasklet);
	spin_unlock_bh(&hsi_ll_if.stop_rx_timer_lock);
#endif
}

/*
 * Invoked, when TX timer expires. Processes TX state machine
 */
#if defined (HSI_LL_ENABLE_TIMERS)
static void hsi_ll_tx_timer_cb(unsigned long channel)
{
	int ch_i;

	if (channel >= HSI_LL_MAX_CHANNELS) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Timer:Invalid channel id.%s %d\n", __func__, __LINE__);
#endif
		return;
	}

	spin_lock_bh(&hsi_ll_if.tx_timer_cb_lock);

	switch(hsi_ll_data.ch[channel].tx.state) {
	case HSI_LL_TX_STATE_OPEN_CONN:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_OPEN_CONN;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_ACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_ACK;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_CONN_READY:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_READY;
		break;
	case HSI_LL_TX_STATE_TX:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_TX;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_CLOSE;
		break;
	case HSI_LL_TX_STATE_SEND_CONF_RATE:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_CONF;
		break;
	case HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
		hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_TO_CONF_ACK;
		break;
	case HSI_LL_TX_STATE_NACK:
		if (hsi_ll_data.ch[channel].tx.retry > HSI_LL_MAX_ERROR_RETRY) {
			hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_TX);
			hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_BREAK_MAX_MS);
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_SEND_BREAK;
		} else {
			hsi_ll_data.ch[channel].tx.retry++;
			hsi_ll_send_command(HSI_LL_MSG_OPEN_CONN_OCTET, channel,
								&hsi_ll_data.ch[channel].tx.size, HSI_LL_PHY_ID_TX);
			hsi_ll_start_tx_timer(channel, HSI_LL_TX_T_OPEN_CONN_MAX_MS);
			hsi_ll_data.ch[channel].tx.state = HSI_LL_TX_STATE_OPEN_CONN;
			goto quit_tx_timer_cb;
		}
		break;
	case HSI_LL_TX_STATE_TO_OPEN_CONN:
	case HSI_LL_TX_STATE_TO_ACK:
	case HSI_LL_TX_STATE_TO_READY:
	case HSI_LL_TX_STATE_TO_TX:
	case HSI_LL_TX_STATE_TO_CLOSE:
	case HSI_LL_TX_STATE_TO_CONF:
	case HSI_LL_TX_STATE_TO_CONF_ACK:
	case HSI_LL_TX_STATE_SEND_BREAK:
		/* no TX possible; BREAK was not sent! -> L1 reset */
		/* reset HSI HW */
		break;
	default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Invalid state.%s %d", __func__, __LINE__);
#endif
		goto quit_tx_timer_cb;
	}
	/* TODO:Complete this */
	/* hsi_ll_set_if_state(IF_STATE_ERROR_RECOVERY); */
	/* Also for other channels */
	for(ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		if (hsi_ll_data.ch[ch_i].tx.state != HSI_LL_TX_STATE_CLOSED) {
			hsi_ll_stop_channel(ch_i);
		}
		if (ch_i != channel) {
			if (hsi_ll_data.ch[ch_i].tx.state != HSI_LL_TX_STATE_CLOSED) {
				hsi_ll_stop_tx_timer(ch_i);
				hsi_ll_data.ch[ch_i].tx.state = HSI_LL_TX_STATE_ERROR;
			}
			if (hsi_ll_data.ch[ch_i].rx.state != HSI_LL_RX_STATE_CLOSED) {
				hsi_ll_stop_rx_timer(ch_i);
				hsi_ll_data.ch[ch_i].rx.state = HSI_LL_RX_STATE_ERROR;
			}
		}
	}

quit_tx_timer_cb:
	spin_unlock_bh(&hsi_ll_if.tx_timer_cb_lock);
}
#endif

/*
 * Invoked, when RX timer expires. Processes RX state machine
 */
#if defined (HSI_LL_ENABLE_TIMERS)
static void hsi_ll_rx_timer_cb(unsigned long channel)
{
	int ch_i;

	if (channel >= HSI_LL_MAX_CHANNELS) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Timer: Invalid channel id.%s %d\n",
				__func__, __LINE__);
#endif
		return;
	}

	spin_lock_bh(&hsi_ll_if.rx_timer_cb_lock);

	switch(hsi_ll_data.ch[channel].rx.state) {
	case HSI_LL_RX_STATE_SEND_ACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_ACK;
		break;
	case HSI_LL_RX_STATE_SEND_NACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_NACK;
		break;
	case HSI_LL_RX_STATE_SEND_CONN_READY:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_CONN_READY;
		break;
	case HSI_LL_RX_STATE_RX:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_RX;
		break;
	case HSI_LL_RX_STATE_SEND_CONN_CLOSED:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_CONN_CLOSED;
		break;
	case HSI_LL_RX_STATE_SEND_CONN_CANCEL:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_CONN_CANCEL;
		break;
	case HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_CONN_CANCEL_ACK;
		break;
	case HSI_LL_RX_STATE_SEND_CONF_ACK:
	case HSI_LL_RX_STATE_SEND_CONF_NACK:
		hsi_ll_send_command(HSI_LL_MSG_BREAK, channel, NULL, HSI_LL_PHY_ID_RX);
		hsi_ll_start_rx_timer(channel, HSI_LL_RX_T_BREAK_MAX_MS);
		hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_TO_CONF_ACK;
		break;
	case HSI_LL_RX_STATE_TO_RX:
	case HSI_LL_RX_STATE_TO_ACK:
	case HSI_LL_RX_STATE_TO_CONN_READY:
	case HSI_LL_RX_STATE_TO_CONN_CLOSED:
	case HSI_LL_RX_STATE_TO_CONN_CANCEL:
	case HSI_LL_RX_STATE_TO_CONN_CANCEL_ACK:
	case HSI_LL_RX_STATE_TO_CONF_ACK:
	case HSI_LL_RX_STATE_TO_NACK:
	case HSI_LL_RX_STATE_SEND_BREAK:
		/* no TX possible; BREAK was not sent! -> L1 reset */
		/* Reset HSI HW */
		break;
	default:
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL: Invalid state.%s %d", __func__, __LINE__);
#endif
		break;
	}

	/* mipi_hsi_hw_set_if_state(IF_STATE_ERROR_RECOVERY); */

	/* Also for other channels */
	for (ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		if (hsi_ll_data.ch[ch_i].rx.state != HSI_LL_RX_STATE_CLOSED) {
			hsi_ll_stop_channel(ch_i);
		}
		if (ch_i != channel) {
			if (hsi_ll_data.ch[ch_i].rx.state != HSI_LL_RX_STATE_CLOSED) {
				hsi_ll_stop_rx_timer(ch_i);
				hsi_ll_data.ch[ch_i].rx.state = HSI_LL_RX_STATE_ERROR;
			}
			if (hsi_ll_data.ch[ch_i].tx.state != HSI_LL_TX_STATE_CLOSED) {
				hsi_ll_stop_tx_timer(ch_i);
				hsi_ll_data.ch[ch_i].tx.state = HSI_LL_TX_STATE_ERROR;
			}
		}
	}  /* next ch_i */

	spin_unlock_bh(&hsi_ll_if.rx_timer_cb_lock);
}
#endif

#if defined (HSI_LL_ENABLE_TIMERS)
static int hsi_ll_timer_init(void)
{
	int ch_i;

	for(ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		init_timer(&hsi_ll_data.ch[ch_i].tx.timer_id);
		init_timer(&hsi_ll_data.ch[ch_i].rx.timer_id);
	}
	tasklet_init(&hsi_ll_if.timer_tasklet, hsi_ll_timer_bh, ch_i);
	return HSI_LL_RESULT_SUCCESS;
}
#endif

static int hsi_ll_events_init(void)
{
	init_waitqueue_head(&hsi_ll_if.wr_complete);
	init_waitqueue_head(&hsi_ll_if.rd_complete);
	init_waitqueue_head(&hsi_ll_if.msg_avaliable);
	init_waitqueue_head(&hsi_ll_if.reg_complete);
#if defined (HSI_LL_ENABLE_PM)
	init_waitqueue_head(&hsi_ll_if.psv_event);
#endif

	/* Initialize spin_lock */
	spin_lock_init(&hsi_ll_if.phy_cb_lock);

#if defined (HSI_LL_ENABLE_TIMERS)
	spin_lock_init(&hsi_ll_if.start_tx_timer_lock);
	spin_lock_init(&hsi_ll_if.start_rx_timer_lock);
	spin_lock_init(&hsi_ll_if.stop_tx_timer_lock);
	spin_lock_init(&hsi_ll_if.stop_rx_timer_lock);
	spin_lock_init(&hsi_ll_if.tx_timer_cb_lock);
	spin_lock_init(&hsi_ll_if.rx_timer_cb_lock);
	spin_lock_init(&hsi_ll_if.timer_bh_lock);
#endif
	spin_lock_init(&hsi_ll_if.wr_cmd_cb_lock);
	spin_lock_init(&hsi_ll_if.rd_cmd_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cb_lock);
	spin_lock_init(&hsi_ll_if.rd_cb_lock);
	spin_lock_init(&hsi_ll_if.wr_cmd_lock);

	return HSI_LL_RESULT_SUCCESS;
}

static int hsi_ll_probe_cb(struct hsi_device *dev)
{
	int port;

	for (port = 0; port < HSI_MAX_PORTS; port++) {
		if (hsi_ll_iface.ch_mask[port])
			break;
	}

	if (port == HSI_MAX_PORTS)
		return -ENXIO;

	if (dev->n_ch >= HSI_LL_MAX_CHANNELS) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Invalid channel %d. %s %d.\n",
					dev->n_ch, __func__, __LINE__);
#endif
		return -ENXIO;
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: probe Channel ID = %d , Dev[%d] = 0x%x . %s %d.\n",
				dev->n_ch, dev->n_ch, (int)dev, __func__, __LINE__);
#endif

	spin_lock_bh(&hsi_ll_if.phy_cb_lock);

	if (dev->n_ch == 0) {
		hsi_set_read_cb(dev, hsi_ll_read_complete_cb);
		hsi_set_write_cb(dev, hsi_ll_write_complete_cb);
	} else {
		hsi_set_read_cb(dev, hsi_ll_read_cb);
		hsi_set_write_cb(dev, hsi_ll_write_cb);
	}

	hsi_set_port_event_cb(dev, hsi_ll_port_event_cb);

	hsi_ll_data.dev[dev->n_ch] = dev;
	hsi_ll_data.ch[dev->n_ch].tx.state     = HSI_LL_TX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.state     = HSI_LL_RX_STATE_CLOSED;
	hsi_ll_data.ch[dev->n_ch].rx.close_req = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.pending   = FALSE;
	hsi_ll_data.ch[dev->n_ch].tx.data_rate = 0;
	hsi_ll_data.ch[dev->n_ch].tx.buffer    = NULL;
	hsi_ll_data.ch[dev->n_ch].rx.buffer    = NULL;

	hsi_ll_if.reg_complete_ch_count++;

	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	if (hsi_ll_if.reg_complete_ch_count == HSI_LL_MAX_CHANNELS) {
		hsi_ll_if.reg_complete_flag = 1;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Registration for %d channels completed.%s %d.\n",
				 (unsigned int)HSI_LL_MAX_CHANNELS, __func__, __LINE__);
#endif
		wake_up_interruptible(&hsi_ll_if.reg_complete);
	}

	return 0;
}

static int hsi_ll_remove_cb(struct hsi_device *dev)
{
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Invoked %s.\n",__func__);
#endif
	spin_lock_bh(&hsi_ll_if.phy_cb_lock);
	hsi_set_read_cb(dev, NULL);
	hsi_set_write_cb(dev, NULL);
	spin_unlock_bh(&hsi_ll_if.phy_cb_lock);

	return 0;
}

static void hsi_ll_port_event_cb(
	struct hsi_device *dev,
	unsigned int event,
	void *arg)
{
	switch(event) {
	case HSI_EVENT_BREAK_DETECTED:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Break Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_ERROR:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Error Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_PRE_SPEED_CHANGE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Pre Speed changer Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_POST_SPEED_CHANGE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Post Speed changer Event detected.%s %d\n",
				 __func__, __LINE__);
#endif
		break;
	case HSI_EVENT_CAWAKE_UP:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:CA wakeup line UP detected.%s %d\n",
				  __func__, __LINE__);
#endif
// mo2haewoon.you@lge.com [START]
#if defined (HSI_LL_WAKE_LOCK)
		if(dev->n_ch == 0) {
                        printk("[%s]UP:CA wakeup line UP detected. all channel\n",  __func__);
			if(wake_lock_active(&hsi_cawake_lock))
				wake_unlock(&hsi_cawake_lock);
			wake_lock(&hsi_cawake_lock);
		}
#endif
// mo2haewoon.you@lge.com [END]
		break;
	case HSI_EVENT_CAWAKE_DOWN:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:CA wakeup line DOWN detected.%s %d\n",
				  __func__, __LINE__);
#endif
// mo2haewoon.you@lge.com [START]
/* Wait for some time(3HZ) and starts to suspend */
#if defined (HSI_LL_WAKE_LOCK)
		if(dev->n_ch == 0) {
                        printk("[%s]DOWN:CA wakeup line DOWN detected. all channel\n\n",  __func__); 
			wake_unlock(&hsi_cawake_lock);
			wake_lock_timeout(&hsi_cawake_lock, HSI_LL_CA_WAKE_TIMEOUT);
		}
#endif
// mo2haewoon.you@lge.com [END]
		break;
	case HSI_EVENT_HSR_DATAAVAILABLE:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:HSR Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	default:
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:Invalid Event detected.%s %d\n",
				  __func__, __LINE__);
#endif
		break;
	}
}


/**
 * hsi_ll_init - Initilizes all resources and registers with HSI PHY driver.
 * @port: Number of ports.
 * @cb: pointer to callback.
 */
int hsi_ll_init(int port, const hsi_ll_notify cb)
{
	int i;
	int ret = HSI_LL_RESULT_SUCCESS;

	if (!hsi_ll_data.initialized) {
		if ((port <= 0)			   ||
			(port > HSI_MAX_PORTS) ||
			(cb == NULL)) {
			ret = HSI_LL_RESULT_INVALID_PARAM;
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Invalid port or callback pointer. %s %d\n",
					__func__, __LINE__);
#endif
			goto quit_init;
		}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* TI HSI driver (from HSI_DRIVER_VERSION 0.4.2) can suppport port 1 and 2, 
	but IMC XMD currently supports port 1 only */
#if 1
		for (i = 0; i < HSI_MAX_PORTS; i++)
			hsi_ll_iface.ch_mask[i] = 0;
		
		for (i = 0; i < port; i++) {
			hsi_ll_iface.ch_mask[i] = (0xFFFFFFFF ^ (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));
		}
#else
		port-=1;

		for (i = port; i < HSI_MAX_PORTS; i++) {
			hsi_ll_iface.ch_mask[i] = (0xFFFFFFFF ^ (0xFFFFFFFF << HSI_LL_MAX_CHANNELS));
		}
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

		if (HSI_LL_RESULT_SUCCESS != hsi_ll_events_init()) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Cannot create Events. %s %d\n",
					__func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

		hsi_ll_cb = cb;
		hsi_ll_if.reg_complete_flag = 0;

		ret = hsi_register_driver(&hsi_ll_iface);

		if (ret) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Error while registering with HSI PHY driver err=%d. %s %d\n",
					ret, __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_INIT_ERROR;
			goto quit_init;
		}

		hsi_ll_data.state = HSI_LL_IF_STATE_READY;

#if defined (HSI_LL_ENABLE_TIMERS)
		if (HSI_LL_RESULT_SUCCESS != hsi_ll_timer_init()) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Cannot create Timers. %s %d\n",
					__func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}
#endif

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:creating threads. %s %d.\n", __func__, __LINE__);
#endif
		hsi_ll_if.wr_th = kthread_run(hsi_ll_wr_ctrl_ch_th,
									  NULL,
									 "hsi_ll_wr_ctrlch");

		if (IS_ERR(hsi_ll_if.wr_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Cannot create write ctrl Thread. %s %d\n",
					  __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

#if defined (HSI_LL_ENABLE_PM)
		hsi_ll_if.psv_th = kthread_run(hsi_ll_psv_th,
									   NULL,
									  "hsi_ll_psv_thread");

		if (IS_ERR(hsi_ll_if.psv_th)) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
			printk("\nHSI_LL:Cannot create PSV Thread. %s %d\n",
					  __func__, __LINE__);
#endif
			ret = HSI_LL_RESULT_ERROR;
			hsi_ll_data.state = HSI_LL_IF_STATE_UN_INIT;
			goto quit_init;
		}

// mo2haewoon.you@lge.com [START]
#if defined (HSI_LL_WAKE_LOCK)
		wake_lock_init(&hsi_acwake_lock, WAKE_LOCK_SUSPEND, "hsi-acwake-lock");
		wake_lock_init(&hsi_cawake_lock, WAKE_LOCK_SUSPEND, "hsi-cawake-lock");
#endif
// mo2haewoon.you@lge.com [END]

#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		hsi_ll_if.hsi_tx_retry_wq = create_workqueue("hsi_tx_retry_wq");
#endif
		hsi_ll_data.state = HSI_LL_IF_STATE_READY;
	}

quit_init:
	return ret;
}

/**
 * hsi_ll_shutdown - Releases all resources.
 */
int hsi_ll_shutdown(void)
{
#if defined (HSI_LL_ENABLE_TIMERS)
	int ch_i;
#endif
	if (hsi_ll_data.initialized) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("\nHSI_LL:HSI Link Layer Shutdown initiated. %s %d\n",
				  __func__, __LINE__);
#endif
		hsi_ll_close(HSI_LL_CTRL_CHANNEL);
		hsi_ioctl(hsi_ll_data.dev[HSI_LL_CTRL_CHANNEL],
				  HSI_IOCTL_ACWAKE_DOWN,
				  NULL);
		kthread_stop(hsi_ll_if.rd_th);
		kthread_stop(hsi_ll_if.wr_th);
#if defined (HSI_LL_ENABLE_PM)

// mo2haewoon.you@lge.com [START]
#if defined (HSI_LL_WAKE_LOCK)
		wake_lock_destroy(&hsi_acwake_lock);
		wake_lock_destroy(&hsi_cawake_lock);
#endif
// mo2haewoon.you@lge.com [END]

		kthread_stop(hsi_ll_if.psv_th);
#endif
#if defined (HSI_LL_ENABLE_TIMERS)
		for(ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
			del_timer(&hsi_ll_data.ch[ch_i].tx.timer_id);
			del_timer(&hsi_ll_data.ch[ch_i].rx.timer_id);
		}
		tasklet_kill(&hsi_ll_if.timer_tasklet);
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
		destroy_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif
		hsi_unregister_driver(&hsi_ll_iface);
		hsi_ll_data.initialized = FALSE;
	}
	return HSI_LL_RESULT_SUCCESS;
}

/* Resets DLP */
int hsi_ll_reset(void)
{
	int ch_i = 0;
	struct hsi_ll_rx_tx_data temp;

	hsi_ll_data.initialized = FALSE;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
	printk("\nHSI_LL: DLP recovery started.\n");
#endif
	hsi_ll_data.state = HSI_LL_IF_STATE_ERR_RECOVERY;
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	flush_workqueue(hsi_ll_if.hsi_tx_retry_wq);
#endif
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Requesting read/write cancel.\n");
#endif
	for(ch_i = 0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		hsi_ll_data.ch[ch_i].open = FALSE;
		hsi_write_cancel(hsi_ll_data.dev[ch_i]);
		hsi_read_cancel(hsi_ll_data.dev[ch_i]);
	}

	hsi_ll_if.wr_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	hsi_ll_if.rd_complete_flag = 1;
	wake_up_interruptible(&hsi_ll_if.rd_complete);

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Unblocking all read/write callbacks.\n");
#endif

	for(ch_i = 1; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
		if (hsi_ll_data.ch[ch_i].tx.buffer != NULL) {
			temp.buffer = hsi_ll_data.ch[ch_i].tx.buffer;
			temp.size   = hsi_ll_data.ch[ch_i].tx.size;
			hsi_ll_cb(ch_i,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_WRITE_COMPLETE,
					  &temp);
		}
		if (hsi_ll_data.ch[ch_i].rx.buffer != NULL) {
			temp.buffer = hsi_ll_data.ch[ch_i].rx.buffer;
			temp.size   = hsi_ll_data.ch[ch_i].rx.size;
			hsi_ll_cb(ch_i,
					  HSI_LL_RESULT_SUCCESS,
					  HSI_LL_EV_READ_COMPLETE,
					  &temp);
		}
		hsi_ll_data.ch[ch_i].tx.state = HSI_LL_TX_STATE_IDLE;
		hsi_ll_data.ch[ch_i].rx.state = HSI_LL_RX_STATE_IDLE;
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 0
		hsi_ll_data.ch[ch_i].open = TRUE;
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Resetting HSI PHY Driver.\n");
#endif
	if(0 > hsi_ioctl(hsi_ll_data.dev[0], HSI_IOCTL_SW_RESET, NULL)) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("HSI_LL: HSI PHY rest returned error.\n");
#endif
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Revoking PSV enable.\n");
#endif

#if defined (HSI_LL_ENABLE_PM)
// mo2haewoon.you@lge.com [START]
#if defined (HSI_LL_WAKE_LOCK)
	wake_unlock(&hsi_acwake_lock);
	wake_unlock(&hsi_cawake_lock);
#endif
// mo2haewoon.you@lge.com [END]

	hsi_ll_if.psv_event_flag = HSI_LL_PSV_EVENT_PSV_DISABLE;

	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
	hsi_ll_data.tx_cfg.ac_wake = HSI_LL_WAKE_LINE_LOW;
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
#endif

	hsi_ll_data.tx_cmd.count	   = 0;
	hsi_ll_data.tx_cmd.write_index = 0;
	hsi_ll_data.tx_cmd.read_index  = 0;
	hsi_ll_if.msg_avaliable_flag   = 0;

	for(ch_i=0; ch_i < HSI_LL_MAX_CHANNELS; ch_i++) {
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 1
		if (0 > hsi_open(hsi_ll_data.dev[ch_i])) {
#if defined (HSI_LL_ENABLE_ERROR_LOG)
		printk("\nHSI_LL:Could not open channel %d.%s %d\n",
				  ch_i, __func__, __LINE__);
#endif
		}
		else {
			hsi_ll_data.ch[ch_i].open = TRUE;
		}
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
	}

#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Reconfiguring HSI Buses.\n");
#endif
	if(0 > hsi_ll_config_bus()) {
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
		printk("HSI_LL: HSI bus reconfiguration failed. Recovery failed.\n");
#endif
		return -1;
	}
	hsi_ll_data.state = HSI_LL_IF_STATE_READY;
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
	printk("\nHSI_LL: Resuming read/write threads.\n");
#endif
	hsi_ll_if.wr_complete_flag = 2;
	hsi_ll_if.rd_complete_flag = 2;
	wake_up_interruptible(&hsi_ll_if.wr_complete);
	wake_up_interruptible(&hsi_ll_if.rd_complete);
	hsi_ll_data.initialized = TRUE;
#if defined (HSI_LL_ENABLE_CRITICAL_LOG)
	printk("\nHSI_LL: DLP recovery completed.\n");
#endif

	return 0;
}

/**
 * hsi_ll_ioctl - HSI LL IOCTL
 * @channel: Channel Number.
 * @command: command to execute
 * @arg: pointer to a variable specific to command.
 */
int hsi_ll_ioctl(int channel, int command, void *arg)
{
	int ret;

	if(hsi_ll_data.state == HSI_LL_IF_STATE_ERR_RECOVERY)
		return HSI_LL_RESULT_ERROR;

	switch(command) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	case HSI_LL_IOCTL_RX_RESUME: {
		if(hsi_ll_data.ch[channel].rx.state == HSI_LL_RX_STATE_BLOCKED) {
			struct hsi_ll_rx_tx_data *tmp = (struct hsi_ll_rx_tx_data*)arg;
			if(arg == NULL) {
				return HSI_LL_RESULT_ERROR;
			}
#if defined (HSI_LL_ENABLE_DEBUG_LOG)
			printk("\nHSI_LL: Got mem(size=%d), so unblocking ch %d.\n",
						tmp->size,channel);
#endif
			hsi_ll_data.ch[channel].rx.buffer = tmp->buffer;
			hsi_ll_data.ch[channel].rx.size   = tmp->size;
			hsi_ll_send_command(HSI_LL_MSG_ACK, channel,
								&tmp->size,
								HSI_LL_PHY_ID_RX);
			HSI_LL_GET_SIZE_IN_WORDS(tmp->size);
			hsi_ll_data.ch[channel].rx.state = HSI_LL_RX_STATE_RX;
			hsi_read(hsi_ll_data.dev[channel],
				tmp->buffer,
				tmp->size);
			ret = HSI_LL_RESULT_SUCCESS;
		} else {
			ret = HSI_LL_RESULT_ERROR;
		}
		}
		break;
#endif
	default:
		ret = HSI_LL_RESULT_ERROR;
		break;
	}

	return ret;
}
