/*
 * xmd-hsi-mcm.c
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 *
 * Author: Chaitanya <Chaitanya.Khened@intel.com>
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include "xmd-ch.h"
#include "xmd-hsi-mcm.h"
#include "xmd-hsi-ll-if.h"
#include "xmd_hsi_mem.h"
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/wakelock.h>

#define MCM_DBG_LOG 0
#define MCM_DBG_ERR_LOG 1
#define MCM_DBG_ERR_RECOVERY_LOG 1

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* TI HSI driver (from HSI_DRIVER_VERSION 0.4.2) can suppport port 1 and 2, 
	but IMC XMD currently supports port 1 only */
#define XMD_SUPPORT_PORT 1
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

static struct hsi_chn hsi_all_channels[MAX_HSI_CHANNELS] = {
	{"CONTROL",  HSI_CH_NOT_USED},
	{"CHANNEL1", HSI_CH_FREE},
	{"CHANNEL2", HSI_CH_FREE},
	{"CHANNEL3", HSI_CH_FREE},
	{"CHANNEL4", HSI_CH_FREE},
	{"CHANNEL5", HSI_CH_FREE},
	{"CHANNEL6", HSI_CH_FREE},
	{"CHANNEL7", HSI_CH_FREE},
	{"CHANNEL8", HSI_CH_FREE},
	{"CHANNEL9", HSI_CH_FREE},
	{"CHANNEL10",HSI_CH_FREE},
	{"CHANNEL11",HSI_CH_FREE},
	{"CHANNEL12",HSI_CH_FREE},
	{"CHANNEL13",HSI_CH_FREE},
	{"CHANNEL14",HSI_CH_FREE},
	{"CHANNEL15",HSI_CH_FREE},
};
/* TODO: Fine tune as required */
#define HSI_MCM_TTY_TX_TIMEOUT_VAL (4*HZ) /*timeout, in seconds*/  /* LGE_UPDAT 2011.12.31_hyungsun.seo@lge.com_HSI pending issue during RIL Recovery*/

static unsigned int hsi_mcm_state;
static int is_dlp_reset_in_progress;

static struct work_struct XMD_DLP_RECOVERY_wq;
static struct hsi_channel hsi_channels[MAX_HSI_CHANNELS];
static struct workqueue_struct *hsi_read_wq;
static struct workqueue_struct *hsi_write_wq;
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static struct workqueue_struct *hsi_buf_retry_wq;
#endif

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#define ENABLE_RECOVERY_WAKE_LOCK

#if defined (ENABLE_RECOVERY_WAKE_LOCK)
#define RECOVERY_WAKELOCK_TIME		(30*HZ)
struct wake_lock xmd_recovery_wake_lock;
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

void (*xmd_boot_cb)(void);
static void hsi_read_work(struct work_struct *work);
static void hsi_write_work(struct work_struct *work);

static void xmd_dlp_recovery(void);
static void xmd_ch_reinit(void);
static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static void hsi_buf_retry_work(struct work_struct *work);
#endif
extern void ifx_pmu_reset(void);
extern void rmnet_restart_queue(int chno);

void init_q(int chno)
{
	int i;

	hsi_channels[chno].rx_q.head  = 0;
	hsi_channels[chno].rx_q.tail  = 0;
	hsi_channels[chno].tx_q.head  = 0;
	hsi_channels[chno].tx_q.tail  = 0;
	hsi_channels[chno].tx_blocked = 0;
	hsi_channels[chno].pending_rx_msgs = 0;
	hsi_channels[chno].pending_tx_msgs = 0;

	for (i=0; i<NUM_X_BUF; i++)
		hsi_channels[chno].rx_q.data[i].being_used = HSI_FALSE;
}

/* Head grows on reading from q. "data=q[head];head++;" */
struct x_data* read_q(int chno, struct xq* q)
{
	struct x_data *data = NULL;

	if (!q) {
#if MCM_DBG_ERR_LOG
		printk("mcm: NULL Q instance");
#endif
		return NULL;
	}

#if MCM_DBG_LOG
	printk("\nmcm: [read_q]  head = %d, tail = %d\n",q->head,q->tail);
#endif

	spin_lock_bh(&hsi_channels[chno].lock);

	if (q->head == q->tail) {
		spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_LOG
		printk("\nmcm: Q empty [read] \n");
#endif
		return NULL;
	}

	data = q->data + q->head;
	q->head = (q->head + 1) % NUM_X_BUF;

	spin_unlock_bh(&hsi_channels[chno].lock);

	return data;
}

/* Tail grows on writing in q. "q[tail]=data;tail++;" */
int write_q(struct xq* q, char *buf, int size, struct x_data **data)
{
	int temp = 0;

	if (!q) {
#if MCM_DBG_ERR_LOG
		printk("mcm: NULL q instance");
#endif
		return 0;
	}

	temp = (q->tail + 1) % NUM_X_BUF;

	if (temp != q->head) {
		q->data[q->tail].buf  = buf;
		q->data[q->tail].size = size;
		if (data) {
			*data = q->data + q->tail;
		}
		q->tail = temp;
	} else {
#if MCM_DBG_LOG
		printk("\nmcm:Q full [write], head = %d, tail = %d\n",q->head,q->tail);
#endif
		return 0;
	}

		return q->tail > q->head ? q->tail - q->head:q->tail - q->head + NUM_X_BUF;
}

static int hsi_ch_net_write(int chno, void *data, int len)
{
	/* Non blocking write */
	void *buf = NULL;
	static struct x_data *d = NULL;
	int n = 0;
	int flag = 1;
	int ret = 0;
#ifdef CONFIG_MACH_LGE_U2
/* [START] Workqueue Flag Check of hsi_write_work 2012-08-24 seunghwan.jin@lge.com */
        unsigned int wq_flags = 0;
/* [END] Workqueue Flag Check of hsi_write_work 2012-08-24 seunghwan.jin@lge.com */
#endif
#ifdef XMD_TX_MULTI_PACKET
	if (d && hsi_channels[chno].write_queued == HSI_TRUE) {
		if (d->being_used == HSI_FALSE && (d->size + len) < HSI_MEM_LARGE_BLOCK_SIZE) {
#if MCM_DBG_LOG
			printk("\nmcm: Adding in the queued buffer for ch %d\n",chno);
#endif
			buf = d->buf + d->size;
			d->size += len;
			flag = 0;
		} else {
			flag = 1;
		}
	}
#endif
	if (flag) {
#ifdef XMD_TX_MULTI_PACKET
		buf = hsi_mem_alloc(HSI_MEM_LARGE_BLOCK_SIZE);
#else
		buf = hsi_mem_alloc(len);
#endif
		flag = 1;
	}

	if (!buf || !data) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: Failed to alloc memory So Cannot transfer packet.\n");
#endif
		return -ENOMEM;
	}

	memcpy(buf, data, len);

	if (flag) {
		d = NULL;
		n = write_q(&hsi_channels[chno].tx_q, buf, len, &d);
		if (n != 0) {
			hsi_channels[chno].pending_tx_msgs++;
		}
#if MCM_DBG_LOG
		printk("\nmcm: n = %d\n",n);
#endif
		if (n == 0) {
#if MCM_DBG_LOG
			printk("\nmcm: rmnet TX queue is full for channel %d, So cannot transfer this packet.\n",chno);
#endif
			hsi_channels[chno].tx_blocked = 1;
			hsi_mem_free(buf);
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
#ifdef CONFIG_MACH_LGE_U2
			wq_flags = *((unsigned int *)hsi_write_wq);
			if (wq_flags & WQ_DYING) {
				printk("mcm: Workqueue Flag Check is done 0x%x.\n", wq_flags);
			} else {
				queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
			}
#else
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
#endif
			ret = -EBUSY;
		} else if (n == 1) {
			PREPARE_WORK(&hsi_channels[chno].write_work, hsi_write_work);
#ifdef CONFIG_MACH_LGE_U2
			wq_flags = *((unsigned int *)hsi_write_wq);
			if (wq_flags & WQ_DYING) {
				printk("mcm: Workqueue Flag Check is done 0x%x.\n", wq_flags);
			} else {
				queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
			}
#else
			queue_work(hsi_write_wq, &hsi_channels[chno].write_work);
#endif
			ret = 0;
		}
	}

	return ret;
}

static int hsi_ch_tty_write(int chno, void *data, int len)
{
	void *buf = NULL;
	int err;

	buf = hsi_mem_alloc(len);

	if (!buf) {
		return -ENOMEM;
	}

	memcpy(buf, data, len);

	hsi_channels[chno].write_happening = HSI_TRUE;

	err = hsi_ll_write(chno, (unsigned char *)buf, len);

	if (err < 0) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: hsi_ll_write(...) failed. err=%d\n",err);
#endif
		hsi_channels[chno].write_happening = HSI_FALSE;
	} else {
#if MCM_DBG_LOG
		printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif

/* LGE_UPDATE_START 2011.12.31_hyungsun.seo@lge.com_HSI pending issue during RIL Recovery*/
#if 0  //ORGINAL
		wait_event (hsi_channels[chno].write_wait,
					hsi_channels[chno].write_happening == HSI_FALSE);
#else  //RIL Recovery fail patch
		wait_event_timeout (hsi_channels[chno].write_wait,
							hsi_channels[chno].write_happening == HSI_FALSE,
							HSI_MCM_TTY_TX_TIMEOUT_VAL);
		if (hsi_channels[chno].write_happening == HSI_TRUE){
			printk("mcm:[RIL Recovery]hsi_ch_tty_write failed. err= -9\n");
			err = -9;
			}
#endif
/* LGE_UPDATE_END 2011.12.31_hyungsun.seo@lge.com_HSI pending issue during RIL Recovery*/
	}

	return err;
}

static void* hsi_ch_net_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	return data->buf;
}

static void* hsi_ch_tty_read(int chno, int* len)
{
	struct x_data *data = hsi_channels[chno].curr;
	*len = data->size;
	return data->buf;
}

int xmd_ch_write(int chno, void *data, int len)
{
	int err;

#if MCM_DBG_LOG
	printk("\nmcm: write entering, ch %d\n",chno);
#endif

	if (!hsi_channels[chno].write) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm:write func NULL for ch: %d\n",chno);
#endif
		return -EINVAL;
	}

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
		printk("\nmcm:Dropping packets of channel %d as error recovery is in progress\n", chno);
#endif
		return -ENOTBLK;
	}

	err = hsi_channels[chno].write(chno, data, len);

#if MCM_DBG_LOG
	printk("\nmcm: write returning, ch %d\n",chno);
#endif
	return err;
}

void* xmd_ch_read(int chno, int* len)
{
	return hsi_channels[chno].read(chno,len);
}

void xmd_ch_close(int chno)
{
	printk("\nmcm:closing channel %d.\n", chno);

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE)
	/******************************************
		LGE-RIL CHANNEL : 1 , 2, 3, 4, 5, 8, 11
	******************************************/
	if((chno == 1)||(chno == 2)||(chno == 3)
	||(chno == 4)||(chno == 5)||(chno == 8)||(chno == 11)){
#else
	if(chno == XMD_RIL_RECOVERY_CHANNEL) {
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if MCM_DBG_ERR_RECOVERY_LOG
		printk("\nmcm: Ch %d closed so starting Recovery.\n", chno);
#endif
		xmd_dlp_recovery();
	}
	
	if (hsi_channels[chno].read_happening == HSI_TRUE) {
#if MCM_DBG_LOG
		printk("\nmcm:locking read mutex for ch: %d\n",chno);
#endif
		wait_event(hsi_channels[chno].read_wait,
					hsi_channels[chno].read_happening == HSI_FALSE);
	}
	
	hsi_ll_close(chno);
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].state = HSI_CH_FREE;
	spin_unlock_bh(&hsi_channels[chno].lock);
}

int xmd_ch_open(struct xmd_ch_info* info, void (*notify_cb)(int chno))
{
	int i;
	int size = ARRAY_SIZE(hsi_channels);

	for (i=0; i<size; i++) {
		if (hsi_channels[i].name)
			if (!strcmp(info->name, hsi_channels[i].name)) {
				if (hsi_channels[i].state == HSI_CH_BUSY ||

					hsi_channels[i].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
					printk("\nmcm:Channel state not suitable %d\n",i);
#endif
					return -EINVAL;
				}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE)
				/******************************************
					LGE-RIL CHANNEL : 1 , 2, 3, 4, 5, 8, 11
				******************************************/
				if(((i == 1)||(i == 2)||(i == 3)
				||(i == 4)||(i == 5)||(i == 8)||(i == 11)) &&
#else
				if ((i == XMD_RIL_RECOVERY_CHANNEL) &&
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
					(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY)) {
#if MCM_DBG_ERR_RECOVERY_LOG
						printk("\nmcm: Recovery completed by chno %d.\n", i);
#endif
					xmd_ch_reinit();
				}

				if (0 != hsi_ll_open(i)) {
#if MCM_DBG_ERR_LOG
					printk("\nmcm:hsi_ll_open failed for channel %d\n",i);
#endif
					return -EINVAL;
				}

				hsi_channels[i].info = info;

				spin_lock_bh(&hsi_channels[i].lock);
				hsi_channels[i].state = HSI_CH_BUSY;
				spin_unlock_bh(&hsi_channels[i].lock);

				hsi_channels[i].notify = notify_cb;
				switch(info->user)
				{
				case XMD_TTY:
					hsi_channels[i].read = hsi_ch_tty_read;
					hsi_channels[i].write = hsi_ch_tty_write;
				break;
				case XMD_NET:
					hsi_channels[i].read = hsi_ch_net_read;
					hsi_channels[i].write = hsi_ch_net_write;
				break;
				default:
#if MCM_DBG_ERR_LOG
					printk("\nmcm:Neither TTY nor NET \n");
#endif
					return -EINVAL;
				}
				INIT_WORK(&hsi_channels[i].read_work, hsi_read_work);
				INIT_WORK(&hsi_channels[i].write_work, hsi_write_work);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
				INIT_WORK(&hsi_channels[i].buf_retry_work, hsi_buf_retry_work);
#endif
				return i;
			}
	}
#if MCM_DBG_ERR_LOG
	printk("\n Channel name not proper \n");
#endif
	return -EINVAL;
}

void hsi_read_work(struct work_struct *work)
{
	/* function registered with read work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													read_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;

	if (hsi_channels[chno].read_queued == HSI_TRUE) {
#if MCM_DBG_LOG
		printk("\nmcm: read wq already in progress\n");
#endif
		return;
	}

	hsi_channels[chno].read_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].rx_q)) != NULL) {
		char *buf = data->buf;
		hsi_channels[chno].curr = data;

		if (hsi_mcm_state != HSI_MCM_STATE_ERR_RECOVERY) {
			hsi_channels[chno].notify(chno);
#if MCM_DBG_ERR_RECOVERY_LOG
		} else {
			printk("\nmcm:Dropping RX packets of channel %d from WQ as error recovery is in progress\n", chno);
#endif
		}

		hsi_mem_free(buf);
		if(chno >= 13) {
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_retry_work, hsi_buf_retry_work);
				queue_work(hsi_buf_retry_wq, &hsi_channels[chno].buf_retry_work);
			}
#endif
			hsi_channels[chno].pending_rx_msgs--;
		}
	}
	hsi_channels[chno].read_queued = HSI_FALSE;
	spin_lock_bh(&hsi_channels[chno].lock);
	hsi_channels[chno].read_happening = HSI_FALSE;
	spin_unlock_bh(&hsi_channels[chno].lock);

	wake_up(&hsi_channels[chno].read_wait);
}

void hsi_write_work(struct work_struct *work)
{
	/* function registered with write work q */
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													write_work);
	int chno = ch->info->chno;
	struct x_data *data = NULL;
	int err;

// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* RIL recovery memory initialization  */
#if 0
	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
		printk("\nmcm:Dropping packets of channel %d from WQ as error recovery is in progress\n", chno);
#endif
		goto quit_write_wq;
	}
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	if (hsi_channels[chno].write_queued == HSI_TRUE) {
#if MCM_DBG_LOG
		printk("\nmcm: write wq already in progress\n");
#endif
		return;
	}

	hsi_channels[chno].write_queued = HSI_TRUE;

	while ((data = read_q(chno, &hsi_channels[chno].tx_q)) != NULL) {
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* RIL recovery memory initialization  */
#if 1
		if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
			printk("\nmcm:Dropping packets of channel %d from WQ "
					"as error recovery is in progress\n", chno);
#endif
			hsi_mem_free(data->buf);
			continue;
		}
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
		hsi_channels[chno].write_happening = HSI_TRUE;
		data->being_used = HSI_TRUE;
		err = hsi_ll_write(chno, (unsigned char *)data->buf, data->size);
		if (err < 0) {
#if MCM_DBG_ERR_LOG
			printk("\nmcm: hsi_ll_write failed\n");
#endif
			hsi_channels[chno].write_happening = HSI_FALSE;
		} else {
#if MCM_DBG_LOG
			printk("\nmcm:locking mutex for ch: %d\n",chno);
#endif
			wait_event(hsi_channels[chno].write_wait,
						hsi_channels[chno].write_happening == HSI_FALSE);
		}
		hsi_channels[chno].pending_tx_msgs--;
		data->being_used = HSI_FALSE;

		if (hsi_channels[chno].tx_blocked == 1) {
			hsi_channels[chno].tx_blocked = 0;
#if MCM_DBG_LOG
			printk("\nmcm: Channel queue free , restarting TX queue for ch %d \n",chno);
#endif
			rmnet_restart_queue(chno);
		}
	}

// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 0
/* RIL recovery memory initialization  */
quit_write_wq:
#endif
// IMC_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
	hsi_channels[chno].write_queued = HSI_FALSE;
}

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
static void hsi_buf_retry_work(struct work_struct *work)
{
	struct hsi_ll_rx_tx_data temp_data;
	struct hsi_channel *ch = (struct hsi_channel*) container_of(work,
													struct hsi_channel,
													buf_retry_work);
	int chno = ch->info->chno;
	temp_data.size = ch->pending_rx_size;
	temp_data.buffer = NULL;

	if (hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
			return;
	}
	/*GFP_NOFAIL not available so switching to while loop*/
	while(temp_data.buffer == NULL) {
		temp_data.buffer = kmalloc(temp_data.size, GFP_DMA | GFP_KERNEL);
	}
#if MCM_DBG_LOG
	printk("\nHSI_LL: Allocating mem(size=%d) in retry Q for ch %d\n",
			temp_data.size,chno);
#endif
	if(0 > hsi_ll_ioctl(chno, HSI_LL_IOCTL_RX_RESUME, &temp_data)) {
		kfree(temp_data.buffer);
	}
	ch->pending_rx_size = 0;
}
#endif

void hsi_ch_cb(unsigned int chno, int result, int event, void* arg)
{
	ll_rx_tx_data *data = (ll_rx_tx_data *) arg;

	if (!(chno <= MAX_HSI_CHANNELS && chno >= 0) ||
		hsi_channels[chno].state == HSI_CH_NOT_USED) {
#if MCM_DBG_ERR_LOG
		printk("\nmcm: Wrong channel number or channel not used\n");
#endif
		return;
	}

	switch(event) {
	case HSI_LL_EV_ALLOC_MEM: {
		if(chno >= 13) {
			if (hsi_channels[chno].pending_rx_msgs >= NUM_X_BUF) {
				data->buffer = 0;
#if !defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
#if MCM_DBG_ERR_LOG
				printk("\nmcm: Channel %d RX queue is full so sending NAK to CP\n",
						chno);
#endif
#else
				hsi_channels[chno].pending_rx_size = data->size;
				hsi_channels[chno].rx_blocked = 1;
#endif
				break;
			} else {
				hsi_channels[chno].pending_rx_msgs++;
			}
		}

#if MCM_DBG_LOG
		printk("\nmcm: Allocating read memory of size %d to channel %d \n",
					data->size, chno);
#endif
		/* MODEM can't handle NAK so we allocate memory and
			drop the packet after recieving from MODEM */
#if 0
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel not yet opened so not allocating memory\n");
#endif
			data->buffer = NULL;
			break;
		}
		spin_unlock_bh(&hsi_channels[chno].lock);
#endif
		data->buffer = (char *)hsi_mem_alloc(data->size);

#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
		if(data->buffer == NULL) {
			hsi_channels[chno].pending_rx_size = data->size;
			PREPARE_WORK(&hsi_channels[chno].buf_retry_work,
						 hsi_buf_retry_work);
			queue_work(hsi_buf_retry_wq,
					   &hsi_channels[chno].buf_retry_work);
		}
#endif
		}
		break;

	case HSI_LL_EV_FREE_MEM: {
#if MCM_DBG_LOG
		printk("\nmcm: Freeing memory for channel %d, ptr = 0x%p \n",
					chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel not yet opened so cant free mem\n");
#endif
			break;
			}
		spin_unlock_bh(&hsi_channels[chno].lock);
		hsi_mem_free(data->buffer);
		}
		break;

	case HSI_LL_EV_RESET_MEM:
		/* if event is break, handle it somehow. */
		break;

	case HSI_LL_EV_WRITE_COMPLETE: {
#if MCM_DBG_LOG
		printk("\nmcm:unlocking mutex for ch: %d\n",chno);
#endif

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* Uplink Throughput issue */
#if 1
		hsi_mem_free(data->buffer);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
		hsi_channels[chno].write_happening = HSI_FALSE;
		wake_up(&hsi_channels[chno].write_wait);
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* Uplink Throughput issue */
#if 0
		hsi_mem_free(data->buffer);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if MCM_DBG_LOG
		printk("\nmcm: write complete cb, ch %d\n",chno);
#endif
		}
		break;

	case HSI_LL_EV_READ_COMPLETE: {
		int n = 0;
#if MCM_DBG_LOG
		printk("\nmcm: Read complete... size %d, channel %d, ptr = 0x%p \n",
					data->size, chno,data->buffer);
#endif
		spin_lock_bh(&hsi_channels[chno].lock);
		if (hsi_channels[chno].state == HSI_CH_FREE) {
			if(chno >= 13) {
				hsi_channels[chno].pending_rx_msgs--;
			}
			spin_unlock_bh(&hsi_channels[chno].lock);
#if MCM_DBG_ERR_LOG
			printk("\nmcm: channel %d not yet opened so dropping the packet\n",chno);
#endif
			hsi_mem_free(data->buffer);
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
			if(hsi_channels[chno].rx_blocked) {
				hsi_channels[chno].rx_blocked = 0;
				spin_lock_bh(&hsi_channels[chno].lock);
				hsi_channels[chno].pending_rx_msgs++;
				spin_unlock_bh(&hsi_channels[chno].lock);
				PREPARE_WORK(&hsi_channels[chno].buf_retry_work, hsi_buf_retry_work);
				queue_work(hsi_buf_retry_wq, &hsi_channels[chno].buf_retry_work);
			}
#endif
			break;
		}

		n = write_q(&hsi_channels[chno].rx_q, data->buffer, data->size, NULL);

		spin_unlock_bh(&hsi_channels[chno].lock);

		if (n == 0) {
#if MCM_DBG_ERR_LOG
			printk("\nmcm: Dropping the packet as channel %d is busy sending already read data\n",chno);
#endif
			hsi_mem_free(data->buffer);
			/* Schedule work Q to send data to upper layers */
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		} else if (n == 1) {
			if (hsi_channels[chno].read_happening == HSI_FALSE) {
				hsi_channels[chno].read_happening = HSI_TRUE;
			}
			PREPARE_WORK(&hsi_channels[chno].read_work, hsi_read_work);
			queue_work(hsi_read_wq, &hsi_channels[chno].read_work);
		}
		/* if n > 1, no need to schdule the wq again. */
		}
		break;
	default:
		/* Wrong event. */
#if MCM_DBG_ERR_LOG
		printk("\nmcm:Wrong event.ch %d event %d", chno, event);
#endif
		break;
	}
}

void xmd_ch_register_xmd_boot_cb(void (*fn)(void))
{
	xmd_boot_cb = fn;
}

void __init xmd_ch_init(void)
{
	int i;
	int size = ARRAY_SIZE(hsi_all_channels);

#if MCM_DBG_LOG
	printk("\nmcm: xmd_ch_init++\n");
#endif

	for (i=0; i<size; i++) {
		hsi_channels[i].state = hsi_all_channels[i].state;
		hsi_channels[i].name = hsi_all_channels[i].name;
		hsi_channels[i].write_happening = HSI_FALSE;
		hsi_channels[i].write_queued = HSI_FALSE;
		hsi_channels[i].read_queued = HSI_FALSE;
		hsi_channels[i].read_happening = HSI_FALSE;
		spin_lock_init(&hsi_channels[i].lock);
		init_waitqueue_head(&hsi_channels[i].write_wait);
		init_waitqueue_head(&hsi_channels[i].read_wait);
		init_q(i);
	}

	hsi_mem_init();

	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
	/* TI HSI driver (from HSI_DRIVER_VERSION 0.4.2) can suppport port 1 and 2, 
		but IMC XMD currently supports port 1 only */
	hsi_ll_init(XMD_SUPPORT_PORT, hsi_ch_cb);
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	/* Create and initialize work q */
//mo2haewoon.you@lge.com => [START]
//RIP-28065 - Fix workqueue for HSI kernel panic
#if 1
    hsi_read_wq = create_singlethread_workqueue("hsi-read-wq");
    hsi_write_wq = create_singlethread_workqueue("hsi-write-wq");
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
    hsi_buf_retry_wq = create_singlethread_workqueue("hsi_buf_retry_wq");
#endif

#else
	hsi_read_wq = create_workqueue("hsi-read-wq");
	hsi_write_wq = create_workqueue("hsi-write-wq");
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	hsi_buf_retry_wq = create_workqueue("hsi_buf_retry_wq");
#endif
//RIP-28065 - Fix workqueue for HSI kernel panic
//mo2haewoon.you@lge.com <= [END]
#endif
	INIT_WORK(&XMD_DLP_RECOVERY_wq, xmd_dlp_recovery_wq);

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (ENABLE_RECOVERY_WAKE_LOCK)
	wake_lock_init(&xmd_recovery_wake_lock, WAKE_LOCK_SUSPEND, "xmd-recovery-wake");
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
}

void xmd_ch_exit(void)
{
#if defined (HSI_LL_ENABLE_RX_BUF_RETRY_WQ)
	flush_workqueue(hsi_buf_retry_wq);
#endif
	flush_workqueue(hsi_read_wq);
	destroy_workqueue(hsi_read_wq);
	flush_workqueue(hsi_write_wq);
	destroy_workqueue(hsi_write_wq);
	hsi_ll_shutdown();
	hsi_mcm_state = HSI_MCM_STATE_UNDEF;

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (ENABLE_RECOVERY_WAKE_LOCK)
	wake_lock_destroy(&xmd_recovery_wake_lock);
#endif	
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
int xmd_is_recovery_state(void)
{
	if(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {		
		return 1;
	}

	return 0;		
}
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

int xmd_ch_reset(void)
{
	int ch_i;
	int size = ARRAY_SIZE(hsi_all_channels);

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (ENABLE_RECOVERY_WAKE_LOCK)
	wake_lock_timeout(&xmd_recovery_wake_lock, RECOVERY_WAKELOCK_TIME);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if 0
	if(	hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
		printk("\nmcm: xmd_ch_reset already HSI_MCM_STATE_ERR_RECOVERY in progress\n");
#endif
		return -1;
	}

	hsi_mcm_state = HSI_MCM_STATE_ERR_RECOVERY;
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE)
	/*Modem Reset */
	printk("\[MIPI-HSI][P2_ICS] ifx_pmu_reset.\n");

	ifx_pmu_reset();	//RIP-11203
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if MCM_DBG_ERR_RECOVERY_LOG
	printk("\nmcm: HSI DLP Error Recovery initiated.\n");
#endif

	for (ch_i=0; ch_i < size; ch_i++) {
		if (hsi_channels[ch_i].write_happening == HSI_TRUE) {
			hsi_channels[ch_i].write_happening = HSI_FALSE;
			wake_up(&hsi_channels[ch_i].write_wait);
		}
	}
	flush_workqueue(hsi_write_wq);
	hsi_ll_reset();
	flush_workqueue(hsi_read_wq);

	for (ch_i=0; ch_i < size; ch_i++) {
		init_q(ch_i);
	}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE)
	/* TODO: Fine tune to required value. */
	msleep(5000);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if MCM_DBG_ERR_RECOVERY_LOG
	printk("\nmcm: HSI DLP Error Recovery completed waiting for CP ready indication from RIL.\n");
#endif
	/* Change MCM state to initilized when CP ready
		indication from tty ctrl channel is issued */

	return 0;
}

static void xmd_ch_reinit(void)
{
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
	if(hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
		hsi_mem_reinit();
		hsi_mcm_state = HSI_MCM_STATE_INITIALIZED;
	}
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
}

void xmd_dlp_recovery(void)
{
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
	if(is_dlp_reset_in_progress == 0) {

		if( hsi_mcm_state == HSI_MCM_STATE_ERR_RECOVERY) {
#if MCM_DBG_ERR_RECOVERY_LOG
			printk("\nmcm: xmd_ch_reset already HSI_MCM_STATE_ERR_RECOVERY in progress\n");
#endif
			return;
		}

		hsi_mcm_state = HSI_MCM_STATE_ERR_RECOVERY;

		is_dlp_reset_in_progress = 1;

		schedule_work(&XMD_DLP_RECOVERY_wq);
	}
	else
	{
#if MCM_DBG_ERR_RECOVERY_LOG
		printk("\nmcm: xmd_dlp_recovery already in progress\n");
#endif
	}
	// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
}

static void xmd_dlp_recovery_wq(struct work_struct *cp_crash_wq)
{
	xmd_ch_reset(); /* Start MCM/DLP cleanup */
	is_dlp_reset_in_progress = 0;
}
