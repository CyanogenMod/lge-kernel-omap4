/*
 * xmd-hsi-ll-internal.h
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

#ifndef __XMD_HSI_LL_INTERNAL_H__
#define __XMD_HSI_LL_INTERNAL_H__

#include <linux/timer.h>

#define HSI_LL_CTRL_CHANNEL 0
#define HSI_LL_INVALID_CHANNEL 0xFF
#define HSI_LL_MAX_CMD_Q_SIZE       32
#define HSI_LL_TIMER_Q_SIZE (3 * HSI_LL_MAX_CHANNELS)

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (HSI_LL_ENABLE_PM)
#define HSI_LL_PV_READ_CMD_Q_TIMEOUT   80
#define HSI_LL_PV_THREAD_SLEEP_TIME    20
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

enum{
	FALSE,
	TRUE,
};

/* Constants to indicate HSI HW Power state */
enum {
	HSI_LL_HSI_HW_ON,
	HSI_LL_HSI_HW_OFF,
};

/* Constants to indicate HSI Wake line status */
enum {
	HSI_LL_WAKE_LINE_LOW,
	HSI_LL_WAKE_LINE_HIGH,
};

/* Constants to indicate COMMAND Initiator*/
enum {
	HSI_LL_PHY_ID_RX,
	HSI_LL_PHY_ID_TX,
	HSI_LL_PHY_UNKNOWN,
};

/* Constants to indicate command initiator role */
enum {
	HSI_LL_ROLE_TRANSMITTER,
	HSI_LL_ROLE_RECEIVER,
};

/* PSV request modes */
enum {
	HSI_LL_PSV_EVENT_INVALID,
	HSI_LL_PSV_EVENT_PSV_DISABLE,
	HSI_LL_PSV_EVENT_PSV_ENABLE,
};

/* Interface state */
enum {
	HSI_LL_IF_STATE_UN_INIT,
	HSI_LL_IF_STATE_READY,
	HSI_LL_IF_STATE_CONFIG,
	HSI_LL_IF_STATE_ERR_RECOVERY,
	HSI_LL_IF_STATE_PERM_ERROR,
};

/* Link Layer commands */
enum {
	HSI_LL_MSG_BREAK           = 0x00,
	HSI_LL_MSG_ECHO            = 0x01,
	HSI_LL_MSG_INFO_REQ        = 0x02,
	HSI_LL_MSG_INFO            = 0x03,
	HSI_LL_MSG_CONFIGURE       = 0x04,
	HSI_LL_MSG_ALLOCATE_CH     = 0x05,
	HSI_LL_MSG_RELEASE_CH      = 0x06,
	HSI_LL_MSG_OPEN_CONN       = 0x07,
	HSI_LL_MSG_CONN_READY      = 0x08,
	HSI_LL_MSG_CONN_CLOSED     = 0x09,
	HSI_LL_MSG_CANCEL_CONN     = 0x0A,
	HSI_LL_MSG_ACK             = 0x0B,
	HSI_LL_MSG_NAK             = 0x0C,
	HSI_LL_MSG_CONF_RATE       = 0x0D,
	HSI_LL_MSG_OPEN_CONN_OCTET = 0x0E,
	HSI_LL_MSG_INVALID         = 0xFF,
};

/* Channel TX state */
enum {
	HSI_LL_TX_STATE_UNDEF,
	HSI_LL_TX_STATE_CLOSED,
	HSI_LL_TX_STATE_IDLE,
	HSI_LL_TX_STATE_POWER_DOWN,
	HSI_LL_TX_STATE_ERROR,
	HSI_LL_TX_STATE_OPEN_CONN,
	HSI_LL_TX_STATE_WAIT_FOR_ACK,
	HSI_LL_TX_STATE_NACK,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_READY,
	HSI_LL_TX_STATE_SEND_CONF_RATE,
	HSI_LL_TX_STATE_WAIT_FOR_CONF_ACK,
	HSI_LL_TX_STATE_TX,
	HSI_LL_TX_STATE_WAIT_FOR_CONN_CLOSED,
	HSI_LL_TX_STATE_TO_OPEN_CONN,
	HSI_LL_TX_STATE_TO_ACK,
	HSI_LL_TX_STATE_TO_READY,
	HSI_LL_TX_STATE_TO_CONF,
	HSI_LL_TX_STATE_TO_CONF_ACK,
	HSI_LL_TX_STATE_TO_TX,
	HSI_LL_TX_STATE_TO_CLOSE,
	HSI_LL_TX_STATE_SEND_BREAK,
	HSI_LL_TX_STATE_WAIT_FOR_TX_COMPLETE,
};

/* Channel RX state */
enum {
	HSI_LL_RX_STATE_UNDEF,
	HSI_LL_RX_STATE_CLOSED,
	HSI_LL_RX_STATE_IDLE,
	HSI_LL_RX_STATE_POWER_DOWN,
	HSI_LL_RX_STATE_ERROR,
	HSI_LL_RX_STATE_BLOCKED,
	HSI_LL_RX_STATE_SEND_ACK,
	HSI_LL_RX_STATE_SEND_NACK,
	HSI_LL_RX_STATE_SEND_CONN_READY,
	HSI_LL_RX_STATE_RX,
	HSI_LL_RX_STATE_SEND_CONN_CLOSED,
	HSI_LL_RX_STATE_SEND_CONN_CANCEL,
	HSI_LL_RX_STATE_WAIT_FOR_CANCEL_CONN_ACK,
	HSI_LL_RX_STATE_SEND_CONF_ACK,
	HSI_LL_RX_STATE_SEND_CONF_NACK,
	HSI_LL_RX_STATE_TO_RX,
	HSI_LL_RX_STATE_TO_ACK,
	HSI_LL_RX_STATE_TO_NACK,
	HSI_LL_RX_STATE_TO_CONN_READY,
	HSI_LL_RX_STATE_TO_CONN_CLOSED,
	HSI_LL_RX_STATE_TO_CONN_CANCEL,
	HSI_LL_RX_STATE_TO_CONN_CANCEL_ACK,
	HSI_LL_RX_STATE_TO_CONF_ACK,
	HSI_LL_RX_STATE_SEND_BREAK,
};

/* Timer initiator */
enum {
	HIS_LL_TIMER_DIR_UNDEF,
	HIS_LL_TIMER_DIR_RX,
	HIS_LL_TIMER_DIR_TX,
};

/* Timer Queue state */
enum {
	HIS_LL_TIMER_QUEUE_CMD_INVALID,
	HIS_LL_TIMER_QUEUE_CMD_START,
	HIS_LL_TIMER_QUEUE_CMD_STOP,
};

/* struct hsi_ll_timer_Q - Timer Queue
 * @timer_cmd: timer commnad, start/stop
 * @timer_dir: direction tx/rx
 * @channel:   channel number
 * @time_out:  timeout value in ms
 */
typedef struct hsi_ll_timer_Q {
	unsigned int timer_cmd;
	unsigned int timer_dir;
	unsigned int channel;
	unsigned int time_out;
} ll_timer_Q;

/* struct hsi_ll_cmd_queue - Command queue
 * @command: HSI command
 * @channel: channel number
 * @phy_id:  TX/RX
 */
typedef struct hsi_ll_cmd_queue {
	unsigned int command;
	unsigned int channel;
	unsigned int phy_id;
} ll_cmd_queue;

/* struct hsi_ll_tx_cmd_q - TX command Queue
 * @read_index: read index
 * @write_index: write index
 * @count: queue count
 * @phy_id: TX/RX
 * @channel:channel number
 * @cmd_q: See struct hsi_ll_cmd_queue
 */
typedef struct hsi_ll_tx_cmd_q {
	unsigned int read_index;
	unsigned int write_index;
	unsigned int count;
	unsigned int phy_id;
	unsigned int channel;
	struct hsi_ll_cmd_queue cmd_q[HSI_LL_MAX_CMD_Q_SIZE];
} ll_tx_cmd_q;

/* struct hsi_ll_tx_ch - TX channel structure
 * @state: TX channel state
 * @close_req: Close req indicator
 * @data_rate: data rate
 * @pending: write pending indication
 * @retry: retry count
 * @buffer: pointer to buffer
 * @size: number of bytes
 * @timer_id: timer id
 */
typedef struct hsi_ll_tx_ch {
	unsigned int channel;
	unsigned int state;
	unsigned int close_req;
	unsigned int data_rate;
	unsigned int pending;
	unsigned int retry;
	void	    *buffer;
	unsigned int size;
	struct timer_list timer_id;
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	struct work_struct retry_work;
#endif
} ll_tx_ch;

/* struct hsi_ll_rx_ch - RX channel structure
 * @state: RX channel state
 * @buffer: pointer to buffer
 * @size: number of bytes
 * @close_req: Close req indicator
 * @timer_id: timer id
 */
typedef struct hsi_ll_rx_ch {
	unsigned int state;
	void        *buffer;
	unsigned int size;
	unsigned int close_req;
	struct timer_list timer_id;
} ll_rx_ch;

/* struct hsi_ll_channel - HSI LL channel structure
 * @open: channel open/close indicator
 * @tx: see struct hsi_ll_tx_ch
 * @rx: see struct hsi_ll_rx_ch
 */
typedef struct hsi_ll_channel {
	unsigned int        open;
	struct hsi_ll_tx_ch tx;
	struct hsi_ll_rx_ch rx;
} ll_channel;

/* struct hsi_ll_tx_cfg - TX configuration structure
 * @new_data_rate_valid: new baud rate
 * @baud_rate: baud rate
 * @ac_wake: AC wakeline status
 * @ctx: see struct hst_ctx
 */
typedef struct hsi_ll_tx_cfg {
	unsigned int   new_data_rate_valid;
	unsigned int   baud_rate;
	unsigned int   ac_wake;
	struct hst_ctx ctx;
} ll_tx_cfg;

/* struct hsi_ll_rx_cfg - RX configuration structure
 * @new_data_rate_valid: new baud rate
 * @baud_rate: baud rate
 * @ca_wake: CA wakeline status
 * @ctx: see struct hsr_ctx
 */
typedef struct hsi_ll_rx_cfg {
	unsigned int   new_data_rate_valid;
	unsigned int   baud_rate;
	unsigned int   ca_wake;
	struct hsr_ctx ctx;
} ll_rx_cfg;

/* struct hsi_ll_data_struct - LL data structure
 * @initialized: HSI LL init state
 * @state: Current HSI LL state
 * @rx_cmd: HSI LL CMD from CP
 * @tx_cmd: see struct hsi_ll_tx_cmd_q
 * @tx_cfg: see struct hsi_ll_tx_cfg
 * @rx_cfg: see struct hsi_ll_rx_cfg
 * @ch: see struct hsi_ll_channel
 * @dev:  pointer to hsi_device
 */
typedef struct hsi_ll_data_struct {
	unsigned int            initialized;
	unsigned int            state;
	unsigned int            rx_cmd;
	struct hsi_ll_tx_cmd_q  tx_cmd;
	struct hsi_ll_tx_cfg    tx_cfg;
	struct hsi_ll_rx_cfg    rx_cfg;
	struct hsi_ll_channel   ch[HSI_LL_MAX_CHANNELS];
	struct hsi_device      *dev[HSI_LL_MAX_CHANNELS];
} ll_data_struct;

/* struct hsi_ll_if_struct - LL IF structure.
 * @wr_complete_flag: Flag to indicate write complete.
 * @rd_complete_flag: Flag to indicate write complete.
 * @msg_avaliable_flag: Flag to indicate message  availability.
 * @reg_complete_flag: Flag to indicate HSI LL registration with HSI PHY driver.
 * @psv_event_flag: Flag to indicate PSV event.
 * @timer_queue_cnt: Timer Queue counter.
 * @timer_queue_wr: Timer Queue write index.
 * @timer_queue_rd: Timer Queue read index.
 * @reg_complete_ch_count: Counter to indicate number of channels registered.
 * @phy_cb_lock: spin_lock for PHY driver calledbacks.
 * @start_tx_timer_lock: spin_lock for TX start timer.
 * @start_rx_timer_lock: spin_lock for RX start timer.
 * @stop_tx_timer_lock:  spin_lock for TX stop timer.
 * @stop_rx_timer_lock:  spin_lock for RX stop timer.
 * @tx_timer_cb_lock:  spin_lock for TX timer callback.
 * @rx_timer_cb_lock:  spin_lock for RX timer callback.
 * @timer_bh_lock:   spin_lock for timer tasklet.
 * @wr_cmd_cb_lock:  spin_lock for ch 0 write complete callback.
 * @rd_cmd_cb_lock: spin_lock for ch 0 read complete callback.
 * @wr_cb_lock: spin_lock for write complete callback.
 * @rd_cb_lock: spin_lock for read complete callback.
 * @wr_complete: event for write complete.
 * @rd_complete: event for read complete.
 * @msg_avaliable: event for message availability.
 * @reg_complete: event for HSI LL registration complete.
 * @psv_event: event for PSV requests.
 * @rd_th: Read thread ID.
 * @wr_th: write Thread ID.
 * @psv_th: PSV Thread ID.
 * @timer_tasklet: Timer tasklet ID.
 */
typedef struct hsi_ll_if_struct {
	int                   wr_complete_flag;
	int                   rd_complete_flag;
	int                   msg_avaliable_flag;
	int                   reg_complete_flag;
#if defined (HSI_LL_ENABLE_PM)
	int                   psv_event_flag;
#endif
#if defined (HSI_LL_ENABLE_TIMERS)
	unsigned int          timer_queue_cnt;
	unsigned int          timer_queue_wr;
	unsigned int          timer_queue_rd;
#endif
	unsigned int          reg_complete_ch_count;
	spinlock_t            phy_cb_lock;
#if defined (HSI_LL_ENABLE_TIMERS)
	spinlock_t            start_tx_timer_lock;
	spinlock_t            start_rx_timer_lock;
	spinlock_t            stop_tx_timer_lock;
	spinlock_t            stop_rx_timer_lock;
	spinlock_t            tx_timer_cb_lock;
	spinlock_t            rx_timer_cb_lock;
	spinlock_t            timer_bh_lock;
#endif
	spinlock_t            wr_cmd_cb_lock;
	spinlock_t            rd_cmd_cb_lock;
	spinlock_t            wr_cb_lock;
	spinlock_t            rd_cb_lock;
	spinlock_t            wr_cmd_lock;
	wait_queue_head_t     wr_complete;
	wait_queue_head_t     rd_complete;
	wait_queue_head_t     msg_avaliable;
	wait_queue_head_t     reg_complete;
#if defined (HSI_LL_ENABLE_PM)
	wait_queue_head_t     psv_event;
#endif
	struct task_struct   *rd_th;
	struct task_struct   *wr_th;
#if defined (HSI_LL_ENABLE_PM)
	struct task_struct   *psv_th;
#endif
#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
	struct workqueue_struct *hsi_tx_retry_wq;
#endif
#if defined (HSI_LL_ENABLE_TIMERS)
	struct tasklet_struct timer_tasklet;
#endif
} ll_if_struct;

static int hsi_ll_probe_cb(struct hsi_device *dev);
static int hsi_ll_remove_cb(struct hsi_device *dev);
static void hsi_ll_read_complete_cb(struct hsi_device *dev, unsigned int size);
static void hsi_ll_write_complete_cb(struct hsi_device *dev, unsigned int size);
static void hsi_ll_port_event_cb(struct hsi_device *dev, unsigned int event, void *arg);
static void hsi_ll_read_cb(struct hsi_device *dev, unsigned int size);
static void hsi_ll_write_cb(struct hsi_device *dev, unsigned int size);

static void hsi_ll_start_tx_timer(unsigned int channel, unsigned int time_out);
static void hsi_ll_start_rx_timer(unsigned int channel, unsigned int time_out);

static void hsi_ll_stop_tx_timer(unsigned int channel);
static void hsi_ll_stop_rx_timer(unsigned int channel);

#if defined (HSI_LL_ENABLE_TIMERS)
static void hsi_ll_tx_timer_cb(unsigned long channel);
static void hsi_ll_rx_timer_cb(unsigned long channel);
static void hsi_ll_stop_channel(unsigned int channel);
#endif

static int hsi_ll_rd_ctrl_ch_th(void *data);
static int hsi_ll_wr_ctrl_ch_th(void *data);

static void hsi_ll_wakeup_cp(unsigned int val);

#if defined (HSI_LL_ENABLE_TX_RETRY_WQ)
static void hsi_ll_retry_work(struct work_struct *work);
#endif

#endif /* __XMD_HSI_LL_INTERNAL_H__ */
