/*
 * xmd-hsi-ll-if.h
 *
 * Header for the HSI link layer interface.
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

#ifndef __XMD_HSI_LL_IF_H__
#define __XMD_HSI_LL_IF_H__

#include "xmd-hsi-ll-cfg.h"

/* HSI Result type*/
enum {
	HSI_LL_RESULT_SUCCESS          =  0, /* Request executed successfully. */
	HSI_LL_RESULT_ERROR            = -1, /* Request execution failed. */
	HSI_LL_RESULT_UN_INIT          = -2, /* Un-initialized. */
	HSI_LL_RESULT_QUEUE_FULL       = -3, /* Queue full. */
	HSI_LL_RESULT_QUEUE_EMPTY      = -4, /* Queue Empty. */
	HSI_LL_RESULT_TIMER_ERROR      = -5, /* Timer error. */
	HSI_LL_RESULT_INIT_ERROR       = -6, /* Init error. */
	HSI_LL_RESULT_IO_ERROR         = -7, /* IO error. */
	HSI_LL_RESULT_INVALID_PARAM    = -8, /* Invalid param. */
};

/* Event types */
enum {
	HSI_LL_EV_AP_READY,        /* AP ready to send/receive data. */
	HSI_LL_EV_CP_READY,        /* CP ready to send/receive data. */
	HSI_LL_EV_BREAK_DETECTED,  /* Break detected. */
	HSI_LL_EV_ERROR_DETECTED,  /* error detected. */
	HSI_LL_EV_ALLOC_MEM,       /* Memory alloction request. */
	HSI_LL_EV_FREE_MEM,        /* Memory free request. */
	HSI_LL_EV_RESET_MEM,       /* request to release all memory and
								  reset mem alloc module. */
	HSI_LL_EV_WRITE_COMPLETE,  /* Write complete indication. */
	HSI_LL_EV_READ_COMPLETE,   /* Read complete indication. */
};

/* IOCTL commands */
enum {
	HSI_LL_IOCTL_RX_RESUME,
	HSI_LL_IOCTL_INVALID,
};

/* Tx/Rx data structure */
typedef struct hsi_ll_rx_tx_data {
	unsigned int size;
	void*        buffer;
} ll_rx_tx_data;

/* Callback function prototype
 * @channel: channel number.
 * @result: result, check for result type.
 * @event: event, check for event type.
 * @arg: Event specific argument.
 */
typedef void (*hsi_ll_notify)(unsigned int channel,
							  int result,
							  int event,
							  void* arg);

/* hsi_ll_init - HSI LL initialization function , returns HSI_LL_RESULT_SUCCESS
				on success.
 * @port: Number of ports to be used. Should match with number of ports defined
		in hsi_driverif.h.
 * @cb: pointer to callback .
 */
int hsi_ll_init(int port, const hsi_ll_notify cb);

/* hsi_ll_open - HSI LL open channel, returns HSI_LL_RESULT_SUCCESS on success.
 * @channel: Channel number.
 */
int hsi_ll_open(int channel);

/* hsi_ll_close - HSI LL Close channel, returns HSI_LL_RESULT_SUCCESS on success.
 * @channel: Channel number.
 */
int hsi_ll_close(int channel);

/* hsi_ll_write - HSI LL write data, returns HSI_LL_RESULT_SUCCESS on success.
 * buffer should be retained until callback for write completion is invoked
 * by HSI LL.
 * @channel: Channel number.
 * @buf: Pointer to buffer.
 * @size: number of bytes to be transferred.
 */
int hsi_ll_write(int channel, unsigned char *buf, unsigned int size);

/* hsi_ll_shutdown - HSI LL shutdown, to be invoked to release all resources
 *					during shutdown,
 * returns HSI_LL_RESULT_SUCCESS on success.
 */
int hsi_ll_shutdown(void);

/* hsi_ll_reset - HSI LL RESET, to be invoked to reset DLP and HSI PHY driver
 *				  to recovery from error,
 * returns HSI_LL_RESULT_SUCCESS on success.
 */
int hsi_ll_reset(void);

/* hsi_ll_ioctl - HSI LL IOCTL, to be invoked to execute IO CTRL CMDs
 * returns HSI_LL_RESULT_SUCCESS on success.
 */
int hsi_ll_ioctl(int channel, int command, void *arg);

#endif /* __XMD_HSI_LL_IF_H__ */
