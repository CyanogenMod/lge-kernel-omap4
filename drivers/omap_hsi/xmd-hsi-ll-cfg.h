/*
 * xmd-hsi-ll-cfg.h
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

#ifndef __XMD_HSI_LL_CFG_H__
#define __XMD_HSI_LL_CFG_H__


#if 0  //for GB
#include <mach/omap_hsi.h>
#else   //for ICS
#include <plat/omap_hsi.h>
#endif


#define HSI_LL_MAX_CHANNELS         HSI_CHANNELS_MAX /* Max channels supported */

/* Receiver modes */
#define HSI_LL_SYNC_DATA_FLOW       HSI_FLOW_SYNCHRONIZED  /* Synchronized Data Flow */
#define HSI_LL_PIPELINED_DATA_FLOW  HSI_FLOW_PIPELINED     /* Pipelined Data Flow    */

/* Interface modes  */
#define HSI_LL_STREAM_MODE          HSI_MODE_STREAM
#define HSI_LL_FRAME_MODE           HSI_MODE_FRAME

/* Priority mode */
#define HSI_LL_ARBMODE_ROUNDROBIN   HSI_ARBMODE_ROUNDROBIN/* Round Robin Mode*/
#define HSI_LL_ARBMODE_PRIORITY     HSI_ARBMODE_PRIORITY  /* Priority Mode */

/* Default settings */
#define HSI_LL_INTERFACE_MODE       HSI_LL_FRAME_MODE
#define HSI_LL_RECEIVER_MODE        HSI_LL_SYNC_DATA_FLOW
#define HSI_LL_ARBMODE_MODE         HSI_LL_ARBMODE_ROUNDROBIN

/* Frame Size */
#define HSI_LL_MAX_FRAME_SIZE       HSI_FRAMESIZE_DEFAULT

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3)
    Divisor value => HSI CLK == HSI base CLK/(1+divisor value)
    Clock Change 48MHz => 96MHz */
#define HSI_LL_DIVISOR_VALUE        HSI_DIVISOR_DEFAULT /* For 96MHZ Base CLK, 96MHZ(0) 48MHZ(1)  24MHZ(3) */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

/*To enable Power management */
#define HSI_LL_ENABLE_PM
//mo2haewoon.you@lge.com [START]
#if defined(CONFIG_MACH_LGE_COSMO) 
#define HSI_LL_WAKE_LOCK

#define HSI_LL_AC_WAKE_TIMEOUT		3 * HZ
#define HSI_LL_CA_WAKE_TIMEOUT		3 * HZ
#endif
//mo2haewoon.you@lge.com [END]

#define HSI_LL_COUNTERS_VALUE	   (HSI_COUNTERS_FT_DEFAULT | \
									HSI_COUNTERS_TB_DEFAULT | \
									HSI_COUNTERS_FB_DEFAULT)

#ifdef CONFIG_MACH_LGE_COSMO_REV_C
/* Work around for Modem Bug for TX/RX data buffer size config. Size should be multiple of 16 */
/* Enable for ES1 XG626(RevC) & Disable ES2 XG626(RevD~)*/
//#define HSI_LL_DATA_MUL_OF_16 */
#endif

/* Enable timers for DLP recovery. */
/* NOTE: This will initiate onlY DLP recovery and not TTY/RMNET or RIL recover,
   For RIL recovery this timer should be disabled as 2 timers can only create
   SYNC issues */
/* #define HSI_LL_ENABLE_TIMERS */

/* Use this define if TX retry delay WQ has to be enabled.
   If MODEM has logic where it does not send NAK, then below define
   is not required.*/
/* #define HSI_LL_ENABLE_TX_RETRY_WQ */

/* Enable this to make sure that NAK is not sent to MODEM if buf is not
   available. When buf is not available AP does not send any response(NAK)
   instead waits for buffer and then sends NAK. Also it's necessarry that
   MODEM TX Timers should be disabled to avoid CP side TX timeouts.*/
#define HSI_LL_ENABLE_RX_BUF_RETRY_WQ

#define HSI_LL_MAX_OPEN_CONN_RETRIES		5 //200       /* Max Retries for OPEN_CONNECT_OCTECT */

/* L1 Recovery */
#define HSI_LL_MAX_ERROR_RETRY               10
#define HSI_LL_ERROR_RECOVERY_TIME_MS        10

/* Timeout in ms */
/* Send OPEN_CONN */
#define HSI_LL_TX_T_OPEN_CONN_MAX_MS          5
/* Send CONF_RATE */
#define HSI_LL_TX_T_CONF_RATE_MAX_MS          5
/* Wait for ACK or NACK */
#define HSI_LL_TX_T_ACK_MAX_MS               10
/* Wait for CONN_READY */
#define HSI_LL_TX_T_CONN_READY_MS            15
/* Wait for CONN_CLOSE */
#define HSI_LL_TX_T_CONN_CLOSED_MAX_MS       10
/* Send BREAK */
#define HSI_LL_TX_T_BREAK_MAX_MS              5
#define HSI_LL_TX_T_CONN_OPEN_RETRY_MS       15

/* To be multiplied with LL_TX_T_CONN_OPEN_RETRY_MS */
#define HSI_LL_TX_N_RETRY_MAX                 5

/* Send ACK or NAK */
#define HSI_LL_RX_T_ACK_NACK_MS               5
/* Send CONN_READY */
#define HSI_LL_RX_T_CONN_READY_MS             5
/* Send BREAK */
#define HSI_LL_RX_T_BREAK_MAX_MS              5
/* Send CONN_CLOSED */
#define HSI_LL_RX_T_CONN_CLOSED_MAX_MS        5
/* Send CANCEL_CONN */
#define HSI_LL_RX_T_CANCEL_CONN_MS            5
/* Wait for ACK/NACK after CANCEL_CONN */
#define HSI_LL_RX_T_CANCEL_ACK_NACK_MS       10

/* #define HSI_LL_ENABLE_DEBUG_LOG */
/* #define HSI_LL_ENABLE_CRITICAL_LOG */
#define HSI_LL_ENABLE_ERROR_LOG

#endif /* __XMD_HSI_LL_CFG_H__ */
