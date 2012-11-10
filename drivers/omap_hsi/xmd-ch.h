/*
 * xmd-ch.h
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

#ifndef __XMD_CH_H__
#define __XMD_CH_H__

#ifdef CONFIG_OMAP4_XMM_SPI
#ifdef CONFIG_OMAP4_XMM_HSI
#error "Both XMD SPI DRIVER and XMD HSI DRIVER should not be enabled"
#endif
#endif

#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#define MAX_SMD_TTYS 12
#define MAX_SMD_NET 3
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[1]))
#endif

#define XMD_RIL_RECOVERY_CHANNEL 1

#ifndef PRIVATE_INFO_LENGTH
#define PRIVATE_INFO_LENGTH 5
#endif

typedef enum _HSI_CH_USER_
{
	XMD_TTY,
	XMD_NET,
} XMD_CH_USER;

struct xmd_ch_info {
	int id;
	char name[32];
	int chno;
	XMD_CH_USER user;
	void *priv;
	int open_count;
	spinlock_t lock;
};

struct hsi_board_info {
	char  name[32];
	const void *platform_data;
};

void xmd_hsi_register_board_info(struct hsi_board_info *board_info, int size);
void xmd_hsi_init(void);
void xmd_hsi_exit(void);
int xmd_boot_enable_fw_tty(void);
void xmd_boot_disable_fw_tty(void);

/* Release interrupt and set all the pins to safe */
int xmd_hsi_power_off(void);

/* Configure gpios and request irq */
void xmd_hsi_power_on_reset(void);

/* runtime gpio configuration like keeping wake line to high */
/* void xmd_hsi_board_init(struct xmd_hsi_platform_data *d); */

void xmd_ch_init(void);
void xmd_ch_exit(void);
int xmd_ch_open(struct xmd_ch_info* xmd_ch, void (*notify_cb)(int chno));
void xmd_ch_close(int chno);

/* Returns the buffer pointer in which recieved data is stored.
   should be called from notify_cb.
   memory will be freed after notify_cb returns. */
void *xmd_ch_read(int chno, int* len);

/* Data is copied in local buffer and then its transmitted.
   Freeing of *data is not done. */
int xmd_ch_write(int  chno, void *data, int len);

void xmd_ch_register_xmd_boot_cb(void (*fn)(void));
int wait_for_xmd_ack(void);

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE_COSMOPOLITAN)
extern void ifx_schedule_cp_dump_or_reset(void);
#endif

int xmd_is_recovery_state(void);
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#endif
