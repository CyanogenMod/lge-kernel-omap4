/*
 *  MUIC client class driver
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 * Author: Seungho Park <seungho1.park@lge.com>
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

#ifndef __LINUX_MUIC_CLIENT_H__
#define __LINUX_MUIC_CLIENT_H__

//#include <linux/muic.h>
typedef enum {
	MUIC_CLIENT_NOTI_DP3T   = 0,    /* LAST */
	MUIC_CLIENT_NOTI_USIF,
	MUIC_CLIENT_NOTI_MUIC,
	MUIC_CLIENT_NOTI_POWER_MHL,             /* FIRST */
} MUIC_ORDER_OF_NOTIFICATION;

struct muic_client_device;

struct muic_client_ops {
#if defined(CONFIG_MUIC_TSU5611) && defined(CONFIG_MACH_LGE_U2)
	int notifier_priority;
#endif
	int (*on_unknown)(struct muic_client_device *);
	int (*on_none)(struct muic_client_device *);
	int (*on_na_ta)(struct muic_client_device *);
	int (*on_lg_ta)(struct muic_client_device *);
	int (*on_1a_ta)(struct muic_client_device *);
	int (*on_invalid_chg)(struct muic_client_device *);
	int (*on_ap_uart)(struct muic_client_device *);
	int (*on_cp_uart)(struct muic_client_device *);
	int (*on_ap_usb)(struct muic_client_device *);
	int (*on_cp_usb)(struct muic_client_device *);
	int (*on_earmic)(struct muic_client_device *);
	int (*on_mhl)(struct muic_client_device *);
};

struct muic_client_device {
	const char	*name;
	struct device	dev;
	unsigned int mode;
	int		index;
	struct mutex ops_lock;

	struct notifier_block muic_notif;
	struct muic_client_ops *ops;
};

extern int muic_client_dev_register(const char *, void *, struct muic_client_ops *);

#define to_muic_client_device(obj) container_of(obj, struct muic_client_device, dev)

#endif /* __LINUX_SWITCH_H__ */
