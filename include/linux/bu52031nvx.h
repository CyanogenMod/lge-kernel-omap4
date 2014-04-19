/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_BU52031NVX_H__
#define _LINUX_BU52031NVX_H__

#define BU52031NVX_MODULE_NAME "bu52031nvx"

#ifdef __KERNEL__

struct bu52031nvx_platform_data {
	unsigned char docked_gpio;
	unsigned char is_desk;
} __attribute__ ((packed));

#endif /* __KERNEL__ */

#endif /* _LINUX_BU52031NVX_H__ */
