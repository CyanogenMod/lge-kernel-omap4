/*
 * xmd-tty.c
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
#include <linux/wait.h>
#include <linux/wakelock.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "xmd-ch.h"

// LGE_ADD_START 20120605 seunghwan.jin@lge.com
// START - dynamic log config
#include <asm/uaccess.h>
#include <linux/proc_fs.h>

char simple_hsi_log_debug_enable = '0';

struct proc_dir_entry *hsi_fp = NULL;

static int read_hsi_proc(char *page, char **start, off_t off, int count, int *eof, void *data_unused)
{
    *page = simple_hsi_log_debug_enable;

    *eof = 1;

    return 1;
}

static int write_hsi_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
    char value;

    if (copy_from_user(&value, buffer, 1))
		return -EFAULT;

    simple_hsi_log_debug_enable = value;

    printk("proc HSI HSI HSI HSI: %d", value);
    return count;
}
// END - dynamic log config
// LGE_ADD_END 20120605 seunghwan.jin@lge.com

/* #define XMD_TTY_ENABLE_DEBUG_MSG */
/* #define XMD_TTY_ENABLE_ERR_MSG */

static DEFINE_MUTEX(xmd_tty_lock);
#if 0
static struct xmd_ch_info tty_channels[MAX_SMD_TTYS] = {
	{0,  "CHANNEL1",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{1,  "CHANNEL2",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{2,  "CHANNEL3",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{3,  "CHANNEL4",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{4,  "CHANNEL5",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{5,  "CHANNEL6",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{6,  "CHANNEL7",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{7,  "CHANNEL8",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{8,  "CHANNEL9",  0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{9,  "CHANNEL10", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{10, "CHANNEL11", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
	{11, "CHANNEL12", 0, XMD_TTY, NULL, 0, SPIN_LOCK_UNLOCKED},
};
#else  //hyungsun.seo_111213 added for ICS
static struct xmd_ch_info tty_channels[MAX_SMD_TTYS] = {
	{0,  "CHANNEL1",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{1,  "CHANNEL2",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{2,  "CHANNEL3",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{3,  "CHANNEL4",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{4,  "CHANNEL5",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{5,  "CHANNEL6",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{6,  "CHANNEL7",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{7,  "CHANNEL8",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{8,  "CHANNEL9",  0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{9,  "CHANNEL10", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{10, "CHANNEL11", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{11, "CHANNEL12", 0, XMD_TTY, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
};
#endif
static int tty_channels_len = ARRAY_SIZE(tty_channels);

static void xmd_ch_tty_send_to_user(int chno)
{
	struct tty_struct *tty = NULL;
	unsigned char *buf = NULL;
	unsigned char *tbuf = NULL;
	int i,len;

	buf = (unsigned char *)xmd_ch_read(chno, &len);

// LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
#if defined(CONFIG_MACH_LGE_COSMO) || defined (CONFIG_MACH_LGE_CX2)
{
    if (simple_hsi_log_debug_enable == '1')
	{
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		printk("xmdtty: Sending data of size %d to ch %d, buf = %s\n", len,chno, str);
		kfree(str);
	}
}
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
{
    if(simple_hsi_log_debug_enable != '1')
    {
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		printk("\nxmdtty: Sending data of size %d to ch %d, buf = %s\n",
					len,chno, str);
		kfree(str);
	}
}
#endif
#else
// start ipc temp

    char *begin;
    char *s;
    char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
#if defined(HSI_PRIVATE_INFO_PROTECTION)
    if(len>PRIVATE_INFO_LENGTH+2)
     {
        memcpy(str, buf, PRIVATE_INFO_LENGTH);
        str[PRIVATE_INFO_LENGTH] ='\n';
        str[PRIVATE_INFO_LENGTH+1] ='\0';
    }
    else
        memcpy(str, buf, len);
#else
    memcpy(str, buf, len);
 #endif
    if(simple_hsi_log_debug_enable == '1')
        printk("xmdtty: Sending data of size %d to ch %d, buf = %s\n", len,chno, str);
    else
    {
        if(chno < 11)
        {
            begin = str;
            while(*begin!='\0') // removing new line character of str
            {
                if(*begin=='\n' || *begin =='\r') begin++;
                else
                {
                    s = begin;
                    break;
                }
            }
            //printk("xmdtty: AP received data size : %d to ch[%d] %s", len,chno, s);
        }
    }
    kfree(str);
// end ipc temp
#endif

// Original code is blocked. Maintain and unblock original code, when you remove updated area.
/*
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
        {
            char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
            memcpy(str, buf, len);
            printk("\nxmdtty: Sending data of size %d to ch %d, buf = %s\n",
                        len,chno, str);
            kfree(str);
        }
#endif
*/
// LGE_UPDATE_END 20120605 seunghwan.jin@lge.com

	for (i=0; i<tty_channels_len; i++) {
		if (tty_channels[i].chno == chno)
			tty = (struct tty_struct *)tty_channels[i].priv;
	}

	if (!tty) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: invalid chno %d \n", chno);
#endif
		return;
	}

	tty->low_latency = 1;

	tty_prepare_flip_string(tty, &tbuf, len);

	if (!tbuf) {
#if defined (XMD_TTY_ENABLE_ERR_MSG)
		printk("\nxmdtty: memory not allocated by tty core to send to user space\n");
#endif
		return;
	}
	memcpy((void *)tbuf, (void *)buf, len);

	tty_flip_buffer_push(tty);
	tty_wakeup(tty);
}

static int xmd_ch_tty_open(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch;
	char init_flag = 0;

	int n = tty->index;
#ifdef CONFIG_MACH_LGE_U2
	int gpio_value = gpio_get_value(122);

	printk("xmdtty: gpio 122 value is %d ##########\n", gpio_value);

	if (gpio_value)
		return -EAGAIN;
#endif
	if (n >= tty_channels_len) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Error opening channel %d\n",n);
#endif
		return -ENODEV;
	}

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty:Opening channel %d\n",n+1);
#endif

	tty_ch = tty_channels + n;

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 0)
		init_flag = 1;

	tty_ch->open_count++;

	if(init_flag) {
		mutex_unlock(&xmd_tty_lock);
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Channel already opened successfully %d\n",
					tty_ch->chno);
#endif
		return 0;
	}

	tty_ch->chno = xmd_ch_open(tty_ch, xmd_ch_tty_send_to_user);
	if (0 > tty_ch->chno) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nError opening channel %d\n",n);
#endif
		mutex_unlock(&xmd_tty_lock);
		tty_ch->open_count = 0;
		tty_ch->chno = 0;
		return -ENOMEM;
	}

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: Channel opened successfully %d\n",tty_ch->chno);
#endif
	tty->driver_data = (void *)tty_ch;
	tty_ch->priv = (void*) tty;
	mutex_unlock(&xmd_tty_lock);

	return 0;
}

static void xmd_ch_tty_close(struct tty_struct *tty, struct file *f)
{
	struct xmd_ch_info *tty_ch = (struct xmd_ch_info*)tty->driver_data;
	char cleanup_flag = 1;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: Channel close function [ch %d]\n",tty_ch->chno);
#endif
	if (!tty_ch) {
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
		printk("\nxmdtty: Channel close function\n");
#endif
		return;
	}

	mutex_lock(&xmd_tty_lock);
	if (tty_ch->open_count > 1)
		cleanup_flag = 0;

	tty_ch->open_count--;
	if (cleanup_flag) {
		xmd_ch_close(tty_ch->chno);
		tty->driver_data = NULL;
	}
	mutex_unlock(&xmd_tty_lock);
}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined(CONFIG_MACH_LGE)
/***********************************************
	LGE-RIL CHANNEL : 1 , 2, 3, 4, 5, 8, 11
	GPS CHANNEL : 7
	AT SERVICE CHANNEL : 10
	VT DATA CHANNEL : 6
	CIQ CHANNEL : 11 (For ATnT)
	DUN : 9 (Not used)

	IMC Modem supports at most 2000 bytes AT command
	io_db.h - SIO_RXP_AT_DEFAULT_BUFLEN (2000)

	IMC Modem VT Data buffer : 8192 bytes
	io_db.h - IO_DS_RX_BUFFER_SIZE (8192)

	IMC Modem CIQ Data buffer : 1024*16
	xia_slave_bridge.c - 1024*16
	
***********************************************/

/* AT command */    
#define XMD_TTY_AT_MAX_WRITE_SIZE	2000

/* VT data */
#define XMD_TTY_VT_DATA_CHANNEL		6
#define XMD_TTY_VT_MAX_WRITE_SIZE	1024

/* CIQ data */
#define XMD_TTY_CIQ_CHANNEL			11
#define XMD_TTY_CIQ_MAX_WRITE_SIZE	1024*8

static int xmd_ch_tty_write(
	struct tty_struct *tty,
	const unsigned char *buf,
	int len)
{
	int written_len = 0;
	struct xmd_ch_info *tty_ch = tty->driver_data;

	/* VT data */
	if(tty_ch->chno == XMD_TTY_VT_DATA_CHANNEL) 
		written_len = min(len, XMD_TTY_VT_MAX_WRITE_SIZE);

#if defined (TARGET_CARRIER_ATT)
	/* CIQ data */
	else if(tty_ch->chno == XMD_TTY_CIQ_CHANNEL) 
		written_len = min(len, XMD_TTY_CIQ_MAX_WRITE_SIZE);
#endif

	/* AT command */
	else 
		written_len = min(len, XMD_TTY_AT_MAX_WRITE_SIZE);
#if defined(CONFIG_MACH_LGE_COSMO) || defined (CONFIG_MACH_LGE_CX2)
// LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
{
    if(simple_hsi_log_debug_enable == '1')
	{
		int max_len = 0;
		char *str = (char *) kzalloc(written_len + 1, GFP_ATOMIC);

		if(tty_ch->chno == XMD_TTY_VT_DATA_CHANNEL) /* VT data */
			max_len = XMD_TTY_VT_MAX_WRITE_SIZE;
#if defined (TARGET_CARRIER_ATT)
		else if(tty_ch->chno == XMD_TTY_CIQ_CHANNEL) /* CIQ data */
			max_len = XMD_TTY_CIQ_MAX_WRITE_SIZE;
#endif
		else /* AT command */
			max_len = XMD_TTY_AT_MAX_WRITE_SIZE;

		if(len > max_len)
			printk("xmdtty: xmd_ch_tty_write len(%d) is bigger than max write size for ch %d\n", len,tty_ch->chno);

		memcpy(str, buf, written_len);
		printk("xmdtty: writing data of size %d to ch %d, data: %s\n", written_len,tty_ch->chno,str);
		kfree(str);
	}
}
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
{
    if(simple_hsi_log_debug_enable != '1')
	{
		int max_len = 0;
		char *str = (char *) kzalloc(written_len + 1, GFP_ATOMIC);

		if(tty_ch->chno == XMD_TTY_VT_DATA_CHANNEL) /* VT data */
			max_len = XMD_TTY_VT_MAX_WRITE_SIZE;
#if defined (TARGET_CARRIER_ATT)		
		else if(tty_ch->chno == XMD_TTY_CIQ_CHANNEL) /* CIQ data */
			max_len = XMD_TTY_CIQ_MAX_WRITE_SIZE;
#endif
		else /* AT command */
			max_len = XMD_TTY_AT_MAX_WRITE_SIZE;

		if(len > max_len)
			printk("\nxmdtty: xmd_ch_tty_write len(%d) is bigger than max write size for ch %d\n",
					len,tty_ch->chno);

		memcpy(str, buf, written_len);
		printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",
					written_len,tty_ch->chno,str);
		kfree(str);
	}
}
#endif
#else
// start ipc temp
{
	int max_len = 0;
	char *str = (char *) kzalloc(written_len + 1, GFP_ATOMIC);

	if(tty_ch->chno == XMD_TTY_VT_DATA_CHANNEL) /* VT data */
		max_len = XMD_TTY_VT_MAX_WRITE_SIZE;
#if defined (TARGET_CARRIER_ATT)
	else if(tty_ch->chno == XMD_TTY_CIQ_CHANNEL) /* CIQ data */
		max_len = XMD_TTY_CIQ_MAX_WRITE_SIZE;
#endif
	else /* AT command */
		max_len = XMD_TTY_AT_MAX_WRITE_SIZE;

	if(len > max_len)
		printk("xmdtty: xmd_ch_tty_write len(%d) is bigger than max write size for ch %d\n", len,tty_ch->chno);
#if defined(HSI_PRIVATE_INFO_PROTECTION)
	if(len>PRIVATE_INFO_LENGTH+2)
	{
		memcpy(str, buf, PRIVATE_INFO_LENGTH);
		str[PRIVATE_INFO_LENGTH] ='\n';
		str[PRIVATE_INFO_LENGTH+1] ='\0';
	}
	else
		memcpy(str, buf, written_len);
#else
	memcpy(str, buf, written_len);
#endif
	if(simple_hsi_log_debug_enable == '1')
		printk("xmdtty: writing data of size %d to ch %d, data: %s\n", written_len,tty_ch->chno,str);
	//else if(tty_ch->chno < 11)
		//printk("xmdtty: CP received data size : %d to ch[%d] %s", written_len,tty_ch->chno,str);

	kfree(str);
}
// end ipc temp
#endif

// Original code is blocked. Maintain and unblock original code when you remove updated area.
//#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
//	{
//		int max_len = 0;
//		char *str = (char *) kzalloc(written_len + 1, GFP_ATOMIC);
//
//		if(tty_ch->chno == XMD_TTY_VT_DATA_CHANNEL) /* VT data */
//			max_len = XMD_TTY_VT_MAX_WRITE_SIZE;
//#if defined (TARGET_CARRIER_ATT)
//		else if(tty_ch->chno == XMD_TTY_CIQ_CHANNEL) /* CIQ data */
//			max_len = XMD_TTY_CIQ_MAX_WRITE_SIZE;
//#endif
//		else /* AT command */
//			max_len = XMD_TTY_AT_MAX_WRITE_SIZE;
//
//		if(len > max_len)
//			printk("\nxmdtty: xmd_ch_tty_write len(%d) is bigger than max write size for ch %d\n",
//					len,tty_ch->chno);
//
//		memcpy(str, buf, written_len);
//		printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",
//					written_len,tty_ch->chno,str);
//		kfree(str);
//	}
//#endif
// LGE_UPDATE_END 20120605 seunghwan.jin@lge.com

/* LGE_UPDATE_START 2011.12.31_hyungsun.seo@lge.com_HSI pending issue during RIL Recovery*/
#if 0
	xmd_ch_write(tty_ch->chno, (void *)buf, written_len);
#else  //RIL Recovery fail patch
	//if (0 > xmd_ch_write(tty_ch->chno, (void *)buf, written_len)) {
	if (-9 == xmd_ch_write(tty_ch->chno, (void *)buf, written_len)) {
		
		printk("xmdtty:[RIL Recovery]xmd_ch_tty_write. err= -9\n");
		written_len = -9;
	}
#endif
/* LGE_UPDATE_START 2011.12.31_hyungsun.seo@lge.com_HSI pending issue during RIL Recovery*/

	return written_len;
}

#else /* CONFIG_MACH_LGE_COSMOPOLITAN */

static int xmd_ch_tty_write(
	struct tty_struct *tty,
	const unsigned char *buf,
	int len)
{
	struct xmd_ch_info *tty_ch = tty->driver_data;
// LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
#if defined(CONFIG_MACH_LGE_COSMO) || defined (CONFIG_MACH_LGE_CX2)
{
    if(simple_hsi_log_debug_enable == '1')
    {
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		printk("xmdtty: writing data of size %d to ch %d, data: %s\n", len,tty_ch->chno,str);
		kfree(str);
    }
}
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
{
    if(simple_hsi_log_debug_enable != '1')
    {
         char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
         memcpy(str, buf, len);
         printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",
                    len,tty_ch->chno,str);
         kfree(str);
    }
}
#endif
#else
        struct xmd_ch_info *tty_ch = tty->driver_data;
 // LGE_UPDATE_START 20120605 seunghwan.jin@lge.com
// start ipc temp

    char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
#if defined(HSI_PRIVATE_INFO_PROTECTION)
    if(len>PRIVATE_INFO_LENGTH+2)
     {
        memcpy(str, buf, PRIVATE_INFO_LENGTH);
        str[PRIVATE_INFO_LENGTH] ='\n';
        str[PRIVATE_INFO_LENGTH+1] ='\0';
     }
    else
        memcpy(str, buf, len);
#else
    memcpy(str, buf, len);
 #endif
    if(simple_hsi_log_debug_enable == '1')
        printk("xmdtty: writing data of size %d to ch %d, data: %s\n", len,tty_ch->chno,str);
    //else if(tty_ch->chno < 11)
      //  printk("xmdtty: CP received data size : %d to ch[%d] %s", len,tty_ch->chno,str);
    kfree(str);


// end ipc temp
#endif
// Original code is blocked. Maintain and unblock original code when you remove updated area.
/*
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	{
		char *str = (char *) kzalloc(len + 1, GFP_ATOMIC);
		memcpy(str, buf, len);
		printk("\nxmdtty: writing data of size %d to ch %d, data: %s\n",
					len,tty_ch->chno,str);
		kfree(str);
	}
#endif
*/
// LGE_UPDATE_END 20120605 seunghwan.jin@lge.com

	xmd_ch_write(tty_ch->chno, (void *)buf, len);

	return len;
}
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

static int xmd_ch_tty_write_room(struct tty_struct *tty)
{
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	struct xmd_ch_info *tty_ch = (struct xmd_ch_info*)tty->driver_data;

	printk("\nxmdtty: xmd_ch_tty_write_room [ch %d]\n", tty_ch->chno);
#endif

	if(xmd_is_recovery_state())
		return 0;

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

	return 8192;
}

static int xmd_ch_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

static void xmd_ch_tty_unthrottle(struct tty_struct *tty)
{
	return;
}

static struct tty_operations xmd_ch_tty_ops = {
	.open = xmd_ch_tty_open,
	.close = xmd_ch_tty_close,
	.write = xmd_ch_tty_write,
	.write_room = xmd_ch_tty_write_room,
	.chars_in_buffer = xmd_ch_tty_chars_in_buffer,
	.unthrottle = xmd_ch_tty_unthrottle,
};

static struct tty_driver *xmd_ch_tty_driver;

static int __init xmd_ch_tty_init(void)
{
	int ret, i;

#if defined (XMD_TTY_ENABLE_DEBUG_MSG)
	printk("\nxmdtty: xmd_ch_tty_init\n");
#endif
	xmd_ch_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (xmd_ch_tty_driver == 0) {
		return -ENOMEM;
	}

	xmd_ch_tty_driver->owner = THIS_MODULE;
	xmd_ch_tty_driver->driver_name = "xmd_ch_tty_driver";
	xmd_ch_tty_driver->name = "xmd-tty"; /* "ttyspi"; "xmd-tty"; */
	xmd_ch_tty_driver->major = 0;
	xmd_ch_tty_driver->minor_start = 0;
	xmd_ch_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	xmd_ch_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	xmd_ch_tty_driver->init_termios = tty_std_termios;
	xmd_ch_tty_driver->init_termios.c_iflag = 0;
	xmd_ch_tty_driver->init_termios.c_oflag = 0;
	xmd_ch_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	xmd_ch_tty_driver->init_termios.c_lflag = 0;
	xmd_ch_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
								TTY_DRIVER_REAL_RAW 	|
								TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(xmd_ch_tty_driver, &xmd_ch_tty_ops);

	ret = tty_register_driver(xmd_ch_tty_driver);
	if (ret) return ret;

	for (i = 0; i < tty_channels_len; i++)
		tty_register_device(xmd_ch_tty_driver, tty_channels[i].id, 0);

	xmd_ch_init();

// LGE_ADD_START 20120605 seunghwan.jin@lge.com
// START - dynamic log config
    hsi_fp = create_proc_entry("driver/hsi", 0777, NULL);
    if (hsi_fp) {
        hsi_fp->read_proc = read_hsi_proc;
        hsi_fp->write_proc = write_hsi_proc;
    }
// END - dynamic log config
// LGE_ADD_END 20120605 seunghwan.jin@lge.com

	return 0;
}

late_initcall(xmd_ch_tty_init);  //hyungsun.seo_111213 added for ICS
//module_init(xmd_ch_tty_init);
