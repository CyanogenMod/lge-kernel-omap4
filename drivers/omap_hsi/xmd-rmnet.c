/*
 * xmd_rmnet.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (C) 2011 Intel Mobile Communications. All rights reserved.
 * Author[xmd-hsi related changes only]: Chaitanya <Chaitanya.Khened@intel.com>
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
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <asm/byteorder.h>

#include "xmd-ch.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define POLL_DELAY 1000000 /* 1 second delay interval */

/* #define RMNET_DEBUG */
/* #define RMNET_CRITICAL_DEBUG */
#define RMNET_ERR

#define RMNET_WD_TMO		400
#define RMNET_DATAT_SIZE   	ETH_DATA_LEN /* This value should be changed according to android MTU setting*/
#define RMNET_MTU_SIZE		( ETH_HLEN + RMNET_DATAT_SIZE )
#define MAX_PART_PKT_SIZE   2500
#define MAX_PART_PKT_MTU    4096

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
/* #define RMNET_CHANGE_MTU */
#define RMNET_ARP_ENABLE

#if defined (RMNET_CHANGE_MTU)
#define RMNET_MIN_MTU		64
#define RMNET_MAX_MTU		4096
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

typedef enum {
	RMNET_FULL_PACKET,
	RMNET_PARTIAL_PACKET,
	RMNET_PARTIAL_HEADER,
} RMNET_PAST_STATE;

struct xmd_ch_partial_packet{
	RMNET_PAST_STATE state;
//	char buf[MAX_PART_PKT_SIZE];
    char* buf;
	int size;
	int type;
    int allocated_buf_len;
};

static struct xmd_ch_partial_packet past_packet[MAX_SMD_NET] = {
	{RMNET_FULL_PACKET,  NULL,  0, 0, 0},
    {RMNET_FULL_PACKET,  NULL,  0, 0, 0},
    {RMNET_FULL_PACKET,  NULL,  0, 0, 0},
};
#if 0    
static struct xmd_ch_info rmnet_channels[MAX_SMD_NET] = {
	{0,  "CHANNEL13",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
	{1,  "CHANNEL14",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
	{2,  "CHANNEL15",  0, XMD_NET, NULL, 0, SPIN_LOCK_UNLOCKED},
};
#else   //hyungsun.seo_111213 added for ICS
static struct xmd_ch_info rmnet_channels[MAX_SMD_NET] = {
	{0,  "CHANNEL13",  0, XMD_NET, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{1,  "CHANNEL14",  0, XMD_NET, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
	{2,  "CHANNEL15",  0, XMD_NET, NULL, 0, __SPIN_LOCK_UNLOCKED(.lock)},
};
#endif
struct rmnet_private {
	struct xmd_ch_info *ch;
	struct net_device_stats stats;
	const char *chname;
	struct wake_lock wake_lock;
	struct ethhdr eth_hdr;
	int ip_type;
#ifdef CONFIG_MSM_RMNET_DEBUG
	ktime_t last_packet;
	short active_countdown; /* Number of times left to check */
	short restart_count; /* Number of polls seems so far */
	unsigned long wakeups_xmit;
	unsigned long wakeups_rcv;
	unsigned long timeout_us;
	unsigned long awake_time_ms;
	struct delayed_work work;
#endif
};

static struct {
	int blocked;
	struct net_device *dev;
} rmnet_ch_block_info[16];

#if defined (RMNET_ARP_ENABLE)
//20110721, ramesh.chandrasekaran@teleca.com, B-IFX enhancement
//Description: arp response structure
struct arp_resp {
	__be16          ar_hrd;         /* format of hardware address   */
	__be16          ar_pro;         /* format of protocol address   */
	unsigned char   ar_hln;         /* length of hardware address   */
	unsigned char   ar_pln;         /* length of protocol address   */
	__be16          ar_op;          /* ARP opcode (command)         */
	unsigned char           ar_sha[ETH_ALEN];       /* sender hardware address      */
	unsigned char           ar_sip[4];              /* sender IP address            */
	unsigned char           ar_tha[ETH_ALEN];       /* target hardware address      */
	unsigned char           ar_tip[4];              /* target IP address            */
};
#endif

#if defined (RMNET_ERR)
#define RMNET_COL_SIZE 30
#define RMNET_ERROR_STR_SIZE 20

//static unsigned int rmnet_cnt = 0;
#endif

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]	
#if 0 //defined (RMNET_ERR)
static void xmd_net_dump(const unsigned char *txt, const unsigned char *buf, int len)
{
	char dump_buf_str[(RMNET_COL_SIZE+1)*3] = {0,};

	int index = 0;
	char ch = 0;
	char* cur_str = dump_buf_str;

	if ((buf != NULL) && (len >= 0))
	{
		while(index < RMNET_COL_SIZE)
		{
			if(index < len)
			{
				ch = buf[index];
				sprintf(cur_str, "x%.2x", ch);
			}
			else
			{
				sprintf(cur_str, "$$$");
			}

			cur_str = cur_str+3;
			index++;
		}

		*cur_str = 0;
		printk("%s: rmnet_cnt [%d]th : len [%d] buf [%s]\n", txt, rmnet_cnt, len, dump_buf_str);
		rmnet_cnt++;
	}
}
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP)) {
		return 0;
	}

	return 1;
}

#ifdef CONFIG_MSM_RMNET_DEBUG
static int in_suspend;
static unsigned long timeout_us;
static struct workqueue_struct *rmnet_wq;

static void do_check_active(struct work_struct *work)
{
	struct rmnet_private *p =
		container_of(work, struct rmnet_private, work.work);

	/*
	 * Soft timers do not wake the cpu from suspend.
	 * If we are in suspend, do_check_active is only called once at the
	 * timeout time instead of polling at POLL_DELAY interval. Otherwise the
	 * cpu will sleeps and the timer can fire much much later than POLL_DELAY
	 * casuing a skew in time calculations.
	 */
	if (in_suspend) {
		/*
		 * Assume for N packets sent durring this session, they are
		 * uniformly distributed durring the timeout window.
		 */
		int tmp = p->timeout_us * 2 -
			(p->timeout_us / (p->active_countdown + 1));
		tmp /= 1000;
		p->awake_time_ms += tmp;

		p->active_countdown = p->restart_count = 0;
		return;
	}

	/*
	 * Poll if not in suspend, since this gives more accurate tracking of
	 * rmnet sessions.
	 */
	p->restart_count++;
	if (--p->active_countdown == 0) {
		p->awake_time_ms += p->restart_count * POLL_DELAY / 1000;
		p->restart_count = 0;
	} else {
		queue_delayed_work(rmnet_wq, &p->work,
				usecs_to_jiffies(POLL_DELAY));
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * If early suspend is enabled then we specify two timeout values,
 * screen on (default), and screen is off.
 */
static unsigned long timeout_suspend_us;
static struct device *rmnet0;

/* Set timeout in us when the screen is off. */
static ssize_t timeout_suspend_store(
	struct device *d,
	struct device_attribute *attr,
	const char *buf, size_t n)
{
	timeout_suspend_us = simple_strtoul(buf, NULL, 10);
	return n;
}

static ssize_t timeout_suspend_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%lu\n", (unsigned long) timeout_suspend_us);
}

static DEVICE_ATTR (timeout_suspend,
					0664,
					timeout_suspend_show,
					timeout_suspend_store);

static void rmnet_early_suspend(struct early_suspend *handler)
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
        if (!p) {
#if defined (RMNET_ERR)
            printk("rmnet_early_suspend: No netdev_priv \n");
#endif
            return;
        }
		p->timeout_us = timeout_suspend_us;
	}
	in_suspend = 1;
}

static void rmnet_late_resume(struct early_suspend *handler)
{
	if (rmnet0) {
		struct rmnet_private *p = netdev_priv(to_net_dev(rmnet0));
        if (!p) {
#if defined (RMNET_ERR)
            printk("rmnet_late_resume: No netdev_priv \n");
#endif
            return;
        }
		p->timeout_us = timeout_us;
	}
	in_suspend = 0;
}

static struct early_suspend rmnet_power_suspend = {
	.suspend = rmnet_early_suspend,
	.resume = rmnet_late_resume,
};

static int __init rmnet_late_init(void)
{
	register_early_suspend(&rmnet_power_suspend);
	return 0;
}

late_initcall(rmnet_late_init);
#endif

/* Returns 1 if packet caused rmnet to wakeup, 0 otherwise. */
static int rmnet_cause_wakeup(struct rmnet_private *p)
{
	int ret = 0;
	ktime_t now;
	if (p->timeout_us == 0) /* Check if disabled */
		return 0;

	/* Start timer on a wakeup packet */
	if (p->active_countdown == 0) {
		ret = 1;
		now = ktime_get_real();
		p->last_packet = now;

		if (in_suspend) {
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(p->timeout_us));
		} else {
			queue_delayed_work(rmnet_wq, &p->work,
					usecs_to_jiffies(POLL_DELAY));
		}
	}

	if (in_suspend) {
		p->active_countdown++;
	} else {
		p->active_countdown = p->timeout_us / POLL_DELAY;
	}

	return ret;
}

static ssize_t wakeups_xmit_show(struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
    if (!p) {
#if defined (RMNET_ERR)
        printk("wakeups_xmit_show: No netdev_priv \n");
#endif
        return 0;
    }
	return sprintf(buf, "%lu\n", p->wakeups_xmit);
}

DEVICE_ATTR(wakeups_xmit, 0444, wakeups_xmit_show, NULL);

static ssize_t wakeups_rcv_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
    if (!p) {
#if defined (RMNET_ERR)
        printk("wakeups_rcv_show: No netdev_priv \n");
#endif
        return 0;
    }
	return sprintf(buf, "%lu\n", p->wakeups_rcv);
}

DEVICE_ATTR(wakeups_rcv, 0444, wakeups_rcv_show, NULL);

/* Set timeout in us. */
static ssize_t timeout_store(
	struct device *d,
	struct device_attribute *attr,
	const char *buf, size_t n)
{
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
    if (!p) {
#if defined (RMNET_ERR)
        printk("timeout_store: No netdev_priv \n");
#endif
        return 0;
    }
	p->timeout_us = timeout_us = simple_strtoul(buf, NULL, 10);
#else
	/* If using early suspend/resume hooks do not write the value on store. */
	timeout_us = simple_strtoul(buf, NULL, 10);
#endif
	return n;
}

static ssize_t timeout_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
//	struct rmnet_private *p = netdev_priv(to_net_dev(d));
//	p = netdev_priv(to_net_dev(d));
	return sprintf(buf, "%lu\n", timeout_us);
}

DEVICE_ATTR(timeout, 0664, timeout_show, timeout_store);

/* Show total radio awake time in ms */
static ssize_t awake_time_show(
	struct device *d,
	struct device_attribute *attr,
	char *buf)
{
	struct rmnet_private *p = netdev_priv(to_net_dev(d));
    if (!p) {
#if defined (RMNET_ERR)
        printk("awake_time_show: No netdev_priv \n");
#endif
        return 0;
    }
	return sprintf(buf, "%lu\n", p->awake_time_ms);
}
DEVICE_ATTR(awake_time_ms, 0444, awake_time_show, NULL);

#endif

#define RMNET_ARP_VER 0x2
#define RMNET_IPV6_VER 0x6
#define RMNET_IPV4_VER 0x4

/*give the packet to TCP/IP*/
static void xmd_trans_packet(
	struct net_device *dev,
	int type,
	void *buf,
	int sz)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = NULL;
    if (!p) {
#if defined (RMNET_ERR)
        printk("xmd_trans_packet: No netdev_priv \n");
#endif
        return;
    }

	sz += ETH_HLEN; /* 14byte ethernet header should be added */

#if defined (RMNET_CRITICAL_DEBUG)
	printk("\nRMNET: %d<\n",sz);
#endif

	if ((type != RMNET_IPV4_VER) && (type != RMNET_IPV6_VER )
#if defined (RMNET_ARP_ENABLE)
		&& (type != RMNET_ARP_VER )
#endif		
		) {
#if defined (RMNET_ERR)
		printk("rmnet:xmd_trans_packet() invalid type %x \n", type);
#endif
		p->stats.rx_errors++;
		return;
	}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (RMNET_CHANGE_MTU)
	if (sz > p->mtu)
#else
	if (sz > RMNET_MTU_SIZE)
#endif
	{
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
#if defined (RMNET_ERR)
		printk("rmnet:xmd_trans_packet() discarding %d pkt len\n", sz);
#endif
		ptr = 0;
		p->stats.rx_errors++;
		return;
	}
	else {
		skb = dev_alloc_skb(sz + NET_IP_ALIGN);
		if (skb == NULL) {
#if defined (RMNET_ERR)
			printk("rmnet:xmd_trans_packet() cannot allocate skb\n");
#endif
			p->stats.rx_dropped++;
			return;
		}
		else {
			skb->dev = dev;
			skb_reserve(skb, NET_IP_ALIGN);
			ptr = skb_put(skb, sz);
            if(ptr == NULL) {
#if defined (RMNET_ERR)
                printk("rmnet:xmd_trans_packet() skb_put fails\n");
#endif
                p->stats.rx_dropped++;
                dev_kfree_skb (skb);
                return;
            }
                
			wake_lock_timeout(&p->wake_lock, HZ / 2);

			/* adding ethernet header */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]	
#if 0
			{
				char temp[] = {0xB6,0x91,0x24,0xa8,0x14,0x72,0xb6,0x91,0x24,0xa8,0x14,0x72,0x08,0x0};
				struct ethhdr *eth_hdr = (struct ethhdr *) temp;

				if (type == RMNET_IPV6_VER) {
					eth_hdr->h_proto = htons(ETH_P_IPV6);
				}
#if defined (RMNET_ARP_ENABLE)
				else if (type == RMNET_ARP_VER) {
					eth_hdr->h_proto = htons(ETH_P_ARP);
				}
#endif				
				else /* RMNET_IPV4_VER */
				{
					eth_hdr->h_proto = htons(ETH_P_IP);
				}

				memcpy((void *)eth_hdr->h_dest,
					   (void*)dev->dev_addr,
					   sizeof(eth_hdr->h_dest));
				memcpy((void *)ptr,
					   (void *)eth_hdr,
					   sizeof(struct ethhdr));
			}
#else
			if (type != p->ip_type)
			{
				if (type == RMNET_IPV6_VER) {
					p->eth_hdr.h_proto = htons(ETH_P_IPV6);
				}
#if defined (RMNET_ARP_ENABLE)				
				else if (type == RMNET_ARP_VER) {
					p->eth_hdr.h_proto = htons(ETH_P_ARP);
				}
#endif
				else /* RMNET_IPV4_VER */
				{
					p->eth_hdr.h_proto = htons(ETH_P_IP);
				}

				p->ip_type = type;
			}
			
			memcpy((void *)ptr, (void *)&p->eth_hdr, ETH_HLEN);
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

			memcpy(ptr + ETH_HLEN, buf, sz - ETH_HLEN);

			skb->protocol = eth_type_trans(skb, dev);
			if (count_this_packet(ptr, skb->len)) {
#ifdef CONFIG_MSM_RMNET_DEBUG
				p->wakeups_rcv += rmnet_cause_wakeup(p);
#endif
				p->stats.rx_packets++;
				p->stats.rx_bytes += skb->len;
			}
			netif_rx(skb);
			wake_unlock(&p->wake_lock);
		}
	}
}

static void rmnet_reset_pastpacket_info(int ch)
{
	if(ch >= MAX_SMD_NET ) {
#if defined (RMNET_DEBUG)
		printk("\nrmnet:Invalid rmnet channel number %d\n", ch);
#endif
		return;
	}	
	past_packet[ch].state = RMNET_FULL_PACKET;
	// reset the buffer
	if(past_packet[ch].buf != NULL) {
        kfree(past_packet[ch].buf);
    }
    past_packet[ch].buf = (char*)kmalloc((MAX_PART_PKT_SIZE * sizeof(char)), GFP_ATOMIC);
	if(past_packet[ch].buf != NULL) {
	    memset(past_packet[ch].buf, 0 , MAX_PART_PKT_SIZE);
	    past_packet[ch].allocated_buf_len = MAX_PART_PKT_SIZE;
	} else {
		// memory allocation failed. not good for the system. But we will retry
		// while processing the packet once more.
#if defined (RMNET_ERR)
		printk("rmnet_reset_pastpacket_info: failed to allocate buffer sz - %d \n", MAX_PART_PKT_SIZE);
#endif
	    past_packet[ch].allocated_buf_len = 0;
	}
	past_packet[ch].size = 0;
	past_packet[ch].type = 0;
}

/* return 0 if successful else -1 */
static int xmd_adjust_past_packet_buffer(struct rmnet_private *p, int id, int req_mem_size)
{
    char* new_buf = NULL;
    int new_buf_size = (past_packet[id].size + req_mem_size);
    if(new_buf_size > MAX_PART_PKT_MTU) {
        // packet size is too much.... can't handle it.
#if defined (RMNET_ERR)
        printk("xmd_adjust_past_packet_buffer: partial pkt size %d exceed max limit %d\n",
                new_buf_size, MAX_PART_PKT_MTU);
#endif
        past_packet[id].state = RMNET_FULL_PACKET;
        p->stats.rx_errors++;
        return -1;
    }
    new_buf = (char*)kmalloc((new_buf_size * sizeof(char)), GFP_ATOMIC);
    if(new_buf != NULL) {
        // buffer is allocated successfully.
        // first copy the contained.
        memset(new_buf, 0, new_buf_size);
        memcpy(new_buf, past_packet[id].buf, past_packet[id].size);
        // free the previous memory and change the buffer location
        kfree(past_packet[id].buf);
        past_packet[id].buf = new_buf;
        // change the current buffer max size.
        past_packet[id].allocated_buf_len = new_buf_size;
    } else {
        // unable to allocate memory. dropping the packet.
#if defined (RMNET_ERR)
        printk("xmd_adjust_past_packet_buffer: partial pkt mem alloc failed of size %d\n",
                new_buf_size);
#endif
        past_packet[id].state = RMNET_FULL_PACKET;
        p->stats.rx_errors++;
        return -1;
    }
    return 0;
}

/* Called in wq context */
static void xmd_net_notify(int chno)
{
	int i;
	struct net_device *dev = NULL;
	void *buf = NULL;
	int tot_sz = 0;
	struct rmnet_private *p = NULL;
	struct xmd_ch_info *info = NULL;
    int available_buf = 0;
	for (i=0; i<ARRAY_SIZE(rmnet_channels); i++) {
		if (rmnet_channels[i].chno == chno)	{
			dev = (struct net_device *)rmnet_channels[i].priv;
			break;
		}
	}

	if (!dev) {
#if defined (RMNET_ERR)
		printk("xmd_net_notify: No device \n");
#endif
		return;
	}

	p = netdev_priv(dev);
	if (!p) {
#if defined (RMNET_ERR)
		printk("xmd_net_notify: No netdev_priv \n");
#endif
		return;
	}

	info = p->ch;
	if (!info) {
#if defined (RMNET_ERR)
		printk("xmd_net_notify: No info \n");
#endif
		return;
	}
	if (info->id < 0 || info->id >= MAX_SMD_NET) {
#if defined (RMNET_ERR)
		printk("xmd_net_notify: Invalid id - %d\n", info->id);
#endif
		return;
	}

	/* contains the full data read from hsi channel.*/
	buf = xmd_ch_read(info->chno, &tot_sz);

	if (!buf) {
#if defined (RMNET_ERR)
		printk("xmd_net_notify: No buf recvd from ch:%d \n", info->chno);
#endif
		return;
	}
	
#if defined (RMNET_DEBUG)
	printk("xmd_net_notify: total size read = %d from ch:%d \n",
			tot_sz, info->chno);
#endif

	switch (past_packet[info->id].state)
	{
	case RMNET_FULL_PACKET:
		/* no need to do anything */
	break;

	case RMNET_PARTIAL_PACKET:
	{
		void *ip_hdr = (void *)past_packet[info->id].buf;
		int sz;
		int copy_size;

#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: past partial packet\n");
#endif
		if (past_packet[info->id].type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet[info->id].type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
			printk("xmd_net_notify: Invalid past version (data), %d\n",
					past_packet[info->id].type);
#endif
			past_packet[info->id].state = RMNET_FULL_PACKET;
			p->stats.rx_errors++;
			return;
		}
        available_buf = past_packet[info->id].allocated_buf_len - past_packet[info->id].size;
		copy_size = sz - past_packet[info->id].size;

		 /* if read size if > then copy size, copy full packet.*/
		if (tot_sz >= copy_size) {
            if(available_buf < copy_size) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, copy_size) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
            memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
            if(available_buf < tot_sz) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, tot_sz) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,tot_sz);
#if defined (RMNET_DEBUG)
			printk("\nxmd_net_notify: RMNET_PARTIAL_PACKET. past size = %d, total size = %d\n",
						past_packet[info->id].size, tot_sz);
#endif
			past_packet[info->id].size += tot_sz;
			return;
		}

		xmd_trans_packet(dev,past_packet[info->id].type,(void*)past_packet[info->id].buf,sz);
#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: pushed reassembled data packet to tcpip, sz = %d\n", sz);
#endif
		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet[info->id].state = RMNET_FULL_PACKET;
	}
	break;

	case RMNET_PARTIAL_HEADER:
	{
		void *ip_hdr = (void *)past_packet[info->id].buf;
		int sz;
		int copy_size;
		int hdr_size = 0;

#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: past partial header packet\n");
#endif
		if (past_packet[info->id].type == RMNET_IPV4_VER)
			hdr_size = sizeof(struct iphdr);
		else if (past_packet[info->id].type  == RMNET_IPV6_VER)
			hdr_size = sizeof(struct ipv6hdr);
		else
		{
#if defined (RMNET_ERR)
			printk("xmd_net_notify: Invalid past version (hdr), %d\n",
					past_packet[info->id].type);
#endif
			past_packet[info->id].state = RMNET_FULL_PACKET;
			p->stats.rx_errors++;
			return;
		}

        available_buf = past_packet[info->id].allocated_buf_len - past_packet[info->id].size;
		copy_size = hdr_size - past_packet[info->id].size;

		if(tot_sz >= copy_size) {
            if(available_buf < copy_size) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, copy_size) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size. */
            if(available_buf < tot_sz) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, tot_sz) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,tot_sz);
#if defined (RMNET_DEBUG)
			printk("xmd_net_notify: Still partial header \n");
#endif
			past_packet[info->id].size += tot_sz;
			return;
		}

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;
		past_packet[info->id].size = past_packet[info->id].size + copy_size;

		if (past_packet[info->id].type == RMNET_IPV4_VER) {
			sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (past_packet[info->id].type == RMNET_IPV6_VER) {
			sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
			printk("xmd_net_notify: Invalid past version, %d\n",
						past_packet[info->id].type);
#endif
			past_packet[info->id].state = RMNET_FULL_PACKET;
			p->stats.rx_errors++;
			return;
		}

        available_buf = past_packet[info->id].allocated_buf_len - past_packet[info->id].size;
		copy_size = sz - past_packet[info->id].size;

		 /* if read size if > then copy size, copy full packet. */
		if (tot_sz >= copy_size) {
            if(available_buf < copy_size) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, copy_size) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,copy_size);
		} else {
			/* copy whatever read if read size < packet size.*/
            if(available_buf < tot_sz) {
                // allocate extra buffer to accodomate the data.
                if(xmd_adjust_past_packet_buffer(p, info->id, tot_sz) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			memcpy(past_packet[info->id].buf + past_packet[info->id].size,buf,tot_sz);
#if defined (RMNET_DEBUG)
			printk("\nxmd_net_notify: RMNET_PARTIAL_HEADER. past size = %d, total size = %d\n",
						past_packet[info->id].size, tot_sz);
#endif
			past_packet[info->id].size += tot_sz;
			past_packet[info->id].state = RMNET_PARTIAL_PACKET;
			return;
		}

		xmd_trans_packet(dev,past_packet[info->id].type,(void *)past_packet[info->id].buf,sz);

		buf = buf + copy_size;
		tot_sz = tot_sz - copy_size;

	}
	break;

	default:
#if defined (RMNET_ERR)
		printk("xmd_net_notify: Invalid past state %d\n",
				(int)past_packet[info->id].state);
#endif
		past_packet[info->id].state = RMNET_FULL_PACKET;
		p->stats.rx_errors++;
		break;
	}

	while (tot_sz > 0) {
		int hdr_size = 0;
		int ver = 0;
		void *ip_hdr = (void *)buf;
		int data_sz = 0;

#if defined(__BIG_ENDIAN_BITFIELD)
		ver = ((char *)buf)[0] & 0x0F;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
		ver = (((char *)buf)[0] & 0xF0) >> 4;
#endif

#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: ver = 0x%x, total size : %d \n", ver, tot_sz);
#endif

		if (ver == RMNET_IPV4_VER) {
			hdr_size = sizeof(struct iphdr);
		} else if (ver == RMNET_IPV6_VER) {
			hdr_size = sizeof(struct ipv6hdr);
		} else {

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]	
#if 1
			/***********************************************************************
				1. Case of "+PBREADY" : RIL recovery is needed
					Modem resets because of low battery without any modem's notification 
			***********************************************************************/
			if(NULL != strnstr((char*)buf, "+PBREADY", 12)) {
								
#if defined (RMNET_ERR)
				printk("xmd_net_notify: +PBREADY Left packet size= %d\n", tot_sz);
#endif
				past_packet[info->id].state = RMNET_FULL_PACKET;

#if 0
				/* Start RIL recovery */
				ifx_schedule_cp_dump_or_reset();
#endif
			}
			
			/**********************************************************************************
				1. Case of "NO CARRIER"
					PDP is disconnected and modem sends "NO CARRIER"
					
				2. Case of "ERROR"
					Left packet data in XMD DLP queue is transimited after network or PDP is disconnected.
					The data  is sent to CAT (modem) and CAT responds "Error"

				3. TODO: Other cases should be checked
			**********************************************************************************/
			else {				
#if defined (RMNET_ERR)				
				char buf_str[RMNET_ERROR_STR_SIZE];

				memset(buf_str, 0x00, RMNET_ERROR_STR_SIZE);
				memcpy(buf_str, buf, min(RMNET_ERROR_STR_SIZE, tot_sz));

				printk("xmd_net_notify: ch:%d Invalid version, 0x%x\n", info->chno, ver);
				printk("xmd_net_notify: Few bytes of pkt : 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
					((char *)buf)[0], ((char *)buf)[1], ((char *)buf)[2],
					((char *)buf)[3], ((char *)buf)[4], ((char *)buf)[5],
					((char *)buf)[6], ((char *)buf)[7], ((char *)buf)[8],
					((char *)buf)[9], ((char *)buf)[10]);
				printk("xmd_net_notify: Converted strings %s\n", buf_str);
				printk("xmd_net_notify: Left packet size = %d, \n", tot_sz);
#endif
			past_packet[info->id].state = RMNET_FULL_PACKET;

			}
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
#endif
		break;
		}

		if (tot_sz < hdr_size) {
            // Although header is very small, safe to check buffer
            // because allocation may fail during reset.
            if(past_packet[info->id].allocated_buf_len < tot_sz) {
                // last time failed to allocate buffer. Retry once more.
                past_packet[info->id].size = 0; // reset the size... we are at the begining
                if(xmd_adjust_past_packet_buffer(p, info->id, tot_sz) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			past_packet[info->id].state = RMNET_PARTIAL_HEADER;
			past_packet[info->id].size = tot_sz;
			memcpy(past_packet[info->id].buf, buf, tot_sz);
			past_packet[info->id].type = ver;
#if defined (RMNET_DEBUG)
			printk("xmd_net_notify: partial header packet copied locally, sz = %d\n",
					tot_sz);
#endif
			return;
		}

		if (ver == RMNET_IPV4_VER) {
			data_sz = ntohs(((struct iphdr*) ip_hdr)->tot_len);
		} else if (ver == RMNET_IPV6_VER) {
			data_sz = ntohs(((struct ipv6hdr*) ip_hdr)->payload_len) + sizeof(struct ipv6hdr);
		} else {
#if defined (RMNET_ERR)
			printk("xmd_net_notify: data sz check -- Invalid version, %d\n",ver);
#endif
			past_packet[info->id].state = RMNET_FULL_PACKET;
			p->stats.rx_errors++;
			break;
		}

#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: data size = %d\n", data_sz);
#endif

		// RIP-12303 : [HSI-RMNET] cannot exit from an infinite loop in the xmd_net_notify if the data_sz in IP header is zero. START
		if (data_sz <= 0) {
#if defined (RMNET_ERR)
			printk("xmd_net_notify: invalid data size(%d). Dropping packets\n", data_sz);
#endif
			break;
		}
		// RIP-12303 : [HSI-RMNET] cannot exit from an infinite loop in the xmd_net_notify if the data_sz in IP header is zero. END

		if (tot_sz < data_sz) {
            if(past_packet[info->id].allocated_buf_len < tot_sz) {
                // allocate extra buffer to accodomate the data.
                past_packet[info->id].size = 0; // reset the size... we are at the begining
                if(xmd_adjust_past_packet_buffer(p, info->id, tot_sz) != 0) {
                    // failed to allocated extra buffer.. parameters are already reset
                    return;
                }
            }
			past_packet[info->id].state = RMNET_PARTIAL_PACKET;
			past_packet[info->id].size = tot_sz;
			memcpy(past_packet[info->id].buf, buf, tot_sz);
			past_packet[info->id].type = ver;
#if defined (RMNET_DEBUG)
			printk("xmd_net_notify: partial data packet copied locally, sz = %d\n",
					tot_sz);
#endif
			return;
		}

		xmd_trans_packet(dev, ver, buf, data_sz);
#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: pushed full data packet to tcpip, sz = %d\n",
				data_sz);
#endif
		tot_sz = tot_sz - data_sz;
		buf = buf + data_sz;
#if defined (RMNET_DEBUG)
		printk("xmd_net_notify: looping for another packet tot_sz = %d\n",
				tot_sz);
#endif
	}
	past_packet[info->id].state = RMNET_FULL_PACKET;
	// treat the packet is full. next packet starts from begining.
	// safe to reset size to 0
	past_packet[info->id].size = 0;
}

static int rmnet_open(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = NULL;

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_open: No netdev_priv \n");
#endif
		return -ENODEV;
	}

	info = p->ch;
	
	if (!info) {
#if defined (RMNET_ERR)
		printk("rmnet_open: No xmd_ch_info \n");
#endif
		return -ENODEV;
	}

	info->chno = xmd_ch_open(info, xmd_net_notify);

	if (info->chno < 0) {
#if defined (RMNET_ERR)
		printk("rmnet_open: error ch %d \n", info->chno);
#endif
		info->chno = 0;
		return -ENODEV;
	}
	
	printk("rmnet_open: ch %d \n", info->chno);

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = NULL;

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_stop: No netdev_priv \n");
#endif
		return -ENODEV;
	}
	
	info = p->ch;
		
	if (!info) {
#if defined (RMNET_ERR)
		printk("rmnet_stop: No xmd_ch_info \n");
#endif
		return -ENODEV;
	}

	printk("rmnet_stop() ch %d \n", info->chno);
	
	netif_stop_queue(dev);
	xmd_ch_close(info->chno);
	rmnet_reset_pastpacket_info(info->id);
	
	return 0;
}

void rmnet_restart_queue(int chno)
{
	if(rmnet_ch_block_info[chno].blocked) {
		rmnet_ch_block_info[chno].blocked = 0;
		netif_wake_queue(rmnet_ch_block_info[chno].dev);
#if defined (RMNET_DEBUG)
		printk("rmnet: FIFO free so unblocking rmnet %d queue\n", chno);
#endif
	}
}

#if defined (RMNET_ARP_ENABLE)
void fake_arp_response(struct net_device *dev, struct sk_buff *skb)
{
	struct arp_resp *arp_resp, *arp_req;
	char macAddr[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
	
	arp_resp = (struct arp_resp *) kmalloc(sizeof(struct arp_resp), GFP_ATOMIC);

	if(arp_resp == NULL)
	{
		printk("rmnet:fake_arp_response() cannot allocate \n");
		return;
	}
	
	memset(arp_resp, sizeof(struct arp_resp), 0);

	arp_req = (struct arp_resp *) (skb->data + ETH_HLEN);

	arp_resp->ar_hrd = arp_req->ar_hrd;
	arp_resp->ar_pro = arp_req->ar_pro;
	arp_resp->ar_hln = arp_req->ar_hln;
	arp_resp->ar_pln = arp_req->ar_pln;
	arp_resp->ar_op = htons(2);

	memcpy((void *)arp_resp->ar_sha, (void *)macAddr, ETH_HLEN);
	memcpy((void *)arp_resp->ar_sip,(void *)arp_req->ar_tip , 4);
	memcpy((void *)arp_resp->ar_tha,(void *)arp_req->ar_sha, ETH_HLEN);
	memcpy((void *)arp_resp->ar_tip,(void *)arp_req->ar_sip, 4);

	xmd_trans_packet(dev, RMNET_ARP_VER, (void *)arp_resp, sizeof(struct arp_resp));
	
	kfree(arp_resp);
}
#endif

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = NULL;
	int ret = NETDEV_TX_OK;

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_xmit: No netdev_priv \n");
#endif
		goto ok_xmit;
	}

	info = p->ch;
		
	if (!info) {
#if defined (RMNET_ERR)
		printk("rmnet_xmit: No xmd_ch_info \n");
#endif
		goto ok_xmit;
	}

#if defined (RMNET_CRITICAL_DEBUG)
	printk("\nRMNET[%d]: %d>\n", info->chno, skb->len);
#endif

	if((skb->len - ETH_HLEN) <= 0) {
#if defined (RMNET_ERR)
		printk("\nrmnet: Got only header for ch %d, return\n", info->chno);
#endif
		goto ok_xmit;
	}

/* Check IPv4 & IPv6 */
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]	
#if 0 //defined (RMNET_ERR)
	if ((skb->protocol != htons(ETH_P_IP)) && (skb->protocol != htons(ETH_P_IPV6))
#if defined (RMNET_ARP_ENABLE)		
		&& (skb->protocol != htons(ETH_P_ARP))
#endif		
		) {

		xmd_net_dump("rmnet_xmit", (char*)skb->data, skb->len);
	}
#endif
#if 0 //defined (RMNET_ERR)
	if ((skb->data[ETH_HLEN] != 0x45) && (skb->data[ETH_HLEN] != 0x60)) {

		xmd_net_dump("rmnet_xmit", (char*)skb->data, skb->len);
	}
#endif	
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#if defined (RMNET_ARP_ENABLE)
	if (skb->protocol == htons(ETH_P_ARP)) {
		fake_arp_response(dev, skb);
		goto ok_xmit;
	}
#endif

	/* ETH_HLEN(14) is the size of ethernet header which is being stripped */
	if ((ret = xmd_ch_write(info->chno,(void *)((char *) skb->data + ETH_HLEN), skb->len - ETH_HLEN)) != 0) {

		p->stats.tx_errors++;
		
		if(ret == -ENOMEM) {
			ret = NETDEV_TX_BUSY;
#if defined (RMNET_ERR)
			printk("\nrmnet: Cannot alloc mem, so returning busy for ch %d\n", info->chno);
#endif
			goto quit_xmit;
		} else if(ret == -EBUSY) {
			netif_stop_queue(dev);
			rmnet_ch_block_info[info->chno].dev = dev;
			rmnet_ch_block_info[info->chno].blocked = 1;
#if defined (RMNET_ERR)
			printk("\nrmnet: Stopping queue for ch %d\n", info->chno);
#endif
			ret = NETDEV_TX_BUSY;
			goto quit_xmit;
		}
		else {
#if defined (RMNET_ERR)
			printk("\nrmnet: xmd_ch_write etc error for ch %d\n", info->chno);
#endif
		}
			
	} else {
		if (count_this_packet(skb->data, skb->len)) {
			p->stats.tx_packets++;
			p->stats.tx_bytes += skb->len;
#ifdef CONFIG_MSM_RMNET_DEBUG
			p->wakeups_xmit += rmnet_cause_wakeup(p);
#endif
		}
	}
	
ok_xmit:
	ret = NETDEV_TX_OK;
	dev_kfree_skb_irq(skb);

quit_xmit:
	return ret;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_get_stats: No netdev_priv \n");
#endif
        return NULL;
	}

	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = NULL;

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_set_multicast_list: No netdev_priv \n");
#endif
		return;
	}

	info = p->ch;
			
	if (!info) {
#if defined (RMNET_ERR)
		printk("rmnet_set_multicast_list: No xmd_ch_info \n");
#endif
		return;
	}

	printk("rmnet_set_multicast_list ch %d \n", info->chno);

}

static void rmnet_tx_timeout(struct net_device *dev)
{
	int chno;

	for(chno=13; chno < 16; chno++) {
		if(rmnet_ch_block_info[chno].dev == dev) {
			rmnet_restart_queue(chno);
			printk("rmnet_tx_timeout()ch %d \n", chno);
			break;
		}
	}
}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (RMNET_CHANGE_MTU)
/* Netdevice change MTU request */
static int rmnet_nd_change_mtu(struct net_device *dev, int new_mtu)
{
	struct rmnet_private *p = netdev_priv(dev);
	struct xmd_ch_info *info = NULL;

	if (!p) {
#if defined (RMNET_ERR)
		printk("rmnet_nd_change_mtu: No netdev_priv \n");
#endif
		return -EINVAL;
	}

	info = p->ch;
			
	if (!info) {
#if defined (RMNET_ERR)
		printk("rmnet_nd_change_mtu: No xmd_ch_info \n");
#endif
		return -EINVAL;
	}

	printk("rmnet_nd_change_mtu ch %d new_mtu [%d]\n", info->chno, new_mtu);

	if (new_mtu < RMNET_MIN_MTU || new_mtu > RMNET_MAX_MTU)
		return -EINVAL;

	p->mtu = new_mtu;

	return 0;
}
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

static struct net_device_ops rmnet_ops = {
	.ndo_open = rmnet_open,
	.ndo_stop = rmnet_stop,
	.ndo_start_xmit = rmnet_xmit,
	.ndo_get_stats = rmnet_get_stats,
	.ndo_set_multicast_list = rmnet_set_multicast_list,
	.ndo_tx_timeout = rmnet_tx_timeout,
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
#if defined (RMNET_CHANGE_MTU)
	.ndo_change_mtu = rmnet_nd_change_mtu,
#endif
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]
};

static void __init rmnet_setup(struct net_device *dev)
{
	dev->netdev_ops = &rmnet_ops;

	dev->watchdog_timeo = RMNET_WD_TMO;

	ether_setup(dev);

	dev->mtu = RMNET_MTU_SIZE;

#if !defined (RMNET_ARP_ENABLE)
	dev->flags |= IFF_NOARP;
#endif

	random_ether_addr(dev->dev_addr);
}

// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
static void rmnet_set_ip4_ethr_hdr(struct net_device *dev, struct rmnet_private *p)
{
	unsigned char faddr[ETH_ALEN] = { 0xb6, 0x91, 0x24, 0xa8, 0x14, 0x72 };

	memcpy((void *)p->eth_hdr.h_dest,
		(void *)dev->dev_addr, ETH_ALEN);

	memcpy((void *)p->eth_hdr.h_source, 
		(void *)faddr, ETH_ALEN);
	
	p->eth_hdr.h_proto = htons(ETH_P_IP);
	
	p->ip_type = RMNET_IPV4_VER;
}
// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

static int __init rmnet_init(void)
{
	int ret;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

#ifdef CONFIG_MSM_RMNET_DEBUG
	struct device *d;

	timeout_us = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
	timeout_suspend_us = 0;
#endif
#endif

#ifdef CONFIG_MSM_RMNET_DEBUG
	rmnet_wq = create_workqueue("rmnet");
#endif

	for (n = 0; n < MAX_SMD_NET; n++) {
		rmnet_reset_pastpacket_info(n);
		dev = alloc_netdev(sizeof(struct rmnet_private),
				   "rmnet%d", rmnet_setup);

		if (!dev) {
#if defined (RMNET_ERR)
			printk("rmnet_init: alloc_netdev fail \n");
#endif	
			return -ENOMEM;
		}

#ifdef CONFIG_MSM_RMNET_DEBUG
		d = &(dev->dev);
#endif
		p = netdev_priv(dev);

		if (!p) {
#if defined (RMNET_ERR)
			printk("rmnet_init: No netdev_priv \n");
#endif
    		return -ENODEV;
		}

		rmnet_channels[n].priv = (void *)dev;
		p->ch = rmnet_channels + n;
		p->chname = rmnet_channels[n].name;
		wake_lock_init(&p->wake_lock,
						WAKE_LOCK_SUSPEND,
						rmnet_channels[n].name);

		// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [START]
		rmnet_set_ip4_ethr_hdr(dev, p);
		// LGE_CHANGE [MIPI-HSI] jaesung.woo@lge.com [END]

#ifdef CONFIG_MSM_RMNET_DEBUG
		p->timeout_us = timeout_us;
		p->awake_time_ms = p->wakeups_xmit = p->wakeups_rcv = 0;
		p->active_countdown = p->restart_count = 0;
		INIT_DELAYED_WORK_DEFERRABLE(&p->work, do_check_active);
#endif

		ret = register_netdev(dev);
		if (ret) {
#if defined (RMNET_ERR)
			printk("rmnet_init: register_netdev fail \n");
#endif			
			free_netdev(dev);
			return ret;
		}

#ifdef CONFIG_MSM_RMNET_DEBUG
		if (device_create_file(d, &dev_attr_timeout))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_xmit))
			continue;
		if (device_create_file(d, &dev_attr_wakeups_rcv))
			continue;
		if (device_create_file(d, &dev_attr_awake_time_ms))
			continue;
#ifdef CONFIG_HAS_EARLYSUSPEND
		if (device_create_file(d, &dev_attr_timeout_suspend))
			continue;

		/* Only care about rmnet0 for suspend/resume tiemout hooks. */
		if (n == 0)
			rmnet0 = d;
#endif
#endif
	}

	return 0;
}

late_initcall(rmnet_init);  //hyungsun.seo_111213 added for ICS
//module_init(rmnet_init);
