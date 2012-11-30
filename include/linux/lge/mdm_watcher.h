/*
 * Header file for Modem Watcher
 */

#ifndef __LGE_MDM_WATCHER_H
#define __LGE_MDM_WATCHER_H

#include <linux/interrupt.h>

#define MDM_WATCHER_NAME "mdm_watcher"

typedef enum {
	MDM_HALT,
	MDM_AUTO_SHUTDOWN,
#if !defined(CONFIG_LGE_SPI_SLAVE)
	MDM_HALT_SRDY,
	MDM_AUTO_SHUTDOWN_SRDY,
	MDM_HALT_MRDY,
	MDM_AUTO_SHUTDOWN_MRDY,	
#ifdef CONFIG_MACH_LGE_U2
	// RIP-41061 : handling interrupt for at-channel ready : byeonggeun.kim@
	MDM_HALT_MODEM_SEND,
	MDM_AUTO_SHUTDOWN_MODEM_SEND,
	// RIP-41061 : handling interrupt for at-channel ready : byeonggeun.kim@
#endif
#endif	
	MDM_EVENT_MAX,
} mdm_event_type;

struct mdm_watcher_event {
	mdm_event_type type;
	unsigned int gpio_irq;
	unsigned long irqf_flags;
	int msecs_delay;
	unsigned int key_code; /* key code to upper layer */
#if !defined(CONFIG_LGE_SPI_SLAVE)
	unsigned int gpio_irq_srdy;
	unsigned int gpio_irq_mrdy;
	unsigned int key_code_srdy;
	unsigned int key_code_mrdy;
	// RIP-41061 : handling interrupt for at-channel ready : byeonggeun.kim@lge.com [START]
	unsigned int gpio_irq_modem_send;
	unsigned int key_code_modem_send;
	// RIP-41061 : handling interrupt for at-channel ready : byeonggeun.kim@lge.com [END]
#endif
};

struct mdm_watcher_platform_data {
	struct mdm_watcher_event *event;
	unsigned len;
};

#endif /* __LGE_MDM_WATCHER_H */
