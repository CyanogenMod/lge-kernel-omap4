/*
 * Cosmo MUIC driver
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * Author: Sookyoung Kim <sookyoung.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>			/* __init, __exit */
#include <linux/uaccess.h>		/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>			/* set_irq_type() */
#include <linux/types.h>		/* kernel data types */
#include <linux/syscalls.h>
#include <asm/system.h>

/*
 * kernel/arch/arm/include/asm/gpio.h includes kernel/arch/arm/plat-omap/include/mach/gpio.h which,
 * in turn, includes kernel/include/asm-generic/gpio.h.
 * <mach/gpio.h> declares gpio_get|set_value(), gpio_to_irq().
 * <asm-generic/gpio.h> declares struct gpio_chip, gpio_request(), gpio_free(), gpio_direction_input|output().
 * The actual descriptions are in kernel/drivers/gpio/gpiolib.c and kernel/arch/arm/plat-omap/gpio.c.
 */
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>		/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/mutex.h>

#include <linux/muic/muic.h>
//#include <linux/cosmo/cosmo_muic.h>
#include <linux/cosmo/charger_rt9524.h>

//#include <plat/lge_nvdata_handler.h>

struct i2c_client *muic_client;
static struct work_struct muic_wq;

TYPE_USIF_MODE usif_mode = USIF_AP;
TYPE_DP3T_MODE dp3t_mode = DP3T_NC;
TYPE_MUIC_MODE muic_path = MUIC_NONE;
TYPE_CHARGING_MODE charging_mode = CHARGING_NONE;
TYPE_UPON_IRQ  upon_irq  = NOT_UPON_IRQ;

static int retain_mode = NO_RETAIN;

atomic_t muic_charger_detected;
static int muic_chager_event = 0;

const char *muic_path_str[] = {
	"MUIC_UNKNOWN",			// 0
	"MUIC_NONE",   			// 1
	"MUIC_NA_TA",   		// 2
	"MUIC_LG_TA",   		// 3
	"MUIC_TA_1A", 	  		// 4
	"MUIC_INVALID_CHG",  	// 5
	"MUIC_AP_UART",   		// 6
	"MUIC_CP_UART",			// 7
	"MUIC_AP_USB", 			// 8
	"MUIC_CP_USB",			// 9
};

const char *charging_mode_str[] = {
	"CHARGING_UNKNOWN",
	"CHARGING_NONE",
	"CHARGING_NA_TA",
	"CHARGING_LG_TA",
	"CHARGING_TA_1A",
	"CHARGING_INVALID_CHG",
	"CHARGING_USB",
	"CHARGING_FACTORY",
};


void (*muic_init_device)(TYPE_RESET);
s32 (*muic_detect_accessory)(s32);

extern void muic_init_ts5usba33402(TYPE_RESET reset);
extern void muic_init_max14526(TYPE_RESET reset);

extern s32 muic_ts5usba33402_detect_accessory(s32 upon_irq);
extern s32 muic_max14526_detect_accessory(s32 upon_irq);

extern void ifx_fota_reset(void);

static s32 muic_proc_set_ap_uart(void);
static s32 muic_proc_set_cp_uart(void);
static s32 muic_proc_set_ap_usb(void);
static s32 muic_proc_set_cp_usb(void);




#ifdef CONFIG_PROC_FS
/*
 * --------------------------------------------------------------------
 *  BEGINS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#define	MUIC_PATH_PROC_FILE "driver/cmuic"
#define MUIC_INT_STAT_PROC_FILE "driver/muic_int_stat"

static struct proc_dir_entry *muic_path_proc_file;
static struct proc_dir_entry *muic_int_stat_proc_file;

static ssize_t muic_path_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
	s32 i;
	u32 val;

	for (i = 0; i <= 5; i++) {
		val = i2c_smbus_read_byte_data(muic_client, (u8)i);
		printk(KERN_INFO "[MUIC] reg 0x%02x, val = 0x%02x\n", i, val);
	}

	len = sprintf(buf, "%d\n", muic_path);
	printk(KERN_INFO "[MUIC] mode = %s\n", muic_path_str[muic_path]);

	return len;
}

static ssize_t muic_path_write(struct file *filp, const char *buf, size_t len, loff_t *off)
{
	u8 cmd;
	u8 dummy;

	u8 nv_muic_retain_mode[1] = {0x00};

	len = sscanf(buf, "%c %c", &cmd, &dummy);
	printk(KERN_INFO "[MUIC] LGE: muic_proc_write, cmd = %c, dummy = %c len = %d\n", cmd, dummy, len);

	switch (cmd) {
	case '0' :
		printk(KERN_INFO "[MUIC] CP usb download mode  \n");
		muic_proc_set_cp_usb();

		ifx_fota_reset();
		retain_mode = NO_RETAIN;
		muic_path = MUIC_CP_USB;
		mdelay(2000);
		printk(KERN_INFO "[MUIC] CP usb download mode  \n");
		break;

	case '1' :
		retain_mode = NO_RETAIN;
		nv_muic_retain_mode[0] = 0x00;
		//lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_MUIC_RETENTION_OFFSET, nv_muic_retain_mode, 1);
		break;

	case '2' :
		muic_proc_set_ap_uart();
		retain_mode = NO_RETAIN;
		muic_path = MUIC_AP_UART;
		break;

	case '3' :
		muic_proc_set_cp_uart();
		retain_mode = NO_RETAIN;
		muic_path = MUIC_CP_UART;
		break;

	case '4' :
		muic_proc_set_ap_usb();
		retain_mode = NO_RETAIN;
		muic_path = MUIC_AP_USB;
		break;

	case '5' :
		muic_proc_set_cp_usb();
		retain_mode = NO_RETAIN;
		muic_path = MUIC_CP_USB;
		break;

	case '6' :
		muic_proc_set_ap_uart();
		retain_mode = RETAIN;
		muic_path = MUIC_AP_UART;
		break;

	case '7' :
		muic_proc_set_cp_uart();
		retain_mode = RETAIN;
		muic_path = MUIC_CP_UART;
		break;

	case '8' :
		muic_proc_set_ap_usb();
		retain_mode = RETAIN;
		muic_path = MUIC_AP_USB;
		break;

	case '9' :
		muic_proc_set_cp_usb();
		retain_mode = RETAIN;
		muic_path = MUIC_CP_USB;
		break;

	case 'a' :
		/* AP USB retain mode after reboot */
		nv_muic_retain_mode[0] = 0x02;
		//lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_MUIC_RETENTION_OFFSET, nv_muic_retain_mode, 1);

		break;

	case 'b' :
		/* CP USB retain mode after reboot */
		nv_muic_retain_mode[0] = 0x03;
		//lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_MUIC_RETENTION_OFFSET, nv_muic_retain_mode, 1);

		break;

	default :
			printk(KERN_INFO "[MUIC] LGE: Cosmo MUIC invalid command\n");
			printk(KERN_INFO "[MUIC] 6: AP_UART, 7: CP_UART, 8: AP_USB, 9: CP_USB\n");
			break;
	}

	check_charging_mode();

	return len;
}

static ssize_t muic_int_stat_read(struct file *filp, char *buf, size_t len, loff_t *offset)
{
	u32 value;

	value = i2c_smbus_read_byte_data(muic_client, INT_STAT);
	printk(KERN_INFO "[MUIC] INT_STAT = 0x%02x\n", value);
	
	len = sprintf(buf, "%02x\n", 0xff & value);

	return len;
}

static struct file_operations muic_path_ops = {
	.read = muic_path_read,
	.write = muic_path_write,
};

static struct file_operations muic_int_stat_ops = {
	.read = muic_int_stat_read,	
};

static void create_cosmo_muic_proc_file(void)
{
	muic_path_proc_file = create_proc_entry(MUIC_PATH_PROC_FILE, 0666, NULL);
	muic_int_stat_proc_file = create_proc_entry(MUIC_INT_STAT_PROC_FILE, 0666, NULL);
	
	if (muic_path_proc_file) {
#if 0 // kernel 32
		cosmo_muic_proc_file->owner = THIS_MODULE;
#endif
		muic_path_proc_file->proc_fops = &muic_path_ops;
	} else
		printk(KERN_INFO "[MUIC] Cosmo MUIC path proc file create failed!\n");

	if (muic_int_stat_proc_file)
		muic_int_stat_proc_file->proc_fops = &muic_int_stat_ops;		
	else
		printk(KERN_INFO "[MUIC] Cosmo MUIC int_stat proc file create failed!\n");
		
}

static void remove_cosmo_muic_proc_file(void)
{
	remove_proc_entry(MUIC_PATH_PROC_FILE, NULL);
	remove_proc_entry(MUIC_INT_STAT_PROC_FILE, NULL);
}

/*
 * --------------------------------------------------------------------
 *  ENDS: Proc file system related functions for MUIC.
 * --------------------------------------------------------------------
 */
#endif //CONFIG_PROC_FS

void check_charging_mode(void)
{
	s32 value;

	value = i2c_smbus_read_byte_data(muic_client, INT_STAT);
	if (value & VBUS) {
		if ((value & IDNO) == IDNO_0010 || 
			(value & IDNO) == IDNO_0100 ||
			(value & IDNO) == IDNO_1001 ||
			(value & IDNO) == IDNO_1010)
			charging_mode = CHARGING_FACTORY;
		else if (value & CHGDET) 
			charging_mode = CHARGING_LG_TA;
		else
			charging_mode = CHARGING_USB;
	} else
		charging_mode = CHARGING_NONE;

	set_muic_charger_detected();
}
EXPORT_SYMBOL(check_charging_mode);

/*
 * Linux bug: If a constant value larger than 20000,
 * compiler issues a __bad_udelay error.
 */
void muic_mdelay(u32 microsec)
{
	do {
		udelay(1000);
	} while (microsec--);
}
EXPORT_SYMBOL(muic_mdelay);

// [jongho3.lee@lge.com] get muic mode for charger
TYPE_MUIC_MODE get_muic_mode(void)
{
	return charging_mode;
}
EXPORT_SYMBOL(get_muic_mode);

// hyoungsuk.jang@lge.com 20110110 MUIC mode change in case of trap [START]
void set_muic_mode(u32 mode)
{
	switch (mode) {
	case MUIC_AP_UART:	// 6
		muic_proc_set_ap_uart();
		break;
	case MUIC_CP_UART:	// 7
		muic_proc_set_cp_uart();
		break;
	case MUIC_AP_USB:	// 8
		muic_proc_set_ap_usb();
		break;
	case MUIC_CP_USB:	// 9
		muic_proc_set_cp_usb();
		break;
	default :
		break;
	}

	check_charging_mode();
}

EXPORT_SYMBOL(set_muic_mode);
// hyoungsuk.jang@lge.com 20110110 MUIC mode change in case of trap [END]

// LGE_CHANGE_S[jongho3.lee] get muic detecting for charger.
#if 0 // this is for wait event which is not unsing just now. block it temporary..
int get_muic_charger_detected(void)
{
	printk(KERN_WARNING "[MUIC] LGE: get_muic_charger_detected\n");

#if 1
	if (muic_chager_event ) {
		muic_chager_event = 0;
		return 1;
	}
#else
	if (muic_charger_detected.counter) {
		atomic_set(&muic_charger_detected,0);
		return 1;
	}
#endif

	return 0;
}
EXPORT_SYMBOL(get_muic_charger_detected);
#endif

// LGE_CHANGE_E[jongho3.lee] get muic detecting for charger.
void set_muic_charger_detected(void)
{
#if 0 // this is for wait event which is not unsing just now. block it temporary..
	//extern wait_queue_head_t muic_event;

	printk(KERN_WARNING "[MUIC] LGE: set_muic_charger_detected\n");

	//atomic_set(&muic_charger_detected,1);
	//wake_up_interruptible(&muic_event);
	muic_chager_event = 1;
	//wake_up(&muic_event);
#endif
	charger_fsm(CHARG_FSM_CAUSE_ANY);
}
EXPORT_SYMBOL(set_muic_charger_detected);

/*
 * Function: Read the MUIC register whose internal address is addr
 * 			and save the u8 content into value.
 * Return value: Negative errno upon failure, 0 upon success.
 */
#if 0 
s32 muic_i2c_read_byte(struct i2c_client * client, u8 addr, u8 *value)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(muic_client, (u8)addr);
	if (ret < 0) {
		printk("[MUIC] muic_i2c_read_byte failed.\n");
		return ret;
	} else {
		*value = (u8)ret;
		return 0;
	}
}
EXPORT_SYMBOL(muic_i2c_read_byte);

/*
 * Function: Write u8 value to the MUIC register whose internal address is addr.
 * Return value: Negative errno upon failure, 0 upon success.
 */
s32 muic_i2c_write_byte(struct i2c_client * client,u8 addr, u8 value)
{
	s32 ret;
	ret = i2c_smbus_write_byte_data(muic_client, (u8)addr, (u8)value);
	if (ret < 0)
		printk(KERN_INFO "[MUIC] muic_i2c_write_byte failed.\n");
	return ret;
}
EXPORT_SYMBOL(muic_i2c_write_byte);
#endif
void usif_switch_ctrl(TYPE_USIF_MODE mode)
{
	if (mode == USIF_AP) {
		gpio_set_value(USIF_IN_1_GPIO, 0);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to AP\n");
	} else if (mode == USIF_DP3T) {
		gpio_set_value(USIF_IN_1_GPIO, 1);
		printk(KERN_INFO "[MUIC] usif_switch_ctrl, CP UART is connected to DP3T (then, MUIC)\n");
	} else {
		/* Just keep the current path */
	}

	usif_mode = mode;
	//printk(KERN_WARNING "[MUIC] usif_switch_ctrl(): usif_mode = %d\n", usif_mode);
}
EXPORT_SYMBOL(usif_switch_ctrl);

void dp3t_switch_ctrl(TYPE_DP3T_MODE mode)
{
	if (mode == DP3T_AP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, AP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_UART) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP UART is connected to MUIC UART\n");
	} else if (mode == DP3T_CP_USB) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 1);
		gpio_set_value(DP3T_IN_1_GPIO, 1);
		gpio_set_value(DP3T_IN_2_GPIO, 1);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, CP USB is connected to MUIC UART\n");
	} else if (mode == DP3T_NC) {
		gpio_set_value(IFX_USB_VBUS_EN_GPIO, 0);
		gpio_set_value(DP3T_IN_1_GPIO, 0);
		gpio_set_value(DP3T_IN_2_GPIO, 0);
		printk(KERN_INFO "[MUIC] dp3t_switch_ctrl, None is connected to MUIC UART\n");
	} else {
		/* Just keep the current path */
	}

	dp3t_mode = mode;
	//printk(KERN_WARNING "[MUIC] dp3t_switch_ctrl(): dp3t_mode = %d\n", dp3t_mode);
}
EXPORT_SYMBOL(dp3t_switch_ctrl);

static s32 muic_proc_set_ap_uart(void)
{
	s32 ret;

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect AP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, DP_UART | DM_UART);

	muic_path = MUIC_AP_UART;
	charging_mode = CHARGING_UNKNOWN;

	printk("[MUIC] muic_proc_set_ap_uart(): AP_UART\n");

	return ret;
}

static s32 muic_proc_set_ap_usb(void)
{
	s32 ret;

	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* AP USB does not pass through DP3T.
	 * Just connect AP UART to MUIC UART.
	 */
	dp3t_switch_ctrl(DP3T_AP_UART);

	/* Connect DP, DM to USB_DP, USB_DM */
	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, DP_USB | DM_USB);

	muic_path = MUIC_AP_USB;
	charging_mode = CHARGING_USB;
	printk("[MUIC] muic_proc_set_ap_usb(): AP_USB\n");

	return ret;
}

static s32 muic_proc_set_cp_uart(void)
{
	s32 ret;

	/* Connect CP UART signals to DP3T */
	usif_switch_ctrl(USIF_DP3T);

	/* Connect CP UART to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_UART);

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, DP_UART | DM_UART);

	muic_path = MUIC_CP_UART;
	charging_mode = CHARGING_UNKNOWN;

	printk("[MUIC] muic_proc_set_cp_uart(): CP_UART\n");

	return ret;
}

static s32 muic_proc_set_cp_usb(void)
{
	s32 ret;

	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, OPEN);
	dp3t_switch_ctrl(DP3T_NC);
	muic_mdelay(100);

	/* Connect CP UART signals to AP */
	usif_switch_ctrl(USIF_AP);

	/* Connect CP USB to MUIC UART */
	dp3t_switch_ctrl(DP3T_CP_USB);

	/* Connect DP, DM to UART_TX, UART_RX */
	ret = muic_i2c_write_byte(muic_client,SW_CONTROL, DP_UART | DM_UART);

	muic_path = MUIC_CP_USB;
	charging_mode = CHARGING_USB;

	printk("[MUIC] muic_proc_set_cp_usb(): CP_USB\n");

	return ret;
}

static void muic_detect_device(void)
{
	s32 ret;
	u8 muic_device;

	printk(KERN_INFO "[MUIC] muic_detect_device()\n");

	ret = muic_i2c_read_byte(muic_client,DEVICE_ID, &muic_device);
	if ((muic_device & 0xf0) == TS5USBA33402)
		muic_device = TS5USBA33402;
	else if ((muic_device & 0xf0) == MAX14526)
		muic_device = MAX14526;
	else if ((muic_device & 0xf0) == ANY_VENDOR)
		muic_device = ANY_VENDOR;

	if (muic_device == TS5USBA33402) {
		muic_init_device = muic_init_ts5usba33402;
		muic_detect_accessory = muic_ts5usba33402_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip : TS5USBA33402\n");
	} else if (muic_device == MAX14526) {
		muic_init_device = muic_init_max14526;
		muic_detect_accessory = muic_max14526_detect_accessory;
		printk(KERN_INFO "[MUIC] muic chip : MAX14526\n");
	} else {
		printk(KERN_INFO "ANY VENDOR\n");
	}
}


static void muic_wq_func(struct work_struct *muic_wq)
{
	s32 ret = 0;

	printk(KERN_INFO "[MUIC] muic_wq_func()\n");

	if (retain_mode == NO_RETAIN) {
		ret = muic_detect_accessory(UPON_IRQ);
		printk(KERN_INFO "[MUIC] muic_detect_accessory(UPON_IRQ)detedted!!!!\n");
		set_muic_charger_detected();	// [jongho3.lee@lge.com]
	} else {
		muic_mdelay(250);

		ret = i2c_smbus_read_byte_data(muic_client, INT_STAT);
		if (muic_path == MUIC_CP_USB)
			muic_proc_set_cp_usb();
		check_charging_mode();

		set_muic_charger_detected();	
		printk(KERN_INFO "[MUIC] Now...path retain mode,  muic_path = %s, charing = %s\n", 
		   	   muic_path_str[muic_path], charging_mode_str[charging_mode]);
	}
}

static irqreturn_t muic_interrupt_handler(s32 irq, void *data)
{
	/* Make the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	schedule_work(&muic_wq);
	return IRQ_HANDLED;
}

/*
 * muic_probe() is called in the middle of booting sequence due to '__init'.
 * '__init' causes muic_probe() to be released after the booting.
 */
static s32 __init muic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = 0;

	muic_client = client;

	/* Initializes gpio_165 (USIF1_SW). */
	ret = gpio_request(USIF_IN_1_GPIO, "USIF switch control GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 165 USIF1_SW is already occupied by other driver!\n");
		/*
		 * We know board_cosmo.c:ifx_n721_configure_gpio() performs a gpio_request on this pin first.
		 * Because the prior gpio_request is also for the analog switch control, this is not a confliction.
		 */
		return -ENOSYS;
	}

	ret = gpio_direction_output(USIF_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_16 USIF_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	/*  Initializes gpio_11 (OMAP_UART_SW) and gpio_12 (IFX_UART_SW) */
	ret = gpio_request(DP3T_IN_1_GPIO, "DP3T switch control 1 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 11 DP3T_IN_1_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_1_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_11 DP3T_IN_1_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	ret = gpio_request(DP3T_IN_2_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_12 DP3T_IN_2_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(DP3T_IN_2_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_12 DP3T_IN_2_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	ret = gpio_request(IFX_USB_VBUS_EN_GPIO, "DP3T switch control 2 GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO is already occupied by other driver!\n");
		return -ENOSYS;
	}

	ret = gpio_direction_output(IFX_USB_VBUS_EN_GPIO, 0);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_55 IFX_USB_VBUS_EN_GPIO direction initialization failed!\n");
		return -ENOSYS;
	}

	/* Initializes gpio_wk8 (MUIC_INT_N) */
	ret = gpio_request(MUIC_INT, "MUIC IRQ GPIO");
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpiio_wk8 MUIC_INT_N is already occupied by other driver!\n");
		return -ENOSYS;
	}

	/* Initializes GPIO direction before use or IRQ setting */
	ret = gpio_direction_input(MUIC_INT);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_wk8 MUIC_INT_N direction initialization failed!\n");
		return -ENOSYS;
	}


	/* Registers MUIC work queue function */
	INIT_WORK(&muic_wq, muic_wq_func);

	/*
	 * Set up an IRQ line and enable the involved interrupt handler.
	 * From this point, a MUIC_INT_N can invoke muic_interrupt_handler().
	 * muic_interrupt_handler merely calls schedule_work() with muic_wq_func().
	 * muic_wq_func() actually performs the accessory detection.
	 */
	ret = request_irq(gpio_to_irq(MUIC_INT), muic_interrupt_handler, IRQF_TRIGGER_FALLING, "muic_irq", &client->dev);
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] gpio_wk7 MUIC_INT_N IRQ line set up failed!\n");
		free_irq(gpio_to_irq(MUIC_INT), &client->dev);
		return -ENOSYS;
	}

	/* Prepares a human accessible method to control MUIC */
	create_cosmo_muic_proc_file();

	/* Selects one of the possible muic chips */
	muic_detect_device();

	if (retain_mode == BOOT_FIRST) {
		muic_proc_set_cp_uart();
		muic_init_device(DEFAULT);
		check_charging_mode();
		printk("[MUIC] muic_init_device... first boot\n");
	} else if (retain_mode == BOOT_AP_USB) {
		muic_path = MUIC_AP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		printk("[MUIC] muic_init_device... retain mode = AP_USB\n");
	} else if (retain_mode == BOOT_CP_USB) {
		muic_proc_set_cp_usb();
		muic_path = MUIC_CP_USB;
		muic_init_device(DEFAULT);
		check_charging_mode();
		printk("[MUIC] muic_detect_accessory... retain mode = CP_USB\n");
	} else {
		if (muic_init_device)
			muic_init_device(DEFAULT);
		else
			printk("[MUIC] You should add codes for new MUIC chip");

		if (muic_detect_accessory)
			muic_detect_accessory(NOT_UPON_IRQ);
		else
			printk("[muic] You should add codes for new MUIC chip");
	}

	//[jongho3.lee@lge.com]
	set_muic_charger_detected();

	/* Makes the interrupt on MUIC INT wake up OMAP which is in suspend mode */
	ret = enable_irq_wake(gpio_to_irq(MUIC_INT));
	if (ret < 0) {
		printk(KERN_INFO "[MUIC] GPIO 40 MUIC_INT_N wake up source setting failed!\n");
		disable_irq_wake(gpio_to_irq(MUIC_INT));
		return -ENOSYS;
	}

	printk(KERN_WARNING "[MUIC] muic_probe()\n");

	return ret;
}

static s32 muic_remove(struct i2c_client *client)
{
	free_irq(gpio_to_irq(MUIC_INT), &client->dev);
	gpio_free(MUIC_INT);
	gpio_free(USIF_IN_1_GPIO);
	gpio_free(DP3T_IN_1_GPIO);
	gpio_free(DP3T_IN_2_GPIO);
	i2c_set_clientdata(client, NULL);	// For what?
	remove_cosmo_muic_proc_file();
	return 0;
}

static s32 muic_suspend(struct i2c_client *client, pm_message_t state)
{
	client->dev.power.power_state = state;

	return 0;
}

static s32 muic_resume(struct i2c_client *client)
{
	client->dev.power.power_state = PMSG_ON;

	return 0;
}

static const struct i2c_device_id muic_ids[] = {
	{"max14526" /* "cosmo_i2c_muic" */, 0},
	{/* end of list */},
};

//LGE_Changes_S chulhwhee.shim@lge.com, 2010.12.7  FOTA update
int fota_ebl_download(void)
{
   return 0;
}
//LGE_Changes_E chulhwhee.shim@lge.com, 2010.12.7  FOTA update

static int __init muic_state(char *str)
{
	int initial_path = simple_strtol(str, NULL, 0);

	if (initial_path == 1)
		retain_mode = BOOT_FIRST;
	else if (initial_path == 2)
		retain_mode = BOOT_AP_USB;
	else if (initial_path == 3)
		retain_mode = BOOT_CP_USB;
	else
		printk("[MUIC] no retain\n");

	printk("[MUIC] muic_state = %d\n", retain_mode);
	return 1;
}
//<jongho3.lee@lge.com> shorten command line..
__setup("muic=", muic_state);
//__setup("muic_state=", muic_state);


/*
 * Allow user space tools to figure out which devices this driver can control.
 * The first parameter should be 'i2c' for i2c client chip drivers.
 */
static struct i2c_driver muic_driver = {
	.probe	 	= muic_probe,
	.remove	 	= muic_remove,
	.suspend 	= muic_suspend,
	.resume  	= muic_resume,
	.id_table	= muic_ids,
	.driver	 	= {
		.name       = "max14526",	 //"cosmo_i2c_muic",
		.owner      = THIS_MODULE,
	},
};

static s32 __init muic_init(void)
{
    printk(KERN_WARNING "[MUIC] muic_init()\n");
	return i2c_add_driver(&muic_driver);
}

static void __exit muic_exit(void)
{
	i2c_del_driver(&muic_driver);
}

module_init(muic_init);
module_exit(muic_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Cosmo MUIC Driver");
MODULE_LICENSE("GPL");

