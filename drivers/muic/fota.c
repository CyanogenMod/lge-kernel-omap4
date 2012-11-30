/*
 * Cosmo FOTA driver
 *
 * Copyright (C) 2010, 2012 LGE, Inc.
 *
 * Author: Chulhwheeshim <Chulhwhee.shim@lge.com>
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
#include <linux/kernel.h>	/* printk() */
#include <linux/init.h>		/* __init, __exit */
#include <linux/uaccess.h>	/* copy_from/to_user() */
#include <linux/interrupt.h>	/* request_irq() */
#include <linux/irq.h>		/* set_irq_type() */
#include <linux/types.h>	/* kernel data types */
#include <asm/system.h>
#include <asm/gpio.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/delay.h>	/* usleep() */
#include <linux/proc_fs.h>
#include <linux/workqueue.h>	/* INIT_WORK() */
#include <linux/wakelock.h>
#include <linux/spi/ifx_n721_spi.h>
//TIME INFORMATION ADD. 2011-05-02 eunae.kim
#include <linux/rtc.h>

/* LGE_SJIT 2012-02-07 [dojip.kim@lge.com]
 * FIXME: redesign gpio definitions
 */
#if defined ( TARGET_CARRIER_LGU )
#define GPIO_MDM_PWR_ON	22
#define GPIO_MDM_RESET	142
#define GPIO_LOW        0
#define GPIO_HIGH       1
#define MODEM_READY_CTRL_GPIO 122

static int gpio_mdm_pwron = GPIO_MDM_PWR_ON;
static int gpio_mdm_pwron_sw = GPIO_MDM_RESET;
#else
static int gpio_mdm_pwron = MODEM_GPIO_PWRON;
static int gpio_mdm_pwron_sw = MODEM_GPIO_PWRON_SW;
#endif


#define LGE_FOTA_TEST_PROC_FILE "driver/fota"

extern int fota_ebl_download(void);
extern void usif_switch_ctrl(int);
extern void dp3t_switch_ctrl(int);
//USIF can't switch
extern void usif_switch_none(int);
//USIF can't switch

void cp_power_down(void)
{
	pr_info("fota: %s\n", __func__);
	// PMU Reset Pin Low
	gpio_direction_output(gpio_mdm_pwron, 0);
	udelay(100);
	gpio_set_value(gpio_mdm_pwron, 0);
	// Power On Pin Low
	gpio_direction_output(gpio_mdm_pwron_sw, 0);
	udelay(100);
	gpio_set_value(gpio_mdm_pwron_sw, 0);
}

void ifx_uart_sw_ctrl(void)
{
	pr_info("fota: %s\n", __func__);
	usif_switch_ctrl(0);
	// dp3t_switch_ctrl(0);
}

void ifx_power_low(void)
{
	int status;

	gpio_direction_output(gpio_mdm_pwron, 0);
	mdelay(500);
	gpio_set_value(gpio_mdm_pwron, 0);
	status = gpio_get_value(gpio_mdm_pwron);
	pr_info("fota: ifx_power_low - [CP POWER]: pin %d\n", status);
	pr_info("fota: %s: FOTA_proc_excute \n", __func__);
}

void ifx_power_high(void)
{
	int status;
	pr_info("fota: %s: FOTA_proc_excute \n", __func__);
	gpio_direction_output(gpio_mdm_pwron, 1);
	mdelay(500);
	gpio_set_value(gpio_mdm_pwron, 1);
	status = gpio_get_value(gpio_mdm_pwron);
	pr_info("fota: ifx_power_high - [CP POWER]: pin %d\n", status);
}

void ifx_reset_low(void)
{
}

void ifx_reset_high(void)
{
}

void ifx_fota_reset(void)
{
#if defined ( TARGET_CARRIER_LGU )
int status;

/*
   usif_switch_ctrl(0);
   ifx_reset_low();
   fota_ebl_download();
*/
	pr_info("%s: FOTA_proc_excute in  \n", __func__);
	
   	if( gpio_request( GPIO_MDM_PWR_ON, "mdm_pwr_on" ) != 0 )
		pr_info("TS0710: %s : Warning : GPIO_MDM_PWR_ON may EBUSY\n", __func__);
	if( gpio_request( GPIO_MDM_RESET, "reset_mdm" ) != 0 )
		pr_info("TS0710: %s : Warning : GPIO_MDM_RESET may EBUSY\n", __func__);
		
	gpio_direction_output( GPIO_MDM_PWR_ON, GPIO_LOW );
	gpio_direction_output( GPIO_MDM_RESET,	GPIO_LOW );
	pr_info("###** MODEM_RESET: Current Reset, %d\n", gpio_get_value(GPIO_MDM_RESET));
	pr_info("###** MODEM_RESET: Current Power, %d\n", gpio_get_value(GPIO_MDM_PWR_ON));	
	msleep(200);

	gpio_set_value( GPIO_MDM_PWR_ON, GPIO_HIGH );
	pr_info("###** MODEM_RESET: After Set High of Power, %d\n", gpio_get_value(GPIO_MDM_PWR_ON));
	msleep(400);

	gpio_set_value( GPIO_MDM_RESET, GPIO_HIGH );
	pr_info("###** MODEM_RESET: After Set High of Reset, %d\n", gpio_get_value(GPIO_MDM_RESET));
	msleep(300);
	
	gpio_set_value( GPIO_MDM_RESET, GPIO_LOW );
	pr_info("###** MODEM_RESET: After Set Low of Reset, %d\n", gpio_get_value(GPIO_MDM_RESET));	
	
	usif_switch_ctrl(0);
	pr_info("###** usif_switch_ctrl %d\n", gpio_get_value(92));
	usif_switch_none(0); //USIF can't switch
   gpio_free( GPIO_MDM_RESET );
   gpio_free( GPIO_MDM_PWR_ON );
	msleep(2500);
	
	/*
	msleep(500);
	gpio_set_value( GPIO_MDM_PWR_ON, GPIO_LOW );
	gpio_free( GPIO_MDM_PWR_ON );	
*/
	//msleep(3000);	
	//gpio_free( GPIO_MDM_RESET );	
       //gpio_free( GPIO_MDM_PWR_ON );	
	
	pr_info("%s: FOTA_proc_excute out \n", __func__);
#else
	int status;

	printk("%s: CP Reset IN\n", __func__);

	gpio_request(gpio_mdm_pwron, "ifx pwron");
	gpio_direction_output(gpio_mdm_pwron, 0);
	udelay(100);
	gpio_set_value(gpio_mdm_pwron, 0);
	status = gpio_get_value(gpio_mdm_pwron);
	printk("%s: MODEM_GPIO_PWRON low- [CP POWER]: pin %d\n", __func__, status);

	mdelay(500); // 500mS delay
	
	gpio_direction_output(gpio_mdm_pwron_sw, 0);  //GPIO_3 OE
	udelay(100);
	gpio_set_value(gpio_mdm_pwron_sw, 0);
	status = gpio_get_value(gpio_mdm_pwron_sw);
	printk("%s: MODEM_GPIO_PWRON_SW low- [CP POWER]: pin %d\n", __func__, status);
	
	mdelay(500); // 500mS delay

	gpio_set_value(gpio_mdm_pwron, 1);
	status = gpio_get_value(gpio_mdm_pwron);
	printk("%s: MODEM_GPIO_PWRON high+ [CP POWER]: pin %d\n", __func__, status);

	mdelay(100); // 1000mS delay

	gpio_set_value(gpio_mdm_pwron_sw, 1);
	status = gpio_get_value(gpio_mdm_pwron_sw);
	printk("%s: MODEM_GPIO_PWRON_SW high+ [CP POWER]: pin %d\n", __func__, status);

	usif_switch_ctrl(0);
/* for compile error : Not defined TARGET_CARRIER_LGU 
    usif_switch_none(0); //USIF can't switch
*/
	printk("%s: CP Reset OUT\n", __func__);
#endif
}

void ifx_pmu_reset(void)
{
	int status;

	// TIME INFORMATION ADD. 2011-05-02 eunae.kim
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	pr_info("fota: %s:", __func__);
	pr_info("fota: (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

	gpio_direction_output(gpio_mdm_pwron, 0);
	mdelay(200);
	gpio_set_value(gpio_mdm_pwron, 0);
	status = gpio_get_value(gpio_mdm_pwron);
	printk("ifx_power_low- [CP POWER]: pin %d\n", status);
	mdelay(200);
	gpio_set_value(gpio_mdm_pwron, 1);
	status = gpio_get_value(gpio_mdm_pwron);
	printk("ifx_power_high- [CP POWER]: pin %d\n", status);
	//cheolgwak 2011 02 01
	gpio_direction_output(gpio_mdm_pwron_sw, 0);
	udelay(100);
	gpio_set_value(gpio_mdm_pwron_sw, 0);
	mdelay(200);		//wait 10ms
	gpio_set_value(gpio_mdm_pwron_sw, 1);
	mdelay(200);
	//cheolgwak 2011 02 01
}
#if defined ( TARGET_CARRIER_LGU )
void ifx_fota_ready_low(void)
{
	gpio_set_value(MODEM_READY_CTRL_GPIO, 0);
	gpio_direction_output(MODEM_READY_CTRL_GPIO, 0);
	pr_info("###** ifx_fota_ready_low (122pin): status %d\n",gpio_get_value(MODEM_READY_CTRL_GPIO));
	
}
void ifx_fota_ready_high(void)
{
	gpio_set_value(MODEM_READY_CTRL_GPIO, 1);
	gpio_direction_output(MODEM_READY_CTRL_GPIO, 1);
	pr_info("###** ifx_fota_ready_high (122pin): status %d\n",gpio_get_value(MODEM_READY_CTRL_GPIO));
	
}
#endif
static ssize_t fota_test_proc_read(struct file *filp, char *buf, size_t len,
				   loff_t * offset)
{
	return 1;
}

static ssize_t fota_test_proc_write(struct file *filp, const char *buf,
				    size_t len, loff_t * off)
{
	char messages[10];
	char cmd;

	if (len > 10)
		len = 10;

	if (copy_from_user(messages, buf, len))
		return -EFAULT;

	sscanf(buf, "%c", &cmd);

	pr_info("fota: %s: FOTA_proc_write \n", __func__);

	switch (cmd) {
	case '0':
		ifx_power_low();
		break;
	case '1':
		ifx_power_high();
		break;
	case '2':
		ifx_reset_low();
		break;
	case '3':
		ifx_reset_high();
		break;
	case '4':
		ifx_uart_sw_ctrl();
		break;
	case '5':
		ifx_fota_reset();
		break;
	case '6':
		ifx_pmu_reset();
		break;
#if defined ( TARGET_CARRIER_LGU )
	case '7':
	       ifx_fota_ready_low();
            break;	
	case '8':
	       ifx_fota_ready_high();
            break;	
#endif
	default:
		pr_warning("fota: FOTA Driver invalid arg\n");
		break;
	}
	return len;
}

static struct file_operations lge_fota_test_proc_ops = {
	.read = fota_test_proc_read,
	.write = fota_test_proc_write,
};

static int create_lge_fota_test_proc_file(void)
{
	struct proc_dir_entry *proc;
	proc = create_proc_entry(LGE_FOTA_TEST_PROC_FILE, 0777, NULL);

	if (!proc) {
		pr_err("fota: %s: no memory\n", __func__);
		return -ENOMEM;
	}

	proc->proc_fops = &lge_fota_test_proc_ops;

	return 0;
}

static void remove_lge_fota_test_proc_file(void)
{
	remove_proc_entry(LGE_FOTA_TEST_PROC_FILE, NULL);
}

static int __init lge_fota_test_init(void)
{
	int ret;

	ret = gpio_request(gpio_mdm_pwron, "modem power");
	if (ret < 0) {
		pr_warn("fota: %s: gpio %d request failed\n", __func__,
				gpio_mdm_pwron);
	}
	gpio_request(gpio_mdm_pwron_sw, "modem power sw");
	if (ret < 0) {
		pr_warn("fota: %s: gpio %d request failed\n", __func__,
				gpio_mdm_pwron_sw);
	}
#if defined ( TARGET_CARRIER_LGU )
      	ret = gpio_request(MODEM_READY_CTRL_GPIO,  "modem fota ctrl");
      	if(ret < 0) {	
      		pr_info("MODEM_READY_CTRL_GPIO - can't get fota_ctrl GPIO\n");
      	}
       gpio_direction_output(MODEM_READY_CTRL_GPIO, 0);
	    pr_info("lge_fota_test_init  out\n");
#endif

	ret =  create_lge_fota_test_proc_file();

	if (ret < 0) {
		pr_err("fota: %s: proc create failed\n", __func__);
		return ret;
	}

	pr_info("fota: controler initialized\n");

	return 0;
}

static void __exit lge_fota_test_exit(void)
{
#if defined ( TARGET_CARRIER_LGU )
	pr_err("lge_fota_test_exit  in\n");
    gpio_free( MODEM_READY_CTRL_GPIO );
    //gpio_free( GPIO_MDM_RESET );	
    //gpio_free( GPIO_MDM_PWR_ON );	
#endif
	remove_lge_fota_test_proc_file();
}

module_init(lge_fota_test_init);
module_exit(lge_fota_test_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LGE FOTA Driver");
MODULE_LICENSE("GPL");
