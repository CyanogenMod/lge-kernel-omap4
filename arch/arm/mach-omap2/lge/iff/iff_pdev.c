/*
 * Platform Devices initializing.
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/input.h>
//                                                                             
#include <linux/lge/pwm-vibrator.h>
//                                                                             
#include <linux/lge/leds_keypad.h>
#include <plat/keypad.h>
#include <plat/omap-serial.h>
#include <lge/common.h>
#include <linux/switch_dp3t.h>
#include <linux/switch_usif.h>
#include <lge/board.h>
//                                                                                 
#if defined(CONFIG_GPS)
#include <linux/lge/gps_gpio.h>
#endif /* CONFIG_GPS */
//                                                                                 
#ifdef CONFIG_RAMOOPS
#include <linux/ramoops.h>
#endif

//                                                 
#include <linux/lbee9qmb-rfkill.h>
//---
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <lge/board_rev.h>
#include <mach-omap2/mux.h>

//                                                              
#define BT_UART_DEV_NAME "/dev/ttyO1"

static bool uart_req;
/* Call the uart disable of serial driver */
static int plat_bt_uart_disable(void)
{
        int port_id = 0;
        int err = 0;
        if (uart_req) {
                sscanf(BT_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        uart_req = false;
        }
        return err;
}

/* Call the uart enable of serial driver */
static int plat_bt_uart_enable(void)
{
        int port_id = 0;
        int err = 0;
        if (!uart_req) {
                sscanf(BT_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        uart_req = true;
        }
        return err;
}
//                                                              

//                                                                                                                                     
static struct lbee9qmb_platform_data lbee9qmb_platform = {
	.gpio_reset = GPIO_BT_RESET,
#ifdef CONFIG_BRCM_HOST_WAKE
	.gpio_hostwake = GPIO_BT_HOST_WAKE,
#endif
#ifdef CONFIG_BRCM_BT_WAKE
	.gpio_btwake = GPIO_BT_WAKE,
#endif
	.active_low = 1,
	.delay = 10,
//                                                              
        .chip_enable = plat_bt_uart_enable,
        .chip_disable = plat_bt_uart_disable,
//                                                              

};

static struct platform_device bcm4330_device = {
	.name = "lbee9qmb-rfkill",
	.dev = {
		.platform_data = &lbee9qmb_platform,
	},
};
//---

//                                          
//ADD: 0018892 [B2][BT] FactoryTest AT command
//                                                                            
struct platform_device bd_address_device = {
	.name = "bd_address",
	.id = -1,
};
//                                                                          
//                                        


/*                                                                           */
#if defined(CONFIG_SND_OMAP_SOC_HDMI)
static struct platform_device hdmi_audio_device = {
	.name = "hdmi-dai",
	.id	  = -1,
};
#endif
/*                                          */

//                                                                                 
#if defined(CONFIG_GPS)
//                                                                       
#define GPS_UART_DEV_NAME "/dev/ttyO2"

static bool gps_uart_req;
/* Call the uart disable of serial driver */
static int plat_gps_uart_disable(void)
{
        int port_id = 0;
        int err = 0;
        if (gps_uart_req) {
                sscanf(GPS_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        gps_uart_req = false;
        }
        return err;
}

/* Call the uart enable of serial driver */
static int plat_gps_uart_enable(void)
{
        int port_id = 0;
        int err = 0;
        if (!gps_uart_req) {
                sscanf(GPS_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        gps_uart_req = true;
        }
        return err;
}
//                                                                       

static struct gps_gpio_platform_data gps_pdata = {
	.pwron	 = GPS_PWR_ON_GPIO,
	.reset_n = GPS_RESET_N_GPIO,
//                                                                       
	.uart_dev_name = GPS_UART_DEV_NAME,
        .uart_enable = plat_gps_uart_enable,
        .uart_disable = plat_gps_uart_disable,
//                                                                       
};
static struct platform_device gps_gpio =
{
	.name = "gps_gpio",
	.id	  = -1,
	.dev = {
		.platform_data = &gps_pdata,
	}
};
#endif /* CONFIG_GPS */
//                                                                                 

//                                                                                     

extern int vibrator_power_control(bool on);

static struct pwm_vibrator_platform_data vib_data = {
	.freq			= 22000, // HZ
	.duty			= 99,	 // %
	.gpio_enable	= 25,
	.power			= vibrator_power_control,
	.port			= 8 	//                                                                 
};
//                                                                                     

static struct platform_device vib = {
	.name = VIB_PWM_NAME,
	.id	  = -1,
	.dev  = {
		.platform_data = &vib_data,
	},
};

/*                                                                           */
static struct leds_keypad_platform_data keypad_led_pdata = {
	.name          = "button-backlight",
	.use_hold_key  = 0,
	.keypad_gpio   = 82,
};

static struct platform_device keypad_led = {
	.name = "keypad_led",
	.id	  = -1,
	.dev = {
		.platform_data = &keypad_led_pdata,
	},
};

/*                                                                                  */
#if defined(CONFIG_LGE_MTC_ETA)
struct platform_device mtc_eta_log_device = {
	.name = "lge_mtc_eta_logger",
};
#endif
/*                                                 */


#ifdef CONFIG_RAMOOPS
static struct ramoops_platform_data ramoops_data = {
	.mem_size = SZ_1M,
	.mem_address = 0x80000000 + SZ_1G - SZ_1M,
};

static struct platform_device ramoops_device = {
	.name = "ramoops",
	.dev = {
		.platform_data = &ramoops_data,
	},
};
#endif

#if defined(CONFIG_LGE_HANDLE_PANIC)
static struct resource crash_log_resource[] = {
	.name = "crash_log",
	.flags = IORESOURCE_MEM,
	.start = LGE_CRASH_LOG_START,
	.end = LGE_CRASH_LOG_START + LGE_CRASH_LOG_SIZE - 1,
};

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.id = -1,
	.num_resources = 1,
	.resource = &crash_log_resource,
	.dev    = {
		.platform_data = NULL,
	}
};
#endif

struct dp3t_switch_platform_data dp3t_pdata = {
	.ctrl_gpio1 = GPIO_DP3T_IN_1,
	.ctrl_gpio2	= GPIO_DP3T_IN_2,
	.ctrl_ifx_vbus_gpio	= GPIO_IFX_USB_VBUS_EN,
};

static struct platform_device dp3t_dev = {
	.name = "switch-dp3t",
	.id	  = -1,
	.dev	= {	
		.platform_data = &dp3t_pdata,
	},
};

struct usif_switch_platform_data usif_pdata = {
	.ctrl_gpio = GPIO_USIF_IN_1,
};

static struct platform_device usif_dev = {
	.name = "switch-usif",
	.id	  = -1,
	.dev	= {	
		.platform_data = &usif_pdata,
	},
};

/*                                        
                                                                
                 
 */
static struct gpio_keys_button gpio_keys_buttons[] = {
	{
		.code = KEY_VOLUMEUP,
		.gpio = GPIO_KEY_VOLUMEUP,
		.desc = "volume up",
		//.active_low = 1,
	},
	{
		.code = KEY_VOLUMEDOWN,
		.gpio = GPIO_KEY_VOLUMEDOWN,
		.desc = "volume down",
		//.active_low = 1,
	},
};

static struct gpio_keys_platform_data gpio_keys_pdata = {
	.buttons = gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(gpio_keys_buttons),
	.rep = 0,
};

static struct platform_device gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &gpio_keys_pdata,
	},
};

static struct platform_device *pdevs[] __initdata = {
/*                                                                           */
#if defined(CONFIG_SND_OMAP_SOC_HDMI)
	&hdmi_audio_device,
#endif
/*                                          */
//                                                                                 
#if defined(CONFIG_GPS)
	&gps_gpio,
#endif /* CONFIG_GPS */
//                                                                                 
	&vib,
	&keypad_led,
	//&keypad_device,

/*                                                                                  */
#if defined(CONFIG_LGE_MTC_ETA)
	&mtc_eta_log_device,
#endif
/*                                                 */

	//                                                 
	&bcm4330_device,
//                                          
//ADD: 0018892 [B2][BT] FactoryTest AT command
	//                                                                            
	&bd_address_device,
	//                                                                          
//                                        
#ifdef CONFIG_RAMOOPS
	&ramoops_device,
#endif
#ifdef CONFIG_LGE_HANDLE_PANIC
	&panic_handler_device,
#endif
};

int __init iff_gpio_keys(void)
{
	/*                                        
                                                                 
                  
  */
	if (system_rev == LGE_PCB_C) {
		int ret;

		omap_mux_init_signal("kpd_row4.gpio_176",
				OMAP_PIN_OUTPUT);

		omap_mux_init_signal("kpd_row3.gpio_175",
				OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("kpd_col3.gpio_171",
				OMAP_PIN_INPUT_PULLDOWN);

		ret = gpio_request(GPIO_KEY_BASE, "gpio key baseline");
		if (ret < 0)
			pr_err("%s: gpio_request(%d) failed\n", __func__,
					GPIO_KEY_BASE);
		gpio_direction_output(GPIO_KEY_BASE, 1);

		platform_device_register(&gpio_keys_device);
	}

	return 0;
}
/* iff_gpio_key should be called after omap4_mux_init */
arch_initcall(iff_gpio_keys);

int __init iff_pdevs_init(void)
{
	return lge_set_pdevs(pdevs, ARRAY_SIZE(pdevs));
}

lge_machine_initcall(iff_pdevs_init);
