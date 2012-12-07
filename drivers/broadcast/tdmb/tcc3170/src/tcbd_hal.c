
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "tcpal_debug.h"
#include "tcbd_hal.h"

#define DMB_EN          45 //GPIO 102
#define DMB_INT_N       44 //GPIO 107
#define DMB_RESET_N     62 //GPIO 101

void TchalInit(void)
{

	TcbdDebug(DEBUG_TCHAL, "TchalInit\n");
	//gpio_request(101, "DMB_RESET_N");
	//gpio_request(102, "DMB_EN");
	//gpio_request(107, "DMB_INT_N");
     gpio_request(DMB_RESET_N, "dmb_reset");
	 gpio_request(DMB_EN, "dmb_en");
	 gpio_request(DMB_INT_N, "dmb_int");
	 //gpio_request(DMB_EAR_ANT_SEL, "dmb_ear_ant");

	gpio_direction_output(DMB_RESET_N, false);
	gpio_direction_output(DMB_EN, false);
	//nthyunjin.yang for test gpio_direction_output(DMB_INT_N, false);
	gpio_direction_input(DMB_INT_N);//nthyunjin.yang for test
	printk("######### nthyunjin.yang TchalInit\n");
}

void TchalResetDevice(void)
{
    gpio_direction_output(DMB_RESET_N, false);

    gpio_set_value(DMB_RESET_N, 0);
    msleep(1);
    gpio_set_value(DMB_RESET_N, 1);
    msleep(1);
	printk("#########  TchaResetDevice\n");
}


void TchalPowerOnDevice(void)
{
    TcbdDebug(DEBUG_TCHAL, "TchalPowerOnDevice\n");

    gpio_direction_output(DMB_EN, false);
	gpio_direction_output(DMB_INT_N, false);
    gpio_set_value(DMB_INT_N, 0);
	
    gpio_set_value(DMB_EN, 0);
    msleep(1);
    gpio_set_value(DMB_EN, 1);
    msleep(1);

    TchalResetDevice();
	TchalIrqSetup();
	printk("######### TchalPowerOnDevice\n");
}


void TchalPowerDownDevice(void)
{
    TcbdDebug(DEBUG_TCHAL, "\n");

    gpio_set_value(DMB_EN, 0);
    gpio_set_value(DMB_RESET_N, 0);

	gpio_direction_output(DMB_INT_N, false);
	gpio_set_value(DMB_INT_N, 0);

	TchalIrqSetup();
	printk("######### TchalPowerDownDevice\n");
}

void TchalIrqSetup(void)
{
    gpio_direction_input(DMB_INT_N);
}

