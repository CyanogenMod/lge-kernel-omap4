/*
 * SPI initializing code.
 *
 * Copyright (C) 2010 LG Electronic Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/ifx_n721_spi.h>
#include <plat/mcspi.h>
#include <lge/common.h>

#define ETH_KS8851_IRQ			34
#define ETH_KS8851_POWER_ON		48
#define ETH_KS8851_QUART		138

#if defined(CONFIG_SPI_IFX)
static void ifx_n721_dev_init(void)
{
	printk("[e]Board-4430: IFX_n721_DEV_INIT\n");

// ebs
#if 0
	if(gpio_request(IFX_SRDY_GPIO, "ifx srdy") < 0) {
		printk(KERN_ERR "Can't get SRDY GPIO\n");
		goto ifx_srdy_gpio_request_err;
	}
	if(gpio_request(IFX_MRDY_GPIO, "ifx mrdy") < 0) {
		printk(KERN_ERR "Can't get MRDY GPIO\n");
		goto ifx_mrdy_gpio_request_err;
	}

	if (gpio_request(MODEM_GPIO_AUDIO, "ifx audio") < 0) {
		printk(KERN_ERR "Can't get MODEM_AUDIO GPIO\n");
		goto modem_audio_gpio_request_err;
	}

	if (gpio_request(MODEM_GPIO_RESET, "ifx reset") < 0) {
		printk(KERN_ERR "Can't get MODEM_RESET GPIO\n");
		goto modem_reset_gpio_request_err;
	}
	if (gpio_request(MODEM_GPIO_PWRON, "ifx pwron") < 0) {
		printk(KERN_ERR "Can't get MODEM_PWRON GPIO\n");
		goto modem_pwron_gpio_request_err;
	}
	
	gpio_direction_input(IFX_SRDY_GPIO);  //gpio_119
	gpio_direction_output(IFX_MRDY_GPIO, 0);

	gpio_direction_output(MODEM_GPIO_AUDIO, 1);
	gpio_direction_output(MODEM_GPIO_RESET, 1);
	gpio_direction_output(MODEM_GPIO_PWRON, 1);

#endif


// ebs

modem_pwron_gpio_request_err:
	gpio_free(MODEM_GPIO_PWRON);
modem_reset_gpio_request_err:
#if 0
	gpio_free(MODEM_GPIO_AUDIO);
modem_audio_gpio_request_err:
#endif
	gpio_free(IFX_MRDY_GPIO);
ifx_mrdy_gpio_request_err:
	gpio_free(IFX_SRDY_GPIO);
ifx_srdy_gpio_request_err:
	return;
}

static struct omap2_mcspi_device_config ifxn721_mcspi_config = {
	.turbo_mode		 = 0,
	.single_channel	 = 1,	/* 0: slave, 1: master */
#if 0
	.mode			 = OMAP2_MCSPI_MASTER,
	.dma_mode		 = 0,
	.force_cs_mode	 = 0,
	.fifo_depth		 = 0,
#endif
};
#else
#define ifx_n721_dev_init	NULL
#endif /* CONFIG_SPI_IFX */

static struct spi_board_info spi_bd_info[] __initdata = {
	{
		.modalias		 = "ks8851",
		.bus_num		 = 1,
		.chip_select	 = 0,
		.max_speed_hz	 = 24000000,
		.irq			 = ETH_KS8851_IRQ,
	},
#if defined(CONFIG_SPI_IFX)
	{
		.modalias		 = "ifxn721",
		.bus_num		 = 4,
		.chip_select	 = 0,
		.max_speed_hz	 = 24000000,
		.controller_data = &ifxn721_mcspi_config,
		.irq			 = OMAP_GPIO_IRQ(119),
	},
#endif /* CONFIG_SPI_IFX */	
};

int __init iff_spi_init(void)
{
//MIPI-HSI jjongjin.park@lge.com 2011.06.20 START
#if defined(CONFIG_SPI_IFX)
	int ret = 0;

	ret = lge_set_spi_board(spi_bd_info, ARRAY_SIZE(spi_bd_info));
	if (ret < 0)
		return ret;
	
	return lge_set_cp_init(ifx_n721_dev_init);
#endif
//MIPI-HSI jjongjin.park@lge.com 2011.06.20 END
	return 0;
};

lge_machine_initcall(iff_spi_init);
