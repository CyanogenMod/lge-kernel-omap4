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
#include <linux/spi/mdm6600_spi.h>
#include <linux/spi/ifx_n721_spi.h>
#include <plat/mcspi.h>
#include <lge/common.h>

#if defined(CONFIG_LGE_BROADCAST_TDMB)
#define TDMB_T3900_IRQ		44
#endif /* CONFIG_LGE_BROADCAST_TDMB */

#if defined(CONFIG_SPI_IFX)
static void ifx_n721_dev_init(void)
{
	printk("[e]Board-4430: IFX_n721_DEV_INIT\n");


modem_pwron_gpio_request_err:
	gpio_free(MODEM_GPIO_PWRON);
modem_reset_gpio_request_err:
//#if 0
	gpio_free(MODEM_GPIO_AUDIO);
modem_audio_gpio_request_err:
//#endif
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
#endif /* CONFIG_SPI_IFX */

#if defined(CONFIG_LGE_SPI_SLAVE)
static void ifx_n721_dev_init(void)
{
	printk("[e]Board-4430: IFX_n721_DEV_INIT\n");


modem_pwron_gpio_request_err:
	gpio_free(MODEM_GPIO_PWRON);
modem_reset_gpio_request_err:
//#if 0
	gpio_free(MODEM_GPIO_AUDIO);
modem_audio_gpio_request_err:
//#endif
	gpio_free(IFX_MRDY_GPIO);
ifx_mrdy_gpio_request_err:
	gpio_free(IFX_SRDY_GPIO);
ifx_srdy_gpio_request_err:
	return;
}

static struct omap2_mcspi_device_config ifxn721_mcspi_config = {
	.turbo_mode		 = 0,
	.single_channel	 = 0,	/* 0: slave, 1: master */
#if 0
	.mode			 = OMAP2_MCSPI_MASTER,
	.dma_mode		 = 0,
	.force_cs_mode	 = 0,
	.fifo_depth		 = 0,
#endif
};
#else
#define ifx_n721_dev_init	NULL
#endif /* CONFIG_LGE_SPI_SLAVE */

#if defined(CONFIG_LGE_BROADCAST_TDMB)
static struct omap2_mcspi_device_config t3900_mcspi_config = {
	.turbo_mode 		= 0,
	.single_channel 	= 1,  /* 0 : slave, 1 : master */
};
#endif /* CONFIG_LGE_BROADCAST_TDMB */

static struct spi_board_info spi_bd_info[] __initdata = {
#if defined(CONFIG_LGE_BROADCAST_TDMB)
	{
		.modalias		= "tdmb_t3900",
		.bus_num	= 1,
		.chip_select	= 0,

		//.max_speed_hz = 3000000,
		.max_speed_hz = 6 * 1000 * 1000,

		.controller_data = &t3900_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(TDMB_T3900_IRQ),
	},
#endif /* CONFIG_LGE_BROADCAST_TDMB */
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

#if defined(CONFIG_LGE_SPI_SLAVE)
    {
        .modalias = "ifxn721",
        .bus_num         = 4,
        .chip_select = 0,
        .max_speed_hz    = 12000000,        //24000000Mhz
        .controller_data = &ifxn721_mcspi_config,
        .irq             = OMAP_GPIO_IRQ(119),
    },
#endif /* CONFIG_LGE_SPI_SLAVE */
};

int __init u2_spi_init(void)
{
	int ret = 0;

	//ret = lge_set_spi_board(spi_bd_info, ARRAY_SIZE(spi_bd_info));
	ret = spi_register_board_info(spi_bd_info, ARRAY_SIZE(spi_bd_info));
	if (ret < 0)
		return ret;

	return lge_set_cp_init(ifx_n721_dev_init);
};

lge_machine_initcall(u2_spi_init);
