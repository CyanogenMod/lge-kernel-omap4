/*
 * P940/IFF/HDK GPS GPIO Control driver
 *
 * Copyright (C) 2011 LGE, Inc.
 * Author: Zabi <mohamed.khadri@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * android\kernel\drivers\lge\gps\gps_gpio.c
 * < GPS on Android >
 * GPS Android SW Architecture.pdf
 * Broadcom GPS driver is a daemon process
 * So, Custom GPIO kernel driver to toggle reset and stndby pins is needed
 * The GPIO toggling can be exposed as a sysfs path.
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio.h>

#include <linux/lge/gps_gpio.h>
/*
	GPS_PWR_ON  :	GPIO_0   
	GPS_RESET_N :	GPIO_1  
	GPS_LNA_SD  :	GPIO_140

	GPS_UART_TXD :	UART3/GPIO_17
	GPS_UART_RXD :	UART3/GPIO_18
	GPS_UART_RTS_N :	UART3/GPIO_19
	GPS_UART_CTS_N :	UART3/GPIO_20
	ttyO2

*/

static ssize_t gps_gpio_reset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;
	
	pr_info("%s\n", __func__);

	value = gpio_get_value(pdata->reset_n);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	sscanf(buf, "%d", &value);

	gpio_set_value(pdata->reset_n, value);

	return size;
}

static ssize_t gps_gpio_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	value = gpio_get_value(pdata->pwron);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_poweron_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	struct gps_gpio_platform_data *pdata = dev->platform_data;

	pr_info("%s\n", __func__);

	sscanf(buf, "%d", &value);

	gpio_set_value(pdata->pwron, value);

#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	gpio_set_value(pdata->lna_sd, value);
#endif
// LGE_SJIT_S 12/21/2011 [mohamed.khadri@lge.com] GPS UART Enable/Disable
	if(value)
		pdata->uart_enable();
	else
		pdata->uart_disable();
// LGE_SJIT_E 12/21/2011 [mohamed.khadri@lge.com] GPS UART Enable/Disable
	return size;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, gps_gpio_reset_show, gps_gpio_reset_store);
static DEVICE_ATTR(poweron, S_IRUGO | S_IWUSR, gps_gpio_poweron_show, gps_gpio_poweron_store);


static int __devinit gps_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct gps_gpio_platform_data *pdata = pdev->dev.platform_data;

	pr_info("%s\n", __func__);

	if (!pdata)
		return -EINVAL;

	ret = gpio_request(pdata->pwron, "GPS power on GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->pwron);
		goto err_gpio_pwron_req;
	}
	gpio_direction_output(pdata->pwron, 0);

	ret = gpio_request(pdata->reset_n, "GPS reset GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->reset_n);
		goto err_gpio_reset_req;
	}
	// LGE_SJIT 01/26/2012 [mohamed.khadri@lge.com] Configure for initial out high
	gpio_direction_output(pdata->reset_n, 1);

#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	ret = gpio_request(pdata->lna_sd, "GPS extend LNA GPIO");
	if (ret) {
		pr_err("%s: failed to request GPIO_%d\n", __func__, pdata->lna_sd);
		goto err_gpio_lns_sd_req;
	}
	gpio_direction_output(pdata->lna_sd, 0);
#endif

	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	if (ret) {
		pr_err("%s: failed to create \"dev_attr_reset\" attribute!\n",
				__func__);
		goto err_reset_attr_create;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_poweron);
	if (ret) {
		pr_err("%s: failed to create \"dev_attr_poweron\" attribute!\n",
				__func__);
		goto err_pwron_attr_create;
	}

	return 0;

err_pwron_attr_create:
	device_remove_file(&pdev->dev, &dev_attr_reset);
err_reset_attr_create:
#if defined(CONFIG_P940_GPS_LNA_SD_USE)
	gpio_free(pdata->lna_sd);
err_gpio_lns_sd_req:
#endif
	gpio_free(pdata->reset_n);
err_gpio_reset_req:
	gpio_free(pdata->pwron);
err_gpio_pwron_req:
	return ret;
}


static int __devexit gps_gpio_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	return 0;
}


static struct platform_driver gps_gpio_driver = {
	.probe	= gps_gpio_probe,
	.remove	= __devexit_p(gps_gpio_remove),
	.driver	= {
		.name   = "gps_gpio",
		.owner  = THIS_MODULE
	},
};


static int __devinit gps_gpio_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&gps_gpio_driver);
}

static void __exit gps_gpio_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&gps_gpio_driver);
}

module_init(gps_gpio_init);
module_exit(gps_gpio_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("GPS GPIO Controller");
MODULE_LICENSE("GPL v2");
