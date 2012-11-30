/*
 * Cosmopolitan GPS GPIO Control driver
 *
 * linux/arch/arm/mach-omap2/cosmo-gps.c
 *
 * Copyright (C) 2010 LGE, Inc.
 * Author: Miok Park <miok.park@lge.com>
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
 * along with this program; if not see <http://www.gnu.org/license/>
 */

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio.h>

#include "mux.h"


#define GPIO_GPS_PWR_ON 	0
#define GPIO_GPS_RESET_N	1

#define GPIO_GPS_LNA_SD 	140

static ssize_t gps_gpio_reset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	int value;
	printk(KERN_INFO "gps_gpio_reset_show\n");

	value = gpio_get_value(GPIO_GPS_RESET_N);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	printk(KERN_INFO "gps_gpio_reset_store\n");

	sscanf(buf, "%d", &value);

	gpio_direction_output(GPIO_GPS_RESET_N, value);
	gpio_set_value(GPIO_GPS_RESET_N, value);

	return size;
}

static ssize_t gps_gpio_poweron_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value;
	printk(KERN_INFO "gps_gpio_poweron_show\n");

	value = gpio_get_value(GPIO_GPS_PWR_ON);

	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_gpio_poweron_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	printk(KERN_INFO "gps_gpio_poweron_store\n");

	sscanf(buf, "%d", &value);

	gpio_direction_output(GPIO_GPS_PWR_ON, value);
	gpio_set_value(GPIO_GPS_PWR_ON, value);

	gpio_direction_output(GPIO_GPS_LNA_SD, value);
	gpio_set_value(GPIO_GPS_LNA_SD, value);

	return size;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, gps_gpio_reset_show, gps_gpio_reset_store);
static DEVICE_ATTR(poweron, S_IRUGO | S_IWUSR, gps_gpio_poweron_show, gps_gpio_poweron_store);


static int gps_gpio_probe(struct platform_device *pdev)
{
	int retval = 0;

	printk(KERN_INFO "gps_gpio_probe\n");

	omap_mux_init_gpio(GPIO_GPS_PWR_ON,  OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_GPS_RESET_N, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_GPS_LNA_SD, OMAP_PIN_OUTPUT);
	retval = gpio_request(GPIO_GPS_PWR_ON,  "GPS power on GPIO");
	if (retval)
	{
		printk(KERN_ERR "gps_gpio_probe: GPIO %d is already used!\n", GPIO_GPS_PWR_ON);
		return retval;
	}
	
	retval = gpio_request(GPIO_GPS_RESET_N, "GPS reset GPIO");
	if (retval)
	{
		printk(KERN_ERR "gps_gpio_probe: GPIO %d is already used!\n", GPIO_GPS_RESET_N);
		return retval;
	}

		retval = gpio_request(GPIO_GPS_LNA_SD, "GPS extend LNA GPIO");
		if (retval)
		{
			printk(KERN_ERR "gps_gpio_probe: GPIO %d is already used!\n", GPIO_GPS_LNA_SD);
			return retval;
		}

	retval = device_create_file(&pdev->dev, &dev_attr_reset);
	if (retval)
		goto error;
	
	retval = device_create_file(&pdev->dev, &dev_attr_poweron);
	if (retval)
		goto error;

	return 0;

error:
	printk(KERN_ERR "gps_gpio_probe: Cannot create file desc.!\n");
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	
	return retval;
}



static int gps_gpio_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "gps_gpio_remove\n");
	device_remove_file(&pdev->dev, &dev_attr_reset);
	device_remove_file(&pdev->dev, &dev_attr_poweron);
	return 0;
}


// platform_driver
static struct platform_driver gps_gpio_driver = {
	.probe	= gps_gpio_probe,
	.remove	= gps_gpio_remove,
	.driver	= {
		.name   = "gps_gpio",
		.owner  = THIS_MODULE
	},
};


static int __devinit gps_gpio_init(void)
{
	printk(KERN_INFO "gps_gpio_init\n");
	return platform_driver_register(&gps_gpio_driver);
}

static void __exit gps_gpio_exit(void)
{
	printk(KERN_INFO "gps_gpio_exit\n");
	platform_driver_unregister(&gps_gpio_driver);
}

module_init(gps_gpio_init);
module_exit(gps_gpio_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("GPS GPIO Controller");
MODULE_LICENSE("GPL v2");

