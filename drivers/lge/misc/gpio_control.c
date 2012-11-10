/* drivers/lge/misc/gpio_contro.c
 *
 * Copyright(C) 2011, 2012, LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

static int num_gpio = -1;

static ssize_t gpio_read_show(struct device *dev, struct device_attribute
		*attr, char *buf)
{
	int val;

	if (!gpio_is_valid(num_gpio)) {
		dev_warn(dev, "wrong value: gpio %d\n", num_gpio);
		return 0;
	}

	val = gpio_get_value(num_gpio);

	dev_info(dev, "gpio %d, value %d\n", num_gpio, val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t gpio_read_store(struct device *dev, struct device_attribute
		 *attr, const char *buf, size_t count)
{
	if (!count)
		return -EINVAL;

	num_gpio = simple_strtoul(buf, NULL, 10);

	if (!gpio_is_valid(num_gpio)) {
		dev_warn(dev, "wrong value: gpio %d\n", num_gpio);
	}

	return count;
}

static DEVICE_ATTR(gpio_read, 0660, gpio_read_show, gpio_read_store);

static ssize_t gpio_control_store(struct device *dev, struct device_attribute
		 *attr, const char *buf, size_t count)
{
	int dir_out, num_value;
	char *last;
	if (!count)
		return -EINVAL;

	num_gpio = simple_strtoul(buf, &last, 10);
	++last;
	dir_out = simple_strtoul(last, &last, 10);
	++last;
	num_value = simple_strtoul(last, &last, 10);

	dev_info(dev, "gpio %d,  direction(0:in, 1:out) %d, value %d\n" 
			,num_gpio, dir_out, num_value);

	if (!gpio_is_valid(num_gpio) || (dir_out & ~1)) {
		dev_warn(dev, "wrong value\n");
		return count;
	}

	if (dir_out)
		gpio_direction_output(num_gpio, num_value);
	else
		gpio_direction_input(num_gpio);

	return count;
}

static DEVICE_ATTR(gpio_control, 0220, NULL, gpio_control_store);

static struct attribute *gpio_control_attributes[] = {
	&dev_attr_gpio_read.attr,
	&dev_attr_gpio_control.attr,
	NULL
};

static const struct attribute_group gpio_control_attr_group = {
	.attrs = gpio_control_attributes,
};

static int __devinit gpio_control_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = sysfs_create_group(&pdev->dev.kobj, &gpio_control_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to create sysfs\n",
				__func__);
	}

	dev_info(&pdev->dev, "gpio control probed\n");

	return 0;
}


static int __devexit gpio_control_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &gpio_control_attr_group);
	return 0;
}

static struct platform_driver gpio_control_driver = {
	.probe	= gpio_control_probe,
	.remove	= __devexit_p(gpio_control_remove),
	.driver	= {
		.name   = "gpio_control",
		.owner  = THIS_MODULE
	},
};

static int __init gpio_control_init(void)
{
	return platform_driver_register(&gpio_control_driver);
}

static void __exit gpio_control_exit(void)
{
	platform_driver_unregister(&gpio_control_driver);
}

module_init(gpio_control_init);
module_exit(gpio_control_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("GPIO Controller");
MODULE_LICENSE("GPL v2");
