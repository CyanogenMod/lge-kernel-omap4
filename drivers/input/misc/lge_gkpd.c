/* drivers/input/misc/lge_gkpd.c
 *
 * Copyright (C) 2012 LGE Inc.
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/lge/lge_input.h>

static unsigned int test_mode = 0;
static int test_code = 0;
static int gkpd_last_index = 0;
static unsigned char gkpd_value[21];
static struct wake_lock gkpd_wake_lock;

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-28, add AT%KEYLOCK
static unsigned int is_key_lock = 0;
static struct wake_lock keylock_wake_lock;
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-28

struct gkpd_device {
	struct key_table *keys;
	int size;
};

static struct gkpd_device *gkpd_dev = NULL;

static int convert_key(int code)
{
	int i = 0;

	if (gkpd_dev == NULL) {
		pr_warning("GKPD: gkpd device is not yet initialized\n");
		return -ENODEV;
	}

	for (i = 0; i < gkpd_dev->size && code != 0xff; i++) {
		if (gkpd_dev->keys[i].in == code)
			return gkpd_dev->keys[i].out;
	}
	return code;
}

int gkpd_get_test_mode(void)
{
	return test_mode;
}
EXPORT_SYMBOL(gkpd_get_test_mode);

void gkpd_write_value(int value)
{
	int i;
	int key;

	key = convert_key(value);
	if (key < 0)
		return;

	if (gkpd_last_index == 20) {
		gkpd_value[gkpd_last_index] = key;
		for (i = 0; i < 20; i++) {
			gkpd_value[i] = gkpd_value[i+1];
		}
		gkpd_value[gkpd_last_index] = '\n';
	}
	else {
		gkpd_value[gkpd_last_index] = key;
		gkpd_value[gkpd_last_index + 1] = '\n';
		gkpd_last_index++;
	}
}
EXPORT_SYMBOL(gkpd_write_value);

void gkpd_report_key(int code, int value)
{
	if (test_mode == 1 && value == 0) {
		test_code = code;
		gkpd_write_value(test_code);
	}
}
EXPORT_SYMBOL(gkpd_report_key);

static ssize_t keypad_test_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i;
	int size = 0;

	for (i = 0; i < gkpd_last_index; i++) {
		printk("GKPD: %s: code value: %d\n", __func__, gkpd_value[i]);
		size += sprintf(buf+size, "%c", gkpd_value[i]);
	}

	gkpd_last_index = 0;
	memset(gkpd_value, 0, 21);

	return size;
}

static ssize_t keypad_test_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%d", &test_mode);

	printk("GKPD: %s: test_mode %d\n", __func__, test_mode);

	if (test_mode == 1) {
		if (wake_lock_active(&gkpd_wake_lock))
			dev_warn(dev, "already locked wakelock\n");
		else
			wake_lock(&gkpd_wake_lock);

		memset(gkpd_value, 0, sizeof(gkpd_value));
		gkpd_last_index = 0;
	}
	else if (test_mode == 0) {
		wake_unlock(&gkpd_wake_lock);
	}

	return count;
}

static DEVICE_ATTR(key_test_mode, 0660, keypad_test_mode_show, keypad_test_mode_store);


// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-28, add function & sysfs path for AT%KEYLOCK
int get_key_lock_status(void)
{
	return is_key_lock;
}
EXPORT_SYMBOL(get_key_lock_status);

static ssize_t key_lock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_key_lock);
}

static ssize_t key_lock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = sscanf(buf, "%d", &is_key_lock);

	printk("KEY_LOCK: %s: is_key_lock %d\n", __func__, is_key_lock);

	if (is_key_lock == 1) {
		if (wake_lock_active(&keylock_wake_lock))
			dev_warn(dev, "already locked wakelock\n");
		else
			wake_lock(&keylock_wake_lock);
	}
	else if (is_key_lock == 0) {
		wake_unlock(&keylock_wake_lock);
	}

	return count;
}

static DEVICE_ATTR(key_lock, 0660, key_lock_show, key_lock_store);

// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-28

static int __devinit gkpd_probe(struct platform_device *pdev)
{
	struct gkpd_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	if (!pdata || !pdata->keys || !pdata->size) {
		dev_err(&pdev->dev, "GKPD: %s: No platform data\n", __func__);
		return -EINVAL;
	}

	gkpd_dev = kzalloc(sizeof(struct gkpd_device), GFP_KERNEL);
	if (!gkpd_dev) {
		dev_err(&pdev->dev, "GKPD: %s: No memory\n", __func__);
		return -ENOMEM;
	}

	gkpd_dev->keys = pdata->keys;
	gkpd_dev->size = pdata->size;

	ret = device_create_file(&pdev->dev, &dev_attr_key_test_mode);
	if (ret < 0)
		goto err_device_create_file_gkpd;

// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-28, create sysfs path for AT%KEYLOCK
	ret = device_create_file(&pdev->dev, &dev_attr_key_lock);
	if (ret < 0)
		goto err_device_create_file_keylock;
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-28

	wake_lock_init(&gkpd_wake_lock, WAKE_LOCK_SUSPEND, "lge-gpkd");
// LGE_CHANGE_S [younglae.kim@lge.com] 2012-06-28, add wake_lock for AT%KEYLOCK
	wake_lock_init(&keylock_wake_lock, WAKE_LOCK_SUSPEND, "lge-keylock");
// LGE_CHANGE_E [younglae.kim@lge.com] 2012-06-28

	dev_info(&pdev->dev, "GKPD: gkpd probed\n");

	return 0;
err_device_create_file_keylock:
	device_remove_file(&pdev->dev, &dev_attr_key_test_mode);
err_device_create_file_gkpd:
	kfree(gkpd_dev);
	gkpd_dev = NULL;
	return ret;
}

static int __devexit gkpd_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_key_test_mode);
	wake_lock_destroy(&gkpd_wake_lock);
	kfree(gkpd_dev);
	gkpd_dev = NULL;

	return 0;
}

static struct platform_driver gkpd_driver = {
	.probe  = gkpd_probe,
	.remove = __devexit_p(gkpd_remove),
	.driver = {
		.name = "lge-gkpd",
		.owner = THIS_MODULE,
	},
};

static int __init gkpd_init(void)
{
	return platform_driver_register(&gkpd_driver);
}

static void __exit gkpd_exit(void)
{
	platform_driver_unregister(&gkpd_driver);
}

module_init(gkpd_init);
module_exit(gkpd_exit);

MODULE_AUTHOR("LGE Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lge-gkpd");
