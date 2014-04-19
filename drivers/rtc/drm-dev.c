/* drivers/rtc/drm-dev.c
 *
 * Copyright (C) 2010-2011 LG Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

DECLARE_WAIT_QUEUE_HEAD(drm_wait_queue);
unsigned long drm_diff_time = 0;
int drm_sign = 0;

DEFINE_SPINLOCK(drm_lock);

static long drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned long diff_time = 0;
	int sign = 0;

	switch (cmd) {
	default:
		wait_event_interruptible(drm_wait_queue, drm_sign != 0);

		if (drm_sign != 0) {
			spin_lock(&drm_lock);
			diff_time = drm_diff_time;
			sign = drm_sign;
			spin_unlock(&drm_lock);

			printk
			    ("secure clock: RTC has been updated!, drm_sign: %d\n",
			     sign);
			// the changed time is less than the previous time (go to past)
			if (sign > 0)
				ret = 0;
			// otherwise
			else
				ret = 1;

			if (copy_to_user((void __user *)arg, &diff_time,
					 sizeof(diff_time))) {
				ret = -1;
			}
		} else {
			printk("secure clock: enter sleep\n");
		}

		break;
	}

	drm_sign = 0;

	printk("secure clock: end of ioctl()\n");
	return ret;
}

static int drm_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static int drm_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = drm_ioctl,
	.open = drm_open,
	.release = drm_release,
};

static struct miscdevice drm_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "secclk",
	.fops = &drm_fops,
};

static int __init drm_dev_init(void)
{
	int err;

	err = misc_register(&drm_device);
	if (err)
		return err;

	return 0;
}

static void __exit drm_dev_exit(void)
{
	misc_deregister(&drm_device);
}

module_init(drm_dev_init);
module_exit(drm_dev_exit);
