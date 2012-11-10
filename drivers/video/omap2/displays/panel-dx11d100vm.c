/*
 * iFF HD DSI Video Mode Panel Driver
 *
 * modified from panel-taal.c
 * choongryeol.lee@lge.com 
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <video/omapdss.h>
#include <video/lge-dsi-panel.h>

#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
#include "../dss/dss.h"
#endif

/* DSI Virtual channel. Hardcoded for now. */
#define TCH 0

#define DCS_READ_NUM_ERRORS	0x05
#define DCS_READ_POWER_MODE	0x0a
#define DCS_READ_MADCTL		0x0b
#define DCS_READ_PIXEL_FORMAT	0x0c
#define DCS_RDDSDR		0x0f
#define DCS_SLEEP_IN		0x10
#define DCS_SLEEP_OUT		0x11
#define DCS_DISPLAY_OFF		0x28
#define DCS_DISPLAY_ON		0x29
#define DCS_COLUMN_ADDR		0x2a
#define DCS_PAGE_ADDR		0x2b
#define DCS_MEMORY_WRITE	0x2c
#define DCS_TEAR_OFF		0x34
#define DCS_TEAR_ON		0x35
#define DCS_MEM_ACC_CTRL	0x36
#define DCS_PIXEL_FORMAT	0x3a
#define DCS_BRIGHTNESS		0x51
#define DCS_CTRL_DISPLAY	0x53
#define DCS_WRITE_CABC		0x55
#define DCS_READ_CABC		0x56
#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc

#define LONG_CMD_MIPI  0
#define SHORT_CMD_MIPI  1
#define END_OF_COMMAND  2
#define DELAY_COMMAND	3

#define DSI_GEN_SHORTWRITE_NOPARAM 0x3
#define DSI_GEN_SHORTWRITE_1PARAM 0x13
#define DSI_GEN_SHORTWRITE_2PARAM 0x23
#define DSI_GEN_LONGWRITE 	  0x29

/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 * @regulators: array of panel regulators
 * @num_regulators: number of regulators in the array
 */
struct panel_config {
	const char *name;
	int type;

	struct omap_video_timings timings;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	struct panel_regulator *regulators;
	int num_regulators;
};

enum {
	PANEL_IFF,
};

static struct panel_config panel_configs[] = {
	{
		.name		= "dx11d100vm_panel",
		.type		= PANEL_IFF,
		.timings	= {
			.x_res		= 720,
			.y_res		= 1280,
			.vfp = 4, 
			.vsw = 1, 
			.vbp = 4, 
			.hfp = 98,
			.hsw = 4,	 
			.hbp = 188, 
		},
		.sleep		= {
			.sleep_in	= 60,
			.sleep_out	= 120,
			.hw_reset	= 5, //5ms
		},
		.reset_sequence	= {
			.high		= 5000, //5ms
			.low		= 5000, //5ms
		},
	},
};

struct dx11d100vm_data {
	struct mutex lock;

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	bool enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;
	int channel;

	struct delayed_work te_timeout_work;

	bool use_dsi_bl;

	bool cabc_broken;
	unsigned cabc_mode;

	bool intro_printed;

	struct workqueue_struct *workqueue;

	struct delayed_work esd_work;
	unsigned esd_interval;

	bool ulps_enabled;
	unsigned ulps_timeout;
	struct delayed_work ulps_work;

	struct panel_config *panel_config;
};

u8 dx11d100vm_init_lcd_command[][32] = {
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0x36, 0x00,},	/* Set Address Mode */ 
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0x3A, 0x70,},	/* Set Pixel Format */ 
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0x11, 0x00,},	/* Exit Sleep */ 
	{DELAY_COMMAND, 100,},	/* Delay 100 ms */
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB0, 0x04,},	/* MACP Off */ 
	/* Gamma Setting A */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x1D, 0xC8, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11,},
	/* Gamma Setting B */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x1D, 0xC9, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11,},
	/* Gamma Setting C */
	{LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x1D, 0xCA, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11, 0x01, 0x0C, 0x11, 0x17, 0x1F, 0x26, 0x28, 0x1E, 0x1E, 0x19, 0x15, 0x13, 0x0B, 0x11,},
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB0, 0x03,},	/* MACP On */ 
	{SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0x29, 0x00,},	/* Display On */ 
	{DELAY_COMMAND, 50,},	/* Delay 50 ms */
	{END_OF_COMMAND, },	
};

static inline struct lge_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct lge_dsi_panel_data *) dssdev->data;
}

static void dx11d100vm_esd_work(struct work_struct *work);
static void dx11d100vm_ulps_work(struct work_struct *work);

static void hw_guard_start(struct dx11d100vm_data *td, int guard_msec)
{
	td->hw_guard_wait = msecs_to_jiffies(guard_msec);
	td->hw_guard_end = jiffies + td->hw_guard_wait;
}

static void hw_guard_wait(struct dx11d100vm_data *td)
{
	unsigned long wait = td->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= td->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int dx11d100vm_dcs_read_1(struct dx11d100vm_data *td, u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(td->dssdev, td->channel, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int dx11d100vm_dcs_write_0(struct dx11d100vm_data *td, u8 dcs_cmd)
{
	return dsi_vc_dcs_write(td->dssdev, td->channel, &dcs_cmd, 1);
}

static int dx11d100vm_dcs_write_1(struct dx11d100vm_data *td, u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(td->dssdev, td->channel, buf, 2);
}

static int dx11d100vm_sleep_in(struct dx11d100vm_data *td)

{
	u8 cmd;
	int r;

	hw_guard_wait(td);

	r = dx11d100vm_dcs_write_0(td, DCS_SLEEP_IN);
	if (r)
		return r;

	hw_guard_start(td, 120);

	if (td->panel_config->sleep.sleep_in)
		msleep(td->panel_config->sleep.sleep_in);

	return 0;
}

static int dx11d100vm_sleep_out(struct dx11d100vm_data *td)
{
	int r;

	hw_guard_wait(td);

	r = dx11d100vm_dcs_write_0(td, DCS_SLEEP_OUT);
	if (r)
		return r;

	hw_guard_start(td, 120);

	if (td->panel_config->sleep.sleep_out)
		msleep(td->panel_config->sleep.sleep_out);

	return 0;
}

static void dx11d100vm_queue_esd_work(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->esd_interval > 0)
		queue_delayed_work(td->workqueue, &td->esd_work,
				msecs_to_jiffies(td->esd_interval));
}

static void dx11d100vm_cancel_esd_work(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->esd_interval > 0)
		cancel_delayed_work(&td->esd_work);
}

static void dx11d100vm_queue_ulps_work(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_timeout > 0)
		queue_delayed_work(td->workqueue, &td->ulps_work,
				msecs_to_jiffies(td->ulps_timeout));
}

static void dx11d100vm_cancel_ulps_work(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_timeout > 0)
		cancel_delayed_work(&td->ulps_work);
}

static int dx11d100vm_enter_ulps(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_enabled)
		return 0;

	dx11d100vm_cancel_ulps_work(dssdev);

	omapdss_dsi_display_disable(dssdev, false, true);

	td->ulps_enabled = true;

	return 0;
}

static int dx11d100vm_exit_ulps(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (!td->ulps_enabled)
		return 0;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err1;
	}

	omapdss_dsi_vc_enable_hs(dssdev, td->channel, true);

	enable_irq(gpio_to_irq(panel_data->ext_te_gpio));

	dx11d100vm_queue_ulps_work(dssdev);

	td->ulps_enabled = false;

	return 0;

err1:
	dx11d100vm_queue_ulps_work(dssdev);

	return r;
}

static int dx11d100vm_wake_up(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_enabled)
		return dx11d100vm_exit_ulps(dssdev);

	dx11d100vm_cancel_ulps_work(dssdev);
	dx11d100vm_queue_ulps_work(dssdev);
	return 0;
}

static int dx11d100vm_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int r=0;
	int level;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	mutex_lock(&td->lock);

	if (td->use_dsi_bl) {
		if (td->enabled) {
			dsi_bus_lock(dssdev);

			r = dx11d100vm_wake_up(dssdev);
			if (!r)
				r = dx11d100vm_dcs_write_1(td, DCS_BRIGHTNESS, level);

			dsi_bus_unlock(dssdev);
		} else {
			r = 0;
		}
	} 

	mutex_unlock(&td->lock);

	return r;
}

static int dx11d100vm_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static const struct backlight_ops dx11d100vm_bl_ops = {
	.get_brightness = dx11d100vm_bl_get_intensity,
	.update_status  = dx11d100vm_bl_update_status,
};

static void dx11d100vm_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void dx11d100vm_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->rotate == 0 || td->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static const char *cabc_modes[] = {
	"off",		/* used also always when CABC is not supported */
	"ui",
	"still-image",
	"moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	int mode;
	int len;

	mode = td->cabc_mode;

	mode_str = "unknown";
	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];
	len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int i;
	int r;

	for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
		if (sysfs_streq(cabc_modes[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(cabc_modes))
		return -EINVAL;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(dssdev);

		if (!td->cabc_broken) {
			r = dx11d100vm_wake_up(dssdev);
			if (r)
				goto err;

			r = dx11d100vm_dcs_write_1(td, DCS_WRITE_CABC, i);
			if (r)
				goto err;
		}

		dsi_bus_unlock(dssdev);
	}

	td->cabc_mode = i;

	mutex_unlock(&td->lock);

	return count;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
	return r;
}

static ssize_t show_cabc_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	int i;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static ssize_t dx11d100vm_store_esd_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);
	dx11d100vm_cancel_esd_work(dssdev);
	td->esd_interval = t;
	if (td->enabled)
		dx11d100vm_queue_esd_work(dssdev);
	mutex_unlock(&td->lock);

	return count;
}

static ssize_t dx11d100vm_show_esd_interval(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&td->lock);
	t = td->esd_interval;
	mutex_unlock(&td->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t dx11d100vm_store_ulps(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(dssdev);

		if (t)
			r = dx11d100vm_enter_ulps(dssdev);
		else
			r = dx11d100vm_wake_up(dssdev);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return count;
}

static ssize_t dx11d100vm_show_ulps(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&td->lock);
	t = td->ulps_enabled;
	mutex_unlock(&td->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t dx11d100vm_store_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);
	td->ulps_timeout = t;

	if (td->enabled) {
		/* dx11d100vm_wake_up will restart the timer */
		dsi_bus_lock(dssdev);
		r = dx11d100vm_wake_up(dssdev);
		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return count;
}

static ssize_t dx11d100vm_show_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&td->lock);
	t = td->ulps_timeout;
	mutex_unlock(&td->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
		show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
		show_cabc_available_modes, NULL);
static DEVICE_ATTR(esd_interval, S_IRUGO | S_IWUSR,
		dx11d100vm_show_esd_interval, dx11d100vm_store_esd_interval);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		dx11d100vm_show_ulps, dx11d100vm_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		dx11d100vm_show_ulps_timeout, dx11d100vm_store_ulps_timeout);

static struct attribute *dx11d100vm_attrs[] = {
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
	NULL,
};

static struct attribute_group dx11d100vm_attr_group = {
	.attrs = dx11d100vm_attrs,
};

static void dx11d100vm_hw_reset(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->reset_gpio == -1)
		return;

	gpio_request(panel_data->reset_gpio, "lcd_reset");
	gpio_direction_output(panel_data->reset_gpio, 1);

	gpio_set_value(panel_data->reset_gpio, 1);
	if (td->panel_config->reset_sequence.high)
		udelay(td->panel_config->reset_sequence.high);
	/* reset the panel */
	gpio_set_value(panel_data->reset_gpio, 0);
	/* assert reset */
	if (td->panel_config->reset_sequence.low)
		udelay(td->panel_config->reset_sequence.low);
	gpio_set_value(panel_data->reset_gpio, 1);
	/* wait after releasing reset */
	if (td->panel_config->sleep.hw_reset)
		msleep(td->panel_config->sleep.hw_reset);
}

static int dx11d100vm_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct dx11d100vm_data *td;
	struct backlight_device *bldev;
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;
	int r, i;

	dev_dbg(&dssdev->dev, "probe\n");
	
	if (!panel_data || !panel_data->name) {
		r = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		r = -EINVAL;
		goto err;
	}
	
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF ;
	dssdev->panel.timings = panel_config->timings;
	dssdev->ctrl.pixel_size = 24;

	/* Since some android application use physical dimension, that information should be set here */
	dssdev->panel.width_in_um = 56000; /* physical dimension in um */
	dssdev->panel.height_in_um = 99000; /* physical dimension in um */ 

	td = kzalloc(sizeof(*td), GFP_KERNEL);
	if (!td) {
		r = -ENOMEM;
		goto err;
	}
	td->dssdev = dssdev;
	td->panel_config = panel_config;
	td->esd_interval = panel_data->esd_interval;
	td->ulps_enabled = false;
	td->ulps_timeout = panel_data->ulps_timeout;

	mutex_init(&td->lock);

	atomic_set(&td->do_update, 0);

	td->workqueue = create_singlethread_workqueue("dx11d100vm_panel_esd");
	if (td->workqueue == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&td->esd_work, dx11d100vm_esd_work);
	INIT_DELAYED_WORK(&td->ulps_work, dx11d100vm_ulps_work);

	dev_set_drvdata(&dssdev->dev, td);

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	memset(&props, 0, sizeof(struct backlight_properties));

	/* dx11d100vm dose not use dsi blacklight control */
	td->use_dsi_bl = false; 

	if (td->use_dsi_bl)
		props.max_brightness = 255;
	else
		props.max_brightness = 127;

	props.type = BACKLIGHT_RAW;
	bldev = backlight_device_register(dev_name(&dssdev->dev), &dssdev->dev,
					dssdev, &dx11d100vm_bl_ops, &props);
	if (IS_ERR(bldev)) {
		r = PTR_ERR(bldev);
		goto err_bl;
	}

	td->bldev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	if (td->use_dsi_bl)
		bldev->props.brightness = 255;
	else
		bldev->props.brightness = 127;

	dx11d100vm_bl_update_status(bldev);

	r = omap_dsi_request_vc(dssdev, &td->channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel\n");
		goto err_req_vc;
	}

	r = omap_dsi_set_vc_id(dssdev, td->channel, TCH);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID\n");
		goto err_vc_id;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &dx11d100vm_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err_vc_id;
	}
	
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	dss_runtime_get ();
#endif

	return 0;

err_vc_id:
	omap_dsi_release_vc(dssdev, td->channel);
err_req_vc:
	if (panel_data->use_ext_te)
		free_irq(gpio_to_irq(panel_data->ext_te_gpio), dssdev);
err_bl:
	destroy_workqueue(td->workqueue);
err_wq:
err:
	return r;
}

static void __exit dx11d100vm_remove(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &dx11d100vm_attr_group);
	omap_dsi_release_vc(dssdev, td->channel);

	bldev = td->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	dx11d100vm_bl_update_status(bldev);
	backlight_device_unregister(bldev);

	dx11d100vm_cancel_ulps_work(dssdev);
	dx11d100vm_cancel_esd_work(dssdev);
	destroy_workqueue(td->workqueue);

	/* reset, to be sure that the panel is in a valid state */
	dx11d100vm_hw_reset(dssdev);

	kfree(td);
}

static int dx11d100vm_power_on(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int i, r;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			return r;
	}
	
	if(!dssdev->skip_init)
	{
		dx11d100vm_hw_reset(dssdev);

		omapdss_dsi_vc_enable_hs(dssdev, td->channel, false);

		mdelay(10);	

		for (i = 0; dx11d100vm_init_lcd_command[i][0] != END_OF_COMMAND; i++) {
			if(dx11d100vm_init_lcd_command[i][0] != DELAY_COMMAND)
			{
				r= dsi_vc_dcs_write(dssdev, td->channel, &dx11d100vm_init_lcd_command[i][3], dx11d100vm_init_lcd_command[i][2]);
				if(r)
					goto err;
			}
			else
				mdelay(dx11d100vm_init_lcd_command[i][1]);
		}

		omapdss_dsi_vc_enable_hs(dssdev, td->channel, true);

		/* LGE_SJIT 2012-03-06 [choongryeol.lee@lge.com]
		 * For ignoring "DISPC_IRQ_SYNC_LOST_DIGIT" that could be happened
		 * when lcd is resumed, we set the "first_vsync" value as false for HDMI channel
		 */
		omap_dispc_set_first_vsync(OMAP_DSS_CHANNEL_DIGIT, false);

		dsi_video_mode_enable(dssdev, 0x3e);
		mdelay(30);
	}
	else
	{
		omapdss_dsi_update_vc_mode(dssdev, 0, 0); //DSI_VC_MODE_VP set
		dssdev->skip_init = false;
	#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
		dss_runtime_put();
	#endif
	}

	td->enabled = 1;

	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	dx11d100vm_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev, true, false);
err0:
	return r;
}

static void dx11d100vm_power_off(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	dsi_video_mode_disable(dssdev);

	r = dx11d100vm_dcs_write_0(td, DCS_DISPLAY_OFF);
	if (!r) 
		r = dx11d100vm_sleep_in(td);
	
	if (r) {
		dev_err(&dssdev->dev,
				"error disabling panel, issuing HW reset\n");
	}

	/* reset  the panel */
	if (panel_data->reset_gpio)
		gpio_set_value(panel_data->reset_gpio, 0);

	/* disable lcd ldo */
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* if we try to turn off dsi regulator (VCXIO), system will be halt  */
	/* So below funtion's sencod args should set as false  */
	omapdss_dsi_display_disable(dssdev, false, false);

	td->enabled = 0;
}

static int dx11d100vm_panel_reset(struct omap_dss_device *dssdev)
{
	dev_err(&dssdev->dev, "performing LCD reset\n");

	dx11d100vm_power_off(dssdev);
	return dx11d100vm_power_on(dssdev);
}

static int dx11d100vm_enable(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	dsi_bus_lock(dssdev);

	r = dx11d100vm_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		goto err;

	dx11d100vm_queue_esd_work(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&td->lock);

	return 0;
err:
	dev_dbg(&dssdev->dev, "enable failed\n");
	mutex_unlock(&td->lock);
	return r;
}

static void dx11d100vm_disable(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&td->lock);

	dx11d100vm_cancel_ulps_work(dssdev);
	dx11d100vm_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		int r;

		r = dx11d100vm_wake_up(dssdev);
		if (!r)
			dx11d100vm_power_off(dssdev);
	}

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&td->lock);
}

static int dx11d100vm_suspend(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "suspend\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dx11d100vm_cancel_ulps_work(dssdev);
	dx11d100vm_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	r = dx11d100vm_wake_up(dssdev);
	if (!r)
		dx11d100vm_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	mutex_unlock(&td->lock);

	return 0;
err:
	mutex_unlock(&td->lock);
	return r;
}

static int dx11d100vm_resume(struct omap_dss_device *dssdev)
{
	struct dx11d100vm_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "resume\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}
	
	dsi_bus_lock(dssdev);

	r = dx11d100vm_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dx11d100vm_queue_esd_work(dssdev);
	}

	mutex_unlock(&td->lock);

	return r;
err:
	mutex_unlock(&td->lock);
	return r;
}

static void dx11d100vm_ulps_work(struct work_struct *work)
{
	struct dx11d100vm_data *td = container_of(work, struct dx11d100vm_data,
			ulps_work.work);
	struct omap_dss_device *dssdev = td->dssdev;

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE || !td->enabled) {
		mutex_unlock(&td->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	dx11d100vm_enter_ulps(dssdev);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
}

static void dx11d100vm_esd_work(struct work_struct *work)
{
	struct dx11d100vm_data *td = container_of(work, struct dx11d100vm_data,
			esd_work.work);
	struct omap_dss_device *dssdev = td->dssdev;
	u8 state1, state2;
	int r;

	mutex_lock(&td->lock);

	if (!td->enabled) {
		mutex_unlock(&td->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	r = dx11d100vm_wake_up(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to exit ULPS\n");
		goto err;
	}

	r = dx11d100vm_dcs_read_1(td, DCS_RDDSDR, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read dx11d100vm status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = dx11d100vm_sleep_out(td);
	if (r) {
		dev_err(&dssdev->dev, "failed to run dx11d100vm self-diagnostics\n");
		goto err;
	}

	r = dx11d100vm_dcs_read_1(td, DCS_RDDSDR, &state2);
	if (r) {
		dev_err(&dssdev->dev, "failed to read dx11d100vm status\n");
		goto err;
	}

	/* Each sleep out command will trigger a self diagnostic and flip
	 * Bit6 if the test passes.
	 */
	if (!((state1 ^ state2) & (1 << 6))) {
		dev_err(&dssdev->dev, "LCD self diagnostics failed\n");
		goto err;
	}

	dsi_bus_unlock(dssdev);

	dx11d100vm_queue_esd_work(dssdev);

	mutex_unlock(&td->lock);
	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	dx11d100vm_panel_reset(dssdev);

	dsi_bus_unlock(dssdev);

	dx11d100vm_queue_esd_work(dssdev);

	mutex_unlock(&td->lock);
}

static int dx11d100vm_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	if (mode != OMAP_DSS_UPDATE_AUTO)
		return -EINVAL;
	return 0;
}

static enum omap_dss_update_mode dx11d100vm_get_update_mode(
		struct omap_dss_device *dssdev)
{
	return OMAP_DSS_UPDATE_AUTO;
}

static struct omap_dss_driver dx11d100vm_driver = {
	.probe		= dx11d100vm_probe,
	.remove		= __exit_p(dx11d100vm_remove),

	.enable		= dx11d100vm_enable,
	.disable	= dx11d100vm_disable,
	.suspend	= dx11d100vm_suspend,
	.resume		= dx11d100vm_resume,

	.set_update_mode = dx11d100vm_set_update_mode,
	.get_update_mode = dx11d100vm_get_update_mode,

	.get_resolution	= dx11d100vm_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= dx11d100vm_get_timings,

	.driver         = {
		.name   = "dx11d100vm_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init dx11d100vm_init(void)
{
	omap_dss_register_driver(&dx11d100vm_driver);

	return 0;
}

static void __exit dx11d100vm_exit(void)
{
	omap_dss_unregister_driver(&dx11d100vm_driver);
}

module_init(dx11d100vm_init);
module_exit(dx11d100vm_exit);

MODULE_AUTHOR("choongryeol.lee  <choongryeol.lee@lge.com>");
MODULE_DESCRIPTION("dx11d100vm Driver");
MODULE_LICENSE("GPL");
