/*
 * lh430wv5_panel  DSI Video/Command Mode Panel Driver
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

//#define DEBUG

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

//CHANGE_S mo2mk.kim@lge.com 2012-07-26 apply kcal code
#if defined(CONFIG_CX2_GAMMA)
#include <plat/lge_nvdata_handler.h>
#endif
//CHANGE_E mo2mk.kim@lge.com 2012-07-26 apply kcal code

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
#define DCS_DEEP_STANDBY_IN		0xC1
#define DCS_GET_ID			0xf8

#define DSI_DT_DCS_SHORT_WRITE_0	0x05
#define DSI_DT_DCS_SHORT_WRITE_1	0x15
#define DSI_DT_DCS_READ			0x06
#define DSI_DT_SET_MAX_RET_PKG_SIZE	0x37
#define DSI_DT_NULL_PACKET		0x09
#define DSI_DT_DCS_LONG_WRITE		0x39

static irqreturn_t lh430wv5_panel_te_isr(int irq, void *data);
static void lh430wv5_panel_te_timeout_work_callback(struct work_struct *work);
static int _lh430wv5_panel_enable_te(struct omap_dss_device *dssdev, bool enable);
static int lh430wv5_panel_enable(struct omap_dss_device *dssdev);

#define DSI_GEN_SHORTWRITE_NOPARAM 0x3
#define DSI_GEN_SHORTWRITE_1PARAM 0x13
#define DSI_GEN_SHORTWRITE_2PARAM 0x23
#define DSI_GEN_LONGWRITE 	  0x29

#define LONG_CMD_MIPI	0
#define SHORT_CMD_MIPI	1
#define END_OF_COMMAND	2


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
	PANEL_LH430WV4,
};

static struct panel_config panel_configs[] = {
	{
		.name		= "lh430wv5_panel",
		.type		= PANEL_LH430WV4,
		.timings	= {
#ifdef CONFIG_HRZ_II
			.x_res		= 960, //gb 960
			.y_res		= 1600, //gb 1600
#else
			.x_res		= 480, //gb 480
			.y_res		= 800, //gb 800
#endif

			.vfp = 10, 
			.vsw =2, 
			.vbp =16, 
			.hfp = 11,
			.hsw =10,	 
			.hbp =67, 
		},
		.sleep		= {
			.sleep_in	= 20,
			.sleep_out	= 5,
			.hw_reset	= 10,
			.enable_te	= 5,
		},
		.reset_sequence	= {
			.high		= 10000,
			.low		= 10000,
		},
	},
};

struct lh430wv5_panel_data {
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
	unsigned int cabc_mode;

	bool intro_printed;

	struct workqueue_struct *workqueue;

	struct delayed_work esd_work;
	unsigned int esd_interval;

	bool ulps_enabled;
	unsigned int ulps_timeout;
	struct delayed_work ulps_work;

	struct panel_config *panel_config;

	bool display_on;
	struct work_struct display_on_work;

    u8 powersave_enable;
    bool barrier_enabled;

//CHANGE_S mo2mk.kim@lge.com 2012-07-26 apply kcal code
#if defined(CONFIG_CX2_GAMMA)	
	struct delayed_work gamma_delayed_work;
#endif
//CHANGE_E mo2mk.kim@lge.com 2012-07-26 apply kcal code
};

u8 lgd_3Dlcd_command_for_mipi[][30] = 
{
    // Reference :: LH430WV5-SD01(CX2)_Flow Chart_V1.0_111004.pdf
    
    // Display Mode setting
    { SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_1PARAM, 0x01, 0x20,},  
    { SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_1PARAM, 0x01, 0x35,},  
    { SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0x3A, 0x77,}, 
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x03, 0xB2, 0x00, 0xC7,}, 
    // Lcd 2-dot inversion(0x02) -> Column inversion(0x00)
    //{ SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB3, 0x02,},
    { SHORT_CMD_MIPI, DSI_GEN_SHORTWRITE_2PARAM, 0x02, 0xB3, 0x00,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x08, 0xB5, 0x20, 0x10, 0x10, 0x00, 0x20, 0x00, 0x00,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x09, 0xB6, 0x00, 0x22, 0x02, 0x40, 0x20, 0x20, 0x05, 0x30,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x08, 0xB7, 0x52, 0x00, 0x10, 0x00, 0x0C, 0x00, 0x00},
    
    // Power Setting
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x04, 0xC0, 0x00, 0x0B, 0x0F,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x03, 0xC2, 0x07, 0x00,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x06, 0xC3, 0x20, 0x67, 0x00, 0x08, 0x08,},
    // LGD recommand Power control(VGH, VGL) 0x10 -> 0x01 :: 20111212
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x07, 0xC4, 0x01, 0x17, 0x00, 0x00, 0x33, 0x0A,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x06, 0xC5, 0x12, 0x32, 0x14, 0x14, 0x12,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x03, 0xC6, 0x22, 0x00,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x06, 0xC7, 0x04, 0x1A, 0x1A, 0x6D, 0x00,},
    { LONG_CMD_MIPI,  DSI_GEN_LONGWRITE,         0x03, 0xC8, 0x44, 0x63,},
    
    // MIPI Video Mode Setting
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x06, 0xE0, 0x43, 0x40, 0x80, 0x00, 0x00,},
    
    // Gamma Setting
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD0, 0x00, 0x01, 0x77, 0x06, 0x01, 0x12, 0x50, 0x11, 0x03,},
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD1, 0x00, 0x01, 0x77, 0x06, 0x01, 0x00, 0x50, 0x11, 0x03,},
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD2, 0x00, 0x01, 0x77, 0x06, 0x01, 0x12, 0x50, 0x11, 0x03,},
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD3, 0x00, 0x01, 0x77, 0x06, 0x01, 0x00, 0x50, 0x11, 0x03,},
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD4, 0x00, 0x01, 0x77, 0x06, 0x01, 0x12, 0x50, 0x11, 0x03,},
    { LONG_CMD_MIPI, DSI_GEN_LONGWRITE,          0x0A, 0xD5, 0x00, 0x01, 0x77, 0x06, 0x01, 0x00, 0x50, 0x11, 0x03,},
    
    { END_OF_COMMAND, },
};

u8 lgd_3d_lcd_on_command_for_mipi[][30] = 
{
    {LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x06, 0xE5, 0x97, 0x19, 0x19, 0x02, 0x01,},
    {LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x04, 0xE6, 0xFF, 0xFF, 0x0D,},
    {END_OF_COMMAND,},
};

u8 lgd_3d_lcd_off_command_for_mipi[][30] = 
{
    {LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x06, 0xE5, 0x87, 0x19, 0x00, 0x00, 0x01,},
    {LONG_CMD_MIPI, DSI_GEN_LONGWRITE, 0x04, 0xE6, 0x00, 0x00, 0x00,},
    {END_OF_COMMAND,},
};

static inline struct lge_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct lge_dsi_panel_data *) dssdev->data;
}

static void lh430wv5_panel_esd_work(struct work_struct *work);
static void lh430wv5_panel_ulps_work(struct work_struct *work);
static void lh430wv5_panel_display_on_work(struct work_struct *work);

#if 1 //defined(3D_LCD_FUNCTION)  //##hwcho_20120427
static int display_setting = 0;

int lm3530_brightness_3D_LCD_Enable = 0;
#if 0 //##
void dsi_enable_video_mode(struct omap_dss_device *dssdev);
#endif //##

//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET	
// Hidden reset skip code
extern int lge_hidden_reset_mode;
static int LcdHiddenBootingCount;
//mo2haewoon.you@lge.com <= [END]

extern void lm3530_PanelUsed_Brightness( int brightness, int is3DEnable );


static void lh430wv5_panel_barrier_3D_disable( struct omap_dss_device *dssdev )
{
    int i = 0;
    struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
    enum omap_dsi_index channel_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) ? DSI1 : DSI2;

    //DBG();
	printk(KERN_ERR "%s, channel=%d", __func__,dssdev->channel);
    if( td->barrier_enabled == true )
    {
        for( i = 0; lgd_3d_lcd_off_command_for_mipi[i][0] != END_OF_COMMAND; i++ ) 
        {
            dsi_vc_dcs_write_nosync(dssdev, channel_ix, &lgd_3d_lcd_off_command_for_mipi[i][3], lgd_3d_lcd_off_command_for_mipi[i][2]);           
            mdelay(2);
        }       

        td->barrier_enabled = false;

        lm3530_brightness_3D_LCD_Enable = 0;
        lm3530_PanelUsed_Brightness( 0, lm3530_brightness_3D_LCD_Enable);
    }
}

static int lh430wv5_panel_barrier_init(struct omap_dss_device *dssdev, struct lh430wv5_panel_data *panel_data, bool enable)
{

    struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
    enum omap_dsi_index channel_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) ? DSI1 : DSI2;    
    int i;

	//printk("[HYUN] %s, channel = %d, barrier = %d\n", __func__, td->channel, panel_data->barrier_enabled);
	
    if ( panel_data->barrier_enabled == enable )
    {
        return 0;
    }

    if(enable)
    {
        if( panel_data->barrier_enabled )    //alredy turn on
        {
            return 0;
        }

		printk("[HYUN] barrier_init Enable\n");
        mutex_lock(&td->lock);
        dsi_bus_lock(dssdev);

		printk("[HYUN] lgd_3d_lcd_on_command_for_mipi\n");
        for( i = 0; lgd_3d_lcd_on_command_for_mipi[i][0] != END_OF_COMMAND; i++ ) 
        {
			dsi_vc_dcs_write_nosync(dssdev,channel_ix, &lgd_3d_lcd_on_command_for_mipi[i][3], lgd_3d_lcd_on_command_for_mipi[i][2]);
            mdelay(2);
        }

        //dsi_enable_video_mode(dssdev); 
        dsi_video_mode_enable(dssdev, 0x3e);

		lm3530_brightness_3D_LCD_Enable = 1;
        panel_data->barrier_enabled = true;
        
        // 3D always 24mA, power save 18mA :: H/W concept
        if( panel_data->powersave_enable == 1 )
        {
            // 0x79 :: 71.875%  = 18.6mA
             printk("[HYUN] 3D Power Save Yes! 18mA \n");
            lm3530_PanelUsed_Brightness( 0x79, lm3530_brightness_3D_LCD_Enable);   
        }
        else
        {
            // 0x7E :: 94.531%  = 24.57mA
            printk("[HYUN] 3D Power Save No!! 24mA \n");
            lm3530_PanelUsed_Brightness( 0x7E, lm3530_brightness_3D_LCD_Enable);
        }

        dsi_bus_unlock(dssdev);
        mutex_unlock(&td->lock);    
    }
    else
    {
        if ( !panel_data->barrier_enabled )    //alredy turn off
        {
            return 0;
        }

		printk("[HYUN] barrier_init Disable\n");

        mutex_lock(&td->lock);
        dsi_bus_lock(dssdev);

        for( i = 0; lgd_3d_lcd_off_command_for_mipi[i][0] != END_OF_COMMAND; i++ ) 
        {
            dsi_vc_dcs_write_nosync(dssdev, channel_ix, &lgd_3d_lcd_off_command_for_mipi[i][3], lgd_3d_lcd_off_command_for_mipi[i][2]);             
            mdelay(2);
        }        
        
        dsi_video_mode_enable(dssdev, 0x3e); 
        //dsi_video_mode_disable(dssdev); 
        lm3530_brightness_3D_LCD_Enable = 0;
        panel_data->barrier_enabled = false;
        lm3530_PanelUsed_Brightness( 0, lm3530_brightness_3D_LCD_Enable);

        dsi_bus_unlock(dssdev);
        mutex_unlock(&td->lock);    
    }

    return 0;
}

static int lh430wv5_panel_enable_s3d(struct omap_dss_device *dssdev, bool enable) 
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	//printk(KERN_ERR "[HYUN] %s", __func__);
	
    return 	lh430wv5_panel_barrier_init(dssdev, td, enable);
}

static bool lh430wv5_panel_get_s3d_enabled(struct omap_dss_device *dssdev) 
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	//printk(KERN_ERR "%s", __func__);
	
    return td->barrier_enabled;
}

int test_panel_enable_s3d(struct omap_dss_device *dssdev, int s)
{
	if (s == 0)
	{
		lh430wv5_panel_enable_s3d(dssdev, false); 
	}
	else
	{
		lh430wv5_panel_enable_s3d(dssdev, true); 
	}	
	
	return 0;
}

static ssize_t display_3d_setting_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len;
	
	len = snprintf(buf, PAGE_SIZE, "%s\n", display_setting? "3D" : "2D");

	return len;
}

static ssize_t display_3d_setting_store(struct device *dev,	struct device_attribute *attr,
		const char *buf, size_t size)
{
    struct omap_dss_device *dssdev = to_dss_device(dev);
    struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	int setting;
	
	sscanf(buf, "%d",&setting);
	switch(setting)
	{
		case 0:
		case 1:
			printk("display is %s\n", setting? "3D" : "2D");
			test_panel_enable_s3d(dssdev, setting);
			display_setting = setting;
			break;
			
		default:
			printk("Wrong Value\n");
			break;
	}
	return size;
}

static DEVICE_ATTR(3D_display, 0660, display_3d_setting_show, display_3d_setting_store);

#endif //##
static void hw_guard_start(struct lh430wv5_panel_data *td, int guard_msec)
{
	td->hw_guard_wait = msecs_to_jiffies(guard_msec);
	td->hw_guard_end = jiffies + td->hw_guard_wait;
}

static void hw_guard_wait(struct lh430wv5_panel_data *td)
{
	unsigned long wait = td->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= td->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int lh430wv5_panel_dcs_read_1(struct lh430wv5_panel_data *td, u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(td->dssdev, td->channel, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int lh430wv5_panel_dcs_write_0(struct lh430wv5_panel_data *td, u8 dcs_cmd)
{
	return dsi_vc_dcs_write(td->dssdev, td->channel, &dcs_cmd, 1);
}

static int lh430wv5_panel_dcs_write_1(struct lh430wv5_panel_data *td, u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(td->dssdev, td->channel, buf, 2);
}

static int lh430wv5_panel_sleep_in(struct lh430wv5_panel_data *td)

{
	int r;

	hw_guard_wait(td);

	r = lh430wv5_panel_dcs_write_0(td, DCS_SLEEP_IN);
	if (r)
		return r;

	r = lh430wv5_panel_dcs_write_1(td, DCS_DEEP_STANDBY_IN, 0x1);
	if (r)
		return r;

	hw_guard_start(td, 120);

	if (td->panel_config->sleep.sleep_in)
		msleep(td->panel_config->sleep.sleep_in);

	return 0;
}

static int lh430wv5_panel_sleep_out(struct lh430wv5_panel_data *td)
{
	int r;

	hw_guard_wait(td);

	r = lh430wv5_panel_dcs_write_0(td, DCS_SLEEP_OUT);
	if (r)
		return r;

	hw_guard_start(td, 120);

	if (td->panel_config->sleep.sleep_out)
		msleep(td->panel_config->sleep.sleep_out);

	return 0;
}

static int lh430wv5_panel_set_addr_mode(struct lh430wv5_panel_data *td, u8 rotate, bool mirror)
{
	int r;
	u8 mode=0;
	int b5, b6, b7;

	r = lh430wv5_panel_dcs_read_1(td, DCS_READ_MADCTL, &mode);
	if (r)
		return r;

	switch (rotate) {
	default:
	case 0:
		b7 = 0;
		b6 = 0;
		b5 = 0;
		break;
	case 1:
		b7 = 0;
		b6 = 1;
		b5 = 1;
		break;
	case 2:
		b7 = 1;
		b6 = 1;
		b5 = 0;
		break;
	case 3:
		b7 = 1;
		b6 = 0;
		b5 = 1;
		break;
	}

	if (mirror)
		b6 = !b6;

	mode &= ~((1<<7) | (1<<6) | (1<<5));
	mode |= (b7 << 7) | (b6 << 6) | (b5 << 5);

	return lh430wv5_panel_dcs_write_1(td, DCS_MEM_ACC_CTRL, mode);
}

//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET	
// Hidden reset skip code
int lh430wv5_panel_HiddenRestStatus( int CheckMode )
{
    int ret = 0;
    if( lge_hidden_reset_mode == 1 )
    {
         if( CheckMode == 1 )
        {
            if( LcdHiddenBootingCount > 3 )
            {
                LcdHiddenBootingCount = 4;
            }
            else
            {
                LcdHiddenBootingCount++;
                ret = 1;
            }
        }
        else if( CheckMode == 2 )
        {
            if( LcdHiddenBootingCount == 3 )
            {
                LcdHiddenBootingCount++;
                ret = 1;
            }
        }
        else if( CheckMode == 3 )
        {
            if( LcdHiddenBootingCount > 3 ) 
            {
                //None
            }
            else
            {
                ret = 1;
            }        
        }
    }

    return ret;
}
//mo2haewoon.you@lge.com => [END]

static int lh430wv5_panel_set_update_window(struct lh430wv5_panel_data *td,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	u16 x1 = x;
	u16 x2 = x + w - 1;
	u16 y1 = y;
	u16 y2 = y + h - 1;

	u8 buf[5];
	buf[0] = DCS_COLUMN_ADDR;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(td->dssdev, td->channel, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = DCS_PAGE_ADDR;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(td->dssdev, td->channel, buf, sizeof(buf));
	if (r)
		return r;

#if !defined (CONFIG_OMAP_USE_CMOS_TE_TRIGGER)
	dsi_vc_send_bta_sync(td->dssdev, td->channel);
#endif

	return r;
}

static void lh430wv5_panel_queue_esd_work(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->esd_interval > 0)
		queue_delayed_work(td->workqueue, &td->esd_work,
				msecs_to_jiffies(td->esd_interval));
}

static void lh430wv5_panel_cancel_esd_work(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->esd_interval > 0)
		cancel_delayed_work(&td->esd_work);
}

static void lh430wv5_panel_queue_ulps_work(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_timeout > 0)
		queue_delayed_work(td->workqueue, &td->ulps_work,
				msecs_to_jiffies(td->ulps_timeout));
}

static void lh430wv5_panel_cancel_ulps_work(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_timeout > 0)
		cancel_delayed_work(&td->ulps_work);
}

static int lh430wv5_panel_enter_ulps(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (td->ulps_enabled)
		return 0;

	lh430wv5_panel_cancel_ulps_work(dssdev);

	r = _lh430wv5_panel_enable_te(dssdev, false);
	if (r)
		goto err;

	disable_irq(gpio_to_irq(panel_data->ext_te_gpio));

	omapdss_dsi_display_disable(dssdev, false, true);

	td->ulps_enabled = true;

	return 0;
err:
	dev_err(&dssdev->dev, "enter ULPS failed");

	td->ulps_enabled = false;

	lh430wv5_panel_queue_ulps_work(dssdev);

	return r;
}

static int lh430wv5_panel_exit_ulps(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
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

	r = _lh430wv5_panel_enable_te(dssdev, true);
	if (r) {
		dev_err(&dssdev->dev, "failed to re-enable TE");
		goto err1;
	}

	enable_irq(gpio_to_irq(panel_data->ext_te_gpio));

	lh430wv5_panel_queue_ulps_work(dssdev);

	td->ulps_enabled = false;

	return 0;

err1:
	lh430wv5_panel_queue_ulps_work(dssdev);

	return r;
}

static int lh430wv5_panel_wake_up(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (td->ulps_enabled)
		return lh430wv5_panel_exit_ulps(dssdev);

	lh430wv5_panel_cancel_ulps_work(dssdev);
	lh430wv5_panel_queue_ulps_work(dssdev);
	return 0;
}

static int lh430wv5_panel_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
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
	//mo2haewoon.you@lge.com => [START]  HIDDEN_RESET			
            // Hidden reset skip code
            if( lh430wv5_panel_HiddenRestStatus(1) == 1 )
            {
                if( lh430wv5_panel_HiddenRestStatus(3) == 1 )
                {
		            mutex_unlock(&td->lock);
                    return;
                }
            }			
	//mo2haewoon.you@lge.com <= [END]		
			dsi_bus_lock(dssdev);

			r = lh430wv5_panel_wake_up(dssdev);
			if (!r)
				r = lh430wv5_panel_dcs_write_1(td, DCS_BRIGHTNESS, level);

			dsi_bus_unlock(dssdev);
		} else {
			r = 0;
		}
	} 

	mutex_unlock(&td->lock);

	return r;
}

static int lh430wv5_panel_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static const struct backlight_ops lh430wv5_panel_bl_ops = {
	.get_brightness = lh430wv5_panel_bl_get_intensity,
	.update_status  = lh430wv5_panel_bl_update_status,
};

static void lh430wv5_panel_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void lh430wv5_panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

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
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	char *mode_str; //const char *mode_str; //mo2seongjae.jang 20120702 WBT issue modification
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
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
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
			r = lh430wv5_panel_wake_up(dssdev);
			if (r)
				goto err;

			r = lh430wv5_panel_dcs_write_1(td, DCS_WRITE_CABC, i);
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
//LGE_CHANGE_S [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
extern int dispc_set_gamma_rgb(enum omap_channel ch, u8 gamma,int red,int green,int blue);

//CHANGE_S mo2mk.kim@lge.com 2012-07-26 apply kcal code
#if defined(CONFIG_CX2_GAMMA)	
extern u32 gamma_rgb_data_dispc_for_extern(char *str);
extern void dispc_set_gamma_table(void); 
extern int dispc_enable_gamma(enum omap_channel ch, u8 gamma);

static void lge_gamma_rgb_data_from_nv2()
{
	int gamma_temp_r=0,gamma_temp_g=0,gamma_temp_b=0;
	char cmd_line[256] = {0,};
	int nv_gamma_r,nv_gamma_g,nv_gamma_b;

	int sizeRead = 0;

	sizeRead += lge_static_nvdata_raw_read(LGE_NVDATA_STATIC_LCD_GAMMA_R_OFFSET, &gamma_temp_r, 4);
	sizeRead += lge_static_nvdata_raw_read(LGE_NVDATA_STATIC_LCD_GAMMA_G_OFFSET, &gamma_temp_g, 4);
	sizeRead += lge_static_nvdata_raw_read(LGE_NVDATA_STATIC_LCD_GAMMA_B_OFFSET, &gamma_temp_b, 4);

	if((sizeRead==4*3) && ((gamma_temp_r & 0xFF) == 0x5A && (gamma_temp_g & 0xFF) == 0x5A && (gamma_temp_b & 0xFF) == 0x5A))
	{
		nv_gamma_r = (gamma_temp_r >> 24) & 0xFF;
		nv_gamma_g = (gamma_temp_g >> 24) & 0xFF;
		nv_gamma_b = (gamma_temp_b >> 24) & 0xFF;	

		sprintf(cmd_line,"%d,%d,%d",nv_gamma_r,nv_gamma_g,nv_gamma_b );

		gamma_rgb_data_dispc_for_extern(cmd_line);
	}
}

static void display_gamma_delayed_work(struct work_struct *work)
{
	lge_gamma_rgb_data_from_nv2();
	dispc_set_gamma_table();
	dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD, 0);
	dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD2, 0);		
}
#endif
//CHANGE_E mo2mk.kim@lge.com 2012-07-26 apply kcal code

static ssize_t display_gamma_tuning_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t display_gamma_tuning_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int red,green,blue;

//CHANGE_S mo2mk.kim@lge.com 2012-07-26 apply kcal code
#if defined(CONFIG_CX2_GAMMA)	
	lge_gamma_rgb_data_from_nv2();
	dispc_set_gamma_table();
#endif	
//CHANGE_E mo2mk.kim@lge.com 2012-07-26 apply kcal code

	sscanf(buf, "%d,%d,%d",&red,&green,&blue);
	printk("SJ	:	RED	:	%d	GREEN :		%d	BLUE :		%d\n",red,green,blue);
	dispc_set_gamma_rgb(OMAP_DSS_CHANNEL_LCD, 0,red,green,blue);
	dispc_set_gamma_rgb(OMAP_DSS_CHANNEL_LCD2, 0,red,green,blue);
	return size;
}


static DEVICE_ATTR(gamma_tuning, 0660, display_gamma_tuning_show, display_gamma_tuning_store);
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
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

static ssize_t lh430wv5_panel_store_esd_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);
	lh430wv5_panel_cancel_esd_work(dssdev);
	td->esd_interval = t;
	if (td->enabled)
		lh430wv5_panel_queue_esd_work(dssdev);
	mutex_unlock(&td->lock);

	return count;
}
//LGE_CHANGE_S [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
extern int dispc_enable_gamma(enum omap_channel ch, u8 gamma);
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com

static ssize_t lh430wv5_panel_show_esd_interval(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned int t;

	mutex_lock(&td->lock);
	t = td->esd_interval;
	mutex_unlock(&td->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t lh430wv5_panel_store_ulps(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);

	if (td->enabled) {
		dsi_bus_lock(dssdev);

		if (t)
			r = lh430wv5_panel_enter_ulps(dssdev);
		else
			r = lh430wv5_panel_wake_up(dssdev);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return count;
}

static ssize_t lh430wv5_panel_show_ulps(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	bool t;

	mutex_lock(&td->lock);
	t = td->ulps_enabled;
	mutex_unlock(&td->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t lh430wv5_panel_store_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&td->lock);
	td->ulps_timeout = t;

	if (td->enabled) {
		/* lh430wv5_panel_wake_up will restart the timer */
		dsi_bus_lock(dssdev);
		r = lh430wv5_panel_wake_up(dssdev);
		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&td->lock);

	if (r)
		return r;

	return count;
}

static ssize_t lh430wv5_panel_show_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	unsigned int t;

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
		lh430wv5_panel_show_esd_interval, lh430wv5_panel_store_esd_interval);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		lh430wv5_panel_show_ulps, lh430wv5_panel_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		lh430wv5_panel_show_ulps_timeout, lh430wv5_panel_store_ulps_timeout);

static struct attribute *lh430wv5_panel_attrs[] = {
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
//LGE_CHANGE_S [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
    &dev_attr_gamma_tuning.attr,
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
	&dev_attr_3D_display.attr,
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com	
	NULL,
};

static struct attribute_group lh430wv5_panel_attr_group = {
	.attrs = lh430wv5_panel_attrs,
};

static void lh430wv5_panel_hw_reset(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
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

static int lh430wv5_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct lh430wv5_panel_data *td;
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
	
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF ; //gb OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF
	dssdev->panel.timings = panel_config->timings; 
	dssdev->ctrl.pixel_size = 24; //gb 24

	/* Since some android application use physical dimension, that information should be set here */
	dssdev->panel.width_in_um = 57000; /* physical dimension in um */
	dssdev->panel.height_in_um = 94000; /* physical dimension in um */ 

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

	td->workqueue = create_singlethread_workqueue("lh430wv5_panel_panel_esd");
	if (td->workqueue == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&td->esd_work, lh430wv5_panel_esd_work);
	INIT_DELAYED_WORK(&td->ulps_work, lh430wv5_panel_ulps_work);
	INIT_WORK(&td->display_on_work, lh430wv5_panel_display_on_work);

	dev_set_drvdata(&dssdev->dev, td);

	/* if no platform set_backlight() defined, presume DSI backlight
	 * control */
	memset(&props, 0, sizeof(struct backlight_properties));

	/* P940 dose not use dsi blacklight control */
	td->use_dsi_bl = false; 

	if (td->use_dsi_bl)
		props.max_brightness = 255;
	else
		props.max_brightness = 127;

	props.type = BACKLIGHT_RAW;
	bldev = backlight_device_register(dev_name(&dssdev->dev), &dssdev->dev,
					dssdev, &lh430wv5_panel_bl_ops, &props);
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

	lh430wv5_panel_bl_update_status(bldev);

	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;

		r = gpio_request(gpio, "lh430wv5_panel irq");
		if (r) {
			dev_err(&dssdev->dev, "GPIO request failed\n");
			goto err_gpio;
		}

		gpio_direction_input(gpio);

		r = request_threaded_irq(gpio_to_irq(gpio), NULL,lh430wv5_panel_te_isr,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"lh430wv5_panel vsync", dssdev);

		if (r) {
			dev_err(&dssdev->dev, "IRQ request failed\n");
			gpio_free(gpio);
			goto err_irq;
		}

		INIT_DELAYED_WORK_DEFERRABLE(&td->te_timeout_work,
					lh430wv5_panel_te_timeout_work_callback);

		dev_dbg(&dssdev->dev, "Using GPIO TE\n");
	}

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

	r = sysfs_create_group(&dssdev->dev.kobj, &lh430wv5_panel_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err_vc_id;
	}

	td->powersave_enable = 0;
	
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	dss_runtime_get ();
#endif

//CHANGE_S mo2mk.kim@lge.com 2012-07-26 apply kcal code
#if defined(CONFIG_CX2_GAMMA)		
    INIT_DELAYED_WORK(&td->gamma_delayed_work, display_gamma_delayed_work);
    schedule_delayed_work(&td->gamma_delayed_work, msecs_to_jiffies(12000/*35000*/));
#endif
//CHANGE_E mo2mk.kim@lge.com 2012-07-26 apply kcal code

	return 0;

err_vc_id:
	omap_dsi_release_vc(dssdev, td->channel);
err_req_vc:
	if (panel_data->use_ext_te)
		free_irq(gpio_to_irq(panel_data->ext_te_gpio), dssdev);
err_irq:
	if (panel_data->use_ext_te)
		gpio_free(panel_data->ext_te_gpio);
err_gpio:
err_bl:
	destroy_workqueue(td->workqueue);
err_wq:
err:
	kfree(td);
	return r;
}

static void __exit lh430wv5_panel_remove(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &lh430wv5_panel_attr_group);
	omap_dsi_release_vc(dssdev, td->channel);

	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	bldev = td->bldev;
	bldev->props.power = FB_BLANK_POWERDOWN;
	lh430wv5_panel_bl_update_status(bldev);
	backlight_device_unregister(bldev);

	lh430wv5_panel_cancel_ulps_work(dssdev);
	lh430wv5_panel_cancel_esd_work(dssdev);
	destroy_workqueue(td->workqueue);

	/* reset, to be sure that the panel is in a valid state */
	lh430wv5_panel_hw_reset(dssdev);

	kfree(td);
}

static int lh430wv5_panel_power_on(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int i, r;
    enum omap_dsi_index channel_ix = (dssdev->channel == OMAP_DSS_CHANNEL_LCD2) ? DSI1 : DSI2;

	printk(KERN_ERR "[HYUN]%s channel =%d\n", __func__, td->channel);

	/* At power on the first vsync has not been received yet */
        dssdev->first_vsync = false;
	
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
#if 1 //defined(3D_LCD_FUNCTION)
		if(!(td->enabled))
			lh430wv5_panel_hw_reset(dssdev);
		mdelay(5);	  
#endif //##

		printk(KERN_ERR "[HYUN] %s lgd_3Dlcd_command_for_mipi\n", __func__);

		omapdss_dsi_vc_enable_hs(dssdev, channel_ix, false);

		mdelay(5);

		for (i = 0; lgd_3Dlcd_command_for_mipi[i][0] != END_OF_COMMAND; i++) {
			dsi_vc_dcs_write_nosync(dssdev, td->channel, &lgd_3Dlcd_command_for_mipi[i][3], lgd_3Dlcd_command_for_mipi[i][2]);
		mdelay(2);
		}

#if 1 //defined(3D_LCD_FUNCTION)
		lh430wv5_panel_barrier_3D_disable(dssdev);
#endif //##

		r = lh430wv5_panel_sleep_out(td);
		if (r)
			goto err;
		
		if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE){
			r = lh430wv5_panel_set_addr_mode(td, td->rotate, td->mirror);
			if (r)
				goto err;
		}
//LGE_CHANGE_S [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
#if defined(CONFIG_CX2_GAMMA)
        dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD, 0);
        dispc_enable_gamma(OMAP_DSS_CHANNEL_LCD2, 0);
#endif
//LGE_CHANGE_E [jeonghoon.cho@lge.com] 2012-0208, P940 : Add sysfile for gamma tuning + at%kcal jeonghoon.cho@lge.com
		if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE){
			r = lh430wv5_panel_dcs_write_0(td, DCS_DISPLAY_ON);
			if (r)
				goto err;
			td->display_on = true;
		}
		else
			td->display_on = false;

		
		if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE){
			r = _lh430wv5_panel_enable_te(dssdev, false);
			if (r)
				goto err;
		}
		
		omapdss_dsi_vc_enable_hs(dssdev, td->channel, true);

		/* LGE_SJIT 2012-03-06 [choongryeol.lee@lge.com]
		 * For ignoring "DISPC_IRQ_SYNC_LOST_DIGIT" that could be happened
		 * when lcd is resumed, we set the "first_vsync" value as false for HDMI channel
		 */
		omap_dispc_set_first_vsync(OMAP_DSS_CHANNEL_DIGIT, false);

		if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE){
			dsi_video_mode_enable(dssdev, 0x3e);
			mdelay(30);
		}
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

	lh430wv5_panel_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev, true, false);
err0:
	return r;
}

static void lh430wv5_panel_power_off(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE)
		dsi_video_mode_disable(dssdev);

	r = lh430wv5_panel_dcs_write_0(td, DCS_DISPLAY_OFF);
	if (!r) 
		r = lh430wv5_panel_sleep_in(td);
	
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

static int lh430wv5_panel_reset(struct omap_dss_device *dssdev)
{
	dev_err(&dssdev->dev, "performing LCD reset\n");

	lh430wv5_panel_power_off(dssdev);
	return lh430wv5_panel_power_on(dssdev);
}

static int lh430wv5_panel_enable(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	printk(KERN_ERR "[HYUN] %s\n", __func__);
	
	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&td->lock);

#if !defined(CONFIG_MACH_LGE_CX2)
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}
#endif //##
	dsi_bus_lock(dssdev);

	r = lh430wv5_panel_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		goto err;

	lh430wv5_panel_queue_esd_work(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&td->lock);

	return 0;
err:
	dev_dbg(&dssdev->dev, "enable failed\n");
	mutex_unlock(&td->lock);
	return r;
}

static void lh430wv5_panel_disable(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&td->lock);

	lh430wv5_panel_cancel_ulps_work(dssdev);
	lh430wv5_panel_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		int r;

		r = lh430wv5_panel_wake_up(dssdev);
		if (!r)
			lh430wv5_panel_power_off(dssdev);
	}

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&td->lock);
}

static int lh430wv5_panel_suspend(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "suspend\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	lh430wv5_panel_cancel_ulps_work(dssdev);
	lh430wv5_panel_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	r = lh430wv5_panel_wake_up(dssdev);
	if (!r)
		lh430wv5_panel_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	mutex_unlock(&td->lock);

	return 0;
err:
	mutex_unlock(&td->lock);
	return r;
}

static int lh430wv5_panel_resume(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "resume\n");

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}
	
	dsi_bus_lock(dssdev);

	r = lh430wv5_panel_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		lh430wv5_panel_queue_esd_work(dssdev);
	}

	mutex_unlock(&td->lock);

	return r;
err:
	mutex_unlock(&td->lock);
	return r;
}

/* LGE_SJIT 2012-02-15 [choongryeol.lee@lge.com]
  *  When lcd is turned on, the garbage image can be displayed in command mode
  *  The root cause of this problem is that DCS_DISPLAY_ON commnad is issued
  *  before image data writting to the lcd gram.
  *  So we send DCS_DISPLAY_ON command after first frame is written to lcd gram
  */
static void lh430wv5_panel_display_on_work(struct work_struct *work)
{
	struct lh430wv5_panel_data *td = container_of(work, struct lh430wv5_panel_data,
			display_on_work);
	struct omap_dss_device *dssdev = td->dssdev;
	int r;

	dev_dbg(&dssdev->dev, "display_on_worker\n");

	dsi_bus_lock(dssdev);
	r = lh430wv5_panel_dcs_write_0(td, DCS_DISPLAY_ON); // display ON
	if(r)
		dev_err(&dssdev->dev, "display on fail\n");
	r = _lh430wv5_panel_enable_te(dssdev, true); // enable TE
	if(r)
		dev_err(&dssdev->dev, "TE enable fail\n");
	td->display_on = true;
	dsi_bus_unlock(dssdev);
}

static void lh430wv5_panel_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(dssdev);
	if(!td->display_on)
		schedule_work(&td->display_on_work);
}

static irqreturn_t lh430wv5_panel_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int old;
	int r;

	old = atomic_cmpxchg(&td->do_update, 1, 0);

	if (old) {
		cancel_delayed_work(&td->te_timeout_work);

		r = omap_dsi_update(dssdev, td->channel,
				td->update_region.x,
				td->update_region.y,
				td->update_region.w,
				td->update_region.h,
				lh430wv5_panel_framedone_cb, dssdev);
		if (r)
			goto err;
	}

	return IRQ_HANDLED;
err:
	dev_err(&dssdev->dev, "start update failed\n");
	dsi_bus_unlock(dssdev);
	return IRQ_HANDLED;
}

static void lh430wv5_panel_te_timeout_work_callback(struct work_struct *work)
{
	struct lh430wv5_panel_data *td = container_of(work, struct lh430wv5_panel_data,
					te_timeout_work.work);
	struct omap_dss_device *dssdev = td->dssdev;

	dev_err(&dssdev->dev, "TE not received for 250ms!\n");

	atomic_set(&td->do_update, 0);
	dsi_bus_unlock(dssdev);
}

static int lh430wv5_panel_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&td->lock);
	dsi_bus_lock(dssdev);

	r = lh430wv5_panel_wake_up(dssdev);
	if (r)
		goto err;

	if (!td->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	r = lh430wv5_panel_set_update_window(td, x, y, w, h);
	if (r)
		goto err;

	if (td->te_enabled && panel_data->use_ext_te) {
		td->update_region.x = x;
		td->update_region.y = y;
		td->update_region.w = w;
		td->update_region.h = h;
		barrier();
		schedule_delayed_work(&td->te_timeout_work,
				msecs_to_jiffies(250));
		atomic_set(&td->do_update, 1);
	} else {
		r = omap_dsi_update(dssdev, td->channel, x, y, w, h,
				lh430wv5_panel_framedone_cb, dssdev);
		if (r)
			goto err;
	}

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&td->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
	return r;
}

static int lh430wv5_panel_sync(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&td->lock);
	dsi_bus_lock(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int _lh430wv5_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	struct lge_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	if (enable)
		r = lh430wv5_panel_dcs_write_1(td, DCS_TEAR_ON, 0);
	else
		r = lh430wv5_panel_dcs_write_0(td, DCS_TEAR_OFF);

	if (!panel_data->use_ext_te)
		omapdss_dsi_enable_te(dssdev, enable);

	if (td->panel_config->sleep.enable_te)
		msleep(td->panel_config->sleep.enable_te);

	td->te_enabled = enable;

	return r;
}

static int lh430wv5_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);

	if (td->te_enabled == enable)
		goto end;

	dsi_bus_lock(dssdev);

	if (td->enabled) {
		r = lh430wv5_panel_wake_up(dssdev);
		if (r)
			goto err;

		r = _lh430wv5_panel_enable_te(dssdev, enable);
		if (r)
			goto err;
	}

	dsi_bus_unlock(dssdev);
end:
	mutex_unlock(&td->lock);

	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);

	return r;
}

static int lh430wv5_panel_get_te(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->te_enabled;
	mutex_unlock(&td->lock);

	return r;
}

static int lh430wv5_panel_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "rotate %d\n", rotate);

	mutex_lock(&td->lock);

	if (td->rotate == rotate)
		goto end;

	dsi_bus_lock(dssdev);

	if (td->enabled) {
		r = lh430wv5_panel_wake_up(dssdev);
		if (r)
			goto err;

		r = lh430wv5_panel_set_addr_mode(td, rotate, td->mirror);
		if (r)
			goto err;
	}

	td->rotate = rotate;

	dsi_bus_unlock(dssdev);
end:
	mutex_unlock(&td->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
	return r;
}

static u8 lh430wv5_panel_get_rotate(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->rotate;
	mutex_unlock(&td->lock);

	return r;
}

static int lh430wv5_panel_mirror(struct omap_dss_device *dssdev, bool enable)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "mirror %d\n", enable);

	mutex_lock(&td->lock);

	if (td->mirror == enable)
		goto end;

	dsi_bus_lock(dssdev);
	if (td->enabled) {
		r = lh430wv5_panel_wake_up(dssdev);
		if (r)
			goto err;

		r = lh430wv5_panel_set_addr_mode(td, td->rotate, enable);
		if (r)
			goto err;
	}

	td->mirror = enable;

	dsi_bus_unlock(dssdev);
end:
	mutex_unlock(&td->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
	return r;
}

static bool lh430wv5_panel_get_mirror(struct omap_dss_device *dssdev)
{
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&td->lock);
	r = td->mirror;
	mutex_unlock(&td->lock);

	return r;
}

static int lh430wv5_panel_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;
	struct lh430wv5_panel_data *td = dev_get_drvdata(&dssdev->dev);

	if (size < w * h * 3)
		return -ENOMEM;

	mutex_lock(&td->lock);

	if (!td->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * 3,
			dssdev->panel.timings.x_res *
			dssdev->panel.timings.y_res * 3);

	dsi_bus_lock(dssdev);

	r = lh430wv5_panel_wake_up(dssdev);
	if (r)
		goto err2;

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	 * use short packets. plen 32 works, but bigger packets seem to cause
	 * an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	lh430wv5_panel_set_update_window(td, x, y, w, h);

	r = dsi_vc_set_max_rx_packet_size(dssdev, td->channel, plen);
	if (r)
		goto err2;

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = dsi_vc_dcs_read(dssdev, td->channel, dcs_cmd,
				buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(&dssdev->dev, "read error\n");
			goto err3;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&dssdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
					"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}

	r = buf_used;

err3:
	dsi_vc_set_max_rx_packet_size(dssdev, td->channel, 1);
err2:
	dsi_bus_unlock(dssdev);
err1:
	mutex_unlock(&td->lock);
	return r;
}

static void lh430wv5_panel_ulps_work(struct work_struct *work)
{
	struct lh430wv5_panel_data *td = container_of(work, struct lh430wv5_panel_data,
			ulps_work.work);
	struct omap_dss_device *dssdev = td->dssdev;

	mutex_lock(&td->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE || !td->enabled) {
		mutex_unlock(&td->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	lh430wv5_panel_enter_ulps(dssdev);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&td->lock);
}

static void lh430wv5_panel_esd_work(struct work_struct *work)
{
	struct lh430wv5_panel_data *td = container_of(work, struct lh430wv5_panel_data,
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

	r = lh430wv5_panel_wake_up(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to exit ULPS\n");
		goto err;
	}

	r = lh430wv5_panel_dcs_read_1(td, DCS_RDDSDR, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read lh430wv5_panel status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = lh430wv5_panel_sleep_out(td);
	if (r) {
		dev_err(&dssdev->dev, "failed to run lh430wv5_panel self-diagnostics\n");
		goto err;
	}

	r = lh430wv5_panel_dcs_read_1(td, DCS_RDDSDR, &state2);
	if (r) {
		dev_err(&dssdev->dev, "failed to read lh430wv5_panel status\n");
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

	lh430wv5_panel_queue_esd_work(dssdev);

	mutex_unlock(&td->lock);
	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	lh430wv5_panel_reset(dssdev);

	dsi_bus_unlock(dssdev);

	lh430wv5_panel_queue_esd_work(dssdev);

	mutex_unlock(&td->lock);
}

static int lh430wv5_panel_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	int update_mode;
	
	if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE)
		update_mode = OMAP_DSS_UPDATE_MANUAL;
	else
		update_mode = OMAP_DSS_UPDATE_AUTO;
		
	if (mode != update_mode)
		return -EINVAL;
	return 0;
}

static enum omap_dss_update_mode lh430wv5_panel_get_update_mode(
		struct omap_dss_device *dssdev)
{
	int update_mode;

	if(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE)
		update_mode = OMAP_DSS_UPDATE_MANUAL;
	else
		update_mode = OMAP_DSS_UPDATE_AUTO;

	return update_mode;
}

static struct omap_dss_driver lh430wv5_panel_driver = {
	.probe		= lh430wv5_panel_probe,
	.remove		= __exit_p(lh430wv5_panel_remove),

	.enable		= lh430wv5_panel_enable,
	.disable	= lh430wv5_panel_disable,
	.suspend	= lh430wv5_panel_suspend,
	.resume		= lh430wv5_panel_resume,

	.set_update_mode = lh430wv5_panel_set_update_mode,
	.get_update_mode = lh430wv5_panel_get_update_mode,

	.update		= lh430wv5_panel_update,
	.sync		= lh430wv5_panel_sync,

	.get_resolution	= lh430wv5_panel_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.enable_te	= lh430wv5_panel_enable_te,
	.get_te		= lh430wv5_panel_get_te,

	.set_rotate	= lh430wv5_panel_rotate,
	.get_rotate	= lh430wv5_panel_get_rotate,
	.set_mirror	= lh430wv5_panel_mirror,
	.get_mirror	= lh430wv5_panel_get_mirror,
	.memory_read	= lh430wv5_panel_memory_read,

	.get_timings	= lh430wv5_panel_get_timings,
	.enable_s3d 	 = lh430wv5_panel_enable_s3d,
	.get_s3d_enabled = lh430wv5_panel_get_s3d_enabled,

	.driver         = {
		.name 	= "lh430wv5_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init lh430wv5_panel_init(void)
{
	omap_dss_register_driver(&lh430wv5_panel_driver);

	return 0;
}

static void __exit lh430wv5_panel_exit(void)
{
	omap_dss_unregister_driver(&lh430wv5_panel_driver);
}

module_init(lh430wv5_panel_init);
module_exit(lh430wv5_panel_exit);

MODULE_AUTHOR("choongryeol.lee  <choongryeol.lee@lge.com>");
MODULE_DESCRIPTION("lh430wv5_panel Driver");
MODULE_LICENSE("GPL");
