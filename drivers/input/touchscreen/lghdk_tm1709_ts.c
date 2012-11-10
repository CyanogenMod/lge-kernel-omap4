/*  drivers/input/keyboard/synaptics_i2c_rmi.c *  * Copyright  (C) 2007  Google,
Inc. * * This software is licensed  under the terms of the GNU General  Public *
License version 2, as  published by the Free  Software Foundation, and *  may be
copied,  distributed,  and modified  under  those terms.  *  * This  program  is
distributed in  the hope  that it  will be  useful, *  but WITHOUT ANY WARRANTY;
without  even  the  implied  warranty of  *  MERCHANTABILITY  or  FITNESS FOR  A
PARTICULAR PURPOSE.  See the * GNU General Public License for more details. * */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/gpio.h>

#include "synaptics_ts_firmware_rev_c.h"

#define SYNAPTICS_TOUCH_DEBUG 1
 #if SYNAPTICS_TOUCH_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif

#define LGHDK_PENDING_TOUCHKEY 		// for preventing wrong touchkey reporting 
#define LGHDK_TOUCH_SET_THRESHOLD   // for preventing jitter during holding
#define LGHDK_TOGGLE_MELT_MODE	    // for helding melt mode ( melt mode for preventing ghost finger)
#define LGHDK_TOUCHKEY_RANGE_TRIM	// for triming touch event out of touchkey range
#define LGHDK_MORE_THREE_FINGER_SPPORT	
#define LGHDK_SYNAPTICS_SUPPORT_FW_UPGRADE
#define LGHDK_TOUCH_GRIP_SUPPRESSION
#define LGHDK_TOUCH_HAND_SUPPRESSION
#define LGHDK_LONGPRESS_TOUCH_DURING_BOOTING
#define LGHDK_USED_RESET_PIN_IN_SUSPEND

#define SYNAPTICS_INT_REG		0x50
#define SYNAPTICS_INT_FLASH		1<<0
#define SYNAPTICS_INT_STATUS 	1<<1
#define SYNAPTICS_INT_ABS0 		1<<2
#define SYNAPTICS_INT_BUTTON	1<<3

#define SYNAPTICS_RMI_QUERY_BASE_REG			0xE3
#define SYNAPTICS_RMI_CMD_BASE_REG				0xE4
#define SYNAPTICS_FLASH_QUERY_BASE_REG			0xE9
#define SYNAPTICS_FLASH_DATA_BASE_REG			0xEC

#define SYNAPTICS_CONTROL_REG		0x4F
#define SYNAPTICS_X_THRESH_REG	0x53
#define SYNAPTICS_Y_THRESH_REG	0x54
#define SYNAPTICS_RESET_REG		0xA4
#define SYNAPTICS_FW_REG	0xAB 

#define SYNAPTICS_FAMILY			0xb1
#define SYNAPTICS_REVISION			0xb2

#define SYNAPTICS_CONTROL_SLEEP 	1<<0
#define SYNAPTICS_CONTROL_NOSLEEP	1<<2
#define SYNAPTICS_INT_ABS0 		1<<2
#define SYNAPTICS_XY_THRESH	0x03

#ifdef LGHDK_SYNAPTICS_SUPPORT_FW_UPGRADE
#define SYNAPTICS_FLASH_CMD_FW_CRC				0x01
#define SYNAPTICS_FLASH_CMD_FW_WRITE			0x02
#define SYNAPTICS_FLASH_CMD_ERASEALL			0x03
#define SYNAPTICS_FLASH_CMD_CONFIG_READ			0x05
#define SYNAPTICS_FLASH_CMD_CONFIG_WRITE		0x06
#define SYNAPTICS_FLASH_CMD_CONFIG_ERASE		0x07
#define SYNAPTICS_FLASH_CMD_ENABLE				0x0F
#define SYNAPTICS_FLASH_NORMAL_RESULT			0x80

#define	SYNAPTICS_TOUCH_MULTI_PANEL_SUPPORT		0
#if SYNAPTICS_TOUCH_MULTI_PANEL_SUPPORT
	#define FW_IMAGE_SIZE 	28929
	unsigned char SynapticsFirmware[FW_IMAGE_SIZE];
#endif

#define TOUCH_INT_N_GPIO						52        //Reference kernel\arch\arm\march-omap2\Board-Cosmopolitan.c

#endif //LGHDK_SYNAPTICS_SUPPORT_FW_UPGRADE

static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	struct delayed_work init_delayed_work;
#ifdef LGHDK_LONGPRESS_TOUCH_DURING_BOOTING	
	struct delayed_work init_delayed_no_melt_work;
#endif
	struct delayed_work touchkey_delayed_work;
#ifdef LGHDK_USED_RESET_PIN_IN_SUSPEND
	struct delayed_work reset_delayed_work;
#endif
	uint32_t flags;
	int (*power)(int on);
	int x_lastpt;
	int y_lastpt;
	struct early_suspend early_suspend;
	uint32_t	button_state;
#ifdef LGHDK_PENDING_TOUCHKEY
	int				pending_touchkey;
	struct timespec	pending_delay_time;
#endif
#ifdef LGHDK_TOGGLE_MELT_MODE
	int				melt_toggle_state;
	struct timespec short_touch_delay_time;
#endif
#ifdef LGHDK_TOUCH_SET_THRESHOLD
	int				count_for_enter_reducemode;
	int				x_pos_for_toggle_reportmode;
	int				y_pos_for_toggle_reportmode;
	int				report_mode;
#endif

};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

/*===========================================================================
                DEFINITIONS AND DECLARATIONS FOR MODULE

This section contains definitions for constants, macros, types, variables
and other items needed by this module.
===========================================================================*/

/*                               Macros                                    */
// 0x00 - not present, 0x01 - present & accurate, 0x10 - present but not accurate, 0x11 - Reserved
#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)(low_reg&0x0F))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)((low_reg&0xF0)/0x10))

#define FINGER_MAX 10 
#define TS_W  1123
#define TS_H  1872

#ifdef LGHDK_PENDING_TOUCHKEY
#define LGHDK_PENDING_TIME	    	175 * NSEC_PER_MSEC // 300ms

#define LGHDK_PENDING_IDLE_STATE				-1
#define LGHDK_PENDING_HANDLED					-2
#define LGHDK_PENDING_SECOND_TOUCH_DETECTED		-3
#endif

#ifdef LGHDK_TOGGLE_MELT_MODE
#define LGHDK_MELT_IN_MELTMODE			0
#define LGHDK_MELT_TRY_ENTER_NOMELT		1
#define LGHDK_MELT_BLOCK_ENTER_NOMELT	2
#define LGHDK_MELT_IN_NOMELTMODE		3

#define LGHDK_MELT_SYNAPTICS_MELT_VALUE		0x01
#define LGHDK_MELT_SYNAPTICS_NOMELT_VALUE	0x00

#define LGHDK_MELT_SHORT_TAP_MAX_TIME	2 // 2sec
#endif


#ifdef LGHDK_TOUCH_SET_THRESHOLD
#define LGHDK_SYNAPTICS_REG_REPORT_MODE	0x51
#define LGHDK_SYNAPTICS_REG_DELTA_X_THRESH	0x53
#define LGHDK_SYNAPTICS_REG_DELTA_Y_THRESH	0x54

#define LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE	0x00
#define LGHDK_SYNAPTICS_VAL_REDUCE_REPORT_MODE	0x01
#define LGHDK_SYNAPTICS_VAL_ABS_POS_FILTER		0x08


//#define LGHDK_SYNAPTICS_VAL_THRESHOLD		0x20
#define LGHDK_SYNAPTICS_VAL_THRESHOLD		0x02


#define	LGHDK_SYNAPTICS_ENTER_HOLD_COUNT	10


#define	LGHDK_SYNAPTICS_JITTER_DIRECTION_UP	1
#define	LGHDK_SYNAPTICS_JITTER_DIRECTION_DOWN	2
#define	LGHDK_SYNAPTICS_JITTER_DIRECTION_LEFT	3
#define	LGHDK_SYNAPTICS_JITTER_DIRECTION_RIGHT	4
#define	LGHDK_SYNAPTICS_JITTER_DIRECTION_CENTER	5

#endif

#define	LGHDK_SYNAPTICS_REG_FINGER_STATUS0 0x15
#define	LGHDK_SYNAPTICS_REG_FINGER_STATUS1 0x16
#define	LGHDK_SYNAPTICS_REG_FINGER_STATUS2 0x17

#define LGHDK_COUNT_PER_FINGER_STATUS		4

#define LGHDK_SYNAPTICS_REG_FINGER_DATA_START_ADDR		0x18
#define LGHDK_SYNAPTICS_REG_FINGER_DATA_GAP				0x05

#define LGHDK_SYNAPTICS_REG_FINGER_VALID_DATA_SIZE		5


#define SYNAPTICS_FLASH_CONTROL_REG				0x12
#define SYNAPTICS_DATA_BASE_REG					0x13
#define SYNAPTICS_INT_STATUS_REG				0x14
#define GESTURE_FLAGS	0x4A


typedef struct
{
	unsigned char device_status_reg;            //0x13
	unsigned char interrupt_status_reg;			//0x14
	unsigned char finger_state_reg[3];			//0x15~0x17
	// Finger 0
	unsigned char X_high_position_finger0_reg;  //0x18
	unsigned char Y_high_position_finger0_reg;	//0x19
	unsigned char XY_low_position_finger0_reg;	//0x1A
	unsigned char XY_width_finger0_reg;			//0x1B
	unsigned char Z_finger0_reg;				//0x1C
	// Finger 1
	unsigned char X_high_position_finger1_reg;  //0x1D
	unsigned char Y_high_position_finger1_reg;	//0x1E
	unsigned char XY_low_position_finger1_reg;	//0x1F
	unsigned char XY_width_finger1_reg;			//0x20
	unsigned char Z_finger1_reg;				//0x21
} ts_sensor_data;

typedef struct {
	unsigned char finger_count;
	int X_position[FINGER_MAX];
	int Y_position[FINGER_MAX];
	int width[FINGER_MAX];
	int pressure[FINGER_MAX];
} ts_finger_data;

static ts_sensor_data ts_reg_data={0};
static ts_finger_data curr_ts_data;

#ifdef LGHDK_TOUCH_GRIP_SUPPRESSION
static int g_gripIgnoreRangeValue = 0;
static int g_receivedPixelValue = 0;
#endif

#ifdef LGHDK_TOUCH_HAND_SUPPRESSION
static int g_handIgnoreValue = 0;
#endif

unsigned char  touch_fw_version = 0;

#if defined(CONFIG_MACH_LGE_COSMO_DOMASTIC)
#define LGHDK_VALID_TOUCHKEY_COUNT	3
// Darren.kang move for global usage
static unsigned int button_map[LGHDK_VALID_TOUCHKEY_COUNT + 1] = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_UNKNOWN};
#else

#define LGHDK_VALID_TOUCHKEY_COUNT	4
// Darren.kang move for global usage
static unsigned int button_map[LGHDK_VALID_TOUCHKEY_COUNT + 1] = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH, KEY_UNKNOWN};
#endif




static int omap_virtualkeymap[] = {
	KEY_1,				
	KEY_2,
	KEY_3,				
	KEY_4,				
	KEY_5,				
	KEY_6,				
	KEY_7,				
	KEY_8,				
	KEY_9,		
	KEY_0,	
//	KEY_STAR,
//	KEY_SHARP,		 
	KEY_A,
	KEY_B,				
	KEY_C,				
	KEY_D,				
	KEY_E,				
	KEY_F,				
	KEY_G,				
	KEY_H,				
	KEY_I,				
	KEY_J,				
	KEY_K,				
	KEY_L,				
	KEY_M,				
	KEY_N,				
	KEY_O,				
	KEY_P,				
	KEY_Q,				
	KEY_R,				
	KEY_S,				
	KEY_T,				
	KEY_U,				
	KEY_V,				
	KEY_W,				
	KEY_X,				
	KEY_Y,				
	KEY_Z,				
	KEY_VOLUMEUP,
	KEY_VOLUMEDOWN,
	KEY_BACKSPACE,
	KEY_LEFTALT,		
	KEY_LEFTSHIFT,
	KEY_SPACE,
	KEY_COMMA,
	KEY_DOT,	
	KEY_SEND,		
	KEY_END,			
	KEY_ENTER,
	KEY_RIGHT,
	KEY_LEFT,		
	KEY_DOWN,		
	KEY_UP,		
	KEY_CAMERA,	
	0,
};

static bool synaptics_ts_handle_is_ignorearea(int xposition, int yposition)
{
	if(g_handIgnoreValue != 0) return false;

	if(xposition <= 13   && yposition <= 2)
	{
		DEBUG_MSG("%s() ------ 1 [%d][%d]\n", __func__, xposition, yposition);
		return true;
	}

	if(xposition >= 1120   && yposition <= 2)
	{
		DEBUG_MSG("%s() ------ 2 [%d][%d]\n", __func__, xposition, yposition);
		return true;
	}	

	if(xposition <= 5   && yposition >= 1802)
	{
		DEBUG_MSG("%s() ------ 3 [%d][%d]\n", __func__, xposition, yposition);
		return true;
	}	

	if(xposition >= 1076   && yposition >= 1769)
	{
		DEBUG_MSG("%s() ------ 4 [%d][%d]\n", __func__, xposition, yposition);
		return true;
	}		
	
	return false;
}


#ifdef LGHDK_PENDING_TOUCHKEY
static int synaptics_ts_handle_detected_touch_key(struct synaptics_ts_data* ts,int btn_index)
{
	int touch_prestate = 0;
	int ret = 0;

	DEBUG_MSG("%s() pending_touchkey[%d] btn_index[%d] \n", __func__, ts->pending_touchkey, btn_index);

	if(ts->pending_touchkey == LGHDK_PENDING_IDLE_STATE)
	{
		//first detect
		ts->pending_touchkey = btn_index;
		ts->pending_delay_time = current_kernel_time();
		timespec_add_ns(&ts->pending_delay_time,LGHDK_PENDING_TIME);//LGHDK_PENDING_TIME);
		touch_prestate = 1;
		schedule_delayed_work(&ts->touchkey_delayed_work, msecs_to_jiffies(300));
	}
	else if (ts->pending_touchkey == LGHDK_PENDING_HANDLED)
	{
		//pending is handled , so send event normally
		DEBUG_MSG("%s() 1 btn_index[%d] Press \n", __func__, button_map[btn_index]);
		input_report_key(ts->input_dev, button_map[btn_index], 1);
		input_sync(ts->input_dev);	
		ts->button_state = (1 << btn_index);
	}
	else if (ts->pending_touchkey == LGHDK_PENDING_SECOND_TOUCH_DETECTED)
	{
		// ignore first touch
		touch_prestate = 1;		
	}
	else if(btn_index == ts->pending_touchkey)
	{
		// during pending
		struct timespec cur_time = current_kernel_time();
		if(timespec_compare(&cur_time,&ts->pending_delay_time) >0)
		{
			DEBUG_MSG("%s() 2 btn_index[%d] Press \n", __func__, button_map[btn_index]);
			input_report_key(ts->input_dev, button_map[btn_index], 1);
			input_sync(ts->input_dev);	
			ts->button_state = (1 << btn_index);
			
			ts->pending_touchkey = LGHDK_PENDING_HANDLED;			
			ret = cancel_delayed_work_sync(&ts->touchkey_delayed_work);
		}
		else
		{
			// keep pending
			touch_prestate = 1;
		}
	}
	else
	{
		// key value is changed during pending
		ts->pending_touchkey = btn_index;
		ts->pending_delay_time = current_kernel_time();
		touch_prestate = 1;
		schedule_delayed_work(&ts->touchkey_delayed_work, msecs_to_jiffies(300));
	}	
	
	return touch_prestate;
}

static void synaptics_ts_handle_detected_whole_up_event(struct synaptics_ts_data* ts)
{
	int i =0;
	int ret = 0;
	// if pended touch is not handled, send down,up event.
	if(ts->button_state == 0 && ts->pending_touchkey >= 0)
	{
		DEBUG_MSG("%s() 3 btn_index[%d] Press \n", __func__, button_map[ts->pending_touchkey]);

		input_report_key(ts->input_dev, button_map[ts->pending_touchkey], 1);
		input_sync(ts->input_dev);
		DEBUG_MSG("%s() 4 btn_index[%d] Release \n", __func__, button_map[ts->pending_touchkey]);
		input_report_key(ts->input_dev, button_map[ts->pending_touchkey], 0);
		input_sync(ts->input_dev);

		ts->pending_touchkey = LGHDK_PENDING_IDLE_STATE;
		ret = cancel_delayed_work_sync(&ts->touchkey_delayed_work);
		return;
	}

	for (i = 0; i < LGHDK_VALID_TOUCHKEY_COUNT; i++) {
		const u32 mask = (1 << i);
		if ((ts->button_state & mask)) 
		{	
			// if down event is already sent , up event should be sent
			if (ts->pending_touchkey == LGHDK_PENDING_HANDLED)
			{
				DEBUG_MSG("%s() 5 btn_index[%d] Release \n", __func__, button_map[i]);
				input_report_key(ts->input_dev, button_map[i], 0);
				input_sync(ts->input_dev);
			}
		}
	}
	
	//init pending value
	ts->pending_touchkey = LGHDK_PENDING_IDLE_STATE;
}
#endif
#ifdef LGHDK_TOGGLE_MELT_MODE
void synaptics_check_meltmode_in_touchdown(struct synaptics_ts_data* ts)
{
	if(ts->melt_toggle_state == LGHDK_MELT_IN_MELTMODE)
	{
		// mode change
		ts->melt_toggle_state = LGHDK_MELT_TRY_ENTER_NOMELT;

		// max delay setting
		ts->short_touch_delay_time = current_kernel_time();
		ts->short_touch_delay_time.tv_sec += LGHDK_MELT_SHORT_TAP_MAX_TIME;			
	}

}

void synaptics_check_meltmode_in_touchup(struct synaptics_ts_data* ts)
{
	if(ts->melt_toggle_state == LGHDK_MELT_TRY_ENTER_NOMELT)
	{
		struct timespec cur_time = current_kernel_time();
		if(timespec_compare(&cur_time,&ts->short_touch_delay_time)>0)
		{
			// timeout
			ts->melt_toggle_state = LGHDK_MELT_IN_MELTMODE;				
		}
		else
		{
			// short tap
			ts->melt_toggle_state = LGHDK_MELT_IN_NOMELTMODE;
			//reg set			
			i2c_smbus_write_byte_data(ts->client, 0xf0, LGHDK_MELT_SYNAPTICS_NOMELT_VALUE);	
			DEBUG_MSG("LGHDK_MELT_ENTER_NOMELT\n");
		}		
	}
	else if(ts->melt_toggle_state == LGHDK_MELT_BLOCK_ENTER_NOMELT)
	{
		ts->melt_toggle_state = LGHDK_MELT_IN_MELTMODE;	
	}
}
#endif

#ifdef LGHDK_TOUCH_SET_THRESHOLD
static void	synaptics_init_report_mode(struct synaptics_ts_data* ts)
{
	// init history for previous touch
	ts->x_pos_for_toggle_reportmode = -1;
	ts->y_pos_for_toggle_reportmode = -1;	
	ts->count_for_enter_reducemode = 0;

	// reset report mode to norma mode
	if(ts->report_mode == LGHDK_SYNAPTICS_VAL_REDUCE_REPORT_MODE)
	{
		ts->report_mode = LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE;
		
		//i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_REPORT_MODE, 		ts->report_mode);		
		DEBUG_MSG("enter normal mode\n");
	}
}

static void	synaptics_handle_report_mode_in_touchdown(struct synaptics_ts_data* ts, int* x, int* y)
{
	if(ts->x_lastpt < 0 && ts->y_lastpt < 0)
	{
		// if there is no history, set history
		ts->x_pos_for_toggle_reportmode = *x;
		ts->y_pos_for_toggle_reportmode = *y;
		ts->count_for_enter_reducemode = 0;
	}
	else
	{
		// calculate differnce between current position and previous postion
		int diff_x = abs(*x - ts->x_pos_for_toggle_reportmode);
		int diff_y = abs(*y - ts->y_pos_for_toggle_reportmode);
		
		
		if( diff_x	<= LGHDK_SYNAPTICS_VAL_THRESHOLD && diff_y <= LGHDK_SYNAPTICS_VAL_THRESHOLD)
		{
			// differnce is smaller than threshold 
			if(ts->count_for_enter_reducemode >= LGHDK_SYNAPTICS_ENTER_HOLD_COUNT)
			{				
				if(ts->report_mode == LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE)
				{
					// enter reduce report mode
					ts->report_mode = LGHDK_SYNAPTICS_VAL_REDUCE_REPORT_MODE;
					#if 0
					i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_REPORT_MODE, 	ts->report_mode	);		
					i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_X_THRESH,   LGHDK_SYNAPTICS_VAL_THRESHOLD);
					i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_Y_THRESH, 	LGHDK_SYNAPTICS_VAL_THRESHOLD);
					DEBUG_MSG("enter reduce mode\n");
					#endif

					ts->x_lastpt = *x;
					ts->y_lastpt = *y;
					
					ts->x_pos_for_toggle_reportmode = 0;
					ts->y_pos_for_toggle_reportmode = 0;					
				}
				else
				{
					ts->x_pos_for_toggle_reportmode += *x - ts->x_lastpt;
					ts->y_pos_for_toggle_reportmode += *y - ts->y_lastpt;

					if(ts->x_pos_for_toggle_reportmode > LGHDK_SYNAPTICS_VAL_THRESHOLD || 
						ts->y_pos_for_toggle_reportmode > LGHDK_SYNAPTICS_VAL_THRESHOLD)
					{
						// detected direction move
						synaptics_init_report_mode(ts);
					}										
				}				

				// count is enough for entering reduce mode
				*x = ts->x_lastpt;
				*y = ts->y_lastpt;	
			}
			else
			{
				
				// start check for entering
				if(ts->count_for_enter_reducemode == 0)
				{
					ts->x_pos_for_toggle_reportmode = *x;
					ts->y_pos_for_toggle_reportmode = *y;					
				}									
				
				ts->x_lastpt = *x;
				ts->y_lastpt = *y;
					
				// count is not enough for entering reduce mode, so count is added.
				ts->count_for_enter_reducemode++;
			}
				
		}
		else
		{
			// difference is larger than threshold , so enter normal report mode.
			synaptics_init_report_mode(ts);
		}
		
	}
		
}
#endif


static int synaptics_ts_get_burton_index(struct synaptics_ts_data* ts, int x, int y)
{
#if defined(CONFIG_MACH_LGE_COSMO_DOMASTIC)
#define LGHDK_BTN_EDGE_WIDTH	373
#define LGHDK_BTN_TRIM_WIDTH	60
#else
#define LGHDK_BTN_EDGE_WIDTH	280
#define LGHDK_BTN_TRIM_WIDTH	60
#endif

	int btn_index = 0;

	// minimum value check
	x = (x < 0) ? 0 : x; 

	// maximum value check
	x = (x >= LGHDK_BTN_EDGE_WIDTH * LGHDK_VALID_TOUCHKEY_COUNT) ? LGHDK_BTN_EDGE_WIDTH * LGHDK_VALID_TOUCHKEY_COUNT - 1 :  x;

	// set btn index 
	btn_index  = x / LGHDK_BTN_EDGE_WIDTH;

#ifdef LGHDK_TOUCHKEY_RANGE_TRIM
	{
#if defined(CONFIG_MACH_LGE_COSMO_DOMASTIC)
		int i = x%373;
#else
		int i = x%280;
#endif
		/* for whole button trimming
		if( i < LGHDK_BTN_TRIM_WIDTH || i > LGHDK_BTN_EDGE_WIDTH - LGHDK_BTN_TRIM_WIDTH)
		{
			//touch position is out of valid button area
			btn_index = LGHDK_VALID_TOUCHKEY_COUNT; // set KEY_UNKNOWN			
		}
		*/
		if((i < LGHDK_BTN_TRIM_WIDTH && btn_index == 0) || (i > LGHDK_BTN_EDGE_WIDTH - LGHDK_BTN_TRIM_WIDTH && btn_index == LGHDK_VALID_TOUCHKEY_COUNT -1))
			//touch position is out of valid button area
			btn_index = LGHDK_VALID_TOUCHKEY_COUNT; // set KEY_UNKNOWN			
	}
#endif
	return btn_index;
}


static int synaptics_ts_is_more_pressed_touch(int finger_index)
{
	int tempPressed = 0;
	int tempNextFinger_index = 0;

	for(tempNextFinger_index = finger_index; tempNextFinger_index < FINGER_MAX; tempNextFinger_index++)
	{
		int tempdivided_index = (tempNextFinger_index % LGHDK_COUNT_PER_FINGER_STATUS)*2;	
					
		// check pressed
		tempPressed = (ts_reg_data.finger_state_reg[tempNextFinger_index/LGHDK_COUNT_PER_FINGER_STATUS] & (0x03 << tempdivided_index));

		if(tempPressed != 0)
		{
			return 1;
		}
	}

	return 0;
}

static int ts_pre_state = 0; 

static int synaptics_handle_single_touch(struct synaptics_ts_data* ts, int finger_index)
{
	int ret  = 0; // is next finger existed?

	int pressed = 0;

	int width = 0;

	int divided_index = (finger_index % LGHDK_COUNT_PER_FINGER_STATUS)*2;	

	// check excetional index
	if(finger_index < 0 || finger_index >= FINGER_MAX)
		return ret;


	if(finger_index == 0)
	{
		i2c_smbus_read_i2c_block_data(ts->client, LGHDK_SYNAPTICS_REG_FINGER_STATUS0, 3, (u8*)&(ts_reg_data.finger_state_reg));	
	}

	// check pressed
	pressed = (ts_reg_data.finger_state_reg[finger_index/LGHDK_COUNT_PER_FINGER_STATUS] & (0x03 << divided_index));

	DEBUG_MSG("%s() [%d][%d][%d][%d] \n", __func__, ts_reg_data.finger_state_reg[0], ts_reg_data.finger_state_reg[1], ts_reg_data.finger_state_reg[2], divided_index);
	
	if(pressed != 0)
	{
		i2c_smbus_read_i2c_block_data(ts->client, LGHDK_SYNAPTICS_REG_FINGER_DATA_START_ADDR + (LGHDK_SYNAPTICS_REG_FINGER_DATA_GAP*finger_index), 
										LGHDK_SYNAPTICS_REG_FINGER_VALID_DATA_SIZE, &(ts_reg_data.X_high_position_finger0_reg));

		curr_ts_data.X_position[finger_index] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
		curr_ts_data.Y_position[finger_index] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg); 	

		////////////////////////////////////////////////////////////debug //////////////////////////////////////////
		if(finger_index == 0 && ts->melt_toggle_state == LGHDK_MELT_IN_MELTMODE)
		{
			printk("touch debug[%d][%d]\n", curr_ts_data.X_position[finger_index], curr_ts_data.Y_position[finger_index]);
		}
		//////////////////////////////////////////////////////debug////////////////////////////////////////


		#ifdef LGHDK_TOUCH_GRIP_SUPPRESSION // 20101215 seven@lge.com grip suppression [start]
		if ( (g_gripIgnoreRangeValue > 0) && ( (curr_ts_data.X_position[finger_index] <= g_gripIgnoreRangeValue ) || (curr_ts_data.X_position[finger_index] >= (TS_W - g_gripIgnoreRangeValue) )) )
		{
			DEBUG_MSG("[TOUCH] Girp Region Pressed. IGNORE!!! X[%d] grip[%d] \n",curr_ts_data.X_position[finger_index], g_gripIgnoreRangeValue );
			goto err_input_grip_suppression_failed;
		}
		#endif // 20101215 seven@lge.com grip suppression [start]

		#ifdef LGHDK_TOUCH_HAND_SUPPRESSION
		if(synaptics_ts_handle_is_ignorearea(curr_ts_data.X_position[finger_index], curr_ts_data.Y_position[finger_index]) == true)
		{
			goto err_input_ignorearea_failed;
		}
		#endif

		if ((((ts_reg_data.XY_width_finger0_reg & 240) >> 4) - (ts_reg_data.XY_width_finger0_reg & 15)) > 0)
			width = (ts_reg_data.XY_width_finger0_reg & 240) >> 4;
		else
			width = ts_reg_data.XY_width_finger0_reg & 15;		

			curr_ts_data.width[finger_index] = (int)width;
			curr_ts_data.pressure[finger_index] = (int)ts_reg_data.Z_finger0_reg;

	
		switch(finger_index)
		{
			case 0: //first finger special care
				if(curr_ts_data.Y_position[0] >TS_H) {
							// in button range
							ret = 0;
							if(ts->button_state==0 && ts_pre_state == 0) {
									int btn_index = synaptics_ts_get_burton_index(ts,curr_ts_data.X_position[0],curr_ts_data.Y_position[0]);
									
								#ifdef LGHDK_PENDING_TOUCHKEY
									ret = synaptics_ts_handle_detected_touch_key(ts,btn_index);			
								#else
									input_report_key(ts->input_dev, button_map[btn_index], 1);
									input_sync(ts->input_dev);	
									ts->button_state = (1 << btn_index);
								#endif
							} 
				}else {
							
							ts_pre_state = 1;
						#ifdef LGHDK_PENDING_TOUCHKEY				
							//init pending value
							ts->pending_touchkey = LGHDK_PENDING_IDLE_STATE;
						#endif

						#ifdef LGHDK_TOGGLE_MELT_MODE
							synaptics_check_meltmode_in_touchdown(ts);
						#endif

						#ifdef LGHDK_TOUCH_SET_THRESHOLD
							synaptics_handle_report_mode_in_touchdown(ts,&(curr_ts_data.X_position[0]),&(curr_ts_data.Y_position[0]));
						#endif

			       			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[finger_index]);
			        		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[finger_index]);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[finger_index]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, curr_ts_data.width[finger_index]);
								
							input_mt_sync(ts->input_dev);	
							ret = synaptics_ts_is_more_pressed_touch(finger_index+1);
							DEBUG_MSG("%s() - 1 [%d][%d][%d][%d] \n", __func__, curr_ts_data.X_position[finger_index], curr_ts_data.Y_position[finger_index], ts_reg_data.Z_finger0_reg, width);
				}	
				break;
			case 1: //second finger special care
				#ifdef LGHDK_PENDING_TOUCHKEY
					// if second touch is detected , pended key is ignored
					if(ts->pending_touchkey >= LGHDK_PENDING_IDLE_STATE)
					{
						ts->pending_touchkey = LGHDK_PENDING_SECOND_TOUCH_DETECTED;	
						ret = cancel_delayed_work_sync(&ts->touchkey_delayed_work);
					}
				#endif
				#ifdef LGHDK_TOGGLE_MELT_MODE
					if(ts->melt_toggle_state != LGHDK_MELT_IN_NOMELTMODE)
					{
						ts->melt_toggle_state = LGHDK_MELT_BLOCK_ENTER_NOMELT;
					}
				#endif
					ts_pre_state = 1;
					//break; <---- disable because below code is shared
			default: // common handling
					ret = synaptics_ts_is_more_pressed_touch(finger_index+1);
			//		curr_ts_data.X_position[1] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
		  	//		curr_ts_data.Y_position[1] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg); 			
					
		    /*    	if ((((ts_reg_data.XY_width_finger0_reg & 240) >> 4) - (ts_reg_data.XY_width_finger0_reg & 15)) > 0)
						width = (ts_reg_data.XY_width_finger0_reg & 240) >> 4;
					else
						width = ts_reg_data.XY_width_finger0_reg & 15;
			*/		
		       		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[finger_index]);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[finger_index]);	
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, curr_ts_data.pressure[finger_index]);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, curr_ts_data.width[finger_index]);

					input_mt_sync(ts->input_dev);

					DEBUG_MSG("%s() - 2 [%d][%d][%d][%d] \n", __func__, curr_ts_data.X_position[finger_index], curr_ts_data.Y_position[finger_index], ts_reg_data.Z_finger0_reg, width);
				break;
		}
	}
	else
	{
		err_input_grip_suppression_failed:
		err_input_ignorearea_failed:

		ret = 0;

		if(finger_index == 0) { //first finger special care
			ts_pre_state = 0;			

			#ifdef LGHDK_PENDING_TOUCHKEY
			synaptics_ts_handle_detected_whole_up_event(ts);
			#else			
			for (i = 0; i < LGHDK_VALID_TOUCHKEY_COUNT; i++) {
			const u32 mask = (1 << i);
			if ((ts->button_state & mask)) {
				DEBUG_MSG("%s() 6 btn_index[%d] Press \n", __func__, button_map[i]);
				input_report_key(ts->input_dev, button_map[i], 0);
				input_sync(ts->input_dev);
				}
			}
			#endif		
			ts->button_state=0;				
			#ifdef LGHDK_TOGGLE_MELT_MODE
			synaptics_check_meltmode_in_touchup(ts);
			#endif

			#ifdef LGHDK_TOUCH_SET_THRESHOLD
			synaptics_init_report_mode(ts);
			#endif
		}

		ret = synaptics_ts_is_more_pressed_touch(finger_index+1);

		DEBUG_MSG("%s() - 3 [%d]\n", __func__, finger_index);
	}

	return ret;	
}

static void synaptics_ts_init_delayed_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, init_delayed_work);
	int ret  = 0;

	ret = i2c_smbus_read_i2c_block_data(ts->client, SYNAPTICS_DATA_BASE_REG, sizeof(ts_reg_data), (u8*)&ts_reg_data);

	printk("%s() - 3 [%d][%d]\n", __func__, ret, gpio_get_value(TOUCH_INT_N_GPIO));	

	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP);
#ifdef LGHDK_TOUCH_SET_THRESHOLD
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_INT_REG, SYNAPTICS_INT_ABS0);
	
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_REPORT_MODE,		LGHDK_SYNAPTICS_VAL_REDUCE_REPORT_MODE|LGHDK_SYNAPTICS_VAL_ABS_POS_FILTER);
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_X_THRESH,	LGHDK_SYNAPTICS_VAL_THRESHOLD);
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_Y_THRESH,	LGHDK_SYNAPTICS_VAL_THRESHOLD);
#endif

#ifdef LGHDK_TOGGLE_MELT_MODE
	ret = i2c_smbus_write_byte_data(ts->client, 0xf0, LGHDK_MELT_SYNAPTICS_MELT_VALUE);
	DEBUG_MSG("LGHDK_ENTER_MELT\n");
#endif

}


#ifdef LGHDK_LONGPRESS_TOUCH_DURING_BOOTING
static void synaptics_ts_init_delayed_No_Melt_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, init_delayed_no_melt_work);
	int ret  = 0;

	DEBUG_MSG("%s() - 3 [%x]\n", __func__, ts);

#ifdef LGHDK_TOGGLE_MELT_MODE
	ts->melt_toggle_state = LGHDK_MELT_IN_NOMELTMODE;
	ret = i2c_smbus_write_byte_data(ts->client, 0xf0, LGHDK_MELT_SYNAPTICS_NOMELT_VALUE);
	DEBUG_MSG("LGHDK_MELT_ENTER_NOMELT\n");
#endif

}
#endif

static void synaptics_ts_touchkey_delayed_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, touchkey_delayed_work);

	DEBUG_MSG("%s() pending_touchkey[%d] \n", __func__, ts->pending_touchkey);	

	if(ts->pending_touchkey >= 0)
	{
		DEBUG_MSG("%s() 7 btn_index[%d] Press \n", __func__, button_map[ts->pending_touchkey]);
		input_report_key(ts->input_dev, button_map[ts->pending_touchkey], 1);
		input_sync(ts->input_dev);	
		ts->button_state = (1 << ts->pending_touchkey);
		
		ts->pending_touchkey = LGHDK_PENDING_HANDLED;			
	}

}

#ifdef LGHDK_USED_RESET_PIN_IN_SUSPEND
static void synaptics_ts_reset_delayed_work(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, reset_delayed_work);

	printk("%s() \n", __func__);	

	ret = i2c_smbus_read_i2c_block_data(ts->client, SYNAPTICS_DATA_BASE_REG, sizeof(ts_reg_data), (u8*)&ts_reg_data);

	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP);
#ifdef LGHDK_TOUCH_SET_THRESHOLD
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_INT_REG, SYNAPTICS_INT_ABS0);
	
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_REPORT_MODE,		LGHDK_SYNAPTICS_VAL_REDUCE_REPORT_MODE|LGHDK_SYNAPTICS_VAL_ABS_POS_FILTER);
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_X_THRESH,	LGHDK_SYNAPTICS_VAL_THRESHOLD);
	i2c_smbus_write_byte_data(ts->client, LGHDK_SYNAPTICS_REG_DELTA_Y_THRESH,	LGHDK_SYNAPTICS_VAL_THRESHOLD);
#endif

	synaptics_init_report_mode(ts);

	ts->report_mode = LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE;

	ts->melt_toggle_state = LGHDK_MELT_IN_MELTMODE;


#ifdef LGHDK_TOGGLE_MELT_MODE
	ret = i2c_smbus_write_byte_data(ts->client, 0xf0, LGHDK_MELT_SYNAPTICS_MELT_VALUE);
	DEBUG_MSG("LGHDK_ENTER_MELT\n");
#endif

	enable_irq(gpio_to_irq(ts->client->irq));

}
#endif


static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	int int_mode = i2c_smbus_read_byte_data(ts->client, 0x14);

	DEBUG_MSG("%s() 0x%x \n", __func__, int_mode);

	if(int_mode & SYNAPTICS_INT_ABS0) {
			
		int is_next_finger_existed = 0;		// is next finger existed?

		int i = 0;

		do {
			is_next_finger_existed = synaptics_handle_single_touch(ts,i);
			
			i++;
		} while(is_next_finger_existed == 1 && i < FINGER_MAX)		;

		input_mt_sync(ts->input_dev);               
		input_sync(ts->input_dev);	
	}

	enable_irq(gpio_to_irq(ts->client->irq));

}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	disable_irq_nosync(gpio_to_irq(ts->client->irq));
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static unsigned char synaptics_ts_check_fwver(struct i2c_client *client)
{
	unsigned char RMI_Query_BaseAddr;
	unsigned char FWVersion_Addr;

	unsigned char SynapticsFirmVersion;

	RMI_Query_BaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_QUERY_BASE_REG);
	FWVersion_Addr = RMI_Query_BaseAddr+3;
	
	SynapticsFirmVersion = i2c_smbus_read_byte_data(client, FWVersion_Addr);
	DEBUG_MSG("[TOUCH] Touch controller Firmware Version = %x\n", SynapticsFirmVersion);

	return SynapticsFirmVersion;
}


static ssize_t	version_show(struct device* dev, struct device_attribute* attr,
								const char* buf, size_t count)
{
#if 1
	return snprintf(buf, PAGE_SIZE,"%d\n", touch_fw_version);
#else
	struct i2c_client*	client	=	to_i2c_client(dev);
	int		family, revision;

	if ((family = i2c_smbus_read_byte_data(client, SYNAPTICS_FAMILY)) < 0)
		goto	err;

	if ((revision = i2c_smbus_read_byte_data(client, SYNAPTICS_REVISION)) < 0)
		goto	err;

	return	snprintf(buf, PAGE_SIZE, "%d.%d\n", family, revision);

err:
	return	snprintf(buf, PAGE_SIZE, "Unknown\n");
#endif	
}

static DEVICE_ATTR(version, 0444, version_show, NULL);


// 20101215 seven@lge.com grip suppression [START]
#ifdef LGHDK_TOUCH_GRIP_SUPPRESSION
static int touch_ConvertPixelToRawData(int pixel)
{
	int result = 0;

	result = (TS_W * pixel) /480;

	return result;
}

ssize_t touch_gripsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", g_receivedPixelValue);	
	pr_debug("[KERNEL] [TOUCH] SHOW (%d) \n", g_receivedPixelValue);

	return (ssize_t)(strlen(buf)+1);

}

ssize_t touch_gripsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	sscanf(buffer, "%d", &g_receivedPixelValue);
	g_gripIgnoreRangeValue = touch_ConvertPixelToRawData(g_receivedPixelValue);
	pr_debug("[KERNEL] [TOUCH] STORE  pixel(%d) Convet (%d) \n", g_receivedPixelValue, g_gripIgnoreRangeValue);
	
	return count;
}

DEVICE_ATTR(gripsuppression, 0664, touch_gripsuppression_show, touch_gripsuppression_store);
#endif /* LGHDK_TOUCH_GRIP_SUPPRESSION */
// 20101215 seven@lge.com grip suppression [END]

#ifdef LGHDK_TOUCH_HAND_SUPPRESSION

ssize_t touch_handsuppression_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "%d\n", g_handIgnoreValue);	

	return (ssize_t)(strlen(buf)+1);

}

ssize_t touch_handsuppression_store(struct device *dev, struct device_attribute *attr, const char *buffer, size_t count)
{
	sscanf(buffer, "%d", &g_handIgnoreValue);

	return count;
}

DEVICE_ATTR(handsuppression, 0664, touch_handsuppression_show, touch_handsuppression_store);
#endif


#ifdef LGHDK_SYNAPTICS_SUPPORT_FW_UPGRADE

static unsigned long ExtractLongFromHeader(const unsigned char *SynaImage)  // Endian agnostic 
{
  return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}



static void CalculateChecksum(uint16_t *data, uint16_t len, uint32_t *dataBlock)
{
  unsigned long temp = *data++;
  unsigned long sum1;
  unsigned long sum2;

  *dataBlock = 0xffffffff;

  sum1 = *dataBlock & 0xFFFF;
  sum2 = *dataBlock >> 16;

  while (len--)
  {
    sum1 += temp;    
    sum2 += sum1;    
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);    
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }

  *dataBlock = sum2 << 16 | sum1;
}

static void SpecialCopyEndianAgnostic(uint8_t *dest, uint16_t src) 
{
  dest[0] = src%0x100;  //Endian agnostic method
  dest[1] = src/0x100;  
}

static bool synaptics_ts_fw_upgrade(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	int i;
	int j;

	uint8_t FlashQueryBaseAddr, FlashDataBaseAddr;
	uint8_t RMICommandBaseAddr;
	
	uint8_t BootloaderIDAddr;
	uint8_t BlockSizeAddr;
	uint8_t FirmwareBlockCountAddr;
	uint8_t ConfigBlockCountAddr;

	uint8_t BlockNumAddr;
	uint8_t BlockDataStartAddr;

	uint8_t current_fw_ver;
	
	uint8_t bootloader_id[2];

	uint8_t temp_array[2], temp_data, flashValue, m_firmwareImgVersion;
	uint8_t checkSumCode;

	uint16_t ts_block_size, ts_config_block_count, ts_fw_block_count;
	uint16_t m_bootloadImgID;
	
	uint32_t ts_config_img_size;
	uint32_t ts_fw_img_size;
	uint32_t pinValue, m_fileSize, m_firmwareImgSize, m_configImgSize, m_FirmwareImgFile_checkSum;

	////////////////////////////

	DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware [START]\n");
/*
	if(!(synaptics_ts_check_fwver(client) < SynapticsFirmware[0x1F]))
	{
		// Firmware Upgrade does not necessary!!!!
		DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
		return true;
	}
*/
#if SYNAPTICS_TOUCH_MULTI_PANEL_SUPPORT
	if (ts->product_value==1)
	{
		memcpy(SynapticsFirmware, SynapticsFirmware_misung, sizeof(SynapticsFirmware_misung));
		current_fw_ver = synaptics_ts_check_fwver(client);
		if((current_fw_ver >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) || (current_fw_ver < 0x64 && SynapticsFirmware[0x1F] < 0x64))
		{
			if(!(current_fw_ver < SynapticsFirmware[0x1F]))
			{
				// Firmware Upgrade does not necessary!!!!
				DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
				return true;
			}
		}
	}
	else if (ts->product_value==2)
	{
			memcpy(SynapticsFirmware, SynapticsFirmware_lgit, sizeof(SynapticsFirmware_lgit));
			current_fw_ver = synaptics_ts_check_fwver(client);
			if((current_fw_ver >= 0x01 && SynapticsFirmware[0x1F] >= 0x01) || (current_fw_ver < 0x01 && SynapticsFirmware[0x1F] < 0x01))
			{
				if(!(current_fw_ver < SynapticsFirmware[0x1F]))
				{
					// Firmware Upgrade does not necessary!!!!
					DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
					return true;
				}
			}
	}
	else
		return true;
#else
		current_fw_ver = synaptics_ts_check_fwver(client);

		DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware CurrentVersion[%d] SynapticsFirmware[%d]\n", current_fw_ver, SynapticsFirmware[0x1F]);
		
		//REV_C
		if(current_fw_ver == 0x01 && SynapticsFirmware[0x1F] == 0x65)
		{
			if((current_fw_ver >= 0x64 && SynapticsFirmware[0x1F] >= 0x64) || (current_fw_ver < 0x64 && SynapticsFirmware[0x1F] < 0x64))
			{
				if(!(current_fw_ver < SynapticsFirmware[0x1F]))
				{
					// Firmware Upgrade does not necessary!!!!
					DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
					return true;
				}
			}
		}
		else
		{
			// Firmware Upgrade does not necessary!!!!
			DEBUG_MSG("[Touch Driver] Synaptics_UpgradeFirmware does not necessary!!!!\n");
			return true;
		}
		
#endif

	// Address Configuration
	FlashQueryBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_QUERY_BASE_REG);

	BootloaderIDAddr = FlashQueryBaseAddr;
	BlockSizeAddr = FlashQueryBaseAddr + 3;
	FirmwareBlockCountAddr = FlashQueryBaseAddr + 5;
	ConfigBlockCountAddr = FlashQueryBaseAddr + 7;
	

	FlashDataBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_DATA_BASE_REG);

	BlockNumAddr = FlashDataBaseAddr;
	BlockDataStartAddr = FlashDataBaseAddr + 2;

	// Get New Firmware Information from Header
	m_fileSize = sizeof(SynapticsFirmware) -1;

	checkSumCode         = ExtractLongFromHeader(&(SynapticsFirmware[0]));
	m_bootloadImgID      = (unsigned int)SynapticsFirmware[4] + (unsigned int)SynapticsFirmware[5]*0x100;
	m_firmwareImgVersion = SynapticsFirmware[7]; 
	m_firmwareImgSize    = ExtractLongFromHeader(&(SynapticsFirmware[8]));
	m_configImgSize      = ExtractLongFromHeader(&(SynapticsFirmware[12]));    

	CalculateChecksum((uint16_t*)&(SynapticsFirmware[4]), (uint16_t)(m_fileSize-4)>>1, &m_FirmwareImgFile_checkSum);

	// Get Current Firmware Information
	i2c_smbus_read_i2c_block_data(client, BlockSizeAddr, sizeof(temp_array), (u8 *)&temp_array[0]);
	ts_block_size = temp_array[0] + (temp_array[1] << 8);
	
	i2c_smbus_read_i2c_block_data(client, FirmwareBlockCountAddr, sizeof(temp_array), (u8 *)&temp_array[0]);
	ts_fw_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_fw_img_size = ts_block_size * ts_fw_block_count;
	
	i2c_smbus_read_i2c_block_data(client, ConfigBlockCountAddr, sizeof(temp_array), (u8 *)&temp_array[0]);
	ts_config_block_count = temp_array[0] + (temp_array[1] << 8);
	ts_config_img_size = ts_block_size * ts_config_block_count;

	i2c_smbus_read_i2c_block_data(client, BootloaderIDAddr, sizeof(bootloader_id), (u8 *)&bootloader_id[0]);
	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: BootloaderID %02x %02x\n", bootloader_id[0], bootloader_id[1]);

	// Compare
	if (m_fileSize != (0x100+m_firmwareImgSize+m_configImgSize))
	{
		DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid FileSize\n");
		return true;
	}

	if (m_firmwareImgSize != ts_fw_img_size)
	{
		DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Firmware Image Size\n");
		return true;
	}

	if (m_configImgSize != ts_config_img_size)
	{
		DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Config Image Size\n");
		return true;
	}

	// Flash Write Ready - Flash Command Enable & Erase
	//i2c_smbus_write_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), &bootloader_id[0]);
	// How can i use 'i2c_smbus_write_block_data'
	for(i = 0; i < sizeof(bootloader_id); i++)
	{
		if(i2c_smbus_write_byte_data(client, BlockDataStartAddr+i, bootloader_id[i]))
			DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Address %02x, Value %02x\n", BlockDataStartAddr+i, bootloader_id[i]);
	}

	do
	{
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
	} while((flashValue & 0x0f) != 0x00);

	i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ENABLE);

	do
	{
		pinValue = gpio_get_value(TOUCH_INT_N_GPIO);
		mdelay(1);
	} while(pinValue);
	do
	{
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
	} while(flashValue != 0x80);
	flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);

	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Flash Program Enable Setup Complete\n");

	//i2c_smbus_write_block_data(client, BlockDataStartAddr, sizeof(bootloader_id), &bootloader_id[0]);
	// How can i use 'i2c_smbus_write_block_data'
	for(i = 0; i < sizeof(bootloader_id); i++)
	{
		if(i2c_smbus_write_byte_data(client, BlockDataStartAddr+i, bootloader_id[i]))
			DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Address %02x, Value %02x\n", BlockDataStartAddr+i, bootloader_id[i]);
	}

	if(m_firmwareImgVersion == 0 && ((unsigned int)bootloader_id[0] + (unsigned int)bootloader_id[1]*0x100) != m_bootloadImgID)
	{
		DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Error : Invalid Bootload Image\n");
		return true;
	}

	i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_ERASEALL);

	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: SYNAPTICS_FLASH_CMD_ERASEALL\n");

	do
	{
		pinValue = gpio_get_value(TOUCH_INT_N_GPIO);
		mdelay(1);
	} while(pinValue);
	do
	{
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
	} while(flashValue != 0x80);
	
	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Flash Erase Complete\n");

	// Flash Firmware Data Write
	for(i = 0; i < ts_fw_block_count; ++i)
	{
		temp_array[0] = i & 0xff;
		temp_array[1] = (i & 0xff00) >> 8;

		// Write Block Number
		//i2c_smbus_write_block_data(client, BlockNumAddr, sizeof(temp_array), &temp_array[0]);
		// How can i use 'i2c_smbus_write_block_data'
		for(j = 0; j < sizeof(temp_array); j++)
		{
			i2c_smbus_write_byte_data(client, BlockNumAddr+j, temp_array[j]);
		}

		// Write Data Block&SynapticsFirmware[0]
		//i2c_smbus_write_block_data(client, BlockDataStartAddr, ts_block_size, &SynapticsFirmware[0x100+i*ts_block_size]);
		// How can i use 'i2c_smbus_write_block_data'
		for(j = 0; j < ts_block_size; j++)
		{
			i2c_smbus_write_byte_data(client, BlockDataStartAddr+j, SynapticsFirmware[0x100+i*ts_block_size+j]);
		}

		// Issue Write Firmware Block command
		i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_FW_WRITE);
		do
		{
			pinValue = gpio_get_value(TOUCH_INT_N_GPIO);
			mdelay(1);
		} while(pinValue);
		do
		{
			flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
			temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
		} while(flashValue != 0x80);
	} //for
	
	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Flash Firmware Write Complete\n");

	// Flash Firmware Config Write
	for(i = 0; i < ts_config_block_count; i++)
	{
		SpecialCopyEndianAgnostic(&temp_array[0], i);

		// Write Configuration Block Number
		i2c_smbus_write_block_data(client, BlockNumAddr, sizeof(temp_array), &temp_array[0]);
		// How can i use 'i2c_smbus_write_block_data'
		for(j = 0; j < sizeof(temp_array); j++)
		{
			i2c_smbus_write_byte_data(client, BlockNumAddr+j, temp_array[j]);
		}

		// Write Data Block
		//i2c_smbus_write_block_data(client, BlockDataStartAddr, ts_block_size, &SynapticsFirmware[0x100+m_firmwareImgSize+i*ts_block_size]);
		// How can i use 'i2c_smbus_write_block_data'
		for(j = 0; j < ts_block_size; j++)
		{
			i2c_smbus_write_byte_data(client, BlockDataStartAddr+j, SynapticsFirmware[0x100+m_firmwareImgSize+i*ts_block_size+j]);
		}

		// Issue Write Configuration Block command to flash command register
		i2c_smbus_write_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG, SYNAPTICS_FLASH_CMD_CONFIG_WRITE);
		do
		{
			pinValue = gpio_get_value(TOUCH_INT_N_GPIO);
			mdelay(1);
		} while(pinValue);
		do
		{
			flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
			temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
		} while(flashValue != 0x80);
	}
	
	DEBUG_MSG("[TOUCH] Synaptics_UpgradeFirmware :: Flash Config Write Complete\n");


	RMICommandBaseAddr = i2c_smbus_read_byte_data(client, SYNAPTICS_RMI_CMD_BASE_REG);
	DEBUG_MSG("[TOUCH] RMICommandBaseAddr :: %x\n", RMICommandBaseAddr);
	i2c_smbus_write_byte_data(client, RMICommandBaseAddr, 0x01);
	mdelay(100);

	do
	{
		pinValue = gpio_get_value(TOUCH_INT_N_GPIO);
		mdelay(1);
	} while(pinValue);
	
	DEBUG_MSG("[TOUCH] pinValue :: %d\n", pinValue);
	
	do
	{
		flashValue = i2c_smbus_read_byte_data(client, SYNAPTICS_FLASH_CONTROL_REG);
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
	} while((flashValue & 0x0f) != 0x00);
	DEBUG_MSG("[TOUCH] SYNAPTICS_FLASH_CONTROL_REG :: %x  SYNAPTICS_INT_STATUS_REG %x\n", flashValue, temp_data);
	// Clear the attention assertion by reading the interrupt status register
	temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_INT_STATUS_REG);
	DEBUG_MSG("[TOUCH] SYNAPTICS_INT_STATUS_REG %x\n", temp_data);
	// Read F01 Status flash prog, ensure the 6th bit is '0'
	do
	{
		temp_data = i2c_smbus_read_byte_data(client, SYNAPTICS_DATA_BASE_REG);
	} while((temp_data & 0x40) != 0);
	DEBUG_MSG("[TOUCH] SYNAPTICS_DATA_BASE_REG %x\n", temp_data);
	return true;
}


#endif




/*************************************************************************************************
 * 1. Set interrupt configuration
 * 2. Disable interrupt
 * 3. Power up
 * 4. Read RMI Version
 * 5. Read Firmware version & Upgrade firmware automatically
 * 6. Read Data To Initialization Touch IC
 * 7. Set some register
 * 8. Enable interrupt
*************************************************************************************************/
static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0,r, i;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	unsigned long irqflags;
	
	DEBUG_MSG("%s() -- start\n\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DEBUG_MSG(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	memset(ts, 0x00, sizeof(struct synaptics_ts_data));

	ts->client = client;
	i2c_set_clientdata(client, ts);

	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_DELAYED_WORK(&ts->init_delayed_work, synaptics_ts_init_delayed_work);
#ifdef LGHDK_LONGPRESS_TOUCH_DURING_BOOTING	
	INIT_DELAYED_WORK(&ts->init_delayed_no_melt_work, synaptics_ts_init_delayed_No_Melt_work);
#endif
	INIT_DELAYED_WORK(&ts->touchkey_delayed_work, synaptics_ts_touchkey_delayed_work);

#ifdef LGHDK_USED_RESET_PIN_IN_SUSPEND
	INIT_DELAYED_WORK(&ts->reset_delayed_work, synaptics_ts_reset_delayed_work);
#endif


#ifdef LGHDK_PENDING_TOUCHKEY
	//init pending value
	ts->pending_touchkey = LGHDK_PENDING_IDLE_STATE;
#endif

	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			DEBUG_MSG(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	if (pdata) {
		ts->flags = pdata->flags;
		irqflags = pdata->irqflags;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		irqflags = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}

  	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
  	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));

	#ifdef LGHDK_SYNAPTICS_SUPPORT_FW_UPGRADE
	synaptics_ts_fw_upgrade(ts->client);
	#endif
	touch_fw_version = synaptics_ts_check_fwver(ts->client);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		DEBUG_MSG(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "heaven_synaptics_touch";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
    	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
    	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	for (i = 0; omap_virtualkeymap[i] != 0; i++)
	{
		set_bit(omap_virtualkeymap[i] , ts->input_dev->keybit);
	}
		

   	set_bit(EV_TG, ts->input_dev->evbit);
    	
	fuzz_x = fuzz_x / 0x10000;
	fuzz_y = fuzz_y / 0x10000;

	DEBUG_MSG(KERN_ERR "synaptics_ts_probe: Unable to fuzz_x [%d] fuzz_y[%d]\n", fuzz_x, fuzz_y);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_W, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_H, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, SYN_TG_REPORT, 0, 3, 0, 0);
	input_set_abs_params(ts->input_dev, TG_DIR, 0, 3, 0, 0);
	input_set_abs_params(ts->input_dev, TG_SPEED, 0, 1681, 0, 0);
	ret = input_register_device(ts->input_dev);
	if (ret) {
		DEBUG_MSG(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	r = gpio_request(client->irq, "Synaptics gpio interrupt");
	if (r < 0){
		dev_dbg(&client->dev,"unable to gent INT GPIO");
		r=-ENODEV;
		goto err_alloc_data_failed;
	}
	gpio_direction_input(client->irq);
	
	
	if (client->irq) {
		ret = request_irq(gpio_to_irq(client->irq), synaptics_ts_irq_handler, irqflags, client->name, ts);
		if (ret == 0) {
			ts->use_irq = 1;
			DEBUG_MSG("request_irq\n");
			}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	ts->use_irq = 1;

	ret = device_create_file(&client->dev, &dev_attr_version);

	// 20101215 seven.kim@lge.com grip suppression [START]
#ifdef LGHDK_TOUCH_GRIP_SUPPRESSION
		ret = device_create_file(&client->dev, &dev_attr_gripsuppression);
		if (ret) {
			dev_err(&client->dev, "synaptics_ts_probe: grip suppression device_create_file failed\n");
			goto err_check_functionality_failed;
		}
#endif /* LGHDK_TOUCH_GRIP_SUPPRESSION */

#ifdef LGHDK_TOUCH_HAND_SUPPRESSION
				ret = device_create_file(&client->dev, &dev_attr_handsuppression);
				if (ret) {
					dev_err(&client->dev, "synaptics_ts_probe: hand suppression device_create_file failed\n");
					goto err_check_functionality_failed;
				}
#endif /* LGHDK_TOUCH_HAND_SUPPRESSION */


		schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(500));	

#ifdef LGHDK_LONGPRESS_TOUCH_DURING_BOOTING
		schedule_delayed_work(&ts->init_delayed_no_melt_work, msecs_to_jiffies(5000));	
#endif

		synaptics_init_report_mode(ts);

		ts->report_mode = LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE;	
		
		ts->melt_toggle_state = LGHDK_MELT_IN_MELTMODE;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = synaptics_ts_early_suspend;
		ts->early_suspend.resume = synaptics_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif


	DEBUG_MSG(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

#ifdef LGHDK_TOUCH_GRIP_SUPPRESSION	//20101216 seven.kim@lge.com [start]
	device_remove_file(&client->dev, &dev_attr_gripsuppression);
#endif //20101216 seven.kim@lge.com [end]
#ifdef LGHDK_TOUCH_HAND_SUPPRESSION
		device_remove_file(&client->dev, &dev_attr_handsuppression);
#endif

	unregister_early_suspend(&ts->early_suspend);

	free_irq(gpio_to_irq(client->irq), ts);

	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	disable_irq(gpio_to_irq(client->irq));

	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) {
		printk(KERN_ERR "TouchScreen-tm709 Suspend Failed\n");
		enable_irq(gpio_to_irq(client->irq));
		return -EBUSY;
	}
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_SLEEP); /* sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");

#if 0
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			DEBUG_MSG(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
#endif

#ifdef LGHDK_PENDING_TOUCHKEY
	//init pending value
	ts->pending_touchkey = LGHDK_PENDING_IDLE_STATE;
	ret = cancel_delayed_work_sync(&ts->touchkey_delayed_work);
#endif

	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret, gpiovalue;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	gpiovalue = gpio_get_value(TOUCH_INT_N_GPIO);

	printk("%s() - 3 [%x][%d]\n", __func__, ts, gpiovalue);

#if 0
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			DEBUG_MSG(KERN_ERR "synaptics_ts_resume power on failed\n");
	}
#endif

#ifdef LGHDK_USED_RESET_PIN_IN_SUSPEND
	if(gpiovalue == 0)
	{
		ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_RESET_REG, 0x01); //Reset
		schedule_delayed_work(&ts->reset_delayed_work, msecs_to_jiffies(400));	
	}
	else
	{
		schedule_delayed_work(&ts->reset_delayed_work, msecs_to_jiffies(100));	
	}

#else
	enable_irq(gpio_to_irq(client->irq));

	schedule_delayed_work(&ts->init_delayed_work, msecs_to_jiffies(200));

	synaptics_init_report_mode(ts);

	ts->report_mode = LGHDK_SYNAPTICS_VAL_NORMAL_REPORT_MODE;
	
	ts->melt_toggle_state = LGHDK_MELT_IN_MELTMODE;

#endif
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "heaven_synaptics_ts", 0 },
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "heaven_synaptics_ts",
		.owner = THIS_MODULE,
	},
};

static int __devinit synaptics_ts_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
    	DEBUG_MSG ("LGE: Synaptics ts_init\n");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
    
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");


