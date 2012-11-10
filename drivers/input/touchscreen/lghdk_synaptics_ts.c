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
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <mach/gpio.h>
#include <linux/slab.h>

//#include <linux/i2c/twl6030.h>

#define SYNAPTICS_TOUCH_DEBUG 1
 #if SYNAPTICS_TOUCH_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif
 
#define HEAVEN_TS_EN_GPIO 		52
#define HEAVEN_TS_POLLING_TIME 	15 /* polling time(msec) when touch was pressed */ 
#define SYNAPTICS_INT_REG		0x50
#define SYNAPTICS_INT_FLASH		1<<0
#define SYNAPTICS_INT_STATUS 	1<<1
#define SYNAPTICS_INT_ABS0 		1<<2
#define SYNAPTICS_INT_BUTTON	1<<3

#if 0
#define SYNAPTICS_CONTROL_REG		0x4F
#else	/* JamesLee :: for 4.3inch touch */
#define SYNAPTICS_CONTROL_REG		0x21
#endif
#define SYNAPTICS_CONTROL_SLEEP 	1<<0
#define SYNAPTICS_CONTROL_NOSLEEP	1<<2


static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	int reported_finger_count;
	int8_t sensitivity_adjust;
	int (*power)(int on);
// 20100504 jh.koo@lge.com, correction of finger space [START_LGE]
	unsigned int count;
	int x_lastpt;
	int y_lastpt;
// 20100504 jh.koo@lge.com, correction of finger space [END_LGE]	
	struct early_suspend early_suspend;
	uint32_t	button_state;

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
#define TS_SNTS_GET_FINGER_STATE_0(finger_status_reg) \
		(finger_status_reg&0x03)
#define TS_SNTS_GET_FINGER_STATE_1(finger_status_reg) \
		((finger_status_reg&0x0C)>>2)
#define TS_SNTS_GET_FINGER_STATE_2(finger_status_reg) \
		((finger_status_reg&0x30)>>4)
#define TS_SNTS_GET_FINGER_STATE_3(finger_status_reg) \
      ((finger_status_reg&0xC0)>>6)
#define TS_SNTS_GET_FINGER_STATE_4(finger_status_reg) \
      (finger_status_reg&0x03)

#define TS_SNTS_GET_X_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)(low_reg&0x0F))
#define TS_SNTS_GET_Y_POSITION(high_reg, low_reg) \
		((int)(high_reg*0x10) + (int)((low_reg&0xF0)/0x10))

#define TS_SNTS_HAS_PINCH(gesture_reg) \
		((gesture_reg&0x40)>>6)
#define TS_SNTS_HAS_FLICK(gesture_reg) \
		((gesture_reg&0x10)>>4)
#define TS_SNTS_HAS_DOUBLE_TAP(gesture_reg) \
		((gesture_reg&0x04)>>2)

#define TS_SNTS_GET_REPORT_RATE(device_control_reg) \
		((device_control_reg&0x40)>>6)
// 1st bit : '0' - Allow sleep mode, '1' - Full power without sleeping
// 2nd and 3rd bit : 0x00 - Normal Operation, 0x01 - Sensor Sleep
#define TS_SNTS_GET_SLEEP_MODE(device_control_reg) \
		(device_control_reg&0x07)

#define BLUE_LED_GPIO (OMAP_MAX_GPIO_LINES + TWL4030_GPIO_MAX)

/*                         CONSTANTS DATA DEFINITIONS                      */
#define FINGER_MAX 10 //최대 5개 손가락 사용  

#define TS_DEBUG
#define TS_DEBUG_BUFFER_SIZE		1024

/*
#ifndef TS_USES_IC_GESTURE
#define FLICK_TIME_THRESHOLD		22		// 8
#define FLICK_DISTANCE_THRESHOLD	30		// 15
#endif//TS_USES_IC_GESTURE 
//#define DEFAULT_POLLING_PERIOD		60
//#define HOLD_THRESHOLD				17		// 20		// 15
//#define LONG_THRESHOLD				1000//40   //25		// 10
*/
#define TS_SNTS_X_AXIS_MAX_TM1343_002 1441
#define TS_SNTS_Y_AXIS_MAX_TM1343_002 840
#define FLICK_RANGE 4
#define LONG_PRESS_RANGE 6
#define TAP_TH      10
#define DISTANCE_TH    15
#define SPREADING_TH   20
#define PINCHING_TH    20 

//1497

//------------------------------------------------------------------------------------------------
// Register setting value
//------------------------------------------------------------------------------------------------

  /* Register map header file for TM1343, Family 0x01, Revision 0x01 */
  /* Automatically generated on 2009-Aug-10  10:45:05, do not edit */

#ifndef SYNA_REGISTER_MAP_H
#define SYNA_REGISTER_MAP_H 1

/*      Register Name                                                                    Address     Register Description */
/*      -------------                                                                 -------     -------------------- */

#define SYNA_F01_RMI_QUERY03                               0x00B3   /* Firmware Revision Query */

//#define SYNA_F11_2D_QUERY00                                0x008E   /* Per-device Query */

/* Start of Page Description Table (PDT) */
#define SYNA_PDT_P00_F11_2D_QUERY_BASE                     0x00DD   /* Query Base */
#define SYNA_PDT_P00_F01_RMI_QUERY_BASE                    0x00E3   /* Query Base */
#define SYNA_PDT_P00_F34_FLASH_QUERY_BASE                  0x00E9   /* Query Base */
#define SYNA_P00_PAGESELECT                                0x00FF   /* Page Select register */

/* Offsets within the configuration block */

/*      Register Name                                                                     Offset      Register Description */
/*      -------------                                                                   ------      -------------------- */
#define SYNA_F01_RMI_DEVICE_00_CFGBLK_OFS                  0x0000   /* Device status & Control */
#define SYNA_F01_RMI_INTERRUPT_01_CFGBLK_OFS               0x0001   /* Interrupt status & Enable 0 */
#define SYNA_F11_RMI_CTRL00_CFGBLK_OFS                     0x0000   /* 2D Reprot Mode */

/* Masks for interrupt sources */
#endif  /* SYNA_REGISTER_MAP_H */

#define TS_SNTS_RMI_CTRL00_DEFAULT_VALUE	0x00	// No configured, 80 Hz, Sleep Enable, Normal operation
#define TS_SNTS_RMI_CTRL00_NO_SLEEP			0x48	// No configured, 40 Hz, Sleep Disable, Normal operation
#define TS_SNTS_RMI_CTRL00_RESET_VALUE		0x00	// No configured, 80 Hz, Sleep Enable, Normal operation

#define TS_SNTS_RMI_CTRL01_DEFAULT_VALUE	0x06	// Abs0 Enable, Status Enable, Flash Disable
#define TS_SNTS_RMI_CTRL01_ONLY_ABS0		0x04	// Abs0 Enable, Status Disable, Flash Disable
#define TS_SNTS_RMI_CTRL01_ONLY_FLASH		0x01	// Abs0 Disable, Status Disable, Flash Enable
#define TS_SNTS_RMI_CTRL01_RESET_VALUE		0x07	// Abs0 Enable, Status Enable, Flash Enable

#define TS_SNTS_2D_CTRL00_DEFAULT_VALUE		0x02	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Down/Up Reporting Mode
#define TS_SNTS_2D_CTRL00_REDUCED_REPORTING	0x01	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Reduced Reporting Mode
#define TS_SNTS_2D_CTRL00_FULL_REPORTING	0x00	// Relative Ballistic Disable, Relative Postion Filter Disable, Absolute Position Filter Disable, Continuous Reporting Mode

//#define TS_SNTS_2D_CTRL10_DEFAULT_VALUE		0x54	// Pinch Enable, Flick Enable, Double Tap Enable
//#define TS_SNTS_2D_CTRL10_RESET_VALUE		0x7F	// All Enable

//#define TS_SNTS_2D_CTRL18_VALUE				0x03	// Minimum Flick Distance (3 * (1 mm))
//#define TS_SNTS_2D_CTRL19_VALUE				0x04	// Minimum Flick Speed (4 * (1 mm / 100 ms))

#define TS_SNTS_2D_CTRL14_VALUE				0x00	// Sensitivity Adjust

///TODO: Must Check this.
//#ifdef TS_I2C_FAIL_MANAGEMENT
#define TS_SNTS_DELAY_BETWEEN_POWER_DOWN_UP 		100	// 100ms	// For T1021
//#endif
#define TS_SNTS_DELAY_TO_LOW_ATTN_FOR_CLEARPAD		500	// 500 ms	// For T1021

//#define TS_SNTS_DEVICE_COMMAND_RESET				1	// Reset
//#define TS_SNTS_FW_VER_RETRY_CNT					10

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                         DATA DEFINITIONS                                */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

typedef struct {
	unsigned char m_QueryBase;
	unsigned char m_CommandBase;
	unsigned char m_ControlBase;
	unsigned char m_DataBase;
	unsigned char m_IntSourceCount;
	unsigned char m_FunctionExists;
} T_RMI4FuncDescriptor;

/* Ku */
typedef int TS_FINGER_STATUS_TYPE;
#define TS_SNTS_NO_PRESENT  0x0
#define TS_SNTS_PRESENT_ACCURATE  0x1


#ifdef TS_VERSION_MANAGEMENT
typedef enum {
	E_TS_RMI4_PAGE_SELECT_00 = 0x00,
} E_TS_RMI_PAGE_SELECT;

typedef enum {
	TS_SNTS_VER_MIN,
	TS_SNTS_VER_MAX
} TS_VERSION_INFO;
#endif /* TS_VERSION_MANAGEMENT */

#ifdef TS_DEBUG
typedef struct {
	int X[FINGER_MAX];
	int Y[FINGER_MAX];
	unsigned char Z[FINGER_MAX];
	unsigned char W[FINGER_MAX];
	//TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	/* Ku */
	//TOUCH_EVENT_TYPE event[FINGER_MAX];
  	// For T1021
  	/*
	unsigned char gesture_flag[2];
	unsigned char pinch_motion_and_X_flick_dist;
	unsigned char Y_flick_dist;
	unsigned char flick_time_reg;
	*/
} touch_event_debugging_data;
#endif /* TS_DEBUG */

//added by jykim
#define START_ADDR      0x13
#define GESTURE_FLAGS	0x4A

#if 0
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
 	// Finger 2
	unsigned char X_high_position_finger2_reg;  //0x22
	unsigned char Y_high_position_finger2_reg;	//0x23
	unsigned char XY_low_position_finger2_reg;	//0x24
	unsigned char XY_width_finger2_reg;			//0x25
	unsigned char Z_finger2_reg;				//0x26
#if 0
	// Finger 3
	unsigned char X_high_position_finger3_reg;  //0x27
	unsigned char Y_high_position_finger3_reg;	//0x28
	unsigned char XY_low_position_finger3_reg;	//0x29
	unsigned char XY_width_finger3_reg;			//0x2A
	unsigned char Z_finger3_reg;				//0x2B
 	// Finger 4
	unsigned char X_high_position_finger4_reg;  //0x2C
	unsigned char Y_high_position_finger4_reg;	//0x2D
	unsigned char XY_low_position_finger4_reg;	//0x2E
	unsigned char XY_width_finger4_reg;			//0x2F
	unsigned char Z_finger4_reg;				//0x30
 	// Finger 5
	unsigned char X_high_position_finger5_reg;  //0x31
	unsigned char Y_high_position_finger5_reg;	//0x32
	unsigned char XY_low_position_finger5_reg;	//0x33
	unsigned char XY_width_finger5_reg;			//0x34
	unsigned char Z_finger5_reg;				//0x35
 	// Finger 6
	unsigned char X_high_position_finger6_reg;  //0x36
	unsigned char Y_high_position_finger6_reg;	//0x37
	unsigned char XY_low_position_finger6_reg;	//0x38
	unsigned char XY_width_finger6_reg;			//0x39
	unsigned char Z_finger6_reg;				//0x3A
 	// Finger 7
	unsigned char X_high_position_finger7_reg;  //0x3B
	unsigned char Y_high_position_finger7_reg;	//C
	unsigned char XY_low_position_finger7_reg;	//D
	unsigned char XY_width_finger7_reg;			//E
	unsigned char Z_finger7_reg;				//F
 	// Finger 8
	unsigned char X_high_position_finger8_reg;  //0x40
	unsigned char Y_high_position_finger8_reg;	//
	unsigned char XY_low_position_finger8_reg;	//
	unsigned char XY_width_finger8_reg;			//
	unsigned char Z_finger8_reg;				//0x44
 	// Finger 9
	unsigned char X_high_position_finger9_reg;  //0x45
	unsigned char Y_high_position_finger9_reg;	//
	unsigned char XY_low_position_finger9_reg;	//
	unsigned char XY_width_finger9_reg;			//
	unsigned char Z_finger9_reg;				//0x49

	
	unsigned char gesture_flag0;     	       //0x4A
	unsigned char gesture_flags_1;			   //0x4B
	unsigned char pinch_motion_X_flick_distance;	//04C
	unsigned char rotation_motion_Y_flick_distance;  //0x4D
	unsigned char finger_separation_flick_time;		//0x4E
//	unsigned char button_data;						//0x4F
	unsigned char device_control;			//0x4F
	//unsigned char firmware_ver_reg;
#endif
} ts_sensor_data;
#else	/* JamesLee :: for 4.3 inch touch */
typedef struct
{
	unsigned char device_status_reg;            //0x13
	unsigned char interrupt_status_reg;			//0x14
	unsigned char finger_state_reg;				//0x15
	// Finger 0
	unsigned char X_high_position_finger0_reg;  //0x16
	unsigned char Y_high_position_finger0_reg;	//0x17
	unsigned char XY_low_position_finger0_reg;	//0x18
	unsigned char XY_width_finger0_reg;			//0x19
	unsigned char Z_finger0_reg;				//0x1A
	// Finger 1
	unsigned char X_high_position_finger1_reg;  //0x1B
	unsigned char Y_high_position_finger1_reg;	//0x1C
	unsigned char XY_low_position_finger1_reg;	//0x1D
	unsigned char XY_width_finger1_reg;			//0x1E
	unsigned char Z_finger1_reg;				//0x1F

	unsigned char button_data;					// 0x20
	unsigned char device_control;				// 0x21
	unsigned char interrupt_enable0;			// 0x22
	unsigned char report_mode;					// 0x23
	unsigned char palm_detect;					// 0x24
	unsigned char delta_x_thresh;				// 0x25
	unsigned char delta_y_thresh;				// 0x26
	unsigned char velocity;						// 0x27
	unsigned char acceleration;					// 0x28
	unsigned char max_x_position_7_0;			// 0x29
	unsigned char max_x_position_11_8;			// 0x2A
	unsigned char max_y_position_7_0;			// 0x2B
	unsigned char max_y_position_11_8;			// 0x2C
} ts_sensor_data;
#endif

typedef struct
{
	unsigned char gesture_flag0;     	       //0x4A
	unsigned char gesture_flags_1;			   //0x4B
	unsigned char pinch_motion_X_flick_distance;	//04C
	unsigned char rotation_motion_Y_flick_distance;  //0x4D
	unsigned char finger_separation_flick_time;		//0x4E
	unsigned char device_control;			//0x4F
} ts_gesture_data;


typedef struct {
	unsigned char finger_count;
//  TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	int X_position[FINGER_MAX];
	int Y_position[FINGER_MAX];
//  TOUCH_EVENT_TYPE Event[FINGER_MAX];
} ts_finger_data;

static ts_sensor_data ts_reg_data={0};
static ts_gesture_data ts_gesture_reg = {0};
static ts_finger_data curr_ts_data;

typedef struct {
	//TS_FINGER_STATUS_TYPE finger_status[FINGER_MAX];
	unsigned int action[FINGER_MAX];
  	unsigned int finger_data[FINGER_MAX];
} TOUCH_MULTIFINGER_DATA_TYPE;

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/*                                                                         */
/*                            External Data                                */
/*                                                                         */
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
/* Ku */
// extern rex_crit_sect_type ts_crit_sect;
// extern rex_timer_type ts_poll_timer;
//extern clk_cb_type ts_polling_clk_cb;

#ifdef TS_HW_VER_MANAGEMENT
#ifndef FEATURE_LGE_I2C_USING_GPIO
// extern unsigned int nTS_INT_OUT;
// extern unsigned int nTS_INT_IN;
// extern unsigned int nTS_INT;
#endif
#endif /* TS_HW_VER_MANAGEMENT */

/*                            Local Data                                   */
#ifdef TS_DEBUG
touch_event_debugging_data touch_event_log[TS_DEBUG_BUFFER_SIZE];
unsigned char touch_event_cnt=0;

touch_event_debugging_data touch_report_log[TS_DEBUG_BUFFER_SIZE];
unsigned char touch_report_cnt=0;

#endif /* TS_DEBUG */

#ifdef TS_I2C_FAIL_MANAGEMENT
static unsigned int nI2CFailCnt = 0;
#endif /* TS_I2C_FAIL_MANAGEMENT */
static int EX_BTN = 0;

typedef struct {
	int X_position[LONG_PRESS_RANGE];
	int Y_position[LONG_PRESS_RANGE];
	int X2_position[LONG_PRESS_RANGE];
	int Y2_position[LONG_PRESS_RANGE];
    int distance[LONG_PRESS_RANGE];
} finger_history;

static finger_history synaptics_history;

#ifdef FEATURE_LGE_DSAT_TOUCHSCREEN_EMULATION
/* Event callback function */
typedef void (*hs_touch_event_cb_type) (void * cmd);
hs_touch_event_cb_type cb_ptr = NULL;
static int ts_cmer_tscrn = 0;
static int ts_cmec_tscrn = 0;
//static BOOL ts_touchable_reporting_to_ds = FALSE;
static BOOL ts_cind_inputstatus_in_sleep = FALSE;
#endif // FEATURE_LGE_DSAT_TOUCHSCREEN_EMULATION


//ended by jykim

static int ts_pre_state = 0; /* for checking the touch state */

static int longpress_pre = 0;
static int flicking = 0;

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int int_mode;
	int i;
	int width0, width1, width2, width3;
	//int_mode = i2c_smbus_read_byte_data(ts->client, 0x14);
	int_mode ;
	
	int i2 = 0;
	int touch2_prestate = 0;
	int touch1_prestate = 0;
    int longpress;

	int x, y;
//	printk("#####################################################################################ts_work_fn\n");
	int_mode = i2c_smbus_read_byte_data(ts->client, 0x14);

	if(int_mode & SYNAPTICS_INT_ABS0)
	/*while (1)*/ {

		i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data);
		i2c_smbus_read_i2c_block_data(ts->client, GESTURE_FLAGS, sizeof(ts_gesture_reg), &ts_gesture_reg);

#if 0		/* JamesLee */
		printk("[JamesLee] X_high=%d, Y_high=%d, XY_low=%d\n", ts_reg_data.X_high_position_finger0_reg, 
					ts_reg_data.Y_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
#endif
#if 0
		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg[0]) == 1 ) //&& ((ts_reg_data.XY_width_finger0_reg & 240) >> 4) > 0 && (ts_reg_data.XY_width_finger0_reg & 15)> 0)
#else	/* JamesLee :: for 4.3inch touch */
		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg) == 1 ) //&& ((ts_reg_data.XY_width_finger0_reg & 240) >> 4) > 0 && (ts_reg_data.XY_width_finger0_reg & 15)> 0)
#endif
		{
			touch1_prestate = 1;
#if 0
			curr_ts_data.X_position[0] = 986 - (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
#else	/* JamesLee */
			curr_ts_data.X_position[0] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
#endif
  			curr_ts_data.Y_position[0] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger0_reg, ts_reg_data.XY_low_position_finger0_reg);
			synaptics_history.X_position[i2] = curr_ts_data.X_position[0];
			synaptics_history.Y_position[i2] = curr_ts_data.Y_position[0];
			
			if ((((ts_reg_data.XY_width_finger0_reg & 240) >> 4) - (ts_reg_data.XY_width_finger0_reg & 15)) > 0)
				width0 = (ts_reg_data.XY_width_finger0_reg & 240) >> 4;
			else
				width0 = ts_reg_data.XY_width_finger0_reg & 15;
			
        	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, width0);
       		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[0]);
        	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[0]);

#if 0		/* JamesLee */
			printk("Synaptics_X, Synaptics_Y = (%d, %d), z = %d, w = %d, %d\n", curr_ts_data.X_position[0], curr_ts_data.Y_position[0], ((ts_reg_data.XY_width_finger0_reg & 240) >> 4), (ts_reg_data.XY_width_finger0_reg & 15), width0);
#endif
		
			input_mt_sync(ts->input_dev);		
			
		}else{
				touch1_prestate = 0;
				i2 = 0;
		}
			
		if(TS_SNTS_GET_FINGER_STATE_0(ts_reg_data.finger_state_reg) == 0 && touch1_prestate == 1) {
			printk(" FINGER_RELEASED\n");
		}
		
		if(TS_SNTS_GET_FINGER_STATE_1(ts_reg_data.finger_state_reg) == 1 && touch1_prestate ==1)
		{
			ts_pre_state = 1;
			touch2_prestate = 1;
  			curr_ts_data.X_position[1] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger1_reg, ts_reg_data.XY_low_position_finger1_reg);
  			curr_ts_data.Y_position[1] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger1_reg, ts_reg_data.XY_low_position_finger1_reg);
 			synaptics_history.X2_position[i2] = curr_ts_data.X_position[1];
			synaptics_history.Y2_position[i2] = curr_ts_data.Y_position[1];
 			synaptics_history.distance[i2] = abs(abs(synaptics_history.X2_position[i2]-synaptics_history.X_position[i2])+abs(synaptics_history.Y2_position[i2]-synaptics_history.Y_position[i2]));
			
        	if ((((ts_reg_data.XY_width_finger1_reg & 240) >> 4) - (ts_reg_data.XY_width_finger1_reg & 15)) > 0)
				width1 = (ts_reg_data.XY_width_finger1_reg & 240) >> 4;
			else
				width1 = ts_reg_data.XY_width_finger1_reg & 15;
			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, width1);

       		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[1]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[1]);
			
			printk("Second_x= %d second_y=%d\n", curr_ts_data.X_position[1], curr_ts_data.Y_position[1]);

			input_mt_sync(ts->input_dev);
		}else{
			touch2_prestate = 0;
			i2 = 0;
		}

#if 0
		if(TS_SNTS_GET_FINGER_STATE_2(ts_reg_data.finger_state_reg) == 1 && touch1_prestate ==1 && touch2_prestate == 1)
		{
			ts_pre_state = 1;
			curr_ts_data.X_position[2] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger2_reg, ts_reg_data.XY_low_position_finger2_reg);
  			curr_ts_data.Y_position[2] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger2_reg, ts_reg_data.XY_low_position_finger2_reg);
			
 			if ((((ts_reg_data.XY_width_finger2_reg & 240) >> 4) - (ts_reg_data.XY_width_finger2_reg & 15)) > 0)
				width2 = (ts_reg_data.XY_width_finger2_reg & 240) >> 4;
			else
				width2 = ts_reg_data.XY_width_finger2_reg & 15;
			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, width2);
        	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[2]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[2]);
			
			printk("Third_x= %d Third_y=%d\n", curr_ts_data.X_position[2], curr_ts_data.Y_position[2]);
		
			input_mt_sync(ts->input_dev);	
		}
#endif
#if 0
		if(TS_SNTS_GET_FINGER_STATE_3(ts_reg_data.finger_state_reg[0]) == 1 && touch1_prestate ==1 && touch2_prestate == 1 )
		{
			ts_pre_state = 1;
  			curr_ts_data.X_position[3] = (int)TS_SNTS_GET_X_POSITION(ts_reg_data.X_high_position_finger3_reg, ts_reg_data.XY_low_position_finger3_reg);
  			curr_ts_data.Y_position[3] = (int)TS_SNTS_GET_Y_POSITION(ts_reg_data.Y_high_position_finger3_reg, ts_reg_data.XY_low_position_finger3_reg);
			
 			if ((((ts_reg_data.XY_width_finger3_reg & 240) >> 4) - (ts_reg_data.XY_width_finger3_reg & 15)) > 0)
				width3 = (ts_reg_data.XY_width_finger3_reg & 240) >> 4;
			else
				width3 = ts_reg_data.XY_width_finger3_reg & 15;
			
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, width3);
        	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, curr_ts_data.X_position[3]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, curr_ts_data.Y_position[3]);
			
			printk("Fourth_x= %d Fourth_y=%d\n", curr_ts_data.X_position[3], curr_ts_data.Y_position[3]);
		
			input_mt_sync(ts->input_dev);	
		}
#endif
			input_mt_sync(ts->input_dev);
	
		if (touch1_prestate ==1 && touch2_prestate == 1)
			i2 = i2+1;
		               
		input_sync(ts->input_dev);
		
//		if (ts_pre_state == 0) {
//			break;
//		}
	}/* End of While(1) */
	
	if (int_mode & SYNAPTICS_INT_BUTTON) {
		unsigned int state = 0;
		unsigned int last_state = 0;
		unsigned int button_map[4] = {KEY_MENU, KEY_HOME, KEY_BACK, KEY_SEARCH};
		int r;

		state = i2c_smbus_read_byte_data(ts->client, 0x20);
		
		last_state = ts->button_state;
		
		//for (i = 0; i < sd->button_caps.button_count; i++) {
		for (i = 0; i < 4; i++) {
			const u32 mask = (1 << i);
			if ((state & mask) ^ (last_state & mask)) {
				//input_report_key(sd->tp[0].idev, pdata->button_map[button_nr], val);
				input_report_key(ts->input_dev, 
						button_map[i], 
						state & mask ? 1 : 0);
				input_sync(ts->input_dev);
			}
		}
		ts->button_state = state;
	}

SYNAPTICS_TS_IDLE:
	if (ts->use_irq) {		
		enable_irq(gpio_to_irq(ts->client->irq));
	}
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL); /* 12.5 msec */

	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

//	DEBUG_MSG("LGE: synaptics_ts_irq_handler\n");
//	printk("LGE: synaptics_Ts_irq_handler>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>..%d\n",ts->client->irq);
	disable_irq_nosync(gpio_to_irq(ts->client->irq));
//	printk("LGE1: synaptics_Ts_irq_handler>>>>>>>>>..\n");
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

//added by jykim using kukyongku source
//#include <mach/control.h>
static void Touch_reboot(void){
    DEBUG_MSG("LGE:Heaven Synaptics Touch IC reboot \n");
   // gpio_direction_output(TOUCH_INT, 0);
   // msleep(500);
   // gpio_set_value(TOUCH_INT, 0);
   // msleep(500);
    gpio_direction_output(HEAVEN_TS_EN_GPIO, 0);
    msleep(500);
    gpio_set_value(HEAVEN_TS_EN_GPIO, 1);
    msleep(400);

}
//ended by jykim
static void check_FW(struct i2c_client *client)
{

	unsigned ret;
	ret = i2c_smbus_read_byte_data(client, 0xAB);

	printk("$$ %s() - FW rev. : %d\n", __func__, ret);
}

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
	int ret = 0,r;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	unsigned long irqflags;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;
	uint32_t panel_version;

	DEBUG_MSG("%s() -- start\n\n\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	if (pdata) {
//	if (pdata) {
		while (pdata->version > panel_version)
			pdata++;
		ts->flags = pdata->flags;
		ts->sensitivity_adjust = pdata->sensitivity_adjust;
		irqflags = pdata->irqflags;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		irqflags = 0;
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}

  	memset(&ts_reg_data, 0x0, sizeof(ts_sensor_data));
  	memset(&curr_ts_data, 0x0, sizeof(ts_finger_data));

	/*************************************************************************************************
	 * 3. Power up
	 *************************************************************************************************/
//	Touch_reboot();		//added by jykim
//    gpio_set_value(BLUE_LED_GPIO, 0);
//	udelay(300);

	/*************************************************************************************************
	 * 4. Read RMI Version
	 * To distinguish T1021 and T1007. Select RMI Version
	 * TODO: Power를 이전에 하는 것으로 변경하면 위치 변경해야 한다.
	 *************************************************************************************************/

	
	i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data);

	
	DEBUG_MSG("[LUCKYJUN77] read data check : status_reg(%d), interrupt_status_reg(%d,)\n", ts_reg_data.device_status_reg, ts_reg_data.interrupt_status_reg);

	if(ts_reg_data.interrupt_status_reg == 6)
	{
		printk("%s() - ts_reg_data.interrupt_status_reg : %d\n", __func__, ts_reg_data.interrupt_status_reg);
		queue_work(synaptics_wq, &ts->work);
	}

	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP);
	#if 0
	ret = i2c_smbus_write_byte_data(ts->client, 0x53, 0x03);
	ret = i2c_smbus_write_byte_data(ts->client, 0x54, 0x03);
	#else	/* JamesLee :: for 4.3inch touch - I guess it's delta x, y thresh */
	ret = i2c_smbus_write_byte_data(ts->client, 0x25, 0x03);
	ret = i2c_smbus_write_byte_data(ts->client, 0x26, 0x03);
	#endif

//	ret = i2c_smbus_write_byte_data(ts->client, 0x9B, 32);
//	ret = i2c_smbus_write_byte_data(ts->client, 0x9C, 32);

//	max_low = (unsigned char)i2c_smbus_read_byte_data(ts->client, 0x58);
//	max_high = (unsigned char)i2c_smbus_read_byte_data(ts->client, 0x59); 

//	max_x = max_high <<8
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
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
    	set_bit(EV_TG, ts->input_dev->evbit);
    	
    	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
	snap_left_on = snap_left_on * max_x / 0x10000;
	snap_left_off = snap_left_off * max_x / 0x10000;
	snap_right_on = snap_right_on * max_x / 0x10000;
	snap_right_off = snap_right_off * max_x / 0x10000;
	snap_top_on = snap_top_on * max_y / 0x10000;
	snap_top_off = snap_top_off * max_y / 0x10000;
	snap_bottom_on = snap_bottom_on * max_y / 0x10000;
	snap_bottom_off = snap_bottom_off * max_y / 0x10000;
	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;
	ts->snap_down[!!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_left;
	ts->snap_up[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x + inactive_area_right;
	ts->snap_down[!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_top;
	ts->snap_up[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y + inactive_area_bottom;
	ts->snap_down_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_on;
	ts->snap_down_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_off;
	ts->snap_up_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_on;
	ts->snap_up_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_off;
	ts->snap_down_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_on;
	ts->snap_down_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_off;
	ts->snap_up_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_on;
	ts->snap_up_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_off;

	DEBUG_MSG(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	DEBUG_MSG(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	DEBUG_MSG(KERN_INFO "synaptics_ts_probe: snap_x %d-%d %d-%d, snap_y %d-%d %d-%d\n",
	       snap_left_on, snap_left_off, snap_right_on, snap_right_off,
	       snap_top_on, snap_top_off, snap_bottom_on, snap_bottom_off);
#if 0
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 986, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1644, fuzz_y, 0);
#else	/* JamesLee :: for 4.3 inch touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 1123, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1872, fuzz_y, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, fuzz_w, 0);
		input_set_abs_params(ts->input_dev, SYN_TG_REPORT, 0, 3, 0, 0);
	input_set_abs_params(ts->input_dev, TG_DIR, 0, 3, 0, 0);
	input_set_abs_params(ts->input_dev, TG_SPEED, 0, 1681, 0, 0);
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk("########## irq [%d], irqflags[0x%x]\n", client->irq, irqflags);
	r = gpio_request(client->irq, "Synaptics gpio interrupt");
	if (r < 0){
		dev_dbg(&client->dev,"unable to gent INT GPIO");
		r=-ENODEV;
		goto err_alloc_data_failed;
	}
	gpio_direction_input(client->irq);
	
	
	if (client->irq) {
		ret = request_irq(gpio_to_irq(client->irq), synaptics_ts_irq_handler, irqflags, client->name, ts);
//		if (ret == 0) {
			//ret = i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */
			/* Enable ABS0 and Button Interrupt */
//			ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_INT_REG, SYNAPTICS_INT_ABS0|SYNAPTICS_INT_BUTTON);
//			if (ret)
//				free_irq(client->irq, ts);
//		}
		if (ret == 0) {
			ts->use_irq = 1;
			DEBUG_MSG("request_irq\n");
			}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	check_FW(ts->client);

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
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(gpio_to_irq(client->irq), ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
//   printk("SELWIN:@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@222\n"); 
//  gpio_set_value(BLUE_LED_GPIO, 1);
	if (ts->use_irq)
		disable_irq(gpio_to_irq(client->irq));
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(gpio_to_irq(client->irq));
	ret = i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_SLEEP); /* sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	check_FW(client);
//    gpio_set_value(BLUE_LED_GPIO, 0);
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power on failed\n");
	}
	
// 20100525 jh.koo@lge.com Turn off touch LDOs [START_LGE]
	i2c_smbus_read_i2c_block_data(ts->client, START_ADDR, sizeof(ts_reg_data), &ts_reg_data);
   	i2c_smbus_write_byte_data(ts->client, SYNAPTICS_CONTROL_REG, SYNAPTICS_CONTROL_NOSLEEP); /* wake up */
	ret = i2c_smbus_write_byte_data(ts->client, 0x53, 0x03);
	ret = i2c_smbus_write_byte_data(ts->client, 0x54, 0x03);
// 20100525 jh.koo@lge.com Turn off touch LDOs [END_LGE]

	if (ts->use_irq)
		enable_irq(gpio_to_irq(client->irq));

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	check_FW(client);
	
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
// 20100525 sookyoung.kim@lge.com [START_LGE]
	{ },
	//{ }
// 20100525 sookyoung.kim@lge.com [END_LGE]
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
// 20100525 sookyoung.kim@lge.com [START_LGE]
		//.name	= "heaven_i2c_ts",
		.name	= "heaven_synaptics_ts",
// 20100525 sookyoung.kim@lge.com [END_LGE]
		.owner = THIS_MODULE,
	},
};

static int __devinit synaptics_ts_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
//	synaptics_wq = __create_workqueue("synaptics_wq", 1, 0, 1);
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


