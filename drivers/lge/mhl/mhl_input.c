/* drivers/lge/mhl/mhl_input.c
 *
 * Copyright (C) 2011 LGE Inc.
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

/*
 * Initial codes: jh.koo kibu.lee 20110810 MHL RCP codes into media keys
 * and transfer theses to the input manager
 * 
 * These codes are seperated from omap4430-keypad.c
 */

#include <linux/input.h>
#include <linux/errno.h>
#include <linux/lge/lge_input.h>
#include <linux/ioctl.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <asm/uaccess.h>
/*
 * This function will be invoked from input driver
 * eg. drivers/input/keyboard/omap4-keypad.c
 */
/* LGE_CHANGE_S [donghyuk79.park@lge.com] 2012-03-08, */
#define I_CANVAS_X          1124
#define I_CANVAS_Y          1873

extern int get_mhl_orientation();
/* LGE_CHANGE_E [donghyuk79.park@lge.com] 2012-03-08, */

void hdmi_common_register_keys(void *dev)
{
	pr_info("mhl_input.c -> hdmi_common_register_keys(void *dev) \n");
	struct input_dev *input = dev;

	if (input == NULL) {
		pr_err("MHL_INPUT: input handle is NULL\n");
		return;
	}

	__set_bit(KEY_ENTER, input->keybit);
	__set_bit(KEY_UP, input->keybit);
	__set_bit(KEY_DOWN, input->keybit);
	__set_bit(KEY_LEFT, input->keybit);
	__set_bit(KEY_RIGHT, input->keybit);
	__set_bit(KEY_BACK, input->keybit);
	__set_bit(KEY_0, input->keybit);
	__set_bit(KEY_1, input->keybit);
	__set_bit(KEY_2, input->keybit);
	__set_bit(KEY_3, input->keybit);
	__set_bit(KEY_4, input->keybit);
	__set_bit(KEY_5, input->keybit);
	__set_bit(KEY_6, input->keybit);
	__set_bit(KEY_7, input->keybit);
	__set_bit(KEY_8, input->keybit);
	__set_bit(KEY_9, input->keybit);
	__set_bit(KEY_PLAYCD, input->keybit);
	__set_bit(KEY_STOPCD, input->keybit);
	__set_bit(KEY_PAUSECD, input->keybit);
	__set_bit(KEY_REWIND, input->keybit);
	__set_bit(KEY_FASTFORWARD, input->keybit);
	__set_bit(KEY_MENU, input->keybit);
	__set_bit(KEY_EJECTCD, input->keybit);
	__set_bit(KEY_BACKSPACE, input->keybit);
	__set_bit(KEY_PREVIOUSSONG, input->keybit);
	__set_bit(KEY_NEXTSONG, input->keybit);
}
EXPORT_SYMBOL(hdmi_common_register_keys);


void hdmi_common_send_keyevent(unsigned char code)
{
/*
#if defined(MAGIC_MOTION_REMOCON_ON)
pr_info("mhl_input.c -> MAGIC_MOTION_REMOCON_ON!!! \n");

#if defined(MAGIC_MOTION_REMOCON_OFF)
pr_info("mhl_input.c -> MAGIC_MOTION_REMOCON_OFF!!! \n");
#endif
*/

#if defined(MAGIC_MOTION_REMOCON_ON)
	pr_info("mhl_input.c -> hdmi_common_send_keyevent(unsigned char code)\n");
	struct input_dev *input;
	
	int key = 0;

	input = lge_input_get();

	if (input == NULL) {
		pr_err("MHL_INPUT: lget_input_get() failed\n");
		return;
	}

	switch (code) {

	case 0x00:
		key = KEY_ENTER;
		break;

	case 0x01:
		key = KEY_UP;
		break;

	case 0x02:
		key = KEY_DOWN;
		break;

	case 0x03:
		key = KEY_LEFT;
		break;

	case 0x04:
		key = KEY_RIGHT;
		break;
	
	case 0x09:
		key = KEY_MENU;
		break;

	case 0x0D:
		key = KEY_BACK;
		break;

	case 0x20:
		key = KEY_0;
		break;

	case 0x21:
		key = KEY_1;
		break;

	case 0x22:
		key = KEY_2;
		break;

	case 0x23:
		key = KEY_3;
		break;

	case 0x24:
		key = KEY_4;
		break;

	case 0x25:
		key = KEY_5;
		break;

	case 0x26:
		key = KEY_6;
		break;

	case 0x27:
		key = KEY_7;
		break;

	case 0x28:
		key = KEY_8;
		break;

	case 0x29:
		key = KEY_9;
		break;
	
	case 0x2B:
		key = KEY_ENTER;
		break;
	
	case 0x2C:
		key = KEY_BACKSPACE;
		break;

	case 0x44:
		key = KEY_PLAYCD;
		break;

	case 0x45:
		key = KEY_STOPCD;
		break;

	case 0x46:
		key = KEY_PAUSECD;
		break;

	case 0x48:
		key = KEY_REWIND;
		break;

	case 0x49:
		key = KEY_FASTFORWARD;
		break;

	case 0x4A:
		key = KEY_EJECTCD;
		break;

	case 0x4B:
		key = KEY_NEXTSONG;
		break;

	case 0x4C:
		key = KEY_PREVIOUSSONG;
		break;
	
	case 0x60:
		key = KEY_PLAYCD;
		break;

	case 0x61:
		key = KEY_PAUSECD;
		break;

	case 0x64:
		key = KEY_STOPCD;
		break;
	
	default:
		key = KEY_RESERVED;
		pr_warn("MHL_INPUT: not supported keycode %d\n", code);
	}
	
	input_report_key(input, key, 1);
	input_sync(input);
	input_report_key(input, key, 0);
	input_sync(input);
	pr_info("MHL_INPUT: sending MHL RCP key (%d)\n", key);
#endif
#if defined(MAGIC_MOTION_REMOCON_OFF)
	pr_info("mhl_input.c -> MAGIC_MOTION_REMOCON_OFF!!! \n");
#endif
}
EXPORT_SYMBOL(hdmi_common_send_keyevent);



/* LGE_CHANGE_S [donghyuk79.park@lge.com] 2012-03-08, */
void hdmi_common_send_uevent(int x, int y, int action, int tvCtl_x, int tvCtl_y)
{
#if defined(MAGIC_MOTION_REMOCON_ON)
	pr_info("[Donghyuk79.park] mhl_input.c -> hdmi_common_send_uevent(char *buf)\n");
	struct input_dev *input_touch;
	
	int temp_x;
	int temp_y;
	
	int temp_last_x;
	int temp_last_y;

	int touchble;

	temp_x=0;
	temp_y=0;

	touchble=1;

	int orient =0;
	orient = get_mhl_orientation();
	if(orient==2){
	orient=0;
	}
	pr_info("[Donghyuk79.park] mhl_input.c -> hdmi_common_send_uevent orient is %d \n",orient);
	input_touch = lge_input_get_touch();
	
	if (input_touch == NULL) {
		pr_info("mhl_input.c -> input_touch is NULL!!!!\n");
		return;
	}else {
		pr_info("mhl_input.c -> input_touch is not null!!!\n");
	}

	switch(orient)
    {
        case 0: // rotation 0
		pr_info("mhl_input.c -> rotation 0\n");
				int vertical_tv_width  = (I_CANVAS_X*tvCtl_y)/I_CANVAS_Y;
				int vertical_tv_edge_l_point = (tvCtl_x - vertical_tv_width)/2;
				int vertical_tv_edge_r_point = tvCtl_x - vertical_tv_edge_l_point;
				int inner_x = x-vertical_tv_edge_l_point;
		pr_info("mhl_input.c -> v_tv_w :%d, v_tv_e_l_p:%d, v_t_e_r_p:%d,inner_x=%d \n",vertical_tv_width,vertical_tv_edge_l_point,vertical_tv_edge_r_point,inner_x);
				if(x<=vertical_tv_edge_l_point){
					temp_x=0;
					touchble=0;
				}else if(x>=vertical_tv_edge_r_point){
					temp_x=I_CANVAS_X;
					touchble=0;
				}else{
					temp_x = (I_CANVAS_X*inner_x)/vertical_tv_width;
				}
					temp_y = (I_CANVAS_Y*y)/tvCtl_y;

				break;

        case 1: // rotation 90 
			pr_info("mhl_input.c -> rotation 3");
				temp_x = I_CANVAS_X-((y*I_CANVAS_X)/tvCtl_y);
				temp_y = (x*I_CANVAS_Y)/tvCtl_x;
            break;

        case 3: // rotation 90
			pr_info("mhl_input.c -> rotation 1");
				temp_x = (y*I_CANVAS_X)/tvCtl_y;
				temp_y = I_CANVAS_Y-((x*I_CANVAS_Y)/tvCtl_x);
            break;

        default:
            return;
            break;
    }

	pr_warn("Original x ->%d y -> %d\n", x,y);
	pr_warn("Translated x ->%d y -> %d\n", temp_x,temp_y);

if(touchble){
	if(action){

		input_report_abs(input_touch, ABS_MT_POSITION_X, temp_x);
		input_report_abs(input_touch, ABS_MT_POSITION_Y, temp_y);
		input_report_abs(input_touch, ABS_MT_PRESSURE, 1);
		input_report_abs(input_touch, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(input_touch, ABS_MT_WIDTH_MINOR, 1);
		
		input_mt_sync(input_touch);
		input_sync(input_touch);

		temp_last_x = temp_x;
		temp_last_y = temp_y;

	}else{

		if(temp_last_x){
			input_report_abs(input_touch, ABS_MT_POSITION_X, temp_last_x);
		}
		if(temp_last_y){
			input_report_abs(input_touch, ABS_MT_POSITION_Y, temp_last_y);
		}
		input_report_abs(input_touch, ABS_MT_PRESSURE, 0);
		input_report_abs(input_touch, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(input_touch, ABS_MT_WIDTH_MINOR, 0);
		input_mt_sync(input_touch);
		input_sync(input_touch);

	}
}else {

		if(temp_last_x){
			input_report_abs(input_touch, ABS_MT_POSITION_X, temp_last_x);
		}
		if(temp_last_y){
			input_report_abs(input_touch, ABS_MT_POSITION_Y, temp_last_y);
		}
		input_report_abs(input_touch, ABS_MT_PRESSURE, 0);
		input_report_abs(input_touch, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(input_touch, ABS_MT_WIDTH_MINOR, 0);
		input_mt_sync(input_touch);
		input_sync(input_touch);
}
#endif
#if defined(MAGIC_MOTION_REMOCON_OFF)
	pr_info("mhl_input.c -> MAGIC_MOTION_REMOCON_OFF!!! \n");
#endif
}
EXPORT_SYMBOL(hdmi_common_send_uevent);
/* LGE_CHANGE_E [donghyuk79.park@lge.com] 2012-03-08, */
