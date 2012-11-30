/***************************************************************************
* 
*   SiI9244 - MHL Transmitter Driver
*
* Copyright (C) (2011, Silicon Image Inc)
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation version 2.
*
* This program is distributed ¡°as is¡± WITHOUT ANY WARRANTY of any
* kind, whether express or implied; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*****************************************************************************/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <asm/uaccess.h> 
#include <linux/regulator/consumer.h>
/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21, */
#if defined(CONFIG_MUIC)
#include <linux/muic/muic.h>
#include <linux/muic/muic_client.h>
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */

//#include "sii9244_driver.h"
//#include "Common_Def.h"

// LGE_CHANGE_S [jh.koo, kibum.lee] 2011-08-04, for MHL interrupt sync
#include <linux/mutex.h>

#include "sii9244_driver.h"

extern void hdmi_common_send_uevent(int x, int y, int action, int tvCtl_x, int tvCtl_y);
extern void hdmi_common_send_keyevent(u8 code);
// LGE_CHANGE_E [jh.koo, kibum.lee] 2011-08-04, for MHL interrupt sync

// LGE_CHANGE_S [jh.koo, kibum.lee] 2011-08-20, for MHL safety
#include <linux/wakelock.h>
static struct wake_lock mhl_lock;
static int mhl_connected = 0;
extern void sii9244_driver_init(void);
extern struct timer_list simg_timer;

struct mhl_work_struct {
	struct work_struct work;
};
// LGE_CHANGE_E [jh.koo, kibum.lee] 2011-08-20, for MHL safety

#if 0
#define SII_LOG_FUNCTION_NAME_ENTRY             printk(KERN_INFO "[SII9244]## %s() ++ ##\n",  __func__);
#define SII_LOG_FUNCTION_NAME_EXIT              printk(KERN_INFO "[SII9244]## %s() -- ##\n",  __func__);
#else
#define SII_LOG_FUNCTION_NAME_ENTRY
#define SII_LOG_FUNCTION_NAME_EXIT
#endif

#ifdef CONFIG_MHL_LG_MOTION_CTL
#define MHL_DEV_INPUT_KBMOUSE_EVENT 		"/dev/input/event9"
#undef MHL_DEV_INPUT_KBMOUSE_EVENT

#define DRIVER_VERSION "v1.0"
#define DRIVER_NAME "mhl_virtual_kbmouse"
#define DISABLE_CURSOR	10000

#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
static struct input_dev *mhl_virtual_kbmouse;
#endif
#endif
struct work_struct sii9244_int_work;
struct workqueue_struct *sii9244_wq = NULL;

struct i2c_driver sii9244_i2c_driver;
struct i2c_client *sii9244_i2c_client = NULL;

struct i2c_driver sii9244a_i2c_driver;
struct i2c_client *sii9244a_i2c_client = NULL;

struct i2c_driver sii9244b_i2c_driver;
struct i2c_client *sii9244b_i2c_client = NULL;

struct i2c_driver sii9244c_i2c_driver;
struct i2c_client *sii9244c_i2c_client = NULL;


//extern bool sii9244_init(void);
//extern void sii9244_mhl_tx_int(void);
//extern void simg_mhl_tx_handler(void);
extern int dss_mainclk_enable(void);
extern void dss_mainclk_disable(void);

// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-27, [P940] add HDMI/MHL driver
extern int mhl_power_control(int on);
// LGE_CHANGE_E [jh.koo@lge.com]
// LGE_CHANGE_S [jh.koo@lge.com] 2011-07-22, [P940] add HDMI hot plug detect enable control
extern int hpd_enable_control(int on);
// LGE_CHANGE_E [jh.koo@lge.com]

// LGE_CHANGE [jh.koo kibum.lee] 20110806 , MHL RCP codes into media keys and transfer theses to the input manager
struct mhl_rcp_dev{
	char *name;
	struct device *dev;
	unsigned char code;
};
static struct i2c_device_id sii9244_id[] = {
	{"sii9244_mhl_tx", 0},
	{}
};

static struct i2c_device_id sii9244a_id[] = {
	{"sii9244_tpi", 0},
	{}
};

static struct i2c_device_id sii9244b_id[] = {
	{"sii9244_hdmi_rx", 0},
	{}
};

static struct i2c_device_id sii9244c_id[] = {
	{"sii9244_cbus", 0},
	{}
};

int MHL_i2c_init = 0;


struct sii9244_state {
	struct i2c_client *client;
};

void sii9244_cfg_power(bool on);

static void sii9244_cfg_gpio(void);

irqreturn_t mhl_int_irq_handler(int irq, void *dev_id);

irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id);

void sii9244_interrupt_event_work(struct work_struct *p);

#define MHL_SWITCH_TEST	1

#ifdef MHL_SWITCH_TEST
struct class *sec_mhl;
EXPORT_SYMBOL(sec_mhl);

struct device *mhl_switch;
EXPORT_SYMBOL(mhl_switch);

//-------------------------------------------------------------------------------------------
// kibum.lee@lge.com Don't worry!! this api is test code.
static ssize_t check_MHL_command(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;

	SII_LOG_FUNCTION_NAME_ENTRY;
	sii9244_cfg_power(1);
	count = sprintf(buf,"%d\n", res );
	sii9244_cfg_power(0);
    SII_LOG_FUNCTION_NAME_EXIT;
	return count;

}

// kibum.lee@lge.com Don't worry!! this api is test code.
static ssize_t change_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	int i;
    SII_LOG_FUNCTION_NAME_ENTRY;
    
	SII_DEV_DBG("Change the switch: %ld\n", value);

	if (value == 0) {
		for (i = 0; i <20; i++) {
			SII_DEV_DBG("[MHL] try %d\n", i+1);
			msleep(500);
		}
// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-21, [P940] add  MHL driver 
//		gpio_request(GPIO_MHL_INT, "MHL_INT");
		gpio_direction_input(GPIO_MHL_INT);
		request_irq(gpio_to_irq(GPIO_MHL_INT), mhl_int_irq_handler, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "mhl_irq", dev);
// LGE_CHANGE_E [jh.koo@lge.com]		
		sii9244_cfg_power(1);
//		sii9244_init();
//		sii9244_mhl_tx_int();
	} else {	
		sii9244_cfg_power(0);
// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-21, [P940] add  MHL driver 
//		gpio_request(GPIO_MHL_SEL, "MUIC/MHL SEL");
		gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
// LGE_CHANGE_E [jh.koo@lge.com]	
	}
    SII_LOG_FUNCTION_NAME_EXIT;

	return size;
}
//-------------------------------------------------------------------------------------------

void MHL_On(bool on)
{
	static DEFINE_MUTEX(mutex);
    SII_LOG_FUNCTION_NAME_ENTRY;

	mutex_lock(&mutex);

	printk("[MHL] USB path change : %d\n", on);
    
	if (on == 1)
    {		
		mhl_connected = true;
		
		if(gpio_get_value(GPIO_MHL_SEL))
		{
			printk("[MHL] GPIO_MHL_SEL : already 1\n");
        }
		else 
        {
			//gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
			sii9244_cfg_power(1);
//			sii9244_init();
			sii9244_mhl_tx_int();
 			gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH); //daniel for mass product issue

            // LGE_CHANGE_S [jh.koo@lge.com] 2011-07-22, [P940] add HDMI hot plug detect enable control
             hpd_enable_control(1);
            // LGE_CHANGE_E [jh.koo@lge.com]
		}
        
		wake_lock(&mhl_lock);		// LGE_CHANGE [jh.koo, kibum.lee] 2011-08-20, for MHL safty
	} 
    else
    {
        mhl_connected = 0;        
		if(! gpio_get_value(GPIO_MHL_SEL) )
		{
			printk("[MHL] GPIO_MHL_SEL : already 0\n");
        }
		else 
        {
			gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
			sii9244_cfg_power(0);
#ifdef CONFIG_MHL_LG_MOTION_CTL
    mhl_ms_hide_cursor();
    mhl_writeburst_uevent(2);
#endif
		}
		
		wake_unlock(&mhl_lock);		// LGE_CHANGE [jh.koo, kibum.lee] 2011-08-20, for MHL safty
		wake_lock_timeout(&mhl_lock, 2*HZ);

	}

	mutex_unlock(&mutex);
	printk("[MHL] USB path change : %d \tP940 MHL Muxtex end\n", on);
    SII_LOG_FUNCTION_NAME_EXIT;
}
EXPORT_SYMBOL(MHL_On);


// LGE_CHANGE_S [jh.koo kibum.lee] 20110915
bool is_MHL_connected(void)
{
    if(mhl_connected)
        return TRUE;
    
	return FALSE;

}
EXPORT_SYMBOL(is_MHL_connected);
// LGE_CHANGE_E [jh.koo kibum.lee] 20110915


static DEVICE_ATTR(mhl_sel, S_IRUGO | S_IWUSR | S_IXOTH, check_MHL_command, change_switch_store);
#endif

static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;
	#if 0

	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_UP);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
	

	//TVout_LDO_ctrl(true);
	
	if(!MHD_HW_IsOn())
	{
		sii9244_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();		
	}
	else
	{
		sii9244_tpi_init();
		res = MHD_Read_deviceID();
	}

	I2C_WriteByte(0x72, 0xA5, 0xE1);
	res = 0;
	res = I2C_ReadByte(0x72, 0xA5);

	printk(KERN_ERR "A5 res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1B);

	printk(KERN_ERR "Device ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1C);

	printk(KERN_ERR "Device Rev ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1D);

	printk(KERN_ERR "Device Reserved ID res %x",res);

	printk(KERN_ERR "\n####HDMI_EN1 %x MHL_RST %x GPIO_MHL_SEL %x\n",gpio_get_value(GPIO_MHL_EN),gpio_get_value(GPIO_MHL_RST),gpio_get_value(GPIO_MHL_SEL));

	res = I2C_ReadByte(0x7A, 0x3D);

	res = I2C_ReadByte(0x7A, 0xFF);
		
	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_NONE);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
#endif
	count = sprintf(buf,"%d\n", res );
	//TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	SII_DEV_DBG("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(MHD_file, S_IRUGO , MHD_check_read, MHD_check_write);

/* LGE_CHANGE_S [donghyuk79.park@lge.com] 2012-03-08, */
char mhl_orient_value[2];


int get_mhl_orientation(){
	int ret=0;
	switch(mhl_orient_value[0]){
		case '0' :
			ret=0;
			break;
		case '1' :
			ret=1;
			break;
		case '2' :
			ret=2;
			break;
		case '3' :
			ret=3;
			break;
		default	 :
			ret=0;
			break;

	}
	return ret;
}EXPORT_SYMBOL(get_mhl_orientation);


static ssize_t MHL_orient_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;

	count = sprintf(buf,"%s", mhl_orient_value );

	return count;
}



static ssize_t MHL_orient_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{

	snprintf(mhl_orient_value, sizeof(mhl_orient_value), "%s", buf);

	return size;
}

static DEVICE_ATTR(MHL_orient, S_IRUGO | S_IWUSR | S_IXOTH , MHL_orient_read, MHL_orient_write);
/* LGE_CHANGE_E [donghyuk79.park@lge.com] 2012-03-08, */

struct i2c_client* get_sii9244_client(u8 device_id)
{

	struct i2c_client* client_ptr;
    SII_LOG_FUNCTION_NAME_ENTRY;

	if(device_id == 0x72)
		client_ptr = sii9244_i2c_client;
	else if(device_id == 0x7A)
		client_ptr = sii9244a_i2c_client;
	else if(device_id == 0x92)
		client_ptr = sii9244b_i2c_client;
	else if(device_id == 0xC8)
		client_ptr = sii9244c_i2c_client;
	else
		client_ptr = NULL;
    
    SII_LOG_FUNCTION_NAME_EXIT;

	return client_ptr;
}
EXPORT_SYMBOL(get_sii9244_client);

u8 sii9244_i2c_read(struct i2c_client *client, u8 reg)
{
	u8 ret;
	SII_LOG_FUNCTION_NAME_ENTRY;
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG_ERROR("I2C not ready");
		return 0;
	}
	
	i2c_smbus_write_byte(client, reg);
	

	ret = i2c_smbus_read_byte(client);

	//printk(KERN_ERR "#######Read reg %x data %x\n", reg, ret);

	if (ret < 0)
	{
		SII_DEV_DBG_ERROR("i2c read fail");
		return -EIO;
	}
    SII_LOG_FUNCTION_NAME_EXIT;
	return ret;

}
EXPORT_SYMBOL(sii9244_i2c_read);


int sii9244_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
    int rc = 0;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
	if(!MHL_i2c_init)
	{
		SII_DEV_DBG_ERROR("I2C not ready");
		return 0;
	}

	//printk(KERN_ERR "#######Write reg %x data %x\n", reg, data);
	rc = i2c_smbus_write_byte_data(client, reg, data);
	SII_LOG_FUNCTION_NAME_EXIT;
	return rc; 
}
EXPORT_SYMBOL(sii9244_i2c_write);


void sii9244_interrupt_event_work(struct work_struct *p)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	struct mhl_work_struct *mhl_work =
		container_of(p, struct mhl_work_struct, work);

	printk(KERN_ERR "[MHL]sii9244_interrupt_event_work() is called\n");
	simg_mhl_tx_handler();
    
    if(mhl_work != NULL)
    {
        kfree(mhl_work);
        mhl_work = NULL;
    }
    SII_LOG_FUNCTION_NAME_EXIT;
}

#if 0
void mhl_int_irq_handler_sched(void)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
	
	//printk(KERN_ERR "mhl_int_irq_handler_sched() is called\n");
	queue_work(sii9244_wq, &sii9244_int_work);		
    SII_LOG_FUNCTION_NAME_EXIT;
}
#else

void mhl_int_irq_handler_sched(void)
{
	struct mhl_work_struct *mhl_work;
    SII_LOG_FUNCTION_NAME_ENTRY;
    
	mhl_work = kmalloc(sizeof(*mhl_work), GFP_ATOMIC);

	if (mhl_work) 
	{
		INIT_WORK(&mhl_work->work, sii9244_interrupt_event_work);
		queue_work(sii9244_wq, &mhl_work->work);
	} 
    else 
	{
		printk(KERN_ERR "Cannot allocate memory to create work");
	}
    SII_LOG_FUNCTION_NAME_EXIT;
}
#endif


irqreturn_t mhl_int_irq_handler(int irq, void *dev_id)
{
    SII_LOG_FUNCTION_NAME_ENTRY;

    if (gpio_get_value(GPIO_MHL_SEL))	
    {   
        SII_DEV_DBG("irq=%d, dev_id=%p\n", irq, dev_id);
        mhl_int_irq_handler_sched();
    }
    SII_LOG_FUNCTION_NAME_EXIT;
    return IRQ_HANDLED;
}

// LGE_CHANGE_S [jh.koo kibum.lee] 20110806 , MHL RCP codes into media keys and transfer theses to the input manager
#if 0
void rcp_cbus_uevent(u8 rcpCode)	
{
	char env_buf[120];
	u8 code= 0x0;


	memset(env_buf, 0, sizeof(env_buf));
	printk("%s : RCP Message Recvd , rcpCode =0x%x\n",__func__,rcpCode);

	switch(rcpCode)
	{
	case 0x60: // Play Function
		code = 0x44;
		break;
	case 0x61: //Pause_Play Func
		code =  0x46;
		break;
	case 0x64://Stop Function
		code = 0x45;
		break;
	default:
		code = rcpCode;
		break;
	}

	printk("%s : change code , rcpCode =0x%x\n",__func__,code);
		
	sprintf(env_buf, "MHL_RCP=%d", code);	

       hdmi_common_send_uevent(env_buf);
	return;
}
#else
void rcp_cbus_uevent(u8 rcpCode)	
{
	u8 code= 0x0;
    SII_LOG_FUNCTION_NAME_ENTRY;

	//SII_DEV_DBG("RCP Message Recvd , rcpCode =0x%x\n",rcpCode);

	code = rcpCode;

	SII_DEV_DBG("RCP Message Recvd  rcpCode =0x%x   change code , rcpCode =0x%x\n",rcpCode, code);

	hdmi_common_send_keyevent(code);	
    SII_LOG_FUNCTION_NAME_EXIT;
	//hdmi_common_send_uevent(code);
	return;
}

#endif
#ifdef CONFIG_MHL_LG_MOTION_CTL
int mhl_ms_ptr(signed short x, signed short y)	// mouse pointer, here because of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct file *fd;
	struct input_event event;
	int ret;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_ms_input, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	event.type = EV_REL;
	event.code = REL_X;
	event.value = x;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_REL;
	event.code = REL_Y;
	event.value = y;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#endif
	return 0;
}

void mhl_ms_hide_cursor(void)	// in case mhl disconnection
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	mhl_ms_ptr(DISABLE_CURSOR,DISABLE_CURSOR);
#endif
}

int mhl_ms_btn(int action)		// mouse button...only left button, here becasue of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct file *fd;
	int ret;
	struct input_event event;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_ms_input, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	event.type = EV_KEY;
	event.code = BTN_LEFT;	// = BTN_MOUSE
	event.value = action;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);
	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#endif
	return 0;
}

int mhl_kbd_key(unsigned int tKeyCode, int value)	// keyboard key input, here because of speed
{
#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
	struct input_event event;
	struct file *fd;
	int ret;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	fd = filp_open(MHL_DEV_INPUT_KBMOUSE_EVENT, O_RDWR, 0);

	if (IS_ERR(fd)) {
		printk("mhl_kbd_key, could not open touch device \n");
	       set_fs(fs);

		return 1;
	}

	// TODO : supported key condition
	event.type = EV_KEY;
	event.code = tKeyCode;
	event.value = value;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	event.type = EV_SYN;
	event.code = SYN_REPORT;
	event.value = 0;
	ret =  vfs_write(fd, (char*)&event, sizeof(event), &fd->f_pos);

	filp_close(fd, NULL);

	set_fs(fs);
#else
	char mbuf[120];

	memset(mbuf, 0, sizeof(mbuf));
	sprintf(mbuf, "MHL_KEY press=%04d, keycode=%04d", value, tKeyCode);
	//hdmi_common_send_uevent(mbuf);
#endif
	return 0;
}

#if 0 // just use the touch of target
static int mhl_virtual_touch_register(void)		// touch emulator
{
	return 0;
}
#endif

int MHLRCPtoKeyboard(byte mhlrcpkey)	// for iot with TV and Monitor
{
	return 0;
}

#ifdef MHL_DEV_INPUT_KBMOUSE_EVENT
static int mhl_virtual_kbmouse_register(void)		// mouse & keyboard emulator...
{
	int rc;
	unsigned short key_cnt = 0x00;

	mhl_virtual_kbmouse = input_allocate_device();
	if (!mhl_virtual_kbmouse) {
		printk("%s: allocation failure for input device\n", __func__);
		return -ENOMEM;
	}

	mhl_virtual_kbmouse->name = DRIVER_NAME;
	mhl_virtual_kbmouse->id.bustype = 0x0000;
	mhl_virtual_kbmouse->id.vendor = 0x0000;
	mhl_virtual_kbmouse->id.product = 0x0001;
	mhl_virtual_kbmouse->id.version = 0x0100;

	mhl_virtual_kbmouse->evbit[0] = BIT_MASK(EV_KEY) |BIT_MASK(EV_REL);
	mhl_virtual_kbmouse->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT);
	mhl_virtual_kbmouse->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);
	for(key_cnt = 0x01; key_cnt < KEY_REDO; key_cnt++)
	{
		set_bit(key_cnt, mhl_virtual_kbmouse->keybit);
	}

	rc = input_register_device(mhl_virtual_kbmouse);
	if (rc)
	{
		printk("%s: register failure for input device\n", __func__);
	}

	printk("%s: register success for input device\n", __func__);
	return rc;
}
#endif

void mhl_writeburst_uevent(unsigned short mev)
{
	char env_buf[120];

	memset(env_buf, 0, sizeof(env_buf));

	switch(mev)
	{
		case 1:
			sprintf(env_buf, "hdmi_kbd_on");
			break;

		case 2:
			sprintf(env_buf, "hdmi_kbd_off");
			break;

		default:

			break;
	}
	printk("sii9244.c -> mhl_writeburst_uevent before hdmi~ \n");
     //  hdmi_common_send_uevent(env_buf);
	return;
}

EXPORT_SYMBOL(mhl_writeburst_uevent);
#endif
EXPORT_SYMBOL(rcp_cbus_uevent);
// LGE_CHANGE_E [jh.koo kibum.lee] 20110806 , MHL RCP codes into media keys and transfer theses to the input manager
 
irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id)
{
    SII_LOG_FUNCTION_NAME_ENTRY;

    if (gpio_get_value(GPIO_MHL_SEL))	
        mhl_int_irq_handler_sched();
    
	SII_LOG_FUNCTION_NAME_EXIT;
	return IRQ_HANDLED;
}

/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21, */
#if defined(CONFIG_MUIC)
int mhl_on_none(struct muic_client_device *mcdev)
{
	MHL_On(0);
	return 0;

}

int mhl_on_mhl(struct muic_client_device *mcdev)
{
	MHL_On(1);
	return 0;
}
static struct muic_client_ops mhl_ops = {
	.on_none = mhl_on_none,
	.on_mhl = mhl_on_mhl,
};
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */



static int sii9244_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* int retval; */

	struct sii9244_state *state;

	struct class *mhl_class;
	struct device *mhl_dev;
/* LGE_CHANGE_S [donghyuk79.park@lge.com] 2012-03-08, */
	struct class *mhl_class_orient;
	struct device *mhl_dev_orient;
/* LGE_CHANGE_E [donghyuk79.park@lge.com] 2012-03-08, */
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);


/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21, */
#if defined(CONFIG_MUIC)
	muic_client_dev_register(client->name, state, &mhl_ops);
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244 attach success!!!\n");

	sii9244_i2c_client = client;

	MHL_i2c_init = 1;

	mhl_class = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(mhl_class))
	{
		SII_DEV_DBG_ERROR("Failed to create class(mhl)!\n");
	}

	mhl_dev = device_create(mhl_class, NULL, 0, NULL, "mhl_dev");
	if (IS_ERR(mhl_dev))
	{
		SII_DEV_DBG_ERROR("Failed to create device(mhl_dev)!\n");
	}

	if (device_create_file(mhl_dev, &dev_attr_MHD_file) < 0)
		SII_DEV_DBG_ERROR("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);

/* LGE_CHANGE_S [donghyuk79.park@lge.com] 2012-03-08, */
	mhl_class_orient = class_create(THIS_MODULE, "mhl_orient");
	if (IS_ERR(mhl_class_orient))
	{
		SII_DEV_DBG_ERROR("Failed to create class(mhl)!\n");
	}

	mhl_dev_orient = device_create(mhl_class_orient, NULL, 0, NULL, "mhl_dev_orient");
	if (IS_ERR(mhl_dev_orient))
	{
		SII_DEV_DBG_ERROR("Failed to create device(mhl_dev)!\n");
	}

	if (device_create_file(mhl_dev_orient, &dev_attr_MHL_orient) < 0)
		SII_DEV_DBG_ERROR("Failed to create device file(%s)!\n", dev_attr_MHL_orient.attr.name);

/* LGE_CHANGE_E [donghyuk79.park@lge.com] 2012-03-08, */
	SII_LOG_FUNCTION_NAME_EXIT;



	return 0;

}



static int __devexit sii9244_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    SII_LOG_FUNCTION_NAME_ENTRY;
    state = i2c_get_clientdata(client);

    // wooho47.jung@lge.com 2011.11.11
    // ADD : for null defense
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

static int sii9244a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244A attach success!!!\n");

	sii9244a_i2c_client = client;
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;

}



static int __devexit sii9244a_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
    
    // wooho47.jung@lge.com 2011.11.11
    // ADD : for null defense    
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

static int sii9244b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244B attach success!!!\n");

	sii9244b_i2c_client = client;

	SII_LOG_FUNCTION_NAME_EXIT;
	return 0;

}



static int __devexit sii9244b_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
    
    // wooho47.jung@lge.com 2011.11.11
    // ADD : for null defense    
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
    
	return 0;
}


static int sii9244c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9244_state *state;
	int ret;
    SII_LOG_FUNCTION_NAME_ENTRY;

	state = kzalloc(sizeof(struct sii9244_state), GFP_KERNEL);
	if (state == NULL) {		
		SII_DEV_DBG_ERROR("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	SII_DEV_DBG("SII9244C attach success!!!\n");

	sii9244c_i2c_client = client;

	msleep(100);	

	sii9244_wq = create_singlethread_workqueue("sii9244_wq");
	//INIT_WORK(&sii9244_int_work, sii9244_interrupt_event_work);

// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-21, [P940] add  MHL driver 
	ret = request_threaded_irq(gpio_to_irq(GPIO_MHL_INT), NULL, mhl_int_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mhl_int", (void *) state); 
// LGE_CHANGE_E [jh.koo@lge.com]	

	if (ret) {
		SII_DEV_DBG_ERROR("unable to request irq mhl_int"
					" err:: %d\n", ret);
		return ret;
	}		
	SII_DEV_DBG("MHL int reques successful %d\n", ret);

	if (ret) {
		SII_DEV_DBG_ERROR("unable to request irq mhl_wake_up"
					" err:: %d\n", ret);
		return ret;
	}		
	SII_LOG_FUNCTION_NAME_EXIT;

	return 0;

}

// LGE_CHANGE_S [jh.koo kibum.lee] 2011-10-05, [P940] when the sleep, disable vdac regulator.
static int sii9244_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
//	client->dev.power.power_state = state;
//	hpd_enable_control(0);	// LGE_DEL [sanghyuk.kwon] 2011.10.25, vdac disable at audio BSP
//	sii9244_cfg_power(0);
	SII_DEV_DBG("[MHL] P940 MHL : vdac Suspend \n");
	return 0;
}

static int sii9244_i2c_resume(struct i2c_client *client)
{
//	client->dev.power.power_state = PMSG_ON;
//	printk(KERN_INFO "[MHL] P940 MHL : Resume \n");
	return 0;
}
// LGE_CHANGE_E [jh.koo kibum.lee] 2011-10-05, [P940] when the sleep, disable vdac regulator.


static int __devexit sii9244c_remove(struct i2c_client *client)
{
	struct sii9244_state *state = NULL;
    
    SII_LOG_FUNCTION_NAME_ENTRY;
    
    state = i2c_get_clientdata(client);
    
    // wooho47.jung@lge.com 2011.11.11
    // ADD : for null defense    
    if(state)
	    kfree(state);
    
    SII_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

struct i2c_driver sii9244_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244_mhl_tx",
	},
	.id_table	= sii9244_id,
	.probe	= sii9244_i2c_probe,
	.remove	= __devexit_p(sii9244_remove),
	.suspend = sii9244_i2c_suspend,
	.resume = sii9244_i2c_resume,
	.command = NULL,
};

struct i2c_driver sii9244a_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244_tpi",
	},
	.id_table	= sii9244a_id,
	.probe	= sii9244a_i2c_probe,
	.remove	= __devexit_p(sii9244a_remove),
	.command = NULL,
};

struct i2c_driver sii9244b_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244_hdmi_rx",
	},
	.id_table	= sii9244b_id,
	.probe	= sii9244b_i2c_probe,
	.remove	= __devexit_p(sii9244b_remove),
	.command = NULL,
};

struct i2c_driver sii9244c_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sii9244_cbus",
	},
	.id_table	= sii9244c_id,
	.probe	= sii9244c_i2c_probe,
	.remove	= __devexit_p(sii9244c_remove),
	.command = NULL,
};


extern struct device * fimc_get_active_device(void);


void sii9244_cfg_power(bool on)
{
    SII_LOG_FUNCTION_NAME_ENTRY;
    //SII_DEV_DBG(" : %d \n",on);

	if(on)
	{
// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-21, [P940] add  MHL driver 		
//		gpio_request(GPIO_MHL_EN, "MHL_EN");
		gpio_direction_output(GPIO_MHL_EN, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_EN, GPIO_LEVEL_HIGH);

		mhl_power_control(1);

		mdelay(2);

//		gpio_request(GPIO_MHL_RST, "MHL_RESET_N");
		gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		mdelay(2);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		mdelay(2);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		mdelay(2);
// LGE_CHANGE_E [jh.koo@lge.com]			
	}
	else
	{
		mhl_power_control(0);
		
		gpio_direction_output(GPIO_MHL_EN, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_MHL_EN, GPIO_LEVEL_LOW);
	}
	
    SII_LOG_FUNCTION_NAME_EXIT;

	return;
}


static void sii9244_cfg_gpio()
{
// LGE_CHANGE_S [jh.koo@lge.com] 2011-05-21, [P940] add  MHL driver
    SII_LOG_FUNCTION_NAME_ENTRY;
	gpio_request(GPIO_MHL_INT, "MHL_INT");
	gpio_direction_input(GPIO_MHL_INT);

	gpio_request(GPIO_MHL_SEL, "MUIC/MHL SEL");
	gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
//	gpio_set_value(GPIO_MHL_WAKE_UP, GPIO_LEVEL_LOW);	

	gpio_request(GPIO_MHL_EN, "MHL_EN");
	gpio_direction_output(GPIO_MHL_EN, GPIO_LEVEL_LOW);
//	gpio_set_value(GPIO_MHL_WAKE_UP, GPIO_LEVEL_LOW);	

	gpio_request(GPIO_MHL_RST, "MHL_RESET_N");
	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
	gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);	

	gpio_request(GPIO_MHL_WAKE_UP, "WAKEUP_MHL");
//	gpio_direction_output(GPIO_MHL_WAKE_UP, GPIO_LEVEL_LOW);
	gpio_direction_input(GPIO_MHL_WAKE_UP);	
//	gpio_set_value(GPIO_MHL_WAKE_UP, GPIO_LEVEL_LOW);
// LGE_CHANGE_E [jh.koo@lge.com]
    SII_LOG_FUNCTION_NAME_EXIT;

}

static int mhl_open(struct inode *ip, struct file *fp)
{
	SII_DEV_DBG("\n");
	return 0;

}

static int mhl_release(struct inode *ip, struct file *fp)
{
	
	SII_DEV_DBG("\n");
	return 0;
}


static int mhl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	SII_DEV_DBG("\n");

#if 0
	byte data;

	switch(cmd)
	{
		case MHL_READ_RCP_DATA:
			data = GetCbusRcpData();
			ResetCbusRcpData();
			put_user(data,(byte *)arg);
			printk(KERN_ERR "MHL_READ_RCP_DATA read");
			break;
		
		default:
		break;
	}
#endif		
	return 0;
}

static struct file_operations mhl_fops = {
	.owner  = THIS_MODULE,
	.open   = mhl_open,
    	.release = mhl_release,
    	.ioctl = mhl_ioctl,
};
                 
static struct miscdevice mhl_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "mhl",
    .fops   = &mhl_fops,
};

static int __init sii9244_module_init(void)
{
	int ret;
    SII_LOG_FUNCTION_NAME_ENTRY;
	sii9244_cfg_gpio();

#ifdef MHL_SWITCH_TEST
	sec_mhl = class_create(THIS_MODULE, "sec_mhl");
	if (IS_ERR(sec_mhl))
		SII_DEV_DBG_ERROR("[MHL] Failed to create class (sec_mhl)\n");

	mhl_switch = device_create(sec_mhl, NULL, 0, NULL, "switch");
	if (IS_ERR(mhl_switch))
		SII_DEV_DBG_ERROR("[MHL] Failed to create device (mhl_switch)\n");
	if (device_create_file(mhl_switch, &dev_attr_mhl_sel) < 0)
		SII_DEV_DBG_ERROR("[MHL] Failed to create file (mhl_sel)\n");
#endif

	ret = misc_register(&mhl_device);
	if(ret) {
		SII_DEV_DBG_ERROR("misc_register failed - mhl \n");
	}

	ret = i2c_add_driver(&sii9244_i2c_driver);
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244] add i2c driver\n");
    }
    
	ret = i2c_add_driver(&sii9244a_i2c_driver);
    
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR( "[MHL SII9244A] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244A] add i2c driver\n");
    }
    
	ret = i2c_add_driver(&sii9244b_i2c_driver);
    
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244B] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244B] add i2c driver\n");
    }

	ret = i2c_add_driver(&sii9244c_i2c_driver);
	if (ret != 0)
	{
	    SII_DEV_DBG_ERROR("[MHL SII9244C] can't add i2c driver\n");	
    }
	else
	{
	    SII_DEV_DBG("[MHL SII9244C] add i2c driver\n");
    }

// LGE_CHANGE_S [jh.koo, kibum.lee] 2011-08-20, for MHL safty
	wake_lock_init(&mhl_lock, WAKE_LOCK_SUSPEND, "mhl_wake_lock");
// LGE_CHANGE_E [jh.koo, kibum.lee] 2011-08-20, for MHL safty
    sii9244_driver_init();
    SII_LOG_FUNCTION_NAME_EXIT;

	return ret;	
}
module_init(sii9244_module_init);
static void __exit sii9244_exit(void)
{
	i2c_del_driver(&sii9244_i2c_driver);
	i2c_del_driver(&sii9244a_i2c_driver);
	i2c_del_driver(&sii9244b_i2c_driver);	
	i2c_del_driver(&sii9244c_i2c_driver);

// LGE_CHANGE_S [jh.koo, kibum.lee] 2011-08-20, for MHL safty
	wake_lock_destroy(&mhl_lock);
// LGE_CHANGE_E [jh.koo, kibum.lee] 2011-08-20, for MHL safty
};
module_exit(sii9244_exit);

MODULE_DESCRIPTION("Sii9244 MHL driver");
MODULE_AUTHOR("Jerry Koo");
MODULE_LICENSE("GPL");
