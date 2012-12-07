#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "broadcast_tcc3170.h"

#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_hal.h"
//#define DELAY_USING_WAIT_EVENT_TIMEOUT  /* wait_event_timeout instead of msleep */

#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#endif // DELAY_USING_WAIT_EVENT_TIMEOUT

struct broadcast_tcc3170_ctrl_data
{
	int								pwr_state;
	struct wake_lock					wake_lock;
	struct spi_device* spi_dev;
	struct i2c_client* pclient;
};

static struct broadcast_tcc3170_ctrl_data  TdmbCtrlInfo;

#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT

#define SLEEP_WAIT_UNTIL_MS 0
#define SLEEP_EXIT_USER_STOP 1

static DECLARE_WAIT_QUEUE_HEAD(msleep_wait_queue);  /*wait_event_timeout queue */
static uint32 msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;		/* sleep exit condition not timeout(sleep) */
static uint32 msleep_wait_queue_init = 0;

#else // DELAY_USING_WAIT_EVENT_TIMEOUT

static uint32 user_stop_flg = 0;
static uint32 mdelay_in_flg = 0;

#endif // DELAY_USING_WAIT_EVENT_TIMEOUT

//extern void TcpalSetI2cIoFunction(void);
extern void TcpalSetCspiIoFunction(void);
extern void TcpalSetSpiDevice(struct spi_device* spi_dev);

struct i2c_client*	TCC_GET_I2C_DRIVER(void)
{
	return TdmbCtrlInfo.pclient;
}

struct spi_device*	TCC_GET_SPI_DRIVER(void)
{
    return TdmbCtrlInfo.spi_dev;
}


#ifdef DELAY_USING_WAIT_EVENT_TIMEOUT
void tdmb_tcc3170_set_userstop(void)
{
	if(msleep_exit_condition == SLEEP_WAIT_UNTIL_MS)
	{
		msleep_exit_condition = SLEEP_EXIT_USER_STOP;
		wake_up(&msleep_wait_queue);
	}
}


int tdmb_tcc3170_mdelay(int32 ms)
{
	int rc = 1;
	int wait_rc = OK;

	if(msleep_wait_queue_init == 0)
	{
		init_waitqueue_head(&msleep_wait_queue);
		msleep_wait_queue_init = 1;
	}

	msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;
	/* sleep during msec set or msleep_exit_condition meet */
	wait_rc = wait_event_timeout(msleep_wait_queue, 
		(msleep_exit_condition == SLEEP_EXIT_USER_STOP), msecs_to_jiffies(ms));

	/* wait exit becaus of user stop not timeout */
	if(msleep_exit_condition == SLEEP_EXIT_USER_STOP)
	{
		rc = 0;
	}
	
	msleep_exit_condition = SLEEP_WAIT_UNTIL_MS;

	return rc;
}
#else // DELAY_USING_WAIT_EVENT_TIMEOUT
void tdmb_tcc3170_set_userstop(void)
{
	//user_stop_flg = mode;
	user_stop_flg = ((mdelay_in_flg == 1)? 1: 0 );
}


int tdmb_tcc3170_mdelay(int32 ms)
{
	int	rc = 1;  /* 0 : false, 1 : ture */
	int32	wait_loop =0;
	int32	wait_ms = ms;

	mdelay_in_flg = 1;
	if(ms > 100)
	{
		wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
		wait_ms = 100;
	}

	do
	{
		msleep(wait_ms);
		if(user_stop_flg == 1)
		{
			TcbdDebug(DEBUG_ERROR,"~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);

	mdelay_in_flg = 0;
	user_stop_flg = 0;

	if(rc == 0)
	{
		TcbdDebug(DEBUG_ERROR,"tdmb_tcc3170_delay return abnormal\n");
	}

	return rc;
}
#endif // DELAY_USING_WAIT_EVENT_TIMEOUT


void tdmb_tcc3170_must_mdelay(int32 ms)
{
	msleep(ms);
}


int tdmb_tcc3170_power_on(void)
{
#if defined(__I2C_STS__)
	if((TdmbCtrlInfo.pctrl_fun == NULL) ||(TdmbCtrlInfo.pctrl_fun->dmb_power_on == NULL))
	{
		TcbdDebug(DEBUG_ERROR,"tdmb_tcc3170_power_on function NULL\n");

		return FALSE;
	}

	wake_lock(&TdmbCtrlInfo.wake_lock);
	TdmbCtrlInfo.pctrl_fun->dmb_power_on( );
#elif defined(__CSPI_ONLY__)
    if(TdmbCtrlInfo.pwr_state != 1)
    {
        wake_lock(&TdmbCtrlInfo.wake_lock);
        TchalPowerOnDevice();
        //TcpalIrqEnable();
    }
    else
    {
        TcbdDebug(DEBUG_ERROR, "aready on!! \n");
    }
#endif //__CSPI_ONLY__
    TdmbCtrlInfo.pwr_state = 1;
    return TRUE;
}

int tdmb_tcc3170_is_power_on()
{
    return (int)TdmbCtrlInfo.pwr_state;
}

int tdmb_tcc3170_power_off(void)
{
    if(TdmbCtrlInfo.pwr_state == 0)
    {
        TcbdDebug(DEBUG_ERROR,"tdmb_tcc3170_power is immediately off\n");

        return TRUE;
    }
#if defined(__I2C__)
    if((TdmbCtrlInfo.pctrl_fun == NULL) ||(TdmbCtrlInfo.pctrl_fun->dmb_power_off == NULL))
    {
        TcbdDebug(DEBUG_ERROR,"tdmb_tcc3170_power_off function NULL\n");

        return FALSE;
    }
    TdmbCtrlInfo.pctrl_fun->dmb_power_off( );
#elif defined(__CSPI_ONLY__)
    else
    {
        //TcpalIrqDisable();    
          TcbdDebug(DEBUG_ERROR,"tdmb_tcc3170_power_off TchalPowerDownDevice\n");
        TchalPowerDownDevice();
    }
#endif //__CSPI_ONLY__

    wake_unlock(&TdmbCtrlInfo.wake_lock);
    TdmbCtrlInfo.pwr_state = 0;

	return TRUE;
}


int tdmb_tcc3170_select_antenna(unsigned int sel)
{
	return FALSE;
}

#if defined(__I2C_STS__)
static int  broadcast_tcc3170_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	struct broadcast_device_platform_data *pdata;

	memset((void*)&TdmbCtrlInfo, 0x00, sizeof(struct broadcast_tcc3170_ctrl_data));

	TcbdDebug(DEBUG_ERROR,"broadcast_tcc3170_i2c_probe( ) client:0x%X\n", (unsigned int)client);

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TcbdDebug(DEBUG_ERROR, "tdmb_tcc3170_i2c_probe: need I2C_FUNC_I2C\n");
		
		rc = -ENODEV;

		return rc;
	}

	TdmbCtrlInfo.pclient = client;
	
	i2c_set_clientdata(client, (void*)&TdmbCtrlInfo);

	/* Register power control function */
	TdmbCtrlInfo.pwr_state = 0;
	pdata = (struct broadcast_device_platform_data*)client->dev.platform_data;
	TdmbCtrlInfo.pctrl_fun = pdata;

	if(TdmbCtrlInfo.pctrl_fun && TdmbCtrlInfo.pctrl_fun->dmb_gpio_init)
	{
		TdmbCtrlInfo.pctrl_fun->dmb_gpio_init( );
	}
	else
	{
		TcbdDebug(DEBUG_ERROR,"broadcast_tcc3170_i2c_probe dmb_gpio_init is not called\n");
	}
	
	wake_lock_init(&TdmbCtrlInfo.wake_lock, WAKE_LOCK_SUSPEND, dev_name(&client->dev));

	return OK;
}


static int broadcast_tcc3170_i2c_remove(struct i2c_client* client)
{
	TcbdDebug(DEBUG_INFO,"broadcast_tcc3170_i2c_remove is called\n");
	
	wake_lock_destroy(&TdmbCtrlInfo.wake_lock);
	
	return OK;
}

static const struct i2c_device_id tdmb_tcc3170_id[] = {
	{"tdmb_tcc3170",	0},
	{},
};


MODULE_DEVICE_TABLE(i2c, tdmb_tcc3170_id);

static struct i2c_driver tcc3170_i2c_driver = {
	.probe = broadcast_tcc3170_i2c_probe,
	.remove = broadcast_tcc3170_i2c_remove,
	.id_table = tdmb_tcc3170_id,
	.driver = {
		.name = "tdmb_tcc3170",
		.owner = THIS_MODULE,
	},
};

#elif defined(__CSPI_ONLY__)
static int broadcast_tdmb_spi_probe(struct spi_device *spi_dev)
{
	int rc = 0;

	spi_dev->mode = SPI_MODE_0;
	spi_dev->bits_per_word = 8;
	spi_dev->max_speed_hz = 24000*1000;
	rc = spi_setup(spi_dev);
	
	

        TdmbCtrlInfo.spi_dev = spi_dev;
        TdmbCtrlInfo.pwr_state = 0;
	TcbdDebug(DEBUG_INFO, "spi : 0x%X\n", (unsigned int)spi_dev);
	TchalInit();
#if defined(__TEST_IRQ_REG_ONCE__)
	TcpalRegisterIrqHandler();
#endif

     TcpalIrqDisable();	

    wake_lock_init(&TdmbCtrlInfo.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi_dev->dev));	

	return rc;
}

static int broadcast_tdmb_spi_remove(struct spi_device *spi)
{
	int rc = 0;
	TcbdDebug(DEBUG_INFO, "\n");
#if defined(__TEST_IRQ_REG_ONCE__)
	TcpalUnRegisterIrqHandler();
#endif
	wake_lock_destroy(&TdmbCtrlInfo.wake_lock);
	memset((unsigned char*)&TdmbCtrlInfo, 0x0, sizeof(struct broadcast_tcc3170_ctrl_data));
	return rc;
}
static int broadcast_tdmb_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	int rc = 0;
	TcbdDebug(DEBUG_INFO, "\n");
	return rc;
}
static int broadcast_tdmb_spi_resume(struct spi_device *spi)
{
	int rc = 0;
	TcbdDebug(DEBUG_INFO, "\n");
	return rc;
}

static struct spi_driver broadcast_tdmb_driver = {
	.probe = broadcast_tdmb_spi_probe,
	.remove	= __devexit_p(broadcast_tdmb_spi_remove),
	.suspend = broadcast_tdmb_spi_suspend,
	.resume  = broadcast_tdmb_spi_resume,
	.driver = {
		.name = "tdmb_tcc3170",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
};
#endif //__CSPI_ONLY__

int __devinit broadcast_tdmb_drv_init(void)
{
	int rc;
	TcbdDebug(DEBUG_INFO,"broadcast_tdmb_drv_init\n");

	rc = broadcast_tdmb_drv_start();	
	if (rc) 
	{
		TcbdDebug(DEBUG_ERROR,"broadcast_tdmb_drv_start %s failed to load\n", __func__);
		return rc;
	}
	
#if defined(__I2C_STS__)
	TcpalSetI2cIoFunction();
	rc = i2c_add_driver(&tcc3170_i2c_driver);
#elif defined(__CSPI_ONLY__)
	TcpalSetCspiIoFunction();
	rc =  spi_register_driver(&broadcast_tdmb_driver);
#endif //__CSPI_ONLY__
	TcbdDebug(DEBUG_INFO,"broadcast_add_driver rc = (%d)\n", rc);
	return rc;
}

static void __exit broadcast_tdmb_drv_exit(void)
{
#if defined(__I2C_STS__)
	i2c_del_driver(&tcc3170_i2c_driver);
#elif defined(__CSPI_ONLY__)
	spi_unregister_driver(&broadcast_tdmb_driver);
#endif //__CSPI_ONLY__
}


/* EXPORT_SYMBOL() : when we use external symbol 
which is not included in current module - over kernel 2.6 */
//EXPORT_SYMBOL(broadcast_tdmb_is_on);

module_init(broadcast_tdmb_drv_init);
module_exit(broadcast_tdmb_drv_exit);
MODULE_DESCRIPTION("broadcast_tdmb_drv_init");
MODULE_LICENSE("TELECHIPS");
