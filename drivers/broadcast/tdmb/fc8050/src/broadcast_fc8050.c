#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h> 		/* wake_lock, unlock */

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_fc8050.h"
#include "../inc/fci_types.h"
#include "../inc/bbm.h"

#include <linux/mfd/pmic8058.h>	//I-pjt

/* external function */
extern int broadcast_drv_if_isr(void);
extern void fc8050_isr_interruptclear(void);
extern void fc8050_isr_control(fci_u8 onoff);

/* proto type declare */
static int broadcast_tdmb_fc8050_probe(struct spi_device *spi);
static int broadcast_tdmb_fc8050_remove(struct spi_device *spi);
static int broadcast_tdmb_fc8050_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_fc8050_resume(struct spi_device *spi);

//I-pjt GPIO
#define DMB_EN 			98 //GPIO 102
#define DMB_INT_N 		75 //GPIO 107
#define DMB_RESET_N 	94 //GPIO 101
//#define DMB_ANT_SEL_P	11 //GPIO 11
//#define DMB_ANT_SEL_N	12 //GPIO 12
//I-pjt for ANT. swtiching
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)

#define DMB_USE_WORKQUEUE
/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
static uint32 mdelay_in_flg = 0;
struct TDMB_FC8050_CTRL
{
	boolean 					TdmbPowerOnState;
	struct i2c_client*			pClient;
	struct spi_device* 			pSpiDevice;
	struct work_struct 			spi_work;
	struct workqueue_struct* 	spi_wq;
	struct mutex				mutex;
	struct wake_lock 			wake_lock;	/* wake_lock,wake_unlock */
	//test
	boolean 					spi_irq_status;
};

//static broadcast_pwr_func pwr_func;

static struct TDMB_FC8050_CTRL TdmbCtrlInfo;

struct i2c_client* INC_GET_I2C_DRIVER(void)
{
	return TdmbCtrlInfo.pClient;
}

struct spi_device *tdmb_fc8050_get_spi_device(void)
{
	return TdmbCtrlInfo.pSpiDevice;
}

void LGD_RW_TEST(void);


void tdmb_fc8050_set_userstop(void)
{
	user_stop_flg = ((mdelay_in_flg == 1)? 1: 0 );
}

int tdmb_fc8050_mdelay(int32 ms)
{
	int32	wait_loop =0;
	int32	wait_ms = ms;
	int		rc = 1;  /* 0 : false, 1 : ture */

	if(ms > 100)
	{
		wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
		wait_ms = 100;
	}

	mdelay_in_flg = 1;

	do
	{
		msleep(wait_ms);
		if(user_stop_flg == 1)
		{
			printk("~~~~~~~~ Ustop flag is set so return false ~~~~~~~~\n");
			rc = 0;
			break;
		}
	}while((--wait_loop) > 0);

	mdelay_in_flg = 0;
	user_stop_flg = 0;

	return rc;
}

void tdmb_fc8050_Must_mdelay(int32 ms)
{
	msleep(ms);
}

int tdmb_fc8050_tdmb_is_on(void)
{
	return (int)TdmbCtrlInfo.TdmbPowerOnState;
}

/* EXPORT_SYMBOL() : when we use external symbol 
which is not included in current module - over kernel 2.6 */
//EXPORT_SYMBOL(tdmb_fc8050_tdmb_is_on);


int tdmb_fc8050_power_on(void)
{
	printk("tdmb_fc8050_power_on \n");
	if ( TdmbCtrlInfo.TdmbPowerOnState == FALSE )
	{
		wake_lock(&TdmbCtrlInfo.wake_lock);

//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 0);
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 1);
		
		gpio_direction_input(DMB_INT_N);
		gpio_direction_output(DMB_RESET_N, false);
		gpio_direction_output(DMB_EN, true);
		gpio_set_value(DMB_EN, 1);
		gpio_set_value(DMB_RESET_N, 1);
		udelay(1000); //500us
		udelay(1000);
		udelay(1000);
		gpio_set_value(DMB_RESET_N, 0);
		udelay(5); //500us
		gpio_set_value(DMB_RESET_N, 1);
		tdmb_fc8050_interrupt_free();
		TdmbCtrlInfo.TdmbPowerOnState = TRUE;

		printk("tdmb_fc8050_power_on OK\n");
		
	}
	else
	{
		printk("tdmb_fc8050_power_on the power already turn on \n");
	}

	printk("tdmb_fc8050_power_on completed \n");
	
	return TRUE;
}

int tdmb_fc8050_power_off(void)
{
	if ( TdmbCtrlInfo.TdmbPowerOnState == TRUE )
	{
		tdmb_fc8050_interrupt_lock();
		TdmbCtrlInfo.TdmbPowerOnState = FALSE;
		gpio_set_value(DMB_RESET_N, 0);
		gpio_set_value(DMB_EN, 0);
		gpio_direction_output(DMB_INT_N, false);
		gpio_set_value(DMB_INT_N, 0);		

//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);	// for ESD TEST
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);	
		wake_unlock(&TdmbCtrlInfo.wake_lock);
	}
	else
	{
		printk("tdmb_fc8050_power_on the power already turn off \n");
	}	

	printk("tdmb_fc8050_power_off completed \n");

	return TRUE;
}

int tdmb_fc8050_select_antenna(unsigned int sel)
{
	if(LGE_BROADCAST_TDMB_ANT_TYPE_INTENNA == sel)
	{

//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 0);
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 1);
//		printk("ANT is %d ",sel);
	}
	else if(LGE_BROADCAST_TDMB_ANT_TYPE_EARANT == sel)
	{
	
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);
//		gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);
//		printk("ANT is %d ",sel);
	}
	else
	{
		return FALSE;
	}
	return TRUE;
}

static struct spi_driver broadcast_tdmb_driver = {
	.probe = broadcast_tdmb_fc8050_probe,
	.remove	= __devexit_p(broadcast_tdmb_fc8050_remove),
	.suspend = broadcast_tdmb_fc8050_suspend,
	.resume  = broadcast_tdmb_fc8050_resume,
	.driver = {
		.name = "tdmb_fc8050",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
};

void tdmb_fc8050_interrupt_lock(void)
{
	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_fc8050_interrupt_lock fail\n");
	}

	disable_irq(TdmbCtrlInfo.pSpiDevice->irq);
}

void tdmb_fc8050_interrupt_free(void)
{
	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_fc8050_interrupt_free fail\n");
	}

	enable_irq(TdmbCtrlInfo.pSpiDevice->irq);
}

int tdmb_fc8050_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
	int rc;

	struct spi_transfer	t = {
			.tx_buf		= tx_data,
			.rx_buf		= rx_data,
			.len		= tx_length+rx_length,
		};

	struct spi_message	m;	

	if (TdmbCtrlInfo.pSpiDevice == NULL)
	{
		printk("tdmb_fc8050_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
	}

	mutex_lock(&TdmbCtrlInfo.mutex);

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	rc = spi_sync(TdmbCtrlInfo.pSpiDevice, &m);

	if ( rc < 0 )
	{
		printk("tdmb_fc8050_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
	}

	mutex_unlock(&TdmbCtrlInfo.mutex);

	return TRUE;
}

#if defined(DMB_USE_WORKQUEUE)
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct TDMB_FC8050_CTRL* pTdmbInfo;

	pTdmbInfo = (struct TDMB_FC8050_CTRL *)handle;	
	if ( pTdmbInfo && pTdmbInfo->TdmbPowerOnState )
	{
		if (pTdmbInfo->spi_irq_status)
		{			
//			printk("########### broadcast_tdmb_spi_isr ###########\n");
//			printk("######### spi read function is so late skip #########\n");			
			return IRQ_HANDLED;
		}		
//		printk("***** broadcast_tdmb_spi_isr coming *******\n"); 	//LGE_BROADCAST_TEST_I
		queue_work(pTdmbInfo->spi_wq, &pTdmbInfo->spi_work);    
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED; 
}
#else
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
	struct TDMB_FC8050_CTRL* pTdmbInfo;

	pTdmbInfo = (struct TDMB_FC8050_CTRL *)handle;	
	if ( pTdmbInfo && pTdmbInfo->TdmbPowerOnState )
	{
		broadcast_tdmb_read_data();
	}
	else
	{
		printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
	}

	return IRQ_HANDLED; 
}
#endif

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
	struct TDMB_FC8050_CTRL *pTdmbWorkData;

	pTdmbWorkData = container_of(tdmb_work, struct TDMB_FC8050_CTRL, spi_work);
	if ( pTdmbWorkData )
	{
//		printk("broadcast_tdmb_spi_work START\n");	//LGE_BROADCAST_TEST_I
		fc8050_isr_control(0);
		pTdmbWorkData->spi_irq_status = TRUE;
		broadcast_drv_if_isr();
		pTdmbWorkData->spi_irq_status = FALSE;
		fc8050_isr_control(1);
//		printk("broadcast_tdmb_spi_work END\n");	//LGE_BROADCAST_TEST_I
//		printk("broacast_tdmb_spi_work is called handle=0x%x\n", (unsigned int)pTdmbWorkData);	//LGE_BROADCAST_TEST_I
	}
	else
	{
		printk("~~~~~~~broadcast_tdmb_spi_work call but pTdmbworkData is NULL ~~~~~~~\n");
	}
}

static int broadcast_tdmb_fc8050_probe(struct spi_device *spi)
{
	int rc;

/*	struct pm8058_gpio GPIO11_CFG = {
				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.out_strength   = PM_GPIO_STRENGTH_HIGH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
				.vin_sel        = 6,// for ESD TEST
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.output_value   = 0,
				};
	struct pm8058_gpio GPIO12_CFG = {

				.direction      = PM_GPIO_DIR_OUT,
				.pull           = PM_GPIO_PULL_NO,
				.out_strength   = PM_GPIO_STRENGTH_HIGH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
				.vin_sel        = 6,// for ESD TEST
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.output_value   = 0,
				};	
*/	
	TdmbCtrlInfo.TdmbPowerOnState = FALSE;
	
	TdmbCtrlInfo.pSpiDevice 				= spi;
	TdmbCtrlInfo.pSpiDevice->mode 			= SPI_MODE_0;
	TdmbCtrlInfo.pSpiDevice->bits_per_word 	= 8;
	TdmbCtrlInfo.pSpiDevice->max_speed_hz 	= 24000*1000;
	rc = spi_setup(spi);
	printk("broadcast_tdmb_fc8050_probe spi_setup=%d\n", rc);
	BBM_HOSTIF_SELECT(NULL, 1);
#if 0      //fc8050 <-> Host(MSM)  Interface TEST code
/* test */	
{
	uint16 i; 
	uint32 wdata = 0; 
	uint32 ldata = 0; 
	uint32 data = 0;
	uint32 temp = 0;
	
	for(i=0;i<5000;i++)
	{
//		dog_kick();
		BBM_WRITE(NULL, 0x05, i & 0xff);
		BBM_READ(NULL, 0x05, (fci_u8*)&data);
		if((i & 0xff) != data)
			printk("FC8000 byte test (0x%x,0x%x)\n", i & 0xff, data);
	}
	for(i=0;i<5000;i++)
	{
		BBM_WORD_WRITE(NULL, 0x0210, i & 0xffff);
		BBM_WORD_READ(NULL, 0x0210, (fci_u16*)&wdata);
		if((i & 0xffff) != wdata)
			printk("FC8000 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
	}
	for(i=0;i<5000;i++)
	{
		BBM_LONG_WRITE(NULL, 0x0210, i & 0xffffffff);
		BBM_LONG_READ(NULL, 0x0210, (fci_u32*)&ldata);
		if((i & 0xffffffff) != ldata)
			printk("FC8000 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
	}

	data = 0;
	
	for(i=0;i<5000;i++)
	{
	  temp = i&0xff;
		BBM_TUNER_WRITE(NULL, 0x12, 0x01, (fci_u8*)&temp, 0x01);
		BBM_TUNER_READ(NULL, 0x12, 0x01, (fci_u8*)&data, 0x01);
		if((i & 0xff) != data)
			printk("FC8000 tuner test (0x%x,0x%x)\n", i & 0xff, data);
	}
	temp = 0x51;
	BBM_TUNER_WRITE(NULL, 0x12, 0x01, (fci_u8*)&temp, 0x01 );	
}

#endif	/* test */

#if defined(DMB_USE_WORKQUEUE)
	INIT_WORK(&TdmbCtrlInfo.spi_work, broacast_tdmb_spi_work);
	TdmbCtrlInfo.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
	if(TdmbCtrlInfo.spi_wq == NULL){
		printk("Failed to setup tdmb spi workqueue \n");
		return -ENOMEM;
	}
#endif
	rc = request_irq(spi->irq, broadcast_tdmb_spi_isr, IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &TdmbCtrlInfo);
	printk("broadcast_tdmb_fc8050_probe request_irq=%d\n", rc);

	gpio_request(94, "DMB_RESET_N");
	gpio_request(98, "DMB_EN");
	gpio_request(75, "DMB_INT_N");
	gpio_direction_output(DMB_RESET_N, false);
	gpio_direction_output(DMB_EN, false);
	gpio_direction_output(DMB_INT_N, false);

//	pm8058_gpio_config(DMB_ANT_SEL_P-1, &GPIO11_CFG);
//	pm8058_gpio_config(DMB_ANT_SEL_N-1, &GPIO12_CFG);
//	gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);	// for ESD TEST
//	gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);	// for ESD TEST
	
	tdmb_fc8050_interrupt_lock();

#if defined(DMB_USE_WORKQUEUE)
	mutex_init(&TdmbCtrlInfo.mutex);
#endif

	wake_lock_init(&TdmbCtrlInfo.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));		

	return rc;
}

static int broadcast_tdmb_fc8050_remove(struct spi_device *spi)
{
	printk("broadcast_tdmb_fc8050_remove \n");

	if (TdmbCtrlInfo.spi_wq)
	{
		flush_workqueue(TdmbCtrlInfo.spi_wq);
		destroy_workqueue(TdmbCtrlInfo.spi_wq);
	}

	free_irq(spi->irq, &TdmbCtrlInfo);
#if defined(DMB_USE_WORKQUEUE)
	mutex_destroy(&TdmbCtrlInfo.mutex);
#endif
	wake_lock_destroy(&TdmbCtrlInfo.wake_lock);

	memset((unsigned char*)&TdmbCtrlInfo, 0x0, sizeof(struct TDMB_FC8050_CTRL));
	return 0;
}

static int broadcast_tdmb_fc8050_suspend(struct spi_device *spi, pm_message_t mesg)
{
	printk("broadcast_tdmb_fc8050_suspend \n");
	return 0;
}

static int broadcast_tdmb_fc8050_resume(struct spi_device *spi)
{
	printk("broadcast_tdmb_fc8050_resume \n");
	return 0;
}

int __devinit broadcast_tdmb_drv_init(void)
{
	int rc;

	rc = broadcast_tdmb_drv_start();
	printk("broadcast_tdmb_fc8050_probe start %d\n", rc);

	return spi_register_driver(&broadcast_tdmb_driver);
}

static void __exit broadcast_tdmb_drv_exit(void)
{
	spi_unregister_driver(&broadcast_tdmb_driver);
}



module_init(broadcast_tdmb_drv_init);
module_exit(broadcast_tdmb_drv_exit);

/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FC8050 tdmb device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");

