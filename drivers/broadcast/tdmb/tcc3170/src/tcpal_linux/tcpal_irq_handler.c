
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#include "broadcast_tcc3170.h"

#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"

#include "tcbd_stream_parser.h"
#include "tcbd_diagnosis.h"
#include "tcbd_hal.h"

#define __WORKQUEUE__

typedef struct _TcbdIrqData_t
{
    struct work_struct work;
    struct workqueue_struct *workQueue;
    TcbdDevice_t *device;
	TcpalSem_t lock;
    int tcbd_irq;
	int isIrqEnable;

    TcpalTime_t startTick;
} TcbdIrqData_t;

static TcbdIrqData_t TcbdIrqData;
static I08U StreamBuffer[TCBD_MAX_THRESHOLD*2];

extern int broadcast_drv_if_push_msc_data(unsigned char *buffer_ptr, unsigned int buffer_size);
extern int tunerbb_drv_tcc3170_put_fic(unsigned char * buffer, unsigned int buffer_size);

extern struct spi_device*	TCC_GET_SPI_DRIVER(void);

inline static void TcbdParseStream(TcbdIrqData_t * irqData)
{
    I32S size = TCBD_MAX_THRESHOLD*2, ret = 0;
    I08S irqStatus;
    I08S irqError;
    TcbdDevice_t *device = irqData->device;

	TcpalSemaphoreLock(&TcbdIrqData.lock);
    ret = TcbdReadIrqStatus(device, &irqStatus, &irqError);
    ret |= TcbdClearIrq(device, irqStatus);
    ret |= TcbdReadStream(device, StreamBuffer, &size);
    if(ret == 0 && !irqError)
    {
        TcbdParser(StreamBuffer, size);
    }
    else
    {
        TcbdDebug(DEBUG_ERROR,
        	"================================================================================\n"
        	"### buffer is full, skip the data (status=0x%02X, error=0x%02X, ret:%d, time%d)  ###\n"
        	"================================================================================\n",
        	irqStatus, irqError, ret, (int)TcpalGetTimeIntervalMs(irqData->startTick));
	TcbdInitStreamDataConfig(device, ENABLE_CMD_FIFO, device->lastSelectedBuffer, device->intrThreshold);
        TcbdInitParser(NULL);
    }
	ret = TcbdReadIrqStatus(device, &irqStatus, &irqError);
	if(irqError)
	{
		TcbdDebug(DEBUG_ERROR,
		   "================================================================================\n"
		   "### TcbdReadIrqStatus  buffer is full, skip the data (status=0x%02X, error=0x%02X, ret:%d, time%d)  ###\n"
		   "================================================================================\n",
			   irqStatus, irqError, (int)ret, (int)TcpalGetTimeIntervalMs(irqData->startTick));
		TcbdInitStreamDataConfig(device, ENABLE_CMD_FIFO, device->lastSelectedBuffer, device->intrThreshold);
		TcbdInitParser(NULL);
	}
	
	TcpalSemaphoreUnLock(&TcbdIrqData.lock);
}

#if defined(__WORKQUEUE__)
static void TcbdStreamParsingWork(struct work_struct *_param)
{
    struct _TcbdIrqData_t * irqData = container_of(_param, struct _TcbdIrqData_t, work);

	TcbdIrqData.isIrqEnable = 1;
    TcbdParseStream(irqData);
	TcbdIrqData.isIrqEnable = 0;
    enable_irq(irqData->tcbd_irq);
}
#endif //__WORKQUEUE__

static irqreturn_t TcbdIrqHandler(int _irq, void *_param)
{
#if defined(__WORKQUEUE__)
    disable_irq_nosync(TcbdIrqData.tcbd_irq);
	if(TcbdIrqData.isIrqEnable)
	{
		TcbdDebug(DEBUG_ERROR,"########### broadcast_tdmb_spi_isr ###########"
				      "######### spi read function is so late skip #########\n");		
		return IRQ_HANDLED;		
	}
	TcbdIrqData.startTick = TcpalGetCurrentTimeMs();
    queue_work(TcbdIrqData.workQueue, &TcbdIrqData.work);
#else  //__WORKQUEUE__
    TcbdParseStream(&TcbdIrqData);
#endif //!__WORKQUEUE__

    return IRQ_HANDLED;
}

static I32S TcbdStreamCallback(I08U *_stream, I32S _size, I08U _subchId, I08U _type)
{
    I08U *stream = NULL;
    TDMB_BB_HEADER_TYPE *lgeHeader;
    switch(_type)
    {
        case 0:
            stream = (_stream - sizeof(TDMB_BB_HEADER_TYPE));
            lgeHeader = (TDMB_BB_HEADER_TYPE*)stream;
            lgeHeader->size = _size;
            lgeHeader->subch_id = _subchId;
            broadcast_drv_if_push_msc_data(stream, _size+sizeof(TDMB_BB_HEADER_TYPE));
            break;
        case 1:
			if(TcbdIrqData.device->isRecvFic == 1)
            	tunerbb_drv_tcc3170_put_fic(_stream, _size);
            break;
        case 2:
            TcbdUpdateStatus(_stream, NULL);
            break;
        default:
            break;
    }
    return 0;
}

#if !defined(__TEST_IRQ_REG_ONCE__)
I32S TcpalRegisterIrqHandler(TcbdDevice_t *_device)
{
#else
I32S TcpalRegisterIrqHandler(void)
{
	extern TcbdDevice_t *GetTcbdDevice(void);
	TcbdDevice_t *_device = GetTcbdDevice();
#endif

    struct spi_device *spi = TCC_GET_SPI_DRIVER();
	TcbdIrqData.isIrqEnable = 0;
#if defined(__WORKQUEUE__)
    TcbdIrqData.workQueue = create_singlethread_workqueue("TcbdStreamParsingWork");
    TcbdIrqData.device = _device;
    TcbdIrqData.tcbd_irq = spi->irq;
	TcpalCreateSemaphore(&TcbdIrqData.lock, "IRQ", 1);

    INIT_WORK(&TcbdIrqData.work, TcbdStreamParsingWork);
#endif //__WORKQUEUE__
    TcbdInitParser(TcbdStreamCallback);

    TchalIrqSetup();

    TcbdDebug(DEBUG_ERROR,"irq:%d\n", TcbdIrqData.tcbd_irq);
    return  request_irq(TcbdIrqData.tcbd_irq, TcbdIrqHandler,
        IRQF_DISABLED | IRQF_TRIGGER_FALLING, "tc317x_stream", &TcbdIrqData);
	
}

I32S TcpalUnRegisterIrqHandler(void)
{
    disable_irq(TcbdIrqData.tcbd_irq);
	flush_workqueue(TcbdIrqData.workQueue);
    destroy_workqueue(TcbdIrqData.workQueue);
	TcpalDeleteSemaphore(&TcbdIrqData.lock);
    return 0;
}

I32S TcpalIrqEnable(void)
{
    enable_irq(TcbdIrqData.tcbd_irq);
	TcbdDebug(DEBUG_ERROR,"%d\n", TcbdIrqData.tcbd_irq);
    return 0;
}

I32S TcpalIrqDisable(void)
{
    disable_irq(TcbdIrqData.tcbd_irq);
	TcbdDebug(DEBUG_ERROR,"%d\n", TcbdIrqData.tcbd_irq);	
    return 0;
}
