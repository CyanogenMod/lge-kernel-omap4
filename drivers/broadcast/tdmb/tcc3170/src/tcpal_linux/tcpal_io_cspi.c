#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_io.h"

#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#define SPICMD_VALID_BITS   36
#define SPICMD_BUFF_LEN      8
#define SPICMD_ACK        0x47

#define SPI_SPEED_HZ      24000*1000 //10000000
#define SPI_BITS_PER_WORD 8

#define DMA_MAX_SIZE    (2048)

#define CSPI_READ  0
#define CSPI_WRITE 1

#define CONTINUOUS_MODE 0
#define FIXED_MODE      1

#if defined(__CSPI_ONLY__)

struct TcpalTcspiData_t 
{
    spinlock_t spin_lock;
    I08U dummyBuffer[DMA_MAX_SIZE+SPICMD_BUFF_LEN*2];
    I08U rwBuffer[DMA_MAX_SIZE+SPICMD_BUFF_LEN*2];
    I08U init_cmd_buf[SPICMD_BUFF_LEN]; /*Set all bit to 1*/
    struct spi_device* spi_dev;
};

static TcbdIo_t *TcbdIos = NULL;
static struct TcpalTcspiData_t TcpalTcspiData;
struct spi_device*	TCC_GET_SPI_DRIVER(void);

static I08U TcpalCalculateCrc7(I08U *data, I32S len)
{
    I16U masking, carry;
    I16U crc;
    I32U i, loop, remain;

    crc = 0x0000;
    loop = len / 8;
    remain = len - loop * 8;

    for (i = 0; i < loop; i++)
    {
        masking = 1 << 8;
        while ((masking >>= 1))
        {
            carry = crc & 0x40;
            crc <<= 1;
            if ((!carry) ^ (!(*data & masking)))
                crc ^= 0x9;
            crc &= 0x7f;
        }
        data++;
    }

    masking = 1 << 8;
    while (remain)
    {
        carry = crc & 0x40;
        crc <<= 1;
        masking >>= 1;
        if ((!carry) ^ (!(*data & masking)))
            crc ^= 0x9;
        crc &= 0x7f;
        remain--;
    }

    return (I08U) crc;
}

static I32S TcpalCspiClose(void)
{
    struct TcpalTcspiData_t *spiData = &TcpalTcspiData;

    //spiData->spi_dev = NULL;
    TcbdDebug(DEBUG_TCPAL_CSPI, "spi_dev :0x%X\n", (unsigned int)spiData->spi_dev);
    return 0;
}

static I32S TcpalCspiOpen(void)
{    
    struct TcpalTcspiData_t *spiData = &TcpalTcspiData;
    memset(&TcpalTcspiData, 0, sizeof(TcpalTcspiData));
    
    spiData->spi_dev = TCC_GET_SPI_DRIVER();
    memset(spiData->init_cmd_buf, 0xFF, SPICMD_BUFF_LEN);

    TcbdDebug(DEBUG_TCPAL_CSPI, "spi_dev :0x%X\n", (unsigned int)spiData->spi_dev);
    return 0;
}

static inline I32S TcpalCspiWriteAndRead(I08U *_buffin, I08U *_buffout, I32U _length)
{
    int ret = 0;
    struct TcpalTcspiData_t *spiData = &TcpalTcspiData;

    struct spi_message msg;
    struct spi_transfer xfer = {
        .tx_buf = _buffin,
        .rx_buf = _buffout,
        .len = _length,
        //.speed_hz = SPI_SPEED_HZ,
        //.bits_per_word = SPI_BITS_PER_WORD,
    };

    if(!spiData->spi_dev || !_length) return -EFAULT;
    if(!_buffin && !_buffout) return -EFAULT;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);

    ret = spi_sync(spiData->spi_dev, &msg);
    //ret = ((unsigned char*)xfer.rx_buf)[0];

    if(ret < 0) 
        return -TCERR_OS_DRIVER_FAIL;

    return 0;
}

static inline I32S TcpalCspiSingle(I08U _writeFlag, I16U _regAddr, I08U *_data)
{
    I32S ret = 0;
    unsigned char buffer[SPICMD_BUFF_LEN];
    unsigned char buffout[SPICMD_BUFF_LEN];
    unsigned char crc;

    buffer[0] =  TcbdIos->chipAddr;           /* start bit(1) + chip_id(7) */
    /* mode(1) + rw(1) + fix(1) + addr(5) */
    buffer[1] = 0 << 7 | _writeFlag << 6 | 1 << 5 | ((_regAddr & 0x7c0) >> 6);
    buffer[2] = (_regAddr & 0x03f) << 2 | 0x0; /* addr(6bit) + NULL(2bit) */

    if (_writeFlag)  
        buffer[3] = _data[0];     /* write */
    else 
        buffer[3] = 0x0;          /* null(8) */

    buffer[4] = 0x00;

    crc = TcpalCalculateCrc7(buffer, 36);
    buffer[4] = 0x00 | ((crc & 0x7f) >> 3);   /* null(4) + crc(4) */
    buffer[5] = ((crc & 0x07) << 5) | 0x0f;   /* crc(3) + end bit(5) */
    buffer[6] = 0xff;
    buffer[7] = 0xff;

    ret = TcpalCspiWriteAndRead(buffer, buffout, SPICMD_BUFF_LEN);
    if(ret < 0) return ret;

    if (buffout[7] != SPICMD_ACK) {                 /* ack */
        TcbdDebug(DEBUG_TCPAL_CSPI, "# Single %s ACK error ChipAddr:0x%X, regAddr:0x%X\n", 
            _writeFlag ? "Write" : "Read", (unsigned int)TcbdIos->chipAddr, _regAddr);
        TcbdDebug(DEBUG_TCPAL_CSPI, "# in  [%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]//[%02x]\n",
                buffer[0], buffer[1], buffer[2], buffer[3],
                buffer[4], buffer[5], buffer[6], buffer[7], crc);
        TcbdDebug(DEBUG_TCPAL_CSPI, "# out [%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]\n",
                buffout[0], buffout[1], buffout[2], buffout[3],
                buffout[4], buffout[5], buffout[6], buffout[7]);
                
        return (-TCERR_ACK_FAIL);
    }

    if (_writeFlag == 0)
        *_data = buffout[6];

    return 0;
}

static inline I32S TcpalCspiMulti(
        I08U _writeFlag, I16U _regAddr, I08U *_data, I32S _size, I08U _fixedMode)
{
    //unsigned int crc32_calc, crc32_src;
    I32S ret = 0;
    struct TcpalTcspiData_t *spiData = &TcpalTcspiData;
    unsigned char crc;
    unsigned char *buffer;
    unsigned char *buffout;

    if(_writeFlag == 0){
        buffer = spiData->dummyBuffer;
        buffout = spiData->rwBuffer;
    } else {
        TcpalMemoryCopy(spiData->rwBuffer+SPICMD_BUFF_LEN, _data, _size);
        buffer = spiData->rwBuffer;
        buffout = spiData->dummyBuffer;
    }
    TcpalMemorySet(buffer+SPICMD_BUFF_LEN+_size, 0xFF, SPICMD_BUFF_LEN);

    if (_size > DMA_MAX_SIZE)
        return (-TCERR_INVALID_ARG);

    _size--; /* MAX 16KB (Output buffer max size 7KB) (LENGTH + 1 Byte) */

    /* start bit(1) + chip_id(7) */
    buffer[0] = TcbdIos->chipAddr;
    /* mode(1) + rw(1) + fix(1) + addr(5) */
    buffer[1] = 1 << 7 | _writeFlag << 6 | _fixedMode << 5 | ((_regAddr & 0x7c0) >> 6);
    /* addr(6bit) + length(2bit) */
    buffer[2] = (_regAddr & 0x03f) << 2 | ((_size & 0x3000) >> 12);
    /* length(8bit) */
    buffer[3] = (_size & 0xff0) >> 4;

    buffer[4] = (_size & 0xf) << 4;
    crc = TcpalCalculateCrc7(buffer, 36);
    /* length(4) + crc(4) */
    buffer[4] = ((_size & 0xf) << 4) | ((crc & 0x7f) >> 3);
    /* crc(3) + end bit(5) */
    buffer[5] = ((crc & 0x07) << 5) | 0x0f;
    buffer[6] = 0xff;
    buffer[7] = 0xff;

    _size++;

    ret =  TcpalCspiWriteAndRead(buffer, buffout, _size+SPICMD_BUFF_LEN*2);
    if(ret < 0) return ret;

    if (buffout[7] != SPICMD_ACK)	{    /* ack */
        TcbdDebug(DEBUG_TCPAL_CSPI, "# Burst %s ACK error, ChipAddr:0x%X, RegAddr:0x%X, size:%d, mode:%d\n", 
                _writeFlag ? "Write" : "Read", (unsigned int)TcbdIos->chipAddr, _regAddr, (int)_size, _fixedMode);
        TcbdDebug(DEBUG_TCPAL_CSPI, "# [%x][%x][%x][%x][%x][%x][%x][%x]//[%x]\n",
                buffer[0], buffer[1], buffer[2], buffer[3],
                buffer[4], buffer[5], buffer[6], buffer[7], crc);
        return (-TCERR_ACK_FAIL);
    }

    if (_writeFlag == 0) {
        memcpy(_data, buffout + SPICMD_BUFF_LEN, _size);
    } 

    return 0;
}

static inline I32S TcpalCspiRegReadEx(I08U _regAddr, I08U *_data, I32S _size, I08U mode)
{
    I32U i;
    I32U cmax, cremain;
    I32S ret;

    cmax = (_size / DMA_MAX_SIZE);
    cremain = (_size % DMA_MAX_SIZE);

    for (i = 0; i < cmax; i++) 
    {
        ret = TcpalCspiMulti(CSPI_READ, _regAddr, &_data[i * DMA_MAX_SIZE], DMA_MAX_SIZE, mode);
        if(ret < 0) return ret;
    }

    if (cremain != 0) 
    {
        ret = TcpalCspiMulti(CSPI_READ, _regAddr, &_data[i * DMA_MAX_SIZE], cremain, mode);
        if(ret < 0) return ret;
    }
    return 0;
}

static inline I32S TcpalCspiRegWriteEx(I08U _regAddr, I08U *_data, I32S _size, I08U mode)
{
    I32U i;
    I32U cmax, cremain;
    I32S ret;

    cmax = (_size / DMA_MAX_SIZE);
    cremain = (_size % DMA_MAX_SIZE);

    for (i = 0; i < cmax; i++) 
    {
        ret = TcpalCspiMulti(CSPI_WRITE, _regAddr, &_data[i * DMA_MAX_SIZE], DMA_MAX_SIZE, mode);
        if(ret < 0) return ret;
    }

    if (cremain) {
        ret = TcpalCspiMulti(CSPI_WRITE, _regAddr, &_data[i * DMA_MAX_SIZE], cremain, mode);
        if(ret < 0) return ret;
    }

    return 0;
}

static I32S TcpalCspiRegRead(I08U _regAddr, I08U *_data) 
{
    return TcpalCspiSingle(CSPI_READ, _regAddr, _data);
}

static I32S TcpalCspiRegWrite(I08U _regAddr, I08U _data)
{
    return TcpalCspiSingle(CSPI_WRITE, _regAddr, &_data);
}

static I32S TcpalCspiRegReadExCon (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalCspiRegReadEx(_regAddr, _data, _size, CONTINUOUS_MODE);
}

static I32S TcpalCspiRegWriteExCon (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalCspiRegWriteEx(_regAddr, _data, _size, CONTINUOUS_MODE);
}

static I32S TcpalCspiRegReadExFix (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalCspiRegReadEx(_regAddr, _data, _size, FIXED_MODE);
}

static I32S TcpalCspiRegWriteExFix (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalCspiRegWriteEx(_regAddr, _data, _size, FIXED_MODE);
}

void TcpalSetCspiIoFunction(void)
{
    TcbdIos = TcbdGetIoStruct();

    TcbdIos->Open = TcpalCspiOpen;
    TcbdIos->Close = TcpalCspiClose;
    TcbdIos->RegWrite = TcpalCspiRegWrite;
    TcbdIos->RegRead  = TcpalCspiRegRead;
    TcbdIos->RegWriteExCon = TcpalCspiRegWriteExCon;
    TcbdIos->RegReadExCon = TcpalCspiRegReadExCon;
    TcbdIos->RegWriteExFix = TcpalCspiRegWriteExFix;
    TcbdIos->RegReadExFix = TcpalCspiRegReadExFix;
}

#endif //__CSPI_ONLY__
