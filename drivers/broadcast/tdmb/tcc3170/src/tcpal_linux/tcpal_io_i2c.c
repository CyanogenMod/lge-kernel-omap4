
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
#include <linux/i2c.h>

#include <asm/io.h>
#include <asm/mach-types.h>

#define MAX_I2C_BURST   (1024*8)
#define Bit7          0x00000080

#define I2C_BUS                1
#define I2C_ADDR       (0xA0>>1)

#if defined(__I2C_STS__)

static unsigned char I2cBuffer[MAX_I2C_BURST+4];
static struct i2c_client *TcpalI2cClient = NULL;

extern struct i2c_client*	TCC_GET_I2C_DRIVER(void);

inline static I32S tdmb_tcc3170_i2c_write(I08U* txdata, I32S length)
{
	I32S rc;
	struct i2c_msg msg = 
	{
		TcpalI2cClient->addr,
		0,
		length,
		(unsigned char*)txdata
	};

	rc = i2c_transfer(TcpalI2cClient->adapter, &msg, 1);
	if(rc < 0)
	{
		printk("tdmb_tcc3170_i2c_write fail rc = (%d) addr =(0x%X) data=0x%02x\n", 
			(int)rc, (unsigned int)TcpalI2cClient->addr, (unsigned int)txdata[1]);

		return 0;
	}
	
	return 1;
}


inline static int tdmb_tcc3170_i2c_read( I08U raddr, I08U *rxdata, I32S length)
{
	int rc;

	struct i2c_msg msgs[2] = 
	{
		{
			.addr	= TcpalI2cClient->addr,
			.flags = 0,
			.len   = 1,
			.buf   = (unsigned char*)&raddr,
		},
		{
			.addr	= TcpalI2cClient->addr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = (unsigned char*)rxdata,
		},
	};

	rc = i2c_transfer(TcpalI2cClient->adapter, msgs, 2);
	if(rc < 0)
	{
		printk("tdmb_tcc3170_i2c_read failed! rc =(%d),%x \n", (int)rc, (unsigned int)TcpalI2cClient->addr);
		
		return 0;
	}
	
	return 1;
};

static I32S TcpalI2cClose(void)
{
    TcbdDebug(DEBUG_TCPAL_I2C, "TcpalI2cClient :0x%X\n", (unsigned int)TcpalI2cClient);
    return 0;
}

static I32S TcpalI2cOpen(void)
{
    TcpalI2cClient = TCC_GET_I2C_DRIVER();
    if(TcpalI2cClient == NULL) 
    {
        TcbdDebug(DEBUG_TCPAL_I2C, "i2c driver is not registered!!\n");
        return -TCERR_OS_DRIVER_FAIL;        
    }
    TcbdDebug(DEBUG_TCPAL_I2C, "TcpalI2cClient : 0x%X\n", (unsigned int)TcpalI2cClient);
    return 0;
}

static I32S TcpalI2cRegRead(I08U _regAddr, I08U *_data) 
{
    I32S ret = 0;

    if(!TcpalI2cClient) 
        return -TCERR_OS_DRIVER_FAIL;

    ret = tdmb_tcc3170_i2c_read(_regAddr, _data, 1);
    if(ret == 0) 
    {
        ret = tdmb_tcc3170_i2c_read(_regAddr, _data, 1);
        if(ret == 0)
        {
            TcbdDebug(DEBUG_TCPAL_I2C, "I2C read error %d\n", (int)ret); 
            return -TCERR_OS_DRIVER_FAIL;
        }
    }
    return 0;
}

static I32S TcpalI2cRegWrite(I08U _regAddr, I08U _data)
{
    I32S ret = 0;
    unsigned char buf[2];

    if(!TcpalI2cClient) 
        return -TCERR_OS_DRIVER_FAIL;

    buf[0] = (unsigned char)(_regAddr);
    buf[1] = (unsigned char)(_data);

    ret = tdmb_tcc3170_i2c_write(buf, 2);
    if(ret == 0) 
    {
        ret = tdmb_tcc3170_i2c_write(buf, 2);
        if(ret == 0) 
        {
            TcbdDebug(DEBUG_TCPAL_I2C, "tdmb_tcc3170_i2c_write failed %d \n", (unsigned int)ret);
            return -TCERR_OS_DRIVER_FAIL;
        }
    }
    return 0;
}

inline static I32S TcpalI2cRegReadEx(I08U _regAddr, I08U *_data, I32S _size)
{
    I32S ret = 0;

    if(!TcpalI2cClient) 
        return -TCERR_OS_DRIVER_FAIL;

    ret = tdmb_tcc3170_i2c_read(_regAddr, _data, _size);
    if(ret == 0)
    {
        TcbdDebug(DEBUG_TCPAL_I2C, "TcpalI2cRegReadEx failed\n");
        return -TCERR_OS_DRIVER_FAIL;
    }

    return 0;
}

inline static I32S TcpalI2cRegWriteEx(I08U _regAddr, I08U *_data, I32S _size)
{
    I32S ret = 0;

    if(!TcpalI2cClient) 
        return -TCERR_OS_DRIVER_FAIL;

    I2cBuffer[0] = _regAddr;		
    TcpalMemoryCopy(I2cBuffer+1, _data, _size);
    ret = tdmb_tcc3170_i2c_write(I2cBuffer, _size+1);
    if(ret == 0)
        return -TCERR_OS_DRIVER_FAIL;
	
    return 0;
}

static I32S TcpalI2cRegReadExFix (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalI2cRegReadEx(_regAddr|Bit7, _data, _size);
}

static I32S TcpalI2cRegWriteExFix (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalI2cRegWriteEx(_regAddr|Bit7, _data, _size);
}

static I32S TcpalI2cRegReadExCon (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalI2cRegReadEx(_regAddr, _data, _size);
}

static I32S TcpalI2cRegWriteExCon (I08U _regAddr, I08U *_data, I32S _size)
{
    return TcpalI2cRegWriteEx(_regAddr, _data, _size);
}

void TcpalSetI2cIoFunction(void)
{
    TcbdIo_t *TcbdIos = TcbdGetIoStruct();

    TcbdIos->Open = TcpalI2cOpen;
    TcbdIos->Close = TcpalI2cClose;
    TcbdIos->RegWrite = TcpalI2cRegWrite;
    TcbdIos->RegRead  = TcpalI2cRegRead;
    TcbdIos->RegWriteExCon = TcpalI2cRegWriteExCon;
    TcbdIos->RegReadExCon = TcpalI2cRegReadExCon;
    TcbdIos->RegWriteExFix = TcpalI2cRegWriteExFix;
    TcbdIos->RegReadExFix = TcpalI2cRegReadExFix;
}

#endif //__I2C_STS__
