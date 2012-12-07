
#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_io.h"

#define TCBD_PERI_SPI_READY_TIME 0x00
#define TCBD_PERI_SPI_WAIT_TIME 0x00

static inline void TcbdPeriSpiSlave(I08U *_periMode, I08U _interfaceSpeed)
{
    _periMode[0] = TCBD_PERI_SPI_MOTOROLA_SSP | 
                   TCBD_PERI_SPI_SLAVE | 
                   TCBD_PERI_SPI_SIZE32BIT | 
                   _interfaceSpeed;
    _periMode[1] = TCBD_PERI_SPI_CLKINIT_LOW | 
                   TCBD_PERI_SPI_CLKPOL_POS | 
                   TCBD_PERI_SPI_BITMSB1 | 
                   (TCBD_PERI_SPI_READY_TIME<<2)| 
                   TCBD_PERI_SPI_WAIT_TIME;
}

static inline void TcbdPeriSpiMaster(I08U *_periMode, I08U _interfaceSpeed)
{
    _periMode[0] = TCBD_PERI_SPI_MOTOROLA_SSP | 
                   TCBD_PERI_SPI_MASTER | 
                   TCBD_PERI_SPI_SIZE32BIT | 
                   _interfaceSpeed;
    _periMode[1] = TCBD_PERI_SPI_CLKINIT_LOW | 
                   TCBD_PERI_SPI_CLKPOL_POS | 
                   TCBD_PERI_SPI_BITMSB1 | 
                   (TCBD_PERI_SPI_READY_TIME<<2) | 
                   TCBD_PERI_SPI_WAIT_TIME;
}

#define TCBD_PERI_TS_STREAM_WAIT_TIME 0x02
#define TCBD_PERI_TS_MSM_SYNC_HIGH_TIME 0x00
#define TCBD_PERI_TS_MSM_SYNC_LOW_TIME 0x00
static inline void TcbdPeriSts(I08U *_periMode, I08U _interfaceSpeed)
{
    _periMode[0] = TCBD_PERI_TS_NORMAL_SLAVE | 
                   TCBD_PERI_TS_STS | 
                   TCBD_PERI_TS_FASTON | 
                   _interfaceSpeed; 

    _periMode[1] = TCBD_PERI_TS_CLKPHASE_POS /*TCBD_PERI_TS_CLKPHASE_NEG*/ | 
                   TCBD_PERI_TS_SYNC_ACTIVEHIGH | 
                   TCBD_PERI_TS_ENPOL_ACTIVEHIGH |
                   TCBD_PERI_TS_STREAM_WAIT_ON | 
                   TCBD_PERI_TS_STREAM_WAIT_TIME;
    _periMode[2] = TCBD_PERI_TS_BITMSB | 
                   (TCBD_PERI_TS_MSM_SYNC_HIGH_TIME<<3) | 
                   TCBD_PERI_TS_MSM_SYNC_LOW_TIME;
    _periMode[3] = TCBD_PERI_TS_ERR_SIG_OFF;
} 

static inline void TcbdPeriPts(I08U *_periMode, I08U _interfaceSpeed)
{
    _periMode[0] = TCBD_PERI_TS_NORMAL_SLAVE | 
                   TCBD_PERI_TS_PTS | 
                   TCBD_PERI_TS_FASTON | 
                   TCBD_PERI_TS_TSCLKMASKON | 
                   _interfaceSpeed; 
    _periMode[1] = TCBD_PERI_TS_CLKPHASE_POS | 
                   TCBD_PERI_TS_SYNC_ACTIVEHIGH | 
                   TCBD_PERI_TS_ENPOL_ACTIVEHIGH | 
                   TCBD_PERI_TS_STREAM_WAIT_ON | 
                   TCBD_PERI_TS_STREAM_WAIT_TIME;
    _periMode[2] = (TCBD_PERI_TS_MSM_SYNC_HIGH_TIME<<3) | 
                   TCBD_PERI_TS_MSM_SYNC_LOW_TIME;
    _periMode[3] = TCBD_PERI_TS_ERR_SIG_OFF;
}

static inline void TcbdPeriHpi(I08U *_periMode)
{
    _periMode[0] = TCBD_PERI_HPI_INTEL;
}

static inline I08U TcbdCalcClock(TcbdDevice_t *_device, I32S _minKhz, I32S _maxKhz)
{
    I32U minDlr = 0, maxDlr = 0, minv = -1, maxv = -1;

    minDlr = (_device->mainClock / (2*_minKhz)) - 1;
    maxDlr = (_device->mainClock / (2*_maxKhz)) - 1;

    if(minDlr >= 0 && minDlr < 8)
        minv = minDlr;
    else if(minDlr >= 8)
        minv = 7;

    if(maxDlr >= 0 && maxDlr < 8)
        maxv = maxDlr;
    else if(maxDlr >= 8)
        maxv = 7;

    if(minv == -1 && maxv == -1)
    {
        TcbdDebug(DEBUG_DRV_PERI, " # Can't find DLR value, DLR will set to zero\n");
        return 0;
    }

    if(maxv == -1)
        maxv = minv;
    else if(minv == -1)
        minv = maxv;

    return minv;
}

I32S TcbdSelectPeri(TcbdDevice_t *_device, TcbdPeri_t _periType)
{
    I32S ret = 0;
    I08U periMode[4] = {0, };
    I08U interfaceSpeed = TcbdCalcClock(_device, 3000, 10000);

    TcbdDebug(DEBUG_DRV_PERI, "peri type:%d, clock div:%d\n", (I32S)(_periType), interfaceSpeed);
    switch(_periType)
    {
        case PeriTypeSpiSlave:
            TcbdPeriSpiSlave(periMode, interfaceSpeed);
            ret |= TcbdRegWriteExCon(_device, TCBD_PERI_MODE0, periMode, 2);
            ret |= TcbdRegWrite(_device, TCBD_PERI_CTRL, 0x90);
            break;
        case PeriTypeSpiMaster:
            TcbdPeriSpiMaster(periMode, interfaceSpeed);
            ret |= TcbdRegWriteExCon(_device, TCBD_PERI_MODE0, periMode, 2);
            ret |= TcbdRegWrite(_device, TCBD_PERI_CTRL, 0x90);
            break;
        case PeriTypePts:
            TcbdPeriPts(periMode, interfaceSpeed);
            ret |= TcbdRegWriteExCon(_device, TCBD_PERI_MODE0, periMode, 4);
            break;
        case PeriTypeSts:
            TcbdPeriSts(periMode, interfaceSpeed);
            ret |= TcbdRegWriteExCon(_device, TCBD_PERI_MODE0, periMode, 4);
            break;
        case PeriTypeHpi:
            TcbdPeriHpi(periMode);
            ret |= TcbdRegWrite(_device, TCBD_PERI_MODE0, periMode[0]);
            break;
        case PeriTypeSpiOnly:
            ret |= TcbdRegWrite(_device, TCBD_PERI_CTRL, 0x80);
        default:
            break; 
    }
    _device->periType = _periType;
    return ret;
}

I32S TcbdInitGpioForPeri(TcbdDevice_t *_device, TcbdPeri_t _periType)
{
    I08U cfgValue[10] = {0, };
    //I08U drvStrength[] = {0xFF, 0xFF};

    TcbdDebug(DEBUG_DRV_PERI, "peri type:%d\n", (I32S)(_periType));
    TcpalMemorySet(cfgValue, 0, sizeof(cfgValue));

    switch(_periType)
    {
        case PeriTypeSpiSlave:
        case PeriTypePts:
        case PeriTypeHpi:
        case PeriTypeSpiOnly:
            return 0;

        case PeriTypeSpiMaster:
            cfgValue[0] = 0x0E;
            cfgValue[1] = 0x34;
            break;
        case PeriTypeSts:
            cfgValue[0] = 0x0E;
            cfgValue[1] = 0x3C;
            break;
        default:
            break;
    }

    TcbdRegWriteExCon(_device, TCBD_GPIO_ALT, cfgValue, 10);
    //TcbdRegWriteExCon(_device, TCBD_GPIO_DRV, drvStrength, 2);

    return 0;
}

I32S TcbdInitGpioForSlave(TcbdDevice_t *_device)
{
    I08U cfgValue[10] = {0, };

    cfgValue[0] = 0x01;
    cfgValue[1] = 0xC0;
    return TcbdRegWriteExCon(_device, TCBD_GPIO_ALT, cfgValue, 10);
}

I32S TcbdEnablePeri(TcbdDevice_t *_device)
{
    I08U data = TCBD_PERI_EN | TCBD_PERI_INIT_AUTOCLR;

    switch(_device->periType)
    {
        case PeriTypeSpiOnly:
            data = 0x80; break;
        case PeriTypeSpiSlave:
        case PeriTypeSpiMaster:
            data |= TCBD_PERI_SEL_SPI | TCBD_PERI_HEADER_ON; break;
        case PeriTypePts:
        case PeriTypeSts:
            data |= TCBD_PERI_SEL_TS | TCBD_PERI_HEADER_ON; break;
        case PeriTypeHpi:
            data |= TCBD_PERI_SEL_HPI | TCBD_PERI_HEADER_ON; break;
        default: break;
    }

    return TcbdRegWrite(_device, TCBD_PERI_CTRL, data);
}

I32S TcbdDisablePeri(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I08U data;

    ret |= TcbdRegRead(_device, TCBD_PERI_CTRL, &data);
    data &= ~TCBD_PERI_EN;
    ret |= TcbdRegWrite(_device, TCBD_PERI_CTRL, data);

    return ret;
}
