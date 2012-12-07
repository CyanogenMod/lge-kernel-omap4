#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_component.h"
#include "tcbd_drv_io.h"
#include "tcbd_drv_rf.h"

#define MAX_BOOT_TABLE 5
typedef struct _TcbdBootBin_t 
{
    I32U size;
    I32U crc;
    I32U addr;
    I08U *data;
} TcbdBootBin_t;

static I32S TcbdParseBootBin(I08U* _data, I32U _size, TcbdBootBin_t *_bootBin)
{
    I32U idx = 0;
    I32U length;
    I32S tableIdx = 0;

    //coldboot      0x00000001
    //dagu          0x00000002
    //dint          0x00000003
    //rand          0x00000004
    //col_order: 0x00000005

    //sizebyte      4byte
    //data          nbyte

    TcpalMemorySet(_bootBin, 0, sizeof(TcbdBootBin_t)*MAX_BOOT_TABLE);

    // cold boot
    if (_data[idx + 3] != 0x01)
    {
        TcbdDebug(DEBUG_ERROR, "# Coldboot_preset_Error\n");
        return 0;
    }
    idx += 4;
    length = (_data[idx] << 24) + (_data[idx + 1] << 16) + (_data[idx + 2] << 8) + (_data[idx + 3]);
    idx += 4;
    _bootBin[tableIdx].addr = CODE_MEM_BASE;
    _bootBin[tableIdx].data = _data + idx;
    _bootBin[tableIdx].crc = *((I32U*)(_data+idx + length-4));
    _bootBin[tableIdx++].size = length - 4;
    idx += length;
    _size -= (length + 8);

    // dagu
    if (_data[idx + 3] != 0x02)
    {
        TcbdDebug(DEBUG_ERROR, "# Coldboot_preset_Error\n");
        return 0;
    }
    idx += 4;
    length = (_data[idx] << 24) + (_data[idx + 1] << 16) + (_data[idx + 2] << 8) + (_data[idx + 3]);
    idx += 4;
    _bootBin[tableIdx].addr = CODE_TABLEBASE_DAGU;
    _bootBin[tableIdx].data = _data + idx;
    _bootBin[tableIdx++].size = length;
    idx += length;
    _size -= (length + 8);

    // dint
    if (_data[idx + 3] != 0x03)
    {
        TcbdDebug(DEBUG_ERROR, "# Coldboot_preset_Error\n");
        return 0;
    }
    idx += 4;
    length = (_data[idx] << 24) + (_data[idx + 1] << 16) + (_data[idx + 2] << 8) + (_data[idx + 3]);
    idx += 4;
    _bootBin[tableIdx].addr = CODE_TABLEBASE_DINT;
    _bootBin[tableIdx].data = _data + idx;
    _bootBin[tableIdx++].size = length;
    idx += length;
    _size -= (length + 8);

    // rand
    if (_data[idx + 3] != 0x04)
    {
        TcbdDebug(DEBUG_ERROR, "# Coldboot_preset_Error\n");
        return 0;
    }

    idx += 4;
    length = (_data[idx] << 24) + (_data[idx + 1] << 16) + (_data[idx + 2] << 8) + (_data[idx + 3]);
    idx += 4;
    _bootBin[tableIdx].addr = CODE_TABLEBASE_RAND;
    _bootBin[tableIdx].data = _data + idx;
    _bootBin[tableIdx++].size = length;
    idx += length;
    _size -= (length + 8);

    if (_size >= 8) /* cmmb */
    {
        // col_order
        if (_data[idx + 3] != 0x05)
        {
            TcbdDebug(DEBUG_ERROR, "# Coldboot_preset_Error\n");
            return 0;
        }
        idx += 4;
        length = (_data[idx] << 24) + (_data[idx + 1] << 16) + (_data[idx + 2] << 8) + (_data[idx + 3]);
        idx += 4;

        _bootBin[tableIdx].addr = CODE_TABLEBASE_COL_ORDER;
        _bootBin[tableIdx].data = _data + idx;
        _bootBin[tableIdx++].size = length;
        idx += length;
        _size -= (length + 8);
    }

    //TcbdDebug(DEBUG_API_COMMON, " # remain size :%d \n", _size);

    return tableIdx;
}

I32S TcbdInitPll(TcbdDevice_t *_device, I32U *_pllData)
{
   /* 
    * _pllData[0] = PLL_WAIT_TIME
    * _pllData[1] = PLL_P
    * _pllData[2] = PLL_M
    * _pllData[3] = PLL_S
    * _pllData[4] = OSC_LOCK
    * 0x60 0x00, 0x0F, 0x03, 19200
    * */ 
    I32S ret = 0;
    I32U out;
    I08U pllData[2];
    I08U pllR, pllF, pllOD, pllM;
    I32U vco;

    if(!_device || !_pllData) 
        return -TCERR_INVALID_ARG;

    pllR = _pllData[1] + 1;
    pllF = _pllData[2] + 1;
    pllOD = _pllData[3] & 0x3;
    pllM = (_pllData[3] & 0x4) >> 2;

    vco = _pllData[4]*(pllF/pllR);

    if(pllOD) out = vco >> pllOD;
    else      out = vco;
    if(pllM)  out = out>>pllM;

    _device->mainClock = out;
    pllData[0] = _pllData[2] | (pllM<<6) | 0x80;
    pllData[1] = (_pllData[1]<<3) | (pllOD<<1);

    TcpalMemoryCopy(_device->pllData, pllData, 2);
    ret = TcbdRegWrite(_device, TCBD_PLL_7, pllData[1]);
    ret = TcbdRegWrite(_device, TCBD_PLL_6, pllData[0]);
    TcpalSleep(1);
    TcbdDebug(DEBUG_API_COMMON, "mail clock:%d, pll[0]=%02X, pll[1]=%02X\n", 
        out, pllData[0], pllData[1]);

    ret |= TcbdInitBiasKey(_device);

    return ret;
}

I32S TcbdRecvBootCodeVersion(TcbdDevice_t *_device, I32U *_bootVersion)
{
    I32S ret = 0;
    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_READ;
    mail.cmd = MBPARA_SYS_VERSION;
    mail.count = 0;
    ret = TcbdSendMail(_device, &mail);
    if(ret < 0) 
    {
        TcbdDebug(DEBUG_ERROR, "failed to send mail! %d\n", ret);
        return ret;
    }

    ret = TcbdRecvMail(_device, &mail);
    if(ret < 0) 
    {
        TcbdDebug(DEBUG_ERROR, "failed to recv mail! %d\n", ret);
        return ret;
    }

    TcpalMemoryCopy(_bootVersion, &mail.data[0], sizeof(I32U));

    return ret;
}

I32S TcbdDownloadBootCode(TcbdDevice_t *_device, I08U *_bootCode, I32S _size)
{
    I32S i, ret = 0;
    I32U dmaCrc = 0;
    I32S numTableEntry = 0;
    TcbdBootBin_t bootBin[MAX_BOOT_TABLE];
    TcpalTime_t tick;

    tick = TcpalGetCurrentTimeMs();
    numTableEntry = TcbdParseBootBin(_bootCode, _size, bootBin);

    ret |= TcbdChangeMemoryView(_device, EpRam0Ram1);

    for(i = 0; i < numTableEntry && bootBin[i].size; i++)
    {
        TcbdDebug(DEBUG_API_COMMON, "# download boot to 0x%X, size %d\n", bootBin[i].addr, bootBin[i].size);
        ret |= TcbdMemWrite(_device, bootBin[i].addr, bootBin[i].data, bootBin[i].size);
        ret |= TcbdRegReadExCon(_device, TCBD_CMDDMA_CRC32, (I08U*)&dmaCrc, 4);
        if(bootBin[i].crc && (SWAP32(dmaCrc) != bootBin[i].crc))
        {
            TcbdDebug(DEBUG_ERROR, "# CRC Error DMA[0x%08X] != BIN[0x%08X]\n", dmaCrc, bootBin[i].crc);
            return -TCERR_CRC_FAIL;
        }
    }

    ret = TcbdColdBoot(_device);
    if(ret < 0)
        TcbdDebug(DEBUG_ERROR, "Failed to cold boot! ret:%d\n", ret);
    else
        TcbdDebug(DEBUG_ERROR, "# %Lums elapsed to download! \n", TcpalGetTimeIntervalMs(tick));

    return ret;
}

I32S TcbdEnableIrq(TcbdDevice_t *_device, I08U _enBit)
{
    I32S ret = 0;

    ret |= TcbdRegWrite(_device, TCBD_IRQ_STAT_CLR, TCBD_IRQ_STATCLR_ALL);
    ret |= TcbdRegWrite(_device, TCBD_IRQ_EN, _enBit);

    return ret;
}

I32S TcbdDisableIrq(TcbdDevice_t *_device, I08U _mask)
{
    I32S ret = 0;

    ret |= TcbdRegWrite(_device, TCBD_IRQ_STAT_CLR, TCBD_IRQ_STATCLR_ALL);
    ret |= TcbdRegWrite(_device, TCBD_IRQ_EN, _mask);

    return ret;
}

I32S TcbdReadIrqStatus(TcbdDevice_t *_device, I08U *_irqStatus, I08U *_errStatus)
{
    I32S ret = 0;
    I16S status;

    ret = TcbdRegWrite(_device, TCBD_IRQ_LATCH, 0x5E);

    ret |= TcbdRegReadExCon(_device, TCBD_IRQ_STAT_CLR, (I08U*)&status, 2);
    *_irqStatus = status & 0xFF;
    *_errStatus = (status>>8) & 0xFF;

    return ret;
}

I32S TcbdClearIrq(TcbdDevice_t *_device, I08U _status)
{
    return TcbdRegWrite(_device, TCBD_IRQ_STAT_CLR, _status);
}

I32S TcbdChangeIrqMode(TcbdDevice_t *_device, TcbdIntrMode_t _mode)
{
    I08U mode = TCBD_IRQ_MODE_PAD_ENABLE;
    switch(_mode)
    {
        case IntrModeLevelHigh: 
            mode |= TCBD_IRQ_MODE_LEVEL | TCBD_IRQ_MODE_RISING;
            break;
        case IntrModeLevelLow: 
            mode |= TCBD_IRQ_MODE_LEVEL | TCBD_IRQ_MODE_FALLING;
            break;
        case IntrModeEdgeRising: 
            mode |= TCBD_IRQ_MODE_TRIGER | TCBD_IRQ_MODE_RISING;
            break;
        case IntrModeEdgeFalling: 
            mode |= TCBD_IRQ_MODE_TRIGER | TCBD_IRQ_MODE_FALLING;
            break;
        default:
            mode |= TCBD_IRQ_MODE_LEVEL | TCBD_IRQ_MODE_FALLING;
            break;
    }
    return TcbdRegWrite(_device, TCBD_IRQ_MODE, mode);
}

I32S TcbdInitStreamDataConfig(TcbdDevice_t *_device, I08U _useCmdFifo, I08U _bufferMask, I32U _threshold)
{
    I32S ret = 0;
    I16U threshold = SWAP16((_threshold>>2));

    ret |= TcbdRegWrite(_device, TCBD_STREAM_CFG0, 0);                          /* Disable internal buffer */
    ret |= TcbdRegWriteExCon(_device, TCBD_STREAM_CFG1, (I08U*)&threshold, 2);  /* Change interrupt threshold */

    if(_useCmdFifo == ENABLE_CMD_FIFO)
        ret |= TcbdRegWrite(_device, TCBD_STREAM_CFG3, 0x12);             /* command fifo reset */
    else
        ret |= TcbdRegWrite(_device, TCBD_STREAM_CFG3, 0x00);             /* command fifo reset */

    ret |= TcbdRegWrite(_device, TCBD_SYS_RESET, TCBD_SYS_COMP_EP);       /* reset ep */
    ret |= TcbdRegWrite(_device, TCBD_STREAM_CFG0, _bufferMask);          /* Enable internal buffer */

    _device->intrThreshold = _threshold;
    _device->lastSelectedBuffer = _bufferMask;
    _device->enCmdFifo = _useCmdFifo;

    return ret;
}

#if defined(__READ_FIXED_LENGTH__)
I32S TcbdReadStream(TcbdDevice_t *_device, I08U *_buff, I32S *_size)
{
    I32U bytesRemain, bytesToRead = _device->intrThreshold;

    if(_device->intrThreshold <= 0 || _buff == NULL)
    {
        return -TCERR_INVALID_ARG;
    }
    TcbdRegWrite(_device, TCBD_STREAM_CFG3, 0x22);
    TcbdRegReadExCon(_device, TCBD_STREAM_CFG1, (I08U*)&bytesRemain, 2);
    bytesRemain = SWAP16(bytesRemain)<<2;

    TcbdDebug(DEBUG_STREAM_READ, "%d bytes read, %d bytes remain\n", bytesToRead, bytesRemain);
    *_size = bytesToRead;

    return TcbdRegReadExFix(_device, TCBD_STREAM_CFG4, _buff, bytesToRead);
}
#else //__READ_VARIABLE_LENGTH__
I32S TcbdReadStream(TcbdDevice_t *_device, I08U *_buff, I32S *_size)
{
    I32U bytesRemain = 0;
    I32U bytesToRead = _device->intrThreshold;

    if(_device->intrThreshold <= 0 || _buff == NULL)
    {
        return -TCERR_INVALID_ARG;
    }

    TcbdRegWrite(_device, TCBD_STREAM_CFG3, 0x22);
    TcbdRegReadExCon(_device, TCBD_STREAM_CFG1, (I08U*)&bytesRemain, 2);
    bytesRemain = SWAP16(bytesRemain)<<2;

    bytesToRead = bytesToRead + bytesRemain - _device->sizeMoreRead;
    TcbdDebug(DEBUG_STREAM_READ, "%d bytes remain, real data size:%d \n", bytesRemain, bytesToRead);

    if( (TCBD_MAX_THRESHOLD<<1) < bytesToRead)
    {
        TcbdDebug(DEBUG_ERROR, "Could not read data over TCBD_MAX_THRESHOLD(%d) \n");
        return -TCERR_INVALID_ARG;
    }
    _device->sizeMoreRead = bytesRemain;
	if (*_size <= bytesToRead)
	{
		TcbdDebug(DEBUG_ERROR, "bytesToRead size too big!!\n");
		return -TCERR_INVALID_ARG;
	}

    *_size = bytesToRead;

    return TcbdRegReadExFix(_device, TCBD_STREAM_CFG4, _buff, bytesToRead);
}
#endif

I32S TcbdTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz)
{
    I32S ret = 0;
    TcpalTime_t tick;

    tick = TcpalGetCurrentTimeMs();

    if(_freqKhz < 1000000)
        _device->currBand = BandTypeBand3;
    else 
        _device->currBand = BandTypeLband;

    TcbdDebug(DEBUG_API_COMMON, "freq:%d, bw:%d\n", _freqKhz, _bwKhz);
    ret |= TcbdResetComponent(_device, TCBD_SYS_COMP_EP, TCBD_SYS_COMP_DSP);
    ret |= TcbdRfTuneFrequency(_device, _freqKhz, _bwKhz);
    if(ret < 0)
    {
        TcbdDebug(DEBUG_ERROR, "failed to tune frequency to RF!! ret:%d\n", ret);
        return ret;
    }

    if(_device->periType == PeriTypeSpiOnly)
    {
        /* enable fic buffer only*/
        ret |= TcbdInitStreamDataConfig(
                _device, 
                ENABLE_CMD_FIFO, 
                STREAM_DATA_ENABLE|STREAM_HEADER_ON|STREAM_MASK_BUFFERA,
                TCBD_THRESHOLD_FIC);
    }

    TcbdInitDiagnosis();

    ret |= TcbdEnablePeri(_device);
    ret |= TcbdInitBuffer(_device);
    ret |= TcbdResetComponent(_device, TCBD_SYS_COMP_ALL, TCBD_SYS_COMP_DSP);
    ret |= TcbdDemodTuneFrequency(_device, _freqKhz, _bwKhz);
    if(ret < 0)
    {
        TcbdDebug(DEBUG_ERROR, "failed to tune frequency to demodulator!! ret:%d\n", ret);
        return ret;
    }
    _device->prevBand = _device->currBand;
    _device->lastFrequency = _freqKhz;

#if defined(__READ_VARIABLE_LENGTH__)
    _device->sizeMoreRead = 0;
#endif /*__READ_VARIABLE_LENGTH__*/
    TcbdDebug(DEBUG_API_COMMON, " # Frequency set time :%Ld\n", TcpalGetTimeIntervalMs(tick));

    return ret;
}

I32S TcbdWaitTune(TcbdDevice_t *_device, I08U *_status)
{
    I32S ret = 0;
    I08U cto, cfo, ofdm; 
    TcpalTime_t timeTick, tuneWaitTime; 

    tuneWaitTime = TDMB_OFDMDETECT_LOCK * TDMB_OFDMDETECT_RETRY;
    timeTick = TcpalGetCurrentTimeMs();
    do 
    {
        ret = TcbdRegRead(_device, TCBD_PROGRAMID, _status);
        if(ret < 0) goto exitWaitTune; 

        cto = ((*_status) >> 1) & 0x01;
        cfo = ((*_status) >> 2) & 0x01;
        if(cto && cfo) goto exitWaitTune;

        ofdm = ((*_status) >> 5) & 0x01;
        if(ofdm) break;
    }
    while(TcpalGetTimeIntervalMs(timeTick) < tuneWaitTime);

    if(ofdm == 0) 
    {
        ret = -TCERR_TUNE_FAILED;
        goto exitWaitTune;
    }

    tuneWaitTime = ((TDMB_CTO_LOCK * TDMB_CTO_RETRY) + (TDMB_CTO_LOCK+TDMB_CFO_LOCK) * TDMB_CFO_RETRY);
    timeTick = TcpalGetCurrentTimeMs();
    do
    {
        ret = TcbdRegRead(_device, TCBD_PROGRAMID, _status);
        if(ret < 0) goto exitWaitTune;

        cto = ((*_status) >> 1) & 0x01;
        cfo = ((*_status) >> 2) & 0x01;
        if(cto && cfo) break;
    }
    while(TcpalGetTimeIntervalMs(timeTick) < tuneWaitTime);

    if(cto && cfo)
        TcbdDebug(DEBUG_API_COMMON, "lock status : 0x%02X\n", *_status);
    else 
        ret = -TCERR_TUNE_FAILED;

exitWaitTune:
    return ret;
}

inline static I32S TcbdCalculateThreshold(TcbdService_t *_service)
{ 
    I32S threshold = 0;
    I32S uepBitrate[] = {32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384}; 

    switch(_service->pType)
    {
        case PROTECTION_TYPE_UEP: 
            threshold = (uepBitrate[_service->bitrate]*6) << 2;
            break;
        case PROTECTION_TYPE_EEP:
            threshold = (_service->bitrate*6) << 5;
            break;
    }

    TcbdDebug(DEBUG_API_COMMON, "ptype:%s, bitrate :%d, interrupt threshold:%d\n", 
        (_service->pType) ? "EEP" : "UEP", _service->bitrate, threshold);

    if(TCBD_MAX_THRESHOLD < threshold)
        threshold  = TCBD_MAX_THRESHOLD;

    return threshold;
}

inline static I32S TcbdFindEmptySlot(TcbdServiceCtrl_t *_serviceCtrl)
{
    I32S i;

    for(i = 0; i < MAX_NUM_SERVICE; i++)
    {
        if(_serviceCtrl->onAir[i] == 0)
            return i;
    }
    return -1;
}

inline static I32S TcbdFindUsedSlot(TcbdServiceCtrl_t *_serviceCtrl, TcbdService_t *_service)
{
    I32S i;
    I32U *serviceInfo = _serviceCtrl->serviceInfo;

    for(i = 0; i < MAX_NUM_SERVICE; i++)
    {
        if( ((serviceInfo[i*2]>>20) & 0x3F) == _service->subchId)
            return i;
    }
    return -1;
}


I32S TcbdRegisterService(TcbdDevice_t *_device, TcbdService_t *_service)
{
    I32S ret = 0;
    I32S emptySlot, emptySlot2x;
    I32U threshold = 0, dataMode = 0;
    I32U selectedBuffer = 0, selectedStream = 0;
    I08U enCmdFifo = 0, enIrq = 0;

    TcbdServiceCtrl_t *serviceCtrl = &_device->serviceCtrl;
    I32U *serviceInfo = serviceCtrl->serviceInfo;

    switch(_service->type)
    {
        case SERVICE_TYPE_DAB:
        case SERVICE_TYPE_DABPLUS:
        case SERVICE_TYPE_DATA:    dataMode = 0; break;
        case SERVICE_TYPE_DMB:     dataMode = 1; break;
        default:                   dataMode = 2; break;
    }

    emptySlot = TcbdFindEmptySlot(serviceCtrl);
    if(emptySlot < 0)
    {
        TcbdDebug(DEBUG_ERROR, "Exceed maxinum number of service!!\n");
        return -TCERR_MAX_NUM_SERVICE;
    }

    emptySlot2x = emptySlot<<1;

    serviceCtrl->onAir[emptySlot] = 1;
    serviceCtrl->serviceCount++;

    serviceInfo[emptySlot2x]  = (_service->reconfig << 26) | (_service->subchId << 20) | 
        (_service->cuSize << 10) | _service->startCu;
    serviceInfo[emptySlot2x+1] = (emptySlot << 20) | /*(0 << 18) |*/ (dataMode << 16) | 
        (_service->pType << 11) | (_service->pLevel << 8) | _service->bitrate;

    ret = TcbdRegRead(_device, TCBD_IRQ_EN, &enIrq);
    ret |= TcbdDisableIrq(_device, 0);

    switch(_device->periType)
    {
        case PeriTypeSpiOnly:
            /* To prevent h/w fifo overflow, we must recalculate the size of interrupt threshold */
#if defined(__CALC_INTRRUPT_THRESHOLD__)
            threshold = TcbdCalculateThreshold(_service);
            selectedStream = STREAM_SET_GARBAGE(threshold) | STREAM_TYPE_ALL;
#else /*__CALC_INTRRUPT_THRESHOLD__*/
            threshold = TCBD_MAX_THRESHOLD;
            selectedStream = STREAM_SET_GARBAGE(TCBD_MAX_THRESHOLD) | STREAM_TYPE_ALL;
#endif/*__CALC_INTRRUPT_THRESHOLD__*/
            TcbdDebug(DEBUG_API_COMMON, "threshold : %d\n", threshold);

            enCmdFifo = ENABLE_CMD_FIFO;
            selectedBuffer = STREAM_DATA_ENABLE | STREAM_HEADER_ON | STREAM_MASK_BUFFERB;
            break;
        case PeriTypeSts:
            enCmdFifo = DISABLE_CMD_FIFO;
            selectedStream = STREAM_TYPE_ALL;
            selectedBuffer = STREAM_DATA_ENABLE | STREAM_MASK_BUFFERB;
            break;
        default: 
            TcbdDebug(DEBUG_ERROR, "not implemented!\n");
            ret |= -1;
            break;
    }

#if defined(__STATUS_IN_INTERNAL__)
    selectedStream &= ~(STREAM_TYPE_STATUS);
#endif /* __STATUS_IN_INTERNAL__ */

#if defined(__READ_VARIABLE_LENGTH__)
    _device->sizeMoreRead = 0;
#endif /*__READ_VARIABLE_LENGTH__*/
    ret |= TcbdChangeStreamType(_device, selectedStream);
    ret |= TcbdInitStreamDataConfig(_device, enCmdFifo, selectedBuffer, threshold);

    ret |= TcbdSendServiceInfo(_device);
    ret |= TcbdInitBuffer(_device);
    ret |= TcbdEnableIrq(_device, enIrq);
    return ret;
}

I32S TcbdUnregisterService(TcbdDevice_t *_device, TcbdService_t *_service)
{
    I32S ret = 0;
    I32S serviceIndex;
    I08U enIrq;

    TcbdServiceCtrl_t *serviceCtrl = &_device->serviceCtrl;
    I32U *serviceInfo = serviceCtrl->serviceInfo;

    ret = TcbdRegRead(_device, TCBD_IRQ_EN, &enIrq);
    ret |= TcbdDisableIrq(_device, 0);

    serviceIndex = TcbdFindUsedSlot(serviceCtrl, _service);
    if(serviceIndex < 0)
    {
        TcbdDebug(DEBUG_ERROR, "aready unregisterd or not registered service!\n");
        return -TCERR_SERVICE_NOT_FOUND;
    } 
    TcbdDisablePeri(_device); 

    serviceInfo[serviceIndex*2] = 0x00;
    serviceInfo[serviceIndex*2 + 1] = 0x00;
    serviceCtrl->onAir[serviceIndex] = 0;
    serviceCtrl->serviceCount--;

    ret |= TcbdSendServiceInfo(_device);
    //ret |= TcbdEnableIrq(_device, enIrq);

    return ret;
}

I32S TcbdReadFicData(TcbdDevice_t *_device, I08U *_buff, I32S _size)
{
    I32S ret = 0;
    I32U ficBuffAddr;
    I08U status, errStatus;

    TcpalTime_t tick;

    if(_size != TCBD_FIC_SIZE)
    {
        TcbdDebug(DEBUG_ERROR, "wrong fic size! %d\n", _size);
        return -TCERR_INVALID_ARG;
    }
    ret = TcbdReadIrqStatus(_device, &status, &errStatus);

    //if(errStatus & TCBD_IRQ_ERROR_FIFOA) return -TCERR_NO_FIC_DATA;
    if(status & TCBD_IRQ_STAT_FIFOAINIT)
    {
        ficBuffAddr = PHY_MEM_ADDR_A_START;
        tick = TcpalGetCurrentTimeMs();    
        TcbdDebug(0, "status:0x%02X, err:0x%02X, %Ld elapsed\n", 
            status, errStatus, TcpalGetTimeIntervalMs(tick));
            
        ret = TcbdMemRead(_device, ficBuffAddr, _buff, _size);
        ret |= TcbdClearIrq(_device, status);
    }
    else 
    {
        ret |= -TCERR_NO_FIC_DATA;
    }

    ret |= TcbdClearIrq(_device, status);
    return ret;
}

I32S TcbdReadSignalInfo(TcbdDevice_t *_device, TcbdDiagnosis_t *_diagInfo)
{
    I32S ret = 0;
#if defined(__STATUS_IN_INTERNAL__)
    I08U status[32] = {0, };
#endif //__STATUS_IN_INTERNAL__

    if(!_diagInfo)
    {
        return -TCERR_INVALID_ARG;
    }
#if defined(__STATUS_IN_INTERNAL__)

    ret = TcbdRegWrite(_device, TCBD_OP_STATUS0, 0x1);
    ret |= TcbdRegReadExFix(_device, TCBD_OP_STATUS1, status, 32);

    TcbdUpdateStatus(status, _diagInfo);
#elif defined(__STATUS_IN_STREAM__)
    TcbdUpdateStatus(NULL, _diagInfo);
#endif //__STATUS_IN_STREAM__
    return ret;
}
