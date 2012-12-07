/* drivers/broadcast/tcc3170/src/tdmb_tunerbbdrv_tcc3170.c
 * Copyright (C) 2011 LGE, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "../../broadcast_tdmb_typedef.h"
#include "../../broadcast_tdmb_drv_ifdef.h"
#include "broadcast_tcc3170.h"
#include "tdmb_tunerbbdrv_tcc3170def.h"

#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_component.h"
#include "tcbd_drv_io.h"

#include "tcbd_diagnosis.h"
#include "tcbd_fic_decoder.h"
#include "tcbd_hal.h"
#include "tcpal_queue.h"


#include "TCC317X_BOOT_TDMB.h"
/* ----------------------------------------------------------
**    1.   DEFINITIONS
-------------------------------------------------------------*/
//#define __SW_RESYNC__

#define MAX_KOREABAND_FULL_CHANNEL  21
#define INVALID_SUBCH_NUM 0xFF

#define TCBD_SKIP_WAIT_LOCK 1
#define TCBD_WAIT_LOCK      0

/* ----------------------------------------------------------
**    2.   External variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    3.   External Functions
-------------------------------------------------------------*/
int8 tunerbb_drv_tcc3170_tune(int nFreqNo, int skipLockWait);

/* ----------------------------------------------------------
**    4.   Local Constant Variables
-------------------------------------------------------------*/


/* ----------------------------------------------------------
**    5.   Local Typedef
-------------------------------------------------------------*/

/* ----------------------------------------------------------
**    6.   Global Variables
-------------------------------------------------------------*/
static I32U g_uiKOREnsembleFullFreq[MAX_KOREABAND_FULL_CHANNEL] =
{
    175280,177008,178736,181280,183008,
    184736,187280,189008,190736,193280,
    195008,196736,199280,201008,202736,
    205280,207008,208736,211280,213008,
    214736
};

#if !defined(STREAM_TS_UPLOAD)
#define TCBD_FIC_BUFFER_SIZE  (5*1024)
static TcbdQueue_t FicQueue;
static I08U FicBuffer[TCBD_FIC_BUFFER_SIZE*2];
#endif

/* ----------------------------------------------------------
**    7.   Static Variables
-------------------------------------------------------------*/
static TcbdDevice_t TcbdDevice;
static TcbdService_t ServiceRegistered;
static TcbdDiagnosis_t SignalInfo;
static I08U ResyncTryCnt = 0;
static I32S FrequencyNum = 0;

void tunerbb_drv_tcc3170_set_userstop(void)
{
    tdmb_tcc3170_set_userstop();
}

int tunerbb_drv_tcc3170_is_on(void)
{
    return tdmb_tcc3170_is_power_on();
}

int8 tunerbb_drv_tcc3170_power_on(void)
{
    int rc = TRUE;

    TcbdDebug(DEBUG_LGE, "\n");
    rc = tdmb_tcc3170_power_on( );

    return rc;
}


int8 tunerbb_drv_tcc3170_power_off(void)
{
    int rc = TRUE;

    TcbdDebug(DEBUG_LGE, "\n");
    if(tdmb_tcc3170_is_power_on())
    {
#if !defined(__TEST_IRQ_REG_ONCE__)
        TcpalUnRegisterIrqHandler();
#endif
        rc = tdmb_tcc3170_power_off( );
    }
    return rc;
}


int8 tunerbb_drv_tcc3170_select_antenna(unsigned int sel)
{
    int rc;

    rc = tdmb_tcc3170_select_antenna(sel);

    return rc;
}


int8 tunerbb_drv_tcc3170_reset_ch(void)
{
    if(ServiceRegistered.subchId != INVALID_SUBCH_NUM)
    {
        TcbdUnregisterService(&TcbdDevice, &ServiceRegistered);
        ServiceRegistered.subchId = INVALID_SUBCH_NUM;
        TcbdDebug(DEBUG_INFO, "unregister service!\n");
    }

    FrequencyNum = 0;
    ResyncTryCnt = 0;

#if defined(STREAM_SPI_UPLOAD)
    TcbdResetQueue(&FicQueue);
#endif //STREAM_SPI_UPLOAD

    return TCBD_TRUE;
}

TcbdDevice_t *GetTcbdDevice(void)
{
	return &TcbdDevice;
}

int8 tunerbb_drv_tcc3170_init(void)
{
    int ret = 0;
    uint8 chipId = 0;

    uint8 *boot = TCC317X_BOOT_DATA_TDMB;
    int  size = TCC317X_BOOT_SIZE_TDMB;
    I32U ver, pllData[] = TCBD_DEF_PLL_VALUE;

    memset(&TcbdDevice, 0x00, sizeof(TcbdDevice));
    TcbdDevice.chipAddr = DEFAULT_CHIP_ADDRESS;

    ret = TcbdIoOpen(&TcbdDevice);
    if(ret < TCERR_SUCCESS)
    {
        TcbdDebug(DEBUG_ERROR, "tcc3170 device open failed  ret:%d\n", ret);
        return TCBD_FALSE;
    }
    TcbdResetComponent(&TcbdDevice, TCBD_SYS_COMP_EP, TCBD_SYS_COMP_ALL);

    TcbdRegRead(&TcbdDevice, TCBD_CHIPID, &chipId);
    TcbdDebug(DEBUG_INFO, "tcc3170 chip id: 0x%X\n", chipId);
    if(chipId != TCBD_CHIPID_VALUE)
        goto tcc3170Init;

    TcbdInitPll(&TcbdDevice, pllData);
#if defined(STREAM_SLAVE_PARALLEL_UPLOAD) // if EBI interface

#elif defined(STREAM_TS_UPLOAD)    // if TSIF interface
    TcbdInitStreamDataConfig(&TcbdDevice, DISABLE_CMD_FIFO, STREAM_DATA_ENABLE|STREAM_MASK_BUFFERB, 0);
    TcbdInitBufferRegion(&TcbdDevice);

    TcbdSelectPeri(&TcbdDevice, PeriTypeSts);
    TcbdInitGpioForPeri(&TcbdDevice, PeriTypeSts);

#elif defined(STREAM_SPI_UPLOAD)    // if SPI interface
    TcbdInitBufferRegion(&TcbdDevice);

    TcbdSelectPeri(&TcbdDevice, PeriTypeSpiOnly);
    TcbdInitGpioForPeri(&TcbdDevice, PeriTypeSpiOnly);
#endif // defined(STREAM_SLAVE_PARALLEL_UPLOAD)

    ret = TcbdDownloadBootCode(&TcbdDevice, boot, size);
    if(ret < TCERR_SUCCESS)
    {
        TcbdDebug(DEBUG_ERROR, "downlaod failed!! error:%d\n", (int32_t)ret);
        goto tcc3170Init;
    }
    else
    {
        TcbdRecvBootCodeVersion(&TcbdDevice, &ver);
        TcbdDebug(DEBUG_INFO, "download success!! version:0x%X\n", (unsigned int)ver);
    }

    FrequencyNum = 0;
    ResyncTryCnt = 0;
    ServiceRegistered.subchId = INVALID_SUBCH_NUM;

    TcbdInitDiversityIo(&TcbdDevice, DivIoTypeSingle);
#if defined(STREAM_SPI_UPLOAD)    // if SPI interface
    TcbdInitQueue(&FicQueue, FicBuffer, TCBD_FIC_BUFFER_SIZE);
    TcbdChangeIrqMode(&TcbdDevice, IntrModeLevelLow);
	TcpalIrqEnable();

#if !defined(__TEST_IRQ_REG_ONCE__)
    TcpalRegisterIrqHandler(&TcbdDevice);
#endif

#endif //STREAM_SPI_UPLOAD

    return TCBD_TRUE;

tcc3170Init:
	TcbdDebug(DEBUG_ERROR, "!!!!!!! tcbd init failed!!! \n");
    TcbdIoClose(&TcbdDevice);
    return TCBD_FALSE;
}

int8 tunerbb_drv_tcc3170_stop(void)
{
	TcpalIrqDisable(); //added 2011-12-16 bsyoo
    TcbdDisableIrq(&TcbdDevice, 0);
    TcbdDeinitQueue(&FicQueue);//20120110 mo2hyungmin.kim 
    TcbdIoClose(&TcbdDevice);

    FrequencyNum = 0;
    ResyncTryCnt = 0;
    ServiceRegistered.subchId = INVALID_SUBCH_NUM;

    return TCBD_TRUE;
}


int8 tunerbb_drv_tcc3170_get_ber(struct broadcast_tdmb_sig_info *dmb_bb_info)
{
       int8 ret = TCBD_TRUE;

    memset(&SignalInfo, 0 , sizeof(SignalInfo));
    if (TcbdReadSignalInfo(&TcbdDevice, &SignalInfo) < 0)
    {
       memset(dmb_bb_info, 0, sizeof(struct broadcast_tdmb_sig_info));
             dmb_bb_info->va_ber = ERROR_LIMIT;
             dmb_bb_info->msc_ber = ERROR_LIMIT;
             ret = TCBD_FALSE;
             goto err_exit;
    }
    dmb_bb_info->tp_lock   = (SignalInfo.lock & TCBD_LOCK_SYNC) ? 1 : 0;
    dmb_bb_info->msc_ber   = SignalInfo.pcberMovingAvg;
    dmb_bb_info->va_ber    = SignalInfo.vberMovingAvg;
    dmb_bb_info->tp_err_cnt = SignalInfo.tsErrorCnt;//20120705 mo2hyugmin.kim for TP_ERROR_COUNT issue in Factrory test

//20120508 mo2hyungmin.kim for antenna_level
    if(dmb_bb_info->msc_ber >= 10000)                         dmb_bb_info->antenna_level = 0;
	else if(dmb_bb_info->msc_ber > 7000 && dmb_bb_info->msc_ber < 10000)   dmb_bb_info->antenna_level = 1;
	else if (dmb_bb_info->msc_ber > 5000 && dmb_bb_info->msc_ber <= 7000)  dmb_bb_info->antenna_level = 2;
	else if (dmb_bb_info->msc_ber > 3000 && dmb_bb_info->msc_ber <= 5000)  dmb_bb_info->antenna_level = 3;
	else if (dmb_bb_info->msc_ber >= 0 && dmb_bb_info->msc_ber <= 3000)    dmb_bb_info->antenna_level = 4;
	
    if(dmb_bb_info->tp_lock)
    {
        //dmb_bb_info->sync_lock = 1;
        dmb_bb_info->cir       = 1;
        dmb_bb_info->afc_ok    = 1;
        dmb_bb_info->dab_ok    = 1;
        dmb_bb_info->sch_ber   = 1;
    }
    else if(ServiceRegistered.type == SERVICE_TYPE_DMB)
    {
    TcbdDebug(DEBUG_LGE,"ServiceRegistered.type == SERVICE_TYPE_DMB\n");
        dmb_bb_info->msc_ber   = ERROR_LIMIT;
        dmb_bb_info->va_ber    = ERROR_LIMIT;
    }
	if(dmb_bb_info->msc_ber == 20000)
	{
		dmb_bb_info->dab_ok = 0; 
	}
 
    TcbdDebug(DEBUG_LGE, "lock:0x%X, rssi=%d, snr:%d, pcber:%d, vber:%d, tsper:%d\n", SignalInfo.lock,
            (int)SignalInfo.rssi, (int)SignalInfo.snr, (int)SignalInfo.pcber, (int)SignalInfo.vber, (int)SignalInfo.tsper);

err_exit:
    return ret;
}



int8 tunerbb_drv_tcc3170_get_msc_ber(uint32 *msc_ber)
{
	int8 ret = TCBD_TRUE;
    TcbdDiagnosis_t signalInfo;

    if(TcbdReadSignalInfo(&TcbdDevice, &signalInfo) < 0)
    {
    	*msc_ber = ERROR_LIMIT;
		ret = TCBD_FALSE;
    }
	else
	{
    	*msc_ber = (uint32)signalInfo.pcberMovingAvg;
	}

    return ret;
}

int8 tunerbb_drv_tcc3170_set_channel(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
    return tunerbb_drv_tcc3170_multi_set_channel(freq_num, 1, &subch_id, &op_mode);
}


int8 tunerbb_drv_tcc3170_re_syncdetector(uint8 op_mode)
{
#if defined(__SW_RESYNC__)
    I32S ret = 0;
    I08U cto = 0, cfo = 0;

    if(op_mode != TCBD_ENSQUERY)
    {
        if(ResyncTryCnt == 0 && FrequencyNum)
        {
            ret = tunerbb_drv_tcc3170_tune((int)FrequencyNum, TCBD_SKIP_WAIT_LOCK);
            ResyncTryCnt++;
        }

        cto = (SignalInfo.lock >> 1) & 0x01;
        cfo = (SignalInfo.lock >> 2) & 0x01;

        TcbdDebug(DEBUG_LGE, "cto : %d, cfo : %d, 0x%X\n", cto, cfo, (unsigned int)SignalInfo.lock);
        if(cto && cfo)
        {
            ResyncTryCnt = 0;
            return tunerbb_drv_tcc3170_re_sync();
        }
    }

    TcbdDebug(DEBUG_ERROR, "no sync\n");
    return TCBD_FALSE;
#endif //__SW_RESYNC__
    return TCBD_TRUE;
}


int8 tunerbb_drv_tcc3170_re_sync(void)
{
    I32S ret = 0;
#if defined(__SW_RESYNC__)
    I32S timeout = 0, buffer_size, type;
    TcbdFicSch_t *subch = NULL;
    I08U buffFic[TCBD_FIC_SIZE];
#endif //__SW_RESYNC__

    if(ServiceRegistered.subchId == INVALID_SUBCH_NUM)
    {
        TcbdDebug(DEBUG_ERROR, "sub channel is not setted!\n");
        return TCBD_FALSE;
    }

#if defined(__SW_RESYNC__)
    TcbdFicInitDb();
    do
    {
#if defined(__I2C_STS__)
        if(TcbdReadFicData(&TcbdDevice,  buffFic, TCBD_FIC_SIZE) == 0)
        {
#elif defined(__CSPI_ONLY__)
        if(TcbdDequeue(&FicQueue, buffFic, &buffer_size, &type) < 0)
        {
            if(buffer_size != TCBD_FIC_SIZE)
            {
                TcbdDebug(DEBUG_ERROR, "wrong fic data! \n");
            }
#else /*__CSPI_ONLY__*/
#error
#endif /*!__CSPI_ONLY__*/
            ret = TcbdFicRunDecoder(buffFic, TCBD_FIC_SIZE-4);
            subch = TcbdIsSubchExist(ServiceRegistered.subchId);
        }
        TcpalSleep(50);
        timeout++;
    }
    while(subch == NULL && timeout < 20);
    TcbdDebug(DEBUG_INFO, "decode status:%d, timeout:%d subch:0x%x\n", (int)ret, (int)timeout, (unsigned int)subch);

    if(subch == NULL || timeout >= 20)
        return TCBD_FALSE;

    if(subch)
    {
        ServiceRegistered.startCu  = subch->startAddr;
        ServiceRegistered.cuSize   = TcbdFicGetSchSize(subch);//subch->subchSize;
        ServiceRegistered.bitrate  = TcbdFicGetBitRate(subch);
        ServiceRegistered.pType    = TcbdFicGetPrtType(subch);
        ServiceRegistered.pLevel   = TcbdFicGetPrtLvl(subch);
        ServiceRegistered.reconfig = 2;
        ServiceRegistered.rsOn     = (ServiceRegistered.type == SERVICE_TYPE_DMB) ? 1: 0;

        ret = TcbdRegisterService(&TcbdDevice, &ServiceRegistered);
        TcbdDebug(DEBUG_INFO, "subchid:%d, start:0x%X, size:0x%X, bitrate:0x%X, ptype:%d, plevel:%d, rsOn:%d\n",
                (int)ServiceRegistered.subchId, (unsigned int)ServiceRegistered.startCu,
                (unsigned int)ServiceRegistered.cuSize, (unsigned int)ServiceRegistered.bitrate,
                (int)ServiceRegistered.pType, (int)ServiceRegistered.pLevel,
                (int)ServiceRegistered.rsOn);
    }
#else //__SW_RESYNC__
    ret = TcbdRegisterService(&TcbdDevice, &ServiceRegistered);
    TcbdDebug(DEBUG_INFO, "subchid:%d, start:0x%X, size:0x%X, bitrate:0x%X, ptype:%d, plevel:%d, rsOn:%d\n",
            (int)ServiceRegistered.subchId, (unsigned int)ServiceRegistered.startCu,
            (unsigned int)ServiceRegistered.cuSize, (unsigned int)ServiceRegistered.bitrate,
            (int)ServiceRegistered.pType, (int)ServiceRegistered.pLevel,
            (int)ServiceRegistered.rsOn);
#endif //!__SW_RESYNC__

    if(ret < TCERR_SUCCESS)
        return TCBD_FALSE;
    else
        return TCBD_TRUE;
}


int8 tunerbb_drv_tcc3170_control_fic(uint8 enable)
{
    return TCBD_TRUE;
}


uint32 tunerbb_drv_tcc3170_get_freq(int nFreqIndex)
{
    int major_ch, minor_ch, fnindex;

    major_ch = nFreqIndex /10;
    minor_ch = nFreqIndex %10;

    if(minor_ch < 1)
        minor_ch = 1;
    else if(minor_ch > 3)
        minor_ch = 3;

    if(major_ch < 7)
        major_ch = 7;
    else if(major_ch > 13)
        major_ch = 13;

    fnindex = (major_ch-7)*3 + (minor_ch-1);

    return g_uiKOREnsembleFullFreq[fnindex];
}

int8 tunerbb_drv_tcc3170_tune(int nFreqNo, int skipLockWait)
{
    I32S ret = 0;
    I08U status;
    uint32 frequency;

    frequency = tunerbb_drv_tcc3170_get_freq(nFreqNo);
    TcbdDebug(DEBUG_INFO, "tunerbb_drv_tcc3170_tune nFreqNo = %d, ulFreq = %d \n", (int)nFreqNo, (int)frequency);

    TcbdDisableIrq(&TcbdDevice, 0);
    ret = TcbdTuneFrequency(&TcbdDevice, frequency, TCBD_DEF_BANDWIDTH);
    if(ret < TCERR_SUCCESS)
    {
        TcbdDebug(DEBUG_ERROR, " ret%d  \n", (int)ret);
        return TCBD_FALSE;
    }

    if(!skipLockWait)
    {  
        ret = TcbdWaitTune(&TcbdDevice, &status);
        if(ret < TCERR_SUCCESS)
        {
            TcbdDebug(DEBUG_ERROR, "ret:%d\n", (int)ret);
            return TCBD_FALSE;
        }
    }
#if defined(__CSPI_ONLY__)
    TcbdEnableIrq(&TcbdDevice, TCBD_IRQ_EN_DATAINT);
#elif defined(__I2C_STS__)
    TcbdEnableIrq(&TcbdDevice, TCBD_IRQ_EN_FIFOAINIT);
#endif //__I2C_SCS__

    return TCBD_TRUE;
}

int8 tunerbb_drv_tcc3170_put_fic(uint8* buffer, uint32 buffer_size)
{
    TcbdEnqueue(&FicQueue, (I08U*)buffer, (I32S)buffer_size, 1);

    return TCBD_TRUE;
}

int8 tunerbb_drv_tcc3170_get_fic(uint8* buffer, uint32* buffer_size/* bool crc_onoff*/)
{
    I32S ret = TCBD_TRUE;
    I32S type;

#if defined(__I2C_STS__)
    ret = TcbdReadFicData(&TcbdDevice, buffer, TCBD_FIC_SIZE);
    if(ret < TCERR_SUCCESS)
    {
        TcbdDebug(DEBUG_ERROR, "ret :%d\n", (int)ret);
        return TCBD_FALSE;
    }
#elif defined(__CSPI_ONLY__)
    TcbdDequeue(&FicQueue, (I08U*)buffer, (I32S*)buffer_size, &type);

    if(*buffer_size != TCBD_FIC_SIZE)
    {
        TcbdDebug(DEBUG_ERROR, "wrong fic sie: %d\n", *buffer_size);
        ret = TCBD_FALSE;
    }
#endif //__CSPI_ONLY__
    *buffer_size = 384;
    return ret;
}


int8 tunerbb_drv_tcc3170_read_data(uint8* buffer, uint32* buffer_size)
{
    return TCBD_FALSE;
}

int8 tunerbb_drv_tcc3170_set_service(int32 freq_num, uint8 subch_id, uint8 op_mode)
{
    int8 srvType[5] = {0x0,
        SERVICE_TYPE_DAB,
        SERVICE_TYPE_DMB,
        SERVICE_TYPE_DMB,
        SERVICE_TYPE_DATA};

    ServiceRegistered.subchId = subch_id;
    ServiceRegistered.type    = srvType[op_mode];

    return tunerbb_drv_tcc3170_re_sync();
}

int8 tunerbb_drv_tcc3170_multi_set_channel(int32 freq_num, uint8 subch_cnt, uint8 subch_id[], uint8 op_mode[])
{
    int32 ret = 0;
    int32 i = 0;

    FrequencyNum = freq_num;
	TcbdDevice.isRecvFic = 0;
    if(*op_mode == TCBD_ENSQUERY) //scan
    {
        ret = tunerbb_drv_tcc3170_tune(freq_num, TCBD_WAIT_LOCK);
		TcbdDevice.isRecvFic = 1;
#if defined(__CSPI_ONLY__)
		TcbdResetQueue(&FicQueue);
#endif //__CSPI_ONLY__  
        if(ret == TCBD_TRUE)
            return TCBD_TRUE;
        else
            return TCBD_FALSE;
    }
    else
    {
        tunerbb_drv_tcc3170_tune(freq_num, TCBD_SKIP_WAIT_LOCK);
    }
    

    for(i = 0; i < subch_cnt; i++)
    {
        ret = tunerbb_drv_tcc3170_set_service(freq_num, subch_id[i], op_mode[i]);
        if(ret == 0) break;
    }

    return ret;
}

int8 tunerbb_drv_tcc3170_get_datatype(uint8 subchid, uint8* datatype)
{
    return TCBD_FALSE;
}


int8 tunerbb_drv_tcc3170_get_datathreshold(uint8 subchid, uint32* threshold)
{
    return TCBD_FALSE;
}


int8 tunerbb_drv_tcc3170_process_multi_data(uint8 subch_cnt,uint8* input_buf, uint32 input_size, uint32* read_size)
{
    return TCBD_FALSE;
}


int8 tunerbb_drv_tcc3170_get_multi_data(uint8 subch_cnt, uint8* buf_ptr, uint32 buf_size)
{
    return TCBD_FALSE;
}


void tunerbb_drv_tcc3170_start_tii(void)
{
}


void tunerbb_drv_tcc3170_stop_tii(void)
{
}


boolean tunerbb_drv_tcc3170_check_tii(uint8* pmain_tii, uint8* psub_tii)
{
    return TCBD_TRUE;
}

