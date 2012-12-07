
#include "tcpal_os.h"
#include "tcbd_feature.h"
#include "tcpal_debug.h"

#include "tcbd_api_common.h"
#include "tcbd_drv_component.h"
#include "tcbd_drv_io.h"
#include "tcbd_drv_rf.h"

typedef struct _TcbdSpurTable
{
    I32S freqKhz;
    I32S numData;
    I32S data[7];
} TcbdSpurTable_t;

typedef struct _TcbdAgcTable_t
{
    I32S cmd;
    I32S numData;
    I32U data[7];
} TcbdAgcTable_t;


static TcbdSpurTable_t SpurTable[] = 
{
    { 181936, 2, {(0x035C<<16)|(0x0342), (0x0359<<16)|(0x033e)}},
    { 183008, 2, {(0x00CA<<16)|(0x036B), (0x00CE<<16)|(0x0368)}}, 
    { 192352, 2, {(0x0081<<16)|(0x0328), (0x0084<<16)|(0x0324)}},
    { 201008, 2, {(0x033A<<16)|(0x0366), (0x0336<<16)|(0x0363)}},
    { 201072, 2, {(0x034B<<16)|(0x0352), (0x0347<<16)|(0x034F)}},
    { 211280, 2, {(0x001E<<16)|(0x0307), (0x001F<<16)|(0x0302)}},
    { 211648, 2, {(0x009F<<16)|(0x033E), (0x00A2<<16)|(0x033A)}},
    { 220352, 2, {(0x0361<<16)|(0x033E), (0x035E<<16)|(0x033A)}},
    { 230784, 2, {(0x008B<<16)|(0x032F), (0x008E<<16)|(0x032B)}},
    {1459808, 2, {(0x00CA<<16)|(0x036B), (0x00CE<<16)|(0x0368)}},
    {1468368, 2, {(0x0366<<16)|(0x033A), (0x0363<<16)|(0x0336)}},
    {1478640, 2, {(0x005A<<16)|(0x0316), (0x005C<<16)|(0x0311)}}
};

#if !defined(__AGC_TABLE_IN_BOOT__)
static TcbdAgcTable_t AgcTableLband[] = 
{
    { MBCMD_AGC_DAB_CFG, 3, {0xC0ACFF44, 0x031EFF53, 0x011EFF07}},
    { MBCMD_AGC_DAB_JAM, 3, {0x00091223, (195<<16)+(23<<8)+24, 0x001C0012}},
    { MBCMD_AGC_DAB_3,   7, {0x52120223, 0x58120823, 0x5c121223, 0x60121c23, 0x62122023, 0x68122423, 0x6a122a23}},
    { MBCMD_AGC_DAB_4,   7, {0x70123223, 0x30093a23, 0x32094023, 0x3a094823, 0x3c095223, 0x40095823, 0x42095c23}},
    { MBCMD_AGC_DAB_5,   7, {0x48096023, 0x4a096223, 0x52096423, 0x4e096823, 0x58096a23, 0x5a097023, 0x5e090812}},
    { MBCMD_AGC_DAB_6,   7, {0x62091212, 0x68090e12, 0x70091c12, 0x72092012, 0x78092412, 0x7a093012, 0x7c093812}},
    { MBCMD_AGC_DAB_7,   4, {0x7e093a12, 0x6f094012, 0x73094212, 0x7f094a12}},
    { MBCMD_AGC_DAB_8,   5, {0x30093a23, 0x32094023, 0x3a094823, 0x3c095223, 0x40095823}},
    { MBCMD_AGC_DAB_9,   5, {0x72123a23, 0x78124023, 0x7a124823, 0x7e125223, 0x73125823}},
    { MBCMD_AGC_DAB_A,   5, {0x5e090812, 0x62091212, 0x68090e12, 0x70091c12, 0x72092012}},
    { MBCMD_AGC_DAB_B,   5, {0x5e097223, 0x62097823, 0x68097a23, 0x70097e23, 0x72097323}},
    { MBCMD_AGC_DAB_C,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_D,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_E,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_F,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
};

static TcbdAgcTable_t AgcTableVhf[] = 
{
    { MBCMD_AGC_DAB_CFG, 3, {0xC0ACFF44, 0x031EFF53, 0x011EFF07}},
    { MBCMD_AGC_DAB_JAM, 3, {0x16112243, (195<<16)+(16<<8)+17, 0x005a0043}},
    { MBCMD_AGC_DAB_3,   7, {0x5a220243, 0x5e220843, 0x60221043, 0x62221443, 0x64221c43, 0x68222043, 0x6c222243}},
    { MBCMD_AGC_DAB_4,   7, {0x70222843, 0x2a112c43, 0x30113443, 0x34113a43, 0x38113e43, 0x3c114243, 0x3e114643}},
    { MBCMD_AGC_DAB_5,   7, {0x40114c43, 0x44115443, 0x48115a43, 0x4a115e43, 0x50116043, 0x54116443, 0x5a112422}},
    { MBCMD_AGC_DAB_6,   7, {0x5e112822, 0x60113222, 0x62113622, 0x66113a22, 0x6a113e22, 0x6e114222, 0x72114622}},
    { MBCMD_AGC_DAB_7,   4, {0x76114822, 0x7a115022, 0x7e115422, 0x7f115622}},
    { MBCMD_AGC_DAB_8,   5, {0x2a112c43, 0x30113443, 0x34113a43, 0x38113e43, 0x3c114243}},
    { MBCMD_AGC_DAB_9,   5, {0x74222c43, 0x78223443, 0x7a223a43, 0x7e223e43, 0x7f224243}},
    { MBCMD_AGC_DAB_A,   5, {0x5a112422, 0x5e112822, 0x60113222, 0x62113622, 0x66113a22}},
    { MBCMD_AGC_DAB_B,   5, {0x5a116843, 0x5e116c43, 0x60117243, 0x62117643, 0x66117a43}},
    { MBCMD_AGC_DAB_C,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_D,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_E,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
    { MBCMD_AGC_DAB_F,   5, {0x0, 0x0, 0x0, 0x0, 0x0}},
};
#else //__AGC_TABLE_IN_BOOT__
static TcbdAgcTable_t AgcTableLband[] = 
{
    {MBCMD_AGC_DAB_JAM, 3, {0x0, (195<<16)+(23<<8)+24, 0x001C0012}},
};

static TcbdAgcTable_t AgcTableVhf[] = 
{
    {MBCMD_AGC_DAB_JAM, 3, {0x0, (195<<16)+(16<<8)+17, 0x005a0043}},
};
#endif //__AGC_TABLE_IN_BOOT__

I32S TcbdSendSpurTable(TcbdDevice_t *_device, I32S _freqKhz)
{
    I32S ret = 0, i;
    I32S tableSize = sizeof(SpurTable)/sizeof(TcbdSpurTable_t); 

    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_WRITE;
    mail.cmd = MBCMD_FP_DAB_IIR;
    for(i = 0; i < tableSize; i++)
    {
        if(SpurTable[i].freqKhz == _freqKhz)
        {
            mail.count = SpurTable[i].numData;
            TcpalMemoryCopy(mail.data, SpurTable[i].data, mail.count*sizeof(I32U));
            TcbdDebug(DEBUG_DRV_COMP, "freq:%d, num mail data:%d\n", _freqKhz, mail.count);

            ret = TcbdSendMail(_device, &mail);
            break;
        }
    }

    return ret;
}

I32S TcbdSendAgcTable(TcbdDevice_t *_device, TcbdBand_t _bandType)
{
    I32S ret = 0, i;
    I32S tableSize; 
    TcbdAgcTable_t *agcTable;
    TcbdMail_t mail = {0, };

    switch(_bandType)
    {
        case BandTypeLband:
            tableSize = sizeof(AgcTableLband)/sizeof(TcbdAgcTable_t);
            agcTable = AgcTableLband;
            break;
        case BandTypeBand3:
            tableSize = sizeof(AgcTableVhf)/sizeof(TcbdAgcTable_t);
            agcTable = AgcTableVhf;
            break;
        default:
            TcbdDebug(DEBUG_ERROR, "Unknown band type:%d \n", _bandType);
            return -1;
    }
    TcbdDebug(DEBUG_DRV_COMP, "agc table size:%d, band:%s\n", 
        tableSize, (_bandType == BandTypeLband) ? "Lband" : "Band3");

    mail.flag = MB_CMD_WRITE;
    for(i = 0; i < tableSize; i++)
    {
        mail.cmd = agcTable[i].cmd;
        mail.count = agcTable[i].numData;
        TcpalMemoryCopy(mail.data, agcTable[i].data, mail.count*sizeof(I32U));

        ret = TcbdSendMail(_device, &mail);
    }

    return ret;
}


I32S TcbdSendFreqeuncy(TcbdDevice_t *_device, I32S _freqKhz)
{
    I32S ret = 0;
    TcbdMail_t mail;

    TcbdDebug(DEBUG_DRV_COMP, "freq:%d\n", _freqKhz);

    mail.flag = MB_CMD_WRITE;
    mail.cmd = MBPARA_SYS_NUM_FREQ;
    mail.count = 1;
    mail.data[0] = 1;
    ret |= TcbdSendMail(_device, &mail);

    mail.cmd = MBPARA_SYS_FREQ_0_6;
    mail.data[0] = _freqKhz;
    ret |= TcbdSendMail(_device, &mail);

    return ret;
}

inline static void TcbdSortStartCu(TcbdServiceCtrl_t *_serviceCtrl, I32U *_serviceInfo)
{
    I32S i, j;
    I32S startCu[MAX_NUM_SERVICE];
    I32U tmpCu, tmpInfo[2];

    TcpalMemoryCopy(_serviceInfo, _serviceCtrl->serviceInfo, sizeof(I32U)*MAX_NUM_SERVICE*2);
    for(i = 0; i < MAX_NUM_SERVICE; i++)
    {
        if(_serviceInfo[i<<1] != 0)
            startCu[i] = _serviceInfo[i*2] & 0x3FF;
        else
            startCu[i] = 0x3FF;
    }

    for(i = 0; i < MAX_NUM_SERVICE; i++)
    {
        for(j = i + 1; j < MAX_NUM_SERVICE; j++)
        {
            if(startCu[i] > startCu[j])
            {
                tmpCu = startCu[i];
                tmpInfo[0] = _serviceInfo[i*2];
                tmpInfo[1] = _serviceInfo[i*2+1];

                startCu[i] = startCu[j];
                _serviceInfo[i*2] = _serviceInfo[j*2];
                _serviceInfo[i*2+1] = _serviceInfo[j*2+1];

                startCu[j] = tmpCu;
                _serviceInfo[j*2] = tmpInfo[0];
                _serviceInfo[j*2+1] = tmpInfo[1];
            }
        }
    }
#if defined(__DEBUG_TCBD__)
    for(i = 0; i < MAX_NUM_SERVICE*2; i++)
    {
        TcbdDebug(DEBUG_DRV_COMP, "%02d: 0x%08X \n", i, _serviceInfo[i]);
    }
#endif //__DEBUG_TCBD__
}

I32S TcbdSendServiceInfo(TcbdDevice_t *_device)
{
    I32S ret = 0;
    TcbdMail_t mail = {0, } ;
    TcbdServiceCtrl_t *serviceCtrl = &_device->serviceCtrl;
    I32U sortedServiceInfo[MAX_NUM_SERVICE*2];

    TcbdSortStartCu(serviceCtrl, sortedServiceInfo);

    mail.cmd = MBPARA_SEL_CH_INFO_PRAM;
    mail.count = 6;
    TcpalMemoryCopy(mail.data, sortedServiceInfo, 6*sizeof(I32U));
    ret = TcbdSendMail(_device, &mail);

    mail.cmd = MBPARA_SEL_CH_INFO_PRAM+1;
    mail.count = 6;
    TcpalMemoryCopy(mail.data, sortedServiceInfo+6, 6*sizeof(I32U));
    ret = TcbdSendMail(_device, &mail);

    mail.cmd = MBPARA_SYS_DAB_MCI_UPDATE;
    mail.count = 1;
    mail.data[0] = serviceCtrl->serviceCount;
    ret |= TcbdSendMail(_device, &mail);

    return ret;
}

I32S TcbdInitBuffer(TcbdDevice_t *_device)
{
    I32S i, ret = 0;
    I08U buffEnValue = 0, buffInitValue = 0;

    I32U bufferSize[] = { 
        TCBD_BUFFER_A_SIZE, TCBD_BUFFER_B_SIZE, TCBD_BUFFER_C_SIZE, TCBD_BUFFER_D_SIZE };
    I08U bufferInit[] = { 
        TCBD_OBUFF_BUFF_A_INIT, TCBD_OBUFF_BUFF_B_INIT, TCBD_OBUFF_BUFF_C_INIT, TCBD_OBUFF_BUFF_D_INIT };
    I32U bufferEn[] = {
        TCBD_OBUFF_CONFIG_BUFF_A_EN | TCBD_OBUFF_CONFIG_BUFF_A_CIRCULAR,
        TCBD_OBUFF_CONFIG_BUFF_B_EN | TCBD_OBUFF_CONFIG_BUFF_B_CIRCULAR,
        TCBD_OBUFF_CONFIG_BUFF_C_EN | TCBD_OBUFF_CONFIG_BUFF_C_CIRCULAR,
        TCBD_OBUFF_CONFIG_BUFF_D_EN | TCBD_OBUFF_CONFIG_BUFF_D_CIRCULAR,
    };

    for(i = 0; i < sizeof(bufferSize)/sizeof(I32U); i++)
    {
        if(bufferSize[i])
        {
            buffInitValue |= bufferInit[i];
            buffEnValue |= bufferEn[i];
        }
    } 
    TcbdDebug(DEBUG_DRV_COMP, "buffer init : 0x%02X, buffer en:0x%02X\n", buffInitValue, buffEnValue);
    ret |= TcbdRegWrite(_device, TCBD_OBUFF_INIT, buffInitValue);
    ret |= TcbdRegWrite(_device, TCBD_OBUFF_CONFIG, buffEnValue);
    return ret;
}

I32S TcbdInitBufferRegion(TcbdDevice_t *_device)
{
    I32S i, ret = 0;
    I32U addrStart, addrEnd;
#if defined(__DEBUG_TCBD__)
    I16U regVal[2];
#endif  //__DEBUG_TCBD__
    I32U bufferSize[] = { 
        TCBD_BUFFER_A_SIZE, TCBD_BUFFER_B_SIZE, TCBD_BUFFER_C_SIZE, TCBD_BUFFER_D_SIZE };
    I08U bufferRegAddr[] = { 
        TCBD_OBUFF_A_SADDR, TCBD_OBUFF_B_SADDR, TCBD_OBUFF_C_SADDR, TCBD_OBUFF_D_SADDR };
    I32U bufferAddrSet[] = {
        (PHY_MEM_ADDR_A_START>>2), (PHY_MEM_ADDR_A_END>>2), 
        (PHY_MEM_ADDR_B_START>>2), (PHY_MEM_ADDR_B_END>>2),
        (PHY_MEM_ADDR_C_START>>2), (PHY_MEM_ADDR_C_END>>2), 
        (PHY_MEM_ADDR_D_START>>2), (PHY_MEM_ADDR_D_END>>2),
    };

    for(i = 0; i < sizeof(bufferSize)/sizeof(I32U); i++)
    {
        if(!bufferSize[i]) continue;

        addrStart = SWAP16(bufferAddrSet[i*2+0]);
        addrEnd   = SWAP16(bufferAddrSet[i*2+1]);
        TcbdDebug(DEBUG_DRV_COMP, "reg:%02x, %c buffer size: %d\n", 
            bufferRegAddr[i], 'A'+i, (bufferAddrSet[i*2+1]<<2)-(bufferAddrSet[i*2+0]<<2)+4);
        ret = TcbdRegWriteExCon(_device, bufferRegAddr[i], (I08U*)&addrStart, 2);
        ret |= TcbdRegWriteExCon(_device, bufferRegAddr[i]+2, (I08U*)&addrEnd, 2);

        if(ret < 0) return ret;
    }
#if defined(__DEBUG_TCBD__)
    for(i = 0; i < 4 && bufferSize[i]; i++)
    {
        TcbdRegReadExCon(_device, bufferRegAddr[i], (I08U*)&regVal[0], 2);
        TcbdRegReadExCon(_device, bufferRegAddr[i]+2, (I08U*)&regVal[1], 2);
        TcbdDebug(DEBUG_DRV_COMP, "%c buffer start:%X, end:%X\n", 
            'A'+i, SWAP16(regVal[0])<<2, SWAP16(regVal[1])<<2 );
    }
#endif //__DEBUG_TCBD__
    TcbdInitBuffer(_device);
    return ret;
}

I32S TcbdChangeMemoryView(TcbdDevice_t *_device, TcbdRemap_t _remap)
{
    return TcbdRegWrite(_device, TCBD_INIT_REMAP, (I08U)_remap);
}

I32S TcbdInitBiasKey(TcbdDevice_t *_device)
{
    I32S ret = 0;

    ret |= TcbdRegWrite(_device, TCBD_OP_XTAL_BIAS, 0x05);
    ret = TcbdRegWrite(_device, TCBD_OP_XTAL_BIAS_KEY, 0x5E);

    return ret;
}

static I32S TcbdIsOpRunning(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I32U status;

    ret = TcbdRegReadExFix(_device, TCBD_MAIL_FIFO_WIND, (I08U*)&status, 4);

    if(status != 0x1ACCE551)
    {
        TcbdDebug(DEBUG_ERROR, " # Error access mail [0x%X] \n", status);
        return -TCERR_CANNOT_ACCESS_MAIL;
    } 

    return ret;
}

I32S TcbdResetComponent(TcbdDevice_t *_device, I08U _compEn, I08U _compRst)
{
    I32S ret = 0;
    _device->processor = _compEn;

    ret |= TcbdRegWrite(_device, TCBD_SYS_EN, _compEn);
    ret |= TcbdRegWrite(_device, TCBD_SYS_RESET, _compRst);

    return ret;
}

I32S TcbdChangeStreamType(TcbdDevice_t *_device, I32U _format)
{
    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_WRITE;
    mail.cmd = MBPARA_SYS_DAB_STREAM_SET;
    mail.count = 1;
    mail.data[0] = _format;
	_device->lastSelectedStream = _format;
    return TcbdSendMail(_device, &mail);
}

I32S TcbdDemodTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz)
{
    I32S ret = 0;
    I32U selectedStream = 0;

    switch(_device->periType)
    {
        case PeriTypeSpiOnly:
            selectedStream = /*STREAM_SET_GARBAGE(TCBD_THRESHOLD_FIC) | */STREAM_TYPE_ALL;
            break;
        case PeriTypeSts:
            selectedStream = STREAM_TYPE_ALL;
            break;
        default:
            TcbdDebug(DEBUG_ERROR, "not implemented!\n");
            return -1;
    }
#if defined(__STATUS_IN_INTERNAL__)
    selectedStream &= ~(STREAM_TYPE_STATUS);
#endif /* __STATUS_IN_INTERNAL__ */

    ret = TcbdChangeStreamType(_device, selectedStream);

#if defined(__TCBD_CLOCK_19200KHZ__)
    ret |= TcbdSendSpurTable(_device, _freqKhz);
#endif /* __TCBD_CLOCK_19200KHZ__ */
    ret |= TcbdSendFreqeuncy(_device, _freqKhz);
    if(ret < 0)
    {
        TcbdDebug(DEBUG_ERROR, "Failed to send spur and freq to op, err:%d\n", ret);
        return ret;
    }

    if(_device->currBand != _device->prevBand)
    {
        TcbdSendAgcTable(_device, _device->currBand);
    }

    ret |= TcbdWarmBoot(_device);
    return ret;
}


I32S TcbdColdBoot(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I16U newPc = SWAP16(START_PC_OFFSET);

    ret = TcbdChangeMemoryView(_device, OpRam0EpRam1Pc2); 
    ret |= TcbdRegWriteExCon(_device, TCBD_INIT_PC, (I08U*)&newPc, 2);

    ret |= TcbdRegWrite(_device, TCBD_SYS_RESET, TCBD_SYS_COMP_DSP);
    ret |= TcbdRegWrite(_device, TCBD_SYS_EN, TCBD_SYS_COMP_DSP | TCBD_SYS_COMP_EP);

    if(ret < 0) return ret;

    return TcbdIsOpRunning(_device);
}

inline static I32S TcbdStart(TcbdDevice_t *_device)
{
    I32S ret = 0;
    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_READ;
    mail.cmd = MBPARA_SYS_START;
    mail.count = 1;
    mail.data[0] = 0x19;
    mail.data[0] |= 0x2 << 28;
    ret |= TcbdSendMail(_device, &mail);
    ret |= TcbdRecvMail(_device, &mail);

    return ret;
}

I32S TcbdWarmBoot(TcbdDevice_t *_device)
{
    I32S ret = 0;
    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_WRITE;
    mail.cmd = MBPARA_SYS_WARMBOOT;
    mail.count = 0;

    ret |= TcbdSendMail(_device, &mail);
    ret |= TcbdRecvMail(_device, &mail);

    if(ret < 0 || mail.data[0] != 0x1ACCE551)
    {
        TcbdDebug(DEBUG_ERROR, " # Could not warm boot! [%08X]\n", mail.data[0]);
        return -TCERR_WARMBOOT_FAIL;
    }
    ret = TcbdStart(_device);

    if(ret >= 0) 
        TcbdDebug(DEBUG_DRV_COMP, "Warm boot succeed! [0x%X] ret:%d\n", mail.data[0], ret);
    return ret;
}
