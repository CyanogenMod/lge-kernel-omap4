
#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_io.h"

static TcbdIo_t TcbdIo;

TcbdIo_t *TcbdGetIoStruct(void)
{
    return &TcbdIo;
}

I32S TcbdIoOpen(TcbdDevice_t *_device)
{
    I32S ret = 0;

    if(TcbdIo.Open == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    TcpalCreateSemaphore(&TcbdIo.sem, "TcbdIoSemaphore", 0);

    ret = TcbdIo.Open();
    if(ret < 0) 
        return ret;

    return ret;
}

I32S TcbdIoClose(TcbdDevice_t *_device)
{
    I32S ret = 0;

    if(TcbdIo.Close == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    TcpalDeleteSemaphore(&TcbdIo.sem);

    ret = TcbdIo.Close();
    TcpalMemorySet(_device, 0, sizeof(TcbdDevice_t));

    return ret;
}

I32S TcbdRegRead(TcbdDevice_t *_device, I08U _addr, I08U *_data)
{
    I32S ret;

    if(TcbdIo.RegRead == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    if(_device == NULL || _data == NULL) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegRead(_addr, _data);
    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdRegWrite(TcbdDevice_t *_device, I08U _addr, I08U _data)
{
    I32S ret;

    if(TcbdIo.RegWrite == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegWrite(_addr, _data);
    TcpalSemaphoreUnLock(&TcbdIo.sem);

    return ret;
}

I32S TcbdRegReadExCon(TcbdDevice_t *_device, I08U _addr, I08U *_data, I32S _size)
{
    I32S ret;

    if(TcbdIo.RegReadExCon == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    if(_device == NULL || _data == NULL) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegReadExCon(_addr, _data, _size);
    TcpalSemaphoreUnLock(&TcbdIo.sem);

    return ret;
}

I32S TcbdRegWriteExCon(TcbdDevice_t *_device, I08U _addr, I08U *_data, I32S _size)
{
    I32S ret;

    if(TcbdIo.RegWriteExCon == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    if(_device == NULL || _data == NULL || _size <= 0) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegWriteExCon(_addr, _data, _size);
    TcpalSemaphoreUnLock(&TcbdIo.sem);

    return ret;
}

I32S TcbdRegReadExFix(TcbdDevice_t *_device, I08U _addr, I08U *_data, I32S _size)
{
    I32S ret;

    if(TcbdIo.RegReadExFix == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    if(_device == NULL || _data == NULL || _size <= 0) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegReadExFix(_addr, _data, _size);
    TcpalSemaphoreUnLock(&TcbdIo.sem);

    return ret;
}

I32S TcbdRegWriteExFix(TcbdDevice_t *_device, I08U _addr, I08U *_data, I32S _size)
{
    I32S ret;

    if(TcbdIo.RegWriteExFix == NULL) 
        return -TCERR_IO_NOT_INITIALIZED;

    if(_device == NULL || _data == NULL || _size <= 0) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;
    ret = TcbdIo.RegWriteExFix(_addr, _data, _size);
    TcpalSemaphoreUnLock(&TcbdIo.sem);

    return ret;
}

I32S TcbdMemWrite(TcbdDevice_t *_device, I32U _addr, I08U *_data, I32U _size)
{
    I32S ret = 0;
    I32U addr = SWAP32(_addr);
    I16U size = SWAP16((I16U)(_size>>2));
    I08U ctrlData;

    if(_size <= 0 || _data == NULL || _device == NULL) 
        return -TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;

    ctrlData = TCBD_CMDDMA_DMAEN | TCBD_CMDDMA_WRITEMODE | TCBD_CMDDMA_CRC32EN;
    ret |= TcbdIo.RegWrite(TCBD_CMDDMA_CTRL, ctrlData);
    ret |= TcbdIo.RegWriteExCon(TCBD_CMDDMA_SADDR, (I08U*)&addr, sizeof(I32U));
    ret |= TcbdIo.RegWriteExCon(TCBD_CMDDMA_SIZE, (I08U*)&size, sizeof(I16U));
    ctrlData = TCBD_CMDDMA_START_AUTOCLR | TCBD_CMDDMA_INIT_AUTOCLR | 
               TCBD_CMDDMA_CRC32INIT_AUTOCLR | TCBD_CMDDMA_CRC32INIT_AUTOCLR;
    ret |= TcbdIo.RegWrite(TCBD_CMDDMA_STARTCTRL, ctrlData);
    ret |= TcbdIo.RegWriteExFix(TCBD_CMDDMA_DATA_WIND, _data, _size);

    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdMemRead(TcbdDevice_t *_device, I32U _addr, I08U *_data, I32U _size)
{
    I32S ret = 0;
    I32U addr = SWAP32(_addr);
    I16U size = SWAP16((I16U)(_size>>2));
    I08U ctrlData;

    if(_size <= 0 || _data == NULL || _device == NULL) 
        return -TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;

    ctrlData = TCBD_CMDDMA_DMAEN | TCBD_CMDDMA_READMODE;
    ret |= TcbdIo.RegWrite(TCBD_CMDDMA_CTRL, ctrlData);
    ret |= TcbdIo.RegWriteExCon(TCBD_CMDDMA_SADDR, (I08U*)&addr, sizeof(I32U));
    ret |= TcbdIo.RegWriteExCon(TCBD_CMDDMA_SIZE, (I08U*)&size, sizeof(I16U));
    ctrlData = TCBD_CMDDMA_START_AUTOCLR | TCBD_CMDDMA_INIT_AUTOCLR | TCBD_CMDDMA_CRC32INIT_AUTOCLR;
    ret |= TcbdIo.RegWrite(TCBD_CMDDMA_STARTCTRL, ctrlData);
    ret |= TcbdIo.RegReadExFix(TCBD_CMDDMA_DATA_WIND, _data, _size);

    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdRfRegWrite(TcbdDevice_t *_device, I08U _addr, I32U _data)
{
    I32S ret = 0;
    I32U data = SWAP32(_data);

    if(_device == NULL) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;

    ret |= TcbdIo.RegWrite(TCBD_RF_CFG0, TCBD_RF_MANAGE_ENABLE|TCBD_RF_WRITE);
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG2, _addr);
    ret |= TcbdIo.RegWriteExCon(TCBD_RF_CFG3, (I08U*)&data, sizeof(I32U));
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG1, TCBD_RF_ACTION);
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG0, TCBD_RF_MANAGE_DISABLE);

    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdRfRegRead(TcbdDevice_t *_device, I32U _addr, I32U *_data)
{
    I32S ret = 0;
    I32U data = 0;

    if(_device == NULL) 
        return TCERR_INVALID_ARG;

    TcpalSemaphoreLock(&TcbdIo.sem);
    TcbdIo.chipAddr = _device->chipAddr;

    ret |= TcbdIo.RegWrite(TCBD_RF_CFG0, TCBD_RF_MANAGE_ENABLE|TCBD_RF_READ);
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG2, _addr);
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG1, TCBD_RF_ACTION);
    ret |= TcbdIo.RegReadExCon(TCBD_RF_CFG3, (I08U*)&data, sizeof(I32U));
    ret |= TcbdIo.RegWrite(TCBD_RF_CFG0, TCBD_RF_MANAGE_DISABLE);

    *_data = SWAP32(data);

    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdSendMail(TcbdDevice_t *_device, TcbdMail_t *_mail)
{
    I32S ret = 0, count;
    I08U mailData[32];
    I08U regData = 0;
    TcpalTime_t timeTick, elapsed;

    TcpalSemaphoreLock(&TcbdIo.sem);

    ret = TcbdIo.RegWrite(TCBD_MAIL_CTRL, TCBD_MAIL_INIT);

    *((I32U*)mailData) = (MB_HOSTMAIL<<24) | (_mail->count<<20) | (_mail->flag<<19) | (MB_ERR_OK<<16) | _mail->cmd;

    count = MIN(_mail->count, MAX_MAIL_COUNT);
    if(count > 0)
        TcpalMemoryCopy(mailData + sizeof(I32U), _mail->data, count*sizeof(I32U)); 
    
    ret |= TcbdIo.RegWriteExFix(TCBD_MAIL_FIFO_WIND, mailData, sizeof(I32U) + count*sizeof(I32U));
    ret |= TcbdIo.RegWrite(TCBD_MAIL_CTRL, TCBD_MAIL_HOSTMAILPOST);
    if(ret < 0)
        goto exitTcbdSendMail;

    timeTick = TcpalGetCurrentTimeMs();
    do
    {
        elapsed = TcpalGetTimeIntervalMs(timeTick);
        if(elapsed > (TcpalTime_t)MAX_TIME_TO_WAIT_MAIL)
        {
            ret = -TCERR_WAIT_MAIL_TIMEOUT;
            goto exitTcbdSendMail;
        }
        ret = TcbdIo.RegWrite(TCBD_MAIL_FIFO_W, 0x5E);    /* latch mail status to register      */
        ret |= TcbdIo.RegRead(TCBD_MAIL_FIFO_W, &regData); /* read ratched status from register  */
        if(ret < 0)  break;
    } while( !(regData & 0x1) ); /* check fifo status */

    TcbdDebug(DEBUG_DRV_IO, "cmd:0x%X, count:%d, elapsed time:%Lu\n", _mail->cmd, count, elapsed);

exitTcbdSendMail:
    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdRecvMail(TcbdDevice_t *_device, TcbdMail_t *_mail)
{
    I32S ret = 0;
    I32S mailHdr;
    I08U mailData[32] = {0, };
    I08U regData = 0;
    I08U bytesToRead;
    TcpalTime_t timeTick, elapsed;

    TcpalSemaphoreLock(&TcbdIo.sem);

    timeTick = TcpalGetCurrentTimeMs();
    do
    {
        ret = TcbdIo.RegWrite(TCBD_MAIL_FIFO_R, 0x5E);    /* latch mail status to register      */
        ret |= TcbdIo.RegRead(TCBD_MAIL_FIFO_R, &regData); /* read ratched status from register  */

        elapsed = TcpalGetTimeIntervalMs(timeTick);
        if(elapsed > (TcpalTime_t)MAX_TIME_TO_WAIT_MAIL)
            ret = -TCERR_WAIT_MAIL_TIMEOUT;

        if(ret < 0) 
            goto exitTcbdRecvMail;
    }
    while((regData & 0xFC) < 3);                      /* check fifo status */
    bytesToRead = (regData>>2) & 0x3F;
    TcbdDebug(DEBUG_DRV_IO, "cmd:0x%X, bytesToRead:%d, elapsed time:%Lu\n", _mail->cmd, bytesToRead, elapsed);

    ret = TcbdIo.RegReadExFix(TCBD_MAIL_FIFO_WIND, mailData, bytesToRead);
    if(bytesToRead == 4) //only warm boot cmd
    {
        TcpalMemoryCopy(_mail->data, mailData, bytesToRead);
        goto exitTcbdRecvMail; 
    }

    mailHdr = *((I32U*)mailData);
    if( (mailHdr>>24) != MB_SLAVEMAIL)
    {
        TcbdDebug(DEBUG_ERROR, "Error : cmd=0x%X bytesToRead=%d\n", _mail->cmd, bytesToRead);
        TcbdDebug(DEBUG_ERROR, " [0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X][0x%02X]\n",
            mailData[0], mailData[1], mailData[2], mailData[3], mailData[4], mailData[5], mailData[6], mailData[7]);

        ret = -TCERR_BROKEN_MAIL_HEADER;
        goto exitTcbdRecvMail; 
    }
    _mail->cmd = mailHdr & 0xFFFF;
    _mail->status = (mailHdr>>16) & 0x7;
    if(_mail->status)
    {
        TcbdDebug(DEBUG_ERROR, "Mail Error : status=0x%X, cmd=0x%X\n", _mail->status, _mail->cmd);
        ret = -TCERR_UNKNOWN_MAIL_STATUS;
        goto exitTcbdRecvMail;
    }
    _mail->count = (bytesToRead>>2) - 1;
    TcpalMemoryCopy(_mail->data, mailData + 4, bytesToRead-4);

exitTcbdRecvMail:
    TcpalSemaphoreUnLock(&TcbdIo.sem);
    return ret;
}

I32S TcbdReadMailBox(TcbdDevice_t *_device, I16U cmd, I32S len, I32U* data)
{
    I32S ret = 0;
    TcbdMail_t mail = {0, };

    mail.flag = MB_CMD_READ;
    mail.cmd = MBPARA_SYS_SYNC;
    mail.count = 1;
    ret = TcbdSendMail(_device, &mail);
    if(ret < 0) 
    {
        TcbdDebug(DEBUG_ERROR, "failed to send mail! %d\n", ret);
        goto exitReadMailBox;
    }

    ret = TcbdRecvMail(_device, &mail);
    if(ret < 0) 
    {
        TcbdDebug(DEBUG_ERROR, "failed to recv mail! %d\n", ret);
        goto exitReadMailBox;
    }

    TcpalMemoryCopy((void*)data, (void*)mail.data, len*sizeof(I32U));

exitReadMailBox:
    return ret;
}
