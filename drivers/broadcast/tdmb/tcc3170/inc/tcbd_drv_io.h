/**
 @file tcbd_drv_io.h
 APIs for reading or writing register.
*/
#ifndef __TCBD_DRV_COMMON_H__
#define __TCBD_DRV_COMMON_H__
#include "tcbd_drv_register.h"
#include "tcbd_error.h"

#define MB_HOSTMAIL     0x47
#define MB_SLAVEMAIL    0x74

#define DEFAULT_CHIP_ADDRESS 0x50
typedef struct _TcbdIo_t
{
    I32S chipAddr;                                              /**< chip address if diversity or dual mode   */
    TcpalSem_t sem;                                             /**< semaphore for I/O                        */
    I32S (*Open)(void);
    I32S (*Close)(void);
    I32S (*RegWrite)(I08U _addr, I08U _data);                   /**< Write one single register                */
    I32S (*RegRead)(I08U _addr, I08U *_data);                   /**< Read one single register                 */
    I32S (*RegWriteExCon)(I08U _addr, I08U *_data, I32S _size); /**< Write multiple register continuous mode  */
    I32S (*RegReadExCon)(I08U _addr, I08U *_data, I32S _size);  /**< Read multiple register continuous mode   */
    I32S (*RegWriteExFix)(I08U _addr, I08U *_data, I32S _size); /**< Write data to memeory fixed mode         */
    I32S (*RegReadExFix)(I08U _addr, I08U *_data, I32S _size);  /**< Read data from memery fixed mode         */
} TcbdIo_t;

#define MAX_MAIL_COUNT 7
#define MAX_TIME_TO_WAIT_MAIL 1000
typedef struct _TcbdMail_t 
{
    I08U flag;
    I08U count;
    I16U cmd;
    I32U status;
    I32U data[MAX_MAIL_COUNT];
    I08U pad[2];
} TcbdMail_t;

TCBB_FUNC TcbdIo_t *TcbdGetIoStruct(void);
TCBB_FUNC void TcpalSetI2cIoFunction(void);
TCBB_FUNC void TcpalSetCspiIoFunction(void);

TCBB_FUNC I32S TcbdIoOpen(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdIoClose(TcbdDevice_t *_device);

TCBB_FUNC I32S TcbdRegRead(TcbdDevice_t *_handle, I08U _addr, I08U *_data);
TCBB_FUNC I32S TcbdRegWrite(TcbdDevice_t *_handle, I08U _addr, I08U _data);
TCBB_FUNC I32S TcbdRegReadExCon(TcbdDevice_t *_handle, I08U _addr, I08U *_data, I32S _size);
TCBB_FUNC I32S TcbdRegWriteExCon(TcbdDevice_t *_handle, I08U _addr, I08U *_data, I32S _size);
TCBB_FUNC I32S TcbdRegReadExFix(TcbdDevice_t *_handle, I08U _addr, I08U *_data, I32S _size);
TCBB_FUNC I32S TcbdRegWriteExFix(TcbdDevice_t *_handle, I08U _addr, I08U *_data, I32S _size);
TCBB_FUNC I32S TcbdMemWrite(TcbdDevice_t *_handle, I32U _addr, I08U *_data, I32U _size);
TCBB_FUNC I32S TcbdMemRead(TcbdDevice_t *_handle, I32U _addr, I08U *_data, I32U _size);
TCBB_FUNC I32S TcbdRfRegWrite(TcbdDevice_t *_handle, I08U _addr, I32U _data);
TCBB_FUNC I32S TcbdRfRegRead(TcbdDevice_t *_handle, I32U _addr, I32U *_data);

TCBB_FUNC I32S TcbdSendMail(TcbdDevice_t *_device, TcbdMail_t *_mail);
TCBB_FUNC I32S TcbdRecvMail(TcbdDevice_t *_device, TcbdMail_t *_mail);
TCBB_FUNC I32S TcbdReadMailBox(TcbdDevice_t *_device, I16U cmd, I32S len, I32U* data);
#endif //__TCBD_DRV_COMMON_H__
