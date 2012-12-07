/**
 @file tcbd_drv_component.h
 APIs for controling internal component of device 
*/
#ifndef __TCBD_DRV_COMPONENT_H__
#define __TCBD_DRV_COMPONENT_H__

typedef enum _TcbdRemap_t
{
    EpRam0Ram1 = 0, /**< Ep can access RAM0 and RAM1 */
    EpRam0Ram1Pc0,  /**< Ep can access RAM0, RAM1 and initial pc is 0 */
    OpRam0Ram1Pc2,  /**< Op can access RAM0, RAM1 and initial pc is 0x2000*/
    OpRam0EpRam1Pc2 /**< Op can access RAM0. Ep can access RAM1. Initial pc is 0x2000 */
} TcbdRemap_t;

TCBB_FUNC I32S TcbdInitBiasKey(TcbdDevice_t *_device);

TCBB_FUNC I32S TcbdSendSpurTable(TcbdDevice_t *_device, I32S _freqKhz);
TCBB_FUNC I32S TcbdSendAgcTable(TcbdDevice_t *_device, TcbdBand_t _bandType);
TCBB_FUNC I32S TcbdSendFreqeuncy(TcbdDevice_t *_device, I32S _freqKhz);
TCBB_FUNC I32S TcbdSendServiceInfo(TcbdDevice_t *_device);

TCBB_FUNC I32S TcbdRecvBootCodeVersion(TcbdDevice_t *_device, I32U *_bootVersion);

TCBB_FUNC I32S TcbdInitBuffer(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdInitBufferRegion(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdChangeMemoryView(TcbdDevice_t *_device, TcbdRemap_t _remap);

TCBB_FUNC I32S TcbdResetComponent(TcbdDevice_t *_device, I08U _compEn, I08U _compRst);

TCBB_FUNC I32S TcbdDemodTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz);

TCBB_FUNC I32S TcbdColdBoot(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdWarmBoot(TcbdDevice_t *_device);

TCBB_FUNC I32S TcbdChangeChipAddress(TcbdDevice_t *_device, I08U addr);
TCBB_FUNC I32S TcbdEnableSlaveCommandAck(TcbdDevice_t *_device);

TCBB_FUNC I32S TcbdEnablePeri(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdDisablePeri(TcbdDevice_t *_device);
TCBB_FUNC I32S TcbdSelectPeri(TcbdDevice_t *_device, TcbdPeri_t _periType);
TCBB_FUNC I32S TcbdInitGpioForPeri(TcbdDevice_t *_device, TcbdPeri_t _periType);

TCBB_FUNC I32S TcbdInitDiversityIo(TcbdDevice_t *_device, TcbdDivIo_t _divIo);

#endif //__TCBD_DRV_COMPONENT_H__
