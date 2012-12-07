/**
 @file tcbd_api_common.h  
 Primitive API for end user
*/
#ifndef __TCBD_API_COMMON_H__
#define __TCBD_API_COMMON_H__

#include "tcbd_diagnosis.h"

/**
 @defgroup ProtectionType
*/
/**@{*/
#define PROTECTION_TYPE_UEP 0
#define PROTECTION_TYPE_EEP 1
/**@}*/

/**
 @defgroup ProtectionLevel
 Protection type UEP
 <table>
    <tr><td>UEP1</td><td>UEP2</td><td>UEP3</td><td>UEP4</td></tr>
    <tr><td>0x00</td><td>0x01</td><td>0x02</td><td>0x03</td></tr></table>

 Protection type EEP
 <table>
    <tr><td>EEP1A</td><td>EEP2A</td><td>EEP3A</td><td>EEP4A</td>
    <td>EEP1B</td><td>EEP2B</td><td>EEP3B</td><td>EEP4B</td></tr>
    <tr><td>0x00</td><td>0x01</td><td>0x02</td><td>0x03</td>
    <td>0x04</td><td>0x05</td><td>0x06</td><td>0x07</td></tr>
*/

/**
 @defgroup ServiceType
 Service type for each sub-channel.
*/
/** @{*/
#define SERVICE_TYPE_DAB     0x01
#define SERVICE_TYPE_DABPLUS 0x02
#define SERVICE_TYPE_DATA    0x03
#define SERVICE_TYPE_DMB     0x04
#define SERVICE_TYPE_FIC     0x07
#define SERVICE_TYPE_FIC_WITH_ERR 0x08
#define SERVICE_TYPE_EWS     0x09
/**@}*/


/**
 @defgroup Bitrate
 If protection type is UEP, the values of bitrate are as following.
 <table>
 <tr><td>Kbps</td>
     <td>32</td><td>48</td><td>56</td><td>64</td><td>80</td><td>96</td><td>112</td>
     <td>128</td><td>160</td><td>192</td><td>224</td><td>256</td><td>320</td><td>384</td></tr>
 <tr><td>value</td>
     <td>0x00</td><td>0x01</td><td>0x02</td><td>0x03</td><td>0x04</td><td>0x05</td><td>0x06</td>
     <td>0x07</td><td>0x08</td><td>0x09</td><td>0x0A</td><td>0x0B</td><td>0x0C</td><td>0x0D</td></tr>
 </table>

 If protection type is EEP, the values of bitrate are as following.   
 <table>
 <tr><td>kbps</td>
     <td>1</td><td>2</td><td>3</td><td>...</td><td>126</td></tr> 
 <tr><td>value</td>
     <td>0x01</td><td>0x02</td><td>0x03</td><td>...</td><td>0x7E</td><tr>
 </table>
 0x00
*/

typedef struct _TcbdTdmbService_t
{
    I32S type;      /**< Type of service. Please refer to @ref ServiceType*/ 
    I32S pType;     /**< Type of protection Please refer to @ref ProtectionType*/
    I32S subchId;   /**< Sub-channel id */
    I32S cuSize;    /**< Size of CU(Capacity Unit) or sub-channel size*/ 
    I32S startCu;   /**< Start address of CU(Capacity Unit) */
    I32S reconfig;  /**< Must be set 0x02*/
    I32S rsOn;      /**< If type is DMB, it must be 0x01, or else 0x00 */
    I32S pLevel;    /**< Protection level Please refer @ref ProtectionLevel*/
    I32S bitrate;   /**< Bitrate. Please refer @ref Bitrate*/
} TcbdService_t;

typedef enum _TcbdPeri_t
{
    PeriTypeSpiSlave = 1,  /**< The stream can be read using SPI(slave)  */
    PeriTypeSpiMaster, /**< The stream can be read using SPI(master) */
    PeriTypePts,       /**< The stream can be read using PTS */
    PeriTypeSts,       /**< The stream can be read using STS */
    PeriTypeHpi,       /**< The stream can be read using HPI */
    PeriTypeSpiOnly    /**< The stream can be read using SPI. And command use same SPI interface. */
} TcbdPeri_t;

typedef enum _TcbdBand_t 
{
   BandTypeBand3 = 1,  /**< band III */
   BandTypeLband       /**< L band */
   //BandTypeVhf,
} TcbdBand_t;

typedef enum _TcbdDivIo_t 
{
    DivIoTypeSingle = 0,
    DivIoTypeDual,
} TcbdDivIo_t;

typedef enum _TcbdIntrMode_t
{
    IntrModeLevelHigh,   /**< Interrupt will be trigered at high level */
    IntrModeLevelLow,    /**< Interrupt will be trigered at low level */
    IntrModeEdgeRising,  /**< Interrupt will be trigered at rising edge */
    IntrModeEdgeFalling  /**< Interrupt will be trigered at falling edge */
} TcbdIntrMode_t;

#define MAX_NUM_SERVICE 6
typedef struct _TcbdServiceCtrl_t
{
    I32S onAir[MAX_NUM_SERVICE];         /* support up to 6 services simultineously */
    I32S serviceIndex[MAX_NUM_SERVICE];
    I32S serviceCount;
    I32U serviceInfo[MAX_NUM_SERVICE*2];
} TcbdServiceCtrl_t;

typedef struct _TcbdDevice_t
{
    I08U chipAddr;                 /**< Chip address for diversity or dual mode*/
    I32U mainClock;
    I08U processor;                /**< Activated interrnal processor */
    I08U pllData[2];

	I32U lastSelectedBuffer;
	I32U lastSelectedStream;
	I08U enCmdFifo;
    I32U intrThreshold;            /**< Interrupt threshold */
#if defined(__READ_VARIABLE_LENGTH__)
    I32U sizeMoreRead;
#endif
	I32U isRecvFic;
    TcpalTime_t slut;              /**< Last update time of status*/
    TcbdPeri_t periType;           /**< Peripheral type for stream */
    TcbdBand_t currBand;           
    TcbdBand_t prevBand;

    I32S lastFrequency;            /**< Last set frequency */
    TcbdServiceCtrl_t serviceCtrl; /**< Registered service infomation */
} TcbdDevice_t;


/**
 Initialize PLL data for Baseband.
 @param[in] _device Instance of device
 @param[in] _pllData Integer array of pll data 
   
 _pllData 
 - _pllData[0] = WAIT_TIME
 - _pllData[1] = PLL_P
 - _pllData[2] = PLL_M
 - _pllData[2] = PLL_S
 - _pllData[4] = Oscillater Clock

 PLL Table for TCC3170 
 <table>
 <tr><td> WAIT_TIME</td> <td>PLL_P</td> <td>PLL_M</td> <td>PLL_S</td> <td>OSC_CLOCK</td>
 <tr><td> 0x60     </td> <td>0x00 </td> <td>0x0F </td> <td>0x03 </td> <td>19200</td>
 </table>

 @return sucess 0, fail -@ref ListOfErrorCode
 */
TCBB_FUNC I32S TcbdInitPll(TcbdDevice_t *_device, I32U *_pllData);

/**
 Change the type of stream. Types are
 - DMB : STREAM_TYPE_DMB
 - DAB : STREAM_TYPE_DAB
 - FIC : STREAM_TYPE_FIC
 - STATUS : STREAM_TYPE_STATUS
 
 You can combine the above types. For more information @ref TypesOfStream
 @param _device Instance of device
 @param _type type of stream
 
 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdChangeStreamType(TcbdDevice_t *_device, I32U _type);

/**
 @remark Download boot code for internal DSP.  Boot code is distributed with this SDK.
 @remark Boot code of TCC3170 is TCC3170_BOOT_TDMB.h. 
 @param _device Instance of device
 @param _bootCode Pointer of boot code
 @param _size Size of boot code
 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdDownloadBootCode(TcbdDevice_t *_device, I08U *_bootCode, I32S _size);


TCBB_FUNC I32S TcbdEnableIrq(TcbdDevice_t *_device, I08U _enBit);
TCBB_FUNC I32S TcbdDisableIrq(TcbdDevice_t *_device, I08U _mask);

/**
 Change the mode of interrupt. 
 @param _device Instance of device
 @param _mode Interrupt mode. For more information of mode, please refer #_TcbdIntrMode_t
 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdChangeIrqMode(TcbdDevice_t *_device, TcbdIntrMode_t _mode);

TCBB_FUNC I32S TcbdReadIrqStatus(TcbdDevice_t *_device, I08U *_irqStatus, I08U *_errStatus);
TCBB_FUNC I32S TcbdClearIrq(TcbdDevice_t *_device, I08U _status);

#define ENABLE_CMD_FIFO 1
#define DISABLE_CMD_FIFO 0
/**
 Initialize stream configuration. 
 @param _device Instance of device.
 @param _useCmdFifo Determin whether or not to use command FIFO when you read stream. 
                    If you use SPI interface for reading stream, you should use command FIFO.
 - ENABLE_CMD_FIFO 
 - DISABLE_CMD_FIFO
 @param _bufferMask Mask unused buffer. For more information, please refer @ref StreamDataConfig
 @param _threshold Threshold for triggering interrupt. The threshold can be set up to 8Kbyes.
 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdInitStreamDataConfig(TcbdDevice_t *_device, I08U _useCmdFifo, I08U _bufferMask, I32U _threshold);

/**
 Tune frequency.
 @param _device Instance of device
 @param _freqKhz frequency
 @param _bwKhz bandwidth

 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz);

/**
 Wait until the frequency is tuned.
 @param _device Instance of device
 @param _status Status.
 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdWaitTune(TcbdDevice_t *_device, I08U *_status);

/**
 Register sub channel.
 @param _device Instance of device
 @param _service Sub channel information.

 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdRegisterService(TcbdDevice_t *_device, TcbdService_t *_service);

/**
 Unregister sub channel.
 @param _device Instance of device
 @param _service Sub channel information previously registered.

 @return success 0, fail -@ref ListOfErrorCode
*/
TCBB_FUNC I32S TcbdUnregisterService(TcbdDevice_t *_device, TcbdService_t *_service);

TCBB_FUNC I32S TcbdReadStream(TcbdDevice_t *_device, I08U *_buff, I32S *_size);
TCBB_FUNC I32S TcbdReadFicData(TcbdDevice_t *_device, I08U *_buff, I32S _size);
TCBB_FUNC I32S TcbdReadSignalInfo(TcbdDevice_t *_device, TcbdDiagnosis_t *_diagInfo);

#if !defined(__TEST_IRQ_REG_ONCE__)
TCBB_FUNC I32S TcpalRegisterIrqHandler(TcbdDevice_t *_device);
#else
TCBB_FUNC I32S TcpalRegisterIrqHandler(void);
#endif
TCBB_FUNC I32S TcpalUnRegisterIrqHandler(void);
TCBB_FUNC I32S TcpalIrqEnable(void);
TCBB_FUNC I32S TcpalIrqDisable(void);
#endif //__TCBD_API_COMMON_H__
