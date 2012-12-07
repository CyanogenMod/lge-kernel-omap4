#ifndef __TCBD_FIC_DECODER_H__
#define __TCBD_FIC_DECODER_H__

#define	MAX_SERVICE_NUM		10	//16
#define	MAX_SUBCH_NUM		10	//32
#define	MAX_SC_NUM	1

typedef struct _TcbdFicDataTime_t
{
    I32U mjd;   ///< Modified Julian Date
    I08U lsi;
    I08U confInd;
    I08U utcFlag;
    I08U hour;
    I08U minute;
    I08U sec;
    I08U millisec;
} TcbdFicDataTime_t;

typedef struct _TcbdFicSch_t
{
    I16U startAddr; ///<Start Address
    I08U subchId;  ///<SubCh ID[5:0]
    I08U slFg;  ///<Short, Long Form
    I08U tableSwitch; ///<UEP Table Switch
    I08U idx;   ///<UEP Table Index
    I16U subchSize;  ///<EEP SubCh Size
    I16U sch_n;
    I16U bitRate; ///<Bit Rate
    I08U prtLvl;  ///<EEP Protection Level
    I08U opt;   ///<EEP option
    I08U tsFlag;  ///<if TS, set '1' else '0'
    I08U tmidScty; ///<tmid_scty
    I08U tmidMatchFlag;
} TcbdFicSch_t;

typedef struct _TcbdFicSc_t
{
    I08U no;
    I08U tmid;
    I16U scid;
    I08U scty;   ///< DSCTy or ASCTy or Upper 6bit of scid
    I08U ps;
    I08U caFlag;
    I08U scStatus;  ///< 3bits is valid. 
    I08U label[16];
    I08U xpadType;
    TcbdFicSch_t subCh;
    I08U sctyTmp;
} TcbdFicSc_t;

/*typedef struct _TcbdFicSelectedSch_t
  {
    I08U num_of_set;      //Num of selected svc or subCh
    I08U subchId[MAX_SERVICE_NUM];
    I32U svc_id[MAX_SUBCH_NUM];
} TcbdFicSelectedSch_t; */


typedef struct _TcbdFicSvc_t
{
    I08U pd;
    I08U svcStatus;    ///< 2bits is valid. 
    I32U sid;
    I08U numOfSvcComp;
    I08U label[16];
    TcbdFicSc_t sc[MAX_SC_NUM];
} TcbdFicSvc_t;

typedef struct _TcbdFicEnsbl_t
{
    I08U changeFlag;
    I08U alFlag;
    I08U esbStatus;    ///< 2bits is valid.
    I08U occurChange;
    I16U eid;
    I16U cifCnt;
    I08U ensembleLabel[16];
    I16U numOfSvc;
    I16U numOfSch;
    I16U numOfSvcLabelEnd;   ///< num of Services which are label found
    I16U numOfSvcLabelBefore;  ///< num of Services of label founding 
    TcbdFicSvc_t svc[MAX_SERVICE_NUM];
} TcbdFicEnsbl_t;

typedef struct _TcbdFicTsSch_t
{
    I32S numOfTs;
    I08U subchId[5];
} TcbdFicTsSch_t;

/*typedef struct _TcbdFicPacket_t
{
    I32S numOfPacket;
    I32U sid[5];
} TcbdFicPacket_t;*/

typedef struct _TcbdFicCrcResult_t
{
    I08U ficCrcErr;   //CRC Err cnt : max 12
    I08U ficCrcGood;   //CRC Good cnt : max 12
} TcbdFicCrcResult_t;

typedef struct _TcbdFicTargetSchId_t
{
    I08U subchId[MAX_SUBCH_NUM];
    I32S checkCnt;
} TcbdFicTargetSchId_t;

TCBB_FUNC void TcbdFicInitDb(void);
TCBB_FUNC I32S TcbdFicRunDecoder(I08U *_fibBuff, I32S _fibBuffSize);
TCBB_FUNC TcbdFicSch_t* TcbdIsSubchExist(I08U _subchId);
TCBB_FUNC I08U TcbdFicGetPrtLvl(TcbdFicSch_t *_subch);
TCBB_FUNC I08U TcbdFicGetBitRate(TcbdFicSch_t *_subch);
TCBB_FUNC I08U TcbdFicGetPrtType(TcbdFicSch_t *_subch);
TCBB_FUNC I32S TcbdFicGetSchSize(TcbdFicSch_t *_subch);
#endif //__TCBD_FIC_DECODER_H__
