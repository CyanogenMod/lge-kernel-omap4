/** 
 @file tcbd_fic_decoder.c
 This file execute all about FIC Decoding. 
*/

#include "tcpal_os.h"
#include "tcpal_types.h"
#include "tcpal_debug.h"

#include "tcbd_fic_decoder.h"

#define MAX_FIC_SIZE      (12 * 32)

typedef struct _TcbdFicBuf_t
{
    I08U  buf[MAX_FIC_SIZE];
    I16U  wrPos;
} TcbdFicBuf_t;

typedef struct _Fig_t{
    I08U cFIGType;
    I08U cFIGLength;
    I08U cFIGData[29];
} Fig_t;

typedef struct _IntntnlTblExtFieldSvcs_t
{
    I08U ECC; 
    I08U sid[16];

} IntntnlTblExtFieldSvcs_t;

typedef struct _IntntnlTblExtField_t
{
    I08U numOfSvc;
    I08U LTO;
    IntntnlTblExtFieldSvcs_t IntntnlTblExtFeldSvcs[16];
} IntntnlTblExtField_t;

typedef struct _InternationalTable_t
{
    I08U ExtFlg; 
    I08U LTO_Unq;
    I08U EnsblLTO;
    I08U EnsblECC;

    I08U InterTableID;
    IntntnlTblExtField_t IntntnlTblExtField[16];
} InternationalTable_t;

static I32U GetBitrate(TcbdFicSch_t * _sch);
static void Fig0_0(void);
static void Fig0_1(I32S *_schTotalNum);
static void Fig0_2(I08U PDFlag);
static void Fig0_3(void);
static void Fig0_5(void);
static void Fig0_8(I08U PDFlag);
static void Fig0_9(void);
static I08U Fig0_10(void);
static void Fig0_13(I08U PDFlag);
static void Fig1_0(void);
static void Fig1_1(void);
static void Fig1_4(void);
static void Fig1_5(void);
static void Fig1_6(void);
static void Fig5(void);
//static void Fig6(void);

static I32U TcbdFigDecode(I32S *_schTotalNum);
static I32U TcbdFibDecode(I08U *_fibBuff, I32S _fibBuffSize);
static void TcbdFicCheckEnsemble(void);
static I32U CheckDuplicateSch(I08U _subchId, I32S _schNum);
static I08U CheckDuplicateSvc(I32U sid, I32S _numOfSvc);
static I08U CheckDuplicateSvcComp(TcbdFicSvc_t *_svc, I08U tmid, I32U _uniqeId);
static I16U Crc16(I08U *buf);
static I32U GetField(I08U _sizeBit);
static void MatchSvc2Sch(void);
static void PutInLabel(I08U *label );

static I32S TcbdFicCnt;
static Fig_t FigList;
static TcbdFicTsSch_t TcbdFicTsBuf;            ///< Save TS sub ch ID
static TcbdFicEnsbl_t TcbdFicEnsbl;              ///< Ensemble Structure for DAB0/DAB1
static TcbdFicSch_t TcbdFicSch[MAX_SUBCH_NUM];   ///< Sub Channel Structure
static TcbdFicCrcResult_t TcbdFicCrcResult;

//static TcbdFicTargetSchId_t TdmbTarget;
static TcbdFicDataTime_t DateTime;

//static TcbdFicSelectedSch_t TcbdFicSelectedSch;

static InternationalTable_t InternationalTable;
static I08U TcbdFicUserStatus = 0;
static I08U BitPosition;
static I08U SchDevideValue[8] = {12,8,6,4,27,21,18,15};
static I08U BitrateValue[64] = 
{
    8,    //BitRate   =  32
    8,    //BitRate   =  32
    8,    //BitRate   =  32
    8,    //BitRate   =  32
    8,    //BitRate   =  32
    12,   //BitRate   =  48
    12,   //BitRate   =  48
    12,   //BitRate   =  48
    12,   //BitRate   =  48
    12,   //BitRate   =  48
    14,   //BitRate   =  56
    14,   //BitRate   =  56
    14,   //BitRate   =  56
    14,   //BitRate   =  56
    16,   //BitRate   =  64
    16,   //BitRate   =  64
    16,   //BitRate   =  64
    16,   //BitRate   =  64
    16,   //BitRate   =  64
    20,   //BitRate   =  80
    20,   //BitRate   =  80
    20,   //BitRate   =  80
    20,   //BitRate   =  80
    20,   //BitRate   =  80
    24,   //BitRate   =  96
    24,   //BitRate   =  96
    24,   //BitRate   =  96
    24,   //BitRate   =  96
    24,   //BitRate   =  96
    28,   //BitRate   = 112
    28,   //BitRate   = 112
    28,   //BitRate   = 112
    28,   //BitRate   = 112
    32,   //BitRate   = 128
    32,   //BitRate   = 128
    32,   //BitRate   = 128
    32,   //BitRate   = 128
    32,   //BitRate   = 128
    40,   //BitRate   = 160
    40,   //BitRate   = 160
    40,   //BitRate   = 160
    40,   //BitRate   = 160
    40,   //BitRate   = 160
    48,   //BitRate   = 192
    48,   //BitRate   = 192
    48,   //BitRate   = 192
    48,   //BitRate   = 192
    48,   //BitRate   = 192
    56,   //BitRate   = 224
    56,   //BitRate   = 224
    56,   //BitRate   = 224
    56,   //BitRate   = 224
    56,   //BitRate   = 224
    64,   //BitRate   = 256
    64,   //BitRate   = 256
    64,   //BitRate   = 256
    64,   //BitRate   = 256
    64,   //BitRate   = 256
    80,   //BitRate   = 320
    80,   //BitRate   = 320
    80,   //BitRate   = 320
    96,   //BitRate   = 384
    96,   //BitRate   = 384
    96,   //BitRate   = 384
}; 

/*************************************************************************************************  
*                       UEP Table   
*   +---------------+-----------+-----------------------------------+
*   |Bit rate(4bits)|Protect Lv |       SubCh Size(9bits)           |
*   |---------------|---+---+---|---+---+---+---+---+---+---+---+---|
*   | x | x | x | x | x | x | x | x | x | x | x | x | x | x | x | x |
*   +---------------+-----------+-----------------------------------+
*
***************************************************************************************************/
static I16U UepTable[64] = { 
    0x0810, 0x0615, 0x0418, 0x021d, 0x0023, 0x1818, 0x161d, 0x1423, 
    0x122a, 0x1034, 0x281d, 0x2623, 0x242a, 0x2234, 0x3820, 0x362a, 
    0x3430, 0x323a, 0x3046, 0x4828, 0x4634, 0x443a, 0x4246, 0x4054, 
    0x5830, 0x563a, 0x5446, 0x5254, 0x5068, 0x683a, 0x6646, 0x6454, 
    0x6268, 0x7840, 0x7654, 0x7460, 0x7274, 0x708c, 0x8850, 0x8668, 
    0x8474, 0x828c, 0x80a8, 0x9860, 0x9674, 0x948c, 0x92a8, 0x90d0, 
    0xa874, 0xa68c, 0xa4a8, 0xa2d0, 0xa0e8, 0xb880, 0xb6a8, 0xb4c0, 
    0xb2e8, 0xb118, 0xc8a0, 0xc6d0, 0xc318, 0xd8c0, 0xd518, 0xd1a0 };

static TcbdFicBuf_t TcbdFicBuf;


/* --------------------------------------------------------------------------------------------
   FUNCTION    
   TcbdFicCheckEnsemble

   DESCRIPTION
   This function checks the running condition of FIC Decoding.
   If you want to change the condition of completing status of FIC decoding, 
   you can modify this function.

   There is a fic decoding status check variable which name is esbStatus(initial of ensenble status).
   The esbStatus has bit allocated values and currenlty 1 bit is valid.

   Basically, when we get service labels as many as services (num_of_svc  ==num_of_svc_label)
   we decide that fic decode is completed and set the 1st bit of esbStatus.

   PARAMETER
   [in] numOfSvc(TcbdFicEnsbl)
   [in] numOfSvcLabelEnd(TcbdFicEnsbl)
   [in/out] esbStatus(TcbdFicEnsbl).

   RETURN VALUE
   -------------------------------------------------------------------------------------------- */
void  TcbdFicCheckEnsemble()
{
    TcbdDebug(0, "numOfSvc:%d, numOfSvcLabelEnd:%d, numOfSch:%d\n", 
            (int)TcbdFicEnsbl.numOfSvc, 
            (int)TcbdFicEnsbl.numOfSvcLabelEnd, 
            (int)TcbdFicEnsbl.numOfSch);

    if(!(TcbdFicEnsbl.esbStatus & 0x1) && 
            (TcbdFicEnsbl.numOfSvc == TcbdFicEnsbl.numOfSvcLabelEnd) 
            && (TcbdFicEnsbl.numOfSvc > 0) && (TcbdFicEnsbl.numOfSvc == TcbdFicEnsbl.numOfSch))
    {
        TcbdFicEnsbl.esbStatus |= 1;  ///<Data Enough !! No more Svc& Subchannel Find
        return ;
    }
    else if((TcbdFicCnt > 100) &&  (
                (TcbdFicEnsbl.numOfSvc > 0) && (TcbdFicEnsbl.numOfSvc == TcbdFicEnsbl.numOfSch)))
    {
        TcbdFicEnsbl.esbStatus = 3;  ///<Data Enough !! No more Svc& Subchannel Find. No more label Update
    }
}

/* --------------------------------------------------------------------------------------------
   FUNCTION    
   TcbdFicRunDecoder

   DESCRIPTION
   This function sends fic raw data to fib_decode function and checks the result of FIC decoding 
   and call TcbdFicCheckEnsemble. 
   There is a variable which name is TcbdFicUserStatus. 
   If you want to skip FIC decoding after a certain status of Fic decoding(ex. after FIC decoding complete) 
   you can use this variable.

   PARAMETER
   [in] _fibBuff( fic raw data ).
   [in] _fibBuffSize( data size : this should be multiple of 32, in MODE 1 this is usually 384 ).
   [in/out] esbStatus(TcbdFicEnsbl).
   [in/out] TcbdFicUserStatus.
   [out] TcbdFicTsBuf.numOfTs.

   RETURN VALUE
   esbStatus( TcbdFicEnsbl)
   -------------------------------------------------------------------------------------------- */
I32S TcbdFicRunDecoder(I08U *_fibBuff, I32S _fibBuffSize)
{
    TcpalMemoryCopy(&TcbdFicBuf.buf[TcbdFicBuf.wrPos], _fibBuff, _fibBuffSize);
    TcbdFicBuf.wrPos += _fibBuffSize;

    if(TcbdFicBuf.wrPos >= MAX_FIC_SIZE)
    {
        TcbdFicTsBuf.numOfTs = 0;

        if(TcbdFibDecode(&TcbdFicBuf.buf[0], TcbdFicBuf.wrPos))
        {
            if(TcbdFicUserStatus < 1)
            {
                TcbdFicCheckEnsemble();
                if(TcbdFicEnsbl.esbStatus == 0x03)
                {
                    TcbdFicUserStatus = 1;
                }
            }
        }
        TcbdFicBuf.wrPos = 0;
    }

    return TcbdFicEnsbl.esbStatus;
}

/* --------------------------------------------------------------------------------------------
   FUNCTION    
   TcbdFibDecode

   DESCRIPTION
   This function is got fic raw data, check CRC by FIB unit and after CRC check, 
   build Fig_t structure, eventually, execute TcbdFigDecode.
   When TcbdFigDecode function is running, it fills up the structures of service and sub channel separately.
   So, after TcbdFigDecode is done, match services and sub channels by executing MatchSvc2Sch.

   PARAMETER
   [in] _fibBuff( fic raw data ).
   [in] _fibBuffSize( data size : this should be multiple of 32, in MODE 1 this is usually 384 ).
   [out] TcbdFicCrcResult.
   [out] Fig_t structure.

   RETURN VALUE
   TcbdFicCrcResult - CRC Error/Good result
   -------------------------------------------------------------------------------------------- */
I32U TcbdFibDecode(I08U *_fibBuff,  I32S _fibBuffSize)//, I32S *sch_tot_num)
{
    I32S i, j;
    I32S iFIGListLength;
    I32S schTotalNum = 0;

    TcbdFicSch_t  *subCh;
    I08U *rawBuff;

    iFIGListLength = 0;
    if(_fibBuffSize > 384)
        _fibBuffSize = 384;

    TcbdFicCrcResult.ficCrcErr = 0;
    TcbdFicCrcResult.ficCrcGood = 0;

    for(i=0; i<_fibBuffSize/32; i++) 
    {
        if((((I16U)_fibBuff[(i*32)+30]<<8) | (I16U)_fibBuff[(i*32)+31]) !=  Crc16(_fibBuff+(i*32)))
        {
            TcbdFicCrcResult.ficCrcErr ++;
            continue;
        }

        for(j=0;;)
        {
            if(j>29) break;

            rawBuff = (_fibBuff + (i*32)+j);
            if(*rawBuff != 0xff)
            {
                FigList.cFIGType = (*rawBuff & 0xe0)>>5;
                FigList.cFIGLength = *rawBuff & 0x1f;
                // Length can't be 0,30,31 except End maker's 31
                TcpalMemoryCopy(FigList.cFIGData, &(_fibBuff[(i*32)+j+1]), FigList.cFIGLength);
                j+= FigList.cFIGLength+1; // type,size

                TcbdFigDecode( &schTotalNum);
                TcbdFicCrcResult.ficCrcGood ++;
            }
            else
                break;
        }
    }

    j = TcbdFicCrcResult.ficCrcGood;
    //==== After handle ====================
    if(j)
    { // if there is more than one TcbdFigDecode execute.
        if(TcbdFicEnsbl.numOfSch <= schTotalNum)    
            TcbdFicEnsbl.numOfSch = schTotalNum;     
        if(!(TcbdFicEnsbl.esbStatus & 0x01))
        { 
            //==== Find TS =====
            //tsSch = &TcbdFicTsBuf;
            for(i = 0; i < TcbdFicEnsbl.numOfSch ; i++)
            {
                subCh = &TcbdFicSch[i];
                for(j = 0; j < TcbdFicTsBuf.numOfTs ; j++)
                {
                    if(subCh->subchId == TcbdFicTsBuf.subchId[j])
                        subCh->tsFlag = 1;
                }
            }
            //===============
            MatchSvc2Sch();
        }
    }
    //==================================
    TcbdFicCnt++;        
    if(TcbdFicCnt >= 8000)
        TcbdFicCnt = 10;

    return TcbdFicCrcResult.ficCrcGood;  // return valid TcbdFigDecode counter

}

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   TcbdFigDecode


   DESCRIPTION
   This function is executing Fig_t parsing.
   FIGs are parsed to each Types and extensions.

   When TcbdFigDecode function is running, it fills up the structures of service and sub channel separately,
   and keeps the total sub channel number(schTotalNum).           

   PARAMETER
   [in] schTotalNum.
   [in/out] Enseble/Service/sub channel structure.

   RETURN VALUE
   Not fixed yet
   -------------------------------------------------------------------------------------------- */
I32U TcbdFigDecode(I32S *_schTotalNum)
{
    I32U tmpInt0;
    I08U tmpChar0;
    I08U PDFlag;

    switch (FigList.cFIGType) ///<==== FIG Type  ====
    {
        case 0: ///< + FIG Type 0 : MCI and part of the SI
            BitPosition = 0;
            tmpChar0 = GetField(1) & 0x1;  //C/N
            tmpChar0 = GetField(1) & 0x1;  //OE
            if(tmpChar0)      // Other Ensemble
                break;
            PDFlag = GetField(1);
            tmpChar0 = GetField(5) & 0x1F; //FIG_Ext

            switch(tmpChar0)
            {  // = FIG Extension  =
                case 0:    ///<==== FIG 0/0 - Ensemble information
                    Fig0_0();
                    break;     
                case 1:    ///<==== FIG 0/1 - Basic Sub-Channel Organization
                    Fig0_1(_schTotalNum);
                    break;
                case 2:    ///<==== FIG 0/2 - Basic Service Organization
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig0_2(PDFlag);
                    break;
                case 3:    ///<==== FIG 0/3 - Additional information about the service component description in packet mode.
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig0_3();
                    break;
                case 4:    ///<==== FIG 0/4 - Service component with Conditional Access in Stream mode for FIC.
                    break;
                case 5:   ///<==== FIG 0/5 - Service Component Language
                    Fig0_5();
                    break;
                case 8:    ///<==== FIG 0/8 - Service Component Global Definition
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig0_8(PDFlag);
                    break;
                case 9: ///<==== FIG 0/9 - Country, LTO and Internation Table
                    Fig0_9();
                    break;       
                case 10: ///<==== FIG 0/10 - Date And Time
                    Fig0_10();
                    break;     
                case 13: ///<==== FIG 0/13 - -User Application Information 
                    Fig0_13(PDFlag);
                    break;
                default:
                    break;
            }
            break;
            //###   FIG Type 1 #####################################################################
        case 1: ///< + FIG Type 1 :  Labels, etc. (part of the SI)
            BitPosition=0;
            tmpInt0 = (I08U)(GetField(4) & 0xf); //Charset.
            if(tmpInt0 == 4 || tmpInt0 == 6)
            {
                break;
            }

            tmpChar0= GetField(1); //OE
            if(tmpChar0)
            {
                break;
            }
            tmpChar0= GetField(3) & 0x1F;  // FIG_Ext

            switch(tmpChar0) 
            { // FIG_Ext
                case 0: ///<==== FIG 1/0 - Ensemble label
                    if(TcbdFicEnsbl.esbStatus & 0x2) break;
                    Fig1_0();
                    break;
                case 1: ///<==== FIG 1/1 - Programme Service label
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig1_1();
                    break;
                case 4: ///<==== FIG 1/4 - Service Component label
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig1_4();

                    break;
                case 5: ///<==== FIG 1/5 - Data Service label
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig1_5();

                    break;
                case 6: ///<==== FIG 0/6 - X-Pad User Application label
                    if(TcbdFicEnsbl.esbStatus & 0x1) break;
                    Fig1_6();
                    break;
            }
            break;
            //###  FIG Type 2 ######################################################################
        case 2: ///<+ FIG Type 2 : Labels, etc. (part of the SI) -We don't Support
            break;

        case 5: ///<+ FIG Type 5 : FIDC
            BitPosition=0;
            if(TcbdFicEnsbl.esbStatus) break;
            Fig5();

            break;
        case 6: ///<+ FIG Type 6 : CA
            BitPosition=0;
            break;

        default:
            break;

    }
    return 0x0;
}

void Fig0_0()
{
    TcbdFicEnsbl.eid = GetField(16) & 0xffff; //eid
    TcbdFicEnsbl.changeFlag = GetField(2);

    if(TcbdFicEnsbl.changeFlag == 0x00)
    {
        if(FigList.cFIGLength < 0x05 || FigList.cFIGLength > 0x06)
        {
            return;
        }
        TcbdFicEnsbl.alFlag = GetField(1);
        TcbdFicEnsbl.cifCnt = GetField(13);
        TcbdFicEnsbl.occurChange = 0;
    }
    else
    {
        if(FigList.cFIGLength != 0x06)
        {
            return;
        }
        TcbdFicEnsbl.alFlag = GetField(1);
        TcbdFicEnsbl.cifCnt = GetField(13);
        TcbdFicEnsbl.occurChange = GetField(8);
    }
}

void Fig0_1(I32S *_schTotalNum)
{

    I08U subchId, tmpChar0;
    I32U tmpInt0;
    TcbdFicSch_t *subCh;

    do
    {
        if(*_schTotalNum > MAX_SUBCH_NUM)
            return; // unusual case.. return

        subchId = GetField(6);
        tmpInt0 = GetField(10);  //StartAddress
        tmpChar0 =  GetField(1) & 0x1; //LSFlag

        subCh = &TcbdFicSch[*_schTotalNum];
        subCh->subchId = subchId;

        if(!CheckDuplicateSch(subCh->subchId, *_schTotalNum))
        { 
            subCh->startAddr = tmpInt0;
            subCh->slFg =  tmpChar0;  //LSFlag

            if(subCh->slFg)
            {
                subCh->opt  = GetField(3);
                subCh->prtLvl  = GetField(2);
                subCh->subchSize = GetField(10);
                if(subCh->prtLvl < 4)
                {
                    if (subCh->opt == 0)
                        subCh->sch_n = subCh->subchSize/SchDevideValue[(I32S)subCh->prtLvl];
                    else 
                        subCh->sch_n = subCh->subchSize/SchDevideValue[(I32S)subCh->prtLvl + 4];
                }
                else
                    subCh->sch_n = 0;

            }
            else 
            { //Short Form
                subCh->tableSwitch = GetField(1);
                subCh->idx = GetField(6);
                subCh->opt = 0;
                subCh->prtLvl = 0;
                subCh->sch_n = 0;
                subCh->subchSize = 0;
            }
            subCh->bitRate = GetBitrate(subCh);
            (*_schTotalNum)++;

        }
        else
        {
            if(tmpChar0) 
            {    //Long Form
                tmpInt0 = GetField(15); //pass (option/ProtectionLevel/SubChSize
            }
            else 
            {      //Short Form
                tmpChar0= GetField(1);
                if(tmpChar0)
                    break;
                tmpChar0 = GetField(6);
            }
        }
    }
    while((BitPosition>>3) < FigList.cFIGLength);
}

void Fig0_2(I08U PDFlag)
{
    I08U tmpChar0,TmpChar1, j;
    I08U subchId = 0, PS_CAFlag = 0, scty = 0;
    I32U sid = 0;
    I32U temp_data = 0;

    TcbdFicSc_t  *sc;
    TcbdFicSvc_t *svc;

    do
    {
        if(PDFlag == 0) 
        {// 16bit sid
            sid = GetField(16);
        }
        else 
        { // 32bit sid
            sid = GetField(32);
            //if(((sid >> 24) & 0xF0 ) != 0xF0)
            // continue;
        }

        tmpChar0 = GetField(1) & 0x1; // LocalFlag
        tmpChar0 = GetField(3);  //CAId
        TmpChar1 = GetField(4);  //NumSvcComp
        //== Temporary fix sc num is always '1' =============
        if(TmpChar1 > 1){  //when NumSvcComp > 1
            break;
        }
        //==========================================
        for(j=0;j<TmpChar1;j++)
        {
            tmpChar0 = GetField(2); //tmid

            if(tmpChar0 < 4){ //
                scty = GetField(6);  //ASCTy or DSCTy
                if(tmpChar0 == 0)
                {
                    if(scty > 2)
                        break;
                }
                else if(tmpChar0 == 1)
                {
                    if(scty == 0 || (scty > 5 && scty <24) || (scty > 24 && scty <59) ||scty > 61)
                        break;
                }
                subchId = GetField(6);
                PS_CAFlag = GetField(1)<< 1 | GetField(1);  // PSFlag, CAFlag
                temp_data = scty << 6 | subchId;
            }
            if((CheckDuplicateSvc(sid, TcbdFicEnsbl.numOfSvc)) == 0xff)
            {
                TcbdFicEnsbl.svc[TcbdFicEnsbl.numOfSvc].sid = sid;
                svc = &TcbdFicEnsbl.svc[TcbdFicEnsbl.numOfSvc];
                svc->numOfSvcComp = TmpChar1;  //NumSvcComp
                if(CheckDuplicateSvcComp(svc, tmpChar0, temp_data)) // tmpChar0 = tmid
                    break;
                sc = &svc->sc[j];  

                sc->no++;
                sc->tmid = tmpChar0;
                sc->caFlag = PS_CAFlag & 0x1;
                if(tmpChar0 < 3)
                {
                    sc->subCh.subchId = subchId;
                    sc->ps = PS_CAFlag >> 0x1;
                    sc->scty = (temp_data & 0x3f);
                    sc->scid = 0xf000 | subchId;
                    sc->scStatus |= 0x1; // subchId found;
                }
                else
                {
                    sc->scid = temp_data;
                    sc->scStatus &= 0x1; // subchId not found yet.
                }
                TcbdFicEnsbl.numOfSvc++;

                sc->sctyTmp = scty;
            }

            //#######################################
            if((tmpChar0 == 1) && (scty == 0x18))             // If this condition, this channel is TS format.
            {  // tmid, DSCTy
                //tsSch = &TcbdFicTsBuf;
                TcbdFicTsBuf.subchId[TcbdFicTsBuf.numOfTs] = subchId;
                TcbdFicTsBuf.numOfTs++;
            }
            //#######################################
        }
    }
    while((BitPosition>>3)<FigList.cFIGLength);
}

void Fig0_3()
{
    I08U tmpChar0,TmpChar1, j, m, DoCnt = 0;
    I32U tmpInt0;
    I08U subchId = 0, DSCTy = 0;
    I16U  scid = 0;
    TcbdFicSc_t  *sc;

    do
    {
        DoCnt++;
        //====================
        if(DoCnt > MAX_SERVICE_NUM)
            return; // unusual case.. return
        //====================
        scid = GetField(12);
        tmpChar0 = GetField(3) & 0x7; // Rfa
        if(tmpChar0)
            break;
        TmpChar1 = GetField(1); //CAOrgFlg
        tmpChar0 = GetField(1); //DGFlag
        tmpChar0  = GetField(1) & 0x1; // Rfu
        if(tmpChar0)
            break;
        DSCTy  = GetField(6); //
        subchId  = GetField(6); //
        tmpInt0 = GetField(10); //Packet Addrress

        if(TmpChar1)
            TmpChar1 = GetField(16); //CAOrg

        //#########################################################################
        for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
        {
            for(m = 0; m < TcbdFicEnsbl.svc[j].numOfSvcComp; m++)
            {
                sc = &TcbdFicEnsbl.svc[j].sc[m];
                if(sc->scid == scid && !(sc->scStatus & 0x01))
                {
                    sc->subCh.subchId = subchId;
                    sc->scty = DSCTy;
                    sc->scStatus |= 0x01;
                    break;
                }
            }
        }
        //#########################################################################
    }
    while((BitPosition>>3)<FigList.cFIGLength);
}

void Fig0_5()
{
    I08U tmpChar0, DoCnt = 0;
    I32U tmpInt0;
    I16U  scid, Id = 0;

    do
    {
        DoCnt++;
        //====================
        if(DoCnt > MAX_SERVICE_NUM)
            return; // unusual case.. return
        //====================
        tmpChar0 = GetField(1);   //LSFlag

        if(tmpChar0) 
        { //Long Form
            tmpInt0 = GetField(3);
            scid = GetField(12);   //scid
            tmpChar0 = GetField(8);  //Language
        }
        else 
        { //Short Form
            tmpChar0 = GetField(1);
            if(tmpChar0) 
            {
                //FIC & FIDCId
                Id = GetField(6);   //FIDCId
                tmpChar0 = GetField(8); //Language
            }
            else 
            {
                //MSC & subchId
                Id = GetField(6);   //subchId
                tmpChar0 = GetField(8); //Language
            }
        }
    }
    while((BitPosition>>3)<FigList.cFIGLength);
}


void Fig0_8(I08U PDFlag)
{
    I08U tmpChar0, DoCnt = 0;
    I32U tmpInt0, TmpInt1, sid;
    I16U  scid, Id = 0;
    do
    {
        DoCnt++;
        //====================
        if(DoCnt > MAX_SERVICE_NUM)
            return; // unusual case.. return
        //====================
        if(PDFlag == 0)
        { // 16bit sid
            sid = GetField(16);
        }
        else 
        {  // 32bit sid
            sid = GetField(32);
        }
        tmpChar0 = GetField(1) & 0x1; //ExtFlag
        tmpInt0 = GetField(3);
        TmpInt1 = GetField(4);   //SCIdS

        tmpChar0= GetField(1);

        if(tmpChar0)
        {  // Long Form
            tmpInt0 = GetField(3);
            scid = GetField(12);
        }
        else 
        {   // Short Form
            tmpChar0 = GetField(1);
            if(tmpChar0)
            { //FIC & FIDCId
                Id = GetField(6);
            }
            else 
            {  // MSC & subchId
                Id = GetField(6);
            }
        }
        if(tmpChar0)  // ExtFlag
        {
            // 8bit Rfa field present
            tmpChar0 = GetField(8) & 0xFF;
        }
    }
    while((BitPosition>>3)<FigList.cFIGLength); 
}

void Fig0_9()
{
    I08U DoCnt = 0;
    InternationalTable_t *IntntnTbl;
    do
    {       
        DoCnt++;
        //====================
        if(DoCnt > MAX_SERVICE_NUM)
            return; // unusual case.. return
        //====================
        IntntnTbl = &InternationalTable;

        IntntnTbl->ExtFlg = GetField(1); // 
        IntntnTbl->LTO_Unq=GetField(1); //
        IntntnTbl->EnsblLTO= GetField(6); //
        IntntnTbl->EnsblECC= GetField(8); //
        IntntnTbl->InterTableID= GetField(8); //

        if(IntntnTbl->InterTableID == 1)
        {// PTY from Table 12 & Announcement type from Table 14

        }
        else{// RBDS PTY from Table 13 & Announcement type from Table 14

        }
        if(IntntnTbl->ExtFlg){

        }
    }
    while((BitPosition>>3) < FigList.cFIGLength);
}

I08U Fig0_10()
{
    I08U tmpChar0;

    tmpChar0 = GetField(1); // Rfa
    DateTime.mjd  = GetField(17); //
    DateTime.lsi  = GetField(1); //
    DateTime.confInd  = GetField(1); //
    DateTime.utcFlag  = GetField(1); //

    if(DateTime.utcFlag)
    {
        tmpChar0 = GetField(5);
        if(tmpChar0 < 24)
            DateTime.hour  =  tmpChar0;//
        else
            return (I08U)0xF9;

        tmpChar0 = GetField(6);
        if(tmpChar0 < 60) 
            DateTime.minute  = tmpChar0;
        else
            return (I08U)0xF9;
        DateTime.sec  = GetField(8); //
        DateTime.millisec  = GetField(10); //
    }
    else
    {
        tmpChar0 = GetField(5);
        if(tmpChar0 < 24)
            DateTime.hour  =  tmpChar0;//
        else
            return (I08U)0xF9;

        tmpChar0 = GetField(6);
        if(tmpChar0 < 60) 
            DateTime.minute  = tmpChar0;
        else
            return (I08U)0xF9;
    }
    return 0;
}

void Fig0_13(I08U PDFlag)
{
    I08U tmpChar0,j,m,TmpChar1, DoCnt= 0;
    I32U tmpInt0, TmpInt1, sid;

    do
    {
        DoCnt++;
        //====================
        if(DoCnt > MAX_SERVICE_NUM)
            return; // unusual case.. return
        //====================
        if(PDFlag == 0)
            sid = GetField(16);
        else
            sid = GetField(32);

        TmpInt1 = GetField(4);  //SCIdS
        tmpChar0 = GetField(4);  //NumUserApp

        for(j=0;j<tmpChar0;j++)
        {
            tmpInt0 = GetField(11); //UserAppType
            TmpChar1 = GetField(5); //UserAppLength
            for(m = 0; m < TmpChar1 ; m++)
            {
                tmpChar0 = GetField(8);//
            }
        }
    }
    while((BitPosition>>3)<FigList.cFIGLength);
}

void PutInLabel(I08U *label )
{
    I08U m;

    for(m = 0; m < 16; m++)
        *label++ = GetField(8);
}

void Fig1_0()
{
    I32U tmpInt0;

    tmpInt0 = GetField(16);  //ID_Field, eid
    if(TcbdFicEnsbl.eid == tmpInt0){
        //====  label input =============
        PutInLabel(&TcbdFicEnsbl.ensembleLabel[0]);
        tmpInt0 = GetField(16); //CharFlagField
        TcbdFicEnsbl.esbStatus |= 0x2;
        return;
    }
}

void Fig1_1()
{
    I32U tmpInt0;
    I08U  j;

    tmpInt0 = GetField(16);  //ID_Field, sid
    for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
    {
        if(TcbdFicEnsbl.svc[j].sid == tmpInt0)
        {
            if(!(TcbdFicEnsbl.svc[j].svcStatus & 0x2))
            {
                //====  label input =============
                PutInLabel(&TcbdFicEnsbl.svc[j].label[0]);
                TcbdFicEnsbl.svc[j].svcStatus |= 0x2; //
            }
            tmpInt0 = GetField(16);   //CharFlagField
            return;
        }
    }
    //==============================================
}

void Fig1_4()
{
    I32U tmpInt0, scid;
    I08U tmpChar0, m,j, PDFlag;

    PDFlag = GetField(1);
    tmpChar0= GetField(3) & 0x7;
    if(tmpChar0)
        return;
    tmpInt0 = GetField(4);  //SCIdS
    if(PDFlag)
        scid = GetField(32);
    else
        scid = GetField(16);

    tmpInt0 = GetField(16);  //CharFlagField
    for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
    {
        for(m = 0; m < TcbdFicEnsbl.svc[j].numOfSvcComp; m++)
        {
            if(TcbdFicEnsbl.svc[j].sc[m].scid == scid)
            {
                //====  label input =============
                PutInLabel(&TcbdFicEnsbl.svc[j].sc[m].label[0]);
                return;
            }
        }
    }
    //==============================================
}

void Fig1_5()
{
    I32U tmpInt0;
    I08U j;
    tmpInt0= GetField(32);    // ID_Field, sid

    for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
    {
        if(TcbdFicEnsbl.svc[j].sid == tmpInt0)
        {
            if(!(TcbdFicEnsbl.svc[j].svcStatus & 0x2))
            {
                //====  label input =============
                PutInLabel(&TcbdFicEnsbl.svc[j].label[0]);
                TcbdFicEnsbl.svc[j].svcStatus |= 0x2;
            }
            tmpInt0 = GetField(16);  //CharFlagField
            return;
        }
    }
    //==============================================
}

void Fig1_6()
{
    I32U sid;
    I08U tmpChar0, m,j, PDFlag, SCIdS;

    PDFlag = GetField(1);
    tmpChar0= GetField(3) & 0x7;
    if(tmpChar0)
        return;

    SCIdS = GetField(4); //SCIdS
    if(PDFlag)
        sid = GetField(32);
    else
        sid = GetField(32);

    tmpChar0 = GetField(2); //Rfa
    tmpChar0 = GetField(1); //Rfu
    tmpChar0 = GetField(5); //XPadAppTy

    for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
    {
        if(TcbdFicEnsbl.svc[j].sid == sid)
        {
            for(m = 0; m < TcbdFicEnsbl.svc[j].numOfSvcComp; m++)
            {
                if(TcbdFicEnsbl.svc[j].sc[m].scid == SCIdS)
                {
                    TcbdFicEnsbl.svc[j].sc[m].xpadType = tmpChar0;
                    return;
                }
            }
        }
    }
    //==============================================
}

void Fig5()
{  // FIDC
} 

#if 0
void Fig6()
{  //CA
    I32U sid;
    I08U tmpChar0, PDFlag,i, CASysId,cnt = 0, CAIntChar[24];
    //==============================================
    tmpChar0= GetField(1); // Rfu . all '0'
    tmpChar0= GetField(1); // C/N
    tmpChar0= GetField(1); // OE 
    PDFlag = GetField(1);  // PD
    tmpChar0= GetField(1); // LEF
    if(!tmpChar0)
    {
        tmpChar0= GetField(3); // Short CASysId
        if(!PDFlag)
            sid = GetField(16); // sid
        else
            sid = GetField(32); // sid
    }
    else
    {
        tmpChar0= GetField(3); // Short CASysId
        cnt = 8;
        if(!PDFlag)
        {
            sid = GetField(16); // sid
            cnt+= 16;
        }
        else
        {
            sid = GetField(32); // sid
            cnt+= 32;
        }

        CASysId = GetField(16); // 
        cnt += 16; 
        for(i = 0; i < FigList.cFIGLength - cnt; i++)
            CAIntChar[i] = GetField(1);
    }
    //==============================================
} 
#endif

/* --------------------------------------------------------------------------------------------
   FUNCTION    
   CheckDuplicateSvc

   DESCRIPTION
   check if there is same service ID and return the sequence of it .

   PARAMETER
   [in] sid.
   [in] s_num.

   RETURN VALUE
   Sqc of found service
   -------------------------------------------------------------------------------------------- */
I08U CheckDuplicateSvc(I32U sid, I32S _numOfSvc)
{
    I08U i;

    for(i = 0; i < _numOfSvc ; i++)
    {
        if(TcbdFicEnsbl.svc[i].sid == sid)
            return i;
    }

    return 0xff;
}

/* --------------------------------------------------------------------------------------------
   FUNCTION    
   CheckDuplicateSvcComp

   DESCRIPTION
   check if there is same service component ID and return the sequence of it.

   PARAMETER
   [in] _svc Pointer of Service Structure.
   [in] _tmid.
   [in] _uniqeId : 
   if tmid 0, ASCTy << 6 | subchId
   if tmid 1, DSCTy << 6 | subchId
   if tmid 2, DSCTy << 6 | FIDCId
   if tmid 3, scid

   RETURN VALUE
   Error
   -------------------------------------------------------------------------------------------- */
I08U CheckDuplicateSvcComp(TcbdFicSvc_t *_svc, I08U _tmid, I32U _uniqeId)
{

    I08U i;
    I08U   scty, subchId;

    for(i = 0 ; i < _svc->numOfSvcComp; i++)
    {
        if(_tmid < 3)
        {
            scty = _uniqeId >> 6;
            subchId = _uniqeId & 0x3f;
            if((_svc->sc[i].subCh.subchId == subchId))
                return 0xff;
        }
        else
        {
            if(_svc->sc[i].scid == _uniqeId)
                return 0xff;
        }
    }
    return 0;
}

/* --------------------------------------------------------------------------------------------
   FUNCTION    
   CheckDuplicateSch

   DESCRIPTION
   check if there is same sub ch ID.

   PARAMETER
   [in] _subchId.
   [in] _schNum - num of total SubCh.

   RETURN VALUE
   Error
   -------------------------------------------------------------------------------------------- */
I32U CheckDuplicateSch(I08U _subchId, I32S _schNum)
{
    I32S i;

    for(i = 0; i < _schNum ; i++)
    {
        if(TcbdFicSch[i].subchId == _subchId)
            return 0xff;
    }
    return 0;
}
/* --------------------------------------------------------------------------------------------
   FUNCTION    
   TcbdFicInitDb

   DESCRIPTION
   Initialize all structures and parameters 
   related FIC Decoding such as Ensemble, 
   service, service component and sub channel etc.

   PARAMETER

   RETURN VALUE
   -------------------------------------------------------------------------------------------- */
void TcbdFicInitDb()
{
    I32S i,j;

    TcpalMemorySet(&TcbdFicBuf, 0, sizeof(TcbdFicBuf_t));

    TcbdFicCnt = 0;

    TcbdFicUserStatus = 0; 
    TcpalMemorySet(&TcbdFicEnsbl, 0, sizeof(TcbdFicEnsbl_t));
    TcbdFicEnsbl.eid = 0xFFFF;
    TcpalMemorySet(TcbdFicEnsbl.ensembleLabel, 0x20, sizeof(I08U) * 16);
    for(i = 0; i < MAX_SERVICE_NUM; i++)
    {
        TcbdFicEnsbl.svc[i].pd = 0xf;
        TcbdFicEnsbl.svc[i].sid = 0xFFFFFFFF;
        TcpalMemorySet(TcbdFicEnsbl.svc[i].label, 0x20, sizeof(I08U) * 16);
        for(j = 0 ; j < MAX_SC_NUM; j++)
        {
            TcbdFicEnsbl.svc[i].sc[j].tmid = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].sctyTmp = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].scid = 0xFFFF;
            TcbdFicEnsbl.svc[i].sc[j].scty = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].ps = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].caFlag = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].subCh.subchId = 0xff;
            TcbdFicEnsbl.svc[i].sc[j].xpadType = 0xff;
            TcpalMemorySet(TcbdFicEnsbl.svc[i].sc[j].label, 0x20, sizeof(I08U) * 16);
        }
    }

    //TcpalMemorySet(&TdmbTarget.subchId[0], 0xff, sizeof(I08U) * MAX_SUBCH_NUM);
    //TdmbTarget.checkCnt = 0;

    TcpalMemorySet(TcbdFicSch, 0, sizeof(TcbdFicSch_t) * MAX_SUBCH_NUM);
    for(j = 0; j < MAX_SUBCH_NUM; j++)
    {
        TcbdFicSch[j].subchId = 0xff;
    }

    TcbdFicTsBuf.numOfTs = 0;      // ts sub channel cnt.

    TcpalMemorySet(&TcbdFicTsBuf.subchId[0], 0xff, sizeof(I08U) * 5);
}

TcbdFicSch_t* TcbdIsSubchExist(I08U _subchId)
{
    I32S i = 0;
    
    for(i = 0; i < MAX_SUBCH_NUM; i++)
    {
        if(TcbdFicSch[i].subchId == _subchId)
        {
            return &TcbdFicSch[i];
        }
    }
    return NULL;
}
/* --------------------------------------------------------------------------------------------

   FUNCTION    
   MatchSvc2Sch

   DESCRIPTION
   Right After TcbdFibDecode, find sub channel from TcbdFicSch which is matched by service
   and copy it to the matched service structure.

   PARAMETER

   RETURN VALUE
   -------------------------------------------------------------------------------------------- */
void  MatchSvc2Sch()
{
    I32S i, j;
    I08U numOfSvcLabelEnd = 0,numOfSvcLabelBefore = 0;

    TcbdFicSc_t  *sc;
    //==============================================
    for(j = 0; j < TcbdFicEnsbl.numOfSvc ; j++)
    {
        if(TcbdFicEnsbl.svc[j].svcStatus == 3)
        {
            numOfSvcLabelEnd++;
            continue;
        }
        else if(TcbdFicEnsbl.svc[j].svcStatus >=1)
            numOfSvcLabelBefore++;

        sc = &TcbdFicEnsbl.svc[j].sc[0];
        for(i = 0; i < TcbdFicEnsbl.numOfSch ; i++)
        {
            if(sc->subCh.subchId ==  TcbdFicSch[i].subchId)
            {
                if((sc->scStatus & 0x03) == 0x01)
                {
                    TcpalMemoryCopy(&(sc->subCh), &TcbdFicSch[i], sizeof(TcbdFicSch_t));
                    sc->scStatus |= 0x3;

                    TcbdFicEnsbl.svc[j].svcStatus |= 0x1;
                    numOfSvcLabelBefore++;

                    if(TcbdFicEnsbl.svc[j].svcStatus == 3)
                        numOfSvcLabelEnd++;
                    break;
                }
            }
        }
    }

    //===========================================
    TcbdFicEnsbl.numOfSvcLabelEnd = numOfSvcLabelEnd;
    TcbdFicEnsbl.numOfSvcLabelBefore = numOfSvcLabelBefore;
    //===========================================
}

/* --------------------------------------------------------------------------------------------

   FUNCTION    
   GetField

   DESCRIPTION
   Selected bit[s] value return.

   PARAMETER
   [in] _sizeBit - size of bit(s).


   RETURN VALUE
   -------------------------------------------------------------------------------------------- */
I32U GetField(I08U _sizeBit )
{
    I08U startByte, subStartBit; 
    I08U sizeByte, subSizeBit, numTemp;
    I08U readTemp[4]={0,0,0,0};
    I32U returnTemp;

    startByte = BitPosition >>3;
    subStartBit = BitPosition - (startByte<<3);

    /*if(_sizeBit > 32) { }*/

    sizeByte = _sizeBit >> 3;
    subSizeBit = _sizeBit - (sizeByte<<3);

    numTemp = 0;
    while(sizeByte != 0)
    {
        readTemp[numTemp] = (*(FigList.cFIGData+startByte) << subStartBit) | 
            (*(FigList.cFIGData+startByte+1) >> (8-subStartBit));
        startByte++;
        sizeByte--;
        numTemp++;
    }

    if(subSizeBit != 0)
    {
        readTemp[numTemp] = (*(FigList.cFIGData+startByte)<< subStartBit) & (0xff<<(8-subSizeBit)); 
    }

    returnTemp = (((I32U) readTemp[0]) << 24) |
        (((I32U) readTemp[1]) << 16) |
        (((I32U) readTemp[2]) << 8) |
        ((I32U) readTemp[3]);

    returnTemp = returnTemp >> (32-_sizeBit);
    BitPosition += _sizeBit;

    return returnTemp;
}

I16U Crc16(I08U *buf)
{
    I32U b, len;
    I08U crcl,crcm;

    crcl = 0xff;
    crcm = 0xff;


    for(len=0;len<30;len++)
    {
        b = *(buf+len) ^ crcm;
        b = b ^ (b>>4);
        crcm = crcl ^ (b>>3) ^ (b<<4);
        crcl = b ^ (b<<5);
    }

    crcl = crcl ^ 0xff;
    crcm = crcm ^ 0xff;

    return ((I16U)crcl | (I16U)(crcm)<<8);
}

I32U GetBitrate(TcbdFicSch_t * _sch)
{
    if (_sch->slFg == 0) 
    {
        return BitrateValue[_sch->idx] << 2;
    }
    else
    {
        if(_sch->opt == 0)
            return (_sch->subchSize/SchDevideValue[(I32S)_sch->prtLvl]) <<3;
        else
            return (_sch->subchSize/SchDevideValue[(I32S)_sch->prtLvl + 4])<< 5;
    }
}

I08U TcbdFicGetPrtLvl(TcbdFicSch_t *_subch)
{
    I16U uepVal = 0;

    if(_subch->slFg)
    {
        return _subch->prtLvl + (_subch->opt*4);
    }
    else
    {
        uepVal = UepTable[_subch->idx];
        return (uepVal & 0x0E00) >> 9;
    }
}

I32S TcbdFicGetSchSize(TcbdFicSch_t *_subch)
{
    I16U uepVal = 0;
    if(_subch->slFg)
    {
        return _subch->subchSize;
    }
    else
    {
        uepVal = UepTable[_subch->idx];
        return (uepVal & 0x1FF);
    }
}

I08U TcbdFicGetBitRate(TcbdFicSch_t *_subch)
{
    I16U uepVal = 0;

    if(_subch->slFg)
    {
        return _subch->sch_n;
    }
    else
    {
        uepVal = UepTable[_subch->idx];
        return (uepVal & 0xF000) >> 12;
    }
}

I08U TcbdFicGetPrtType(TcbdFicSch_t *_subch)
{
    if(_subch->slFg)
        return 1;
    else
        return 0;
}
