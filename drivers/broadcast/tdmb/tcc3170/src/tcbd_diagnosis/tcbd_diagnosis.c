#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_diagnosis.h"
#include "tcbd_error.h"

#define OFFSET_RF_LOOP_GAIN   4
#define OFFSET_BB_LOOP_GAIN   5
#define OFFSET_PRS_SNR        8
#define OFFSET_PCBER         12
#define OFFSET_RS_PKT_CNT    16
#define OFFSET_RS_OVER_CNT   20
#define OFFSET_RS_ERR_CNT_LO 24
#define OFFSET_RS_ERR_CNT_HI 28

#define MAX_MAVG_ARRAY_SIZE 5

#define MIN_SNR  0
#define MAX_SNR  25

#define MIN_PCBER  0
#define MAX_PCBER  1

#define MIN_VITERBIBER  0
#define MAX_VITERBIBER  1

#define MIN_TSPER  0
#define MAX_TSPER  1

#define MIN_RSSI  (-105)
#define MAX_RSSI  3

typedef struct _TcbdMovingAvg_t {
    I32U array[MAX_MAVG_ARRAY_SIZE+1];
    I32U preSum;
    I64U totalSum;
    I32S index;
    I32S movingCnt;
    I32S totalCnt;
} TcbdMovingAvg_t;

typedef struct _TcbdStatusInfo_t
{
    I08U lockStatus;
    I08U rfLoopGain;
    I08U bbLoopGain;

    I32S pcber;
    I32U prsSnr;

    I32U rsOverCnt;
    I32U rsPktCnt;
    I64U rsErrCnt;
} TcbdStatusInfo_t;


typedef struct _TcbdDiagnosisInfo_t
{
    I32U resynced;
    TcbdStatusInfo_t oldStatus;
    TcbdStatusInfo_t currStatus;

    TcbdDiagnosis_t diagInfo;
} TcbdDiagnosisData_t;

static TcbdMovingAvg_t MovingAvg[4];
static TcbdDiagnosisData_t Diagnosis;

static inline I32U TcbdLog10(I32U _val)
{
    I32U i, snr = 0;
    I32U snr_table[28] = { 
              1024,   1289,   1622,  2043,   2572,   3238,   4076,   5132,  
              6461,   8133,  10240, 12891,   6229,  20431,  25721,  32381,  
             40766,  51321,  64610, 81339, 102400, 128913, 162293, 204314, 
            257217, 323817, 407661, 513215 };

    if (_val < snr_table[0])
    {
        return  0;
    }

    if (_val >= snr_table[27])
    {
        return 27;
    }

    for (i = 0; i < 27; i++)
    {
        if (_val >= snr_table[i] && _val < snr_table[i + 1])
        {
            snr = i;
            break;
        }
    }

    return snr;
}

static I32S TcbdMovingAvg(TcbdMovingAvg_t *_slot, I32U _value, I32S _windowSize)
{
    I32S movingSum;
    I32U result;

    if(MAX_MAVG_ARRAY_SIZE < _windowSize)
    {
        TcbdDebug(0, "max window size is %d\n", MAX_MAVG_ARRAY_SIZE);
        return -TCERR_INVALID_ARG;
    }

    if(_slot->movingCnt >= _windowSize)
        _slot->preSum += _slot->array[_slot->index];
    else 
        _slot->movingCnt++;

    _slot->array[_slot->index] = _value;
    _slot->index++;

    _slot->index %= _windowSize;
    _slot->totalCnt++;
    _slot->totalSum += _value;

    movingSum = _slot->totalSum - _slot->preSum;
    result =  movingSum / _slot->movingCnt;

    if(_slot->index == 0)
    {
        _slot->totalSum = movingSum;
        _slot->preSum = 0;
    }

    return result;
}


static inline void TcbdCalcRssi(TcbdDiagnosis_t *_diagInfo)
{
    TcbdStatusInfo_t *currStatus = &Diagnosis.currStatus;

    if (currStatus->rfLoopGain <= 120)
    {
        _diagInfo->rssi = 1500 - ((I32S)currStatus->bbLoopGain)*37 - 27*((I32S)currStatus->rfLoopGain);
    }
    else if (currStatus->rfLoopGain <= 216)
    {
        _diagInfo->rssi = 1500 - ((I32S)currStatus->bbLoopGain)*35 - 24*((I32S)currStatus->rfLoopGain);
    }
    else
    {
        _diagInfo->rssi = 1500 - ((I32S)currStatus->bbLoopGain)*33 - 22*((I32S)currStatus->rfLoopGain);
    }

    _diagInfo->rssi /= 100;
    if (_diagInfo->rssi < MIN_RSSI)
        _diagInfo->rssi = MIN_RSSI;
    else if (_diagInfo->rssi > MAX_RSSI)
        _diagInfo->rssi = MAX_RSSI;
}

static inline void TcbdCalcPcber(TcbdDiagnosis_t *_diagInfo)
{
    I64S over;
    TcbdStatusInfo_t *currStatus = &Diagnosis.currStatus;

    over = (I64U)currStatus->pcber * SCALE_FACTOR;
    _diagInfo->pcber = (I32U)(over>>16);

    if(_diagInfo->pcber < MIN_PCBER)
        _diagInfo->pcber = MIN_PCBER;
    else if(_diagInfo->pcber > MAX_PCBER * ERROR_LIMIT)
        _diagInfo->pcber = MAX_PCBER * ERROR_LIMIT;

    _diagInfo->pcberMovingAvg = TcbdMovingAvg(&MovingAvg[0], _diagInfo->pcber, MAX_MAVG_ARRAY_SIZE);
}

static inline void TcbdCalcSnr(TcbdDiagnosis_t *_diagInfo)
{
    TcbdStatusInfo_t *currStatus = &Diagnosis.currStatus;

    if ((I32S)currStatus->prsSnr < 0)
    {
        _diagInfo->snr = MAX_SNR;
    }
    else if (currStatus->prsSnr == 0)
    {
        _diagInfo->snr = MIN_SNR;
    }
    else
    {
       _diagInfo->snr = TcbdLog10(currStatus->prsSnr);
    }

    if (_diagInfo->snr < MIN_SNR)
        _diagInfo->snr = MIN_SNR;
    else if (_diagInfo->snr > MAX_SNR)
        _diagInfo->snr = MAX_SNR;

    _diagInfo->snrMovingAvg = TcbdMovingAvg(&MovingAvg[1], _diagInfo->snr, MAX_MAVG_ARRAY_SIZE);
}

static inline void TcbdCalcViterbiber(TcbdDiagnosis_t *_diagInfo)
{
    I32S rsError, rsOver, rsUnder;
    TcbdStatusInfo_t *oldStatus, *currStatus;

    oldStatus = &Diagnosis.oldStatus;
    currStatus = &Diagnosis.currStatus;

    if(Diagnosis.resynced)
    {
        _diagInfo->vber = MAX_VITERBIBER * ERROR_LIMIT;
        goto exitCalcViterbiber;
    }

    if(Diagnosis.resynced && currStatus->rsPktCnt == 0)
    {
        _diagInfo->vber = MIN_VITERBIBER;
        goto exitCalcViterbiber;
    }

    rsError = ((I32U)(currStatus->rsOverCnt - oldStatus->rsOverCnt)) * SCALE_FACTOR;

    rsOver =  (rsError*(8*8) + ((I32U)(currStatus->rsErrCnt - oldStatus->rsErrCnt)))* SCALE_FACTOR;
    rsUnder = (currStatus->rsPktCnt - oldStatus->rsPktCnt) * 204 * 8;
    if(!rsUnder) return;

    _diagInfo->vber = rsOver / rsUnder;

    if(_diagInfo->vber < MIN_VITERBIBER)
        _diagInfo->vber = MIN_VITERBIBER;
    else if(_diagInfo->vber > MAX_VITERBIBER * ERROR_LIMIT)
        _diagInfo->vber = MAX_VITERBIBER * ERROR_LIMIT;

exitCalcViterbiber:
    _diagInfo->vberMovingAvg = TcbdMovingAvg(&MovingAvg[3], _diagInfo->vber, MAX_MAVG_ARRAY_SIZE);
}

static inline void TcbdCalcTsper(TcbdDiagnosis_t *_diagInfo)
{
    I32S tsper;
    I32S rsOver, rsUnder;
    TcbdStatusInfo_t *oldStatus, *currStatus;

    oldStatus = &Diagnosis.oldStatus;
    currStatus = &Diagnosis.currStatus;

    if(Diagnosis.resynced)
    {
        _diagInfo->tsper = MAX_TSPER * ERROR_LIMIT;
        goto exitCalcTsper;
    }

    if(Diagnosis.resynced && currStatus->rsPktCnt == 0)
    {
        _diagInfo->tsper = MIN_TSPER;
        goto exitCalcTsper;
    }

    rsOver = (currStatus->rsOverCnt - oldStatus->rsOverCnt) * SCALE_FACTOR;
    rsUnder = currStatus->rsPktCnt - oldStatus->rsPktCnt;
    if(!rsUnder) return;

    _diagInfo->tsErrorCnt = rsOver;
    _diagInfo->tsper = rsOver / rsUnder;
    if(tsper < MIN_TSPER)
        _diagInfo->tsper = MIN_TSPER;
    else if(tsper > MAX_TSPER * ERROR_LIMIT)
        _diagInfo->tsper = MAX_TSPER * ERROR_LIMIT;

exitCalcTsper:
    _diagInfo->tsperMovingAvg = TcbdMovingAvg(&MovingAvg[2], _diagInfo->tsper, MAX_MAVG_ARRAY_SIZE);
}

void TcbdUpdateStatus(I08U *_rawData, TcbdDiagnosis_t *_diagInfo)
{
    TcbdStatusInfo_t *oldStatus, *currStatus;
    TcbdDiagnosis_t *diagInfo;// = (_diagInfo) ? _diagInfo: &Diagnosis.diagInfo;

    if(!_rawData && !_diagInfo) return;

    if(_rawData == NULL) 
    {
        TcpalMemoryCopy(_diagInfo, &Diagnosis.diagInfo, sizeof(TcbdDiagnosis_t));
        return;
    }

    diagInfo   = (_diagInfo) ? _diagInfo: &Diagnosis.diagInfo;
    oldStatus  = &Diagnosis.oldStatus;
    currStatus = &Diagnosis.currStatus;
    TcpalMemoryCopy(oldStatus, currStatus, sizeof(TcbdStatusInfo_t));

    currStatus->rfLoopGain = _rawData[OFFSET_RF_LOOP_GAIN];
    currStatus->bbLoopGain = _rawData[OFFSET_BB_LOOP_GAIN];

    currStatus->prsSnr    = *((I32U*)(_rawData+OFFSET_PRS_SNR));
    currStatus->pcber     = *((I32U*)(_rawData+OFFSET_PCBER));

    currStatus->rsPktCnt  = *((I32U*)(_rawData+OFFSET_RS_PKT_CNT));
    currStatus->rsOverCnt = *((I32U*)(_rawData+OFFSET_RS_OVER_CNT));
    currStatus->rsErrCnt  = *((I64U*)(_rawData+OFFSET_RS_ERR_CNT_LO));
    currStatus->rsErrCnt |=(*((I64U*)(_rawData+OFFSET_RS_ERR_CNT_HI))) <<32;

    //TcbdDebug(1, "RFGain:%d, BBGain:%d pcber:%d\n", 
    //    currStatus->rfLoopGain, currStatus->bbLoopGain, currStatus->pcber);

    if(currStatus->rsPktCnt != oldStatus->rsPktCnt)
    {
        Diagnosis.resynced = 0;
        if(currStatus->rsPktCnt == 0 && oldStatus->rsPktCnt)
        {
            Diagnosis.resynced = 1;
        }
    }
    else if(currStatus->rsPktCnt < oldStatus->rsPktCnt)
    {
        if(oldStatus->rsPktCnt < 0x80000000)
        {
            Diagnosis.resynced = 1;
        }
    }

    TcbdCalcSnr(diagInfo);
    TcbdCalcPcber(diagInfo);
    TcbdCalcViterbiber(diagInfo);
    TcbdCalcTsper(diagInfo);
    TcbdCalcRssi(diagInfo);
    
    diagInfo->lock = *((I08U*)_rawData);
    if(Diagnosis.resynced)
        TcpalMemorySet(MovingAvg, 0, sizeof(MovingAvg));
}

void TcbdInitDiagnosis(void)
{
    TcpalMemorySet(&Diagnosis, 0, sizeof(Diagnosis));
    TcpalMemorySet(MovingAvg, 0, sizeof(MovingAvg));
}
