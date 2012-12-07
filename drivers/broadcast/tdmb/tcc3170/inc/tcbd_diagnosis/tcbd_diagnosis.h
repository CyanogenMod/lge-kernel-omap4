#ifndef __TCBD_DIAGNOSIS_H__
#define __TCBD_DIAGNOSIS_H__

//0 agc, 1 cto, 2 cfo, 3 fto, 4 sync, 5, ofdm
#define TCBD_LOCK_AGC   0x01
#define TCBD_LOCK_CTO   0x02
#define TCBD_LOCK_CFO   0x03
#define TCBD_LOCK_FTO   0x04
#define TCBD_LOCK_SYNC  0x10
#define TCDD_LOCK_OFDM  0x20

#define SCALE_FACTOR 100000
#define ERROR_LIMIT   20000

typedef struct _TcbdDiagnosis_t
{
    I08U lock;
    I32S rssi;
    I32S snr;
    I32S snrMovingAvg;
    I32S pcber;
    I32S pcberMovingAvg;
    I32S vber;
    I32S vberMovingAvg;
    I32S tsErrorCnt;
    I32S tsper;
    I32S tsperMovingAvg;
} TcbdDiagnosis_t;

TCBB_FUNC void TcbdUpdateStatus(I08U *_rawData, TcbdDiagnosis_t *_diagInfo);
TCBB_FUNC void TcbdInitDiagnosis(void);

#endif //__TCBD_SIGNAL_INFO_H__
