
#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_io.h"
#include "tcbd_drv_rf.h"

typedef struct _TcbdRfTable 
{
    I08U addr;
    I32U data;
} TcbdRfTable_t;

static TcbdRfTable_t Tc3170RfTableBand3[] = 
{
    {0x02, 0x00030000},
    {0x04, 0x00002002},
    {0x05, 0x5500640B},
    {0x06, 0x1AADA754},
    {0x07, 0x00002000},
    {0x08, 0x42222249},
    {0x09, 0x000D2299},
    {0x0A, 0xD87060DD},
    {0x0B, 0x00000E72},
    {0x0C, 0x02AE7077},
    {0x0D, 0x00240376},
    {0x0E, 0X7F7F0711}
};


static TcbdRfTable_t Tc3170RfTableBandL[] = 
{
    {0x02, 0x00030000},
    {0x04, 0x00004001},
    {0x06, 0x0AADA754},
    {0x07, 0x00002000},
    {0x08, 0x42222249},
    {0x09, 0x000D2299},
    {0x0A, 0xD87060DD},
    {0x0B, 0x00000E72},
    {0x0C, 0x02AE7077},
    {0x0D, 0x00240376},
    {0x0E, 0X7F7F0907}
};

static I64U TcbdDiv64(I64U x, I64U y)
{
    I64U a, b, q, counter;

    q = 0;
    if (y != 0) {
        while (x >= y) {
            a = x >> 1;
            b = y;
            counter = 1;
            while (a >= b) {
                b <<= 1;
                counter <<= 1;
            }
            x -= b;
            q += counter;
        }
    }
    return q;
}


I32S TcbdRfInit(TcbdDevice_t *_device, TcbdBand_t _band)
{
    I32S i = 0;
    I32S size;
    I32S ret = 0;
    TcbdRfTable_t *rfTable;

    switch(_band)
    {
        case BandTypeBand3:
            size = sizeof(Tc3170RfTableBand3)/sizeof(TcbdRfTable_t);
            rfTable = Tc3170RfTableBand3;
            break;
        case BandTypeLband:
            size = sizeof(Tc3170RfTableBandL)/sizeof(TcbdRfTable_t);
            rfTable = Tc3170RfTableBandL;
            break;
        default:
            return -TCERR_UNKNOWN_BAND;
            break;
    }

    //TcbdDebug(DEBUG_DRV_RF, "bandType:%s, RfTableLen:%d\n",
    //    (_band == BandTypeBand3) ? "Band3" : "Lband", size);
    for(i = 0; i < size; i++)
    {
        ret |= TcbdRfRegWrite(_device, rfTable[i].addr, rfTable[i].data);
    }

    return ret;
}

#define Scale       22
#define Fixed(x)    (x<<Scale)
#define Mul(A,B)    ((A*B)>>Scale)
#define Div(A,B)    (TcbdDiv64((A<<Scale), B))
inline static I32S TcbdRfSetFrequency(TcbdDevice_t *_device, I32S _freqKhz, I32S _bwKhz)
{
    I32S ret = 0;
    I64U N, F;
    I64U Flo, VCO_DIV, FOffset, Fvco, FR, PLL_MODE;
    I64U N_int, intF, intVCO_DIV;
    I64U fXtal, fpfd, f_freq_khz;
    I64U Temp1, Temp2;
    I32U DIV_MODE;
    I32S RST_PLL = 1, band = 0;
    I32U rfCfg2 = 0, rfCfg4 = 0;

    FOffset = 0;

    if(_freqKhz > 1000000)//l-band
        band = 1;

    if(band== 0)    /* band III */
    {
        ret |= TcbdRfRegWrite(_device, 0x04, 0x00002002);
        ret |= TcbdRfRegWrite(_device, 0x0e, 0x7F7F0711);
    }
    else            /* LBAND */
    {
        ret |= TcbdRfRegWrite(_device, 0x04, 0x00004001);
        ret |= TcbdRfRegWrite(_device, 0x0e, 0x7F7F0907);
    }

    ret |= TcbdRfRegRead(_device, 0x06, (I32U*)&rfCfg2);
    if( ((rfCfg2 >> 28) & 0x01) == 0)
        FR = 0;
    else
        FR = 1;

    if( ((rfCfg2 >> 31) & 0x01) == 0)
        PLL_MODE = 2;
    else
        PLL_MODE = 4;

    fXtal = TCBD_DEF_OSCCLK_RF;
    f_freq_khz = _freqKhz;

    // Calculate PLL
    if (f_freq_khz < 250000) {
        // VHF
        DIV_MODE = 0x00;
        fpfd = fXtal >> FR;
        VCO_DIV = 16;

        Flo = f_freq_khz - FOffset;
        Fvco = Flo * VCO_DIV;

        Temp1 = Fvco<<FR;
        Temp2 = PLL_MODE*fXtal;
        N = Div(Temp1, Temp2);
        N_int = (N>>Scale) << Scale;
        F = ((N-N_int) * (2 << 21)) >> Scale;

        if (N_int < (19<<Scale) ) {
            FR = 1;
            fpfd = fXtal >> FR;
            VCO_DIV = 16;
            Flo = f_freq_khz - FOffset;
            Fvco = Flo * VCO_DIV;

            Temp1 = Fvco*FR;
            Temp2 = PLL_MODE*fXtal;
            N = Div(Temp1, Temp2);
            N_int = (N>>Scale) << Scale;
            F = ((N - N_int) * (2<<21)) >> Scale;
        }
        intF = F;
        intVCO_DIV = VCO_DIV;
    }
    else {
        // LBAND
        DIV_MODE = 0x01;
        fpfd = fXtal >> FR;
        VCO_DIV = 2;

        Flo = f_freq_khz - FOffset;
        Fvco = Flo * VCO_DIV;

        Temp1 = Fvco << FR;
        Temp2 = PLL_MODE*fXtal;
        N = Div(Temp1, Temp2);//Div(Temp1, Temp2);
        N_int = (N>>Scale) << Scale;
        F = ((N-N_int) * (2<<21)) >> Scale;

        if (N_int < (19<<Scale)) {
            FR = 1;

            VCO_DIV = 2;
            Flo = f_freq_khz - FOffset;
            Fvco = Flo * VCO_DIV;

            Temp1 = Fvco<<FR;
            Temp2 = PLL_MODE*fXtal;
            N = Div(Temp1, Temp2);
            N_int = (N>>Scale) << Scale;
            F = ((N-N_int) * (2<<21)) >> Scale;
        }
        intF = F;
        intVCO_DIV = VCO_DIV;
    }

    rfCfg4 = (I32U)((N_int>>Scale) & 0xFF) ;
    rfCfg4 |= ((intF&0x3FFFFF) << 8);
    rfCfg4 |= ((RST_PLL&0x01) << 30);
    rfCfg4 |= ((DIV_MODE&0x01) << 31);
    ret |= TcbdRfRegWrite(_device, 0x08, rfCfg4);

    ret |= TcbdRfRegRead(_device, 0x06, (I32U*)&rfCfg2);
    if(FR==0)
        BITCLR(rfCfg2, Bit28);
    else
        BITSET(rfCfg2, Bit28);

    ret |= TcbdRfRegWrite(_device, 0x06, rfCfg2);

    return ret;
}

I32S TcbdRfTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz)
{
    I32S ret = 0;

    if(_device->currBand != _device->prevBand)
    {
        ret |= TcbdRfInit(_device, _device->currBand);
    }
    ret |= TcbdRfSetFrequency(_device, _freqKhz, _bwKhz);

    if(ret < 0)
    {
        TcbdDebug(DEBUG_ERROR, "Failed to set frequency to RF, err:%d\n", ret);
    }
    return ret;
}
