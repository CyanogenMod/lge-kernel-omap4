
#ifndef __TCBD_DRV_RF_H__
#define __TCBD_DRV_RF_H__

TCBB_FUNC I32S TcbdRfInit(TcbdDevice_t *_device, TcbdBand_t _band);
TCBB_FUNC I32S TcbdRfTuneFrequency(TcbdDevice_t *_device, I32U _freqKhz, I32S _bwKhz);

#endif //__TCBD_DRV_RF_H__
