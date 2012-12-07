
#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_io.h"

I32S TcbdChangeChipAddress(TcbdDevice_t *_device, I08U addr)
{
    _device->chipAddr = addr;
    return TcbdRegWrite(_device, TCBD_CHIPADDR, addr);
}

I32S TcbdEnableSlaveCommandAck(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I16U outData = SWAP16((0x1<<15));
    I16U outEnable = SWAP16((0x1<<15));

    ret |= TcbdRegWriteExCon(_device, TCBD_GPIO_LR, (I08U*)&outData, 2);
    ret |= TcbdRegWriteExCon(_device, TCBD_GPIO_DR, (I08U*)&outEnable, 2);

    return ret;
}

I32S TcbdInitDiversityIo(TcbdDevice_t *_device, TcbdDivIo_t _divIo)
{
    return  TcbdRegWrite(_device, TCBD_DIVIO, (I08U)_divIo);
}
