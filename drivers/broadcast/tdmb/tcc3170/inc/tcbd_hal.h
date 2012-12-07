
#ifndef __TCBD_HAL_H__
#define __TCBD_HAL_H__

//#define IRQ_TC317X INT_EI3
//#define __USE_TC_CPU__

void TchalInit(void);
void TchalResetDevice(void);
void TchalPowerOnDevice(void);
void TchalPowerDownDevice(void);
void TchalIrqSetup(void);

#endif //__TCBD_HAL_H__
