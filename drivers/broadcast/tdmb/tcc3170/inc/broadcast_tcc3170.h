#ifndef __BROADCAST_TCC3170_H__
#define __BROADCAST_TCC3170_H__
#include "../../broadcast_tdmb_typedef.h"

void tdmb_tcc3170_set_userstop(void);
int tdmb_tcc3170_mdelay(int32 ms);
void tdmb_tcc3170_must_mdelay(int32 ms);
int tdmb_tcc3170_power_on(void);
int tdmb_tcc3170_power_off(void);
int tdmb_tcc3170_is_power_on(void);

int tdmb_tcc3170_select_antenna(unsigned int sel);
int tdmb_tcc3170_i2c_write_burst(uint16 waddr, uint8* wdata, int length);
int tdmb_tcc3170_i2c_read_burst(uint16 raddr, uint8* rdata, int length);
int tdmb_tcc3170_i2c_write16(unsigned short reg, unsigned short val);
int tdmb_tcc3170_i2c_read16(uint16 reg, uint16 *ret);

#endif // __BROADCAST_TCC3170_H__
