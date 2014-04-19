/* include/linux/lge/apds9900.h
 */

#ifndef __APDS9190_H
#define __APDS9190_H

struct apds9190_platform_data {
	unsigned int atime;
	unsigned int wtime;
	unsigned int ptime;
	unsigned int pers;
	unsigned int ppcount;
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold;
	unsigned int irq_gpio;
	unsigned int ldo_gpio;
};

#endif
