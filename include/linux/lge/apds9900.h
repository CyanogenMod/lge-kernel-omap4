/* include/linux/lge/apds9900.h
 */

#ifndef __APDS9900_H
#define __APDS9900_H

struct apds9900_platform_data {
	unsigned int atime;
	unsigned int wtime;
	unsigned int ptime;
	unsigned int pers;
	unsigned int ppcount;
	unsigned int pdrive;
	unsigned int pdiode;
	unsigned int pgain;
	unsigned int again;
	unsigned int threshhold_high;
	unsigned int threshhold_low;
	unsigned int coeff_b;
	unsigned int coeff_c;
	unsigned int coeff_d;
	unsigned int apds_ga;
	unsigned int apds_df;
	unsigned int irq_gpio;
	unsigned int ldo_gpio;
};

#endif
