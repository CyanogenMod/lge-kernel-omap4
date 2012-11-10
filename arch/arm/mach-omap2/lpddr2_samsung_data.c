/*
 * LPDDR2 data as per JESD209-2
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * Jugwan Eom <jugwan.eom@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <mach/emif.h>

const struct lpddr2_timings timings_samsung_400_mhz = {
	.max_freq	= 400000000,
	.RL		= 6,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

const struct lpddr2_timings timings_samsung_333_mhz = {
	.max_freq	= 333000000,
	.RL		= 5,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 15,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

const struct lpddr2_timings timings_samsung_200_mhz = {
	.max_freq	= 200000000,
	.RL		= 3,
	.tRPab		= 21,
	.tRCD		= 18,
	.tWR		= 15,
	.tRASmin	= 42,
	.tRRD		= 10,
	.tWTRx2		= 20,
	.tXSR		= 140,
	.tXPx2		= 15,
	.tRFCab		= 130,
	.tRTPx2		= 15,
	.tCKE		= 3,
	.tCKESR		= 15,
	.tZQCS		= 90,
	.tZQCL		= 360,
	.tZQINIT	= 1000,
	.tDQSCKMAXx2	= 11,
	.tRASmax	= 70,
	.tFAW		= 50
};

const struct lpddr2_min_tck min_tck_samsung = {
	.tRL		= 3,
	.tRP_AB		= 3,
	.tRCD		= 3,
	.tWR		= 3,
	.tRAS_MIN	= 3,
	.tRRD		= 2,
	.tWTR		= 2,
	.tXP		= 2,
	.tRTP		= 2,
	.tCKE		= 3,
	.tCKESR		= 3,
	.tFAW		= 8
};

struct lpddr2_device_info samsung_2G_S4 = {
	.device_timings = {
		&timings_samsung_200_mhz,
		&timings_samsung_333_mhz,
		&timings_samsung_400_mhz
	},
	.min_tck	= &min_tck_samsung,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_2Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

struct lpddr2_device_info samsung_2G_S2 = {
	.device_timings = {
		&timings_samsung_200_mhz,
		&timings_samsung_333_mhz,
		&timings_samsung_400_mhz
	},
	.min_tck	= &min_tck_samsung,
	.type		= LPDDR2_TYPE_S2,
	.density	= LPDDR2_DENSITY_2Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144,
};

struct lpddr2_device_info samsung_4G_S4 = {
	.device_timings = {
		&timings_samsung_200_mhz,
		&timings_samsung_333_mhz,
		&timings_samsung_400_mhz
	},
	.min_tck	= &min_tck_samsung,
	.type		= LPDDR2_TYPE_S4,
	.density	= LPDDR2_DENSITY_4Gb,
	.io_width	= LPDDR2_IO_WIDTH_32,
	.emif_ddr_selfrefresh_cycles = 262144, // LGE_SJIT 2011-12-2 [jongrak.kwon@lge.com] resolving SDRAM performance loss
};
