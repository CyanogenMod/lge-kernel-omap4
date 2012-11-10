/*
 * Header file for Display Sub-System
 */

#ifndef __LGE_PLAT_DISPLAY_H
#define __LGE_PLAT_DISPLAY_H

#define OMAP_DSI_MODE_CMD		0
#define OMAP_DSI_MODE_VIDEO		1

struct lge_dss_info {
	struct omap_dss_device dev;
	unsigned dsi_mode;			/* 0 : command, 1 : video */ 
	unsigned num_data_lanes;
	struct {
		unsigned vfp;
		unsigned vsa;
		unsigned vbp;
		unsigned hfp;
		unsigned hsa;
		unsigned hbp;
	} video_mode_timings;
	bool ext_te;
	unsigned ext_te_gpio;
};

#endif /* __LGE_PLAT_DISPLAY_H */
