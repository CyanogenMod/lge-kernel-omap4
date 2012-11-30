/*
 * MMC definitions for OMAP2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach/mmc.h>

struct mmc_card;

typedef int (*mmc_card_detect_func)(struct device *dev, int slot);

struct omap2_hsmmc_info {
	u8	mmc;		/* controller 1/2/3 */
	u32	caps;		/* 4/8 wires and any additional host
				 * capabilities OR'd (ref. linux/mmc/host.h) */
	bool	transceiver;	/* MMC-2 option */
	bool	ext_clock;	/* use external pin for input clock */
	bool	cover_only;	/* No card detect - just cover switch */
	bool	nonremovable;	/* Nonremovable e.g. eMMC */
	bool	power_saving;	/* Try to sleep or power off when possible */
	bool	no_off;		/* power_saving and power is not to go off */
	bool	no_off_init;	/* no power off when not in MMC sleep state */
	bool	vcc_aux_disable_is_sleep; /* Regulator off remapped to sleep */
	int	gpio_cd;	/* or -EINVAL */
	int	gpio_wp;	/* or -EINVAL */
#ifdef CONFIG_MACH_LGE_MMC_COVER//20110409	KIMBYUNGCHUL SD_CARD_DETECTION_UPDATE_0409	[START]
		int sd_cover;
#endif
	char	*name;		/* or NULL for default */
	struct device *dev;	/* returned: pointer to mmc adapter */
	int	ocr_mask;	/* temporary HACK */
	struct mmc_platform_data *mmc_data;
	/* Remux (pad configuration) when powering on/off */
	void (*remux)(struct device *dev, int slot, int power_on);
	/* init some special card */
	void (*init_card)(struct mmc_card *card);
	/* LGE_SJIT 2012-01-09 [dojip.kim@lge.com]
	 * no_suspend. Some drivers don't use pm on MMC (eg. BRCM WiFi)
	 */
	bool    no_suspend;
};

/* LGE_SJIT_S 2011-09-27 [choongryeol.lee@lge.com] */
#ifdef CONFIG_TIWLAN_SDIO
int omap_wifi_status_register(void (*callback)(int card_present,
        void *dev_id), void *dev_id);
int omap_wifi_status(struct device *dev, int slot);
#endif
/* LGE_SJIT_E 2011-09-27 [choongryeol.lee@lge.com] */

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)

void omap2_hsmmc_init(struct omap2_hsmmc_info *);

#else

static inline void omap2_hsmmc_init(struct omap2_hsmmc_info *info)
{
}

#endif
