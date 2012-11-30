#ifndef __LM3530_H
#define __LM3530_H

#include <linux/kernel.h>
#include <asm/gpio.h>
#include <linux/i2c.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define LM3530_I2C_NAME		"lm3530"
#define LM3530_I2C_ADDR		0x38

struct lm3530_private_data {
	unsigned char	reg_gp;
	unsigned char	reg_brr;
	unsigned char	reg_brt;

	struct i2c_client*	client;
	struct mutex	update_lock;
};

struct lm3530_platform_data {
	int		gpio_hwen;
        int             gpio_pwm;
	struct lm3530_private_data	private;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define	LM3530_REG_GP		0x10
#define	LM3530_REG_BRR   	0x30
#define	LM3530_REG_BRT		0xA0

#define	LM3530_BMASK		0x7f	// Brightness Mask
void	lm3530_brr_write(struct lm3530_private_data* pdata);
int	lm3530_get_brightness_control(struct lm3530_private_data* pdata);
int	lm3530_set_brightness_control(struct lm3530_private_data* pdata, int val);
int	lm3530_get_hwen(struct lm3530_private_data* pdata, int gpio);
int	lm3530_set_hwen(struct lm3530_private_data* pdata, int gpio, int status);
int	lm3530_init(struct lm3530_private_data* pdata, struct i2c_client* client);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)

int lm3530_get_lcd_on_off();
#endif
#endif
