#ifndef __LM3533_H
#define __LM3533_H

#include <linux/kernel.h>
#include <asm/gpio.h>
#include <linux/i2c.h>

#define LM3533_I2C_NAME		"lm3533"
#define LM3533_I2C_ADDR		0x36

struct lm3533_private_data {
	unsigned char	reg_be; // control bank enable register
	unsigned char	reg_brt; //BRIGHTNESS REGISTERS
	unsigned char	reg_obps; // OVP/BOOST FREQUENCY/PWM POLARITY SELECT
	unsigned char	reg_ocr; //OUTPUT CONFIGURATION REGISTER 1
	unsigned char	reg_ssttr; // LED CURRENT START UP/SHUTDOWN TRANSITION TIME REGISTER
	unsigned char	reg_rttr; // LED CURRENT RUN-TIME TRANSITION TIME REGISTE
	unsigned char	reg_fscr; // CONTROL BANK FULL-SCALE CURRENT REGISTERS
	unsigned char	reg_bcr; // CONTROL BANK A/B BRIGHTNESS CONFIGURATION REGISTER
	

	struct i2c_client*	client;
	struct mutex	update_lock;
};

struct lm3533_platform_data {
	int		gpio_hwen;
        int             gpio_pwm;
	struct lm3533_private_data	private;
};

#define LM3533_MAX_BRT			0xff
#define LM3533_DEFAULT_BRT		0xe6
#define LM3533_MIN_BRT			0x64
#define UI_MAX					255
#define UI_DEFAULT				153
#define UI_MIN					30


#define	LM3533_REG_BE		0x27
#define	LM3533_REG_BRT   	0x40
#define	LM3533_REG_OBPS		0x2C
#define LM3533_REG_OCR		0x10
#define LM3533_REG_SSTTR	0x12
#define LM3533_REG_RTTR		0x13
#define LM3533_REG_FSCR		0x1f
#define LM3533_REG_BCR		0x1a

#define	LM3533_BMASK		0xff	// Brightness Mask

int	lm3533_get_brightness_control(struct lm3533_private_data* pdata);
int	lm3533_set_brightness_control(struct lm3533_private_data* pdata, int val);
int	lm3533_get_hwen(struct lm3533_private_data* pdata, int gpio);
int	lm3533_set_hwen(struct lm3533_private_data* pdata, int gpio, int status);
int	lm3533_init(struct lm3533_private_data* pdata, struct i2c_client* client);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)

int lm3533_get_lcd_on_off();
#endif
#endif
