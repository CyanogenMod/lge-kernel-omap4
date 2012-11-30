#include <linux/lge/lm3533.h>
#include <linux/power_supply.h>

#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)

static int bl_on_off=1;

int lm3533_get_lcd_on_off()
{
	        return bl_on_off;
}
#endif


static int	lm3533_read_byte(struct lm3533_private_data* pdata, int reg)
{
	int		ret;

	mutex_lock(&pdata->update_lock);
	ret	=	i2c_smbus_read_byte_data(pdata->client, reg);
	mutex_unlock(&pdata->update_lock);

	return	ret;
}

static int	lm3533_write_byte(struct lm3533_private_data* pdata, int reg, int value)
{
	int		ret;

	mutex_lock(&pdata->update_lock);
	ret	=	i2c_smbus_write_byte_data(pdata->client, reg, value);
	mutex_unlock(&pdata->update_lock);

	return	ret;
}

static void	lm3533_store(struct lm3533_private_data* pdata)
{
	lm3533_write_byte(pdata, LM3533_REG_BE, pdata->reg_be);
	lm3533_write_byte(pdata, LM3533_REG_BRT, pdata->reg_brt);
	lm3533_write_byte(pdata, LM3533_REG_OBPS, pdata->reg_obps);
	lm3533_write_byte(pdata, LM3533_REG_OCR, pdata->reg_ocr);
	lm3533_write_byte(pdata, LM3533_REG_SSTTR, pdata->reg_ssttr);
	lm3533_write_byte(pdata, LM3533_REG_RTTR, pdata->reg_rttr);
	lm3533_write_byte(pdata, LM3533_REG_FSCR, pdata->reg_fscr);
	lm3533_write_byte(pdata, LM3533_REG_BCR, pdata->reg_bcr);

}

static void	lm3533_load(struct lm3533_private_data* pdata)
{
	pdata->reg_be   =	0x01;	// Only Bank A Enable.
	pdata->reg_brt   =	lm3533_read_byte(pdata, LM3533_REG_BRT);	// brightness value.
	pdata->reg_obps   =	0x0a;	// Boost OVP 24V for 10 LEDs.
	pdata->reg_ocr   =	0x90;	// HVLED1, HVLED2 -> Bank A only use.
	pdata->reg_ssttr   =	lm3533_read_byte(pdata, LM3533_REG_SSTTR);	// default 2.048ms
	pdata->reg_rttr   =       lm3533_read_byte(pdata, LM3533_REG_RTTR);	// default 2.048ms
	pdata->reg_fscr	=	0x13;	// Max current 20.2mA.
	pdata->reg_bcr   =	0x00;	// Exponential Type.
}

int	lm3533_set_hwen(struct lm3533_private_data* pdata, int gpio, int status)
{
	if (status == 0) {
		lm3533_load(pdata);
		gpio_set_value(gpio, 0);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
		bl_on_off=0;
#endif
		return	0;
	}

	gpio_set_value(gpio, 1);
	lm3533_store(pdata);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
	bl_on_off=1;
#endif
	return	1;
}

int	lm3533_get_hwen(struct lm3533_private_data* pdata, int gpio)
{
	return	gpio_get_value(gpio);
}

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C) || defined(CONFIG_LG_FW_MAX17048_FUEL_GAUGE_I2C)
extern int get_bat_present(void);
extern enum power_supply_type get_charging_ic_status(void);
#endif

int	lm3533_set_brightness_control(struct lm3533_private_data* pdata, int val)
{
	if ((val < 0) || (val > UI_MAX))
		return	-EINVAL;

		if(val >= UI_MIN && val <= UI_DEFAULT)
		{
			val = (val - UI_MIN) * (LM3533_DEFAULT_BRT - LM3533_MIN_BRT) / (UI_DEFAULT - UI_MIN) + LM3533_MIN_BRT;
		}
		else if(val >UI_DEFAULT)
		{
			val = (val - UI_DEFAULT) * (LM3533_MAX_BRT - LM3533_DEFAULT_BRT) / (UI_MAX - UI_DEFAULT) + LM3533_DEFAULT_BRT;
		}
		printk("[dyotest]%s val=0x%x\n",__func__,val);
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-23, To prevent Backlight control in Factory*/
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C) || defined(CONFIG_LG_FW_MAX17048_FUEL_GAUGE_I2C)
	if (get_bat_present() == 0 &&
	    get_charging_ic_status() == POWER_SUPPLY_TYPE_FACTORY)
		return -1;
#endif
	return	lm3533_write_byte(pdata, LM3533_REG_BRT, val);
}

int	lm3533_get_brightness_control(struct lm3533_private_data* pdata)
{
	int		val;

	val	=	lm3533_read_byte(pdata, LM3533_REG_BRT);
	if (val < 0)
		return	val;

	return	(val & LM3533_BMASK);
}

int	lm3533_init(struct lm3533_private_data* pdata, struct i2c_client* client)
{
	mutex_init(&pdata->update_lock);
	pdata->client	=	client;

	lm3533_load(pdata);
// LGE_CHANGE [bk.shin@lge.com] 2012-02-01, LGE_P940, add from P940 GB
	lm3533_store(pdata);

	return 0;
}

EXPORT_SYMBOL(lm3533_init);
EXPORT_SYMBOL(lm3533_set_hwen);
EXPORT_SYMBOL(lm3533_get_hwen);
EXPORT_SYMBOL(lm3533_set_brightness_control);
EXPORT_SYMBOL(lm3533_get_brightness_control);

MODULE_AUTHOR("LG Electronics (dongjin73.kim@lge.com)");
MODULE_DESCRIPTION("Multi Display LED driver");
MODULE_LICENSE("GPL");
