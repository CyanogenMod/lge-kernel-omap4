#include <linux/lge/lm3530.h>
#include <linux/power_supply.h>

#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)

static int bl_on_off=1;

int lm3530_get_lcd_on_off()
{
	        return bl_on_off;
}
#endif


int	lm3530_read_byte(struct lm3530_private_data* pdata, int reg)
{
	int		ret;

	mutex_lock(&pdata->update_lock);
	ret	=	i2c_smbus_read_byte_data(pdata->client, reg);
	mutex_unlock(&pdata->update_lock);

	return	ret;
}

int	lm3530_write_byte(struct lm3530_private_data* pdata, int reg, int value)
{
	int		ret;

	mutex_lock(&pdata->update_lock);
	ret	=	i2c_smbus_write_byte_data(pdata->client, reg, value);
	mutex_unlock(&pdata->update_lock);

	return	ret;
}
void	lm3530_brr_write(struct lm3530_private_data* pdata)
{

	lm3530_write_byte(pdata, LM3530_REG_BRR, pdata->reg_brr);

}
static void	lm3530_store(struct lm3530_private_data* pdata)
{
	lm3530_write_byte(pdata, LM3530_REG_GP, pdata->reg_gp);
	lm3530_write_byte(pdata, LM3530_REG_BRR, pdata->reg_brr);
	lm3530_write_byte(pdata, LM3530_REG_BRT, pdata->reg_brt);

}

static void	lm3530_load(struct lm3530_private_data* pdata)
{
//                                                                    
#if defined(CONFIG_MACH_LGE_P2) || defined(CONFIG_MACH_LGE_U2)
	pdata->reg_gp   =	0x15;
#else
	pdata->reg_gp	=	lm3530_read_byte(pdata, LM3530_REG_GP);
#endif
	pdata->reg_brr   =       lm3530_read_byte(pdata, LM3530_REG_BRR);
	pdata->reg_brt	=	lm3530_read_byte(pdata, LM3530_REG_BRT);
}

int	lm3530_set_hwen(struct lm3530_private_data* pdata, int gpio, int status)
{
	if (status == 0) {
		lm3530_load(pdata);
		gpio_set_value(gpio, 0);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
		bl_on_off=0;
#endif
		return	0;
	}

	gpio_set_value(gpio, 1);
	lm3530_write_byte(pdata, LM3530_REG_GP, 0x14);
	lm3530_write_byte(pdata, LM3530_REG_BRT, 0x01);
	lm3530_store(pdata);
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
	bl_on_off=1;
#endif
	return	1;
}

int	lm3530_get_hwen(struct lm3530_private_data* pdata, int gpio)
{
	return	gpio_get_value(gpio);
}

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C) || defined(CONFIG_LG_FW_MAX17048_FUEL_GAUGE_I2C)
extern int get_bat_present(void);
extern enum power_supply_type get_charging_ic_status(void);
#endif

int	lm3530_set_brightness_control(struct lm3530_platform_data* pdata, int val)
{
	int ret;
	static int old_brightness;
	if ((val < 0) || (val > UI_MAX))
		return	-EINVAL;

	if(old_brightness == val)
	{
		printk("%s: new val==old_brightness already. just return.\n",__func__);
		return 0;
	}

	old_brightness = val;

	if(val >= UI_MIN && val <= UI_DEFAULT)
	{
		val = (val - UI_MIN) * (LM3530_DEFAULT_BRT - LM3530_MIN_BRT) / (UI_DEFAULT - UI_MIN) + LM3530_MIN_BRT;
	}
	else if(val >UI_DEFAULT)
	{
		val = (val - UI_DEFAULT) * (LM3530_MAX_BRT - LM3530_DEFAULT_BRT) / (UI_MAX - UI_DEFAULT) + LM3530_DEFAULT_BRT;
	}

	/*                                                                                    */


	if(val)
	{
		if(lm3530_get_hwen(&pdata->private, pdata->gpio_hwen))
		{
			printk("%s:hwen=1 already. leave it.\n",__func__);
		}
		else
		{
			lm3530_set_hwen(&pdata->private, pdata->gpio_hwen, 1);
			printk("%s:hwen=0 now. setting it 1.\n",__func__);
		}
		ret = lm3530_write_byte(&pdata->private, LM3530_REG_BRT, val);
		printk("[dyotest]%s,%d val=0x%x\n",__func__,__LINE__,val);
	}
	else
	{
		ret = lm3530_write_byte(&pdata->private, LM3530_REG_BRT, val);
		printk("[dyotest]%s,%d val=0x%x\n",__func__,__LINE__,val);

		lm3530_set_hwen(&pdata->private, pdata->gpio_hwen, 0);
	}

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C) || defined(CONFIG_LG_FW_MAX17048_FUEL_GAUGE_I2C)
	if (get_bat_present() == 0 &&
	    get_charging_ic_status() == POWER_SUPPLY_TYPE_FACTORY)
		return -1;
#endif
	return ret;
}

int	lm3530_get_brightness_control(struct lm3530_private_data* pdata)
{
	int		val;

	val	=	lm3530_read_byte(pdata, LM3530_REG_BRT);
	if (val < 0)
		return	val;

	return	(val & LM3530_BMASK);
}

int	lm3530_init(struct lm3530_private_data* pdata, struct i2c_client* client)
{
	mutex_init(&pdata->update_lock);
	pdata->client	=	client;

	lm3530_load(pdata);
//                                                                    
	lm3530_store(pdata);

	return 0;
}
EXPORT_SYMBOL(lm3530_brr_write);
EXPORT_SYMBOL(lm3530_init);
EXPORT_SYMBOL(lm3530_set_hwen);
EXPORT_SYMBOL(lm3530_get_hwen);
EXPORT_SYMBOL(lm3530_set_brightness_control);
EXPORT_SYMBOL(lm3530_get_brightness_control);
EXPORT_SYMBOL(lm3530_read_byte);
EXPORT_SYMBOL(lm3530_write_byte);

MODULE_AUTHOR("LG Electronics (dongjin73.kim@lge.com)");
MODULE_DESCRIPTION("Multi Display LED driver");
MODULE_LICENSE("GPL");
