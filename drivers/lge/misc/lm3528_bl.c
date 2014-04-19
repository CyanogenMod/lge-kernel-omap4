#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/lge/lm3528.h>


#define LM3528_DEBUG 0
 #if LM3528_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif
 

static int	old_brightness	=	-1;

/* SYSFS for brightness control
 */
static ssize_t	brightness_show(struct device* dev,
								struct device_attribute* attr, char* buf)
{
	struct lm3528_platform_data*	pdata	=	dev->platform_data;
	int		val;

	if ((val = lm3528_get_bmain(&pdata->private)) < 0)
		return	0;

	return	snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t brightness_store(struct device* dev,
								struct device_attribute* attr,
								const char* buf, size_t count)
{
	struct lm3528_platform_data*	pdata	=	dev->platform_data;
	int		brightness	=	simple_strtol(buf, NULL, 10);

	DEBUG_MSG("brightness_store = [%d] \n",brightness);

	if (brightness > 0 && brightness < 30)	// MIN brightness to be off
		brightness	=	30;

	if ((brightness < 0) || (brightness > 127)) // Invalid brightness
		goto	exit;

	if (old_brightness == brightness) // No need to change the brightness
		goto	exit;

	if (brightness == 0) {	// Zero-Brightness, Turn off LM3528
		lm3528_set_hwen(&pdata->private, pdata->gpio_hwen, 0);
		old_brightness	=	brightness;
		goto	exit;
	}

	//if (old_brightness < brightness) 
	{	// Dimming up
		if (old_brightness == 0)
		{		
			 pdata->private.reg_gp = pdata->private.reg_gp & 0xE7;
			lm3528_set_hwen(&pdata->private, pdata->gpio_hwen, 1);	
		}
		else if((pdata->private.reg_gp & 0x18) == 0)
		{
			 pdata->private.reg_gp = pdata->private.reg_gp | 0x18;
			lm3528_set_hwen(&pdata->private, pdata->gpio_hwen, 1);				
		}
	}

	lm3528_set_bmain(&pdata->private, brightness);

	old_brightness	=	brightness;

exit:
	return	count;
}

static DEVICE_ATTR(brightness, 0664, brightness_show, brightness_store);

/* SYSFS for LCD backlight ON/OFF
 */
static ssize_t	enable_show(struct device* dev,
								struct device_attribute* attr, char *buf)
{
	struct lm3528_platform_data*	pdata	=	dev->platform_data;
	int		val	=	lm3528_get_hwen(&pdata->private, pdata->gpio_hwen);

	return	snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t enable_store(struct device* dev,
								struct device_attribute* attr,
								const char* buf, size_t count)
{
	struct lm3528_platform_data*	pdata	=	dev->platform_data;

	DEBUG_MSG("enable_store = [%d] \n",(int)simple_strtol(buf, NULL, 10));

	lm3528_set_hwen(&pdata->private, pdata->gpio_hwen, (int)simple_strtol(buf, NULL, 10));

	return	count;
}

static DEVICE_ATTR(enable, 0664, enable_show, enable_store);

/* Driver
 */
static int __devinit lm3528bl_probe(struct i2c_client* client,
							const struct i2c_device_id* id)
{
	struct lm3528_platform_data*	pdata;
	int		ret = 0;

	pdata	=	client->dev.platform_data;
	gpio_request(pdata->gpio_hwen, "backlight_enable");
	gpio_direction_output(pdata->gpio_hwen, 1);	// OUTPUT

	lm3528_init(&pdata->private, client);

	ret = device_create_file(&client->dev, &dev_attr_brightness);
	ret = device_create_file(&client->dev, &dev_attr_enable);

	old_brightness	=	lm3528_get_bmain(&pdata->private);

	return	ret;
}

static int __devexit lm3528bl_remove(struct i2c_client* client)
{
	device_remove_file(&client->dev, &dev_attr_brightness);
	device_remove_file(&client->dev, &dev_attr_enable);	
	return	0;
}

static const struct i2c_device_id lm3528bl_ids[] = {
	{	LM3528_I2C_NAME, 0 },	// LM3528
	{},
};

static struct i2c_driver lm3528bl_driver = {
	.probe		= lm3528bl_probe,
	.remove		= __devexit_p(lm3528bl_remove),
	.id_table	= lm3528bl_ids,
	.driver = {
		.name	= LM3528_I2C_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lm3528bl_init(void)
{
	return	i2c_add_driver(&lm3528bl_driver);
}

static void __exit lm3528bl_exit(void)
{
	i2c_del_driver(&lm3528bl_driver);
}

module_init(lm3528bl_init);
module_exit(lm3528bl_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Backlight driver (LM3528)");
MODULE_LICENSE("GPL");
