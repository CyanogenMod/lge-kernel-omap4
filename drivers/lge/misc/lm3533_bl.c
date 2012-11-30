#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/lge/lm3533.h>


#define LM3533_DEBUG 0
 #if LM3533_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif
 

static int	old_brightness	=	-1;

/* 20120522 jeonghoon.cho@lge.com change backlight brightness step [LGE_START]*/
static int get_brightness(int data)
{
	int mLcdBacklightValues[] =   {
	0x52,0x52,0x53,0x53,0x56,0x56,0x58,0x59,0x5A,0x5B,
	0x5C,0x5D,0x5F,0x5F,0x60,0x61,0x62,0x63,0x64,0x65,
	0x67,0x68,0x6A,0x6A,0x6B,0x6C,0x6E,0x6F,0x70,0x71,
	0x72,0x73,0x74,0x75,0x78,0x79,0x7E,0x80,0x83,0x84,
	0x86,0x87,0x88,0x89,0x8D,0x8D,0x8F,0x90,0x91,0x92,
	0x93,0x94,0x95,0x96,0x97,0x98,0x9A,0x9B,0x9C,0x9D,
	0x9E,0x9F,0xA0,0xA1,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,
	0xA9,0xAA,0xAC,0xAC,0xAD,0xAD,0xAE,0xAE,0xAF,0xB0,
	0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB8,0xB9,0xB9,
	0xBA,0xBB,0xBD,0xBE,0xBF,0xC0,0xC1,0xC2,0xC4,0xC4,
	0xC4,0xC5,0xC5,0xC5,0xC6,0xC6,0xC7,0xC7,0xC8,0xC8,
	0xC9,0xC9,0xCA,0xCA,0xCA,0xCB,0xCB,0xCB,0xCC,0xCE,
	0xCF,0xD0,0xD1,0xD2,0xD3,0xD3,0xD4,0xD4,0xD5,0xD5,
	0xD6,0xD6,0xD7,0xD7,0xD8,0xD9,0xDA,0xDA,0xDA,0xDA,
	0xDB,0xDB,0xDC,0xDC,0xDE,0xDE,0xDF,0xDF,0xE0,0xE0,
	0xE0,0xE1,0xE1,0xE1,0xE2,0xE2,0xE3,0xE4,0xE5,0xE5,
	0xE5,0xE5,0xE6,0xE6,0xE6,0xE6,0xE7,0xE7,0xE7,0xE8,
	0xE8,0xE8,0xE9,0xE9,0xEA,0xEA,0xEB,0xEB,0xEB,0xEC,
	0xEC,0xEC,0xED,0xED,0xEF,0xEF,0xF0,0xF0,0xF1,0xF1,
	0xF2,0xF2,0xF3,0xF3,0xF4,0xF4,0xF5,0xF5,0xF6,0xF6,
	0xF6,0xF7,0xF7,0xF7,0xF8,0xF8,0xF8,0xF9,0xF9,0xF9,
	0xFA,0xFA,0xFB,0xFB,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,
	0xFD,0xFD,0xFE,0xFE,0xFF,0xFF,
	};

	return (mLcdBacklightValues[data-30]);
}
/* 20120522 jeonghoon.cho@lge.com change backlight brightness step [LGE_END]*/

/* SYSFS for brightness control */
static ssize_t	brightness_show(struct device* dev, 
		struct device_attribute* attr, char* buf)
{
	struct	lm3533_platform_data*	pdata	=	dev->platform_data;
	int	val;
#if 0
	if ((val = lm3533_get_brightness_control(&pdata->private)) < 0)
		return	0;
#endif
	return	snprintf(buf, PAGE_SIZE, "%d\n", old_brightness);
}

static ssize_t	brightness_store(struct device* dev, 
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct	lm3533_platform_data*	pdata	=	dev->platform_data;
	int	brightness	=	simple_strtol(buf, NULL, 10);

	DEBUG_MSG("brightness_store = [%d] \n",brightness);

	if (brightness > 0 && brightness < 30)	// MIN brightness to be off
		brightness	=	30;

	if ((brightness < 0) || (brightness > 255)) // Invalid brightness
		goto	exit;

	if (old_brightness == brightness) // No need to change the brightness
		goto	exit;

	if (brightness == 0) {	// Zero-Brightness, Turn off LM3533
		lm3533_set_hwen(&pdata->private, pdata->gpio_hwen, 0);
		old_brightness	=	brightness;
		goto	exit;
	}	
		if(old_brightness==0)
			lm3533_set_hwen(&pdata->private, pdata->gpio_hwen, 1);	
	lm3533_set_brightness_control(&pdata->private, brightness);
	//printk("[dyotest]lm3533 brightness UI value=%d, reg=0x%x\n",brightness,get_brightness(brightness));

	old_brightness	=	brightness;

exit:
	return	count;
}

static DEVICE_ATTR(brightness, 0660, brightness_show, brightness_store);

/* SYSFS for LCD backlight ON/OFF
 */
static ssize_t	enable_show(struct device* dev, 
		struct device_attribute* attr, char *buf)
{
	struct	lm3533_platform_data*	pdata	=	dev->platform_data;
	int	val	=	lm3533_get_hwen(&pdata->private, pdata->gpio_hwen);

	return	snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t	enable_store(struct device* dev, 
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct	lm3533_platform_data*	pdata	=	dev->platform_data;

	DEBUG_MSG("enable_store = [%d] \n",(int)simple_strtol(buf, NULL, 10));

	lm3533_set_hwen(&pdata->private, pdata->gpio_hwen, (int)simple_strtol(buf, NULL, 10));

	return	count;
}

static DEVICE_ATTR(enable, 0664, enable_show, enable_store);

/* Driver
 */
static int __devinit lm3533bl_probe(struct i2c_client* client,
							const struct i2c_device_id* id)
{
	struct lm3533_platform_data*	pdata;
	int		ret = 0;

	pdata	=	client->dev.platform_data;
	gpio_request(pdata->gpio_hwen, "backlight_enable");
	gpio_direction_output(pdata->gpio_hwen, 1);	// OUTPUT

	lm3533_init(&pdata->private, client);

	ret = device_create_file(&client->dev, &dev_attr_brightness);
	ret = device_create_file(&client->dev, &dev_attr_enable);

	old_brightness	=	lm3533_get_brightness_control(&pdata->private);

	return	ret;
}

static int __devexit lm3533bl_remove(struct i2c_client* client)
{
	device_remove_file(&client->dev, &dev_attr_brightness);
	device_remove_file(&client->dev, &dev_attr_enable);	
	return	0;
}

static const struct i2c_device_id lm3533bl_ids[] = {
	{	LM3533_I2C_NAME, 0 },	// LM3533
	{},
};

static struct i2c_driver lm3533bl_driver = {
	.probe		= lm3533bl_probe,
	.remove		= __devexit_p(lm3533bl_remove),
	.id_table	= lm3533bl_ids,
	.driver = {
		.name	= LM3533_I2C_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lm3533bl_init(void)
{
	return	i2c_add_driver(&lm3533bl_driver);
}

static void __exit lm3533bl_exit(void)
{
	i2c_del_driver(&lm3533bl_driver);
}

module_init(lm3533bl_init);
module_exit(lm3533bl_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Backlight driver (LM3533)");
MODULE_LICENSE("GPL");
