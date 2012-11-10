
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <asm/gpio.h>
#include <asm/system.h>
#include <linux/lge/lp8720.h>

static unsigned char lp8720_output_status = 0x3f;//0x00; //default on 0x3F
struct i2c_client *lp8720_client=NULL;

static inline int lp8720_read_reg(struct i2c_client *client, int reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static inline int lp8720_write_reg(struct i2c_client *client,
		int reg, int value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

void subpm_set_output(subpm_output_enum outnum, int onoff)
{
	if (outnum > 5) {
		dev_err(&lp8720_client->dev, "outnum error\n");
		return;
	}

	if (onoff == 0)
		lp8720_output_status &= ~(1<<outnum);
	else
		lp8720_output_status |= (1<<outnum);
}
EXPORT_SYMBOL(subpm_set_output);

void subpm_output_enable(void)
{
	if(lp8720_client == NULL)
		return;
	
	lp8720_write_reg(lp8720_client,
			LP8720_OUTPUT_ENABLE, 0x80 | lp8720_output_status);
}
EXPORT_SYMBOL(subpm_output_enable);

static void lp8720_init(struct i2c_client *client)
{
	int ret;
	struct lp8720_platform_data *pdata;

	pdata = client->dev.platform_data;
	ret = gpio_request(pdata->en_gpio_num, "lp8720");
	gpio_direction_output(pdata->en_gpio_num, 1);
	gpio_set_value(pdata->en_gpio_num, 0);
	mdelay(10);
	lp8720_write_reg(client, LP8720_GENERAL_SETTING, 0x01);
	lp8720_write_reg(client, LP8720_LDO1_SETTING,
			LP8720_STARTUP_DELAY_2TS | 0x0c); //1.8v
	lp8720_write_reg(client, LP8720_LDO2_SETTING,
			LP8720_STARTUP_DELAY_4TS | 0x17); //2.7v
	lp8720_write_reg(client, LP8720_LDO3_SETTING,
			LP8720_STARTUP_DELAY_2TS | 0x19); //2.8v
	lp8720_write_reg(client, LP8720_LDO4_SETTING,
			LP8720_STARTUP_DELAY_0 | 0x11); //1.8v
	lp8720_write_reg(client, LP8720_LDO5_SETTING,
			LP8720_STARTUP_DELAY_5TS | 0x19); //2.8v
	lp8720_write_reg(client, LP8720_BUCK_SETTING1,
			LP8720_STARTUP_DELAY_6TS | 0x09);
	lp8720_write_reg(client, LP8720_BUCK_SETTING2, 0x09); //1.2v
	lp8720_write_reg(client, LP8720_OUTPUT_ENABLE, 0x40);
	//lp8720_write_reg(client, LP8720_OUTPUT_ENABLE,
	//		0x80 | lp8720_output_status);
	mdelay(10);
	gpio_set_value(pdata->en_gpio_num, 0);
	mdelay(10);
	gpio_free(pdata->en_gpio_num);
	mdelay(10);
}

static int __devinit lp8720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (i2c_get_clientdata(client))
		return -EBUSY;

	lp8720_client = client;
	lp8720_init(client);

	return 0;
}

static int __devexit lp8720_remove(struct i2c_client *client)
{
	struct lp8720_platform_data *pdata;
	pdata = client->dev.platform_data;
	
	gpio_direction_output(pdata->en_gpio_num, 0);
	return 0;
}	

static const struct i2c_device_id lp8720_ids[] = {
	{ LP8720_I2C_NAME, 0 },	/*lp8720*/
	{ /* end of list */ },
};

static struct i2c_driver subpm_lp8720_driver = {
	.probe = lp8720_probe,
	.remove = __devexit_p(lp8720_remove),
	.id_table = lp8720_ids,
	.driver = {
		.name = LP8720_I2C_NAME,
		.owner = THIS_MODULE,
	},
};


static int __init subpm_lp8720_init(void)
{
	return i2c_add_driver(&subpm_lp8720_driver);
}

static void __exit subpm_lp8720_exit(void)
{
	i2c_del_driver(&subpm_lp8720_driver);
}

module_init(subpm_lp8720_init);
module_exit(subpm_lp8720_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("LP8720 sub pmic Driver");
MODULE_LICENSE("GPL");

