#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <linux/mutex.h>
#include <linux/lge/aat2862.h>

#define MAX_BRIGHTNESS 		0x1f	// 0001 1111 =   0.48mA
#define DEFAULT_BRIGHTNESS 	0x08	// 0000 1000 = 22.26mA
#define LCD_CP_EN             98

static int aat2862_show_Brightness(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct aat2862data *data = i2c_get_clientdata(client);

	printk(KERN_INFO "Brightness Range : 0 ~ 255\n");
	return sprintf(buf,"Brightness : %d\n",data->current_brightness);	
}
static int aat2862_store_Brightness(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client*client = to_i2c_client(dev);
	struct aat2862data *data = i2c_get_clientdata(client);
	int val, for_main, for_sub;

	val = simple_strtoul(buf, NULL, 10);
	if(val > 255)    val = 255;
	else if(val < 0) val = 0;
	data->current_brightness = val; 

	if(val == 0){   //TURN OFF BACKLIGHT//
		for_main = 0x00;
		for_sub  = 0x00;
	}
	else{    //SET THE BACKLIGHT CURRENT//
	val = 31 - (val / 8) ; //AAT_LED Current : 32states
	for_main = 0xE0 | val;
	for_sub  = 0x60 | val;
	}
	aat2862_write_byte(client,MAIN_CURRENT_CONTROL_REG,for_main);
	aat2862_write_byte(client, SUB_CURRENT_CONTROL_REG, for_sub);
	aat2862_write_byte(client,AUX1_CURRENT_CONTROL_REG, for_sub);
	aat2862_write_byte(client,AUX2_CURRENT_CONTROL_REG, for_sub);
	return count;
	
}
static DEVICE_ATTR(brightness, 0777, aat2862_show_Brightness, aat2862_store_Brightness );

static int __init aat2862_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aat2862data *data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct aat2862_platform_data *pdata;
	int err = 0;
	
	printk("##AAT2862_probe_START##\n");
	if(!i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE)){
		err = -EIO;
		goto exit;
	}
	
	data = kzalloc(sizeof(struct aat2862data),GFP_KERNEL);
	if(!data){
		err = -ENOMEM;
		goto exit;
	}
	
	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	
	pdata = data->client->dev.platform_data;
	
	gpio_request(pdata->gpio_hwen, "aat2862");
	gpio_direction_output(pdata->gpio_hwen, 1); // OUTPUT 
	gpio_set_value(pdata->gpio_hwen, 1);	    // ON 
	
	data->current_brightness = DEFAULT_BRIGHTNESS*8;
	
	if( err = aat2862_init_client(client) ) 
		goto exit_kfree;
	
	err = device_create_file(&client->dev, &dev_attr_brightness);
	if(err)
		goto exit_kfree;
	
	printk("##AAT2862_ALL_SUCCESS##\n");
	return 0;

	exit_kfree:
		kfree(data);
	exit:
		return err;
}

static int aat2862_remove(struct i2c_client *client)
{
	
	gpio_free(LCD_CP_EN);
 	device_remove_file(&client->dev, &dev_attr_brightness);
	kfree(i2c_get_clientdata(client));

	return 0;
}	

static const struct i2c_device_id aat2862_bl_id[] = {
	{ AAT_I2C_BL_NAME, 0 },
	{ }
};
static struct i2c_driver main_aat2862_driver = {
	.probe = aat2862_probe,
	.remove = aat2862_remove,
	.id_table = aat2862_bl_id, 
	.driver = {
		.name = AAT_I2C_BL_NAME,
	},
};

static int __init aat2862_init(void)
{
	return i2c_add_driver(&main_aat2862_driver);
}
static void __exit aat2862_exit(void)
{
	return i2c_del_driver(&main_aat2862_driver);
}
module_init(aat2862_init);
module_exit(aat2862_exit);

MODULE_DESCRIPTION("AAT2862 Backlight Control");
MODULE_AUTHOR("Jaekyung Oh <jaekyung.oh@lge.com>");
MODULE_LICENSE("GPL");
