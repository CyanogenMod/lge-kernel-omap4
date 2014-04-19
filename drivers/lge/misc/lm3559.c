#include <linux/module.h>                                                                                            
#include <linux/i2c.h>                                                                                               
#include <linux/delay.h>                                                                                             
#include <linux/pm.h>                                                                                                
#include <mach/gpio.h>                                                                                               
#include <linux/lge/lm3559.h>                                                                                             
                                                                                                                     
#define setbits(data, masks)		data |=  masks                                                                
#define clrbits(data, masks)		data &= ~masks                                                                
                                                                                                                     
static struct i2c_client *this_client	=	0;                                                                    
static struct lm3559_platform_data*	pdata	=	0;                                                            
static unsigned char	flash_brightness	=	13;                                                           
                                                                                                                     
static int lm3559_enable(int status)                                                                                 
{                                                                                                                    
	gpio_set_value(pdata->gpio_hwen, (status != 0));                                                              
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_write(unsigned char index, unsigned char data)                                                     
{                                                                                                                    
	return i2c_smbus_write_byte_data(this_client,index,data);                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_read(unsigned char index)                                                                          
{                                                                                                                    
	return i2c_smbus_read_byte_data(this_client,index);                                                           
}                                                                                                                    
                                                                                                                     
static int lm3559_flash(unsigned char timeout)                                                                       
{                                                                                                                    
	unsigned char value;                                                                                          
	if (timeout > 0x1f)                                                                                           
		return	-1;                                                                                           
	                                                                                                              
	// Set flash brightness                                                                                       
	lm3559_write(FLASH_BRIGHTNESS_REG_INDEX, (flash_brightness << 4) | flash_brightness);                         
	                                                                                                              
	value	=	lm3559_read(FLASH_DURATION_REG_INDEX);                                                        
                                                                                                                     
	clrbits(value, 0x1F);                                                                                         
	setbits(value, timeout);                                                                                      
	lm3559_write(FLASH_DURATION_REG_INDEX, value);                                                                
	                                                                                                              
                                                                                                                     
	value	=	lm3559_read(ENABLE_REG_INDEX);                                                                
	setbits(value, 0x03);                                                                                         
	lm3559_write(ENABLE_REG_INDEX, value);                                                                        
	                                                                                                              
	return 0;                                                                                                     
}                                                                                                                    
                                                                                                                     
static void camera_flash(unsigned long timeout)                                                                      
{                                                                                                                    
	if (timeout > 1024)                                                                                           
		return	;                                                                                             
                                                                                                                     
	lm3559_flash(timeout/32);                                                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_torch(unsigned char brightness)                                                                    
{                                                                                                                    
	unsigned char value;                                                                                          
                                                                                                                     
	if (brightness > 0x1F)                                                                                        
		brightness	=	0x1F;                                                                         
                                                                                                                     
	lm3559_write(TORCH_BRIGHTNESS_REG_INDEX, brightness);//set torch brightness                                   
                                                                                                                     
	value = lm3559_read(ENABLE_REG_INDEX);                                                                        
	clrbits(value, 0x01);                                                                                         
	setbits(value, 0x02);                                                                                         
	lm3559_write(ENABLE_REG_INDEX,value);                                                                         
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
                                                                                                                     
static ssize_t flash_store(struct device* dev,                                                                       
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	camera_flash(simple_strtoul(buf, NULL, 10));                                                                  
	                                                                                                              
	return count;                                                                                                 
}                                                                                                                    
static DEVICE_ATTR(flash, 0666, NULL, flash_store);                                                                  
                                                                                                                     
static ssize_t flash_brightness_store(struct device* dev,                                                            
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	unsigned long	value	=	simple_strtoul(buf, NULL, 10);                                                
                                                                                                                     
	if ((0 <= value) && (value < 16))                                                                             
		flash_brightness	=	(unsigned char)value;                                                 
                                                                                                                     
	return	count;                                                                                                
}                                                                                                                    
                                                                                                                     
static ssize_t flash_brightness_show(struct device* dev,                                                             
							struct device_attribute* attr, char* buf) 
{                                                                                                                    
	return	snprintf(buf, PAGE_SIZE, "%d\n", flash_brightness);                                                   
}                                                                                                                    
static DEVICE_ATTR(flash_brightness, 0666, flash_brightness_show, flash_brightness_store);                           
                                                                                                                     
static ssize_t torch_store(struct device* dev,                                                                       
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	lm3559_torch(simple_strtoul(buf, NULL, 10));                                                                  
                                                                                                                     
	return	count;                                                                                                
}                                                                                                                    
static DEVICE_ATTR(torch, 0666, NULL, torch_store);                                                                  
                                                                                                                     
static ssize_t enable_store(struct device* dev,                                                                      
							 struct device_attribute* attr, const char* buf, size_t count)
{                                                                                                                    
	lm3559_enable(simple_strtoul(buf, NULL, 10));                                                                 
                                                                                                                     
	return	count;                                                                                                
}                                                                                                                    
                                                                                                                     
static ssize_t enable_show(struct device* dev,                                                                       
							 struct device_attribute* attr, char* buf)
{                                                                                                                    
	return	snprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->gpio_hwen));                                   
}                                                                                                                    
static DEVICE_ATTR(enable, 0666, enable_show, enable_store);                                                         
                                                                                                                     
static int lm3559_remove(struct i2c_client *client)                                                                  
{                                                                                                                    
	lm3559_enable(0);                                                                                             
                                                                                                                     
	gpio_free(pdata->gpio_hwen);                                                                                  
	                                                                                                              
	if (client != 0) {                                                                                            
		device_remove_file(&client->dev, &dev_attr_enable);                                                   
		device_remove_file(&client->dev, &dev_attr_flash);                                                    
		device_remove_file(&client->dev, &dev_attr_flash_brightness);                                         
		device_remove_file(&client->dev, &dev_attr_torch);                                                    
	}                                                                                                             
                                                                                                                     
	client	=	0;                                                                                            
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_probe(struct i2c_client *client, const struct i2c_device_id *devid)                                
{                                                                                                                    
	int res = 0;                                                                                                  
                                                                                                                     
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))                                                  
		return	-ENODEV;                                                                                      
                                                                                                                     
	this_client	=	client;                                                                               
                                                                                                                     
	pdata	=	client->dev.platform_data;                                                                    
	                                                                                                              
	gpio_request(pdata->gpio_hwen, "flash_en");                                                                   
	gpio_direction_output(pdata->gpio_hwen, 1);                                                                   
	                                                                                                              
	res = device_create_file(&client->dev, &dev_attr_flash);                                                      
	if(res){                                                                                                      
		printk("[lm3559:] device create file flash fail!\n");                                                 
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_flash_brightness);                                           
	if(res){                                                                                                      
		printk("[lm3559:] device create file flash_brightness fail!\n");                                      
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_torch);                                                      
	if(res){                                                                                                      
		printk("[lm3559:] device create file torch fail!\n");                                                 
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	res = device_create_file(&client->dev, &dev_attr_enable);                                                     
	if(res){                                                                                                      
		printk("[lm3559:] device create file enable fail!\n");                                                
		goto exit;                                                                                            
	}                                                                                                             
                                                                                                                     
	return res;                                                                                                   
                                                                                                                     
exit:	                                                                                                              
	lm3559_remove(this_client);                                                                                   
	this_client	=	0;                                                                                    
	pdata		=	0;                                                                                    
                                                                                                                     
	return	res;                                                                                                  
                                                                                                                     
}                                                                                                                    
                                                                                                                     
static int lm3559_suspend(struct i2c_client *client, pm_message_t message)                                           
{                                                                                                                    
	lm3559_enable(0);                                                                                             
                                                                                                                     
	return	0;                                                                                                    
}                                                                                                                    
                                                                                                                     
static int lm3559_resume(struct i2c_client *client)                                                                  
{                                                                                                                    
	lm3559_enable(1);                                                                                             
                                                                                                                     
	return 0;                                                                                                     
}                                                                                                                    
                                                                                                                     
static const struct i2c_device_id id_table_lm3559[] =                                                                
{                                                                                                                    
	{	LM3559_I2C_NAME,	0	},                                                                    
	{}                                                                                                            
};                                                                                                                   
                                                                                                                     
static struct i2c_driver i2c_driver_lm3559 =                                                                         
{                                                                                                                    
	.driver = {                                                                                                   
		.owner	=	THIS_MODULE,                                                                          
		.name	=	LM3559_I2C_NAME,                                                                      
	},                                                                                                            
	.probe		=	lm3559_probe,                                                                         
	.remove		=	__devexit_p(lm3559_remove),                                                           
	.suspend	=	lm3559_suspend,                                                                       
	.resume		=	lm3559_resume,                                                                        
	.id_table	=	id_table_lm3559,                                                                      
};                                                                                                                   
                                                                                                                     
static int __init lm3559_init(void)                                                                                  
{	                                                                                                              
	return i2c_add_driver(&i2c_driver_lm3559);	                                                              
}                                                                                                                    
                                                                                                                     
static void __exit lm3559_exit(void)                                                                                 
{                                                                                                                    
	i2c_del_driver(&i2c_driver_lm3559);                                                                           
}                                                                                                                    
                                                                                                                     
module_init(lm3559_init);                                                                                            
module_exit(lm3559_exit);                                                                                            
                                                                                                                     
MODULE_AUTHOR("chen.xuming@lge.com");                                                                                
MODULE_DESCRIPTION("lm3559 synchronous boost flash driver");                                                         
MODULE_LICENSE("GPL");                                                                                               
