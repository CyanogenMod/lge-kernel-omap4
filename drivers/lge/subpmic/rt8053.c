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
#include <linux/lge/rt8053.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define RT8063_POWER_ALL_OFF 0
#define RT8063_POWER_LEFT_ON 0x2b
#define RT8063_POWER_ALL_ON  0x3f

#define RT8063_VCM_ALL_OFF 0x2d
#define RT8063_VCM_LEFT_ON 0x2f
#define RT8063_VCM_ALL_ON  0x3f


static unsigned char rt8053_output_status = 0x00; //default on 0x3F
struct i2c_client *rt8053_client=NULL;

static unsigned char rt8053_status = 0;
static unsigned char rt8053_power_status = RT8063_POWER_ALL_OFF;
static unsigned char rt8053_vcm_status = RT8063_VCM_ALL_OFF;

static struct early_suspend rt8053_early_suspend;

void rt8053_early_suspend_func(struct early_suspend *h);
void rt8053_early_resume_func(struct early_suspend *h);


static void rt8053_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int err;
	
	struct i2c_msg	msg;
	u8 buf[2];
	
	msg.addr = (u16)client->addr;
	msg.flags =0;
	msg.len =2;

	buf[0]=reg;
	buf[1]=data;

	msg.buf = &buf[0];
	
	if ((err = i2c_transfer(client->adapter, &msg, 1)) < 0) {
		dev_err(&client->dev, "i2c write error\n");
		printk(KERN_ERR "rt8053_write_reg ERROR!!!\n");
	}
	
	return;
}

void subpm_set_output(subpm_output_enum outnum, int onoff)
{
    if(outnum > 5){
        dev_err(&rt8053_client->dev, "outnum error\n");
		return;
    }
	
    if(onoff == 0)
	    rt8053_output_status &= ~(1<<outnum);
    else
		rt8053_output_status |= (1<<outnum);
}

void subpm_output_enable(void)
{
    if(rt8053_client == NULL)
		return;
	
	rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_output_status);
}

EXPORT_SYMBOL(subpm_set_output);
EXPORT_SYMBOL(subpm_output_enable);



static ssize_t rt8053_show_power_on_off(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "0x%x 0x%x\n", rt8053_power_status, rt8053_status);
}


static ssize_t rt8063_store_power_on_off(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        int ret;
	int num = 0;
	struct rt8053_platform_data *pdata;

        pdata = client->dev.platform_data;
        ret = gpio_request(pdata->en_gpio_num, "rt8053");

        if (sscanf(buf, "%d", &num) != 1){
                ret = -1;
		goto err_sscanf;
        }

	switch(num){
		case 0:
			rt8053_power_status = RT8063_POWER_ALL_OFF;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
       			gpio_direction_output(pdata->en_gpio_num, 0);
        		mdelay(5);
			break;	
		case 1:
			gpio_direction_output(pdata->en_gpio_num, 1);
        		mdelay(5);
			rt8053_power_status = RT8063_POWER_LEFT_ON;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | 0);
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
			break;
		case 2:
			gpio_direction_output(pdata->en_gpio_num, 1);
        		mdelay(5);
			rt8053_power_status = RT8063_POWER_ALL_ON;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
			break;
		default:
			break;
	}

	return count;

err_sscanf:
	return ret;

}

static DEVICE_ATTR(poweronoff, 0666, 
     rt8053_show_power_on_off, rt8063_store_power_on_off);


static ssize_t rt8053_show_vcm(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "0x%x 0x%x\n", rt8053_vcm_status, rt8053_status);
}


static ssize_t rt8063_store_vcm(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        int ret;
	int num = 0;
	struct rt8053_platform_data *pdata;

        pdata = client->dev.platform_data;
        ret = gpio_request(pdata->en_gpio_num, "rt8053");

        if (sscanf(buf, "%d", &num) != 1){
                ret = -1;
		goto err_sscanf;
        }

	switch(num){
		case 0:
			rt8053_vcm_status = RT8063_VCM_ALL_OFF;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
			if(!rt8053_power_status){
       				gpio_direction_output(pdata->en_gpio_num, 0);
        			mdelay(5);
			}
			break;	
		case 1:
			gpio_direction_output(pdata->en_gpio_num, 1);
        		mdelay(5);
			rt8053_vcm_status = RT8063_VCM_LEFT_ON;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | 0);
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
			break;
		case 2:
			gpio_direction_output(pdata->en_gpio_num, 1);
        		mdelay(5);
			rt8053_vcm_status = RT8063_VCM_ALL_ON;
			rt8053_status = rt8053_power_status & rt8053_vcm_status;
			rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);
			break;
		default:
			break;
	}

	return count;

err_sscanf:
	return ret;

}

static DEVICE_ATTR(vcm, 0666, 
     rt8053_show_vcm, rt8063_store_vcm);


static void rt8053_init(struct i2c_client *client)
{
	int ret;
	struct rt8053_platform_data *pdata;

	pdata = client->dev.platform_data;
	ret = gpio_request(pdata->en_gpio_num, "rt8053");
	gpio_direction_output(pdata->en_gpio_num, 0);
	mdelay(5);
	rt8053_write_reg(client, RT8053_LDO1_SETTING, RT8053_STARTUP_DELAY_3TS | 0x17); //2.7v
	rt8053_write_reg(client, RT8053_LDO2_SETTING, RT8053_STARTUP_DELAY_3TS | 0x19); //2.8v
	rt8053_write_reg(client, RT8053_LDO3_SETTING, RT8053_STARTUP_DELAY_3TS | 0x17); //2.7v
	rt8053_write_reg(client, RT8053_LDO4_SETTING, RT8053_STARTUP_DELAY_3TS | 0x11); //1.8v
	rt8053_write_reg(client, RT8053_LDO5_SETTING, RT8053_STARTUP_DELAY_3TS | 0x19); //2.8v
	rt8053_write_reg(client, RT8053_BUCK_SETTING1, RT8053_STARTUP_DELAY_3TS | 0x09);
	rt8053_write_reg(client, RT8053_BUCK_SETTING2, 0x09); //1.2v
	rt8053_write_reg(client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_output_status);
	mdelay(5);
	gpio_direction_output(pdata->en_gpio_num, 1);

	return;
}

static int __init rt8053_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rt8053_platform_data *pdata;
	int err = 0;
	int ret;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	rt8053_client = client;
	rt8053_init(client);

        /* Register sysfs hooks */
       /* err = sysfs_create_group(&client->dev.kobj, &rt8053_attr_group);*/
	err = device_create_file(&client->dev, &dev_attr_poweronoff);
        if (err)
                goto exit_sysfs;
	err = device_create_file(&client->dev, &dev_attr_vcm);
        if (err)
                goto exit_sysfs;

#ifdef CONFIG_HAS_EARLYSUSPEND
        rt8053_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        rt8053_early_suspend.suspend = rt8053_early_suspend_func;
        rt8053_early_suspend.resume = rt8053_early_resume_func;
        register_early_suspend(&rt8053_early_suspend);
#endif

	pdata = client->dev.platform_data;
	ret = gpio_request(pdata->en_gpio_num, "rt8053");

	return 0;

exit_sysfs:
	return err;

}

static int rt8053_remove(struct i2c_client *client)
{
	struct rt8053_platform_data *pdata;
	pdata = client->dev.platform_data;
	
    	gpio_direction_output(pdata->en_gpio_num, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
        unregister_early_suspend(&rt8053_early_suspend);
#endif
	return 0;
}	

#ifdef CONFIG_PM
static int rt8053_suspend(struct i2c_client *client, pm_message_t mesg)
{
 	struct rt8053_platform_data*	pdata	=	client->dev.platform_data;
	gpio_direction_output(pdata->en_gpio_num, 0);

	printk("enter suspend!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	return 0;
}


static int rt8053_resume(struct i2c_client *client)
{
	struct rt8053_platform_data*	pdata = client->dev.platform_data;

	gpio_direction_output(pdata->en_gpio_num, 1);
    mdelay(5);

	/*resume status before suspend*/
    rt8053_write_reg(rt8053_client, RT8053_OUTPUT_ENABLE, 0x80 | rt8053_status);

	printk("enter resume!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	return 0;
}
#else

#define rt8053_suspend        NULL
#define rt8053_resume         NULL

#endif /* CONFIG_PM */


#ifdef CONFIG_HAS_EARLYSUSPEND
void rt8053_early_suspend_func(struct early_suspend *h)
{
	pm_message_t pt;
	printk("enter early_suspend!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	rt8053_suspend(rt8053_client, pt);
}


void rt8053_early_resume_func(struct early_suspend *h)
{
	printk("enter early_resume!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	rt8053_resume(rt8053_client);
}
#endif


static const struct i2c_device_id rt8053_ids[] = {
	{ RT8053_I2C_NAME, 0 },	/*rt8053*/
	{ /* end of list */ },
};

static struct i2c_driver subpm_rt8053_driver = {
	.probe = rt8053_probe,
	.remove = __devexit_p(rt8053_remove),
	.suspend = rt8053_suspend,
        .resume = rt8053_resume,
	.id_table	= rt8053_ids,
	.driver = {
		.name = RT8053_I2C_NAME,
		.owner = THIS_MODULE,
    },
};


static int __init subpm_rt8053_init(void)
{
	return i2c_add_driver(&subpm_rt8053_driver);
}

static void __exit subpm_rt8053_exit(void)
{
	i2c_del_driver(&subpm_rt8053_driver);
}

module_init(subpm_rt8053_init);
module_exit(subpm_rt8053_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("RT8053 sub pmic Driver");
MODULE_LICENSE("GPL");
