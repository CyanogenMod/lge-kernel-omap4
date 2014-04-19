#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/lge/lm3530.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define LGE_TEMPERATURE_ADAPTED_BACKLIGHT

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
#include <linux/syscalls.h>	/* open, close */
#include <linux/workqueue.h>
#include <linux/i2c/twl6030-gpadc.h>
static struct workqueue_struct *thermal_wq;
static struct delayed_work thermal_wk;
static struct lm3530_platform_data*	pdata_t;
#endif

#define LM3530_DEBUG 0
 #if LM3530_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif

static int	old_brightness	=	-1;
static int  reg_adr = LM3530_REG_BRT;
atomic_t  late_resume_flag;

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
#define PCB_THM_ADC_CHANNEL 4
#define HOT_THRESHOLD_VALUE 45000
#define VERY_HOT_THRESHOLD_VALUE 50000
#define PANIC_HOT_THRESHOLD_VALUE 55000

typedef enum  {
PCB_THM_NORMAL,
PCB_THM_HOT,
PCB_THM_VERY_HOT,
PCB_THM_PANIC_HOT
} pcb_thm_stage;

static pcb_thm_stage curr_stage=PCB_THM_NORMAL,prev_stage=PCB_THM_NORMAL;
static bool hot_stage_enable = false;

#define ADC_START_VALUE 126
#define ADC_END_VALUE   978
/*
 * Temperature values in milli degrees celsius ADC code values from 978 to 126
 */
int adc_to_temp[] = {
	-40000, -40000, -40000, -40000, -40000, -37000, -35000, -33000, -31000,
	-29000, -28000, -27000, -25000, -24000, -23000, -22000, -21000, -20000,
	-19000, -18000, -17000, -17000, -16000, -15000, -14000, -14000, -13000,
	-12000, -12000, -11000, -11000, -10000, -10000, -9000, -8000, -8000,
	-7000, -7000, -6000, -6000, -6000, -5000, -5000, -4000, -4000, -3000,
	-3000, -3000, -2000, -2000, -1000, -1000, -1000, 0, 0, 0, 1000, 1000,
	1000, 2000, 2000, 2000, 3000, 3000, 3000, 4000, 4000, 4000, 5000, 5000,
	5000, 6000, 6000, 6000, 6000, 7000, 7000, 7000, 8000, 8000, 8000, 8000,
	9000, 9000, 9000, 9000, 10000, 10000, 10000, 10000, 11000, 11000,
	11000, 11000, 12000, 12000, 12000, 12000, 12000, 13000, 13000, 13000,
	13000, 14000, 14000, 14000, 14000, 14000, 15000, 15000, 15000, 15000,
	15000, 16000, 16000, 16000, 16000, 16000, 17000, 17000, 17000, 17000,
	17000, 18000, 18000, 18000, 18000, 18000, 19000, 19000, 19000, 19000,
	19000, 20000, 20000, 20000, 20000, 20000, 20000, 21000, 21000, 21000,
	21000, 21000, 22000, 22000, 22000, 22000, 22000, 22000, 23000, 23000,
	23000, 23000, 23000, 23000, 24000, 24000, 24000, 24000, 24000, 24000,
	25000, 25000, 25000, 25000, 25000, 25000, 25000, 26000, 26000, 26000,
	26000, 26000, 26000, 27000, 27000, 27000, 27000, 27000, 27000, 27000,
	28000, 28000, 28000, 28000, 28000, 28000, 29000, 29000, 29000, 29000,
	29000, 29000, 29000, 30000, 30000, 30000, 30000, 30000, 30000, 30000,
	31000, 31000, 31000, 31000, 31000, 31000, 31000, 32000, 32000, 32000,
	32000, 32000, 32000, 32000, 33000, 33000, 33000, 33000, 33000, 33000,
	33000, 33000, 34000, 34000, 34000, 34000, 34000, 34000, 34000, 35000,
	35000, 35000, 35000, 35000, 35000, 35000, 35000, 36000, 36000, 36000,
	36000, 36000, 36000, 36000, 36000, 37000, 37000, 37000, 37000, 37000,
	37000, 37000, 38000, 38000, 38000, 38000, 38000, 38000, 38000, 38000,
	39000, 39000, 39000, 39000, 39000, 39000, 39000, 39000, 39000, 40000,
	40000, 40000, 40000, 40000, 40000, 40000, 40000, 41000, 41000, 41000,
	41000, 41000, 41000, 41000, 41000, 42000, 42000, 42000, 42000, 42000,
	42000, 42000, 42000, 43000, 43000, 43000, 43000, 43000, 43000, 43000,
	43000, 43000, 44000, 44000, 44000, 44000, 44000, 44000, 44000, 44000,
	45000, 45000, 45000, 45000, 45000, 45000, 45000, 45000, 45000, 46000,
	46000, 46000, 46000, 46000, 46000, 46000, 46000, 47000, 47000, 47000,
	47000, 47000, 47000, 47000, 47000, 47000, 48000, 48000, 48000, 48000,
	48000, 48000, 48000, 48000, 48000, 49000, 49000, 49000, 49000, 49000,
	49000, 49000, 49000, 49000, 50000, 50000, 50000, 50000, 50000, 50000,
	50000, 50000, 51000, 51000, 51000, 51000, 51000, 51000, 51000, 51000,
	51000, 52000, 52000, 52000, 52000, 52000, 52000, 52000, 52000, 52000,
	53000, 53000, 53000, 53000, 53000, 53000, 53000, 53000, 53000, 54000,
	54000, 54000, 54000, 54000, 54000, 54000, 54000, 54000, 55000, 55000,
	55000, 55000, 55000, 55000, 55000, 55000, 55000, 56000, 56000, 56000,
	56000, 56000, 56000, 56000, 56000, 56000, 57000, 57000, 57000, 57000,
	57000, 57000, 57000, 57000, 57000, 58000, 58000, 58000, 58000, 58000,
	58000, 58000, 58000, 58000, 59000, 59000, 59000, 59000, 59000, 59000,
	59000, 59000, 59000, 60000, 60000, 60000, 60000, 60000, 60000, 60000,
	60000, 61000, 61000, 61000, 61000, 61000, 61000, 61000, 61000, 61000,
	62000, 62000, 62000, 62000, 62000, 62000, 62000, 62000, 62000, 63000,
	63000, 63000, 63000, 63000, 63000, 63000, 63000, 63000, 64000, 64000,
	64000, 64000, 64000, 64000, 64000, 64000, 64000, 65000, 65000, 65000,
	65000, 65000, 65000, 65000, 65000, 66000, 66000, 66000, 66000, 66000,
	66000, 66000, 66000, 66000, 67000, 67000, 67000, 67000, 67000, 67000,
	67000, 67000, 68000, 68000, 68000, 68000, 68000, 68000, 68000, 68000,
	68000, 69000, 69000, 69000, 69000, 69000, 69000, 69000, 69000, 70000,
	70000, 70000, 70000, 70000, 70000, 70000, 70000, 70000, 71000, 71000,
	71000, 71000, 71000, 71000, 71000, 71000, 72000, 72000, 72000, 72000,
	72000, 72000, 72000, 72000, 73000, 73000, 73000, 73000, 73000, 73000,
	73000, 73000, 74000, 74000, 74000, 74000, 74000, 74000, 74000, 74000,
	75000, 75000, 75000, 75000, 75000, 75000, 75000, 75000, 76000, 76000,
	76000, 76000, 76000, 76000, 76000, 76000, 77000, 77000, 77000, 77000,
	77000, 77000, 77000, 77000, 78000, 78000, 78000, 78000, 78000, 78000,
	78000, 79000, 79000, 79000, 79000, 79000, 79000, 79000, 79000, 80000,
	80000, 80000, 80000, 80000, 80000, 80000, 81000, 81000, 81000, 81000,
	81000, 81000, 81000, 82000, 82000, 82000, 82000, 82000, 82000, 82000,
	82000, 83000, 83000, 83000, 83000, 83000, 83000, 83000, 84000, 84000,
	84000, 84000, 84000, 84000, 84000, 85000, 85000, 85000, 85000, 85000,
	85000, 85000, 86000, 86000, 86000, 86000, 86000, 86000, 87000, 87000,
	87000, 87000, 87000, 87000, 87000, 88000, 88000, 88000, 88000, 88000,
	88000, 88000, 89000, 89000, 89000, 89000, 89000, 89000, 90000, 90000,
	90000, 90000, 90000, 90000, 91000, 91000, 91000, 91000, 91000, 91000,
	91000, 92000, 92000, 92000, 92000, 92000, 92000, 93000, 93000, 93000,
	93000, 93000, 93000, 94000, 94000, 94000, 94000, 94000, 94000, 95000,
	95000, 95000, 95000, 95000, 96000, 96000, 96000, 96000, 96000, 96000,
	97000, 97000, 97000, 97000, 97000, 97000, 98000, 98000, 98000, 98000,
	98000, 99000, 99000, 99000, 99000, 99000, 100000, 100000, 100000,
	100000, 100000, 100000, 101000, 101000, 101000, 101000, 101000, 102000,
	102000, 102000, 102000, 102000, 103000, 103000, 103000, 103000, 103000,
	104000, 104000, 104000, 104000, 104000, 105000, 105000, 105000, 105000,
	106000, 106000, 106000, 106000, 106000, 107000, 107000, 107000, 107000,
	108000, 108000, 108000, 108000, 108000, 109000, 109000, 109000, 109000,
	110000, 110000, 110000, 110000, 111000, 111000, 111000, 111000, 112000,
	112000, 112000, 112000, 112000, 113000, 113000, 113000, 114000, 114000,
	114000, 114000, 115000, 115000, 115000, 115000, 116000, 116000, 116000,
	116000, 117000, 117000, 117000, 118000, 118000, 118000, 118000, 119000,
	119000, 119000, 120000, 120000, 120000, 120000, 121000, 121000, 121000,
	122000, 122000, 122000, 123000, 123000, 123000, 124000, 124000, 124000,
	124000, 125000, 125000, 125000, 125000, 125000, 125000, 125000, 125000,
	125000, 125000, 125000, 125000,
};

int adc_to_temp_conversion(int adc_val)
{
	if ((adc_val < ADC_START_VALUE) ||
		(adc_val > ADC_END_VALUE)) {
		pr_err("%s:Temp read is invalid %i\n", __func__, adc_val);
		return -EINVAL;
	}

	return adc_to_temp[ADC_END_VALUE - adc_val];
}

static int write_intToFile(const char *path, int i)
{
	char buf[1024];
	int fd = sys_open(path, O_WRONLY | O_NONBLOCK,0);
	size_t count;

	if (fd == -1) {
		return -1;
	}

	sprintf(buf, "%d", i);
	count = sys_write(fd, buf, strlen(buf));


	sys_close(fd);
	return count;
}

int get_pcb_therm()
{
	struct twl6030_gpadc_request req;
	int ret;

	req.channels = (1 << PCB_THM_ADC_CHANNEL);
	req.method = TWL6030_GPADC_SW2;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);

	return adc_to_temp_conversion(req.buf[PCB_THM_ADC_CHANNEL].code);
}

int reduce_brightness_by_stage()
{
	int brightness;

	if (old_brightness == 0)
		return old_brightness;

	switch (curr_stage)
	{
		case PCB_THM_NORMAL :
			brightness = old_brightness;
			break;

		case PCB_THM_HOT :
			brightness = old_brightness - 3; // 1%
			break;

		case PCB_THM_VERY_HOT :
			brightness = old_brightness - 6; // 2%
			break;

		case PCB_THM_PANIC_HOT :
			brightness = old_brightness - 9; // 3%
			break;

		default:
			brightness = old_brightness;
			break;
	}

	if (brightness < 30)	// MIN brightness to be off
		brightness	=	30;

	return brightness;
}


static void tab_work_func(struct work_struct *work)
{
	int current_pcb_thm;
	int	brightness = old_brightness;

	current_pcb_thm = get_pcb_therm();

	if (current_pcb_thm >= PANIC_HOT_THRESHOLD_VALUE)
		curr_stage = PCB_THM_PANIC_HOT;
	else if (current_pcb_thm >= VERY_HOT_THRESHOLD_VALUE)
		curr_stage = PCB_THM_VERY_HOT;
	else if (current_pcb_thm >= HOT_THRESHOLD_VALUE)
		curr_stage = PCB_THM_HOT;
	else
		curr_stage = PCB_THM_NORMAL;


	if (curr_stage !=  PCB_THM_NORMAL)
		hot_stage_enable = true;
	else
		hot_stage_enable = false;

	//printk("[TAB]thermal backlight work!!! pcb thm:%d\n",current_pcb_thm);

	if (prev_stage != curr_stage)
	{
		brightness = reduce_brightness_by_stage();

		printk("[TAB]thermal backlight work!!! pcb thm:%d stage:%d brightness:%d\n",current_pcb_thm,curr_stage,brightness);
		lm3530_set_brightness_control(pdata_t, brightness);
		prev_stage = curr_stage;
	}

	queue_delayed_work(thermal_wq, &thermal_wk, 10*HZ);
}

#endif

void lm3530bl_shutdown(struct i2c_client* client)
{
	struct lm3530_platform_data*	pdata;

	pdata	=	client->dev.platform_data;
#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	cancel_delayed_work(&thermal_wk);
#endif

	lm3530_set_hwen(&pdata->private, pdata->gpio_hwen, 0);
}


/* SYSFS for brightness control */
static ssize_t	brightness_show(struct device* dev,
		struct device_attribute* attr, char* buf)
{
	struct	lm3530_platform_data*	pdata	=	dev->platform_data;
	int	val;
#if 0
	if ((val = lm3530_get_brightness_control(&pdata->private)) < 0)
		return	0;
#endif

	return	snprintf(buf, PAGE_SIZE, "%d\n", old_brightness);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static int bl_common_flag = 0;

static void  lm3530_early_suspend(struct early_suspend *h)
{
	struct	lm3530_platform_data*	pdata = container_of(h, struct lm3530_platform_data, early_suspend);
	printk("[BL]%s\n",__func__);
#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	cancel_delayed_work(&thermal_wk);
#endif
	pdata->private.reg_brr = (pdata->private.reg_brr & 0x07) | 0x8;
	bl_common_flag = 0;

	atomic_set(&late_resume_flag, 0);

	printk("[dyotest] %s, set brightness: 0, :flag:%d", __func__, atomic_read(&late_resume_flag));

	lm3530_set_brightness_control(pdata, 0);


	return;
}

static void  lm3530_late_resume(struct early_suspend *h)
{
	struct	lm3530_platform_data*	pdata = container_of(h, struct lm3530_platform_data, early_suspend);
	int brightness;
	printk("[BL]%s, old_brightness=%d, \n",__func__,old_brightness);
	pdata->private.reg_brr = 0x00;

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	if (hot_stage_enable == true)
		brightness= reduce_brightness_by_stage();
#endif
	printk("[dyotest] late_resume [BL]%s, old_brightness=%d, \n",__func__,old_brightness);

	atomic_set(&late_resume_flag, 1);
	brightness = old_brightness;

	if(brightness)
	{
		printk("[dyotest] %s, set brightness:%d, :flag:%d\n", __func__, brightness, atomic_read(&late_resume_flag));
		lm3530_set_brightness_control(pdata, brightness);
	}

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	queue_delayed_work(thermal_wq, &thermal_wk, 10*HZ);
#endif
	return;
}
#endif
static ssize_t	brightness_store(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct	lm3530_platform_data*	pdata	=	dev->platform_data;
	int	brightness	=	simple_strtol(buf, NULL, 10);

	printk("[dyotest] brightness_store = [%d] \n",brightness);
/* bl_common_flag = 1 -> write fade in/out register for backlight effect*/
/* Not writing this register first entering is why the target dosen't adapt backlight effect in first backlight on from suspend*/
	if(bl_common_flag < 1)
	{
		bl_common_flag++;
	}
	else if(pdata->private.reg_brr == 0)
	{
		pdata->private.reg_brr = (pdata->private.reg_brr & 0x7) | 0x8;
		lm3530_brr_write(&pdata->private);
	}

	if (brightness > 0 && brightness < 30)	// MIN brightness to be off
		brightness	=	30;

	if ((brightness < 0) || (brightness > 255)) // Invalid brightness
		goto	exit;

	if (old_brightness == brightness) // No need to change the brightness
		goto	exit;


	old_brightness	=	brightness;

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	if (hot_stage_enable == true)
		brightness= reduce_brightness_by_stage();
#endif

	if(!brightness)
	{
		printk("[dyotest] %s, set brightness:%d, :flag:%d", __func__, brightness, atomic_read(&late_resume_flag));
		lm3530_set_brightness_control(pdata, brightness);
	}

	if(atomic_read(&late_resume_flag))
	{
		printk("[dyotest] %s, set brightness:%d, :flag:%d", __func__, brightness, atomic_read(&late_resume_flag));
		lm3530_set_brightness_control(pdata, brightness);
	}


exit:
	return	count;
}

/* SYSFS for brightness control */

static ssize_t  reg_read_show(struct device* dev,
		struct device_attribute* attr, char* buf)
{
	struct  lm3530_platform_data*   pdata   =   dev->platform_data;
	int reg_val;

	reg_val = lm3530_read_byte(&pdata->private, reg_adr);

	printk("%s, reg_adr=%x, reg_val=%x", __func__, reg_adr, reg_val);

	return  snprintf(buf, PAGE_SIZE, "reg_adr: %x, reg_val: %x\n", reg_adr, reg_val);
}

static ssize_t  reg_read_store(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct  lm3530_platform_data*   pdata   =   dev->platform_data;
	int reg_val;

	sscanf(buf, "%x", &reg_adr);

	reg_val = lm3530_read_byte(&pdata->private, reg_adr);

	printk("%s, reg_adr=%x, reg_val=%x", __func__, reg_adr, reg_val);

	return count;
}

static ssize_t  reg_write_store(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct  lm3530_platform_data*   pdata   =   dev->platform_data;
	int reg_val;

	sscanf(buf, "%x,%x", &reg_adr, &reg_val);

	lm3530_write_byte(&pdata->private, reg_adr, reg_val);

	printk("%s, reg_adr=%x, reg_val=%x", __func__, reg_adr, reg_val);

	return count;
}

static DEVICE_ATTR(brightness, 0660, brightness_show, brightness_store);
static DEVICE_ATTR(reg_read, 0660, reg_read_show, reg_read_store);
static DEVICE_ATTR(reg_write, 0220, NULL , reg_write_store);

/* SYSFS for LCD backlight ON/OFF
 */
static ssize_t	enable_show(struct device* dev,
		struct device_attribute* attr, char *buf)
{
	struct	lm3530_platform_data*	pdata	=	dev->platform_data;
	int	val	=	lm3530_get_hwen(&pdata->private, pdata->gpio_hwen);

	return	snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t	enable_store(struct device* dev,
		struct device_attribute* attr, const char* buf, size_t count)
{
	struct	lm3530_platform_data*	pdata	=	dev->platform_data;

	DEBUG_MSG("enable_store = [%d] \n",(int)simple_strtol(buf, NULL, 10));

	lm3530_set_hwen(&pdata->private, pdata->gpio_hwen, (int)simple_strtol(buf, NULL, 10));

	return	count;
}

static DEVICE_ATTR(enable, 0664, enable_show, enable_store);

/* Driver
 */
static int __devinit lm3530bl_probe(struct i2c_client* client,
							const struct i2c_device_id* id)
{
	struct lm3530_platform_data*	pdata;
	int		ret = 0;

	atomic_set(&late_resume_flag, 1);

	pdata	=	client->dev.platform_data;
	gpio_request(pdata->gpio_hwen, "backlight_enable");
	gpio_direction_output(pdata->gpio_hwen, 1);	// OUTPUT
#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	pdata_t = client->dev.platform_data;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	pdata->early_suspend.suspend = lm3530_early_suspend;
	pdata->early_suspend.resume = lm3530_late_resume;
	pdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 40;
	register_early_suspend(&pdata->early_suspend);
#endif
	lm3530_init(&pdata->private, client);

	ret = device_create_file(&client->dev, &dev_attr_brightness);
	ret = device_create_file(&client->dev, &dev_attr_enable);
	ret = device_create_file(&client->dev, &dev_attr_reg_write);
	ret = device_create_file(&client->dev, &dev_attr_reg_read);

	old_brightness	=	lm3530_get_brightness_control(&pdata->private);

#ifdef LGE_TEMPERATURE_ADAPTED_BACKLIGHT
	thermal_wq = create_workqueue("tab_workqueue");
	if (thermal_wq != NULL)
	{
		INIT_DELAYED_WORK_DEFERRABLE(&thermal_wk,tab_work_func);
		queue_delayed_work(thermal_wq,&thermal_wk, 240*HZ);
		printk("[TAB] init TAB work queue! first check time is 240sec after boot\n");
	}
#endif

	return	ret;
}

static int __devexit lm3530bl_remove(struct i2c_client* client)
{
	device_remove_file(&client->dev, &dev_attr_brightness);
	device_remove_file(&client->dev, &dev_attr_enable);
	device_remove_file(&client->dev, &dev_attr_reg_write);
	device_remove_file(&client->dev, &dev_attr_reg_read);
	return	0;
}

static const struct i2c_device_id lm3530bl_ids[] = {
	{	LM3530_I2C_NAME, 0 },	// LM3530
	{},
};

static struct i2c_driver lm3530bl_driver = {
	.probe		= lm3530bl_probe,
	.remove		= __devexit_p(lm3530bl_remove),
	.id_table	= lm3530bl_ids,
	.shutdown	= lm3530bl_shutdown,
	.driver = {
		.name	= LM3530_I2C_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lm3530bl_init(void)
{
	return	i2c_add_driver(&lm3530bl_driver);
}

static void __exit lm3530bl_exit(void)
{
	i2c_del_driver(&lm3530bl_driver);
}

module_init(lm3530bl_init);
module_exit(lm3530bl_exit);

MODULE_AUTHOR("LG Electronics");
MODULE_DESCRIPTION("Backlight driver (LM3530)");
MODULE_LICENSE("GPL");
