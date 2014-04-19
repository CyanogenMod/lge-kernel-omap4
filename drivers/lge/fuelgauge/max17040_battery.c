/*
 *  max17040_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/max17040_battery.h>
#include <linux/slab.h>
#include <mach/gpio.h>


#define MAX17040_VCELL_MSB	0x02
#define MAX17040_VCELL_LSB	0x03
#define MAX17040_SOC_MSB	0x04
#define MAX17040_SOC_LSB	0x05
#define MAX17040_MODE_MSB	0x06
#define MAX17040_MODE_LSB	0x07
#define MAX17040_VER_MSB	0x08
#define MAX17040_VER_LSB	0x09
#define MAX17040_CFG_RCOMP_MSB	0x0C
#define MAX17040_CFG_RCOMP_LSB	0x0D
#define MAX17040_CMD_MSB	0xFE
#define MAX17040_CMD_LSB	0xFF

#define MAX17040_DELAY		1000
#define MAX17040_BATTERY_FULL	95

struct max17040_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct delayed_work		alert_work;	
	struct power_supply		battery;
	struct max17040_platform_data	*pdata;

	/* State Of Connect */
	int online;
	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;

	int config;
} ;
struct max17040_chip *p_max17043_chip = NULL;

extern int max8971_start_charging(unsigned mA);

static int max17040_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17040_chip *chip = container_of(psy,
				struct max17040_chip, battery);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int max17040_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	//if (ret < 0)
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int max17040_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	//if (ret < 0)
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void max17040_reset(struct i2c_client *client)
{
	max17040_write_reg(client, MAX17040_CMD_MSB, 0x54);
	max17040_write_reg(client, MAX17040_CMD_LSB, 0x00);
}

static void max17040_get_vcell(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VCELL_MSB);
	lsb = max17040_read_reg(client, MAX17040_VCELL_LSB);

	chip->vcell = (msb << 4) + (lsb >> 4);
	
	if(msb < 0 || lsb < 0){
		//printk("[Gauge]: max17040_get_vcell failed!!!\n");
		chip->vcell = 0;
	}
}

static void max17040_get_soc(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_SOC_MSB);
	lsb = max17040_read_reg(client, MAX17040_SOC_LSB);

	chip->soc = msb;

	if(msb > 100)
		chip->soc = 100;
	if(msb < 0 ){
		//printk("[Gauge]: max17040_get_soc failed!!!\n");
		chip->soc = 0;
	}	
}

static int max17040_read_config(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_CFG_RCOMP_MSB);
	lsb = max17040_read_reg(client, MAX17040_CFG_RCOMP_LSB);

	if(msb < 0 || lsb < 0)
		return -1;

	chip->config = msb << 8 | lsb;

	return 0;
}
static int max17040_write_config(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	max17040_write_reg(client, MAX17040_CFG_RCOMP_MSB, (chip->config & 0xFF00)>>8);
	max17040_write_reg(client, MAX17040_CFG_RCOMP_LSB, chip->config & 0xFF);

	return 0;
}

static int max17043_set_athd(int level)
{
	if(p_max17043_chip ==NULL){
		return -1;
	}

	if(level > 32)
		level = 32;
	else if(level < 1)
		level = 1;

	//printk("level:%d(%)=>32-level:%d(0x%x)",level, 32-level, 32-level);
	level = 32 - level;
	if(level == (p_max17043_chip->config & 0x1F))
		return level;

	p_max17043_chip->config = ((p_max17043_chip->config & 0xffe0) | level);
	max17040_write_config(p_max17043_chip->client);

	return level;
}

static int max17043_set_sleep(bool sleep)
{
	if(p_max17043_chip ==NULL){
		return -1;
	}

	if(sleep == (p_max17043_chip->config & 0x80))
		return 0;

	p_max17043_chip->config = ((p_max17043_chip->config & 0xff7f) | sleep);
	max17040_write_config(p_max17043_chip->client);

	return 0;
}

/*get Battery Voltage through external Fuel Gauge*/
int get_bat_vcell(void)
{
	if(p_max17043_chip ==NULL){
		printk("p_max17043_chip should be inited!\n");
		//msleep(500);
		return 3000;// if Fgauge max17043 not been inited yet, and TWL6030 wanna get vcell or soc, just return a fake value temply.
	}

	max17040_get_vcell(p_max17043_chip->client);
	return p_max17043_chip->vcell;
}

EXPORT_SYMBOL(get_bat_vcell);

/*get Battery SOC through external Fuel Gauge*/
int get_bat_soc(void)
{
	if(p_max17043_chip ==NULL){
		//msleep(500);
		printk("p_max17043_chip should be inited!\n");
		return 30;// if Fgauge max17043 not been inited yet, and TWL6030 wanna get vcell or soc, just return a fake value temply.
	}
	max17040_get_soc(p_max17043_chip->client);
	return p_max17043_chip->soc;
}

EXPORT_SYMBOL(get_bat_soc);

static void max17040_get_version(struct i2c_client *client)
{
	u8 msb;
	u8 lsb;

	msb = max17040_read_reg(client, MAX17040_VER_MSB);
	lsb = max17040_read_reg(client, MAX17040_VER_LSB);

	dev_info(&client->dev, "MAX17040 Fuel-Gauge Ver %d%d\n", msb, lsb);
}

static void max17040_get_online(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void max17040_get_status(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if (!chip->pdata->charger_online || !chip->pdata->charger_enable) {
		chip->status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	if (chip->pdata->charger_online()) {
		if (chip->pdata->charger_enable())
			chip->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			chip->status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (chip->soc > MAX17040_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

static void max17040_work(struct work_struct *work)
{
	struct max17040_chip *chip;
static int cnt_max = 0;
	chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_vcell(chip->client);
	max17040_get_soc(chip->client);
	max17040_get_online(chip->client);
	max17040_get_status(chip->client);
//printk("max17043: (cnt_maxin %d) chip->vcell %d, chip->soc %d%\n", cnt_max++, chip->vcell, chip->soc);
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
}

static enum power_supply_property max17040_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int max17043_update(struct max17040_chip *chip)
{
	//struct max17040_chip *chip;
	//chip = container_of(work, struct max17040_chip, work.work);

	max17040_get_soc(chip->client);

	max17040_get_vcell(chip->client);

	if(chip->soc > 100)
		chip->soc = 100;
	else if(chip->soc < 0)
		chip->soc = 0;

	return 0;
}

static irqreturn_t max17043_clear_interrupt(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	if(chip->config & 0x20) {
		chip->config &= 0xffdf;
		max17040_write_config(chip->client);
	}

	return IRQ_HANDLED;
}

static void max17043_alert_work(struct work_struct *alert_work)
{
	struct max17040_chip *chip;

	chip = container_of(alert_work, struct max17040_chip, alert_work.work);

	max17043_update(chip);
	printk("[max17043:] max17043_alert_work IRQ,39 ***********************************\n");

	max8971_start_charging(800);// default charge current = 800 mA.

	max17043_clear_interrupt(chip->client);
}

static irqreturn_t max17043_interrupt_handler(int irq, void *data)
{
	printk("[max17043:] SOC_ LOW! (irq %d) **********************************\n", irq);
	schedule_delayed_work(&p_max17043_chip->alert_work, 0);

	return IRQ_HANDLED;
}

static int __devinit max17040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max17040_chip *chip;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);

	p_max17043_chip = chip;
/*
	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max17040_get_property;
	chip->battery.properties	= max17040_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max17040_battery_props);

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		kfree(chip);
		return ret;
	}
*/
	max17040_reset(client);
	max17040_get_version(client);
	max17040_read_config(client);

	max17043_set_athd(15);//soc alert thd = 15%.

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, max17040_work);
	schedule_delayed_work(&chip->work, MAX17040_DELAY);
	printk(KERN_WARNING "[Gauge] max17040_probe()......End!\n");
/*
      ret = request_threaded_irq(client->irq, NULL, max17043_interrupt_handler,
              IRQF_ONESHOT | IRQF_TRIGGER_LOW, client->name, chip);//IRQF_TRIGGER_FALLING
      if (unlikely(ret < 0))
      {
          printk("max17043: failed to request IRQ	%X, (client->irq %d)\n", ret, client->irq);
      }
*/
#define GPIO_FUELGAUGE_INT	39
	gpio_request(GPIO_FUELGAUGE_INT, "max17043");
	gpio_direction_input(GPIO_FUELGAUGE_INT);
	ret = request_irq(gpio_to_irq(GPIO_FUELGAUGE_INT), max17043_interrupt_handler, IRQF_TRIGGER_FALLING, "max17043", chip); 
	if (ret < 0) {
		printk(KERN_INFO "[max17043] gpio_39 GPIO_FUELGAUGE_INT IRQ line set up failed!\n");
		free_irq(gpio_to_irq(GPIO_FUELGAUGE_INT), &client->dev);
		return -ENOSYS;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&chip->alert_work, max17043_alert_work);

	max17043_clear_interrupt(client);
	printk(KERN_WARNING "[Gauge] max17040_probe(): ......gpio_to_irq(39) %d. End!\n", gpio_to_irq(GPIO_FUELGAUGE_INT));

	return 0;
}

static int __devexit max17040_remove(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int max17040_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max17040_resume(struct i2c_client *client)
{
	struct max17040_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, MAX17040_DELAY);
	return 0;
}

#else

#define max17040_suspend NULL
#define max17040_resume NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id max17040_id[] = {
	{ "max17043", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17040_id);

static struct i2c_driver max17040_i2c_driver = {
	.driver	= {
		.name	= "max17043",
	},
	.probe		= max17040_probe,
	.remove		= __devexit_p(max17040_remove),
	.suspend	= max17040_suspend,
	.resume		= max17040_resume,
	.id_table	= max17040_id,
};

static int __init max17040_init(void)
{
	return i2c_add_driver(&max17040_i2c_driver);
}
module_init(max17040_init);

static void __exit max17040_exit(void)
{
	i2c_del_driver(&max17040_i2c_driver);
}
module_exit(max17040_exit);

MODULE_AUTHOR("Minkyu Kang <mk7.kang@samsung.com>");
MODULE_DESCRIPTION("MAX17040 Fuel Gauge");
MODULE_LICENSE("GPL");
