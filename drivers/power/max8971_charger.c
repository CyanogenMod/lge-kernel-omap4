
/*
 * MAXIM MAX8971 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/max8971.h>

#include <linux/delay.h>
/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21 */
#if defined(CONFIG_MUIC)
#include <linux/muic/muic.h>
#include <linux/muic/muic_client.h>
#else
#include <linux/muic/muic.h>
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] */

// there is no this part in max8971 patch file
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>              // might need to get fuel gauge info

// define register map

//int charging_done_flag = 0;

int charger_source=0;
// there is no this part in max8971 patch file

#define MAX8971_REG_CHGINT      0x0F

#define MAX8971_REG_CHGINT_MASK 0x01

#define MAX8971_REG_CHG_STAT    0x02

#define MAX8971_DCV_MASK        0x80
#define MAX8971_DCV_SHIFT       7
#define MAX8971_TOPOFF_MASK     0x40
#define MAX8971_TOPOFF_SHIFT    6
#define MAX8971_DCI_OK          0x40    // for status register
#define MAX8971_DCI_OK_SHIFT    6
#define MAX8971_DCOVP_MASK      0x20
#define MAX8971_DCOVP_SHIFT     5
#define MAX8971_DCUVP_MASK      0x10
#define MAX8971_DCUVP_SHIFT     4
#define MAX8971_CHG_MASK        0x08
#define MAX8971_CHG_SHIFT       3
#define MAX8971_BAT_MASK        0x04
#define MAX8971_BAT_SHIFT       2
#define MAX8971_THM_MASK        0x02
#define MAX8971_THM_SHIFT       1
#define MAX8971_PWRUP_OK_MASK   0x01
#define MAX8971_PWRUP_OK_SHIFT  0
#define MAX8971_I2CIN_MASK      0x01
#define MAX8971_I2CIN_SHIFT     0

#define MAX8971_REG_DETAILS1    0x03
#define MAX8971_DC_V_MASK       0x80
#define MAX8971_DC_V_SHIFT      7
#define MAX8971_DC_I_MASK       0x40
#define MAX8971_DC_I_SHIFT      6
#define MAX8971_DC_OVP_MASK     0x20
#define MAX8971_DC_OVP_SHIFT    5
#define MAX8971_DC_UVP_MASK     0x10
#define MAX8971_DC_UVP_SHIFT    4
#define MAX8971_THM_DTLS_MASK   0x07
#define MAX8971_THM_DTLS_SHIFT  0

#define MAX8971_THM_DTLS_COLD   1       // charging suspended(temperature<T1)
#define MAX8971_THM_DTLS_COOL   2       // (T1<temperature<T2)
#define MAX8971_THM_DTLS_NORMAL 3       // (T2<temperature<T3)
#define MAX8971_THM_DTLS_WARM   4       // (T3<temperature<T4)
#define MAX8971_THM_DTLS_HOT    5       // charging suspended(temperature>T4)

#define MAX8971_REG_DETAILS2    0x04
#define MAX8971_BAT_DTLS_MASK   0x30
#define MAX8971_BAT_DTLS_SHIFT  4
#define MAX8971_CHG_DTLS_MASK   0x0F
#define MAX8971_CHG_DTLS_SHIFT  0

#define MAX8971_BAT_DTLS_BATDEAD        0   // VBAT<2.1V
#define MAX8971_BAT_DTLS_TIMER_FAULT    1   // The battery is taking longer than expected to charge
#define MAX8971_BAT_DTLS_BATOK          2   // VBAT is okay.
#define MAX8971_BAT_DTLS_GTBATOV        3   // VBAT > BATOV

#define MAX8971_CHG_DTLS_DEAD_BAT           0   // VBAT<2.1V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_PREQUAL            1   // VBAT<3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CC     2   // VBAT>3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CV     3   // VBAT=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TOP_OFF            4   // VBAT>=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_DONE               5   // VBAT>VBATREG, T>Ttopoff+16s, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TIMER_FAULT        6   // VBAT<VBATOV, TJ<TJSHDN
#define MAX8971_CHG_DTLS_TEMP_SUSPEND       7   // TEMP<T1 or TEMP>T4
#define MAX8971_CHG_DTLS_USB_SUSPEND        8   // charger is off, DC is invalid or chaarger is disabled(USBSUSPEND)
#define MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE    9   // TJ > REGTEMP
#define MAX8971_CHG_DTLS_CHG_OFF            10  // charger is off and TJ >TSHDN

#define MAX8971_REG_CHGCNTL1    0x05
#define MAX8971_DCMON_DIS_MASK  0x02
#define MAX8971_DCMON_DIS_SHIFT 1
#define MAX8971_USB_SUS_MASK    0x01
#define MAX8971_USB_SUS_SHIFT   0

#define MAX8971_REG_FCHGCRNT    0x06
#define MAX8971_CHGCC_MASK      0x1F
#define MAX8971_CHGCC_SHIFT     0
#define MAX8971_FCHGTIME_MASK   0xE0
#define MAX8971_FCHGTIME_SHIFT  5


#define MAX8971_REG_DCCRNT      0x07
#define MAX8971_CHGRSTRT_MASK   0x40
#define MAX8971_CHGRSTRT_SHIFT  6
#define MAX8971_DCILMT_MASK     0x3F
#define MAX8971_DCILMT_SHIFT    0

#define MAX8971_REG_TOPOFF          0x08
#define MAX8971_TOPOFFTIME_MASK     0xE0
#define MAX8971_TOPOFFTIME_SHIFT    5
#define MAX8971_IFST2P8_MASK        0x10
#define MAX8971_IFST2P8_SHIFT       4
#define MAX8971_TOPOFFTSHLD_MASK    0x0C
#define MAX8971_TOPOFFTSHLD_SHIFT   2
#define MAX8971_CHGCV_MASK          0x03
#define MAX8971_CHGCV_SHIFT         0

#define MAX8971_REG_TEMPREG     0x09
#define MAX8971_REGTEMP_MASK    0xC0
#define MAX8971_REGTEMP_SHIFT   6
#define MAX8971_THM_CNFG_MASK   0x08
#define MAX8971_THM_CNFG_SHIFT  3
#define MAX8971_SAFETYREG_MASK  0x01
#define MAX8971_SAFETYREG_SHIFT 0

#define MAX8971_REG_PROTCMD     0x0A
#define MAX8971_CHGPROT_MASK    0x0C
#define MAX8971_CHGPROT_SHIFT   2

//dukwung.kim [start]


#define UNLIMITED_TEMP_VAL      0xA4

#define CHG_EN_SET_N_OMAP               83

#define CHR_IC_DEALY                            200     /* 200 us */
#define CHR_IC_SET_DEALY                        1500    /* 1500 us */
#define CHR_TIMER_SECS                          3600 /* 7200 secs*/
#define TEMP_NO_BATTERY  1   //[jongho3.lee@lge.com] in kernel muic doesn't work now..

#define MAX8971_TOPOFF_WORKAROUND

#ifdef MAX8971_TOPOFF_WORKAROUND
#define MAX8971_TOPOFF_DELAY    ((HZ) * 60 + 0 /* + topoff timer */ )  // 60 seconds + toppoff timer
#endif
struct max8971_chip *test_chip;
static DEFINE_MUTEX(charging_lock);
static int bat_soc;
struct delayed_work     charger_timer_work;
struct timer_list charging_timer;
enum power_supply_type charging_ic_status;


struct max8971_chip {
        struct i2c_client *client;

#ifdef MAX8971_TOPOFF_WORKAROUND
	struct delayed_work topoff_work;
        u8 start_topoff_work;
	u8 prev_chg_dtl;
#endif
	struct power_supply charger;
	struct max8971_platform_data *pdata;
    int irq;
    int chg_online;
};

static struct max8971_chip *max8971_chg;
/*
int get_charging_done_flag(void)
{

	return charging_done_flag ;
}
*/
static int max8971_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int i=3;
	int ret;

	while (i--) {
        ret= i2c_smbus_write_byte_data(client, reg, value);

        if (ret < 0)
                dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	else 
		break;
	}

        return ret;


}


enum power_supply_type get_charging_ic_status()
{
        return charging_ic_status;
}
EXPORT_SYMBOL(get_charging_ic_status);

// LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-09, [P940] Add Charger IC(Max8971) in Rev.A
void set_boot_charging_mode(int charging_mode)
{
        charging_ic_status = charging_mode;
       // DCHG("charging_ic_status: %d", charging_ic_status);
}
EXPORT_SYMBOL(set_boot_charging_mode);
// LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-09, [P940]




static int max8971_read_reg(struct i2c_client *client, u8 reg)
{
	int i=3;
	int ret;
	
	while(i--){
		ret=i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	else
		break;
	
	}
	return ret;
}

static int max8971_set_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	u8 value = 0;
	int ret;

	ret = max8971_read_reg(client, reg);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= data;
	ret = max8971_write_reg(client, reg, value);
out:
	return ret;
}


static int __set_charger(struct max8971_chip *chip, int enable)
{
    u8  reg_val= 0;

//	 D("  __set_charger\n");
    // unlock charger protection
    reg_val = MAX8971_CHGPROT_UNLOCKED<<MAX8971_CHGPROT_SHIFT;
    max8971_write_reg(chip->client, MAX8971_REG_PROTCMD, reg_val);   

//	 D("  __set_charger before enable \n");
	if (enable) {
		/* enable charger */

        // Set fast charge current and timer
        reg_val = ((chip->pdata->chgcc<<MAX8971_CHGCC_SHIFT) |
                   (chip->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
        max8971_write_reg(chip->client, MAX8971_REG_FCHGCRNT, reg_val);

       // Set input current limit and charger restart threshold
        reg_val = ((chip->pdata->chgrstrt<<MAX8971_CHGRSTRT_SHIFT) |
                   (chip->pdata->dcilmt<<MAX8971_DCILMT_SHIFT));
        max8971_write_reg(chip->client, MAX8971_REG_DCCRNT, reg_val);

        // Set topoff condition
        reg_val = ((chip->pdata->topofftime<<MAX8971_TOPOFFTIME_SHIFT) |
                   (chip->pdata->topofftshld<<MAX8971_TOPOFFTSHLD_SHIFT) |
                   (chip->pdata->chgcv<<MAX8971_CHGCV_SHIFT));
        max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, reg_val);

        // Set temperature condition
        reg_val = ((chip->pdata->regtemp<<MAX8971_REGTEMP_SHIFT) |
                   (chip->pdata->thm_config<<MAX8971_THM_CNFG_SHIFT) |
                   (chip->pdata->safetyreg<<MAX8971_SAFETYREG_SHIFT));
        max8971_write_reg(chip->client, MAX8971_REG_TEMPREG, reg_val);       

        // USB Suspend and DC Voltage Monitoring
        // Set DC Voltage Monitoring to Enable and USB Suspend to Disable
//        reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
//        max8971_write_reg(chip->client, MAX8971_REG_CHGCNTL1, reg_val);  

	} else {
		/* disable charge */
        max8971_set_bits(chip->client, MAX8971_REG_CHGCNTL1, MAX8971_USB_SUS_MASK, 1);
	}
	dev_info(&chip->client->dev, "%s\n", (enable) ? "Enable charger" : "Disable charger");
	return 0;
}
int max8971_start_charging(unsigned mA)
{
    u8 reg_val=0;
    int i;

//    reg_val = ((FCHG_CURRENT(250)<<MAX8971_CHGCC_SHIFT) |
//               (max8971_chg->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
//    max8971_write_reg(max8971_chg->client, MAX8971_REG_FCHGCRNT, reg_val);
    reg_val = ((max8971_chg->pdata->chgrstrt<<MAX8971_CHGRSTRT_SHIFT) |
               (0x09<<MAX8971_DCILMT_SHIFT));				// 100mA
    max8971_write_reg(max8971_chg->client, MAX8971_REG_DCCRNT, reg_val);

    if(mA==900)
    {
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
	reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==850) 
    {
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);

        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==800) 
    {
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);

        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==600) 
    {
	// charging_ic_status = POWER_SUPPLY_TYPE_USB;
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==500) 
    {
	// charging_ic_status = POWER_SUPPLY_TYPE_USB;
	if(charger_source==2)
       	    reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	else
            reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);

        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==450) 
    {
	charging_ic_status = POWER_SUPPLY_TYPE_USB;
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }

    else if(mA==1550)
    {
	charging_ic_status = POWER_SUPPLY_TYPE_FACTORY ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==400)  
    {
	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==350)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==300)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==250)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }

    else
	printk("[max8971_start_chargin] can not find current\n");


   // reg_val = max8971_chg->pdata->int_mask=0xF3 ;
    reg_val = max8971_chg->pdata->int_mask=0xE3 ;
    max8971_write_reg(max8971_chg->client,MAX8971_REG_CHGINT_MASK , reg_val);
    // charger inserted
    max8971_chg->chg_online = 1;
    max8971_chg->pdata->chgcc = FCHG_CURRENT(mA);
    printk("[max8971] max8971_chg->pdata->chgcc: 0x%x \n", max8971_chg->pdata->chgcc);

    reg_val = ((FCHG_CURRENT(mA)<<MAX8971_CHGCC_SHIFT) |
               (max8971_chg->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
    max8971_write_reg(max8971_chg->client, MAX8971_REG_FCHGCRNT, reg_val);
    for (i=0x09; i<=0x1F; i++)
    {
        reg_val = ((max8971_chg->pdata->chgrstrt<<MAX8971_CHGRSTRT_SHIFT) |
                   (i<<MAX8971_DCILMT_SHIFT));
        max8971_write_reg(max8971_chg->client, MAX8971_REG_DCCRNT, reg_val);
 //       reg_val = ((FCHG_CURRENT(i)<<MAX8971_CHGCC_SHIFT) |
      //  reg_val = ((FCHG_CURRENT(450)<<MAX8971_CHGCC_SHIFT) |
//                   (max8971_chg->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
  //      max8971_write_reg(max8971_chg->client, MAX8971_REG_FCHGCRNT, reg_val);
    	printk("[max8971] max8971_chg->pdata->dcilmt: %d \n",i);
        udelay(200);
    }
    reg_val = ((max8971_chg->pdata->chgrstrt<<MAX8971_CHGRSTRT_SHIFT) |
               (max8971_chg->pdata->dcilmt<<MAX8971_DCILMT_SHIFT));
    max8971_write_reg(max8971_chg->client, MAX8971_REG_DCCRNT, reg_val);
    
    __set_charger(max8971_chg, 1);



    return 0;
}
EXPORT_SYMBOL(max8971_start_charging);

static int max8971_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;

    switch (irq) 
    {
    case MAX8971_IRQ_PWRUP_OK:
 //       dev_info(&chip->client->dev, "Power Up OK Interrupt\n");
        if ((val[0] & MAX8971_DCUVP_MASK) == 0) {
            // check DCUVP_OK bit in CGH_STAT
            // Vbus is valid //
            // Mask interrupt regsiter //
            max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);            
            // DC_V valid and start charging
            chip->chg_online = 1;
        //   __set_charger(chip, 1);
        }
        break;

    case MAX8971_IRQ_THM:
  //      dev_info(&chip->client->dev, "Thermistor Interrupt: details-0x%x\n", (val[1] & MAX8971_THM_DTLS_MASK));
        break;

    case MAX8971_IRQ_BAT:
        dev_info(&chip->client->dev, "Battery Interrupt: details-0x%x\n", (val[2] & MAX8971_BAT_DTLS_MASK));
        switch ((val[2] & MAX8971_BAT_MASK)>>MAX8971_BAT_SHIFT) 
        {
        case MAX8971_BAT_DTLS_BATDEAD:
            break;
        case MAX8971_BAT_DTLS_TIMER_FAULT:
            break;
        case MAX8971_BAT_DTLS_BATOK:
            break;
        case MAX8971_BAT_DTLS_GTBATOV:
            break;
        default:
            break;
        }
        break;

    case MAX8971_IRQ_CHG:
  //      dev_info(&chip->client->dev, "Fast Charge Interrupt: details-0x%x\n", (val[2] & MAX8971_CHG_DTLS_MASK));
        switch (val[2] & MAX8971_CHG_DTLS_MASK) 
        {
        case MAX8971_CHG_DTLS_DEAD_BAT:
            // insert event if a customer need to do something //
       		dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_DEAD_BAT\n");
            break;
        case MAX8971_CHG_DTLS_PREQUAL:
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
	    	dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_FAST_CHARGE_CC\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_FAST_CHARGE_CV\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_TOP_OFF:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_TOP_OFF\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_DONE:
            // insert event if a customer need to do something //
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_DONE\n");
	    charging_done_flag = 1;
            // Charging done and charge off automatically
            break;
        case MAX8971_CHG_DTLS_TIMER_FAULT:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_TIMER_FAULT\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_TEMP_SUSPEND:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_TEMP_SUSPEND\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_USB_SUSPEND:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_USB_SUSPEND\n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE \n");
            // insert event if a customer need to do something //
            break;
        case MAX8971_CHG_DTLS_CHG_OFF:
	    dev_info(&chip->client->dev, "MAX8971_CHG_DTLS_CHG_OFF \n");
            // insert event if a customer need to do something //
            break;
        default:
            break;
        }
        break;

    case MAX8971_IRQ_DCUVP:
        if ((val[1] & MAX8971_DC_UVP_MASK) == 0) {
            // VBUS is invalid. VDC < VDC_UVLO
            //__set_charger(chip, 0);
        }
        dev_info(&chip->client->dev, "DC Under voltage Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_UVP_MASK));
        break;

    case MAX8971_IRQ_DCOVP:
        if (val[1] & MAX8971_DC_OVP_MASK) {
            // VBUS is invalid. VDC > VDC_OVLO
//            __set_charger(chip, 0);
        }
        dev_info(&chip->client->dev, "DC Over voltage Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_OVP_MASK));
        break;

//    case MAX8971_IRQ_DCI:
//        dev_info(&chip->client->dev, "DC Input Current Limit Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_I_MASK));
//        break;
    case  MAX8971_IRQ_TOPOFF:
        dev_info(&chip->client->dev, "DC MAX8971_IRQ_TOPOFF Interrupt: details-0x%x\n", (val[1] & MAX8971_TOPOFF_MASK));
        break;


    case MAX8971_IRQ_DCV:
        dev_info(&chip->client->dev, "DC Input Voltage Limit Interrupt: details-0x%x\n", (val[1] & MAX8971_DC_V_MASK));
        break;

    }
    return 0;
}

static irqreturn_t max8971_charger_handler(int irq, void *data)
{	
    struct max8971_chip *chip = (struct max8971_chip *)data;
    int irq_val, irq_mask, irq_name;
    u8 val[3];
//    D("  max8971_charger_handler\n");
    irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
    if(irq_val < 0)
	return 0;   
    irq_mask = max8971_read_reg(chip->client, MAX8971_REG_CHGINT_MASK);
    if(irq_mask < 0)
	return 0;   

    val[0] = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
    if( val[0] < 0)
	return 0;   
    val[1] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
    if( val[1] < 0)
	return 0;   
    val[2] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
    if( val[2] < 0)
	return 0;   

        dev_info(&chip->client->dev, "MAX8971_REG_CHG_STAT- 0x%x\n", val[0]);
        dev_info(&chip->client->dev, "MAX8971_REG_DETAILS1- 0x%x\n", val[1]);
        dev_info(&chip->client->dev, "MAX8971_REG_DETAILS2- 0x%x\n", val[2]);

    for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name<MAX8971_NR_IRQS; irq_name++) {
        if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
            max8971_charger_detail_irq(irq_name, data, val);
        }
    }
	return IRQ_HANDLED;
}

int max8971_stop_charging(void)
{
    u8 reg_val=0;
//    reg_val = max8971_chg->pdata->int_mask=0xFF ;
//    max8971_write_reg(max8971_chg->client,MAX8971_REG_CHGINT_MASK , reg_val);
    // Charger removed
    if(max8971_chg != NULL )
    {
	printk(" max8971_chg pointer is not NULL\n");
     	max8971_chg->chg_online = 0;
    	__set_charger(max8971_chg, 0);
    }
    else
	printk(" max8971_chg pointer is NULL\n");

        charging_ic_status = POWER_SUPPLY_TYPE_BATTERY;
    // Disable GSM TEST MODE
  //  max8971_set_bits(max8971_chg->client, MAX8971_REG_TOPOFF, MAX8971_IFST2P8_MASK, 0<<MAX8971_IFST2P8_SHIFT);

    return 0;
}
EXPORT_SYMBOL(max8971_stop_charging);

int max8971_stop_factory_charging(void)
{
    u8 reg_val=0;
    // Charger removed
    if(max8971_chg != NULL)
    {
	printk(" max8971_chg pointer is not NULL\n");
    	max8971_chg->chg_online = 0;
    	__set_charger(max8971_chg, 0);

    // Disable GSM TEST MODE
    max8971_set_bits(max8971_chg->client, MAX8971_REG_TOPOFF, MAX8971_IFST2P8_MASK, 0<<MAX8971_IFST2P8_SHIFT);
    }
    else
        printk(" max8971_chg pointer is NULL\n");
    return 0;
}
EXPORT_SYMBOL(max8971_stop_factory_charging);

/*
int max8971_start_charging(unsigned mA)
{
    u8 reg_val=0;
    int i;

    reg_val = ((FCHG_CURRENT(250)<<MAX8971_CHGCC_SHIFT) |
               (max8971_chg->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
    max8971_write_reg(max8971_chg->client, MAX8971_REG_FCHGCRNT, reg_val);

    if(mA==900)
    {
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
	reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==850) 
    {
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);

        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==800) 
    {
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);

        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
	charging_ic_status = POWER_SUPPLY_TYPE_MAINS;
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==600) 
    {
	// charging_ic_status = POWER_SUPPLY_TYPE_USB;
        reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==500) 
    {
	// charging_ic_status = POWER_SUPPLY_TYPE_USB;
        reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==450) 
    {
	charging_ic_status = POWER_SUPPLY_TYPE_USB;
        reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }

    else if(mA==1550)
    {
	charging_ic_status = POWER_SUPPLY_TYPE_FACTORY ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
        max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==400)  
    {
	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==350)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==300)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }
    else if(mA==250)  
    {
//	charging_ic_status = POWER_SUPPLY_TYPE_USB ;
	reg_val = (0<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
	max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
        dev_info(&max8971_chg->client->dev, "[LGE_MAX8971] MAX8971_REG_CHGCNTL1 = 0x%x", reg_val);
    }

    else
	printk("[max8971_start_chargin] can not find current\n");


    //reg_val = max8971_chg->pdata->int_mask=0xF3 ;
    reg_val = max8971_chg->pdata->int_mask=0xE3 ;
    max8971_write_reg(max8971_chg->client,MAX8971_REG_CHGINT_MASK , reg_val);
    // charger inserted
    max8971_chg->chg_online = 1;
    max8971_chg->pdata->chgcc = FCHG_CURRENT(mA);
    printk("[max8971] max8971_chg->pdata->chgcc: 0x%x \n", max8971_chg->pdata->chgcc);

    for (i=250; i<=450; i+=50)
    {
        //reg_val = ((FCHG_CURRENT(i)<<MAX8971_CHGCC_SHIFT) |
        reg_val = ((FCHG_CURRENT(450)<<MAX8971_CHGCC_SHIFT) |
                   (max8971_chg->pdata->fchgtime<<MAX8971_FCHGTIME_SHIFT));
        max8971_write_reg(max8971_chg->client, MAX8971_REG_FCHGCRNT, reg_val);
    	printk("[max8971] max8971_chg->pdata->chgcc: %d \n",i);
        udelay(200);
    }
    
    __set_charger(max8971_chg, 1);



    return 0;
}
EXPORT_SYMBOL(max8971_start_charging);

*/

int max8971_start_Factory_charging(void)
{
    printk("[max8971] max8971_start_Factory_charging\n");
    u8 reg_val=0;
    charging_ic_status = POWER_SUPPLY_TYPE_FACTORY;
    reg_val = MAX8971_CHGPROT_UNLOCKED<<MAX8971_CHGPROT_SHIFT;
    max8971_write_reg(max8971_chg->client, MAX8971_REG_PROTCMD, reg_val);   
    reg_val = max8971_chg->pdata->int_mask=0xF3 ;
    max8971_write_reg(max8971_chg->client,MAX8971_REG_CHGINT_MASK , reg_val);
    max8971_chg->chg_online = 1;
    reg_val = (1<<MAX8971_DCMON_DIS_SHIFT) | (0<<MAX8971_USB_SUS_SHIFT);
    max8971_write_reg(max8971_chg->client, MAX8971_REG_CHGCNTL1, reg_val);
    max8971_set_bits(max8971_chg->client, MAX8971_REG_TOPOFF, MAX8971_IFST2P8_MASK, 1<<MAX8971_IFST2P8_SHIFT);
    return 0;
}
EXPORT_SYMBOL(max8971_start_Factory_charging);

static int max8971_charger_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
	struct max8971_chip *chip = container_of(psy, struct max8971_chip, charger);
	int ret = 0;
    	int chg_dtls_val;
	printk(" max8971_charger_get_property: psp= %d\n",psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->chg_online;
		break;
    case POWER_SUPPLY_PROP_STATUS:
		ret = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
        chg_dtls_val = (ret & MAX8971_CHG_DTLS_MASK);
        if (chip->chg_online) {
            if (chg_dtls_val == MAX8971_CHG_DTLS_DONE) {
                val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
            }
            else if ((chg_dtls_val == MAX8971_CHG_DTLS_TIMER_FAULT) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_TEMP_SUSPEND) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_USB_SUSPEND) ||
                     (chg_dtls_val == MAX8971_CHG_DTLS_CHG_OFF)) {
                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
            }
            else {
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
            }
        }
        else {
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        ret = 0;
		break;	
    default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static enum power_supply_property max8971_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_STATUS,
};

static struct power_supply max8971_charger_ps = {
   .name = "max8971",
  // .type = POWER_SUPPLY_TYPE_MAINS,
   .type = POWER_SUPPLY_TYPE_UNKNOWN ,
   .properties = max8971_charger_props,
   .num_properties = ARRAY_SIZE(max8971_charger_props),
   .get_property = max8971_charger_get_property,
};
/*
#if defined(TOPOFF_WORKAROUND)
static void max8971_topoff(struct work_struct *max8971_top)
{
	struct max8971_chip *chip;
	
	chip = container_of(max8971_top, struct max8971_chip, topoff_work);

        int irq_val, irq_mask, irq_name;
        u8 val[3];
         D("  max8971_charger_handler\n");
        irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
        irq_mask = max8971_read_reg(chip->client, MAX8971_REG_CHGINT_MASK);

        val[0] = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
        val[1] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS1);
        val[2] = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);

    for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name<MAX8971_NR_IRQS; irq_name++) {
        if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
            max8971_charger_detail_irq(irq_name, chip, val);
        }
    }
	max8971_write_reg(chip->client,MAX8971_REG_CHGINT_MASK,0xF3);
}
#endif
*/

/* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-07-29, Add for AT%CHARGE*/
/*
ssize_t charging_ic_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
        printk(" charging_ic_status: %d\n", charging_ic_status);
	if(charging_ic_status == POWER_SUPPLY_TYPE_BATTERY)
		return snprintf(buf, PAGE_SIZE, "0\n");
	else
		return snprintf(buf, PAGE_SIZE, "1\n");
}
ssize_t charging_ic_store_status(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf,
			  size_t count)
{
	if(buf[0] == '0') {
	//	charging_ic_deactive();
		max8971_stop_charging();
              
	} else if(buf[0] == '1') {
	//     	charging_ic_set_ta_mode();
		 max8971_start_charging(900);
	}
	return count;
}
DEVICE_ATTR(charging_state, 0660, charging_ic_show_status, charging_ic_store_status);
*/
/* LGE_CHANGE_E [wonhui.lee@lge.com] 2011-07-29, Add for AT%CHARGE*/

// LGE_CHANGE_S [MUIC] jaesung.woo@lge.com 20120613
#if defined(CONFIG_MUIC)
static int muic_control_max8971_charger(struct muic_client_device *mcdev)
{
//	TYPE_MUIC_MODE muic_mode = muic_get_mode();
	TYPE_MUIC_MODE muic_mode = muic_detect_cable();

	switch (muic_mode)
	{
		case MUIC_NA_TA :
		case MUIC_LG_TA :
		case MUIC_TA_1A :
			pr_info("%s: TA.\n", __func__);
#if defined(CONFIG_MACH_LGE_U2)
			charger_source=2;	
			max8971_start_charging(TA_CHARING_CURRENT);
#else
			max8971_start_charging(800);
#endif
			break;

		case MUIC_AP_USB :
			if(get_unlimited_temp() ==  UNLIMITED_TEMP_VAL){
				pr_info("%s: AP_USB.\n", __func__);
				max8971_start_charging(USB_CHARING_CURRENT_MST);
			}
			else{
				pr_info("%s: AP_USB.\n", __func__);
				max8971_start_charging(USB_CHARING_CURRENT);
			}
			break;

		case MUIC_CP_USB :
			pr_info("%s: CP_USB.\n", __func__);
			max8971_start_Factory_charging();
			break;

		case MUIC_CP_UART :
			pr_info("%s: CP_UART.\n", __func__);
			max8971_start_Factory_charging();
			break;

		case MUIC_MHL :
			pr_info("%s: MHL.\n", __func__);
			max8971_start_charging(400);
			break;

		case MUIC_NONE :
			charger_source=0;
			pr_info("%s: NONE.\n", __func__);
			max8971_stop_charging();
			break;

		default :
			pr_info("%s: No charging.\n", __func__);
			break;
	}

	return 0;
}

static struct muic_client_ops max8971_opt = {
	.notifier_priority = MUIC_CLIENT_NOTI_POWER_MHL,
	.on_none = muic_control_max8971_charger,
	.on_na_ta = muic_control_max8971_charger,
	.on_lg_ta = muic_control_max8971_charger,
	.on_1a_ta = muic_control_max8971_charger,
	.on_cp_uart = muic_control_max8971_charger,
	.on_ap_usb = muic_control_max8971_charger,
	.on_cp_usb = muic_control_max8971_charger,
	.on_mhl = muic_control_max8971_charger,
};
#endif //CONFIG_MUIC
// LGE_CHANGE_E [MUIC] jaesung.woo@lge.com 20120613

static __devinit int max8971_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max8971_chip *chip;
    int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	max8971_chg = chip;
//    test_chip = chip;

	i2c_set_clientdata(client, chip);

//	irq_val = max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
       //TEMP_DUK_S
	chip->charger = max8971_charger_ps;
	//TEMP_DUK_E

	//TEMP_DUK_S
	//ret = power_supply_register(&client->dev, &max8971_charger_ps);
	ret = power_supply_register(&client->dev, &chip->charger);
	//TEMP_DUK_E

	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		goto out;
	}
//    charger_fsm_max8971(CHARG_FSM_CAUSE_ANY);

    ret = request_threaded_irq(client->irq, NULL, max8971_charger_handler,
            IRQF_ONESHOT | IRQF_TRIGGER_LOW, client->name, chip);
    if (unlikely(ret < 0))
    {
        pr_debug("max8971: failed to request IRQ	%X\n", ret);
        goto out;
    }

        /* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-07-29, Add for AT%CHARGE*/
        // for AT Command AT%CHARGE
       
	//ret = device_create_file(&client->dev,&dev_attr_charging_state);
        
/*	if (ret < 0) {
                pr_err("%s:File device creation failed: %d\n", __func__, ret);
        }
*/
    /* LGE_CHANGE_E [wonhui.lee@lge.com] 2011-07-29, Add for AT%CHARGE*/


        ret=max8971_read_reg(chip->client, MAX8971_REG_CHGINT);
        printk(" MAX8971_REG_CHGINT : %d\n", ret);
	chip->chg_online = 0;
	ret = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);

	if (ret >= 0) 
    	{
		chip->chg_online = (ret & MAX8971_DCUVP_MASK) ? 0 : 1;
        	if (chip->chg_online) 
        	{
        	    // Set IRQ MASK register
            	max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);
	          //  __set_charger(chip, 1);
        	}
	}

// LGE_CHANGE_S [MUIC] jaesung.woo@lge.com 20120613
#if defined(CONFIG_MUIC)
	ret = muic_client_dev_register(client->name, (void*)chip, &max8971_opt);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register max8971_opt\n");
		goto out;
	}
#endif
// LGE_CHANGE_E [MUIC] jaesung.woo@lge.com 20120613

	return 0;
out:
	kfree(chip);
	return ret;
}

static __devexit int max8971_remove(struct i2c_client *client)
{
    struct max8971_chip *chip = i2c_get_clientdata(client);

	free_irq(client->irq, chip);
	power_supply_unregister(&max8971_charger_ps);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id max8971_id[] = {
	{ "max8971", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max8971_id);

static struct i2c_driver max8971_i2c_driver = {
	.driver = {
		.name = "max8971",
	},
	.probe		= max8971_probe,
	.remove		= __devexit_p(max8971_remove),
	.id_table	= max8971_id,
};

static int __init max8971_init(void)
{
	return i2c_add_driver(&max8971_i2c_driver);
}
module_init(max8971_init);

static void __exit max8971_exit(void)
{
	i2c_del_driver(&max8971_i2c_driver);
}
module_exit(max8971_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maxim-ic.com>");
MODULE_DESCRIPTION("Power supply driver for MAX8971");
MODULE_VERSION("3.3");
MODULE_ALIAS("platform:max8971-charger");

