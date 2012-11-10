/*
 * MAX8971 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX8971_CHARGER_H
#define __LINUX_MAX8971_CHARGER_H

#define MAX8971_I2C_NAME        "MAX8971"
#define MAX8971_I2C_ADDR        (u8)0x6A >> 1

////////////////////////////////////////////////////////////////////////

#define OMAP_SEND                               122  //for fuel gauge reset on CP.

#define D(fmt, args...) //printk("D]%s() :: " fmt "\n", __func__, ##args) //euiseop.shin

#define TEMP_LOW_NO_BAT                 -300
#define TEMP_LOW_DISCHARGING    -100
#define TEMP_HIGH_DISCHARGING   550

#define TEMP_LOW_RECHARGING             -50
#define TEMP_HIGH_RECHARGING            500

#define TEMP_CHANGE_CHARGING_MODE      450    /* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-23, To change the charging mode according battery temperature*/

#define RECHARGING_BAT_SOC_CON  97

/* Function Prototype */
enum power_supply_type get_charging_ic_status(void);

void charging_ic_active_default(void);
void charging_ic_deactive(void);

typedef enum {
        CHARG_FSM_CAUSE_ANY = 0,
        CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED,
}charger_fsm_cause;
void charger_fsm(charger_fsm_cause reason);


// [jongho3.lee@lge.com] export temperature func.
int twl6030battery_temperature(void);
int get_bat_soc(void);
struct delayed_work* get_charger_work(void);


typedef enum {
        FACTORY_CHARGER_ENABLE,
        FACTORY_CHARGER_DISABLE,
}charge_factory_cmd;

typedef enum {
        CHARGER_DISABLE,
        BATTERY_NO_CHARGER,
        CHARGER_NO_BATTERY,
        CHARGER_AND_BATTERY,
}charge_enable_state_t ;

typedef enum {
        RECHARGING_WAIT_UNSET,
        RECHARGING_WAIT_SET,
}recharging_state_t;


typedef enum {
        CHARGER_LOGO_STATUS_UNKNOWN,
        CHARGER_LOGO_STATUS_STARTED,
        CHARGER_LOGO_STATUS_END,
}charger_logo_state_t;


/*
enum power_supply_type {
        POWER_SUPPLY_TYPE_BATTERY = 0,
        POWER_SUPPLY_TYPE_UPS,
        POWER_SUPPLY_TYPE_MAINS,
        POWER_SUPPLY_TYPE_USB,
        //LGE_CHANGE [jongho3.lee@lge.com]  adding factory charger
        POWER_SUPPLY_TYPE_FACTROY,
        POWER_SUPPLY_TYPE_UNKNOWN,
};
*/



/////////////////////////////////////////////////////////////////////////////

#define FCHG_CURRENT(x) ((x-250)/50+5)  // 0 and from 250mA to 1550mA in 50mA steps.

enum {
    MAX8971_FCHGTIME_DISABLE,
    MAX8974_FCHGTIME_4HRS,
    MAX8974_FCHGTIME_5HRS,  
    MAX8974_FCHGTIME_6HRS,  
    MAX8974_FCHGTIME_7HRS,  
    MAX8974_FCHGTIME_8HRS,  
    MAX8974_FCHGTIME_9HRS,  
    MAX8974_FCHGTIME_10HRS,  
};

enum {
    MAX8971_TOPOFFTIME_0MIN,
    MAX8971_TOPOFFTIME_10MIN,
    MAX8971_TOPOFFTIME_20MIN,
    MAX8971_TOPOFFTIME_30MIN,
    MAX8971_TOPOFFTIME_40MIN,
    MAX8971_TOPOFFTIME_50MIN,
    MAX8971_TOPOFFTIME_60MIN,
    MAX8971_TOPOFFTIME_70MIN,
};

enum {
    MAX8971_TOPOFFTSHLD_50mA,
    MAX8971_TOPOFFTSHLD_100mA,
    MAX8971_TOPOFFTSHLD_150mA,
    MAX8971_TOPOFFTSHLD_200mA,
};

enum {
    MAX8971_CHGCV_4P20V,
    MAX8971_CHGCV_4P10V,
    MAX8971_CHGCV_4P35V,
};

enum {
    MAX8971_CHGRSTRT_150mV,
    MAX8971_CHGRSTRT_100mV,
};

#define DCI_LIMIT(x) ((x<=100) ? 0 :           \
					  (x>=250 && x<=1550) ? ((x-250)/25+10) : -EINVAL)

enum {
    MAX8971_REGTEMP_105degree,
    MAX8971_REGTEMP_90degree,
    MAX8971_REGTEMP_120degree,
    MAX8971_REGTEMP_DISABLE,
};

enum {
    MAX8971_THM_CNFG_CONTINUOUS,
    MAX8971_THM_CNFG_NOT_MONITOR,
};

enum {
    MAX8971_SAFETYREG_REGION1,
    MAX8971_SAFETYREG_REGION2,
};

#define MAX8971_CHGPROT_LOCKED      0x00
#define MAX8971_CHGPROT_UNLOCKED    0x03

/* IRQ definitions */
enum {
	MAX8971_IRQ_PWRUP_OK =0,
	MAX8971_IRQ_THM,
	MAX8971_IRQ_BAT,
	MAX8971_IRQ_CHG,
	MAX8971_IRQ_DCUVP,
	MAX8971_IRQ_DCOVP,
	//MAX8971_IRQ_DCI,
	MAX8971_IRQ_TOPOFF,
	MAX8971_IRQ_DCV,      
	MAX8971_NR_IRQS,
};

struct max8971_platform_data {
    u8      chgcc;              // Fast Charge Current
    u8      fchgtime;           // Fast Charge Time

    u8      chgrstrt;           // Fast Charge Restart Threshold
    u8      dcilmt;             // Input Current Limit Selection

    u8      topofftime;         // Top Off Timer Setting 
    u8      topofftshld;        // Done Current Threshold
    u8      chgcv;              // Charger Termination Voltage
    u8      ifst2p8;            // factory mode test

    u8      regtemp;            // Die temperature thermal regulation loop setpoint
    u8      safetyreg;          // JEITA Safety region selection
    u8      thm_config;         // Thermal monitor configuration

    u8      int_mask;           // CHGINT_MASK
};
#if !defined(CONFIG_MACH_LGE_U2)

#define RECHARGING_BAT_VOLT_LOW        		4150
#define RECHARGING_BAT_VOLT_HIGH                4193

#else

#define RECHARGING_BAT_VOLT_LOW        		4270
#define RECHARGING_BAT_VOLT_HIGH                4350

#endif

#define GPIO_CHG_STATUS_INT     51


#if defined(CONFIG_MACH_LGE_U2_P760)

#define CURRENT_HEAT_SCENE       400

#define TA_CHARING_CURRENT      850
#define TA_CHARING_CURRENT_3800      800

#define TA_RECHARING_CURRENT      500

#define USB_CHARING_CURRENT     500
#define USB_CHARING_CURRENT_MST     450

#endif

#if defined(CONFIG_MACH_LGE_U2_P769)

#define CURRENT_HEAT_SCENE       400

#define TA_CHARING_CURRENT      850
#define TA_CHARING_CURRENT_3800      800


#define TA_RECHARING_CURRENT      500

#define USB_CHARING_CURRENT     500
#define USB_CHARING_CURRENT_MST     450

#endif

#if defined(CONFIG_MACH_LGE_U2_P768)

#define CURRENT_HEAT_SCENE      400

#define TA_CHARING_CURRENT      850
#define TA_CHARING_CURRENT_3800      800

#define TA_RECHARING_CURRENT      500

#define USB_CHARING_CURRENT     500
#define USB_CHARING_CURRENT_MST     450

#endif



int max8971_stop_charging(void);
int max8971_stop_factory_charging(void);
int max8971_start_charging(unsigned mA);
void charger_fsm_max8971(charger_fsm_cause fsm_cause);
/// max17043 fuel gauge..
void set_boot_charging_mode(int charging_mode);
int max8971_start_Factory_charging(void);

extern int charging_done_flag ;

int get_unlimited_temp(void);
#endif

