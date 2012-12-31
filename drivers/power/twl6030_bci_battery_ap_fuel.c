/*
 * linux/drivers/power/twl6030_bci_battery_ap_fuel.c
 *
 * based on twl6030_bci_battery.c
 *
 * OMAP4:TWL6030 battery driver for Linux
 *
 * Copyright (C) 2011-2012 LGE Inc.
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/bq2415x.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <mach/gpio.h>
//roy.park@lge.com 2010.12.8 Start-->[
#include <linux/wakelock.h>
//roy.park@lge.com <---]
/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21 */
#if defined(CONFIG_MUIC)
#include <linux/muic/muic.h>
#include <linux/muic/muic_client.h>
#else
#include <linux/muic/muic.h>
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] */

#if !defined(CONFIG_MAX8971_CHARGER)
// LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13, LGE_P940, Bring in Cosmo.
// LGE_CHANGE [jongho3.lee@lge.com] add driver headers. /start
#include <linux/charger_rt9524.h>
// LGE_CHANGE [jongho3.lee@lge.com] add driver headers. /end
#else
#include <linux/leds.h>
#if defined(CONFIG_MAX8971_CHARGER)
#include <linux/lge/lm3530.h>
#endif

#include <linux/max8971.h>
#endif

#include <linux/fuel_gauge_max17043.h>

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-14, [P940] Add a SMPL feature */
/* LGE_CHANGE [wonhui.lee@lge.com] 2011-07-12, apply SMPL Setting*/
//#define FEATURE_SMPL
#undef FEATURE_SMPL   //nthyunjin.yang 120615 SMPL is not used for CX2.
#if 1 //defined(FEATURE_SMPL)  //nthyunjin.yang 120724 for nv read/write from 0 to 1
#include <plat/lge_nvdata_handler.h>	//[hyunhee.jeon@lge.com]
#endif

/*Battery temp test*/
//#define BAT_TEMP_TEST

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
 * LGE_P940, for battery,power debug
 */
#ifndef  __DEBUG_POWER
#define __DEBUG_POWER

#define __DEBUG_PMIC
#define __DEBUG_CHARGER
#define __DEBUG_FUELGAUGE
#define __DEBUG_MUIC
//#define __DEBUG_TEMP

#ifdef __DEBUG_PMIC 
//#define DPWR(fmt, args...) printk(fmt " :: file=%s, func=%s, line=%d\n", ##args, __FILE__, __func__, __LINE__ ) 
#define DPWR(fmt, args...) printk("[PWR] " fmt "\n", ##args) 
#else
#define DPWR(fmt, args...) 
#endif

#ifdef __DEBUG_CHARGER
#define DCHG(fmt, args...) printk("[CHG] " fmt "\n", ##args) 
#else
#define DCHG(fmt, args...) 
#endif

#ifdef __DEBUG_FUELGAUGE
#define DGAU(fmt, args...) printk("[GAU] " fmt "\n", ##args) 
#else
#define DGAU(fmt, args...) 
#endif

#ifdef __DEBUG_TEMP
#define DTEMP(fmt, args...) printk("[TEMP] " fmt "\n", ##args) 
#else
#define DTEMP(fmt, args...) 
#endif

#ifdef __DEBUG_MUIC
#define DMUIC(fmt, args...) printk("[MUIC] " fmt "\n", ##args) 
#else
#define DMUIC(fmt, args...) 
#endif

#endif /* __DEBUG_POWER */


/* LGE_CHANGE [euiseop.shin@lge.com] 2011-12-12
 * [P2] Add a definition of shutdown voltage
 */
#define BATT_VOLT_SHUTDOWN		3400 //3300->3400

/* LGE_CHANGE [jongho3.lee@lge.com]
 * add INVALID_CAPACITY for gauge control. gauge reset .
 */
#define RESET_CAP			198000
#define INVALID_CAP			199000
#define NOT_INIT_CAP			200000
#define USE_GAUGE_GRAPH_VALIDATION	1

#define CHARGING_WAKE_LOCK		1
#define FLEX_CHARGING_MODE		1

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-06,
 * To change the charging mode according battery SOC
 */
/* 
 * It's will be change charging mode according to battery soc
 * on over-heat scene. over-heat scene has two case as below 
 *   1. while camera recording
 *   2. Youtube streaming
 */ 
#define CHANGE_CHG_MODE_ON_OVERHEAT_SCENE	1

#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
#define LOW_SOC_CHANGE_CHG_MODE			20
#define HIGH_SOC_CHANGE_CHG_MODE		80
#define HIGH_SOC_RECHARGING			78
#endif
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-06,
 * To change the charging mode according battery SOC
 */

#define TWL6030_PMIC_CHARGING	1
#define TWL6030_PMIC_FUELGAUGE	1

#define TEMP_BAT_GAUTE_BY_VOLT	1
#define GET_BET_TEMP		1
#define BAT_TEMP_CHANNEL	2
#if defined(CONFIG_MAX8971_CHARGER)
#define BAT_CURRENT_CHANNEL	1      				// LGE_CHANGE [dukwung.kim@lge.com] 2011-09-16 for check batt current adc value

#define GET_TEMP_SENSOR_DOCOMO	 1 
#define CHARGER_TEMP_CHANNEL	6			/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger current adc value */
#define PARM_TEMP_CHANNEL	5			/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger current adc value */
#define TEMP_SENSOR_OFFSET	100
#define MAX8971_RECHARGING__DELAY    ((HZ) * 5 + 0  )   //// LGE_CHANGE [dukwung.kim@lge.com] 2011-09-17 For recharging 
#endif
#define CREATE_OWN_WORKQUEUE	1

#define CHR_TIMER_SECS		300
#ifdef GET_TEMP_SENSOR_DOCOMO
#define THERM_SENSOR_CHARGER_INDEX 0
#define THERM_SENSOR_PARM_INDEX       1
#endif

/* LGE_CHANGE_S [hyunhee.jeon@lge.com] 2011-02-07
 * access control values for PH_CFG_VBATLOW
 * PH_CFG_VBATLOWV REG ADDR: 0x28
 */
#define PH_CFG_VBATLOW_REG_OFFSET		0x9
#define BB_SEL					(1 << 7)
#define BB_MASK					(1 << 6)
/* LGE_CHANGE_E [hyunhee.jeon@lge.com] 2011-02-07 */

//#if defined(TWL6030_PMIC_CHARGING)
//#if defined(TWL6030_PMIC_FUELGAUGE)

//#if !defined(CONFIG_LG_FW_RT9524_CHARGER)
//#if !defined(CONFIG_LGE_COSMO_MUIC)
//#if !defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE)

//#define NO_DEBUG

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13, LGE_P940, Bring in Cosmo. */
#define UNLIMITED_TEMP_VAL	0xA4 

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-10-28,
 * Change charging temperature value for LG senario [START_LGE]
 */
#define UNLIMITED_TEMP_HIGH	390
#define UNLIMITED_TEMP_LOW	-50
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-10-28,
 * Change charging temperature value for LG senario [END_LGE]
 */

//#define POWER_OFF_WHEN_NO_BAT 1

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-10-19
 * [P2]Enable bk_battery feature.
 */
#define FEATURE_BK_BATTERY	1

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-16,
 * [P940] Add gauge control feature.
 */
/* euiseop.shin 2011-07-12 Enable this feature, because side effect. */
#define FEATURE_GAUGE_CONTROL

/* Global function */
static int twl6030_get_gpadc_conversion(int channel_no);
int move_queue_point(int point, int move_count, int queue_size, int limit);
int validate_gauge_value(int capacity);
#if defined(CONFIG_MAX8971_CHARGER)
//#######################################################
// LGE_CHANGE_S [dukwung.kim@lge.com] 2011-09-17  Check  deceasing battery
int decease_cap_count = 0;
int decease_cap_count_flag=0;
int current_adc_code=0;
int battery_full_status=0;
int batt_temp_adc_old[3] = {600,600,600}; 
int batt_temp_adc_count=0;
int diff_temp_adc=0 ;
int init_temp_adc_flag=0;
int temp_charging_stop_flag=0;
#define TWL6030_PHONIX_DEV_ON   0x25
#define APP_DEVOFF      (1<<0)
#define CON_DEVOFF      (1<<1)
#define MOD_DEVOFF      (1<<2)
#define SW_RESET        (1<<6)
// LGE_CHANGE_E [dukwung.kim@lge.com] 2011-09-17  Check  deceasing battery
#endif
/* Ptr to thermistor table */
static const unsigned int fuelgauge_rate[4] = {4, 16, 64, 256};

#define AVG_VOLT_QUEUE_SIZE	16

struct twl6030_bci_device_info {
	struct device	*dev;
	int		*battery_tmp_tbl;
	unsigned int	tblsize;

	int		voltage_uV;
	int		fg_voltage_mV;
	int		bk_voltage_uV;
	int		current_uA;
	int		current_avg_uA;
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20,
	 * add to check BATT TEMP ADC value
	 */
	int		batt_temp_adc;
#if defined(CONFIG_MAX8971_CHARGER)
	int			batt_current_adc; /* LGE_CHANGE [dukwung.kim@lge.com] 2011-09-16, add to check BATT CURRENT ADC value*/
	int			temp_charger_adc; /* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger temp adc value */
	int 			temp_parm_adc;	/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check parm adc value */
	int			temp_charger_C;	/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger temp value */
	int			temp_parm_C;	/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check parm temp value */
	int			temp_high_C;	/* LGE_CHANGE [jude84.kim@lge.com] 2011-11-10 update temp highest one */
	int			temp_low_C;	/* LGE_CHANGE [jude84.kim@lge.com] 2011-11-10 update temp lowest one */
#endif
	int		temp_C;
	int		charge_status;
	int		charger_source;
	int		vac_priority;
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * adding battery related variables.. [START_LGE]
	 */
	int 		capacity;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-27,
	 * [P940] Capacity ui update error fix
	 */
	int 		ui_capacity;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	int 		modem_alive;
	int 		pmic_capacity;
	int 		cap_seamless;
	int 		capacity_validation;
	int		battery_present;
	int		charger_interrupt;
	int		charger_logo_status;
	int		valid_charging_source;
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
	int		factory_control;
#endif
	/* [jongho3.lee@lge.com] average volt. */
	int 		avg_volt_queue[AVG_VOLT_QUEUE_SIZE];
	int 		avg_volt_queue_count;
	int 		avg_volt_head;
	int 		avg_volt_tail;
	int		avg_voltage_mV;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	int		gauge_control_count;
#endif 
#if defined (CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)
	int		lock_on_overheat_scene;
#endif
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo.
	 */
	u8		temp_control;
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * adding battery related variables.. [END_LGE]
	 */
	int		bat_health;

	int		fuelgauge_mode;
	int		timer_n2;
	int		timer_n1;
	s32		charge_n1;
	s32		charge_n2;
	s16		cc_offset;
	u8		usb_online;
	u8		ac_online;
	u8		stat1;
	u8		status_int1;
	u8		status_int2;
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14,
	 * [P940] Add a SMPL feature
	 */
#if defined(FEATURE_SMPL)
	/* LGE_CHANGE [hyunhee.jeon@lge.com] 0;disalbe 1:enable*/
	u8		smpl_en;
#endif
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14, [P940] */

	u8			watchdog_duration;
	u16			current_avg_interval;
	u16			monitoring_interval;
	unsigned int		min_vbus;
	unsigned int		max_charger_voltagemV;
	unsigned int		max_charger_currentmA;
	unsigned int		charger_incurrentmA;
	unsigned int		charger_outcurrentmA;
	unsigned int		regulation_voltagemV;
	unsigned int		low_bat_voltagemV;
	unsigned int		termination_currentmA;

	struct power_supply	bat;
	struct power_supply	usb;
	struct power_supply	ac;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
#ifdef FEATURE_BK_BATTERY
	struct power_supply	bk_bat;
#endif
	struct delayed_work	twl6030_bci_monitor_work;
	struct delayed_work	twl6030_current_avg_work;
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * adding charger work. [START_LGE]
	 */
	struct delayed_work	charger_work;
	struct delayed_work	gauge_reset_work;
	struct delayed_work	power_off_work;
	struct delayed_work	charger_timer_work;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,[P940]
	 * Imported from Cosmopolitan
	 */
	struct delayed_work	twl_power_supply_changed_work;
#if defined(CONFIG_MAX8971_CHARGER)

//LGE_CHANGE [dukwung.kim@lge.com] 2011-09-17,[L02D] 
	struct delayed_work	twl6030_recharging_work;
#endif
	struct timer_list	off_mode_timer;
	int	   		off_mode_timer_working;

#if defined(CHARGING_WAKE_LOCK)
	struct wake_lock 	charger_wake_lock;
	struct wake_lock 	off_mode_charger_wake_lock;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	//struct wake_lock charger_timed_wake_lock;
	unsigned int wake_lock_count;
#endif

#if defined(CREATE_OWN_WORKQUEUE)
	struct workqueue_struct *charger_workqueue;
#endif
	/* LGE_CHANGE [jongho3.lee@lge.com] adding charger work. [END_LGE] */
	int irq;
};

static struct blocking_notifier_head notifier_list;
//LGE_CHANGE [jongho3.lee@lge.com] expose device info for external charger.
static struct twl6030_bci_device_info* p_di = NULL;
//DECLARE_WAIT_QUEUE_HEAD(muic_event);
//LGE_CHANGE [jongho3.lee@lge.com] set recharging condition on this.
static recharging_state_t recharging_status;
//LGE_CHANGE [jongho3.lee@lge.com]
static DEFINE_MUTEX(charging_fsm_lock);
//static DEFINE_MUTEX(muic_detect_lock);
// LGE_CHANGE_S [jongho3.lee@lge.com] battery graphs...

#define STRT_ON_PLUG_DET            (1 << 3)
static int start_cond;

/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-11-02,
 * Fixed lockup issue while power on/off error.
 */
static int charger_fsm_shutdown = 0;

/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-09,
 * To prevent changing charging mode in charging test
 */
static int charging_test_on = 0;

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-07,
 * To change the charging mode according battery SOC
 */
#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
static int reinit_ta_charger = 0;
#endif
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-07,
 * To change the charging mode according battery SOC
 */

#define SOC_TIMES 	1000
#define VALID_RANGE  	10

battery_graph_prop battery_soc_graph[] =
{
	//s32	(voltage, soc);
	{4200, 100 * SOC_TIMES},
	{4100,	98 * SOC_TIMES},
	{3800,  61 * SOC_TIMES},
	{3700,	34 * SOC_TIMES},
	{3620,	13 * SOC_TIMES},
	{3580,	5 * SOC_TIMES},
	{3300,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_no_charger[] =
{
	//s32	(voltage, soc);
	{4170, 100 * SOC_TIMES},
	{4132,	97 * SOC_TIMES},
	{4058,	90 * SOC_TIMES},
	{3986,  80 * SOC_TIMES},
	{3909,	70 * SOC_TIMES},
	{3840,	60 * SOC_TIMES},
	{3783,	50 * SOC_TIMES},
	{3747,	40 * SOC_TIMES},
	{3726,	30 * SOC_TIMES},
	{3697,	20 * SOC_TIMES},
	{3644,	10 * SOC_TIMES},
	{3550,	2 * SOC_TIMES},
	{3400,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_ac[] =
{
	//s32	(voltage, soc);
	{4279, 100 * SOC_TIMES},
	{4220,	97 * SOC_TIMES},
	{4132,	90 * SOC_TIMES},
	{4112,  80 * SOC_TIMES},
	{4089,	70 * SOC_TIMES},
	{4038,	60 * SOC_TIMES},
	{3993,	50 * SOC_TIMES},
	{3967,	40 * SOC_TIMES},
	{3947,	30 * SOC_TIMES},
	{3910,	20 * SOC_TIMES},
	{3850,	10 * SOC_TIMES},
	{3680,	2 * SOC_TIMES},
	{3500,	0 * SOC_TIMES},
};

battery_graph_prop battery_soc_graph_with_usb[] =
{
	//s32	(voltage, soc);
	{4250, 100 * SOC_TIMES},
	{4200,	97 * SOC_TIMES},
	{4123,	90 * SOC_TIMES},
	{4052,  80 * SOC_TIMES},
	{3968,	70 * SOC_TIMES},
	{3910,	60 * SOC_TIMES},
	{3854,	50 * SOC_TIMES},
	{3808,	40 * SOC_TIMES},
	{3768,	30 * SOC_TIMES},
	{3728,	20 * SOC_TIMES},
	{3688,	10 * SOC_TIMES},
	{3600,	2 * SOC_TIMES},
	{3450,	0 * SOC_TIMES},
};

#define TEMP_TIMES 10000


/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-07-19,
 * Modify battery temp. table [START_LGE]
 */
#if defined(CONFIG_MACH_LGE_P2_SU540) || defined(CONFIG_MACH_LGE_P2_KU5400)
/* LGE_CHANGE [wonhui.lee@lge.com] 2011-09-16,
 * update Battery Temp ADC table for SU540
 */
battery_graph_prop battery_temp_graph[] =
{
	//s32	(adc, temp.);
	{1688, -50 * TEMP_TIMES},
	{1666, -40 * TEMP_TIMES},
	{1622, -30 * TEMP_TIMES},
	{1500, -20 * TEMP_TIMES},
	{1392, -10 * TEMP_TIMES},
	{1256, 0 * TEMP_TIMES},
	{1042, 10 * TEMP_TIMES},
	{855, 20 * TEMP_TIMES},
	{665, 30 * TEMP_TIMES},
	{498, 40 * TEMP_TIMES},
	{366, 50 * TEMP_TIMES},
	{265, 60 * TEMP_TIMES},
	{212, 70 * TEMP_TIMES},
	{154, 80 * TEMP_TIMES},
	{113, 90 * TEMP_TIMES},
	{83, 100 * TEMP_TIMES},
};
#else
battery_graph_prop battery_temp_graph[] =
{
	//s32	(adc, temp.);
	{1668, -50 * TEMP_TIMES},
	{1646, -40 * TEMP_TIMES},
	{1602, -30 * TEMP_TIMES},
	{1480, -20 * TEMP_TIMES},
	{1360, -10 * TEMP_TIMES},
	{1210, 0 * TEMP_TIMES},
	{1030, 10 * TEMP_TIMES},
	{840, 20 * TEMP_TIMES},
	{650, 30 * TEMP_TIMES},
	{490, 40 * TEMP_TIMES},
	{365, 50 * TEMP_TIMES},
	{265, 60 * TEMP_TIMES},
	{212, 70 * TEMP_TIMES},
	{154, 80 * TEMP_TIMES},
	{113, 90 * TEMP_TIMES},
	{83, 100 * TEMP_TIMES},
};
#endif
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-07-19,
 * Change battery temp table [END_LGE]
 */
#if defined(CONFIG_MAX8971_CHARGER)
/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 Docomo temperature sensor convertor table[START_LGE] */
battery_graph_prop dcm_temp_sensor_graph[] =
{
	//s32 (adc, temp.);
	{1000, -30 * TEMP_TIMES},
	{998, -20 * TEMP_TIMES},
	{947, -10 * TEMP_TIMES},
	{887, 0 * TEMP_TIMES},
	{836, 10 * TEMP_TIMES},
	{778, 20 * TEMP_TIMES},
	{737, 30 * TEMP_TIMES},
	{696, 40 * TEMP_TIMES},
	{655, 50 * TEMP_TIMES},
	{614, 60 * TEMP_TIMES},
	{573, 70 * TEMP_TIMES},
	{532, 80 * TEMP_TIMES},
	{491, 90 * TEMP_TIMES},
	{450, 100 * TEMP_TIMES},
};
/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 Docomo temperature sensor convertor table[END_LGE] */
#endif

/*  ksmin for temp average */

static int average_temp(int temp)
{
#define MAX_ABNORMAL_COUNT 2
	static int abnormal_temp_count = 0;
	static int old_temp = 200;
	int av_temp;

	if((temp > 600) || (temp < 0)) {
		if( abnormal_temp_count < MAX_ABNORMAL_COUNT ) {
			abnormal_temp_count++;
			av_temp = old_temp;
		}
		else {
			av_temp = temp;
		}
	}
	else {
		av_temp = temp;
		old_temp = temp;
		abnormal_temp_count = 0;
	}

	D("temp avg value %d\n",av_temp);

	return 	av_temp;
}

#ifdef GET_TEMP_SENSOR_DOCOMO
int average_temp_therm_ic(int temp, int temp_sensor_index)
{
#define MAX_ABNORMAL_COUNT_THERM_IC 2
	static int abnormal_temp_count_therm_ic[2] = {0, 0};
	static int old_temp_therm_ic[2] = {200, 200};
	int av_temp;

	if (temp_sensor_index < 0 || temp_sensor_index > 1)
		temp_sensor_index = 0;   //charger 
	
	if(temp > 600)
	{
		if( abnormal_temp_count_therm_ic[temp_sensor_index] < MAX_ABNORMAL_COUNT_THERM_IC )
		{
			abnormal_temp_count_therm_ic[temp_sensor_index]++;
			av_temp = old_temp_therm_ic[temp_sensor_index];
		}
		else
		{
			av_temp = temp;
		}
	}
	else
	{
		av_temp = temp;
		old_temp_therm_ic[temp_sensor_index] = temp;
		abnormal_temp_count_therm_ic[temp_sensor_index] = 0;
	}

//	D("temp avg value %d\n",av_temp);

	return 	av_temp;
}
#endif
/* ksmin for temp average */
static int reference_graph(int __x, battery_graph_prop* ref_battery_graph,
		int arraysize)
{
	int i = 1;
	int __y = 0;
	int slope, const_term;
	int delta_y, delta_x;

	D(" battery graph array size = %d", arraysize );

	while( __x < ref_battery_graph[i].x && i < (arraysize - 1) ) {
		i++;
	}

	delta_x = ref_battery_graph[i-1].x - ref_battery_graph[i].x;
	delta_y = (ref_battery_graph[i-1].y - ref_battery_graph[i].y);

	slope = delta_y  / delta_x;

	const_term = (ref_battery_graph[i].y) -
		(ref_battery_graph[i].x * slope);

	__y = (__x* slope + const_term);

//	D(" ####### array_size = %d ##########", arraysize);
//	D(" ##### SLOPE = %d, CONST_TERM = %d ##########", slope, const_term);
//	D(" ##### CALCULATED __y = %d ##########", __y);

	if(ref_battery_graph[i-1].y > ref_battery_graph[i].y) {
		if(__y > ref_battery_graph[i-1].y) {
			__y = ref_battery_graph[i-1].y;
//			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y < ref_battery_graph[i].y) {
			__y = ref_battery_graph[i].y;
//			D(" ##### fixing __y = %d ##########", __y);
		}
	}
	else {
		if(__y < ref_battery_graph[i-1].y) {
			__y = ref_battery_graph[i-1].y;
//			D(" ##### fixing __y = %d ##########", __y);
		}
		else if(__y > ref_battery_graph[i].y) {
			__y = ref_battery_graph[i].y;
//			D(" ##### fixing __y = %d ##########", __y);
		}
	}

	return __y;
}
/* LGE_CHANGE_E [jongho3.lee@lge.com] battery graphs... */

int get_temp(void)
{
	if (!p_di)
		return 0;

	return p_di->temp_C;
}
EXPORT_SYMBOL(get_temp);

void charger_schedule_delayed_work(struct delayed_work *work,
		unsigned long delay)
{
#if defined(CREATE_OWN_WORKQUEUE)
	if (!p_di)
		return;

	queue_delayed_work(p_di->charger_workqueue, work, delay);
#else
	schedule_delayed_work(work, delay);
#endif
}
EXPORT_SYMBOL(charger_schedule_delayed_work);

static void twl6030_work_interval_changed(struct twl6030_bci_device_info *di)
{
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work,
	msecs_to_jiffies(1000 * di->monitoring_interval));
}

// LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-19, Check it this pointer *p_di is redefined..
//struct twl6030_bci_device_info *p_di;

static void twl6030_config_min_vbus_reg(struct twl6030_bci_device_info *di,
						unsigned int value)
{
	u8 rd_reg = 0;
	if (value > 4760 || value < 4200) {
		dev_dbg(di->dev, "invalid min vbus\n");
		return;
	}

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, ANTICOLLAPSE_CTRL2);
	rd_reg = rd_reg & 0x1F;
	rd_reg = rd_reg | (((value - 4200)/80) << BUCK_VTH_SHIFT);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, rd_reg, ANTICOLLAPSE_CTRL2);
}

static void twl6030_config_iterm_reg(struct twl6030_bci_device_info *di,
				     unsigned int term_currentmA)
{
	if ((term_currentmA > 400) || (term_currentmA < 50)) {
		dev_dbg(di->dev, "invalid termination current\n");
		return;
	}

	term_currentmA = ((term_currentmA - 50)/50) << 5;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
						CHARGERUSB_CTRL2);
}

static void twl6030_config_voreg_reg(struct twl6030_bci_device_info *di,
				     unsigned int voltagemV)
{
	if (voltagemV < 3500 || voltagemV > 4760) {
		dev_dbg(di->dev, "invalid charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_VOREG);
}

#if defined(TWL6030_PMIC_CHARGING)
// we use exteranl charger.
static void twl6030_config_vichrg_reg(struct twl6030_bci_device_info *di,
				      unsigned int currentmA)
{
	if (currentmA >= 300 && currentmA <= 450)
		currentmA = (currentmA - 300) / 50;
	else if (currentmA >= 500 && currentmA <= 1500)
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid charger_currentmA\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_VICHRG);
}
#endif

static void twl6030_config_cinlimit_reg(struct twl6030_bci_device_info *di,
					unsigned int currentmA)
{
	if (currentmA >= 50 && currentmA <= 750)
		currentmA = (currentmA - 50) / 50;
	else if (currentmA >= 750)
		currentmA = (800 - 50) / 50;
	else {
		dev_dbg(di->dev, "invalid input current limit\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
					CHARGERUSB_CINLIMIT);
}

static void twl6030_config_limit1_reg(struct twl6030_bci_device_info *di,
							unsigned int voltagemV)
{
	if (voltagemV < 3500 || voltagemV > 4760) {
		dev_dbg(di->dev, "invalid max_charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_CTRLLIMIT1);
}

static void twl6030_config_limit2_reg(struct twl6030_bci_device_info *di,
							unsigned int currentmA)
{
	if (currentmA >= 300 && currentmA <= 450)
		currentmA = (currentmA - 300) / 50;
	else if (currentmA >= 500 && currentmA <= 1500)
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid max_charger_currentmA\n");
		return;
	}

	currentmA |= LOCK_LIMIT;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_CTRLLIMIT2);
}

/* LGE_CHANGE [jongho3.lee@lge.com]
 * Since we are using external charger and fuel gauge,
 * we have to disable PMIC charger and gauge.
 * These functions will control external charger and it's logic.
 */
//LGE_CHANGE [jongho3.lee@lge.com]

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-11-22,
 * Add battery present for Factory mode [START_LGE]
 */
int get_bat_present(void)
{
	if (p_di == NULL)
		return 0;

	return p_di->battery_present;
}
EXPORT_SYMBOL(get_bat_present);
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-11-22,
 * Add battery present for Factory mode [END_LGE]
 */

int get_bat_soc(void)
{
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	// kibum.lee@lge.com
	if (p_di == NULL)
		return 0;

	return p_di->capacity;
}
EXPORT_SYMBOL(get_bat_soc);

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
 * [P940] Imported from Cosmopolitan
 */
int get_bat_volt(void)
{
	// kibum.lee@lge.com
	if (p_di == NULL)
		return 0;

	return p_di->fg_voltage_mV;
}
EXPORT_SYMBOL(get_bat_volt);

struct delayed_work* get_charger_work(void)
{
	if (!p_di)
		return NULL;

	return &(p_di->charger_work);
}
EXPORT_SYMBOL(get_charger_work);

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-10-28,
 * Change charging temperature value for LG senario [START_LGE]
 */
static recharging_state_t
recharging_wait_temperature_state = RECHARGING_WAIT_UNSET;

/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-12-06,
 * To change the charging mode according battery SOC
 */
static recharging_state_t recharging_wait_soc_state = RECHARGING_WAIT_UNSET;

int is_recharging_temperature(int temp)
{
	// Check recharging flag
	if((temp >= TEMP_LOW_RECHARGING &&
	    temp <= TEMP_HIGH_RECHARGING) ||
	    p_di->temp_control == UNLIMITED_TEMP_VAL){
		// clear flag
		recharging_wait_temperature_state = RECHARGING_WAIT_UNSET;
		DCHG("Clear recharging wait temperature flag");
		return true;
	}
	else
		return false;
}	

/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-06,
 * To change the charging mode according battery SOC
 */
int is_recharging_soc(int capacity)
{
	// Check recharging flag
	if (capacity <= HIGH_SOC_RECHARGING ||
	    p_di->temp_control == UNLIMITED_TEMP_VAL) {
		// clear flag
		recharging_wait_soc_state = RECHARGING_WAIT_UNSET;
		DCHG("Clear recharging wait capacity flag");
		return true;
	}
	else
		return false;
}	
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-06,
 * To change the charging mode according battery SOC
 */

#if 0	
int is_tbat_good(int temp)
{
	if( (temp > (TEMP_LOW_DISCHARGING) &&
	     temp < (TEMP_HIGH_DISCHARGING)) ||
	     temp <= (TEMP_LOW_NO_BAT) ||
	     p_di->temp_control == UNLIMITED_TEMP_VAL ) {
		//DCHG("is_tbat_good() is True !!");
		return true;
	}
	DCHG("is_tbat_good() is False !!");
	return false;
}
#else
int is_tbat_good(int temp)
{
	if ((temp > (TEMP_LOW_DISCHARGING) &&
	     temp < (TEMP_HIGH_DISCHARGING)) ||
	    temp <= (TEMP_LOW_NO_BAT) ||
	    p_di->temp_control == UNLIMITED_TEMP_VAL ) {
		if (recharging_wait_temperature_state &&
		   !is_recharging_temperature(temp) &&
		   !(p_di->temp_control == UNLIMITED_TEMP_VAL)) {
			DCHG("Wait for appropriate recharging temperature");
			return false;
		}
		/* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-11-24,
		 * add discharging condition, over 45 degree and over 4.0V
		 */
#if defined(CONFIG_MACH_LGE_P2_SU540) || defined(CONFIG_MACH_LGE_P2_KU5400) || defined(CONFIG_MACH_LGE_P2_LU5400) || defined(CONFIG_MACH_LGE_COSMO)	
		else if (temp > TEMP_CHANGE_CHARGING_MODE &&
			 p_di->fg_voltage_mV > 4000 &&
			 p_di->temp_control != UNLIMITED_TEMP_VAL) {
//			recharging_wait_temperature_state =
//				RECHARGING_WAIT_SET; // set flag
			DCHG("Set recharging wait temperature flag : temp_C[%d], fg_v[%d]", temp, p_di->fg_voltage_mV);			
			return false;
		}
#endif
		/* LGE_CHANGE_E [wonhui.lee@lge.com] 2011-11-24,
		 * add discharging condition, over 45 degree and over 4.0V
		 */

		//DCHG("is_tbat_good() is True !!");
		return true;
	}
	
	if( temp > TEMP_LOW_NO_BAT &&
	    p_di->temp_control != UNLIMITED_TEMP_VAL) {
		recharging_wait_temperature_state = RECHARGING_WAIT_SET; // set flag
		DCHG("Set recharging wait temperature flag : temp_C[%d]", temp);
	}
	
	DCHG("is_tbat_good() is False !!");
	return false;
}
#endif
/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-10-28,
 * Change charging temperature value for LG senario [END_LGE]
 */

/* LGE_CHANGE [jongho3.lee@lge.com] charging finite state machine */


/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21,
 * temporary muic client operation function
 */
#if defined(CONFIG_MUIC)
int charger_fsm2(struct muic_client_device *mcdev)
{
	charger_fsm(CHARG_FSM_CAUSE_ANY);

	return 0;
}
EXPORT_SYMBOL(charger_fsm2);
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] */
#if defined(CONFIG_MAX8971_CHARGER)

bool dcm_temp_sensor_status(int temp_h, int temp_l)
{
	int temp = 0;
#if 1
	if (temp_h < 0 || temp_l < 0)
		temp = p_di->temp_C;
	else
		temp = temp_h;
#else
	if(((temp_h + temp_l)/2) > 0)
		temp = temp_h;
	else
		temp = temp_h; //temp = temp_l;
#endif
	printk("[dcm_temp_sensor_status] temp : %d, temp_h:  %d, temp_l: %d \n",temp , temp_h , temp_l);

	// TEMP_LOW_DISCHARGING = -10, TEMP_HIGH_DISCHARGING = 50 , TEMP_LOW_NO_BAT= -30,
	if(( temp >= (TEMP_LOW_DISCHARGING) && temp <= (TEMP_HIGH_DISCHARGING))\
		|| (p_di->temp_C <= (TEMP_LOW_NO_BAT)) || (p_di->temp_control == UNLIMITED_TEMP_VAL ) ) 
	{
		if(recharging_wait_temperature_state == RECHARGING_WAIT_SET){
			if(recharging_wait_temperature_state && !is_recharging_temperature( temp ) ) {
                       		printk("\nWait for appropriate recharging temperature \n");
                       		return false;
			}
		}
               printk("\n appropriate recharging temperature \n");
//	       temp_charging_stop_flag = 0;
	       return true;
		
	}
	if( !( temp <= TEMP_LOW_NO_BAT) && !(p_di->temp_control == UNLIMITED_TEMP_VAL ) ) {
                recharging_wait_temperature_state = RECHARGING_WAIT_SET; // set flag
                printk("\nSet recharging wait temperature flag\n");
        }
	DCHG("dcm_temp_sensor_status() is false\n");
	return false;
}
#endif

/*mo2seongjae.jang@lge.com 20120813*/
/*issue:
복합 MST TEST issue: AP_USB retaion 설정 후, Idle에서 
"휴대폰 온도가 너무 낮습니다.중전을 일시 중지 합니다. popup 발생 이슈
*/
#if defined(CONFIG_MUIC_MAX14526)
extern int su760_factory_cable_detect;
extern int get_muic_retain_mode(void);
#endif
/*mo2seongjae.jang@lge.com 20120813*/

void charger_fsm(charger_fsm_cause fsm_cause)
{
	TYPE_MUIC_MODE charging_mode = MUIC_UNKNOWN;
	//u8 chargerusb_int_status;
	u8 controller_status_1 = 0;
	int ret, old_charge_status, old_valid_charging_source;

	if (!p_di)
		return;

	//int timeout;
	old_charge_status = p_di->charge_status;
	old_valid_charging_source = p_di->valid_charging_source;
#if defined(CONFIG_MAX8971_CHARGER)

	u8 uninitialized_var(val);
	int err;
	int i=0;
#endif
	DCHG("##### CHARGING FSM ##########");
	
	if (fsm_cause == CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED) {
		DCHG("charging timer expired!!!");
	}
#if 0
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &chargerusb_int_status,
		CHARGERUSB_INT_STATUS);

	//[jongho3.lee@lge.com] vbus detection from charger.
	D(" di->charger_source = %d", p_di->charger_source);
	//D(" vbus = %d", present_charge_state  & CHARGERUSB_STAT);
#endif

	mutex_lock(&charging_fsm_lock);

	/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while power on/off error [START_LGE]
	 */
	if (charger_fsm_shutdown) {
		DCHG("Charger fsm will be shutdown!!");
		mutex_unlock(&charging_fsm_lock);
		return;
	}

	/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while power on/off error [END_LGE]
	 */
	
	/* [jongho3.lee@lge.com]
	 * make it sure that muic examine the charger type.
	 */

	/* this is for wait event which is not unsing just now.
	 * block it temporary..
	 */
#if 0
	if(p_di->charger_interrupt ) {
		/* [jongho3.lee@lge.com]
		 * make it sure that muic examine the charger type.
		 */
		/* jongho3.lee@lge.com] in kernel muic doesn't work now.. */
		D("START : wait muic event...");
		p_di->charger_interrupt = false;
		/* [jongho3.lee] wait event stops scheculing...
		 * don't use this and find the reason..
		 */
		//wait_event_interruptible_timeout(muic_event, muic_get_charger_detected(), HZ*5);
		//wait_event(muic_event, muic_get_charger_detected());
		//wait_event_timeout
		D("wait_event timeout = %d", timeout);
		//wait_event_timeout(muic_event, muic_charger_detected.counter == 1, 2*HZ );
		//msleep_interruptible(1000);
		D("END : wait muic event...");
		//muic_get_charger_detected();
	}
#endif
	
//#if defined(CONFIG_MUIC_MAX14526)	
//       charging_mode = get_muic_mode();
//#else
	charging_mode = muic_get_mode();
//#endif

	DCHG("charging_mode : %d", charging_mode);


	if (charging_mode == MUIC_NA_TA ||
	    charging_mode == MUIC_LG_TA ||
	    charging_mode == MUIC_TA_1A) {
		/* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-11-23,
		 * To change the charging mode according battery temperature
		 */
#if defined(FLEX_CHARGING_MODE)

		/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-06,
		 * To change the charging mode according battery SOC
		 */
#if defined(CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)	
		if (p_di->lock_on_overheat_scene == 1) {
			DCHG("Check SOC = %d", p_di->capacity  );
			if (/*p_di->capacity > LOW_SOC_CHANGE_CHG_MODE &&*/
			    p_di->capacity < HIGH_SOC_CHANGE_CHG_MODE &&
			    (!recharging_wait_soc_state ||
			     is_recharging_soc(p_di->capacity)) &&
			    p_di->temp_control != UNLIMITED_TEMP_VAL) {
				DCHG("Change charging mode to USB");
#if defined(CONFIG_MAX8971_CHARGER)
				DCHG("start charing in video mode 250mA 1");

				max8971_start_charging(250);
#endif
				p_di->charger_source = POWER_SUPPLY_TYPE_USB;
				recharging_wait_soc_state = RECHARGING_WAIT_UNSET; // clear flag
			}
			else if (p_di->capacity >= HIGH_SOC_CHANGE_CHG_MODE &&
				 p_di->temp_control != UNLIMITED_TEMP_VAL) {
				DCHG("Change charging mode to BATTERY");
#if defined(CONFIG_MAX8971_CHARGER)
				DCHG("stop charing in video mode ");

				max8971_stop_charging();
#endif
				p_di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
				recharging_wait_soc_state = RECHARGING_WAIT_SET; // set flag
			}
			else if (p_di->temp_C > TEMP_CHANGE_CHARGING_MODE &&
				 p_di->temp_control != UNLIMITED_TEMP_VAL ) {
				DCHG("Change charging mode to USB cause of high temp.(over 45 degree)");
#if defined(CONFIG_MAX8971_CHARGER)
				DCHG("start charing in video mode 250mA 2");

				max8971_start_charging(250);
#endif
				p_di->charger_source = POWER_SUPPLY_TYPE_USB;
				recharging_wait_soc_state = RECHARGING_WAIT_UNSET; // clear flag
			}
			else {
				DCHG("Change charging mode to TA");
#if defined(CONFIG_MAX8971_CHARGER)
				max8971_start_charging(900);
#endif
				p_di->charger_source = POWER_SUPPLY_TYPE_MAINS;
				recharging_wait_soc_state = RECHARGING_WAIT_UNSET; // clear flag
			}
		}
		else {
		#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
			if (reinit_ta_charger)
				reinit_ta_charger = 0;
		#endif
		/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-07,
		 * To change the charging mode according battery SOC
		 */
			
			if (p_di->temp_C > TEMP_CHANGE_CHARGING_MODE &&
			    p_di->temp_control != UNLIMITED_TEMP_VAL )
				p_di->charger_source = POWER_SUPPLY_TYPE_USB;
			else
				p_di->charger_source = POWER_SUPPLY_TYPE_MAINS;
	        }
#else /* !CHANGE_CHG_MODE_ON_OVERHEAT_SCENE */
		/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-06,
		 * To change the charging mode according battery SOC
		 */
		if (p_di->temp_C > TEMP_CHANGE_CHARGING_MODE &&
		    p_di->temp_control != UNLIMITED_TEMP_VAL )
			p_di->charger_source = POWER_SUPPLY_TYPE_USB;
		else
			p_di->charger_source = POWER_SUPPLY_TYPE_MAINS;
#endif /* CHANGE_CHG_MODE_ON_OVERHEAT_SCENE */
#else
		p_di->charger_source = POWER_SUPPLY_TYPE_MAINS;
#endif /* FLEX_CHARGING_MODE */
#if defined(CHARGING_WAKE_LOCK)
		if(!p_di->wake_lock_count) {
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}
#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-09-01, for MHL charging*/
	else if (charging_mode == MUIC_AP_USB || charging_mode == MUIC_MHL || charging_mode == CHARGING_USB)
#else
	else if (charging_mode == MUIC_AP_USB)
#endif
	{
#if defined(CONFIG_MUIC_MAX14526)/*mo2seongjae.jang@lge.com 20120813*/
		if(su760_factory_cable_detect && (get_muic_retain_mode() ==BOOT_AP_USB))
			p_di->charger_source = POWER_SUPPLY_TYPE_FACTORY;
		else
#endif
		p_di->charger_source = POWER_SUPPLY_TYPE_USB;

#if defined(CHARGING_WAKE_LOCK)
		if (!p_di->wake_lock_count) {
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}
	else if (charging_mode == MUIC_UNKNOWN ||
		 charging_mode == MUIC_NONE) {
		p_di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
#if defined(CHARGING_WAKE_LOCK)
		if((p_di->wake_lock_count)) {
			/* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-11,
			 * From cosmo.
			 */
			//wake_lock_timeout(&(p_di->charger_timed_wake_lock), 2*HZ);
			wake_unlock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 0;
		}
#endif
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-15,
		 * To prevent sending uevent in charging test
		 */
		//charging_test_on = 0;
	}
	else if (charging_mode == MUIC_UNINITED) {
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-09 */
		DCHG("muic isn't initialized");
		goto UPDATE_UI;
	}
	else if (charging_test_on == 1) {
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-09,
		 * To prevent changing charging mode in charging test
		 */
		DCHG("In chargingTest mode is on");
		goto UPDATE_UI;
	}
#if !defined(CONFIG_MAX8971_CHARGER)
	else {
#else
	else if( charging_mode == MUIC_CP_USB || charging_mode == MUIC_CP_UART) {
#endif
		p_di->charger_source = POWER_SUPPLY_TYPE_FACTORY;
#if defined(CHARGING_WAKE_LOCK)
		if (!p_di->wake_lock_count) {
			wake_lock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 1;
		}
#endif
	}

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_status_1,
			CONTROLLER_STAT1);

	if (ret) {
#if !defined(CONFIG_MAX8971_CHARGER)
		D("[charger_rt9524]:: fail to read  CONTROLLER_STAT1 in %s !! \n", __func__);
		DCHG("charger deactive check point 1 \n"); 
		charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 1 \n");
		max8971_stop_charging();
#endif
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		goto UPDATE_UI;
	}

	p_di->valid_charging_source = controller_status_1 &
				      CONTROLLER_STAT1_VBUS_DET;
	D("[charger_rt9524]:: valid charger_source=%d !! \n",
			p_di->valid_charging_source);

	/* [jongho3.lee@lge.com] CHECKGIN BATTERY PRESENCE  start. */

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
	if (p_di->factory_control == FACTORY_CHARGER_DISABLE) {
#if !defined(CONFIG_MAX8971_CHARGER)
		DCHG("charger deactive check point 1-1  \n");		
		charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 2\n");
		max8971_stop_charging();
#endif
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		goto UPDATE_UI;
	}
#endif

	if (!p_di->battery_present) {
		recharging_status = RECHARGING_WAIT_UNSET;
		D("[charger_rt9524]:: NO BATTERY charger_logo_status=%d, charger_source=%d !! \n", p_di->charger_logo_status, p_di->charger_source);
		if (p_di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
			if (p_di->charger_logo_status != CHARGER_LOGO_STATUS_STARTED) {
				//FIXME: I need normal turn off process.
				/* workaroud - CHARGER_LOGO_STATUS_END is no longer necessary. */
#if !defined(CONFIG_MAX8971_CHARGER)
				DCHG("charger deactive check point 2 \n");
				charging_ic_deactive();
#else
//	D("[twl6030] max8971_stop_charging 3 \n");
		                //max8971_stop_charging();

//	D("[twl6030] max8971_stop_charging 500mA \n");
//		max8971_start_charging(500); 
	
	val = SW_RESET | APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;
	printk("[twl6030] power off due to No battery\n");
	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,TWL6030_PHONIX_DEV_ON);	
	if(err)
	{
		printk("[twl6030] Retry due to i2c error,  power off due to No battery\n");
		err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,TWL6030_PHONIX_DEV_ON);
	}
	if(err)
	{	
		D("[twl6030] Retry due to i2c error, max8971_start_charging 500mA for power off\n");
		max8971_start_charging(500);
	}
		

	
#endif
				p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			else if (p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED) {
				if (p_di->valid_charging_source) {
#if defined(POWER_OFF_WHEN_NO_BAT)
					charger_schedule_delayed_work(&p_di->power_off_work, HZ*2);
#else
					charger_schedule_delayed_work(&p_di->power_off_work, 0);
#endif
				}
			}
			goto UPDATE_UI;
		}
	}
	/* [jongho3.lee@lge.com] CHECKGIN BATTERY PRESENCE  end. */

	/* [jongho3.lee@lge.com] DISCHARGING CONDITION CHECK start. */

	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-22,
	 * To prevent reset in Factory
	 */
	if (!p_di->valid_charging_source &&
	    p_di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
		/* we need check over voltage,
		 * battery temperature condition here
		 */
#if !defined(CONFIG_MAX8971_CHARGER)	
		DCHG("charger deactive check point 3 \n");
		charging_ic_deactive();
#else
//	D("[twl6030] max8971_stop_charging 4 \n");
       //        max8971_stop_charging();
#endif
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		p_di->charger_source = POWER_SUPPLY_TYPE_BATTERY;

#if defined(CHARGING_WAKE_LOCK)
		if (p_di->wake_lock_count) {
			wake_unlock(&(p_di->charger_wake_lock));
			p_di->wake_lock_count = 0;
		}
#endif

		goto UPDATE_UI;
	}

	// recharging status check!!
//	DCHG("Recharging Status Check!");
//nthyunjin.yang 120426
//	DCHG("ui_cap = %d, di_cap = %d, fg_voltage_mv = %d, charge_status = %d, charger_source=%d.", 
//		p_di->ui_capacity, p_di->capacity, p_di->fg_voltage_mV, p_di->charge_status, p_di->charger_source);
//	DCHG("capacity=%d, charger_source=%d", p_di->capacity, p_di->charger_source);
	if (p_di->capacity > 99 &&
	   p_di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
		if (p_di->fg_voltage_mV > RECHARGING_BAT_VOLT_HIGH) {
			/* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-07,
			 * From cosmo. cause discharging
			 * while Not complete charging process.
			 */

#if !defined(CONFIG_MAX8971_CHARGER)
			DCHG("charger deactive check point 4 \n");
			charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 5: battery is FULL \n");
	               max8971_stop_charging();
#endif
			if (p_di->valid_charging_source) {
				recharging_status = RECHARGING_WAIT_SET;
			}
			else {
				recharging_status = RECHARGING_WAIT_UNSET;
			}
			p_di->charge_status = POWER_SUPPLY_STATUS_FULL;
		}
		else if (p_di->fg_voltage_mV < RECHARGING_BAT_VOLT_LOW) {
			/* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-16,
			 * Added condition
			 */
#if defined(CONFIG_MACH_LGE_U2)

			if(battery_full_status == 1)
			{
			printk(" Recharging for U2 , voltage: %d\n", p_di->fg_voltage_mV);
			max8971_start_charging(850);
			}
#endif
			goto START_RECHARGING;
		}

		if (p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED) {
			if (p_di->off_mode_timer_working) {
				del_timer(&(p_di->off_mode_timer));
				p_di->off_mode_timer_working = 0;
			}
		}

		goto UPDATE_UI;
	}
	else if (p_di->charge_status == POWER_SUPPLY_STATUS_FULL) {
		/* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-13,
		 * Added condition.
		 */
		if (!p_di->valid_charging_source)
			p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	//DCHG("charge_status=%d", p_di->charge_status);

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
 * [P940] Imported from Cosmopolitan
 */
#if 0 //#if defined(LOW_TEMPERATURE_LIMITED_CHARGING)
	if (!is_tbat_good(p_di->temp_C) &&
	    p_di->charger_source != POWER_SUPPLY_TYPE_BATTERY) {
		if (p_di->temp_C < TEMP_LOW_DISCHARGING) {
			if (p_di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING &&
			    p_di->fg_voltage_mV < OVER_RANGE_TEMP_CHARGING_VOLT_LOW) {
				goto CHARGING_START;
			}
			else if (p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING &&
				 p_di->fg_voltage_mV < OVER_RANGE_TEMP_CHARGING_VOLT_HIGH) {
				goto UPDATE_UI;
			}
		}

	}
#endif

	/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-08-01,
	 * Add for Factory mode [START_LGE]
	 */
	if(get_charging_ic_status() == POWER_SUPPLY_TYPE_FACTORY)
		p_di->charger_source = POWER_SUPPLY_TYPE_FACTORY;
#if !defined(CONFIG_MAX8971_CHARGER)

	if ((p_di->charger_source == POWER_SUPPLY_TYPE_BATTERY ||
	    !is_tbat_good(p_di->temp_C))  &&
	    p_di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
#else 
 	if(  ((p_di->charger_source == POWER_SUPPLY_TYPE_BATTERY) || (!dcm_temp_sensor_status(p_di->temp_high_C, p_di->temp_low_C)))  && (p_di->charger_source != POWER_SUPPLY_TYPE_FACTORY) ){ /* charging deactive condition */
#endif
		/* charging deactive condition */
		/* we need check over voltage,
		 * battery temperature condition here
		 */
#if !defined(CONFIG_MAX8971_CHARGER)
		D("[charger_rt9524]:: bat_temp=%d !! \n", p_di->temp_C);
//		DCHG("charger source = %d\n", p_di->charger_source);
//		DCHG("p_di->temp_C = %d\n", p_di->temp_C);
		DCHG("charger deactive check point 5 \n");
		charging_ic_deactive();
#else
		DCHG("[twl6030]  max8971_stop_charging due to temp \n");
//	        max8971_stop_charging();
		//temp_charging_stop_flag = 1;
//		DCHG("[twl6030] temp_charging_stop_flag:%d\n",temp_charging_stop_flag);
#endif
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		goto UPDATE_UI;
	}
	
	if (fsm_cause == CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED) {
		D("charging stop since...timer expired!!!");
#if !defined(CONFIG_MAX8971_CHARGER)
		DCHG("charger deactive check point 6 \n");
		charging_ic_deactive();
#else
		DCHG("[twl6030] max8971_stop_charging 7 \n");
	        max8971_stop_charging();
#endif		
		recharging_status = RECHARGING_WAIT_UNSET;
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

		goto UPDATE_UI;
	}

	/* [jongho3.lee@lge.com] DISCHARGING CONDITION CHECK end. */

START_RECHARGING:

	/* [jongho3.lee@lge.com]
	 * FIXME : recharging start soc should be modified with H/W team.
	 */
	if (recharging_status == RECHARGING_WAIT_SET &&
	    p_di->capacity > RECHARGING_BAT_SOC_CON &&
	    p_di->fg_voltage_mV > RECHARGING_BAT_VOLT_LOW) {
		goto UPDATE_UI;
	}

	recharging_status = RECHARGING_WAIT_UNSET;

	//DCHG("charger setting is %d\n", p_di->charger_source );

	if (p_di->charger_source == POWER_SUPPLY_TYPE_MAINS) {
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_set_ta_mode();
#else
//		max8971_start_charging(900); //[dukwung.kim@lge.com] FIXME: charger ic for max8971
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if (p_di->charger_source == POWER_SUPPLY_TYPE_USB) {
//	D("[twl6030] start charing USB\n");
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_active_default();
#else
//		max8971_start_charging(500);  //[dukwung.kim@lge.com] FIXME: charger ic for max8971
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else if (p_di->charger_source == POWER_SUPPLY_TYPE_FACTORY) {
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_set_factory_mode();
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	else {
#if !defined(CONFIG_MAX8971_CHARGER)
		DCHG("charger deactive check point 7 \n");
		charging_ic_deactive();
#else
//	D("[twl6030] max8971_stop_charging  8\n");

//	        max8971_stop_charging();
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

UPDATE_UI:
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-15,
	 * To prevent sending uevent in charging test
	 */
	if (charging_test_on == 0)
		power_supply_changed(&p_di->bat);

	if (old_charge_status != p_di->charge_status) {
		//DCHG("old_charger_status=%d, charge_status=%d \n", old_charge_status, p_di->charge_status);
		/* charger is changed..  gaterh voltage data again. */
		p_di->avg_volt_head = move_queue_point(p_di->avg_volt_head,
				AVG_VOLT_QUEUE_SIZE,
				AVG_VOLT_QUEUE_SIZE,
				p_di->avg_volt_tail);
		p_di->avg_volt_queue_count = 0;

		if (p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
			p_di->monitoring_interval = 2;   //sec
			twl6030_work_interval_changed(p_di);
		}
		else {
			p_di->monitoring_interval = 10;    //sec
			twl6030_work_interval_changed(p_di);
		}
	}

#if 0
	if (!p_di->valid_charging_source && old_valid_charging_source) {
		wake_lock_timeout(&p_di->charger_back_light_wake_lock, HZ*3);
	}
#endif

	mutex_unlock(&charging_fsm_lock);
}
EXPORT_SYMBOL(charger_fsm);

/*
 * Interrupt service routine
 *
 * Attends to TWL 6030 power module interruptions events, specifically
 * USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 *
 */
static irqreturn_t twl6030charger_ctrl_interrupt(int irq, void *_di)
{
	struct twl6030_bci_device_info *di = _di;
	int ret;
	int charger_fault = 0;

	u8 stat_toggle, stat_reset, stat_set = 0;
	u8 charge_state;
	u8 present_charge_state,chargerusb_int_status;

	/* read charger controller_stat1 */
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &present_charge_state,
		CONTROLLER_STAT1);
	if (ret)
		return IRQ_NONE;

	/* [jongho3.lee@lge.com] vbus detection from charger. */

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &chargerusb_int_status,
		CHARGERUSB_INT_STATUS);
#if 1
	if ((di->charger_source && !(present_charge_state & VBUS_DET)) ||
	    (!di->charger_source && (present_charge_state & VBUS_DET)))
#else
	if (chargerusb_int_status & CHARGERUSB_STAT)
#endif
	{
		di->charger_interrupt = true;
	}
	else {
		di->charger_interrupt = false;
	}

	charge_state = di->stat1;

	stat_toggle = charge_state ^ present_charge_state;
	stat_set = stat_toggle & present_charge_state;
	stat_reset = stat_toggle & charge_state;


	if (stat_set & CONTROLLER_STAT1_FAULT_WDG) {
		charger_fault = 1;
		dev_dbg(di->dev, "Fault watchdog fired\n");
	}
	if (stat_reset & CONTROLLER_STAT1_FAULT_WDG)
		dev_dbg(di->dev, "Fault watchdog recovered\n");
	if (stat_set & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery removed\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery inserted\n");
	if (stat_set & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature overrange\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature within range\n");


/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-09,
 * [P940] Add Charger IC(Max8971) in Rev.A
 */
#if defined(CONFIG_CHARGER_RT9524) || defined(CONFIG_CHARGER_MAX8971) ||  defined(CONFIG_MAX8971_CHARGER)
	if (!di->charger_interrupt) {
		charger_schedule_delayed_work(&di->charger_work, 0);
	}
	//charger_fsm();
#endif

	//power_supply_changed(&di->bat);

	return IRQ_HANDLED;
}

//FIXME  : we need to fix this.

#if !defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE) && !defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
/* LGE_CHANGE [jongho3.lee@lge.com]
 * we don't need and don't have the way to monitor bat_current.
 */
static void twl6030battery_current(struct twl6030_bci_device_info *di)
{
	int ret;
	u16 read_value;
	s16 temp;
	int current_now;

	/* FG_REG_10, 11 is 14 bit signed instantaneous current sample value */
	ret = twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *)&read_value,
								FG_REG_10, 2);
	if (ret < 0) {
		dev_dbg(di->dev, "failed to read FG_REG_10: current_now\n");
		return;
	}

	temp = ((s16)(read_value << 2) >> 2);
	current_now = temp - di->cc_offset;

	/* current drawn per sec */
	current_now = current_now * fuelgauge_rate[di->fuelgauge_mode];
	/* current in mAmperes */
	current_now = (current_now * 3000) >> 14;
	/* current in uAmperes */
	current_now = current_now * 1000;
	di->current_uA = current_now;

	return;
}
#endif

/*
 * Return channel value
 * Or < 0 on failure.
 */
static int twl6030_get_gpadc_conversion(int channel_no)
{
	struct twl6030_gpadc_request req;
	int temp = 0;
	int ret;

	req.channels = (1 << channel_no);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);
	if (ret < 0)
		return ret;

	if (req.rbuf[channel_no] > 0)
		temp = req.rbuf[channel_no];

	return temp;
}

/*
 * Setup the twl6030 BCI module to enable backup
 * battery charging.
 */
static int twl6030backupbatt_setup(void)
{
	int ret = 0;
	u8 rd_reg = 0;

        /*
         * add_by lu.guo@lge.com  we should enable VBAT work at low power mode
         * when we take out  the main battery
         */

	//ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, BBSPOR_CFG);
	rd_reg |= BB_CHG_EN | VRTC_EN_SLP_STS | VRTC_EN_OFF_STS | BB_SEL_2V6;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, BBSPOR_CFG);

	return ret;
}

/*
 * Setup the twl6030 BCI module to measure battery
 * temperature
 */
static int twl6030battery_temp_setup(void)
{
	int ret = 0;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL_MODULE_MADC, &rd_reg, TWL6030_GPADC_CTRL);
	/* LGECHANGE [jongho3.lee@lge.com]
	 * batttery temp is connected to GPADCIN2.
	 */
	rd_reg |= GPADC_CTRL_SCALER_EN | GPADC_CTRL_SCALER_DIV4;
	ret |= twl_i2c_write_u8(TWL_MODULE_MADC, rd_reg, TWL6030_GPADC_CTRL);

	return ret;
}

static int twl6030battery_voltage_setup(void)
{
	int ret = 0;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &rd_reg, REG_MISC1);
	rd_reg = rd_reg | VAC_MEAS | VBAT_MEAS | BB_MEAS;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID0, rd_reg, REG_MISC1);

	ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_VBUS_CTRL_SET);
	rd_reg = rd_reg | VBUS_MEAS;
	ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_VBUS_CTRL_SET);

	ret |= twl_i2c_read_u8(TWL_MODULE_USB, &rd_reg, REG_USB_ID_CTRL_SET);
	rd_reg = rd_reg | ID_MEAS;
	ret |= twl_i2c_write_u8(TWL_MODULE_USB, rd_reg, REG_USB_ID_CTRL_SET);

	return ret;
}

#if defined(TWL6030_PMIC_CHARGING)
//LGE_CHANGE [jongho3.lee@lge.com] we don't need and don't have the way to monitor bat_current.
static int twl6030battery_current_setup(void)
{
	int ret;
	u8 rd_reg = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID1, &rd_reg, PWDNSTATUS2);
	rd_reg = (rd_reg & 0x30) >> 2;
	rd_reg = rd_reg | FGDITHS | FGS;
	ret |= twl_i2c_write_u8(TWL6030_MODULE_ID1, rd_reg, REG_TOGGLE1);
	ret |= twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

	return ret;
}
#endif

static enum power_supply_property twl6030_bci_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo. [START_LGE]
	 */
	POWER_SUPPLY_PROP_PMIC_SOC,
	POWER_SUPPLY_PROP_GAUGE_VOLTAGE,
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	POWER_SUPPLY_PROP_GAUGE_CONTROL,
	POWER_SUPPLY_PROP_GAUGE_CONTROL_COUNT,
#endif
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-16, [P940] */
	POWER_SUPPLY_PROP_CHARGER_MODE,
	POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL,
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * adding android related attributes to battery props. //end.
	 */
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo. [END_LGE]
	 */
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20,
	 * add to check BATT TEMP ADC value
	 */
	POWER_SUPPLY_PROP_BATT_TEMP_ADC,
#if defined(CONFIG_MAX8971_CHARGER)
	POWER_SUPPLY_PROP_BATT_CURRENT_ADC,    /* LGE_CHANGE [dukwung.kim@lge.com] 2011-09-16, add to check BATT CURRENT ADC value*/
	POWER_SUPPLY_PROP_CHARGER_TEMP,		/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger current adc value */
	POWER_SUPPLY_PROP_PARM_TEMP, 		/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger current adc value */
	POWER_SUPPLY_PROP_CHARGER_TEMP_ADC,
	POWER_SUPPLY_PROP_PARM_TEMP_ADC,
	POWER_SUPPLY_PROP_TEMP_HIGH,
	POWER_SUPPLY_PROP_TEMP_LOW,
#endif
};

static enum power_supply_property twl6030_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property twl6030_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

#ifdef FEATURE_BK_BATTERY
static enum power_supply_property twl6030_bk_bci_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};
#endif

#if defined(TWL6030_PMIC_CHARGING)
/* LGE_CHANGE [jongho3.lee@lge.com]
 * we don't need and don't have the way to monitor bat_current.
 */
static void twl6030_current_avg(struct work_struct *work)
{
	s32 samples;
	s16 cc_offset = 0;
	int current_avg_uA;
	struct twl6030_bci_device_info *di = container_of(work,
		struct twl6030_bci_device_info,
		twl6030_current_avg_work.work);

	di->charge_n2 = di->charge_n1;
	di->timer_n2 = di->timer_n1;

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->timer_n1,
							FG_REG_01, 3);
	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->charge_n1,
							FG_REG_04, 4);
	/* FG_REG_08, 09 is 10 bit signed calibration offset value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &cc_offset,
							FG_REG_08, 2);
	cc_offset = ((s16)(cc_offset << 6) >> 6);
	di->cc_offset = cc_offset;

	samples = di->timer_n1 - di->timer_n2;
	/* check for timer overflow */
	if (di->timer_n1 < di->timer_n2)
		samples = samples + (1 << 24);

	cc_offset = cc_offset * samples;
	current_avg_uA = ((di->charge_n1 - di->charge_n2 - cc_offset)
							* 3000) >> 12;
	if (samples)
		current_avg_uA = current_avg_uA / samples;
	di->current_avg_uA = current_avg_uA * 1000;

	charger_schedule_delayed_work(&di->twl6030_current_avg_work,
		msecs_to_jiffies(1000 * di->current_avg_interval));
}
#endif

#if defined(GET_BET_TEMP)
static int get_adc_value(struct twl6030_gpadc_request* req, u16 channels)
{
	int ret;
	req->channels = channels;

	req->method = TWL6030_GPADC_SW2;
	req->active = 0;
	req->func_cb = NULL;
	ret = twl6030_gpadc_conversion(req);

	if (ret < 0) {
		dev_dbg(p_di->dev, "gpadc conversion failed: %d\n", ret);
	}

	return ret;
}
#endif

static void charging_timer_work(struct work_struct *work)
{
	charger_fsm(CHARG_FSM_CAUSE_CHARGING_TIMER_EXPIRED);
}

void off_mode_timer_func(unsigned long try)
{
	del_timer(&(p_di->off_mode_timer));
	p_di->off_mode_timer_working = 0;

	D("\n timer EXPIRED!!!!\n");

	if (p_di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED) {
		charger_schedule_delayed_work(&(p_di->charger_timer_work), 0);
	}
}

int move_queue_point(int point, int move_count, int queue_size, int limit)
{
	int i;

	if (point < 0 || queue_size < 1) {
		return 0;
	}

	for (i = 0 ; i < move_count ; i++) {
		point++;

		if (point >= queue_size) {
			point = 0;
		}

		if (point == limit) {
			return point;
		}
	}

	return point;
}

int get_avg_volt(int volt_now)
{
	int count, sum_of_volt, temp_point;
	static ktime_t old_time = {0};
	static ktime_t interval;
	/* [LGE_SJIT] 2011-11-3 [jongrak.kwon@lge.com]
	 * for remove ktime_t structure dependency 
	 */
	struct timeval time_val;
	int monitoring_gap_count;
	long max_interval;

	count = 0;
	sum_of_volt = 0;
	temp_point = 0;


	/* [jongho3.lee@lge.com] if system has slept,
	 * we should ignore previous voltage data.
	 */
	interval =  ktime_sub(ktime_get(), old_time);

	/* [LGE_SJIT_S] 2011-11-3 [jongrak.kwon@lge.com]
	 * for remove ktime_t structure dependency
	 */
	time_val = ktime_to_timeval(interval); 
	max_interval = p_di->monitoring_interval * AVG_VOLT_QUEUE_SIZE * 20;

	if (time_val.tv_sec > max_interval ||
	    !p_di->battery_present) {
		monitoring_gap_count =
			time_val.tv_sec / p_di->monitoring_interval;
		p_di->avg_volt_head =
			move_queue_point(p_di->avg_volt_head,
					monitoring_gap_count,
					AVG_VOLT_QUEUE_SIZE,
					p_di->avg_volt_tail);
		p_di->avg_volt_queue_count = 0;
	}
	/* [LGE_SJIT_E] 2011-11-3 [jongrak.kwon@lge.com]
	 * for remove ktime_t structure dependency
	 */

	p_di->avg_volt_queue[p_di->avg_volt_tail] = volt_now;
	p_di->avg_volt_tail = move_queue_point(p_di->avg_volt_tail,
			1, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_head);

	if (p_di->avg_volt_queue_count < AVG_VOLT_QUEUE_SIZE) {
		p_di->avg_volt_queue_count++;
	}

	temp_point = p_di->avg_volt_head;

	D("avg_volt_head = %d, avg_volt_tail= %d", p_di->avg_volt_head, p_di->avg_volt_tail);

	do {
		sum_of_volt += p_di->avg_volt_queue[temp_point];
		temp_point = move_queue_point(temp_point,
				1, AVG_VOLT_QUEUE_SIZE, p_di->avg_volt_tail);
		count++;
		D("sum_of_volt = %d, p_di->avg_volt_queue[temp_point] = %d", sum_of_volt, p_di->avg_volt_queue[temp_point]);
	} while (count < p_di->avg_volt_queue_count);

	if (p_di->avg_volt_tail == p_di->avg_volt_head) {
		p_di->avg_volt_head =
			move_queue_point(p_di->avg_volt_head,
					1,
					AVG_VOLT_QUEUE_SIZE,
					p_di->avg_volt_tail);
	}

	D("avg_volt = %d, count = %d", sum_of_volt / count, count);
	D("avg_volt_queue_count = %d", p_di->avg_volt_queue_count);

	old_time = ktime_get();

	return sum_of_volt / count;
}

static int get_pmic_soc(void)
{
	/* this should be 1 to set initial soc. */
	static int battery_replace_toggle = 1;
	int temp_val;

	if (!p_di->battery_present) {
		battery_replace_toggle = 1;
	}

	if (p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING &&
	    p_di->valid_charging_source) {
		if (p_di->charger_source == POWER_SUPPLY_TYPE_USB) {
			temp_val = reference_graph((s64)p_di->avg_voltage_mV,
					battery_soc_graph_with_usb,
					ARRAY_SIZE(battery_soc_graph_with_usb)) / SOC_TIMES;
		}
		else {
			/* ta or factory. */
			temp_val = reference_graph((s64)p_di->avg_voltage_mV,
					battery_soc_graph_with_ac,
					ARRAY_SIZE(battery_soc_graph_with_ac)) / SOC_TIMES;
		}

		/* [jongho3.lee@lge.com]
		 * gather voltage data at least 3 times.
		 */
		if (p_di->avg_volt_queue_count > 3) {
			/* [jongho3.lee@lge.com]
			 * capacity should not drop below
			 * RECHARGING_BAT_SOC_CON while charger is plugged.
			 */
			if (p_di->pmic_capacity >= RECHARGING_BAT_SOC_CON &&
			    p_di->valid_charging_source) {
				if (temp_val < RECHARGING_BAT_SOC_CON) {
					temp_val = RECHARGING_BAT_SOC_CON;
				}
			}

			/* [jongho3.lee@lge.com]
			 * prevent from decreasing capacity while charging.
			 */
			if ((battery_replace_toggle && p_di->battery_present) ||
			    temp_val > p_di->pmic_capacity) {
				p_di->pmic_capacity = temp_val;

				if (battery_replace_toggle) {
					battery_replace_toggle = 0;
				}
			}
		}
		else {
			p_di->pmic_capacity = temp_val;
		}
	}
	else {
		temp_val = reference_graph((s64)p_di->avg_voltage_mV,
				battery_soc_graph_with_no_charger,
				ARRAY_SIZE(battery_soc_graph_with_no_charger)) / SOC_TIMES;

		/* [jongho3.lee@lge.com]
		 * gather voltage data at least 3 times.
		 */
		if (p_di->avg_volt_queue_count > 3) {
			/* [jongho3.lee@lge.com]
			 * capacity should not drop below RECHARGING_BAT_SOC_CON
			 * while charger is plugged.
			 */
			if (p_di->pmic_capacity >= RECHARGING_BAT_SOC_CON &&
			    p_di->valid_charging_source) {
				if (temp_val < RECHARGING_BAT_SOC_CON) {
					temp_val = RECHARGING_BAT_SOC_CON;
				}
			}

			/* [jongho3.lee@lge.com]
			 * prevent from increasing capacity while discharging.
			 */
			if ((battery_replace_toggle && p_di->battery_present) ||
			    temp_val < p_di->pmic_capacity) {
				p_di->pmic_capacity = temp_val;

				if (battery_replace_toggle) {
					battery_replace_toggle = 0;
				}
			}
		}
		else {
			p_di->pmic_capacity = temp_val;
		}
	}

	if (p_di->pmic_capacity > 100) {
		p_di->pmic_capacity = 100;
	}
	else if (p_di->pmic_capacity <= 0) {
		/* keep alive for power board... */
		p_di->pmic_capacity = 1;
	}

	temp_val = p_di->pmic_capacity;

	if (p_di->avg_volt_queue_count < 3) {
		temp_val = (temp_val * (p_di->avg_volt_queue_count + 3) / 6);
	}

	return temp_val;
}
#if defined(CONFIG_MAX8971_CHARGER)
static void twl6030_max8971_recharging_work(struct work_struct *work)
{
//        TYPE_CHARGING_MODE charging_mode;
        TYPE_MUIC_MODE charging_mode = MUIC_UNKNOWN;
	//charging_mode = get_muic_detected_cable();
	charging_mode = muic_get_mode();
//	charging_mode = get_muic_mode();
	DCHG("[twl6030] charging_mode: %d \n, charging_mode");
        struct twl6030_bci_device_info *di = container_of(work,
		        struct twl6030_bci_device_info, twl6030_recharging_work.work);

	if(charging_mode == MUIC_NA_TA || charging_mode == MUIC_LG_TA || charging_mode == MUIC_TA_1A)
	{
		DCHG("[twl6030] max8971_Restart_charging TA \n");
		if(p_di->lock_on_overheat_scene == 1)
		{
		DCHG("[twl6030] max8971_Restart_charging TA 250mA in recording mode\n");
			max8971_start_charging(250);
		}
		else	
		 max8971_start_charging(900);
	
		decease_cap_count_flag=0;
	}
	else if(charging_mode == MUIC_AP_USB )
	{
		 D("[twl6030] max8971_Restart_charging USB \n");
	    	max8971_start_charging(500);
		 decease_cap_count_flag=0;
	}
	else if(charging_mode == MUIC_MHL )
	{
		 D("[twl6030] max8971_Restart_charging MHL \n");
	    	max8971_start_charging(400);
		 decease_cap_count_flag=0;
	}
	else if((charging_mode == MUIC_CP_USB) && (charging_mode == MUIC_CP_UART))
	{
		 D("[twl6030] do noting MUIC_CP_USB MUIC_CP_UART\n");
		decease_cap_count_flag=0;
	}
	else
	{
		D("[twl6030] can not find muic mode , doing default charging current  \n");
		decease_cap_count_flag=0;
	//	max8971_start_charging(500);
	}
}
#endif

static void twl6030_bci_battery_work(struct work_struct *work)
{
	struct twl6030_bci_device_info *di = container_of(work,
			struct twl6030_bci_device_info,
			twl6030_bci_monitor_work.work);
	struct twl6030_gpadc_request req;
	int adc_code;
	int temp_C, temp_gap;
#if defined(CONFIG_MAX8971_CHARGER)
	int charger_adc_code = 0;
	int parm_adc_code = 0;
	int temp_parm_C = 0;
	int temp_charger_C = 0;
	static int saved_temp_parm_C = 0;
	static int saved_temp_charger_C = 0;
	int temp_high_C = 0;
	int temp_low_C = 0;
#endif
	int fsm_work_needed = 0;
	int ui_update_needed = 0;
	static int old_capacity, old_temp = 200;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-27,
	 * [P940] Capacity ui update error fix
	 */
	static int old_ui_capacity = -1;
	int charging_ic_status;
	int ret;//nthyunjin.yang 120724 for unlimited temp charging in hidden menu
	u8 temp_val;//nthyunjin.yang 120724 for unlimited temp charging in hidden menu
#if !defined(GET_BET_TEMP)
	int ret;
#endif
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	u8 controller_status_1;
#if defined(CONFIG_MAX8971_CHARGER)
      //  TYPE_CHARGING_MODE charging_mode = CHARGING_UNKNOWN;
        TYPE_MUIC_MODE charging_mode = MUIC_UNKNOWN;
	//charging_mode = get_muic_detected_cable();
	charging_mode = muic_get_mode();
#endif
	D("check if CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C");
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C)
	D("CONFIG_LG_FW_MAX17043_FUEL_GAUGE_I2C is defined!!!");
#endif

	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work,
			msecs_to_jiffies(1000 * di->monitoring_interval));

	charging_ic_status = get_charging_ic_status();
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)

//	printk("charger_source= %d\n",p_di->charger_source);
	if ((p_di->charger_source != POWER_SUPPLY_TYPE_BATTERY) && (p_di->charger_source !=POWER_SUPPLY_TYPE_FACTORY ))
	{
//		printk(">>>>>>>>>>>>charger_source = %d charge_status= %d>>>>>>>>>>>\n",p_di->charger_source, p_di->charge_status);
		if(battery_full_status == 1)
		//if(battery_full_status == 1||p940_get_lcd_on_off()==0)
//		if(battery_full_status == 1 || get_lcd_value()>0) 
		{
		
			printk("[twl6030]: SYSFS_LED Off!\n");
			//gpio_set_value(HOLD_KEY_LED_EN, 0);
			set_pw_led_on_off(PW_LED_OFF);
		}
		else 
		{
//			printk(KERN_ERR ">>>>>[kwuiseok.kim]: lh430wv4_get_lcd_on_off Value: %d\n", lh430wv4_get_lcd_on_off());
			
#if defined(CONFIG_MAX8971_CHARGER)&&  defined(CONFIG_MACH_LGE_P2_DCM)
				if(lm3530_get_lcd_on_off()==0)
#endif
			{
				printk("[twl6030]: SYSFS_LED ON!LCD BACKLIGHT OFF\n");
				//gpio_set_value(HOLD_KEY_LED_EN, 1);
				set_pw_led_on_off(PW_LED_ON);
			}
		}
	}
#endif
// LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13, LGE_P940, Bring in Cosmo. [START_LGE]
#if 1
	if(di->temp_control == 0xFF)
	{
		ret = lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val , 1);
		D("@@@@@@@@@@@@@@@E_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET= %x", di->temp_control);
		if(ret == 1)
		{
			di->temp_control = temp_val;
		}
	}
#endif
// LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13, LGE_P940, Bring in Cosmo. [END_LGE]

//	printk("Battery current adc  = %d\n", current_adc_code);
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
	if (get_fg_enable()) {
		di->capacity = max17043_get_capacity();
		di->fg_voltage_mV = max17043_get_voltage();
#if defined(CONFIG_MAX8971_CHARGER)
		// LGE__CHANGE_S [dukwung.kim@lge.com] 2011.07.28 recharging , when max8971 (DCM charger IC) recharging mode  is not working
		if((decease_cap_count==4) || ((current_adc_code > 60) && (current_adc_code < 95)))
		{
			DCHG("[twl6030] decreasing battery soc  \n");
//			DCHG("[twl6030] temp_charging_stop_flag:%d\n",temp_charging_stop_flag);
	       //         printk("Battery current adc  = %d\n", di->batt_current_adc);
		//	if ((charging_mode == MUIC_NA_TA || charging_mode == MUIC_LG_TA || charging_mode == MUIC_TA_1A ||  charging_mode == MUIC_AP_USB || charging_mode == MUIC_MHL) && (decease_cap_count_flag==0)&& (temp_charging_stop_flag==0))
			if ((charging_mode == MUIC_NA_TA || charging_mode == MUIC_LG_TA || charging_mode == MUIC_TA_1A ||  charging_mode == MUIC_AP_USB || charging_mode == MUIC_MHL) && (decease_cap_count_flag==0))
			{
				DCHG("[twl6030] max8971_stop_charging for recharging  \n");
				max8971_stop_charging();
				decease_cap_count_flag=1;
				decease_cap_count=0;
				schedule_delayed_work(&di->twl6030_recharging_work,MAX8971_RECHARGING__DELAY); 
			}

		}

              
		// LGE__CHANGE_E [dukwung.kim@lge.com] 2011.07.28 recharging , when max8971 (DCM charger IC) recharging mode  is not working

#endif
#if 1
#if defined(CONFIG_MAX8971_CHARGER)
// LGE__CHANGE_S [dukwung.kim@lge.com] 2011-10-13 ui soc 100% when charger is inserted and battery is full
                if(di->ui_capacity == 100)
                {
                        battery_full_status = 1;
                }
                else
                {
                        battery_full_status = 0;
                }
                if(battery_full_status==1 && (p_di->charger_source != POWER_SUPPLY_TYPE_BATTERY))
                {
                        di->ui_capacity=100;
                }
                else
                {
                        di->ui_capacity = (validate_gauge_value(max17043_get_ui_capacity()));
                }

		if((p_di->charger_source == POWER_SUPPLY_TYPE_FACTORY)&& (di->ui_capacity < 50)) 
		{
			printk("[twl6030] ui_capacity =95 in POWER_SUPPLY_TYPE_FACTORY mode\n");
			di->ui_capacity =95;
		}
#endif
		di->ui_capacity = validate_gauge_value(max17043_get_ui_capacity());
		//printk("\n cap:%d, ui_cap:%d%, avg_v:%dmv, fg_v:%dmv\n", di->capacity, di->ui_capacity, di->avg_voltage_mV, di->fg_voltage_mV);
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-12-12
		 * [P2] Add a definition of shutdown voltage and
		 * modify a shutdown condition
		 */
		//printk("[PWR] UI CAP :: %d chg_test_on :: %d\n",di->ui_capacity,charging_test_on);
		if(di->charger_source==POWER_SUPPLY_TYPE_FACTORY && !charging_test_on) {
			di->ui_capacity = 100;
		}
		if (di->fg_voltage_mV <= BATT_VOLT_SHUTDOWN) {
			//printk("[TWL6030]cap:%d, ui_cap:%d, volt:%dmv<=%dmv(shutdown voltage)\n", di->capacity, di->ui_capacity, di->fg_voltage_mV, BATT_VOLT_SHUTDOWN);
			if (di->ui_capacity > 0) {
				printk("[TWL6030]cap:%d, ui_cap:%d, volt:%dmv<=%dmv, but don't shutdown!!\n", di->capacity, di->ui_capacity, di->fg_voltage_mV, BATT_VOLT_SHUTDOWN);
			}
		}
		else {
			if (di->ui_capacity == 0) {
				DPWR("cap:%d, ui_cap:0 && volt:%dmv>%dmv => ui_cap=1", di->capacity, di->fg_voltage_mV, BATT_VOLT_SHUTDOWN);
				di->ui_capacity = 1;
			}
		}
#if defined(CONFIG_MAX8971_CHARGER)
		printk("\n cap:%d, ui_cap:%d%, fg_v:%dmv current_adc:%d,charging_current: %d,  charger_source:%d decease_cap_count:%d \n", di->capacity, di->ui_capacity, di->fg_voltage_mV, current_adc_code,di->batt_current_adc, p_di->charger_source, decease_cap_count);
#endif
#endif //LGE_CHANGE_E [euiseop.shin@lge.com] 2011-07-27,[P940]
		D("@@@@@@@@@@@@@@@@@@@@@di->capacity = %d, fg_volt = %d", di->capacity, di->fg_voltage_mV);
	}
	else {
		di->capacity = p_di->pmic_capacity;
		di->voltage_uV = p_di->avg_voltage_mV;
	}
#else
	D("@@@@@@@@@@@@@@@@CONFIG_LG_FW_MAX17043_FUEL_GAUGE is not defined!!!!!");
#endif

#if 0
	//[jongho3.lee@lge.com] correct charging status missmatch caused by charging timer or any...
	if (((charging_ic_status != POWER_SUPPLY_TYPE_BATTERY) &&
	    (di->charge_status != POWER_SUPPLY_STATUS_CHARGING) ) ||
	    ((charging_ic_status == POWER_SUPPLY_TYPE_BATTERY) &&
	    (di->charge_status == POWER_SUPPLY_STATUS_CHARGING) ) ) {
		if(charging_ic_status == POWER_SUPPLY_TYPE_BATTERY) {
			di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else {
			di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		ui_update_needed = 1;
	}
#endif
#if !defined(CONFIG_MAX8971_CHARGER)
#if defined(GET_BET_TEMP)
	D(" ####### BATTERY SOC = %d ##########", di->capacity );


	req.channels = (1 << BAT_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);
#else
	req.channels = (1 << BAT_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	ret = twl6030_gpadc_conversion(&req);

	if (ret < 0) {
		dev_dbg(di->dev, "gpadc conversion failed: %d\n", ret);
		return;
	}
#endif
#else
#if defined(GET_BET_TEMP)

/*	get_adc_value(&req, BAT_TEMP_CHANNEL);
	get_adc_value(&req, BAT_CURRENT_CHANNEL);
*/
	
	req.channels = (1 << BAT_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);


	req.channels = (1 << BAT_CURRENT_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);
#if defined(GET_TEMP_SENSOR_DOCOMO)
/* LGE_CHANGE [jude84.kim@lge.com] 2011-09-30 for check charger current adc value [START] */
	if(system_rev>5){
	req.channels = (1 <<CHARGER_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);
	
/*	get_adc_value(&req, CHARGER_TEMP_CHANNEL);
	get_adc_value(&req, PARM_TEMP_CHANNEL);
*/

	req.channels = (1 <<PARM_TEMP_CHANNEL) | (1 << 7) | (1 << 8);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);
	}
	printk("BAT_TEMP_CHANNEL: %d,req.rbuf:%d \n",BAT_TEMP_CHANNEL ,req.rbuf[BAT_TEMP_CHANNEL]);
	printk("BAT_CURRENT_CHANNEL: %d,req.rbuf:%d \n",BAT_CURRENT_CHANNEL ,req.rbuf[BAT_CURRENT_CHANNEL]);
//	printk("CHARGER_TEMP_CHANNEL: %d,req.rbuf:%d \n",CHARGER_TEMP_CHANNEL ,req.rbuf[CHARGER_TEMP_CHANNEL]);
//	printk("PARM_TEMP_CHANNEL: %d,req.rbuf:%d \n",PARM_TEMP_CHANNEL ,req.rbuf[PARM_TEMP_CHANNEL]);
#endif
#endif
#endif



	if (req.rbuf[7] > 0)
		di->voltage_uV = req.rbuf[7];
	if (req.rbuf[8] > 0)
		di->bk_voltage_uV = req.rbuf[8];


	D("pmic _volt = %d", req.rbuf[7]);

	//[jongho3.lee@lge.com] get avg_voltage of battery.

	/*
	 * Let's check if charging ui update is needed.
	 */
	/* LGE_CHANGE [jongho3.lee@lge.com] monitor changes of tbat. */
	{
		adc_code = req.rbuf[BAT_TEMP_CHANNEL];
#if defined(CONFIG_MAX8971_CHARGER)
		current_adc_code = req.rbuf[BAT_CURRENT_CHANNEL];
		di->batt_temp_adc = adc_code;   /* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20, add to check BATT TEMP ADC value*/
		di->batt_current_adc = ((current_adc_code * 100) / 145);   /* LGE_CHANGE [dukwung.kim@lge.com] 2011-09-16, add to check BATT CURRENT ADC value*/
		/* LGE_CHANGE_S [dukwung.kim@lge.com] 2011-12-08 For filtering .*/
	/*	if( di->batt_temp_adc < 80 || di->batt_temp_adc > 1800)
		{
			di->batt_temp_adc = batt_temp_adc_old;
		}
	*/
		// with No battery case 
		if( di->batt_temp_adc > 1500 && init_temp_adc_flag == 0 )
		{	
			init_temp_adc_flag = 1;
			batt_temp_adc_old[0]= 1700;
			batt_temp_adc_old[1]= 1700;
			batt_temp_adc_old[2]= 1700;
		}
		else if( di->batt_temp_adc < 1500 && init_temp_adc_flag == 0) // with battery case
		{	
			init_temp_adc_flag = 1;	
			batt_temp_adc_old[0]= 700;
			batt_temp_adc_old[1]= 700;
			batt_temp_adc_old[2]= 700;
		}


		if(batt_temp_adc_count==1)
		{
			batt_temp_adc_old[1] = batt_temp_adc_old[0];
			
		}
		else if(batt_temp_adc_count==2)
		{
			batt_temp_adc_old[2] = batt_temp_adc_old[1];
		}
		
			batt_temp_adc_old[0] = di->batt_temp_adc ;
		diff_temp_adc = di->batt_temp_adc - batt_temp_adc_old[2];
		
		if(diff_temp_adc > 300 || diff_temp_adc < -300 || di->batt_temp_adc < 80 || di->batt_temp_adc > 1800 )
		{
			 printk("[twl6030] diff_temp_adc :%d\n",diff_temp_adc);
			di->batt_temp_adc = batt_temp_adc_old[2];
		}
			
		batt_temp_adc_count++;
		batt_temp_adc_count = batt_temp_adc_count%3;

		/* LGE_CHANGE_E [dukwung.kim@lge.com] 2011-12-08 For filtering .*/

		printk("[twl6030] batt_temp_adc:%d\n",di->batt_temp_adc);
#if defined(GET_TEMP_SENSOR_DOCOMO)
	if(system_rev>5){
		charger_adc_code = req.rbuf[CHARGER_TEMP_CHANNEL];
              parm_adc_code = req.rbuf[PARM_TEMP_CHANNEL];

              printk("Charger_ADC_Value = %d, PARM_ADC_Value = %d\n", charger_adc_code,parm_adc_code);
	
		di->temp_charger_adc = charger_adc_code;
		di->temp_parm_adc = parm_adc_code;
		temp_charger_C = average_temp_therm_ic(reference_graph((s64)charger_adc_code, dcm_temp_sensor_graph, ARRAY_SIZE(dcm_temp_sensor_graph)) / (TEMP_TIMES / 10), 
												THERM_SENSOR_CHARGER_INDEX);
              temp_parm_C = average_temp_therm_ic(reference_graph((s64)parm_adc_code, dcm_temp_sensor_graph, ARRAY_SIZE(dcm_temp_sensor_graph)) / (TEMP_TIMES / 10), 
			  									THERM_SENSOR_PARM_INDEX);

              printk("Convert to Celsius temperature : Charger = %d, PARM = %d\n", temp_charger_C ,temp_parm_C);
		}
	else
	{
              printk("system_rev < 5 \n");
               	temp_charger_C = 0;
               	temp_parm_C = 0;
	}
#endif
#endif
		/*euiseop.shin 2011-11-09
		 * di->batt_temp_adc = adc_code;
		 */
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20,
		 * add to check BATT TEMP ADC value
		 */
#if 1
		temp_C = average_temp(reference_graph((s64)adc_code,
				battery_temp_graph,
				ARRAY_SIZE(battery_temp_graph)) / (TEMP_TIMES / 10));
#else
		temp_C = 10;
#endif
		D(" ####### CALCULATED TEMP = %d ##########", temp_C );

		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-11-09
		 * [P2] Low temperature popup error fix
		 */
#if 1
		if ((adc_code < 80 || adc_code > 1700) &&
		    ((temp_C - di->temp_C) > 10 ||
		     (temp_C - di->temp_C) < -10)) {
			DPWR("Original Battery adc = %d, temp_C=%d", adc_code, temp_C);
			adc_code = di->batt_temp_adc;
			temp_C = di->temp_C;
		}
#endif
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20,
		 * add to check BATT TEMP ADC value
		 */
		di->batt_temp_adc = adc_code;

#ifndef BAT_TEMP_TEST
#if !defined(CONFIG_MAX8971_CHARGER)
		if ((temp_C/10) != ((di->temp_C)/10)) {
			/* first 2 values are for negative temperature */
			di->temp_C = temp_C;
			ui_update_needed = 1;
		}
		DTEMP("Battery adc = %d, temp_C=%d", adc_code, temp_C);
#else
	if((get_charging_ic_status() != POWER_SUPPLY_TYPE_FACTORY) && (charging_mode != MUIC_CP_UART) && (charging_mode != MUIC_CP_USB)){
		if( (temp_C/10) != ((di->temp_C)/10)  )
		{
			/* first 2 values are for negative temperature */
			di->temp_C = temp_C;
			ui_update_needed = 1;
		}
	}
	else{
		di->temp_C = 0;
		ui_update_needed = 1;
	}
#if defined(GET_TEMP_SENSOR_DOCOMO)
#if 1 //[START]LGE_CHANGE [jude84@lge.com] 2011-12-09 [P2] Low temperature popup error fix
		if((charger_adc_code < 450 || charger_adc_code > 1000) && ((temp_charger_C - di->temp_charger_C) > 10 || (temp_charger_C - di->temp_charger_C) < -10))
		{
			DPWR("[Defence]Original charger temp adc = %d, temp_charger_C=%d, saved_temp_charger_C=%d", charger_adc_code, temp_charger_C,saved_temp_charger_C);
			temp_charger_C = saved_temp_charger_C;
			//temp_charger_C = di->temp_charger_C;
		}
		if((parm_adc_code < 450 || parm_adc_code > 1000) && ((temp_parm_C - di->temp_parm_C) > 10 || (temp_parm_C - di->temp_parm_C) < -10))
		{
			DPWR("[Defence]Original charger parm adc = %d, parm_charger_C=%d, saved_temp_parm_C=%d", parm_adc_code, temp_parm_C,saved_temp_parm_C);
			temp_parm_C =  saved_temp_parm_C;
			//temp_parm_C =  di->temp_parm_C;
		}
#endif
#endif//[END]LGE_CHANGE [jude84@lge.com] 2011-12-09 [P2] Low temperature popup error fix
#if defined(CONFIG_MAX8971_CHARGER)
#if defined(GET_TEMP_SENSOR_DOCOMO)
	if((system_rev>5) && (get_charging_ic_status() != POWER_SUPPLY_TYPE_FACTORY) && (charging_mode != MUIC_CP_UART) && (charging_mode != MUIC_CP_USB)){
		if( (temp_charger_C/10) != ((di->temp_charger_C)/10)|| (temp_parm_C/10) != ((di->temp_parm_C)/10))
		{
			/* save previous therm value for Charger/PARM before calculate the OFFSET */
			saved_temp_charger_C = temp_charger_C;
			saved_temp_parm_C    = temp_parm_C;
			di->temp_charger_C = temp_charger_C - TEMP_SENSOR_OFFSET;
			di->temp_parm_C = temp_parm_C - TEMP_SENSOR_OFFSET;
			ui_update_needed =1;
		}
	}
	else{
		di->temp_charger_C = 0;
		di->temp_parm_C = 0;
		ui_update_needed =1;
	}
	if(di->temp_charger_C > di->temp_parm_C){
		temp_high_C = di->temp_charger_C;
		temp_low_C = di->temp_parm_C;
	}
	else{
		temp_high_C = di->temp_parm_C;
		temp_low_C = di->temp_charger_C;
	}
	if(temp_high_C < di->temp_C){
		temp_high_C = di->temp_C;}
	else if(temp_low_C > di->temp_C){
		temp_low_C = di->temp_C;}
			
	di->temp_high_C = temp_high_C;
	di->temp_low_C = temp_low_C;
#endif
	printk("TEMP is %d, %d, %d\n", di->temp_charger_C,di->temp_parm_C,di->temp_C);
	DTEMP("Battery adc = %d, temp_C=%d", adc_code, temp_C);
	DTEMP("Battery current adc = %d ,charging current= %d ", current_adc_code,di->batt_current_adc);
//	printk("Battery current adc = %d\n", current_adc_code);
	DTEMP("\nParm temp adc = %d\n", req.rbuf[PARM_TEMP_CHANNEL]);
	DTEMP("\ncharger temp adc = %d\n", req.rbuf[CHARGER_TEMP_CHANNEL]);
	DTEMP("\nparm temp = %dC\n",temp_parm_C);
	DTEMP("\ncharger temp = %dC\n",temp_charger_C);
	//charging_mode = get_muic_detected_cable();
	charging_mode = muic_get_mode();
	if((charging_mode == MUIC_CP_USB) && (di->batt_temp_adc < 1500)&&(charging_test_on != 1))
	{
		//printk("[twl6030] start max8971 charging due to cp usb with battery 500mA\n");
		//max8971_start_charging(500);
		printk("[twl6030] stop  max8971 factory charging due to cp usb with battery \n");
		//max8971_stop_charging();
		 max8971_stop_factory_charging();
		 p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	if((charging_mode == MUIC_CP_UART) && (di->batt_temp_adc < 1500)&&(charging_test_on != 1))
	{
		printk("[twl6030] stop  max8971 factory  charging due to cp uart with battery \n");
		//max8971_stop_charging();
		 max8971_stop_factory_charging();
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#endif
		DTEMP("Battery adc = %d, temp_C=%d", adc_code, temp_C);
#endif

	}
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
	/* LGE_CHANGE <jongho3.lee@lge.com>
	 * we need to set rcomp value with temp..
	 */

	temp_gap = old_temp - temp_C;

	if (temp_gap < 0) {
		temp_gap *= -1;
	}

	if (temp_gap > 30) {
		DPWR("temp_gap=%d-%d=%d", old_temp, temp_C, temp_gap);
		max17043_set_rcomp_by_temperature(temp_C);
		old_temp = temp_C;
	}
#endif


/*
 * Let's check if charging fsm work is needed.
 */
/* LGE_CHANGE [jongho3.lee@lge.com] monitoring battery presence by tbat. */
/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
 * [P940] Imported from Cosmopolitan
 */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER,
			&controller_status_1, CONTROLLER_STAT1);

	if ((controller_status_1 & CONTROLLER_STAT1_VBUS_DET) !=
			p_di->valid_charging_source ) {
		fsm_work_needed = 1;
	}

	if (di->temp_C < TEMP_LOW_NO_BAT &&
	    di->temp_control != UNLIMITED_TEMP_VAL) {
		di->capacity_validation = NOT_INIT_CAP;
		di->battery_present = 0;
		fsm_work_needed = 1;
	}
	else if (!di->battery_present) {
		di->battery_present = 1;
		fsm_work_needed = 1;
	}

	di->avg_voltage_mV = get_avg_volt(di->voltage_uV);
	get_pmic_soc();


	/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-06,
	 * To change the charging mode according battery SOC
	 */
#if defined(CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)	
        if ((!di->lock_on_overheat_scene &&
	     di->charge_status == POWER_SUPPLY_STATUS_CHARGING) &&
	    (recharging_wait_soc_state == RECHARGING_WAIT_SET ||
	     reinit_ta_charger))         	{
		if (!di->lock_on_overheat_scene &&
		   recharging_wait_soc_state == RECHARGING_WAIT_SET) {
			DCHG("Recharging wait:%d so, will be call fsm()", recharging_wait_soc_state);
			recharging_wait_soc_state = RECHARGING_WAIT_UNSET; // clear flag
		}
		else if (!di->lock_on_overheat_scene && reinit_ta_charger) {
			DCHG("Reinit. TA Charger Flag :%d so, will be call fsm()", reinit_ta_charger);
			reinit_ta_charger = 0;
		}
		fsm_work_needed = 1;
	}
#endif
	/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-06,
	 * To change the charging mode according battery SOC
	 */

	/* LGE_CHANGE [jongho3.lee@lge.com] monitor changes of bat soc. */
	if (di->capacity != old_capacity) {
#if defined(CONFIG_MAX8971_CHARGER)
//		printk("[twl6030] fsm_work_needed capacity update!! old:%d new:%d\n", old_capacity, di->capacity);
		if(old_capacity > di->capacity) 
		{
		   decease_cap_count ++;
		   decease_cap_count = decease_cap_count%5;
//		   printk("[twl6030] decease_cap_count:%d \n", decease_cap_count);
 
		}
#endif
		old_capacity = di->capacity;
		ui_update_needed = 1;

//		printk("[BATTERY] batt_voltage = %d, ui_capacity = %d, pmic_capacity = %d, batt_temp = %d, batt_status = %d, batt_present = %d, charger = %d \n", 
//			di->fg_voltage_mV, di->ui_capacity, di->pmic_capacity, di->temp_C, di->charge_status, di->battery_present, di->charger_source);
		
		/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-06,
		 * To change the charging mode according battery SOC
		 */
#if defined(CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)	
	        if ((di->lock_on_overheat_scene == 1 &&
		     di->charge_status == POWER_SUPPLY_STATUS_CHARGING) ||
		    recharging_wait_soc_state == RECHARGING_WAIT_SET ||
		    reinit_ta_charger )         	{
			DCHG("While_cam_recording:%d, Recharging wait:%d", di->lock_on_overheat_scene, recharging_wait_soc_state);
			// clear flag
			if (di->lock_on_overheat_scene == 0 &&
			   recharging_wait_soc_state == RECHARGING_WAIT_SET)
				recharging_wait_soc_state = RECHARGING_WAIT_UNSET;
			fsm_work_needed = 1;
		}
#endif
	/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-06,
	 * To change the charging mode according battery SOC
	 */

	}
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-07-27,
	 * [P940] cut-off voltage is fixed to 3.3v and ui update error fix
	 */
	if (di->ui_capacity != old_ui_capacity) {
		old_ui_capacity = di->ui_capacity;
		ui_update_needed = 1;
	}
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-07-27,[P940] */

	//LGE_CHANGE [jongho3.lee@lge.com] monitoring recharging condition.
	/* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-16,
	 * Added condition " ||di->charge_status == POWER_SUPPLY_STATUS_FULL"
	 */
	if (di->charger_source != POWER_SUPPLY_TYPE_BATTERY &&
	    (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING ||
	     di->charge_status == POWER_SUPPLY_STATUS_FULL) &&
	    (di->capacity <= RECHARGING_BAT_SOC_CON ||
	     di->fg_voltage_mV < RECHARGING_BAT_VOLT_LOW)) {
		fsm_work_needed = 1;
	}

	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * monitoring charging tbat condition..
	 */
#if !defined(CONFIG_MAX8971_CHARGER)
	if ((!is_tbat_good(di->temp_C) &&
	    di->charge_status == POWER_SUPPLY_STATUS_CHARGING) ||
	   (is_tbat_good(di->temp_C) &&
	    di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING &&
	    recharging_status == RECHARGING_WAIT_UNSET &&
	    di->charger_source != POWER_SUPPLY_TYPE_BATTERY)) {
#else
	if(( (!dcm_temp_sensor_status(p_di->temp_high_C, p_di->temp_low_C)) && (di->charge_status == POWER_SUPPLY_STATUS_CHARGING)) \
		|| (dcm_temp_sensor_status(p_di->temp_high_C, p_di->temp_low_C))  && (di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) &&
		(recharging_status == RECHARGING_WAIT_UNSET) && (di->charger_source != POWER_SUPPLY_TYPE_BATTERY)  ) 
	{
#endif
		fsm_work_needed = 1;
	}
	/* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-11-23,
	 * To change the charging mode according battery temperature
	 */
#if defined(FLEX_CHARGING_MODE)	
	else if (di->charge_status == POWER_SUPPLY_STATUS_CHARGING &&
		((old_temp <= TEMP_CHANGE_CHARGING_MODE &&
		  di->temp_C > TEMP_CHANGE_CHARGING_MODE) ||
		 (old_temp > TEMP_CHANGE_CHARGING_MODE &&
		  di->temp_C <= TEMP_CHANGE_CHARGING_MODE)) ) {
		fsm_work_needed = 1;
	}
#endif
	/* LGE_CHANGE_E [wonhui.lee@lge.com] 2011-11-23,
	 * To change the charging mode according battery temperature
	 */

	/*LGE_CHANGE [jongho3.lee@lge.com]
	 * monitoring full charging condition by soc..
	 */
	if (di->capacity > 99 &&
	    di->charge_status != POWER_SUPPLY_STATUS_FULL) {
		fsm_work_needed = 1;
	}
	else if (di->capacity < 100 &&
		 di->charge_status == POWER_SUPPLY_STATUS_FULL) {
		fsm_work_needed = 1;
	}
	 /* LGE_CHANGE [byoungcheol.lee@lge.com]  2011-07-18,
	  * Added condition to check discharging condition
	  */
	else if (recharging_status == RECHARGING_WAIT_UNSET &&
		 di->capacity > 99 &&
		 di->charge_status == POWER_SUPPLY_STATUS_FULL) {
//		DCHG("It's discharging condition!! cap = %d", di->capacity);
		fsm_work_needed = 1;
	}

	if (di->capacity_validation > 100) {
		if (!di->off_mode_timer_working) {
			if (di->pmic_capacity > (RECHARGING_BAT_SOC_CON - 3) &&
			    di->pmic_capacity < 100) {
				/* [jongho3.lee@lge.com] charging timer setting */
				u32 wait;

				init_timer(&(di->off_mode_timer));
				(di->off_mode_timer).data = 0;
				wait = (HZ*CHR_TIMER_SECS * ( 100 - di->pmic_capacity) ) ;
				//wait = (HZ*9 ) ;
				(di->off_mode_timer).expires = jiffies + wait;
				(di->off_mode_timer).function = off_mode_timer_func;
				add_timer(&(di->off_mode_timer));
				di->off_mode_timer_working = 1;
				D("#########################################\n timer START!!!!\n #####################################");
				/* [jongho3.lee@lge.com] charging timer setting */
			}
		}
	}

	/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-05-12,
	 * added for test.. [START_LGE]
	 */
//	D("[bclee][TWL6030] di->valid_charging_source=%d, charging_ic_status=%d", di->valid_charging_source, charging_ic_status);
  /* log added to check fuel gauge stability */
//nthyunjin.yang 120615 remove log
	printk("[Power] ui_cap = %d, volt = %d, chg_status = %d, temp_C = %d, chg_source = %d. \n",
		di->ui_capacity, di->fg_voltage_mV, di->charge_status, di->temp_C, di->charger_source);

	if (max17043_get_ui_capacity()==100 &&
	   di->charger_source != POWER_SUPPLY_TYPE_BATTERY)
		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-09-03,
		 * add charging condition to prevent
		 * the full-charging message in the discharging condition
		 */
		di->charge_status = POWER_SUPPLY_STATUS_FULL;
	/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-05-12,
	 * added for test.. [END_LGE]
	 */

	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * do charging fsm if battery is present..
	 */
	if (fsm_work_needed) {
//		DCHG("chg_src:%d, chg_s:%d, rchg_s:%d, batt_p:%d, ui_cap:%d, pmic_v:%d, fg_v:%d, adc:%d, temp_C:%d", di->charger_source, di->charge_status, recharging_status, di->battery_present, di->ui_capacity, di->voltage_uV, di->fg_voltage_mV, adc_code, temp_C);
		charger_schedule_delayed_work(&di->charger_work, 0);
		return;
	}

	if (ui_update_needed) {
		//printk("[TWL6030] cap:%d, ui_cap:%d%%, pmic_v:%d, avg_v:%d, fg_v:%d, adc:%d, temp_C:%d, bk_v:%d ovht: %d \n", di->capacity, di->ui_capacity, di->voltage_uV, di->avg_voltage_mV, di->fg_voltage_mV, adc_code, temp_C, di->bk_voltage_uV, di->lock_on_overheat_scene);

		/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-15,
		 * To prevent sending uevent in charging test
		 */
		if (charging_test_on == 0)
			power_supply_changed(&di->bat);
	}
}

#if defined(TWL6030_PMIC_CHARGING)
static void twl6030_current_mode_changed(struct twl6030_bci_device_info *di)
{

	/* FG_REG_01, 02, 03 is 24 bit unsigned sample counter value */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->timer_n1,
							FG_REG_01, 3);
	/*
	 * FG_REG_04, 5, 6, 7 is 32 bit signed accumulator value
	 * accumulates instantaneous current value
	 */
	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &di->charge_n1,
							FG_REG_04, 4);

	cancel_delayed_work(&di->twl6030_current_avg_work);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work,
		msecs_to_jiffies(1000 * di->current_avg_interval));
}
#endif

#define to_twl6030_bci_device_info(x) container_of((x), \
			struct twl6030_bci_device_info, bat);

static void twl6030_bci_battery_external_power_changed(struct power_supply *psy)
{
	//struct twl6030_bci_device_info *di = to_twl6030_bci_device_info(psy);

#if 0
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 0);
#endif
}

#define to_twl6030_ac_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, ac);

static int twl6030_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_ac_device_info(psy);

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, add P940_MUIC
	 */

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
		 * [P940] Imported from Cosmopolitan
		 */
		if (di->charger_source == POWER_SUPPLY_TYPE_MAINS &&
		    di->valid_charging_source) {
			if (di->charger_logo_status ==
					CHARGER_LOGO_STATUS_STARTED &&
			    (start_cond & 0x20))
				val->intval = 0;
			else
				val->intval = 1;
		}
		else {
			/* LGE_CHANGE [wonhui.lee@lge.com] 2011-10-04,
			 * In the case of power-on by Plug-det
			 * but Plug is removed, it should power-off
			 */
			   /* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-22,
			    * To prevent reset in Factory Mode
			    */
			if (di->charger_source == POWER_SUPPLY_TYPE_BATTERY &&
			    get_charging_ic_status() != POWER_SUPPLY_TYPE_FACTORY &&
			    di->charger_logo_status ==
			    	CHARGER_LOGO_STATUS_STARTED &&
			    (start_cond & 0x40))
				val->intval = 2;
			else
				val->intval = 0;
		}

		if (di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED)
			printk("TWL6030: ac online(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(9);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define to_twl6030_usb_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, usb);

static int twl6030_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_usb_device_info(psy);

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, add P940_MUIC
	 */
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
		if (di->charger_source == POWER_SUPPLY_TYPE_USB &&
		    di->valid_charging_source) {
			if (di->charger_logo_status ==
					CHARGER_LOGO_STATUS_STARTED &&
			   (start_cond & 0x20))
				val->intval = 0;
			else
				val->intval = 1;
		}
		else {
			/* LGE_CHANGE [wonhui.lee@lge.com] 2011-10-04,
			 * In the case of power-on by Plug-det
			 * but Plug is removed, it should power-off
			 */
			  /* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-22,
			   * To prevent reset in Factory Mode
			   */
			if (di->charger_source == POWER_SUPPLY_TYPE_BATTERY &&
			    get_charging_ic_status() !=
			    		POWER_SUPPLY_TYPE_FACTORY &&
			    di->charger_logo_status ==
			    		CHARGER_LOGO_STATUS_STARTED &&
			   (start_cond & 0x40))
				val->intval = 2;
			else
				val->intval = 0;
		}
		if (di->charger_logo_status == CHARGER_LOGO_STATUS_STARTED)
			printk("TWL6030: usb online(%d)\n", val->intval);

		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = twl6030_get_gpadc_conversion(10);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE] */
static void battery_gauge_reset_work(struct work_struct *work)
{
#if defined(OFF_MODE_RESET)
	struct twl6030_bci_device_info *di = container_of(work,
			struct twl6030_bci_device_info,
			gauge_reset_work.work);

	mutex_lock(&charging_fsm_lock);
#if defined(CONFIG_MAX8971_CHARGER)
	DCHG("charger deactive check point 8 \n");
                charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 9 \n");
                max8971_stop_charging();

#endif	
	recharging_status = RECHARGING_WAIT_UNSET;
	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (di->charger_logo_status == CHARGER_LOGO_STATUS_END) {
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
		msleep(250);   //wait until fuel gauge voltage stable.
		max17043_do_calibrate();
		msleep(250);   //wait reseting fuel gauge.
#endif
	}

	mutex_unlock(&charging_fsm_lock);

	if (di->charger_logo_status == CHARGER_LOGO_STATUS_END) {
		charger_schedule_delayed_work(&di->charger_work, 0);
	}
#endif
}

// LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE]
static void battery_power_off_work(struct work_struct *work)
{
	// LGE_CHAGNE_S [jongho3.lee@lge.com] resister map
	// /* PHOENIX_DEV_ON*/
//#define TWL6030_CHIP_PM	0x48
#define TWL6030_CHIP_PM		0x14

#define PHOENIX_DEV_ON		0x06
//#define PHOENIX_DEV_ON	0x25
#if defined(CONFIG_MAX8971_CHARGER)

u8 uninitialized_var(val);
int err;
#endif
#define SW_RESET		(1 << 6)
#define MOD_DEVON		(1 << 5)
#define CON_DEVON		(1 << 4)
#define APP_DEVON		(1 << 3)
#define MOD_DEVOFF		(1 << 2)
#define CON_DEVOFF		(1 << 1)
#define APP_DEVOFF		(1 << 0)

#define DEV_OFF			MOD_DEVOFF | CON_DEVOFF | APP_DEVOFF

	mutex_lock(&charging_fsm_lock);

	DCHG("charger deactive check point 9 \n");
#if !defined(CONFIG_MAX8971_CHARGER)
                charging_ic_deactive();
#else
//	D("[twl6030] max8971_stop_charging  10\n");
//                max8971_stop_charging();

	val =  SW_RESET | APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;
        printk("[twl6030] power off due to No battery\n");
        err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,TWL6030_PHONIX_DEV_ON);
        if(err)
        {
                printk("[twl6030] Retry due to i2c error,  power off due to No battery\n");
                err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,TWL6030_PHONIX_DEV_ON);
        }
        if(err)
        {
                printk("[twl6030] Retry due to i2c error, max8971_start_charging 500mA for power off\n");
                max8971_start_charging(500);
        }
	
	
#endif
#if defined(POWER_OFF_WHEN_NO_BAT)
	//twl_i2c_write_u8(TWL6030_CHIP_PM, DEV_OFF, PHOENIX_DEV_ON); /* PHOENIX_DEV_ON Reg */
	//kernel_halt();
	//do_exit(0);
	//panic("cannot halt");

	msleep_interruptible(1900);   //wait until fuel gauge voltage stable.

	machine_halt();
#endif
	mutex_unlock(&charging_fsm_lock);

	//kernel_power_off();
}

/* LGE_CHANGE [jongho3.lee@lge.com] charger work func. [START_LGE] */
static void charging_battery_work(struct work_struct *work)
{
	/*
	struct twl6030_bci_device_info *di = container_of(work,
		struct twl6030_bci_device_info,
		charger_work.work);
	twl6030_bk_bci_battery_read_status(di);
	charger_schedule_delayed_work(&di->twl6030_bk_bci_monitor_work, 5000);
	*/
	charger_fsm(CHARG_FSM_CAUSE_ANY);
}

/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31, LGE_P940, remove bk_battery. */
#ifdef FEATURE_BK_BATTERY
#define to_twl6030_bk_bci_device_info(x) container_of((x), \
		struct twl6030_bci_device_info, bk_bat);

static int twl6030_bk_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di = to_twl6030_bk_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->bk_voltage_uV; //euiseop.shin 2011-10-19
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif /* FEATURE_BK_BATTERY */

/* LGE_CHANGE [jongho3.lee@lge.com] validate_gauge_value. [START_LGE] */


int validate_gauge_value(int capacity)
{
	/* Capacity validation is performed in modem.
	 * this is just for in case of no battery..
	 */
	static int old_capacity = -1;
	static int retain_uicap_cnt = 0;

	if (!p_di->battery_present) {
		return capacity;
	}

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	if (p_di->cap_seamless && old_capacity > 1) {
		if (p_di->charge_status == POWER_SUPPLY_STATUS_DISCHARGING) {
			/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
			 * [P940] Imported from Cosmopolitan
			 */
			if((capacity - old_capacity) > 0) {
				return old_capacity;
			}

			/*LGE_CHANGE [euiseop.shin@lge.com] 2011-10-24
			 * [P2] ui-capacity display error fix
			 */
#if 1
			if (p_di->charger_source == POWER_SUPPLY_TYPE_BATTERY) {
				if ((old_capacity - capacity) == 2) {
					DPWR("old_capacity:%d new_cap:%d =>new_cap:%d cnt:%d", old_capacity,capacity,(old_capacity-1),retain_uicap_cnt);
					if(retain_uicap_cnt++ < 3) {
						return old_capacity - 1;
					}
					else {
						retain_uicap_cnt = 0;
						old_capacity = old_capacity - 1;
						return old_capacity;
					}
				}
			}
#endif
		}
		else if (p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
			if ((old_capacity - capacity) > 0 &&
			   (old_capacity - capacity) < 2) {
				return old_capacity;
			}
		}

		retain_uicap_cnt = 0;
		old_capacity = capacity;
		return capacity;
	}
	else {
		p_di->cap_seamless = 1;
		old_capacity = capacity;
		return capacity;
	}
}

/* LGE_CHANGE [jongho3.lee@lge.com] report battery status.[start] */
int get_battery_health(struct twl6030_bci_device_info *di)
{
		int temp = di->temp_C;
		if (temp >= TEMP_LOW_DISCHARGING &&
		    temp < TEMP_HIGH_DISCHARGING) {
			return POWER_SUPPLY_HEALTH_GOOD;
		}
		else if (temp < TEMP_LOW_DISCHARGING ) {
			return POWER_SUPPLY_HEALTH_COLD;
		}
		else if (temp >= TEMP_HIGH_DISCHARGING) {
			return POWER_SUPPLY_HEALTH_OVERHEAT;
		}

		/* FIXME: [jongho3.lee@lge.com] TODO : voltage check. */
		return POWER_SUPPLY_HEALTH_UNKNOWN;
}

/* [jongho3.lee@lge.com] add to set_property. //start */
int twl6030_bci_battery_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di;
	//int charging_mode = 0;

	di = to_twl6030_bci_device_info(psy);

	switch (psp) {
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
		printk("[TWL6030] set GAUGE_CONTROL SOC = %d \n", val->intval);
		di->gauge_control_count++;
		if (di->charger_logo_status != CHARGER_LOGO_STATUS_END) {
			if (val->intval == 300001) {
				di->charger_logo_status = CHARGER_LOGO_STATUS_END;
				wake_unlock(&(p_di->off_mode_charger_wake_lock));

#if 0
				if (di->battery_present &&
				    di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
					charger_schedule_delayed_work(&di->gauge_reset_work, 0);
				}
#endif

				D("@@@@@@ OFF MODE CHARGING is finished : ");

				if (di->off_mode_timer_working) {
					del_timer(&(p_di->off_mode_timer));
					p_di->off_mode_timer_working = 0;
				}
        break;
			}
			else if (val->intval == 300000) {
				di->charger_logo_status = CHARGER_LOGO_STATUS_STARTED;
				wake_lock(&(p_di->off_mode_charger_wake_lock));

				if (!di->battery_present &&
				    di->charger_source != POWER_SUPPLY_TYPE_FACTORY) {
					if (p_di->valid_charging_source) {
#if defined(POWER_OFF_WHEN_NO_BAT)
						charger_schedule_delayed_work(&di->power_off_work, HZ*2);
#else
						charger_schedule_delayed_work(&di->power_off_work, 0);
#endif
					}
				}
				D("@@@@@@ OFF MODE CHARGING is started : ");
        break;
			}
		}
		if (val->intval == 600001) {
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
			//DPMIC("-> max17043_do_calibrate()");
			max17043_do_calibrate();
#endif
			break;
		}

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
		if (val->intval == FG_CONTROL_ENABLE) {
			//DPMIC("-> set_fg_enable()");
			set_fg_enable(1);
			cancel_delayed_work(&di->twl6030_bci_monitor_work);
			charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 5 * HZ);
			break;
		}

		if (val->intval == FG_CONTROL_DISABLE) {
			set_fg_enable(0);
			break;
		}
#endif

		if (val->intval == 600003) {
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
			//DPMIC("-> max17043_quickstart()");
			max17043_quickstart();
#endif
			break;
		}

#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
		if (val->intval == 700000) {
			//DPMIC("-> FACTORY_CHARGER_DISABLE");
			p_di->factory_control = FACTORY_CHARGER_DISABLE;
			cancel_delayed_work(&di->twl6030_bci_monitor_work);
			charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, HZ/10);
			break;
		}

		if (val->intval == 700001) {
			//DPMIC("-> FACTORY_CHARGER_ENABLE");
			p_di->factory_control = FACTORY_CHARGER_ENABLE;
			cancel_delayed_work(&di->twl6030_bci_monitor_work);
			charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, HZ/10);
			break;
		}
#endif

#if defined(CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)
		if (val->intval == 800000) {
//			printk("antispoon camera recording START\n");
			p_di->lock_on_overheat_scene = 1; 
			
			/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-14,
			 * To change the charging mode according battery SOC
			 */
#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
			printk("[BCLEE]Reinit TA Charger:%d, charge_status:%d, p_di->charge_S:%d \n", reinit_ta_charger, di->charge_status, p_di->charge_status);
			if (di->capacity < HIGH_SOC_CHANGE_CHG_MODE &&
			    p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
				printk("[BCLEE] Changer Charger Mode to USB\n");
#if !defined(CONFIG_MAX8971_CHARGER)
				charging_ic_active_default();
#else 
				max8971_start_charging(250);
#endif
				//p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
			}
			else if (di->capacity >= HIGH_SOC_CHANGE_CHG_MODE &&
				 p_di->charge_status == POWER_SUPPLY_STATUS_CHARGING) {
				printk("[BCLEE] Changer Charger Mode to OFF!!\n");
#if !defined(CONFIG_MAX8971_CHARGER)				
				charging_ic_deactive();
#else
				max8971_stop_charging();
#endif
			}
#endif /* CHANGE_CHG_MODE_ON_OVERHEAT_SCENE */
			/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-14,
			 * To change the charging mode according battery SOC
			 */
			break;
		}
		if (val->intval == 800001) {
//			printk("antispoon camera recording STOP \n");
			p_di->lock_on_overheat_scene = 0; 
			
			/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-07,
			 * To change the charging mode according battery SOC
			 */
#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
			reinit_ta_charger = 1;
			cancel_delayed_work(&di->charger_work);
			charger_schedule_delayed_work(&di->charger_work, 0);
#endif
			/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-07,
			 * To change the charging mode according battery SOC
			 */
			break;
		}
#endif /* CHANGE_CHG_MODE_ON_OVERHEAT_SCENE */
		break;
#endif /* FEATURE_GAUGE_CONTROL */
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-16, [P940] */

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		break;
	case POWER_SUPPLY_PROP_CHARGER_MODE:
		printk("[TWL6030] set p_di->charger_mode = %s ", val->strval);
		if (strncmp((char*)(val->strval), "ac", 2) == 0) {
#if !defined(CONFIG_MAX8971_CHARGER)
			charging_ic_set_ta_mode();
#else
	printk("[twl6030] max8971_start_charging due to ATC(TA) \n");
			max8971_start_charging(900);
#endif
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
			/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-09,
			 * To prevent changing charging mode in charging test
			 */
			charging_test_on = 1;
		}
		else if (strncmp((char*)(val->strval), "usb", 3) == 0) {
#if !defined(CONFIG_MAX8971_CHARGER)
			charging_ic_active_default();
#else
	printk("[twl6030] max8971_start_charging due to ATC(TA) \n");
			max8971_start_charging(500);
#endif
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if (strncmp((char*)(val->strval), "factory", 7) == 0) {

#if !defined(CONFIG_MAX8971_CHARGER)
			charging_ic_set_factory_mode();
#endif
			p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		else if (strncmp((char*)(val->strval), "stop", 4) == 0) {
#if !defined(CONFIG_MAX8971_CHARGER)
			DCHG("charger deactive check point 10 \n");
			charging_ic_deactive();
#else
	D("[twl6030] max8971_stop_charging due to ATC \n");
                        max8971_stop_charging();
#endif		
			p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
			/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-15,
			 * To prevent sending uevent in charging test
			 */
			//charging_test_on = 0;
		}
		break;
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
		 * LGE_P940, Bring in Cosmo. [START_LGE]
		 */
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
		printk("[TWL6030] set p_di->temp_control = %02X ", val->intval);
		if (val->intval == UNLIMITED_TEMP_VAL) {
			p_di->temp_control = val->intval;
		}
		else {
			p_di->temp_control = 0;
		}
		//FIXME: write to nv data.
		lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &(p_di->temp_control), 1);
		break;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo. [END_LGE]
	 */

#ifdef BAT_TEMP_TEST
	case POWER_SUPPLY_PROP_TEMP:
		p_di->temp_C = val->intval;
		break;
#endif

	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(twl6030_bci_battery_set_property);
/* [jongho3.lee@lge.com] add to set_property. //end */

int adjust_capacity(int level)
{
	return ((((level * 10000) / RECHARGING_BAT_SOC_CON) + 50) / 100);
}
EXPORT_SYMBOL(adjust_capacity);

/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-07-06,
 * [P940] Imported from Cosmopolitan
 */
static void twl_power_supply_changed_work(struct work_struct *work)
{
	struct twl6030_bci_device_info *di = container_of(work,
			struct twl6030_bci_device_info,
			twl_power_supply_changed_work.work);

	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-15,
	 * To prevent sending uevent in charging test
	 */
	if (charging_test_on == 0)
		power_supply_changed(&(di->bat));
}

void set_modem_alive(int alive)
{
	if (!p_di)
		return;

	p_di->modem_alive = alive;
	charger_schedule_delayed_work(
			&(p_di->twl_power_supply_changed_work), 0);
}
EXPORT_SYMBOL(set_modem_alive);
/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-07-06,[P940] */

/* LGE_CHANGE [jongho3.lee@lge.com] validate_gauge_value. [END_LGE] */
static int twl6030_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl6030_bci_device_info *di;
	u8 temp_val = 0;
	int ret = 0;

	di = to_twl6030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (p_di->valid_charging_source &&
		    p_di->battery_present &&
		    recharging_status == RECHARGING_WAIT_SET &&
		    (di->charger_source == POWER_SUPPLY_TYPE_USB ||
		     di->charger_source == POWER_SUPPLY_TYPE_MAINS )) {
			//DCHG("POWER_SUPPLY_STATUS_FULL check point 1");
			//val->intval = POWER_SUPPLY_STATUS_CHARGING;
			/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-05-12,
			 * Fixed return value as STATUS_FULL.
			 * Should be returnd FULL while recharging wait set.
			 */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		else {
			if (di->capacity >= RECHARGING_BAT_SOC_CON &&
			   !p_di->valid_charging_source) {
				//DCHG("POWER_SUPPLY_STATUS_FULL check point 2");
				//val->intval = POWER_SUPPLY_STATUS_FULL;
				/* LGE_CHANGE [byoungcheol.lee@lge.com]
				 * 2011-05-13, Fixed return value
				 * as STATUS_DISCHARGING.
				 * Should be returnd DISCHARGING
				 * while valid_charging_source==0.
				 */
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			}
			else {
				//DCHG("POWER_SUPPLY_STATUS_FULL Check Point 3");
				/* LGE_CHANGE [euiseop.shin@lge.com] 2011-08-29
				 * [P940] battery full charge display error fix,
				 * in Lock screen.[TD#20029]
				 */
				if(max17043_get_ui_capacity()==100 &&
				   p_di->valid_charging_source &&
				   (di->charger_source == POWER_SUPPLY_TYPE_USB ||
				    di->charger_source == POWER_SUPPLY_TYPE_MAINS ||
				    di->charger_source == POWER_SUPPLY_TYPE_FACTORY )) {
					val->intval = POWER_SUPPLY_STATUS_FULL;
				}
				else {
					val->intval = di->charge_status;
				}
			}
		}
		//DCHG("charge_status:%d, recharging_status:%d \n", val->intval, recharging_status);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-09-29
		 * [P2]voltage_now is a fuel gauge voltage
		 */
		val->intval = di->fg_voltage_mV;
#else
		val->intval = di->avg_voltage_mV;
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, add MAX17043_FUEL_GAUGE
	 */
#if !defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE) && !defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
		twl6030battery_current(di);
#endif
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CONFIG_MACH_LGE_COSMO
		if(di->temp_control == 0xFF)
		{
			ret = lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val , 1);
			D("@@@@@@@@@@@@@@@E_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET= %x", di->temp_control);
			if(ret == 1)
			{
				di->temp_control = temp_val;
			}
		}
#endif
		if (p_di->temp_control == UNLIMITED_TEMP_VAL) {
		/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-10-28,
		 * Change charging temperature value for LG senario [START_LGE]
		 */
			if (di->temp_C > UNLIMITED_TEMP_HIGH)
				val->intval = UNLIMITED_TEMP_HIGH;
			else if (di->temp_C < UNLIMITED_TEMP_LOW)
				val->intval = UNLIMITED_TEMP_LOW;
		/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-10-28,
		 * Change charging temperature value for LG senario [END_LGE]
		 */
			else
				val->intval = di->temp_C;
		}
		else {
			val->intval = di->temp_C;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = di->charger_source;
		break;
	/* LGE_CHANGE [jongho3.lee@lge.com] report battery status.[start] */
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	/* LGE_CHANGE [jongho3.lee@lge.com] report battery status.[end] */
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = di->current_avg_uA;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		di->bat_health = get_battery_health(di);
		val->intval = di->bat_health;
		break;
	/* LGE_CHANGE [jongho3.lee@lge.com] report battery status.[end] */

	case POWER_SUPPLY_PROP_CAPACITY:
		/* FIXME correct the threshold
		 * need to get the correct percentage value per the
		 * battery characteristics. Approx values for now.
		 */
	/*FIXME: fuel gauge should be in action. */
//#if defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE)
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, add MAX17043_FUEL_GAUGE
	 */
#if defined(CONFIG_LG_FW_MAX17040_FUEL_GAUGE) || defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
#if 0 //LGE_CHANGE [euiseop.shin@lge.com] 2011-09-29 [P2] Maybe this is not needed.
		if(!get_fg_enable()) {
			val->intval = adjust_capacity(di->pmic_capacity);
			D("@@@@@@@@ pmic CAPACITY : %d , @@@@ UI CAPACITY : %d ", di->capacity, val->intval);
		}
		else
#endif
		{
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, select a fuelgauge capacity or pmic.
	 */
#if defined(CONFIG_LG_FW_MAX17043_FUEL_GAUGE)
			//val->intval = (validate_gauge_value(max17043_get_ui_capacity()));
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-27,
	 * [P940] Capacity ui update error fix
	 */
			val->intval = di->ui_capacity;
#else
			val->intval = di->capacity;
#endif
			D("@@@@@@@@ real CAPACITY : %d , @@@@ UI CAPACITY : %d ", di->capacity, val->intval);
		}
#else
		val->intval = di->capacity;
#endif

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
#if 0
		if(val->intval > 97 &&
		   recharging_status != RECHARGING_WAIT_UNSET) {
			val->intval = 100;
		}
#endif
		if (val->intval > 100) {
			val->intval = 100;
		}

		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
		 * [P940] Imported from Cosmopolitan
		 */
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-09-29
		 * [P2] Maybe this is not needed.
		 */
#if 0
		if (!di->modem_alive &&
		   (val->intval < SAFE_SHUTDOWN_SOC_CON) ) {
			val->intval = 0;
		}
#endif
		/* LGE_CHANGE_S <jongho3.lee@lge.com>
		 * set capacity to 100 if capacity is 0 while factory charging.
		 */
		if ((di->charger_source == POWER_SUPPLY_TYPE_FACTORY ||
		    charging_test_on == 1)) {
			val->intval = 100;
		}
		
		D("@@@@@@@@ CAPACITY : %d , @@@@ UI CAPACITY : %d   MODEM_ALICVE = %d ", di->capacity, val->intval, di->modem_alive);
		break;

#if defined(CONFIG_MACH_LGE)
		/* [jongho3.lee@lge.com] add to gauge_property. //start */
	case POWER_SUPPLY_PROP_GAUGE_VOLTAGE:
		/* euiseop.shin 2011-07-25 modify */
		val->intval = di->fg_voltage_mV;
		break;
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
	/* FIXME: this is temp code. battery gauge graph should be calculated.. */
		val->intval = di->capacity_validation;
		break;
#endif
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-16, [P940] */

	case POWER_SUPPLY_PROP_PMIC_SOC:
		val->intval = di->pmic_capacity;
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
		 * [P940] Imported from Cosmopolitan
		 */
		/* LGE_CHANGE_S <jongho3.lee@lge.com>
		 * set capacity to 100 if capacity is 0 while factory charging.
		 */
		if (di->charger_source == POWER_SUPPLY_TYPE_FACTORY &&
		    !val->intval) {
			val->intval = 90;
		}
		break;
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	case POWER_SUPPLY_PROP_GAUGE_CONTROL_COUNT:
		val->intval = di->gauge_control_count;
		break;
#endif
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-16, [P940] */

	case POWER_SUPPLY_PROP_CHARGER_MODE:
		val->intval = get_charging_ic_status();
		break;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo. [START_LGE]
	 */
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
		/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-08-17,
		 * Added for viewing to temp_control on adb shell
		 */
#ifdef CONFIG_MACH_LGE_COSMO
		lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET, &temp_val, 1);
		val->intval = (int)temp_val;
#else
		val->intval = (int)p_di->temp_control;
#endif
		D("p_di->temp_control = %02X ", (char)val->intval);
		break;
		/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
		 * LGE_P940, Bring in Cosmo. [END_LGE]
		 */

	/* LGE_CHANGE_S [wonhui.lee@lge.com] 2011-08-20,
	 * add to check BATT TEMP ADC value
	 */
	case POWER_SUPPLY_PROP_BATT_TEMP_ADC:
		val->intval = (int)di->batt_temp_adc;
		break;
	/* LGE_CHANGE_E [wonhui.lee@lge.com] 2011-08-20,
	 * add to check BATT TEMP ADC value
	 */
#if defined(CONFIG_MAX8971_CHARGER)
	case POWER_SUPPLY_PROP_BATT_CURRENT_ADC:
		val->intval = (int)di->batt_current_adc;
		break;
#endif
#endif

#if defined(CONFIG_MAX8971_CHARGER)
#if defined(GET_TEMP_SENSOR_DOCOMO)
/* LGE_CHANGE [jude84..kim@lge.com] 2011-09-30, add to check charger temp value[START]*/
	case POWER_SUPPLY_PROP_CHARGER_TEMP_ADC:
		val->intval = (int)di->temp_charger_adc;
		break;
	case POWER_SUPPLY_PROP_PARM_TEMP_ADC:
		val->intval = (int)di->temp_parm_adc;
		break;
	case POWER_SUPPLY_PROP_CHARGER_TEMP:
                if( p_di->temp_control == UNLIMITED_TEMP_VAL )
                {
// LGE_CHANGE_S [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [START_LGE]
                        if( di->temp_charger_C > UNLIMITED_TEMP_HIGH )
                                val->intval = UNLIMITED_TEMP_HIGH;
                        else if( di->temp_charger_C < UNLIMITED_TEMP_LOW )
                                val->intval = UNLIMITED_TEMP_LOW;
// LGE_CHANGE_E [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [END_LGE]
                        else
                                val->intval = di->temp_charger_C;
                }
                else
                {
                        val->intval = di->temp_charger_C;
                }
                break;
        case POWER_SUPPLY_PROP_PARM_TEMP:
		if( p_di->temp_control == UNLIMITED_TEMP_VAL )
                {
// LGE_CHANGE_S [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [START_LGE]
                        if( di->temp_parm_C > UNLIMITED_TEMP_HIGH )
                                val->intval = UNLIMITED_TEMP_HIGH;
                        else if( di->temp_parm_C < UNLIMITED_TEMP_LOW )
                                val->intval = UNLIMITED_TEMP_LOW;
// LGE_CHANGE_E [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [END_LGE]
                        else
                                val->intval = di->temp_parm_C;
                }
                else
                {
                        val->intval = di->temp_parm_C;
                }
                break;
/* LGE_CHANGE [jude84..kim@lge.com] 2011-09-30, add to check charger temp value[END]*/
	case POWER_SUPPLY_PROP_TEMP_HIGH:
		if( p_di->temp_control == UNLIMITED_TEMP_VAL )
                {
// LGE_CHANGE_S [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [START_LGE]
                        if( di->temp_high_C > UNLIMITED_TEMP_HIGH )
                                val->intval = UNLIMITED_TEMP_HIGH;
                        else if( di->temp_high_C < UNLIMITED_TEMP_LOW )
                                val->intval = UNLIMITED_TEMP_LOW;
// LGE_CHANGE_E [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [END_LGE]
                        else
                                val->intval = di->temp_high_C;
                }
                else
                {
                        val->intval = di->temp_high_C;
                }
                break;
	case POWER_SUPPLY_PROP_TEMP_LOW:
		if( p_di->temp_control == UNLIMITED_TEMP_VAL )
                {
// LGE_CHANGE_S [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [START_LGE]
                        if( di->temp_low_C > UNLIMITED_TEMP_HIGH )
                                val->intval = UNLIMITED_TEMP_HIGH;
                        else if( di->temp_low_C < UNLIMITED_TEMP_LOW )
                                val->intval = UNLIMITED_TEMP_LOW;
// LGE_CHANGE_E [jude84.kim@lge.com] 2011-10-28, Change charging temperature value for LG senario [END_LGE]
                        else
                                val->intval = di->temp_low_C;
                }
                else
                {
                        val->intval = di->temp_low_C;
                }
                break;
#endif
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

int twl6030_register_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_register(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_register_notifier);

int twl6030_unregister_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_unregister(&notifier_list, nb);
}
EXPORT_SYMBOL_GPL(twl6030_unregister_notifier);

#if defined(TWL6030_PMIC_FUELGAUGE)
/* LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge. */
static ssize_t set_fg_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;
	di->fuelgauge_mode = val;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, (val << 6) | CC_CAL_EN,
							FG_REG_00);
	twl6030_current_mode_changed(di);
	return count;
}

static ssize_t show_fg_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->fuelgauge_mode;
	return sprintf(buf, "%d\n", val);
}
#endif

#if defined(TWL6030_PMIC_CHARGING)
/* FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute. */
static ssize_t set_charge_src(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 2) || (val > 3))
		return -EINVAL;
	di->vac_priority = val;
	return count;
}

/* FIXME: [jongho3.lee@lge.com] show charger source in properway. */
static ssize_t show_charge_src(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->vac_priority;
	return sprintf(buf, "%d\n", val);
}
#endif

static ssize_t show_vbus_voltage(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(10);

	return sprintf(buf, "%d\n", val);
}

static ssize_t show_id_level(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(14);

	return sprintf(buf, "%d\n", val);
}

static ssize_t set_watchdog(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 1) || (val > 127))
		return -EINVAL;
	di->watchdog_duration = val;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, val, CONTROLLER_WDG);

	return count;
}

static ssize_t show_watchdog(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->watchdog_duration;
	return sprintf(buf, "%d\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
/* LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge. */
static ssize_t show_fg_counter(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int fg_counter = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_counter,
							FG_REG_01, 3);
	return sprintf(buf, "%d\n", fg_counter);
}

static ssize_t show_fg_accumulator(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	long fg_accum = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_accum, FG_REG_04, 4);

	return sprintf(buf, "%ld\n", fg_accum);
}

static ssize_t show_fg_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	s16 fg_offset = 0;

	twl_i2c_read(TWL6030_MODULE_GASGAUGE, (u8 *) &fg_offset, FG_REG_08, 2);
	fg_offset = ((s16)(fg_offset << 6) >> 6);

	return sprintf(buf, "%d\n", fg_offset);
}

static ssize_t set_fg_clear(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	long val;

	if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
		return -EINVAL;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_AUTOCLEAR, FG_REG_00);

	return count;
}

static ssize_t set_fg_cal(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	long val;

	if ((strict_strtol(buf, 10, &val) < 0) || (val != 1))
		return -EINVAL;
	twl_i2c_write_u8(TWL6030_MODULE_GASGAUGE, CC_CAL_EN, FG_REG_00);

	return count;
}
#endif //we dont control PMIC fuel gauge.

static ssize_t set_charging(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
#if defined(TWL6030_PMIC_CHARGING)
	//struct twl6030_bci_device_info *di = dev_get_drvdata(dev);
#endif

	/* LGE_CHANGE [jongho3.lee@lge.com] we control external charger. */
	if (strncmp(buf, "startac", 7) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY ) {
#if !defined(CONFIG_MAX8971_CHARGER)
			DCHG("charger deactive check point 11 \n");
			charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 13 \n");
	                 max8971_stop_charging();
#endif
		}
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_set_ta_mode();
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (strncmp(buf, "startusb", 8) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY ) {
#if !defined(CONFIG_MAX8971_CHARGER)
			DCHG("charger deactive check point 12 \n");
			charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 14 \n");
                	 max8971_stop_charging();
#endif			
		}
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_active_default();
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (strncmp(buf, "startfactory", 12) == 0) {
		if(get_charging_ic_status() != POWER_SUPPLY_TYPE_BATTERY ) {
#if !defined(CONFIG_MAX8971_CHARGER)
			DCHG("charger deactive check point 13 \n");
			charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging 15 \n");
              		 max8971_stop_charging();
#endif			
		}
#if !defined(CONFIG_MAX8971_CHARGER)
		charging_ic_set_factory_mode();
#endif
		p_di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (strncmp(buf, "stop" , 4) == 0) {
#endif
#if !defined(CONFIG_MAX8971_CHARGER)
		DCHG("charger deactive check point 14 \n");
		charging_ic_deactive();
#else
	DCHG("[twl6030] max8971_stop_charging  16 \n");
          	       max8971_stop_charging();
#endif		
		p_di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else
		return -EINVAL;

	return count;
}

static ssize_t set_regulation_voltage(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 3500)
				|| (val > di->max_charger_voltagemV))
		return -EINVAL;
	di->regulation_voltagemV = val;
	twl6030_config_voreg_reg(di, val);

	return count;
}

static ssize_t show_regulation_voltage(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->regulation_voltagemV;
	return sprintf(buf, "%u\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
// we use exteranl charger. and no current control
static ssize_t set_termination_current(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 400))
		return -EINVAL;
	di->termination_currentmA = val;
	twl6030_config_iterm_reg(di, val);

	return count;
}

static ssize_t show_termination_current(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->termination_currentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_cin_limit(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 50) || (val > 1500))
		return -EINVAL;
	di->charger_incurrentmA = val;
	twl6030_config_cinlimit_reg(di, val);

	return count;
}

static ssize_t show_cin_limit(struct device *dev, struct device_attribute *attr,
								  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_incurrentmA;
	return sprintf(buf, "%u\n", val);
}

static ssize_t set_charge_current(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 300)
				|| (val > di->max_charger_currentmA))
		return -EINVAL;
	di->charger_outcurrentmA = val;
	twl6030_config_vichrg_reg(di, val);

	return count;
}

static ssize_t show_charge_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->charger_outcurrentmA;
	return sprintf(buf, "%u\n", val);
}
#endif

static ssize_t set_min_vbus(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 4200) || (val > 4760))
		return -EINVAL;
	di->min_vbus = val;
	twl6030_config_min_vbus_reg(di, val);

	return count;
}

static ssize_t show_min_vbus(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->min_vbus;
	return sprintf(buf, "%u\n", val);
}

#if defined(TWL6030_PMIC_CHARGING)
// we don't control current.
static ssize_t set_current_avg_interval(struct device *dev,
	  struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
		return -EINVAL;
	di->current_avg_interval = val;
	twl6030_current_mode_changed(di);

	return count;
}

static ssize_t show_current_avg_interval(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->current_avg_interval;
	return sprintf(buf, "%u\n", val);
}
#endif

static ssize_t set_monitoring_interval(struct device *dev,
	  struct device_attribute *attr, const char *buf, size_t count)
{
	long val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val < 10) || (val > 3600))
		return -EINVAL;
	di->monitoring_interval = val;
	twl6030_work_interval_changed(di);

	return count;
}

static ssize_t show_monitoring_interval(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	unsigned int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->monitoring_interval;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_bsi(struct device *dev,
		  struct device_attribute *attr, char *buf)
{
	int val;

	val = twl6030_get_gpadc_conversion(0);
	return sprintf(buf, "%d\n", val);
}

static ssize_t show_stat1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->stat1;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->status_int1;
	return sprintf(buf, "%u\n", val);
}

static ssize_t show_status_int2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	val = di->status_int2;
	return sprintf(buf, "%u\n", val);
}

/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14, [P940] Add a SMPL feature */
#if defined(FEATURE_SMPL)
/* LGE_CHANGE_S [hyunhee.jeon@lge.com] 2011-02-07, battery bounce function set */
static u8 get_smpl_mode(void)
{
	u8 val = 0;
	u8 ret = 0;
	
	twl_i2c_read_u8(TWL_MODULE_PM_MASTER, &val, PH_CFG_VBATLOW_REG_OFFSET);
	D("PH_CFG_VBATLOW_REG[0x%x]", val);
	if( val & BB_SEL )
		ret = 1;

	return ret;
}

static ssize_t set_smpl_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;

	/* VBAT low delay = 500 msec
	Minimum: delay x 40ms + 20ms
	Maximum: delay x 40ms + 30ms */
	u16 delay = 0xC;

	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

	di->smpl_en = val;

	if (di->smpl_en) {
		/* Select battery bounce restart mode */
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER,
				BB_SEL | delay, PH_CFG_VBATLOW_REG_OFFSET);
	} else {
		/* 1: It does not restart on a VBATLOW event */
		twl_i2c_write_u8(TWL_MODULE_PM_MASTER,
				BB_MASK, PH_CFG_VBATLOW_REG_OFFSET);
	}
	return count;
}

static ssize_t show_smpl_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val;
	struct twl6030_bci_device_info *di = dev_get_drvdata(dev);

	/* read smpl saved value */
	//lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_SMPL_EN_OFFSET, &di->smpl_en, 1);

	val = di->smpl_en;

	return sprintf(buf, "%d\n", di->smpl_en);
}
/* LGE_CHANGE_E [hyunhee.jeon@lge.com] 2011-02-07 */
#endif
/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-14, [P940] */

/* roy.park@lge.com 2010.12.8 Start-->[ */
int abnormal_vl;

void abnormal_wake_unlock_call(int value)
{
	if(value == 0)
		abnormal_vl = 0;
	else
		abnormal_vl = 1;
}

static ssize_t abnormal_wakelock_show(struct device* dev,
		struct device_attribute* attr, char* buf)
{
	return sprintf(buf,"%d\n", abnormal_vl);
}
/* roy.park@lge.com <---] */

/* LGE_CHANGE_S [kibum.lee@lge.com] 2011-02-16,
 * common : abnormal_wakelock enable
 */
int abnormal_dis_vl;
static ssize_t abnormal_wakelock_dis_show(struct device* dev,
		struct device_attribute* attr, char* buf)
{
	return sprintf(buf,"%d\n", abnormal_dis_vl);
}

static ssize_t abnormal_wakelock_dis_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long val;

	if ((strict_strtol(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

	abnormal_dis_vl = val;
	printk("abnormal_wakelock_dis_set count=%d, abnormal_dis_vl=%d\n", count, abnormal_dis_vl);
	return count;
}
/* LGE_CHANGE_E [kibum.lee@lge.com] 2011-02-16,
 * common : abnormal_wakelock enable
 */


#if defined(TWL6030_PMIC_FUELGAUGE)
static DEVICE_ATTR(fg_mode, S_IWUSR | S_IRUGO, show_fg_mode, set_fg_mode);
#endif
#if defined(TWL6030_PMIC_CHARGING)
/* FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute. */
static DEVICE_ATTR(charge_src, S_IWUSR | S_IRUGO, show_charge_src,
		set_charge_src);
#endif
static DEVICE_ATTR(vbus_voltage, S_IRUGO, show_vbus_voltage, NULL);
static DEVICE_ATTR(id_level, S_IRUGO, show_id_level, NULL);
static DEVICE_ATTR(watchdog, S_IWUSR | S_IRUGO, show_watchdog, set_watchdog);
#if defined(TWL6030_PMIC_CHARGING)
/* LGE_CHANGE [jongho3.lee@lge.com] we have external fuel gauge. */
static DEVICE_ATTR(fg_counter, S_IRUGO, show_fg_counter, NULL);
static DEVICE_ATTR(fg_accumulator, S_IRUGO, show_fg_accumulator, NULL);
static DEVICE_ATTR(fg_offset, S_IRUGO, show_fg_offset, NULL);
static DEVICE_ATTR(fg_clear, S_IWUSR, NULL, set_fg_clear);
static DEVICE_ATTR(fg_cal, S_IWUSR, NULL, set_fg_cal);
#endif
static DEVICE_ATTR(charging, S_IWUSR | S_IRUGO, NULL, set_charging);
static DEVICE_ATTR(regulation_voltage, S_IWUSR | S_IRUGO,
		show_regulation_voltage, set_regulation_voltage);
#if defined(TWL6030_PMIC_CHARGING)
static DEVICE_ATTR(termination_current, S_IWUSR | S_IRUGO,
		show_termination_current, set_termination_current);
static DEVICE_ATTR(cin_limit, S_IWUSR | S_IRUGO, show_cin_limit,
		set_cin_limit);
static DEVICE_ATTR(charge_current, S_IWUSR | S_IRUGO, show_charge_current,
		set_charge_current);
#endif
static DEVICE_ATTR(min_vbus, S_IWUSR | S_IRUGO, show_min_vbus, set_min_vbus);
static DEVICE_ATTR(monitoring_interval, S_IWUSR | S_IRUGO,
		show_monitoring_interval, set_monitoring_interval);
#if defined(TWL6030_PMIC_FUELGAUGE)
static DEVICE_ATTR(current_avg_interval, S_IWUSR | S_IRUGO,
		show_current_avg_interval, set_current_avg_interval);
#endif
static DEVICE_ATTR(bsi, S_IRUGO, show_bsi, NULL);
static DEVICE_ATTR(stat1, S_IRUGO, show_stat1, NULL);
static DEVICE_ATTR(status_int1, S_IRUGO, show_status_int1, NULL);
static DEVICE_ATTR(status_int2, S_IRUGO, show_status_int2, NULL);
/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14, [P940] Add a SMPL feature */
#if defined(FEATURE_SMPL)
/* LGE_CHANGE [wonhui.lee@lge.com] 2011-10-04, remove other's write permission*/
static DEVICE_ATTR(smpl_en, 0664, show_smpl_mode, set_smpl_mode);
#endif
/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14, [P940] */
/* roy.park@lge.com 2010.12.8 Start-->[ */
static DEVICE_ATTR(abnormal_wakelock, S_IWUSR | S_IRUGO, abnormal_wakelock_show, NULL);
/* roy.park@lge.com <---] */

/* LGE_CHANGE_S [kibum.lee@lge.com] 2011-02-16,
 * common : abnormal_wakelock enable
 */
/* LGE_CHANGE [wonhui.lee@lge.com] 2011-10-04, remove other's write permission*/
static DEVICE_ATTR(abnormal_wakelock_dis, 0664, abnormal_wakelock_dis_show, abnormal_wakelock_dis_set);
/* LGE_CHANGE_E [kibum.lee@lge.com] 2011-02-16,
 * common : abnormal_wakelock enable
 */

static struct attribute *twl6030_bci_attributes[] = {
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_fg_mode.attr,
#endif
#if defined(TWL6030_PMIC_CHARGING)
/* FIXME: [jongho3.lee@lge.com] charger src is shown in ac/usb attribute. */
	&dev_attr_charge_src.attr,
#endif
	&dev_attr_vbus_voltage.attr,
	&dev_attr_id_level.attr,
	&dev_attr_watchdog.attr,
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_fg_counter.attr,
	&dev_attr_fg_accumulator.attr,
	&dev_attr_fg_offset.attr,
	&dev_attr_fg_clear.attr,
	&dev_attr_fg_cal.attr,
#endif
	&dev_attr_charging.attr,
	&dev_attr_regulation_voltage.attr,
#if defined(TWL6030_PMIC_CHARGING)
	&dev_attr_termination_current.attr,
	&dev_attr_cin_limit.attr,
	&dev_attr_charge_current.attr,
#endif
	&dev_attr_min_vbus.attr,
	&dev_attr_monitoring_interval.attr,
#if defined(TWL6030_PMIC_FUELGAUGE)
	&dev_attr_current_avg_interval.attr,
#endif
	&dev_attr_bsi.attr,
	&dev_attr_stat1.attr,
	&dev_attr_status_int1.attr,
	&dev_attr_status_int2.attr,
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-06-14,
	 * [P940] Add a SMPL feature
	 */
#if defined(FEATURE_SMPL)
	&dev_attr_smpl_en.attr,
#endif
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-06-14, [P940] */
	&dev_attr_abnormal_wakelock.attr,	// //roy.park@lge.com
	&dev_attr_abnormal_wakelock_dis.attr,	// kibum.lee@lge.com
	NULL,
};

static const struct attribute_group twl6030_bci_attr_group = {
	.attrs = twl6030_bci_attributes,
};

static char *twl6030_bci_supplied_to[] = {
	/* [jongho3.lee@lge.com]
	 * attribute name should be battery for android. //end
	 */
	"battery",
};


static int twl6030_bci_battery_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	case POWER_SUPPLY_PROP_GAUGE_CONTROL:
#endif
	case POWER_SUPPLY_PROP_CHARGER_MODE:
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo.
	 */
	case POWER_SUPPLY_PROP_CHARGER_TEMP_CONTROL:
#ifdef BAT_TEMP_TEST
	case POWER_SUPPLY_PROP_TEMP:
#endif		
		return 1;

	default:
		break;
	}

	return 0;
}

/* MUIC mode */
typedef enum {
	BOOT_MUIC_OPEN,              // 0
	BOOT_MUIC_CHRGER,            // 1
	BOOT_MUIC_FACTORY_MODE,      // 2
	BOOT_MUIC_HAEDSET,           // 3
	BOOT_MUIC_USB,               // 4
	BOOT_MUIC_TV_OUT,            // 5
	BOOT_MUIC_FULL_USB,          // 6
	BOOT_MUIC_OTG,               // 7
	BOOT_MUIC_DEVELOP_MODE,      // 8
	BOOT_MUIC_TA_1A,             // 9
	BOOT_MUIC_RESERVE_1,
	BOOT_MUIC_RESERVE_2,
	BOOT_MUIC_END,
} UBOOT_MUIC_MODE_TYPE;

static int __init charger_state(char *str)
{
	int chg_status = simple_strtol(str, NULL, 0);
	int charging_mode;

	charging_mode = chg_status & 0x0F;
	start_cond = (chg_status & 0xF0);

	D("chg_state = %x", chg_status);

	switch(charging_mode)
	{
		case BOOT_MUIC_OPEN :
			charging_mode = POWER_SUPPLY_TYPE_BATTERY;
			break;
		case BOOT_MUIC_CHRGER:
			/* LGE_CHANGE [wonhui.lee@lge.com] 2011-11-22,
			 * Apply 1A TA charging mode
			 */
		case BOOT_MUIC_TA_1A:
			charging_mode = POWER_SUPPLY_TYPE_MAINS;
			break;
		case BOOT_MUIC_USB:
			charging_mode = POWER_SUPPLY_TYPE_USB;
			break;
		case BOOT_MUIC_FACTORY_MODE:
		default :
			charging_mode = POWER_SUPPLY_TYPE_FACTORY;
			break;
	}

	set_boot_charging_mode(charging_mode);
	return 1;
}
__setup("chg=", charger_state);

/* LGE_CHANGE_S [seungho1.park@lge.com] 2011-11-21,
 * dont need to init mux again in LGE board.
 */
#if defined(CONFIG_MUIC)
static struct muic_client_ops twl6030_bci_ops = {
	.on_none = charger_fsm2,
	.on_na_ta = charger_fsm2,
	.on_lg_ta = charger_fsm2,
	.on_cp_uart = charger_fsm2,
	.on_ap_usb = charger_fsm2,
	.on_cp_usb = charger_fsm2,
	.on_mhl = charger_fsm2,
};
#endif
/* LGE_CHANGE_E [seungho1.park@lge.com] ,  */


static int __devinit twl6030_bci_battery_probe(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl6030_bci_device_info *di;
	int ret;
	struct twl6030_gpadc_request req;
	u8 controller_stat = 0;
#if defined(GET_BET_TEMP)
	int adc_code;
#endif

	D("twl6030_bci_battery_probe................");

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform_data not available\n",
				__func__);
		return -EINVAL;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "%s: no memory\n", __func__);
		return -ENOMEM;
	}

	/* LGE_CHANGE_S [jongho3.lee@lge.com]
	 * expose di struct for external charger..
	 */
#if defined(CHARGING_WAKE_LOCK)
	wake_lock_init(&di->charger_wake_lock, WAKE_LOCK_SUSPEND,
			"charger_wakelock");
	wake_lock_init(&di->off_mode_charger_wake_lock, WAKE_LOCK_SUSPEND,
			"off_mode_charger_wakelock");
#endif

	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * for fuel gauge reset on CP we need one gpio.
	 */
	/* LGE_SJIT 2012-01-25 [dojip.kim@lge.com]
	 * FIXME: Really need? Fuel gauge is on AP
	 * Use platform data, enable gpio if # of gpio is not zero (buggy)
	 */
#if !defined(CONFIG_MACH_LGE_COSMO)
	if (pdata->gpio_omap_send > 0) {
		ret = gpio_request(pdata->gpio_omap_send, "omap_send");
		if (ret < 0) {
			dev_warn(&pdev->dev,
					"%s: Failed to request GPIO %d\n",
					__func__, pdata->gpio_omap_send);
		}
		gpio_direction_output(pdata->gpio_omap_send, 0);
	}
#endif

	p_di = di;
	di->charger_interrupt = true;
	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	di->capacity_validation = 50;
	di->capacity = 1;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-27,
	 * [P940] Capacity ui update error fix
	 */
	di->ui_capacity = 1;

	di->modem_alive = 1;
	di->charger_source = POWER_SUPPLY_TYPE_BATTERY;
	di->charger_logo_status = CHARGER_LOGO_STATUS_UNKNOWN;
	di->wake_lock_count = 0;
	di->battery_present = true;
	di->temp_C = TEMP_LOW_NO_BAT - 100;
#if defined(CONFIG_MAX8971_CHARGER)
#if defined(GET_TEMP_SENSOR_DOCOMO)
	di->temp_charger_C = 0;
	di->temp_parm_C = 0;
	//di->temp_charger_adc = 0;
	//di->temp_parm_adc = 0;
	di->temp_high_C = 0;
	di->temp_low_C = 0;
#endif
#endif
	di->off_mode_timer_working = 0;
	recharging_status = RECHARGING_WAIT_UNSET;
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-16,
	 * [P940] Add gauge control feature.
	 */
#if defined(FEATURE_GAUGE_CONTROL)
	di->gauge_control_count = 0;
#endif

#if defined (CHANGE_CHG_MODE_ON_OVERHEAT_SCENE)    
	di->lock_on_overheat_scene = 0;
#endif
	di->temp_control = 0xFF;

	di->factory_control = FACTORY_CHARGER_ENABLE;
	di->cap_seamless = 0;

	/* [jongho3.lee] WBT test fix. */
	//di->avg_volt_queue[AVG_VOLT_QUEUE_SIZE];
	di->avg_volt_queue_count = 0;
	di->avg_volt_head = 0;
	di->avg_volt_tail = 0;
	di->avg_voltage_mV = 0;
	di->pmic_capacity = 0;
	//memset((di->avg_volt_queue), 0, sizeof(int)*AVG_VOLT_QUEUE_SIZE);
	/* LGE_CHANGE_E [jongho3.lee@lge.com]
	 * expose di struct for external charger..
	 */

	if (pdata->monitoring_interval == 0) {
		di->monitoring_interval = 10;
		di->current_avg_interval = 10;
	} else {
		di->monitoring_interval = pdata->monitoring_interval;
		di->current_avg_interval = pdata->monitoring_interval;
	}

	di->max_charger_currentmA = pdata->max_charger_currentmA;
	di->max_charger_voltagemV = pdata->max_bat_voltagemV;
	di->termination_currentmA = pdata->termination_currentmA;
	di->regulation_voltagemV = pdata->max_bat_voltagemV;
	di->low_bat_voltagemV = pdata->low_bat_voltagemV;
	di->battery_tmp_tbl = pdata->battery_tmp_tbl;
	di->tblsize = pdata->tblsize;

	di->dev = &pdev->dev;
	/* [jongho3.lee@lge.com]
	 * attribute name should be battery for android.
	 */

	di->bat.name = "battery";
	di->bat.supplied_to = twl6030_bci_supplied_to;
	di->bat.num_supplicants = ARRAY_SIZE(twl6030_bci_supplied_to);
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = twl6030_bci_battery_props;
	di->bat.num_properties = ARRAY_SIZE(twl6030_bci_battery_props);
	di->bat.get_property = twl6030_bci_battery_get_property;
	// [jongho3.lee@lge.com] add to set_property. //start
	di->bat.set_property = twl6030_bci_battery_set_property;
	di->bat.property_is_writeable = twl6030_bci_battery_property_is_writeable;
	// [jongho3.lee@lge.com] add to set_property. //end
	di->bat.external_power_changed =
	twl6030_bci_battery_external_power_changed;

	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * replacing charger usb_bat supply type. [START_LGE]
	 */
	di->usb.name = "usb";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = twl6030_usb_props;
	di->usb.num_properties = ARRAY_SIZE(twl6030_usb_props);
	di->usb.get_property = twl6030_usb_get_property;

	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * replacing charger usb_bat supply type. [START_LGE]
	 */
	di->ac.name = "ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.properties = twl6030_ac_props;
	di->ac.num_properties = ARRAY_SIZE(twl6030_ac_props);
	di->ac.get_property = twl6030_ac_get_property;

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
#ifdef FEATURE_BK_BATTERY
	/* [jongho3.lee@lge.com]
	 * attribute name should be bk_battery for android. //start
	 */
	di->bk_bat.name = "bk_battery";
	/* [jongho3.lee@lge.com]
	 * attribute name should be bk_battery for android. //end
	 */
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	di->bk_bat.type = POWER_SUPPLY_TYPE_UPS;
	di->bk_bat.properties = twl6030_bk_bci_battery_props;
	di->bk_bat.num_properties = ARRAY_SIZE(twl6030_bk_bci_battery_props);
	di->bk_bat.get_property = twl6030_bk_bci_battery_get_property;
#endif

	di->vac_priority = 2;

	platform_set_drvdata(pdev, di);

	/* settings for temperature sensing */
	ret = twl6030battery_temp_setup();
	if (ret)
		goto temp_setup_fail;


	/* request charger ctrl interruption */
	di->irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(di->irq, NULL,
			twl6030charger_ctrl_interrupt, 0, "twl_bci_ctrl", di);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"%s: could not request irq %d, status %d\n",
				__func__, di->irq, ret);
		goto chg_irq_fail;
	}

	ret = power_supply_register(&pdev->dev, &di->bat);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to register main battery\n",
				__func__);
		goto batt_failed;
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&notifier_list);

#if defined(CREATE_OWN_WORKQUEUE)
	di->charger_workqueue = create_workqueue("charger_workqueue");
	if (!di->charger_workqueue) {
		dev_err(&pdev->dev, "%s: failed to create workqueue\n",
				__func__);
		goto workqueue_create_failed;
	}
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&di->twl6030_bci_monitor_work,
				twl6030_bci_battery_work);
	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 0);

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-06-09,
	 * [P940] Add Charger IC(Max8971) in Rev.A
	 */
#if defined(CONFIG_CHARGER_RT9524)|| defined(CONFIG_MAX8971_CHARGER)
	INIT_DELAYED_WORK_DEFERRABLE(&di->charger_work,
				charging_battery_work);
	charger_schedule_delayed_work(&di->charger_work, HZ/3);

	INIT_DELAYED_WORK_DEFERRABLE(&di->gauge_reset_work,
				battery_gauge_reset_work);

	INIT_DELAYED_WORK_DEFERRABLE(&di->power_off_work,
				battery_power_off_work);

	INIT_DELAYED_WORK_DEFERRABLE(&di->charger_timer_work,
				charging_timer_work);
	/* LGE_CHANGE [euiseop.shin@lge.com]
	 * 2011-07-06,[P940] Imported from Cosmopolitan
	 */
	INIT_DELAYED_WORK_DEFERRABLE(&di->twl_power_supply_changed_work,
				twl_power_supply_changed_work);
#endif
#if defined(CONFIG_MAX8971_CHARGER)
	INIT_DELAYED_WORK_DEFERRABLE(&di->twl6030_recharging_work,
				twl6030_max8971_recharging_work);
#endif

	ret = power_supply_register(&pdev->dev, &di->usb);
	if (ret) {
		dev_err(&pdev->dev,
				"%s: failed to register usb power supply\n",
				__func__);
		goto usb_failed;
	}

	ret = power_supply_register(&pdev->dev, &di->ac);
	if (ret) {
		dev_err(&pdev->dev,
				"%s: failed to register ac power supply\n",
				__func__);
		goto ac_failed;
	}

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
#ifdef FEATURE_BK_BATTERY
	ret = power_supply_register(&pdev->dev, &di->bk_bat);
	if (ret) {
		dev_err(&pdev->dev,
				"%s: failed to register backup battery\n",
				__func__);
		goto bk_batt_failed;
	}
#endif
	di->charge_n1 = 0;
	di->timer_n1 = 0;


	/* LGE_CHANGE [seungho1.park@lge.com] 2011-11-21, */
#if defined(CONFIG_MUIC)	
	muic_client_dev_register(pdev->name, di, &twl6030_bci_ops);
#endif

#if defined(TWL6030_PMIC_CHARGING)
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * we don't need and don't have the way to monitor bat_current.
	 */
	INIT_DELAYED_WORK_DEFERRABLE(&di->twl6030_current_avg_work,
						twl6030_current_avg);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work, 500);
#endif

	ret = twl6030battery_voltage_setup();
	if (ret < 0) {
		dev_warn(&pdev->dev,
				"%s: voltage measurement setup failed\n",
				__func__);
		D("voltage measurement setup failed\n");
	}
	else {
		D("voltage measurement setup done\n");
	}

#if defined(TWL6030_PMIC_CHARGING)
	/* LGE_CHANGE [jongho3.lee@lge.com]
	 * we don't need and don't have the way to monitor bat_current.
	 */
	ret = twl6030battery_current_setup();
	if (ret < 0)
		dev_warn(&pdev->dev,
				"%s: current measurement setup failed\n",
				__func__);
#endif

	/* initialize for USB charging */
	twl6030_config_limit1_reg(di, pdata->max_charger_voltagemV);
	twl6030_config_limit2_reg(di, di->max_charger_currentmA);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			CONTROLLER_INT_MASK);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MASK_MCHARGERUSB_THMREG,
			CHARGERUSB_INT_MASK);

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &controller_stat,
			CONTROLLER_STAT1);

	di->charger_incurrentmA = di->max_charger_currentmA;
	di->charger_outcurrentmA = di->max_charger_currentmA;
	di->watchdog_duration = 32;
	di->voltage_uV = twl6030_get_gpadc_conversion(7);
//	dev_info(&pdev->dev, "Battery Voltage at Bootup is %d mV\n",di->voltage_uV);

	/* LGE_CHANGE [jongho3.lee@lge.com] we use external charger. */

	ret = twl6030backupbatt_setup();
	if (ret < 0)
		dev_warn(&pdev->dev,
				"%s: Backup Bat charging setup failed\n",
				__func__);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	/*
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);
	*/

	ret = sysfs_create_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "%s: could not create sysfs files\n",
				__func__);
		goto sysfs_create_failed;
	}

	/* FIXME : [jongho3.lee@lge.com]
	 * need to fix with PMIC datasheet which I don't have now.
	 */
#if defined(GET_BET_TEMP)
	req.channels = (1 << BAT_TEMP_CHANNEL);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	get_adc_value(&req, req.channels);

	adc_code = req.rbuf[BAT_TEMP_CHANNEL];
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-08-20,
	 * add to check BATT TEMP ADC value
	*/
	di->batt_temp_adc = adc_code;
	di->temp_C = average_temp(reference_graph((s64)adc_code, battery_temp_graph, ARRAY_SIZE(battery_temp_graph)) / (TEMP_TIMES / 10));
	if(di->temp_C < TEMP_LOW_NO_BAT) {
		di->battery_present = 0;
	}
#endif

#ifdef FEATURE_SMPL
	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-07-12, apply SMPL Setting*/
	di->smpl_en = get_smpl_mode();
#endif

	// mask vbat low interrupt
	twl6030_interrupt_mask(TWL6030_VBAT_LOW_MASK, REG_INT_MSK_LINE_A);

	//unmask PREQ transition
	twl_i2c_write_u8(0x0D, 0x00, 0x20);

	//USB_VBUS_CTRL_CLR
	twl_i2c_write_u8(0x0E, 0xFF, 0x05);
	//USB_ID_CRTL_CLR
	twl_i2c_write_u8(0x0E, 0xFF, 0x07);
	twl_i2c_write_u8(0x0D, 0xC3, 0x31);

	//MISC1
	// TODO. after add VBAT_MEAS, david.seo, kibum.lee
	//twl_i2c_write_u8(0x0D, 0x00, 0xE4);


	/* BBSPOR_CFG - Disable BB charging.
	 * It should be taken care by proper driver
	 */
	//twl_i2c_write_u8(0x0D, 0x62, 0xE6);

	//CFG_INPUT_PUPD2
	twl_i2c_write_u8(0x0D, 0x65, 0xF1);

	//CFG_INPUT_PUPD4
	twl_i2c_write_u8(0x0D, 0x00, 0xF3);

	//CFG_LDO_PD2
	twl_i2c_write_u8(0x0D, 0x00, 0xF5);

	//CHARGERUSB_CTRL3
	twl_i2c_write_u8(0x0E, 0x21, 0xEA);

	//TOGGLE2  100uA
	twl_i2c_write_u8(0x0E, 0x40, 0x91);
	//TOGGLE3
	twl_i2c_write_u8(0x0E, 0x00, 0x92);

	//Broadcast sleep state to MOD, CON
	twl_i2c_write_u8(0x0D, 0xC3, 0x31);

#if 0 
	/* LGE_CHANGE_S [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	//CLK32KG
	twl_i2c_write_u8(0x0D, 0x01, 0xBC);
	twl_i2c_write_u8(0x0D, 0x05, 0xBD);
	twl_i2c_write_u8(0x0D, 0x21, 0xBE);

	//CLK32KAUDIO
	twl_i2c_write_u8(0x0D, 0x01, 0xBF);
	twl_i2c_write_u8(0x0D, 0x05, 0xC0);
	twl_i2c_write_u8(0x0D, 0x21, 0xC1);

	//VUSB
	twl_i2c_write_u8(0x0D, 0x01, 0xA0);
	twl_i2c_write_u8(0x0D, 0x01, 0xA1);
	twl_i2c_write_u8(0x0D, 0x21, 0xA2);

	//VPP
	twl_i2c_write_u8(0x0D, 0x7F, 0xF4); //disable internal PD for VPP LDO
	twl_i2c_write_u8(0x0D, 0x00, 0x9C);
	twl_i2c_write_u8(0x0D, 0x00, 0x9D);
	twl_i2c_write_u8(0x0D, 0x00, 0x9E);

#if defined(CONFIG_MACH_LGE_VMMC_ALWAYSON_FORCED)
	/* 20110504 FW1 KIMBYUNGCHUL
	 * SD_CARD_LOCKUP_IN_omap_hsmmc_resume_FUNC	 [START]
	 */

	twl_i2c_write_u8(0x0D, 0x01, 0x98);
	twl_i2c_write_u8(0x0D, 0x15, 0x99);
	twl_i2c_write_u8(0x0D, 0xE1, 0x9A);
#else
#if defined(CONFIG_MACH_LGE_MMC_ALWAYSON)
	twl_i2c_write_u8(0x0D, 0x01, 0x98);
	twl_i2c_write_u8(0x0D, 0x3F, 0x99);
	twl_i2c_write_u8(0x0D, 0x21, 0x9A);
#else
	//
#endif
#endif	//CONFIG_MACH_LGE_VMMC_ALWAYSON_FORCED
	/* 20110504 FW1 KIMBYUNGCHUL
	 * SD_CARD_LOCKUP_IN_omap_hsmmc_resume_FUNC	 [END]
	 */
	/* LGE_CHANGE_E [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
#endif

	dev_info(&pdev->dev, "twl6030 bci probed\n");

	return 0;

	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
sysfs_create_failed:
#ifdef FEATURE_BK_BATTERY
	power_supply_unregister(&di->bk_bat);
bk_batt_failed:
#endif
	power_supply_unregister(&di->ac);
ac_failed:
	power_supply_unregister(&di->usb);
usb_failed:
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	power_supply_unregister(&di->bat);
batt_failed:
#ifdef CREATE_OWN_WORKQUEUE
//nthyunjin.yang 120709 temp block for Kernel Panic (work queue)
//	destroy_workqueue(di->charger_workqueue);
workqueue_create_failed:
#endif
	free_irq(di->irq, di);
chg_irq_fail:
temp_setup_fail:
#if defined(CHARGING_WAKE_LOCK)
	wake_lock_destroy(&di->charger_wake_lock);
	wake_lock_destroy(&di->off_mode_charger_wake_lock);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(di);
	p_di = NULL;

	return ret;
}

static int __devexit twl6030_bci_battery_remove(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
#if defined(CONFIG_MAX8971_CHARGER)
	int irq;
#endif

	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);

#if defined(CONFIG_MAX8971_CHARGER)
	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);
#endif
	sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
#if defined(TWL6030_PMIC_CHARGING)
	cancel_delayed_work(&di->twl6030_current_avg_work);
#endif
#if defined(CONFIG_LG_FW_RT9524_CHARGER) || defined(CONFIG_LG_FW_MAX8971_CHARGER)|| defined(CONFIG_MAX8971_CHARGER) || defined(CONFIG_CHARGER_RT9524)
	cancel_delayed_work(&di->charger_work);
	cancel_delayed_work(&di->gauge_reset_work);
	cancel_delayed_work(&di->power_off_work);
	cancel_delayed_work(&di->charger_timer_work);
	cancel_delayed_work(&di->twl_power_supply_changed_work);
#endif
#if defined(CONFIG_MAX8971_CHARGER)
	cancel_delayed_work(&di->twl6030_recharging_work);
#endif
	flush_scheduled_work();

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
#ifdef FEATURE_BK_BATTERY
	power_supply_unregister(&di->bk_bat);
#endif
#ifdef CREATE_OWN_WORKQUEUE
//nthyunjin.yang 120709 temp block for Kernel Panic (work queue)
//	destroy_workqueue(di->charger_workqueue);
#endif
	free_irq(di->irq, di);

#if defined(CHARGING_WAKE_LOCK)
	wake_lock_destroy(&di->charger_wake_lock);
	wake_lock_destroy(&di->off_mode_charger_wake_lock);
#endif

	platform_set_drvdata(pdev, NULL);
	kfree(di);
	p_di = NULL;

	return 0;
}

static void twl6030_bci_battery_shutdown(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
#if defined(CONFIG_MAX8971_CHARGER)
	int irq;
#endif

	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);
#if defined(CONFIG_MAX8971_CHARGER)

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);
#endif

	sysfs_remove_group(&pdev->dev.kobj, &twl6030_bci_attr_group);
	cancel_delayed_work(&di->twl6030_bci_monitor_work);
#if defined(CONFIG_MAX8971_CHARGER)
	cancel_delayed_work(&di->twl6030_recharging_work);
#endif
#if defined(TWL6030_PMIC_CHARGING)
	cancel_delayed_work(&di->twl6030_current_avg_work);
#endif
	/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while power on/off error [START_LGE]
	 */
#if defined(CONFIG_LG_FW_RT9524_CHARGER) || defined(CONFIG_LG_FW_MAX8971_CHARGER) || defined(CONFIG_MAX8971_CHARGER) || defined(CONFIG_CHARGER_RT9524)
	cancel_delayed_work(&di->charger_work);
	cancel_delayed_work(&di->gauge_reset_work);
	cancel_delayed_work(&di->power_off_work);
	cancel_delayed_work(&di->charger_timer_work);
	cancel_delayed_work(&di->twl_power_supply_changed_work);
#endif
	/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while ower on/off error [END_LGE]
	 */
	flush_scheduled_work();

	/* [jongho3.lee@lge.com] */
#if 1
	/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while ower on/off error 
	 */
	mutex_lock(&charging_fsm_lock);
#if !defined(CONFIG_MAX8971_CHARGER)
	DCHG("charger deactive check point 15 \n");
	charging_ic_deactive();
#else
	if(di->batt_temp_adc < 1500)
	{ 
		D("[twl6030] max8971_stop_charging 17 \n");
                 max8971_stop_charging();
	}
#endif
	di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while power on/off error.
	 */
	charger_fsm_shutdown = 1;
	/* LGE_CHANGE [byoungcheol.lee@lge.com] 2011-11-02,
	 * Fixed lockup issue while ower on/off error 
	 */
	mutex_unlock(&charging_fsm_lock);
#endif

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-05-31,
	 * LGE_P940, remove bk_battery.
	 */
#ifdef FEATURE_BK_BATTERY
	power_supply_unregister(&di->bk_bat);
#endif
#ifdef CREATE_OWN_WORKQUEUE
//nthyunjin.yang 120709 temp block for Kernel Panic (work queue)
//	destroy_workqueue(di->charger_workqueue);
#endif

#if defined(CHARGING_WAKE_LOCK)
	wake_lock_destroy(&di->charger_wake_lock);
	wake_lock_destroy(&di->off_mode_charger_wake_lock);
#endif

	free_irq(di->irq, di);

	platform_set_drvdata(pdev, NULL);
	kfree(di);
	p_di = NULL;

	return;
}

#ifdef CONFIG_PM
static int twl6030_bci_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	u8 rd_reg;

	/* mask to prevent wakeup due to 32s timeout from External charger */
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg |= MVAC_FAULT;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);

	cancel_delayed_work(&di->twl6030_bci_monitor_work);
	cancel_delayed_work(&di->twl6030_current_avg_work);
	//euiseop.shin delete	flush_scheduled_work();

	pdev->dev.power.power_state = state;

	/* LGE_CHANGE_S [byoungcheol.lee@lge.com] 2011-12-08,
	 * To change the charging mode according battery SOC
	 */
#ifdef CHANGE_CHG_MODE_ON_OVERHEAT_SCENE
	di->lock_on_overheat_scene = 0;
#endif
	/* LGE_CHANGE_E [byoungcheol.lee@lge.com] 2011-12-08,
	 * To change the charging mode according battery SOC
	 */

	return 0;
}

static int twl6030_bci_battery_resume(struct platform_device *pdev)
{
	struct twl6030_bci_device_info *di = platform_get_drvdata(pdev);
	u8 rd_reg;

	/* LGE_CHANGE [wonhui.lee@lge.com] 2011-09-29,
	 * recover twl6030 registers to the initial data
	 */
#if 1
	twl6030battery_temp_setup();
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-04-13,
	 * LGE_P940, Bring in Cosmo.
	 */
	/* LGE_CHANGE_S [david.seo@lge.com] 2011-03-26,
	 * common : move from pm44xx.c
	 */
	/* currently disabling of below module is done in pm44xx.c
	 * .prepare() instead of twl6030_bci_battery_resume() need a revisit.
	 */
	//GPADC_CTRL
	twl_i2c_write_u8(0x0E, 0xFF, 0x2E);
	//TOGGLE1
	twl_i2c_write_u8(0x0E, 0xA2, 0x90);
	//MISC2
	twl_i2c_write_u8(0x0D, 0x10, 0xE5);
	/* LGE_CHANGE_ [david.seo@lge.com] 2011-03-26,
	 * common : move from pm44xx.c
	 */
#endif

	//euiseop.shin deleted by cosmo	di->cap_seamless = 0;

	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &rd_reg, CONTROLLER_INT_MASK);
	rd_reg &= ~(0xFF & MVAC_FAULT);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
						CONTROLLER_INT_MASK);

	charger_schedule_delayed_work(&di->twl6030_bci_monitor_work, 0);
	charger_schedule_delayed_work(&di->twl6030_current_avg_work, 50);
	/* LGE_CHANGE [euiseop.shin@lge.com] 2011-07-06,
	 * [P940] Imported from Cosmopolitan
	 */
	charger_schedule_delayed_work(&(p_di->twl_power_supply_changed_work), 0);

	pdev->dev.power.power_state = PMSG_ON;

	return 0;
}
#else
#define twl6030_bci_battery_suspend	NULL
#define twl6030_bci_battery_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver twl6030_bci_battery_driver = {
	.probe		= twl6030_bci_battery_probe,
	.remove		= __devexit_p(twl6030_bci_battery_remove),
	.shutdown	= twl6030_bci_battery_shutdown,
	.suspend	= twl6030_bci_battery_suspend,
	.resume		= twl6030_bci_battery_resume,
	.driver		= {
		.name	= "twl6030_bci",
	},
};

static int __init twl6030_battery_init(void)
{
	return platform_driver_register(&twl6030_bci_battery_driver);
}
module_init(twl6030_battery_init);

static void __exit twl6030_battery_exit(void)
{
	platform_driver_unregister(&twl6030_bci_battery_driver);
}
module_exit(twl6030_battery_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl6030_bci");
MODULE_AUTHOR("Texas Instruments Inc");
