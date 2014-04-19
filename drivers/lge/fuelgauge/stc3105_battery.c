/*
 *  stc3105_battery.c 
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2011 STMicroelectronics.
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
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/lge/stc3105_battery.h>
#include <linux/slab.h>

/*Function declaration*/
int STC310x_SetPowerSavingMode(void);
int STC310x_StopPowerSavingMode(void);
int STC310x_AlarmSet(void);
int STC310x_AlarmStop(void);
int STC310x_AlarmGet(void);
int STC310x_AlarmClear(void);
int STC310x_AlarmSetVoltageThreshold(int VoltThresh);
int STC310x_AlarmChangeVoltageThreshold(int VoltThresh);
int STC310x_AlarmSetSOCThreshold(int SOCThresh);
int STC310x_AlarmChangeSOCThreshold(int SOCThresh);
int STC310x_RelaxTmrSet(int CurrentThreshold);
int STC310x_Reset(void);

#define GG_VERSION "1.6"

/****************************************************************************/
/*        CURRENT AND VOLTAGE SENSING PARAMETERS                            */
/*   TO BE DEFINED ACCORDING TO HARDWARE IMPLEMENTATION                     */
/*--------------------------------------------------------------------------*/

#define SENSERESISTOR   20 /* current sense resistor in milliOhms (10min, 100max) */
#define VINRESISTOR   1000 /* VIN serial resistor in Ohms (0min, 5000max)         */

/* note:  use 0 for VINRESISTOR if no R C filter is used on VIN             */
/****************************************************************************/

/****************************************************************************/
/*        BATTERY PARAMETERS                                                */
/*   TO BE DEFINED ACCORDING TO BATTERY CHARACTERISTICS                     */
/*--------------------------------------------------------------------------*/

#define BATT_DEF_CAPACITY  1508	/* battery nominal capacity in mAh          */
#define BATT_INT_RESIST     280	/* nominal internal resistance (milliOhms)  */
#define APP_MIN_VOLTAGE    3200	/* application cut-off voltage              */
/* % capacity derating at 60, 40, 25, 10, 0, -10 °C  for typ current        */
#define CAPDERATING { 0, 0, 1, 3, 11, 28 }
#define APP_TYP_CURRENT   (-250) /* typ application current consumption in mA ( <0 !) */

/* tables of open-circuit voltage (OCV) in mV and state-of-charge (SOC) levels in % */
/* OCV and SOC tables are in decreasing order, NTABLE define the number of points:  */
#define NTABLE		9 /* number of points in OCV and SOC tables         */
#define OCVTABLE	{ 4170, 4060, 3980, 3900, 3810, 3760, 3690, 3630, 3490 }
#define SOCTABLE	{  100,   90,   80,   65,   50,   25,   10,    3,    0 }
/****************************************************************************/

/****************************************************************************/
/*        ALARM PARAMETERS                                                  */
/*   TO BE DEFINED ACCORDING TO APPLICATION REQUIREMENTS                    */
/*--------------------------------------------------------------------------*/

#define BATT_LOW_SOC_ALM     20	/* low battery SOC alarm (in %) using IO0 output (set to 0 if not used) */
#define BATT_LOW_VOLT_ALM  3300	/* low battery voltage alarm (mV) using IO0 output (set to 0 if not used) */
/****************************************************************************/

/****************************************************************************/
/*        INTERNAL PARAMETERS                                               */
/*   TO BE ADJUSTED ACCORDING TO BATTERY/APPLICATION CHARACTERISTICS        */
/*--------------------------------------------------------------------------*/

#define BATT_CHG_VOLTAGE   4100	/* min voltage at the end of the charge (mV)      */
#define BATT_MAX_SOC        100	/* corresponding SOC at battery full (%)          */
#define BATT_MIN_VOLTAGE   3300	/* nearly empty battery detection level (mV)      */
#define CHG_MIN_CURRENT     150	/* min charge current in mA                       */
#define CHG_END_CURRENT      20	/* end charge current in mA                       */
#define APP_MIN_CURRENT     (-5) /* minimum application current consumption in mA ( <0 !) */
#define BATT_COMP_CURRENT  (-50) /* temperature compensation current threshold     */
#define BATT_LOW_CURRENT   (-75) /* battery max discharge current for OCV measurement in mA(<0) */
#define BATT_RELAX_TIME     600	/* battery relaxation time before OCV measurement in sec */
#define RITEMPTABLE     { 5, 8, 10, 15, 20, 30 } /* normalized Ri at 60, 40, 25, 10, 0, -10°C */
#define AVG_WEIGHT	3
#define	AVG_THR		3
#define	AVG_LEN		10
#define AVR_CURRENT_THR	150
/****************************************************************************/

/* Private define ----------------------------------------------------------*/

#define STC310x_SLAVE_ADDRESS		0xE0	/* STC310x 8-bit address byte */

/*Address of the STC310x register -------------------------------------------*/
#define STC310x_REG_MODE		0x00	/* Mode Register             */
#define STC310x_REG_CTRL		0x01	/* Control and Status Register */
#define STC310x_REG_CHARGE		0x02	/* Gas Gauge Charge Data (2 bytes) */
#define STC310x_REG_COUNTER		0x04	/* Number of Conversion (2 bytes) */
#define STC310x_REG_CURRENT		0x06	/* Battery Current (2 bytes) */
#define STC310x_REG_VOLTAGE		0x08	/* Battery Voltage (2 bytes) */

#define STC310x_REG_SOC_BASE		0x0A	/* SOC base value (2 bytes)     */
#define STC310x_REG_ALARM_SOC		0x0C	/* SOC alarm level (2 bytes)    */
#define STC310x_REG_ALARM_VOLTAGE	0x0E	/* Low voltage alarm level      */
#define STC310x_REG_CURRENT_THRES	0x0F	/* Current threshold for relaxation */
#define STC310x_REG_RELAX_COUNT		0x10	/* Voltage relaxation counter   */

/*Bit mask definition*/
#define STC310x_PWR_SAVE		0x04	/* Power saving bit mask     */
#define STC310x_ALM_ENA			0x08	/* Alarm enable bit mask     */

#define STC310x_REG_ID			0x18	/* Chip ID (1 byte)       */
#define STC310x_ID			0x12	/* STC3105 ID */

#define STC310x_REG_RAM			0x20	/* General Purpose RAM Registers */
#define RAM_SIZE			16	/* Total RAM size of STC3105 in bytes */

#define M_EOC  0x0400		/* GG_EOC mask in STC310x_BattDataTypeDef status word */
#define M_STAT 0x1010		/* GG_RUN & PORDET mask in STC310x_BattDataTypeDef status word */
#define M_RUN  0x0010		/* GG_RUN mask in STC310x_BattDataTypeDef status word */

#define OK 0

/* Battery charge state definition for BattState */
#define  BATT_CHARGING  3
#define  BATT_ENDCHARG  2
#define  BATT_FULCHARG  1
#define  BATT_IDLE      0
#define  BATT_DISCHARG (-1)
#define  BATT_LOWBATT  (-2)

/* STC310x RAM test word */
#define RAM_TSTWORD 0x53A9

/* Gas gauge states */
#define GG_INIT     'I'
#define GG_RUNNING  'R'
#define GG_POWERDN  'D'

/* gas gauge structure definition ------------------------------------*/

#ifndef TEST_DRIVER
typedef struct {
	int Voltage;		/* battery voltage in mV */
	int Current;		/* battery current in mA */
	int Temperature;	/* battery temperature in 0.1°C */
	int ChargeValue;	/* battery absolute state-of-charge value (mAh) */
	int ChargePercentNC;	/* battery relative state-of-charge value (%) */
	int ChargePercent;	/* battery relative state-of-charge value (%) */
	int RemTime;		/* battery remaining operating time during discharge (min) */
	int ChargeNominal;	/* battery full charge capacity (mAh) */
	signed char State;	/* charge (>0)/discharge(<0) state */
} GasGauge_DataTypeDef;
#endif

/* gas gauge structure definition -------------------------------------------*/

/* Private constants --------------------------------------------------------*/

static const int SocPoint[NTABLE] = SOCTABLE;
static const int OcvPoint[NTABLE] = OCVTABLE;

#define NTEMP 6
static const int CapacityDerating[NTEMP] = CAPDERATING;
static const int RiTempTable[NTEMP] = RITEMPTABLE;
static const int TempTable[NTEMP] = { 60, 40, 25, 10, 0, -10 };	/* temperature table from 60°C to -10°C (descending order!) */

/* Private variables --------------------------------------------------------*/

/* structure of the STC310x battery monitoring data */
typedef struct {
	short Voltage;		/* voltage in mV            */
	short Current;		/* current in mA            */
	short Temperature;	/* temperature in 0.1°C     */
	short Charge;		/* Coulomb counter charge in mA.h   */
	short ConvCounter;	/* convertion counter       */
	short RelaxTimer;
	short Status;		/* status word              */
} STC310x_BattDataTypeDef;

static STC310x_BattDataTypeDef BattData;	/* STC310x data */

/* Gas Gauge local variables */
static int BattState;		/* battery GG state              */
static int BattChargeNominal;	/* battery nominal charge (mAh)  */
static int BattChargeValue;	/* battery charge value (mAh)    */
static char BattFullCharge;	/* full charge done              */
static char BattFullDischarge;	/* full discharge done           */
static char BattOCVcomp;	/* OCV compensation done */
static char BattCalStat;	/* capacity calibration done     */
static int BattRelaxTime;	/* battery relaxation time in 0.5s units */
static int BattAvgOCV;		/* battery averaged OCV */
static int BattAvrChargePerccent[4];	/* battery average charge value in percent */
static int NbrOfTask;		/*Number of task already done */
static int SocThreshold;	/*SOC alarm threshold */
static int VoltageThreshold;	/*OCV alarm threshold */

/* structure of the STC310x RAM registers for the Gas Gauge algorithm data */
static union {
	unsigned char db[RAM_SIZE];	/* last byte holds the CRC */
	struct {
		short int TstWord;		/* 0-1 */
		short int BattCap;		/* 2-3 */
		short int BaseCharge;		/* 4-5 */
		short int AvrOCV;		/* 6-7 */
		char Status;			/* 8  */
		char MinChargePercent;		/* 9  */
		char AvrCount;			/*10 */
		short int PreviousCurrent;	/*11-12 */
		char AvrStatus;	/*13 */
		/* bytes ..RAM_SIZE-2 are free, last byte RAM_SIZE-1 is the CRC */
	} reg;
} GG_Ram;

#define STC3100_DELAY		1000
//Used between two work functions, timming accuracy is not mandatory, several seconds is OK 5 to 10 for instance
#define STC3100_DELAY_FOR_FIRST_TASK	200
// This constant has to changed to be equal to 1.2s or 1.3s of delay. 

#define STC3100_BATTERY_FULL	95

struct stc31xx_chip {
	struct i2c_client *client;
	struct delayed_work work;
	struct power_supply battery;
	struct stc31xx_platform_data *pdata;

	/* State Of Connect */
	int online;
	/* battery SOC (capacity) */
	int batt_soc;
	/* battery voltage */
	int batt_voltage;
	/* Current */
	int batt_current;
	/* State Of Charge */
	int status;

};

static int stc31xx_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct stc31xx_chip *chip = container_of(psy,
						 struct stc31xx_chip, battery);

/* from power_supply.h:
 * All voltages, currents, charges, energies, time and temperatures in uV,
 * ÂµA, ÂµAh, ÂµWh, seconds and tenths of degree Celsius unless otherwise
 * stated. It's driver's job to convert its raw values to units in which
 * this class operates.
 */

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->batt_voltage * 1000;	/* in uV */
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->batt_current * 1000;	/* in uA */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->batt_soc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* -------------------------------------------------------------------------- */
/* I2C interface */

static struct i2c_client *sav_client;

static int STC310x_Write(int length, int reg, unsigned char *values)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(sav_client, reg, length, values);
	if (ret < 0)
		dev_err(&sav_client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int STC310x_Read(int length, int reg, unsigned char *values)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(sav_client, reg, length, values);
	if (ret < 0)
		dev_err(&sav_client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

/* -------------------------------------------------------------------------- */
/* STC310x gas gauge algorithm code */

/*******************************************************************************
* Function Name  : STC310x_ReadByte
* Description    : utility function to read the value stored in one register
* Input          : RegAddress: STC310x register,
* Return         : 8-bit value, or 0 if error
*******************************************************************************/
static int STC310x_ReadByte(int RegAddress)
{
	int value;
	unsigned char data[2];
	int res;

	res = STC310x_Read(1, RegAddress, data);

	if (res >= 0) {
		/* no error */
		value = data[0];
	} else
		value = 0;

	return (value);
}

/*******************************************************************************
* Function Name  : STC310x_WriteByte
* Description    : utility function to write a 8-bit value into a register
* Input          : RegAddress: STC310x register, Value: 8-bit value to write
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_WriteByte(int RegAddress, unsigned char Value)
{
	int res;
	unsigned char data[2];

	data[0] = Value;
	res = STC310x_Write(1, RegAddress, data);

	return (res);

}

/*******************************************************************************
* Function Name  : STC310x_Startup
* Description    :  initialize and start the STC310x at application startup
* Input          : None
* Return         : 0 is ok, 1 if STC310x reset, -1 if error
*******************************************************************************/
static int STC310x_Startup(void)
{
	int res, value;

	/* first, check the presence of the STC310x by reading first byte of dev. ID */
	res = STC310x_ReadByte(STC310x_REG_ID);
	if (res != STC310x_ID)
		return (-1);

	/* read REG_CTRL to detect possible reset event */
	value = STC310x_ReadByte(STC310x_REG_CTRL);

	/* write 0x03 into the REG_CTRL to reset the accumulator and counter and clear the PORDET bit (IO0 pin open), */
	STC310x_WriteByte(STC310x_REG_CTRL, 0x03);

	res = STC310x_ReadByte(STC310x_REG_MODE);
/* then 0x10 into the REG_MODE register to start the STC310x in 14-bit resolution mode. */
	STC310x_WriteByte(STC310x_REG_MODE, 0x10);

	STC310x_RelaxTmrSet((-BATT_LOW_CURRENT));

	if (value & 0x10) {
		return (1);	/* reset detected */
	}
	return (0);		/* ok */
}

/*******************************************************************************
* Function Name  : STC310x_Powerdown
* Description    :  stop the STC310x at application power down
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_Powerdown(void)
{
	int res;

	/* write 0x01 into the REG_CTRL to release IO0 pin open, */
	STC310x_WriteByte(STC310x_REG_CTRL, 0x01);

	/* write 0 into the REG_MODE register to put the STC310x in standby mode */
	res = STC310x_WriteByte(STC310x_REG_MODE, 0);
	if (res != OK) {
		return (res);
	}

	return (OK);
}

/* -------------------------------------------------------------------------- */

#define CurrentFactor (48210/SENSERESISTOR)	/* LSB=11.77uV/R= ~48210/R/4096 - convert to mA  */
#define ChargeCountFactor (27443/SENSERESISTOR)	/* LSB=6.7uVh/R ~27443/R/4096 - converter to mAh */
#define VoltageFactor		9994	/* LSB=2.44mV ~9994/4096 - convert to mV         */
#define TemperatureFactor	5120	/* LSB=0.125°C ~5120/4096 - convert to 0.1°C     */

#define VinResFactor  (VINRESISTOR/100)	 /* VINRESISTOR/400000 = ~(VINRESISTOR/100)/4096  */
#define SenseResFactor (SENSERESISTOR*4) /* SENSERESISTOR/1000 = ~(SENSERESISTOR*4)/4096  */

/*******************************************************************************
* Function Name  : conv
* Description    : conversion utility 
*  convert a raw 16-bit value from STC310x registers into user units (mA, mAh, mV, °C)
*  (optimized routine for efficient operation on 8-bit processors such as STM8)
* Input          : value, factor
* Return         : result = value * factor / 4096
*******************************************************************************/
static int conv(short value, unsigned short factor)
{
	return (((long)value * factor) >> 12);
}

/*******************************************************************************
* Function Name  : STC310x_ReadBatteryData
* Description    :  utility function to read the battery data from STC310x
*                  to be called every 5s or so
* Input          : ref to BattData structure
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_ReadBatteryData(STC310x_BattDataTypeDef * BattData)
{
	unsigned char data[12];
	int res;
	int value;

	/* read STC310x registers 0 to 11 */
	res = STC310x_Read(12, 0, data);
	if (res < 0)
		return (res);	/* read failed */

	/* fill the battery status data */
	/* STC310x status */
	value = data[1];
	value = (value << 8) + data[0];
	BattData->Status = value;

	/* charge count */
	value = data[3];
	value = (value << 8) + data[2];
	BattData->Charge = conv(value, ChargeCountFactor); /* result in mAh */

	/* conversion counter */
	value = data[5];
	value = (value << 8) + data[4];
	BattData->ConvCounter = value;

	/* current */
	value = data[7];
	value = (value << 8) + data[6];
	value &= 0x3fff;         /* mask unused bits */
	if (value >= 0x2000)
		value -= 0x4000; /* convert to signed value */
	BattData->Current = conv(value, CurrentFactor);	/* result in mA */

	/* voltage */
	value = data[9];
	value = (value << 8) + data[8];
	value &= 0x0fff;         /* mask unused bits */
	if (value >= 0x0800)
		value -= 0x1000; /* convert to signed value */
	value = conv(value, VoltageFactor);	/* result in mV */

	/* correction for the voltage drop in Rsense: */
	value -= conv(BattData->Current, SenseResFactor);	/* i.e. value -= (long) (BattData->Current) * SENSERESISTOR / 1000; */
	BattData->Voltage = value;	/* result in mV */

	res = STC310x_Read(1, STC310x_REG_RELAX_COUNT, data);
	if (res < 0) {
		return (res);	/* read failed */
	}
	value = data[0];
	BattData->RelaxTimer = value * 8;	/* result in sec */

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_ReadRamData
* Description    : utility function to read the RAM data from STC310x
* Input          : ref to RAM data array
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_ReadRamData(unsigned char *RamData)
{
	return (STC310x_Read(RAM_SIZE, STC310x_REG_RAM, RamData));
}

/*******************************************************************************
* Function Name  : STC310x_WriteRamData
* Description    : utility function to write the RAM data into STC310x
* Input          : ref to RAM data array
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_WriteRamData(unsigned char *RamData)
{
	return (STC310x_Write(RAM_SIZE, STC310x_REG_RAM, RamData));
}

/******************************************************************************* 
* Function Name  : Interpolate
* Description    : interpolate a Y value from a X value and X, Y tables (n points)
* Input          : x
* Return         : y
*******************************************************************************/
static int interpolate(int x, int n, int const *tabx, int const *taby)
{
	int index;
	int y;

	if (x >= tabx[0]) {
		y = taby[0];
	} else if (x <= tabx[n - 1]) {
		y = taby[n - 1];
	}

	else {
		/*  find interval */
		for (index = 1; index < n; index++)
			if (x > tabx[index])
				break;
		/*  interpolate */
		y = (taby[index - 1] - taby[index]) * (x -
						       tabx[index]) /
		    (tabx[index - 1] - tabx[index]);
		y += taby[index];
	}

	return y;
}

/******************************************************************************* 
* Function Name  : OCV2Cap
* Description    : Finds the %age of battery depending on the open circuit 
                   voltage of the battery
* Input          : Open Circuit Voltage
* Return         : relative capacity (0-100%)
*******************************************************************************/
static int OCV2Cap(int voltage)
{
	return (interpolate(voltage, NTABLE, OcvPoint, SocPoint));
}

/******************************************************************************* 
* Function Name  : Cap2OCV
* Description    : Find the OCV corresponding to a SOC level
* Input          : relative capacity (0-100%)
* Return         : Open Circuit Voltage
*******************************************************************************/
static int Cap2OCV(int cap)
{
	return (interpolate(cap, NTABLE, SocPoint, OcvPoint));
}

/*******************************************************************************
* Function Name  : calcCRC8
* Description    : calculate the CRC8
* Input          : data: pointer to byte array, n: number of vytes
* Return         : CRC calue
*******************************************************************************/
static int calcCRC8(unsigned char *data, int n)
{
	int crc = 0;		/* initial value */
	int i, j;

	for (i = 0; i < n; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			crc <<= 1;
			if (crc & 0x100)
				crc ^= 7;
		}
	}
	return (crc & 255);

}

/*******************************************************************************
* Function Name  : UpdateRamCrc
* Description    : calculate the RAM CRC
* Input          : none
* Return         : CRC value
*******************************************************************************/
static int UpdateRamCrc(void)
{
	int res;

	res = calcCRC8(GG_Ram.db, RAM_SIZE - 1);
	GG_Ram.db[RAM_SIZE - 1] = res;	/* last byte holds the CRC */
	return (res);
}

/*******************************************************************************
* Function Name  : Init_RAM
* Description    : Init the STC310x RAM registers with valid test word and CRC
* Input          : none
* Return         : none
*******************************************************************************/
static void Init_RAM(void)
{
	int index;

	for (index = 0; index < RAM_SIZE; index++)
		GG_Ram.db[index] = 0;
	GG_Ram.reg.TstWord = RAM_TSTWORD;	/* id. to check RAM integrity */
	GG_Ram.reg.BattCap = BATT_DEF_CAPACITY;
	GG_Ram.reg.Status = GG_INIT;
	GG_Ram.reg.MinChargePercent = BATT_MAX_SOC;
	/* update the crc */
	UpdateRamCrc();
}

/*******************************************************************************
* Function Name  : UpdateCapacity
* Description    : update the nominal capacity (fully charge battery)
* Use            : BattData structure
* Affect         : BattChargeNominal, BattChargeValue
*******************************************************************************/
static void UpdateCapacity(void)
{
	int tentativeValue, diff;

	/* a full charge has just been done, update capacity */
	tentativeValue = BattChargeValue;
	/* allow up to 5% correction max  */
	diff = tentativeValue - BattChargeNominal;
	if (diff > (BattChargeNominal / 20)) {
		diff = BattChargeNominal / 20;
	}
	if (diff < (-BattChargeNominal / 20)) {
		diff = -BattChargeNominal / 20;
	}
	BattChargeNominal += diff;
	BattCalStat += 2;
}

/*******************************************************************************
* Function Name  : CheckUpdateSOC
* Description    : update the SOC using measured SOC %
* Use            : BattData structure
* Affect         : BattChargeValue
*******************************************************************************/
static void CheckUpdateSOC(void)
{
	int SOC;
	int v, diff, res, value;
	int tentativeValue;
	int ok;

	STC310x_RelaxTmrSet((-BATT_LOW_CURRENT));	/*Configure the Relax threshold if needed */

	if (BattData.Current > BATT_LOW_CURRENT) {
		BattAvgOCV = (BattAvgOCV * 3 + BattData.Voltage) / 4;
	} else {
		BattRelaxTime = BATT_RELAX_TIME >> 3;
		BattAvgOCV = BattData.Voltage;
	}

	ok = 1;
	if (STC310x_ReadByte(STC310x_REG_RELAX_COUNT) < (BattRelaxTime))
		ok = 0;		/*do not allow OCV if relax counter is under specified time */
	if (BattData.Temperature < 100)
		ok = 0;		/* do not allow capacity update at low temperature <10°C  */
	if (BattData.Temperature > 450)
		ok = 0;		/* do not allow capacity update at high temperature >45°C */

	if (ok) {
		/* we know here that current is low, relaxation time is ok */
		v = BattAvgOCV;
		v -= BATT_INT_RESIST * BattData.Current / 1000;	/* correction for non-true OCV */
		SOC = OCV2Cap(v);
		diff = SOC - (long)BattChargeValue *100 / BattChargeNominal;
		if (diff < 0)
			diff = -diff;

		/* SOC is updated if diff > 10, without other condition
		   otherwise SOC updated with filtering unless:
		   voltage >4.1V or <3.6V
		   diff<=6 and voltage inside 3.7-3.8V
		   diff<=3
		 */

		if (((v > 4100) || (v < 3600)) && (diff <= 10))
			ok = 0;	/* nok at >4.1V or < 3.6V */
		if ((v > 3700) && (v < 3800) && (diff <= 6))
			ok = 0;	/* nok */
		if (ok) {
			if (diff > 3) {
				tentativeValue =
				    (long)BattChargeNominal *SOC / 100;
				if (tentativeValue < BattChargeValue) {
					BattChargeValue = (BattChargeValue * 3 + tentativeValue) / 4;	/* simple filter to avoid sharp SOC step */
				} else {
					BattChargeValue = (BattChargeValue * 7 + tentativeValue) / 8;	/* simple filter to avoid sharp SOC step */
				}

				//Update the SOC base register
				res =
				    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) <<
				     8) + STC310x_ReadByte(STC310x_REG_CHARGE);
				res = conv(res, ChargeCountFactor);

				res = BattChargeValue - res;

				value =
				    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1)
				     << 8) +
				    STC310x_ReadByte(STC310x_REG_SOC_BASE);
				value = conv(value, ChargeCountFactor);

				value = value + res;

				STC310x_WriteByte(STC310x_REG_SOC_BASE,
						  ((((long)value << 12) *
						    SENSERESISTOR /
						    27443) & 0xFF));
				STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
						  (((((long)value << 4) *
						     SENSERESISTOR /
						     27443)) & 0xFF));
				BattCalStat++;
			}
			BattOCVcomp = 1;
		}

		STC310x_RelaxTmrSet(0);	/*To clear the Relax Timer */
		BattRelaxTime = BATT_RELAX_TIME >> 4;	/* restart relax time counter with half period */
	}
	if (BattCalStat >= 100)
		BattCalStat = 0;
}

/* battery internal resistance value compensated with temperature */
static int BattIntResist(int temp)
{
	return (BATT_INT_RESIST *
		interpolate(temp / 10, NTEMP, TempTable,
			    RiTempTable) / RiTempTable[2]);
}

/* SOC derating value depending on temperature and early empty condition */
static int SOCd(int value)
{
	int r, v;

	v = (long)BattIntResist(BattData.Temperature) * APP_TYP_CURRENT / 1000;
	if (BattData.Current < BATT_LOW_CURRENT) {
		r = BattData.Voltage - Cap2OCV(value);
		if (r < v) {
			v = r;
		}
	}
	v = OCV2Cap(APP_MIN_VOLTAGE - v);
	/* battery SOC derating value with temperature & current */

	r = interpolate(BattData.Temperature / 10, NTEMP, TempTable, CapacityDerating);	/* for APP_TYP_CURRENT */
	if ((BattData.Current < APP_TYP_CURRENT)
	    && (APP_TYP_CURRENT < BATT_COMP_CURRENT)) {
		r = (long)r *(BattData.Current -
			      BATT_COMP_CURRENT) / (APP_TYP_CURRENT -
						    BATT_COMP_CURRENT);
	}

	if (r < v) {
		v = r;
	}

	return (v);
}

static int CompensateSOC(int value)
{
	int r, v;

	r = SOCd(value);	/* SOC derating value */
	v = (long)(value - r) * 1000 / (100 - r);	/* compensate */
	v = (v + 5) / 10;	/* rounding */

	return (v);
}

/*******************************************************************************
* Function Name  : GG_FSM
* Description    : process the Gas Gauge state machine
* Input          : BattData
* Return         : 
* Affect         : Global Gas Gauge data
*******************************************************************************/
static void GG_FSM(void)
{
	int res, value;

	switch (BattState) {
	case BATT_CHARGING:
		if (BattData.Current < APP_MIN_CURRENT) {
			BattFullDischarge = 0;
			BattState = BATT_DISCHARG;	/*  discharging */
		} else if ((BattData.Current < CHG_MIN_CURRENT)) {
			BattState = BATT_ENDCHARG;	/* end of charge */
		}
		break;
	case BATT_ENDCHARG:	/* end of charge state. check if fully charged or charge interrupted */
		if (BattData.Current < CHG_END_CURRENT)
			BattState = BATT_IDLE;	/* charge interrupted */
		else
			BattState = BATT_FULCHARG;	/* end of charge */
		break;
	case BATT_FULCHARG:	/* full charge state. wait for actual end of charge current */
		if ((BattData.Current > CHG_MIN_CURRENT))
			BattState = BATT_CHARGING;	/* charge again */
		else if (BattData.Current < CHG_END_CURRENT) {
			if (BattData.Voltage > BATT_CHG_VOLTAGE) {
				if (BattOCVcomp) {
					UpdateCapacity();
				}
				BattChargeValue = BattChargeNominal;

				/* Update the base charge register */
				res =
				    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) <<
				     8) + STC310x_ReadByte(STC310x_REG_CHARGE);
				res = conv(res, ChargeCountFactor);

				res = BattChargeValue - res;

				value =
				    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1)
				     << 8) +
				    STC310x_ReadByte(STC310x_REG_SOC_BASE);
				value = conv(value, ChargeCountFactor);

				value = value + res;

				STC310x_WriteByte(STC310x_REG_SOC_BASE,
						  ((((long)value << 12) *
						    SENSERESISTOR /
						    27443) & 0xFF));
				STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
						  (((((long)value << 4) *
						     SENSERESISTOR /
						     27443)) & 0xFF));

				BattFullDischarge = 0;
				BattFullCharge = 1;	/* full charge done */
				BattOCVcomp = 0;
			}
			BattState = BATT_IDLE;	/* end of charge cycle */
		}
		break;
	case BATT_IDLE:	/* no charging, no discharging */
		if (BattData.Current > CHG_END_CURRENT) {
			BattFullCharge = 0;
			BattState = BATT_CHARGING;	/* charging again */
		} else if (BattData.Current < APP_MIN_CURRENT) {
			BattFullDischarge = 0;
			BattState = BATT_DISCHARG;	/* discharging again */
		}
		break;
	case BATT_DISCHARG:
		if (BattData.Current > APP_MIN_CURRENT) {
			BattState = BATT_IDLE;
		} else if (BattData.Voltage < BATT_MIN_VOLTAGE) {
			BattFullCharge = 0;
			BattFullDischarge = 1;	/* full discharge done */
			BattState = BATT_LOWBATT;
		}
		break;
	case BATT_LOWBATT:	/* battery nearly empty... */
		if (BattData.Current > APP_MIN_CURRENT) {
			BattState = BATT_IDLE;	/* idle */
		}
		break;
	default:
		BattState = BATT_IDLE;	/* idle */

	}			/* end switch */

	if ((BattState == BATT_DISCHARG) || (BattState == BATT_IDLE)) {
		CheckUpdateSOC();	/* check if time to update SOC from OCV */
	}
}

/*******************************************************************************
* Function Name  : Reset_FSM_GG
* Description    : reset the gas gauge state machine and flags
* Input          : None
* Return         : None
*******************************************************************************/
static void Reset_FSM_GG(void)
{
	BattState = BATT_IDLE;
	BattFullCharge = 0;
	BattFullDischarge = 0;
	BattOCVcomp = 0;
	BattCalStat = 0;
	STC310x_RelaxTmrSet(0);	/*To clear the Relax Timer */
	BattRelaxTime = BATT_RELAX_TIME >> 3;	//init to 600s
	BattAvgOCV = BattData.Voltage;
}

/*******************************************************************************
* Function Name  : UpdateBaseCharge
* Description    : update the base charge with the last charge count from STC310x
* Input          : None
* Return         : None
*******************************************************************************/
static void UpdateBaseCharge(void)
{
	//GG_Ram.reg.BaseCharge = GG_Ram.reg.LastCharge;
	//GG_Ram.reg.LastCharge= 0;
}

/* -------------------- firmware interface functions ------------------------------------------- */

static int GasGauge_Task(GasGauge_DataTypeDef * GG);

/*******************************************************************************
* Function Name  : GasGauge_Start
* Description    : Start the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if STC310x not found or I2C error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
static int GasGauge_Start(GasGauge_DataTypeDef * GG)
{
	int res, ii, value;

	/*initialize average value to 0 */
	NbrOfTask = 0;
	for (ii = 0; ii < 4; ii++) {
		BattAvrChargePerccent[ii] = 0;
	}

	/*Initialise Alarm thresholds */
	SocThreshold = BATT_LOW_SOC_ALM;
	VoltageThreshold = BATT_LOW_VOLT_ALM;

	/* check STC310x reset bit and start STC310x */
	res = STC310x_Startup();
	if (res < 0)
		return (-1);	/* error */

	/* check RAM valid */
	STC310x_ReadRamData(GG_Ram.db);

	if ((res == 1) || (GG_Ram.reg.TstWord != RAM_TSTWORD)
	    || (calcCRC8(GG_Ram.db, RAM_SIZE) != 0)) {
		/* RAM invalid */
		Init_RAM();
	} else {
		/* RAM valid, check status */
		switch (GG_Ram.reg.Status) {
		case GG_RUNNING:
			/* the application restarted without stopping the gas Gauge properly, do it now */
			//UpdateBaseCharge();
			/* Update the base charge register with the RAM data */
			res =
			    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_CHARGE);
			res = conv(res, ChargeCountFactor);

			res = GG_Ram.reg.BaseCharge - res;

			value =
			    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_SOC_BASE);
			value = conv(value, ChargeCountFactor);

			value = value + res;

			STC310x_WriteByte(STC310x_REG_SOC_BASE,
					  ((((long)value << 12) *
					    SENSERESISTOR / 27443) & 0xFF));
			STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
					  (((((long)value << 4) *
					     SENSERESISTOR / 27443)) & 0xFF));
			STC310x_AlarmSet();
			break;
		case GG_POWERDN:
			/* goto RUNNING state ( INIT state is also an option) */
			GG_Ram.reg.Status = GG_RUNNING;
			/* Update the base charge register with the RAM data */
			res =
			    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_CHARGE);
			res = conv(res, ChargeCountFactor);

			res = GG_Ram.reg.BaseCharge - res;

			value =
			    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_SOC_BASE);
			value = conv(value, ChargeCountFactor);

			value = value + res;

			STC310x_WriteByte(STC310x_REG_SOC_BASE,
					  ((((long)value << 12) *
					    SENSERESISTOR / 27443) & 0xFF));
			STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
					  (((((long)value << 4) *
					     SENSERESISTOR / 27443)) & 0xFF));
			STC310x_AlarmSet();
			break;
		default:
			GG_Ram.reg.Status = GG_INIT;
		}
	}
	/* update the crc */
	UpdateRamCrc();
	STC310x_WriteRamData(GG_Ram.db);

	Reset_FSM_GG();

	return (0);
}

/*******************************************************************************
* Function Name  : GasGauge_Stop
* Description    : Stop the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if I2C error
*******************************************************************************/
static int GasGauge_Stop(void)
{
	int res;

	STC310x_ReadRamData(GG_Ram.db);
	GG_Ram.reg.Status = GG_POWERDN;
	/* at application power down, the last charge count must be saved in the charge base in STC310x RAM */
	//UpdateBaseCharge();
	/* update the crc */
	UpdateRamCrc();
	STC310x_WriteRamData(GG_Ram.db);

	res = STC310x_Powerdown();
	if (res != 0) {
		return (-1);	/* error */
	}

	return (0);
}

/*******************************************************************************
* Function Name  : GasGauge_Restart
* Description    : Restart the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if STC310x not found or I2C error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
int GasGauge_Restart(GasGauge_DataTypeDef * GG)
{
	int res;

	res = STC310x_Reset();
	if (res != OK) {
		return (res);
	}

	GasGauge_Start(GG);

	return (OK);
}

EXPORT_SYMBOL(GasGauge_Restart);
static int gVoltage = 0, gChargePercent = 0;
/*******************************************************************************
* Function Name  : GasGauge_Task
* Description    : Periodic Gas Gauge task, to be called e.g. every 5 sec.
* Input          : pointer to gas gauge data structure
* Return         : 1 if data available, 0 si no data, -1 if error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
static int GasGauge_Task(GasGauge_DataTypeDef * GG)
{
	int res, ii;
	int value, OCV;
	int SOC_avg, SOC_val, diff;
	static int cnt_stc = 0;
	res = STC310x_ReadBatteryData(&BattData);	/* read battery data into global variables */
	if (res != 0)
		return (-1);	/* abort in case of I2C failure */

	/* check if RAM data is ok (battery has not been changed) */
	STC310x_ReadRamData(GG_Ram.db);
	if ((GG_Ram.reg.TstWord != RAM_TSTWORD)
	    || (calcCRC8(GG_Ram.db, RAM_SIZE) != 0)) {
		/* if RAM non ok, reset it and set default nominal capacity */
		Init_RAM();
		STC310x_Startup();	/* restart STC310x */
	}
	//Check temperature consistency
	if ((BattData.Temperature < -500) || (BattData.Temperature > 1500)) {
		BattData.Temperature = 250;	//default value
	}
	//Check the local variable integrity
	if (BattRelaxTime != (BATT_RELAX_TIME >> 3)
	    && BattRelaxTime != (BATT_RELAX_TIME >> 4)) {
		Reset_FSM_GG();	/*Initialise Alarm thresholds */
		SocThreshold = BATT_LOW_SOC_ALM;
		VoltageThreshold = BATT_LOW_VOLT_ALM;
		NbrOfTask = 0;
	}
	if (GG_Ram.reg.Status == GG_INIT) {
		/* INIT state, wait for a voltage value available: */
		if (BattData.ConvCounter > 1) {
			OCV = BattData.Voltage;
			OCV -= (long)BattData.Current * BattIntResist(BattData.Temperature) / 1000;	/* correction for non-true OCV */

			if ((OCV > 2500) && (OCV < 4500)) {

				GG_Ram.reg.BaseCharge =
				    (long)BATT_DEF_CAPACITY *OCV2Cap(OCV) / 100;
				BattData.Charge = GG_Ram.reg.BaseCharge;

				/* Initialise the base charge register */
				res =
				    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) <<
				     8) + STC310x_ReadByte(STC310x_REG_CHARGE);
				res = conv(res, ChargeCountFactor);

				res = BattData.Charge - res;

				value =
				    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1)
				     << 8) +
				    STC310x_ReadByte(STC310x_REG_SOC_BASE);
				value = conv(value, ChargeCountFactor);

				value = value + res;

				STC310x_WriteByte(STC310x_REG_SOC_BASE,
						  ((((long)value << 12) *
						    SENSERESISTOR /
						    27443) & 0xFF));
				STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
						  (((((long)value << 4) *
						     SENSERESISTOR /
						     27443)) & 0xFF));

				GG_Ram.reg.Status = GG_RUNNING;
				STC310x_AlarmSet();
				Reset_FSM_GG();

				//Init OCV averaging
				GG_Ram.reg.AvrOCV = OCV;
				GG_Ram.reg.AvrCount = AVG_LEN;
				GG_Ram.reg.PreviousCurrent = BattData.Current;
				GG_Ram.reg.AvrStatus = 0;
			}
		} else {
			if ((BattData.Status & M_STAT) == M_RUN)	//be sure that the chip is running
			{
				return 0;	//not enougth voltage and current information so return error
			}
		}
	}

	if ((BattData.Status & M_STAT) != M_RUN) {
		STC310x_Startup();	/* and restart STC310x */
		/* unexpected STC310x status */
		if (GG_Ram.reg.Status == GG_RUNNING) {
			/* Restore the base charge register with the RAM data */
			BattData.Charge = GG_Ram.reg.BaseCharge;

			value = BattData.Charge;
			STC310x_WriteByte(STC310x_REG_SOC_BASE,
					  ((((long)value << 12) *
					    SENSERESISTOR / 27443) & 0xFF));
			STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
					  (((((long)value << 4) *
					     SENSERESISTOR / 27443)) & 0xFF));
			STC310x_AlarmSet();
		}
	}

	/* Battery state of charge (mAh): */
	BattChargeValue = BattData.Charge;
	/* Battery nominal capacity (mAh) */
	BattChargeNominal = GG_Ram.reg.BattCap;

	/* ---------- process the Gas Gauge algorithm -------- */
	if (GG_Ram.reg.Status == GG_RUNNING) {
		/* if Gas Gauge is running ... */
		if ((BattData.Status & M_EOC) != 0) {
			/* ... and if new data available: */
			GG_FSM();
		}
	}
	/* --------------------------------------------------- */

	//OCV average Algorithm
	if (GG_Ram.reg.AvrCount > 0) {

		//check current delta
		diff = BattData.Current - GG_Ram.reg.PreviousCurrent;
		if (diff < 0)
			diff = -diff;

		//Present OCV
		OCV = BattData.Voltage;
		OCV -= (long)BattData.Current * BattIntResist(BattData.Temperature) / 1000;	/* correction for non-true OCV */

		if ((GG_Ram.reg.AvrStatus == 0) || (diff <= AVR_CURRENT_THR && GG_Ram.reg.AvrStatus == 1))	//averaguing in any cases
		{
			//New OCV average
			GG_Ram.reg.AvrOCV =
			    (GG_Ram.reg.AvrOCV * AVG_WEIGHT +
			     OCV) / (AVG_WEIGHT + 1);

			//SOC translation
			SOC_avg = OCV2Cap(GG_Ram.reg.AvrOCV);
			SOC_val =
			    (long)BattChargeValue *1000 / GG_Ram.reg.BattCap;
			SOC_val = (SOC_val + 5) / 10;	/* rounding */

			//Bring SOC value inside guardbande
			if (SOC_val > (SOC_avg + AVG_THR)) {
				SOC_val = (SOC_avg + AVG_THR);
			}

			if (SOC_val < (SOC_avg - AVG_THR)) {
				SOC_val = (SOC_avg - AVG_THR);
			}
			//Update charge value
			BattChargeValue = SOC_val * GG_Ram.reg.BattCap / 100;

			/* Initialise the base charge register */
			res =
			    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_CHARGE);
			res = conv(res, ChargeCountFactor);

			res = BattChargeValue - res;

			value =
			    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1) << 8) +
			    STC310x_ReadByte(STC310x_REG_SOC_BASE);
			value = conv(value, ChargeCountFactor);

			value = value + res;

			STC310x_WriteByte(STC310x_REG_SOC_BASE,
					  ((((long)value << 12) *
					    SENSERESISTOR / 27443) & 0xFF));
			STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
					  (((((long)value << 4) *
					     SENSERESISTOR / 27443)) & 0xFF));
		}
		if (diff <= AVR_CURRENT_THR && GG_Ram.reg.AvrStatus == 0 && GG_Ram.reg.AvrCount < AVG_LEN)	//first correct information 
		{
			GG_Ram.reg.AvrStatus = 1;
			if (GG_Ram.reg.AvrCount < (AVG_LEN - 1))	//avoid to initialise if both first data are correct
			{
				//New OCV average
				GG_Ram.reg.AvrOCV = OCV;;

				//SOC translation
				SOC_val = OCV2Cap(GG_Ram.reg.AvrOCV);
				//Update charge value
				BattChargeValue =
				    SOC_val * GG_Ram.reg.BattCap / 100;

				/* Initialise the base charge register */
				res =
				    (STC310x_ReadByte(STC310x_REG_CHARGE + 1) <<
				     8) + STC310x_ReadByte(STC310x_REG_CHARGE);
				res = conv(res, ChargeCountFactor);

				res = BattChargeValue - res;

				value =
				    (STC310x_ReadByte(STC310x_REG_SOC_BASE + 1)
				     << 8) +
				    STC310x_ReadByte(STC310x_REG_SOC_BASE);
				value = conv(value, ChargeCountFactor);

				value = value + res;

				STC310x_WriteByte(STC310x_REG_SOC_BASE,
						  ((((long)value << 12) *
						    SENSERESISTOR /
						    27443) & 0xFF));
				STC310x_WriteByte(STC310x_REG_SOC_BASE + 1,
						  (((((long)value << 4) *
						     SENSERESISTOR /
						     27443)) & 0xFF));
			}
		}
		if ((diff > AVR_CURRENT_THR && GG_Ram.reg.AvrStatus == 1)) {
			//do nothing
		}

		GG_Ram.reg.PreviousCurrent = BattData.Current;
		GG_Ram.reg.AvrCount--;
	}

	/* --------------------------------------------------- */

	/* update the Gas Gauge status variables back into STC310x registers  */
	GG_Ram.reg.BaseCharge = BattChargeValue;
	GG_Ram.reg.BattCap = BattChargeNominal;

	/* -------- APPLICATION RESULTS ------------ */

	/* fill gas gauge data with battery data */
	GG->State = BattState;
	GG->Voltage = BattData.Voltage;
	GG->Current = BattData.Current;
	GG->Temperature = BattData.Temperature;

	GG->ChargeNominal = GG_Ram.reg.BattCap;

	/* State-of-Charge (mAh) */
	GG->ChargeValue = GG_Ram.reg.BaseCharge;

	/* State-Of-Charge (%)  - no load/temperature compensation */
	value = (long)GG->ChargeValue * 1000 / GG->ChargeNominal;
	value = (value + 5) / 10;	/* rounding */

	if (value < 0)
		value = 0;
	if (value > 100)
		value = 100;

	GG->ChargePercentNC = value;

	/* State-Of-Charge (%)  - load/temperature compensated */
	value = CompensateSOC(value);

//to avoid to use the filtering and minimum clamping during first SOC estimation
	if (GG_Ram.reg.AvrCount == 0) {
		/* State-Of-Charge (%)  - filtering */

		//averaging tab init
		if (NbrOfTask == 0) {
			for (ii = 0; ii < 4; ii++) {
				BattAvrChargePerccent[ii] = value;
			}
			NbrOfTask++;
		} else {
			BattAvrChargePerccent[3] = value;
		}

		//averaging calculation
		value = 0;
		for (ii = 0; ii < 4; ii++) {
			value += BattAvrChargePerccent[ii];
		}
		value = value / 4;	/*averaged compensated SOC value */

		/*Shift data on the left for next task */
		for (ii = 0; ii < 3; ii++) {
			BattAvrChargePerccent[ii] =
			    BattAvrChargePerccent[ii + 1];
		}

		/* State-Of-Charge (%)  - clamping to min during discharge  */
		if (GG->State == BATT_IDLE || GG->State == BATT_DISCHARG
		    || GG->State == BATT_LOWBATT) {
			if (value > GG_Ram.reg.MinChargePercent) {
				value = GG_Ram.reg.MinChargePercent;
			} else {
				GG_Ram.reg.MinChargePercent = value;
			}
		} else {
			GG_Ram.reg.MinChargePercent = value;
		}
	}
	//Write the actual SOC value
	STC310x_WriteByte(0x2D, (GG->ChargePercentNC & 0xFF));
	STC310x_WriteByte(0x2D, (GG_Ram.reg.AvrCount & 0xFF));
	/*Save RAM register values */
	UpdateRamCrc();
	STC310x_WriteRamData(GG_Ram.db);

	if (value < 0)
		value = 0;
	if (value > 100)
		value = 100;
	if (BattData.Current > CHG_END_CURRENT) {
		if (value > 99)
			value = 99;	/* if still charging, do not display 100% */
	}
/*Update capcity value*/
	GG->ChargePercent = value;
	GG->ChargeValue = GG->ChargePercent * GG->ChargeNominal / 100;
#if 0
	printk(KERN_EMERG
	       "/*** stc3105 (cnt_stc %d) ********************************/",
	       cnt_stc++);
	switch (GG->State) {
	case BATT_CHARGING:
		printk(KERN_EMERG "Battery is charging\n");
		break;
	case BATT_ENDCHARG:
		printk(KERN_EMERG "Battery is end charging\n");
		break;
	case BATT_FULCHARG:
		printk(KERN_EMERG "Battery is full charging\n");
		break;
	case BATT_IDLE:
		printk(KERN_EMERG "Battery is idle state\n");
		break;
	case BATT_DISCHARG:
		printk(KERN_EMERG "Battery is now discharging state \n");
		break;
	case BATT_LOWBATT:
		printk(KERN_EMERG "Battery is now low batt state \n");
		break;
	default:
		break;
	}
	printk(KERN_EMERG "Battery voltage = %d\n", GG->Voltage);
	printk(KERN_EMERG "Battery Current = %d\n", GG->Current);
	printk(KERN_EMERG "Battery Temperature = %d\n", GG->Temperature);
	printk(KERN_EMERG "Battery Percent = %d\n", GG->ChargePercent);
	printk(KERN_EMERG "Battery ChargeValue = %d\n", GG->ChargeValue);
	printk(KERN_EMERG "/*********************************************/");
#endif
	gVoltage = GG->Voltage;
	gChargePercent = GG->ChargePercent;

	/*Update alarm threshold values */
	STC310x_AlarmSetSOCThreshold(SocThreshold);
	STC310x_AlarmSetVoltageThreshold(VoltageThreshold);

	/* remaining operating time (during discharge only i.e. current is <0): */
	/* use compensated SOC for realistic operating time */
	if (BattData.Current < APP_MIN_CURRENT) {
		value = (long)GG->ChargeNominal * GG->ChargePercent * 60 / 100 / (-BattData.Current);	/* in minutes */
		if (value < 0)
			value = 0;
	} else
		value = -1;	/* to indicate no value */

	GG->RemTime = value;

#ifdef TEST_DRIVER
	GG->CalStat = BattCalStat;
#endif

	if (GG_Ram.reg.Status == GG_RUNNING) {
		return (1);
	} else {
		return (0);
	}
}

/*******************************************************************************
* Function Name  : STC310x_SetPowerSavingMode
* Description    :  Set the power saving mode
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_SetPowerSavingMode(void)
{
	int res;

	/* Read the mode register */
	res = STC310x_ReadByte(STC310x_REG_MODE);

	/* Set the PWR_SAVE bit to 1 */
	res = STC310x_WriteByte(STC310x_REG_MODE, (res | STC310x_PWR_SAVE));
	if (res != OK)
		return (res);

	return (OK);
}

EXPORT_SYMBOL(STC310x_SetPowerSavingMode);

/*******************************************************************************
* Function Name  : STC310x_StopPowerSavingMode
* Description    :  Stop the power saving mode
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_StopPowerSavingMode(void)
{
	int res;

	/* Read the mode register */
	res = STC310x_ReadByte(STC310x_REG_MODE);

	/* Set the PWR_SAVE bit to 1 */
	res = STC310x_WriteByte(STC310x_REG_MODE, (res & ~STC310x_PWR_SAVE));
	if (res != OK)
		return (res);

	return (OK);
}

EXPORT_SYMBOL(STC310x_StopPowerSavingMode);

/*******************************************************************************
* Function Name  : STC310x_AlarmSet
* Description    :  Set the alarm function and set the alarm threshold
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmSet(void)
{
	int res;

	/* Set the ALM_SOC_Threshold to 0 */
	res = STC310x_WriteByte(STC310x_REG_ALARM_SOC, 0);
	if (res != OK)
		return (res);
	res = STC310x_WriteByte((STC310x_REG_ALARM_SOC + 1), 0);
	if (res != OK)
		return (res);

	/* Set the ALM_Voltage_Threshold to 0 */
	res = STC310x_WriteByte(STC310x_REG_ALARM_VOLTAGE, 0);
	if (res != OK)
		return (res);

	/* Read the CTRL register */
	res = STC310x_ReadByte(STC310x_REG_CTRL);
	/* the IO0data driven by alarm and ALM bits cleared */
	res = STC310x_WriteByte(STC310x_REG_CTRL, ((res | 0x01) & 0x1F));
	if (res != OK)
		return (res);

	/* Read the mode register */
	res = STC310x_ReadByte(STC310x_REG_MODE);
	/* Set the ALM_ENA bit to 1 */
	res = STC310x_WriteByte(STC310x_REG_MODE, (res | STC310x_ALM_ENA));
	if (res != OK)
		return (res);

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_AlarmStop
* Description    :  Stop the alarm function 
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmStop(void)
{
	int res;

	/* Read the mode register */
	res = STC310x_ReadByte(STC310x_REG_MODE);

	/* Set the ALM_ENA bit to 0 and the IO0data forced to low */
	res = STC310x_WriteByte(STC310x_REG_MODE, (res & ~STC310x_ALM_ENA));
	if (res != OK)
		return (res);

	/* the IO0data forced to low and ALM bits cleared */
	res = STC310x_WriteByte(STC310x_REG_CTRL, (0x00));
	if (res != OK)
		return (res);

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_AlarmGet
* Description    : Return the ALM status
* Input          : None
* Return         : ALM status 00 : no alarm 
*                             01 : SOC alarm
*                             10 : Voltage alarm
*                             11 : SOC and voltage alarm
*******************************************************************************/
int STC310x_AlarmGet(void)
{
	int res;

	/* Read the mode register */
	res = STC310x_ReadByte(STC310x_REG_CTRL);
	res = res >> 5;

	return (res);
}

EXPORT_SYMBOL(STC310x_AlarmGet);

/*******************************************************************************
* Function Name  : STC310x_AlarmClear
* Description    :  Clear the alarm signal
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmClear(void)
{
	int res;

	res = STC310x_ReadByte(STC310x_REG_CTRL);
	/* clear ALM bits */
	res = STC310x_WriteByte(STC310x_REG_CTRL, ((res | 0x1) & 0x1F));
	if (res != OK)
		return (res);

	return (res);
}

EXPORT_SYMBOL(STC310x_AlarmClear);

/*******************************************************************************
* Function Name  : STC310x_AlarmSetVoltageThreshold
* Description    : Set the alarm threshold
* Input          : int voltage threshold
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmSetVoltageThreshold(int VoltThresh)
{
	int res;

	VoltThresh = ((VoltThresh << 9) / VoltageFactor);	/* LSB=2.44mV*8 ~9994/4096 - convert to mV         */

	/* Set the ALM_ENA bit to 1 */
	res = STC310x_WriteByte(STC310x_REG_ALARM_VOLTAGE, VoltThresh);
	if (res != OK)
		return (res);

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_AlarmChangeVoltageThreshold
* Description    : Change the alarm threshold
* Input          : int voltage threshold
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmChangeVoltageThreshold(int VoltThresh)
{
	VoltageThreshold = VoltThresh;

	return (OK);
}

EXPORT_SYMBOL(STC310x_AlarmChangeVoltageThreshold);

/*******************************************************************************
* Function Name  : STC310x_AlarmSetSOCThreshold
* Description    : Set the alarm threshold
* Input          : int SOC threshold in %
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmSetSOCThreshold(int SOCThresh)
{
	int res;

	SOCThresh = SOCThresh * BattChargeNominal / 100;

	/* Set the ALM_SOC_Threshold value to the correponding value */
	res =
	    STC310x_WriteByte(STC310x_REG_ALARM_SOC,
			      (((SOCThresh << 12) * SENSERESISTOR /
				27443) & 0xFF));
	if (res != OK)
		return (res);
	res =
	    STC310x_WriteByte((STC310x_REG_ALARM_SOC + 1),
			      ((((SOCThresh << 4) * SENSERESISTOR /
				 27443)) & 0xFF));
	if (res != OK)
		return (res);

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_AlarmChangeSOCThreshold
* Description    : Change the alarm threshold
* Input          : int voltage threshold
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_AlarmChangeSOCThreshold(int SOCThresh)
{
	SocThreshold = SOCThresh;

	return (OK);
}

EXPORT_SYMBOL(STC310x_AlarmChangeSOCThreshold);

/*******************************************************************************
* Function Name  : STC310x_RelaxTmrSet
* Description    :  Set the current threshold register to the passed value in mA
* Input          : int current threshold
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_RelaxTmrSet(int CurrentThreshold)
{
	int res;

	/* Set the relaxation current register to passed value */
	res = ((CurrentThreshold / (CurrentFactor >> 7))) + 1;	//+1 is due to the test condition
	res = STC310x_WriteByte(STC310x_REG_CURRENT_THRES, res);
	if (res != OK)
		return (res);

	return (OK);
}

/*******************************************************************************
* Function Name  : STC310x_Reset function
* Description    :  Reset all the chip by setting the PORDET to 1
* Input          : nothing
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_Reset(void)
{
	int res;

	//Clear the PORDET bit
	res = STC310x_WriteByte(STC310x_REG_CTRL, 0x10);
	if (res != OK)
		return (res);

	return (OK);
}

/* -------------------------------------------------------------------------- */

static void stc31xx_get_version(struct i2c_client *client)
{
	dev_info(&client->dev, "STC3105 Fuel-Gauge Ver %s\n", GG_VERSION);
}

static void stc31xx_get_online(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	if (chip->pdata && chip->pdata->battery_online)
		chip->online = chip->pdata->battery_online();
	else
		chip->online = 1;
}

static void stc31xx_get_status(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	if (!chip->pdata || !chip->pdata->charger_online ||
	    !chip->pdata->charger_enable) {
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
	if (chip->pdata) {
		BattData.Temperature = (chip->pdata->Temperature_value()) * 10;	/* assume temperature is return in degree */
	} else {
		BattData.Temperature = 250;	/* assume normal ambient temperature (25°C) */
	}
	if (chip->batt_soc > STC3100_BATTERY_FULL)
		chip->status = POWER_SUPPLY_STATUS_FULL;
}

/* -------------------------------------------------------------- */

static void stc31xx_work(struct work_struct *work)
{
	struct stc31xx_chip *chip;
	GasGauge_DataTypeDef GasGaugeData;
	int res;

	chip = container_of(work, struct stc31xx_chip, work.work);

	sav_client = chip->client;

	stc31xx_get_status(sav_client);

	stc31xx_get_online(sav_client);

	res = GasGauge_Task(&GasGaugeData);	/* process gas gauge algorithm, returns results */
	if (res > 0) {
		/* results available */
		chip->batt_soc = GasGaugeData.ChargePercent;
		chip->batt_voltage = GasGaugeData.Voltage;
		chip->batt_current = GasGaugeData.Current;
	}

	schedule_delayed_work(&chip->work, STC3100_DELAY);
}

static enum power_supply_property stc31xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};

static int __devinit stc31xx_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct stc31xx_chip *chip;
	int ret, res;

	GasGauge_DataTypeDef GasGaugeData;

	/*First check the functionality supported by the host */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -EIO;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
		return -EIO;

	/*OK. For now, we presume we have a valid client. We now create the
	   client structure */
	chip = kzalloc(sizeof(struct stc31xx_chip), GFP_KERNEL);
	if (!chip) {
		printk
		    ("Out of memory to create client structure for STC31xx\n");
		return -ENOMEM;	/*Out of memory */
	}

	printk("STC31xx probe started\n");

	/* The common I2C client data is placed right specific data. */
	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
	/*  
	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= stc31xx_get_property;
	chip->battery.properties	= stc31xx_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(stc31xx_battery_props);

	if (chip->pdata && chip->pdata->power_supply_register)
		ret = chip->pdata->power_supply_register(&client->dev, &chip->battery);
	else
		ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		//dev_err(&client->dev, "failed: power supply register\n");

		kfree(chip);
		return ret;
	}
	*/
	dev_info(&client->dev, "power supply register,%d\n", ret);

	stc31xx_get_version(client);

	/* init gas gauge system */
	sav_client = chip->client;
	res = GasGauge_Start(&GasGaugeData);

	INIT_DELAYED_WORK_DEFERRABLE(&chip->work, stc31xx_work);

	//The fallow scheduled task is using specific delay to improve measurement accuracy. 
	//This delay should be set between 1.2 or 1.3 seconds. I2C signals can help do debug the good behavior of the delay
	schedule_delayed_work(&chip->work, STC3100_DELAY_FOR_FIRST_TASK);
	//The specified delay depends of every platform and Linux kernel. It has to be checked physically during the driver integration
#if 0
	int j = 0;
	for (j = 0; gChargePercent < 10; j++) {
		//printk("stc3105: j %d, gVoltage %d, gChargePercent %d%\n", j, gVoltage, gChargePercent);
		msleep(1000);
	}
#endif
	return 0;

}

static int __devexit stc31xx_remove(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	/* stop gas gauge system */
	sav_client = chip->client;
	GasGauge_Stop();

	if (chip->pdata && chip->pdata->power_supply_unregister)
		chip->pdata->power_supply_unregister(&chip->battery);
	else
		power_supply_unregister(&chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM

static int stc31xx_suspend(struct i2c_client *client, pm_message_t state)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int stc31xx_resume(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, STC3100_DELAY);
	return 0;
}

#else

#define stc31xx_suspend NULL
#define stc31xx_resume NULL

#endif /* CONFIG_PM */

/* Every chip have a unique id */
static const struct i2c_device_id stc31xx_id[] = {
	{"stc3105", 0},
	{}
};

/* Every chip have a unique id and we need to register this ID using MODULE_DEVICE_TABLE*/
MODULE_DEVICE_TABLE(i2c, stc31xx_id);

static struct i2c_driver stc31xx_i2c_driver = {
	.driver = {
		.name = "stc3105",
	},
	.probe = stc31xx_probe,
	.remove = __devexit_p(stc31xx_remove),
	.suspend = stc31xx_suspend,
	.resume = stc31xx_resume,
	.id_table = stc31xx_id,
};

/*To register this I2C chip driver, the function i2c_add_driver should be called 
with a pointer to the struct i2c_driver*/
static int __init stc31xx_init(void)
{
	return i2c_add_driver(&stc31xx_i2c_driver);
}

module_init(stc31xx_init);

/*To unregister the I2C chip driver, the i2c_del_driver function should be called
with the same pointer to the struct i2c_driver*/
static void __exit stc31xx_exit(void)
{
	i2c_del_driver(&stc31xx_i2c_driver);
}

module_exit(stc31xx_exit);

MODULE_AUTHOR("ST IMS SYSTEMS LAB");
MODULE_DESCRIPTION("STC3105 Fuel Gauge");
MODULE_LICENSE("GPL");
