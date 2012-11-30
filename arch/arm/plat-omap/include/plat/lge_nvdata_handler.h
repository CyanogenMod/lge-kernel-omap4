#ifndef __ARCH_ARM_PLAT_OMAP_LGE_NVDATA_HANDLER_H
#define __ARCH_ARM_PLAT_OMAP_LGE_NVDATA_HANDLER_H


/* Non volatile data partition is divided as Dynamic and Static
    Static partion is skipped during download */

// ------------ For Common definition ----------------------------//
#define LGE_NVDATA_MMC_SECTOR_SIZE	0x200 // 512 byte



// ------------- For Dynamic NV data partion ----------------------//

// Partion name <- please check GPT
/*LGE_CHANGE_S [yuntaek.kim@lge.com] 2011-12-16 P720 Partition Change */
//#define LGE_NVDATA_DYNAMIC_PARTITION			"/dev/block/mmcblk0p2"  // <- It will be changed by partition change
#define LGE_NVDATA_DYNAMIC_PARTITION			"/dev/block/platform/omap/omap_hsmmc.1/by-name/nv1"  // <- It will be changed by partition change
/*LGE_CHANGE_E [yuntaek.kim@lge.com] 2011-12-16 P720 Partition Change */

// Offset list (Unit : bytes) - please consider sector size
enum lge_dynamic_nvdata_offset {
	LGE_NVDATA_DYNAMIC_REMOVE_FAT_OFFSET = 510,

	LGE_NVDATA_DYNAMIC_RESET_CAUSE_OFFSET 	= 512,  // used size 1 byte
	LGE_NVDATA_DYNAMIC_CRASH_DUMP_OFFSET 	= 514,  // used size 1 byte
	LGE_NVDATA_DYNAMIC_AP_CRASH_DUMP_OFFSET = 515,  // used size 1 byte
	LGE_NVDATA_DYNAMIC_FORCE_CRASH_OFFSET 	= 516,  // used size 1 byte

	LGE_NVDATA_DYNAMIC_FACTORY_RESET_STATUS_OFFSET 	= 518,  // used size 1 byte, for at%frst & at%frstatus
	LGE_NVDATA_DYNAMIC_FBOOT_OFFSET 	= 520,  // used size 1 byte, for at%fboot
	//LGE_NVDATA_DYNAMIC_FRSTSTATUS_OFFSET    = 522,// at%frststatus //moves to static nv area.

// CHEOLGWAK  2011-2-26 CP_CRASH_COUNT
	LGE_NVDATA_DYNAMIC_CP_CRASH_COUNT_OFFSET = 523,  // used size 1 byte
// CHEOLGWAK  2011-2-26 	CP_CRASH_COUNT

	//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [START]
	LGE_NVDATA_DYNAMIC_CIQ_NVDATA_RESET_OFFSET = 524, // used size 2 byte //RESET SIDE & CAUSE
	//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [END]

	//LGE_S woosock.yang@lge.com 20110512
	LGE_NVDATA_DYNAMIC_HARD_RESET_OFFSET = 526, 
	//LGE_E

	LGE_NVDATA_DYNAMIC_RTC_INIT_OFFSET = 528,
	LGE_NVDATA_DYNAMIC_FRSTSTATUS3_OFFSET = 530,//11.07.23 if set, must jump to recovery mode.

	LGE_NVDATA_DYNAMIC_910K_DETECT_OFFSET	= 1024,  // used size 1 byte

	LGE_NVDATA_DYNAMIC_QEM_OFFSET			= 2048, //length 4    ==> move to static nvdata
	LGE_NVDATA_DYNAMIC_DEVICETEST_OFFSET	= 2052, //length 8     ==> move to static nvdata
	LGE_NVDATA_DYNAMIC_DEVICDTEST_DATE_OFFSET = 2060,//length 8  ==> move to static nvdata	

	LGE_NVDATA_DYNAMIC_MUIC_RETENTION_OFFSET = 2560, // used size 1 byte

	LGE_NVDATA_DYNAMIC_SMPL_EN_OFFSET	= 2570,	// used size 1 byte
	LGE_NVDATA_DYNAMIC_SMPL_COUNT_OFFSET	= 2572,	// used size 4 byte

	LGE_NVDATA_DYNAMIC_CHARGING_TEMP_OFFSET	= 2600,	// used size 1 byte

	LGE_NVDATA_DYNAMIC_UBOOT_TIME = 4096//check boot time. //woosock.yang@lge.com
	// Please add offset of data that you want to use


};

// Value define for nv data
#define LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_USER_RESET	0x94
#define LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_AP_CRASH		0xDE
#define LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_CP_CRASH		0xAD
#define LGE_NVDATA_DYNAMIC_RESET_CAUSE_FACTORY_RESET	0x46


#define LGE_NDATA_DYNAMIC_CRASH_DUMP_INITIAL_VALUE	0x00
#define LGE_NDATA_DYNAMIC_CRASH_DUMP_ENABLE_VALUE	0xA5
#define LGE_NDATA_DYNAMIC_CRASH_DUMP_DISABLE_VALUE	0xB3

#define LGE_NVDATA_FORCE_CRASH_VALUE	0xB9


// ------------- For Static NV data partion ----------------------//

// Partion name <- please check GPT
/*LGE_CHANGE_S [yuntaek.kim@lge.com] 2011-12-16 P720 Partition Change */
//#define LGE_NVDATA_STATIC_PARTITION			"/dev/block/mmcblk0p3"  // <- It will be changed by partition change
#define LGE_NVDATA_STATIC_PARTITION			"/dev/block/platform/omap/omap_hsmmc.1/by-name/nv2"  // <- It will be changed by partition change
/*LGE_CHANGE_E [yuntaek.kim@lge.com] 2011-12-16 P720 Partition Change */



// Offset list (Unit : bytes) - please consider sector size
enum lge_static_nvdata_offset {
	LGE_NVDATA_STATIC_WIFI_MAC_ADDR 	= 0,
	LGE_NVDATA_STATIC_BT_MAC_ADDR 	= 512,

	LGE_NVDATA_STATIC_QEM_OFFSET		= 5116,  //length 4 
	LGE_NVDATA_STATIC_DEVICETEST_OFFSET	= 5120, //length 8   
	LGE_NVDATA_STATIC_DEVICETEST_DATE_OFFSET = 5128,//length 8 
	
	LGE_NVDATA_STATIC_LCD_GAMMA_R_OFFSET = 5136,//length 4
	LGE_NVDATA_STATIC_LCD_GAMMA_G_OFFSET = 5140,//length 4
	LGE_NVDATA_STATIC_LCD_GAMMA_B_OFFSET = 5144,//length 4

	LGE_NVDATA_STATIC_FRSTSTATUS_OFFSET = 5200 //1 byte.
	// Please add offset of data that you want to use
};


// ------------- API ----------------------------------------//

extern int lge_dynamic_nvdata_read(enum lge_dynamic_nvdata_offset offset, char* buf, int size);
extern int lge_dynamic_nvdata_write(enum lge_dynamic_nvdata_offset offset, char* buf, int size);


extern int lge_static_nvdata_read(enum lge_static_nvdata_offset offset, char* buf, int size);
extern int lge_static_nvdata_write(enum lge_static_nvdata_offset offset, char* buf, int size);

extern void lge_clean_dynamic_nvdata_partition(); // <-- for factory reset


// ---- this api is only for sysfs driver  , so do not use int other kernel side -------//
extern int lge_dynamic_nvdata_raw_read(int offset, char* buf, int size);
extern int lge_dynamic_nvdata_raw_write(int offset, char* buf, int size);
extern int lge_static_nvdata_raw_read(int offset, char* buf, int size);
extern int lge_static_nvdata_raw_write(int offset, char* buf, int size);

#endif
