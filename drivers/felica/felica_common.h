/*
 *  felicacommon.h
 *  
 */

#ifndef __FELICACOMMON_H__
#define __FELICACOMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  INCLUDE FILES FOR MODULE
 */
#include <linux/module.h>/*THIS_MODULE*/
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>   /* printk() */
#include <linux/types.h>    /* size_t */
#include <linux/miscdevice.h>/*misc_register, misc_deregister*/
#include <linux/vmalloc.h>
#include <linux/fs.h>/*file_operations*/
#include <linux/delay.h>/*mdelay*/
#include <linux/irq.h>

#include <asm/uaccess.h>/*copy_from_user*/
#include <asm/io.h>/*static*/
#include <mach/gpio.h>
/*
 *  DEFINE
 */
/* function feature */
/* to use hw switch control for serial */
#define FUNCTION_P2_ONLY

/* debug message */
//#define FEATURE_DEBUG_LOW
#define FELICA_DEBUG_MSG printk


/* felica_pon */
#define FELICA_PON_NAME		"felica_pon"

/* felica */
#define FELICA_NAME		"felica"

/* felica_cen */
#define FELICA_CEN_NAME		"felica_cen"

/* felica_rfs */
#define FELICA_RFS_NAME		"felica_rfs"

/* felica_cal */
#define FELICA_CAL_NAME		"felica_cal"

/* felica I2C */
#define FELICA_I2C_NAME		"felica_i2c"

/* felica_int */
#define FELICA_RWS_NAME		"felica_rws"

/*
 *  EXTERNAL VARIABLE
 */
//Must check each model's path in 'android/system/core/rootdir/init.rc'file
#define FELICA_LD_LIBRARY_PATH "LD_LIBRARY_PATH=/vendor/lib:/system/lib"
#define FELICA_BOOTCLASSPATH "BOOTCLASSPATH=/system/framework/core.jar:/system/framework/core-junit.jar:/system/framework/bouncycastle.jar:/system/framework/ext.jar:/system/framework/framework.jar:/system/framework/framework2.jar:/system/framework/android.policy.jar:/system/framework/services.jar:/system/framework/apache-xml.jar:/system/framework/filterfw.jar:/system/framework/com.lge.bluetooth.jar:/system/framework/com.lge.core.jar:/system/framework/kafdex.jar:/system/framework/SmilHal.jar"
#define FELICA_PATH "PATH=/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin"

//Must check each model's VALUE from UART developer
#define FELICA_IC2_NAME "/dev/i2c-2"
#define FELICA_UART_NAME "/dev/ttyHSL1"

void lock_felica_wake_lock(void);
void unlock_felica_wake_lock(void);
void init_felica_wake_lock(void);
void destroy_felica_wake_lock(void);

#ifdef FELICA_LED_SUPPORT
void lock_felica_rfs_wake_lock(void);
void unlock_felica_rfs_wake_lock(void);
void init_felica_rfs_wake_lock(void);
void destroy_felica_rfs_wake_lock(void);
#endif


#ifdef __cplusplus
}
#endif

#endif // __FELICACOMMON_H__
