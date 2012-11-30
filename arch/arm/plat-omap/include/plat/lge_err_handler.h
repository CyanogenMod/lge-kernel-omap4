#ifndef __ARCH_ARM_PLAT_OMAP_ERR_HANDLER_H
#define __ARCH_ARM_PLAT_OMAP_ERR_HANDLER_H

//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [START]
typedef enum
{
	 LGE_NVDATA_IQ_RESET_CAUSE_UNKNOWN = 0
	,LGE_NVDATA_IQ_RESET_WATCHDOG_EXPIRED
	,LGE_NVDATA_IQ_RESET_EXCEPTION 			 //Any general exception (more specific ones below)
	,LGE_NVDATA_IQ_POOL_EXTENSION_FAILED		 //Pool extension failed
	,LGE_NVDATA_IQ_BUFFER_TOO_LARGE			 //Too large signal buffer requested
	,LGE_NVDATA_IQ_CONCURRENT_TICK_CALL		 //The tick system call was called concurrently by several processes.
	,LGE_NVDATA_IQ_SUPERV_STACK_OVERFLOW		 //A supervisor stack overflow was encountered
	,LGE_NVDATA_IQ_INTERRUPT_STACK_OVERFLOW	 //An interrupt stack overflow was encountered
	,LGE_NVDATA_IQ_UNKNOWN_BREAKPOINT			 //An unknown breakpoint was encountered
	,LGE_NVDATA_IQ_UNKNOWN_INTERRUPT			 //An unknown interrupt was encountered
	,LGE_NVDATA_IQ_START_STACK_OVERFLOW		 //Startup stack overflow
	,LGE_NVDATA_IQ_UNRECOGNIZED_CPU			 //The CPU model is not recognized
	,LGE_NVDATA_IQ_INCOMPATIBLE_CPU_HAL		 //The kernel configuration refers to at least one cpu hal with an interface version that isn't compatible with the kernel. 
	,LGE_NVDATA_IQ_UNEXPECTED_EXCEPTION		 //An unexpected exception occurred
	,LGE_NVDATA_IQ_PRIORITY_ERROR				 //An interrupt has occurred with equal or less priority than the currently running interrupt process
	,LGE_NVDATA_IQ_UNEXPECTED_EXCEPTION_REGDUMP //An unexpected exception occurred. 
	,LGE_NVDATA_IQ_EARLY_ERROR 				 //error() or error2() called before the kernel was completely initialized.
	,LGE_NVDATA_IQ_SYSCALL_TOO_EARLY			 //A kernel system call was used before the kernel was completely initialized
	,LGE_NVDATA_IQ_INCOMPATIBLE_CONFIGURATION	 //The kernel configuration file is incompatible with the kernel library
	,LGE_NVDATA_IQ_NO_BUFFER_END_MARK			 //A valid end mark could not be found in the signal buffer presented to the kernel
	,LGE_NVDATA_IQ_TOO_MANY_ATTACHED			 //Too many signals were attached
	,LGE_NVDATA_IQ_RECURSIVE_ERROR 			 //An error was encountered while an error handler was executing
	,LGE_NVDATA_IQ_RECURSIVE_SYSERROR			 //An error was encountered while an error handler was executing
	,LGE_NVDATA_IQ_TOO_MANY_MUTEXES			 //Too many mutexes in use
	,LGE_NVDATA_IQ_USER_STACK_OVERFLOW 		 //A user stack overflow was encountered
	,LGE_NVDATA_IQ_FACTORY_DATA_RESET			 //User-initiated master reset/clear to restore factory data to device
} lge_nvdata_iq_reset_cause_t;
//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [END]

extern int 	lge_is_force_ap_crash();
extern int  lge_is_crash_dump_enabled();
extern int lge_is_ap_crash_dump_enabled();
extern void lge_mark_ap_crash();
// hyoungsuk.jang@lge.com 20110128 CP Crash [START]
extern void lge_mark_cp_crash();
extern int  lge_is_mark_cp_crash();
// hyoungsuk.jang@lge.com 20110128 CP Crash [END]
extern void lge_dump_ap_crash();
extern void lge_user_reset();

//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [START]
extern void lge_store_ciq_reset(int is_ap, int cause);
//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [END]

#endif
