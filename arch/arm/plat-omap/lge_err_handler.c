#include <linux/signal.h>
#include <linux/personality.h>
#include <linux/kallsyms.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/hardirq.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/kexec.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/system.h>
#include <asm/unistd.h>
#include <asm/traps.h>
#include <asm/unwind.h>

#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/i2c/twl.h>
#include <plat/lge_nvdata_handler.h>
#include <plat/lge_err_handler.h>


int lge_is_force_ap_crash()
{
	char data[2] = {0x00,0x00};

	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_FORCE_CRASH_OFFSET,data,1);
	msleep(100);

	if(data[0] == LGE_NVDATA_FORCE_CRASH_VALUE)
	{	
		printk("force ap crash\n");
		data[0] = 0x00;
		lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_FORCE_CRASH_OFFSET,data,1);
		msleep(100);
		return 1;
	}
	else
		return 0;
}


int lge_is_crash_dump_enabled()
{
	char data[2] = {0x00,0x00};

	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_CRASH_DUMP_OFFSET,data,1);
	msleep(100);

	if(data[0] == LGE_NDATA_DYNAMIC_CRASH_DUMP_ENABLE_VALUE)
		return 1;
	else
		return 0;
}

int lge_is_ap_crash_dump_enabled()
{	
	char data[2] = {0x00,0x00};
	
	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_AP_CRASH_DUMP_OFFSET,data,1);
	msleep(100);
	
	if(data[0] == LGE_NDATA_DYNAMIC_CRASH_DUMP_ENABLE_VALUE)
		return 1;
	else
		return 0;
}

void lge_mark_ap_crash()
{
	char trap_buffer[2] = { LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_AP_CRASH, 0x00 };

	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_RESET_CAUSE_OFFSET,trap_buffer,1);
}

// hyoungsuk.jang@lge.com 20110128 CP Crash [START]
void lge_mark_cp_crash()
{
	char trap_buffer[2] = { LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_CP_CRASH, 0x00 };

	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_RESET_CAUSE_OFFSET,trap_buffer,1);
}

int lge_is_mark_cp_crash()
{
	char data[2] = {0x00,0x00};

	lge_dynamic_nvdata_read(LGE_NVDATA_DYNAMIC_RESET_CAUSE_OFFSET,data,1);

	if(data[0] == LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_CP_CRASH)
		return 1;
	else
		return 0;
}
// hyoungsuk.jang@lge.com 20110128 CP Crash [END]


void lge_user_reset()
{
	char nvdata_user_reset_buffer[2] = {LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_USER_RESET, 0x00 };
	
	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_RESET_CAUSE_OFFSET,nvdata_user_reset_buffer,1);

}

static void lge_dump_kernel_log()
{
	extern int log_buf_copy(char *dest, int idx, int len);
	char log_buf[1024];
	int idx = 0;
	int cnt;
	int h_file = 0; 
	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);
	
	h_file = sys_open("/data/panic.txt", O_RDWR|O_CREAT,0644);

	if(h_file >= 0)
	{
		for (;;) {
			cnt = log_buf_copy(log_buf, idx, 1023);
			if (cnt <= 0)
				break;
			// WBT 20110314	[START]
			log_buf[cnt] = 0;
			// WBT 20110314	[END]
			sys_write(h_file,log_buf,cnt);
			idx += cnt;
		}		
		sys_close(h_file);
	}
	else
	{
		printk("Can't open log file ret = %d.\n",h_file);			
	}
	
	sys_sync();
	set_fs(oldfs);
}

static int lge_dump_android_log()
{
	static char * dontpanic_path = "/system/bin/logcat";
	char *argv[] = { dontpanic_path,"-t","1000" ">","/data/panic_an.txt",NULL };
	static char *envp[] = { "HOME=/",
							"PATH=/sbin:/bin:/system/bin",	NULL };

	return call_usermodehelper(dontpanic_path,
				   argv, envp, UMH_WAIT_PROC);

}

extern int oops_in_progress;
void lge_dump_ap_crash()
{
	int saved_oip;
	
	/* setting oops_in_progress prevents log_buf_copy()
	 * from trying to take a spinlock which will make it
	 * very unhappy in some cases...
	 */
	saved_oip = oops_in_progress;
	oops_in_progress = 1;

	lge_dump_kernel_log();

	// TO DO : adb log is not saved.
	//lge_dump_android_log();	
	//msleep(100);
	twl_i2c_write_u8(0x14, 0x47, 0x06); 
	return;
}

//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [START]
void lge_store_ciq_reset(int is_ap, int cause)
{
	char ciq_buffer[3] = { 0x00, 0x00, 0x00 };

	ciq_buffer[0] = (is_ap == 1) ? LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_AP_CRASH : LGE_NVDATA_DYNAMIC_RESET_CAUSE_VAL_CP_CRASH;
	ciq_buffer[1] = cause;	

	lge_dynamic_nvdata_write(LGE_NVDATA_DYNAMIC_CIQ_NVDATA_RESET_OFFSET, ciq_buffer, 2);
}
//LGE_ChangeS jaesung.woo@lge.com 20110131 CIQ [END]

