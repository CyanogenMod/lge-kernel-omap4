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
#include <plat/lge_nvdata_handler.h>

// ------------------------- For Dynamic NV data ---------------------------------------------------//

int lge_dynamic_nvdata_raw_read(int offset, char* buf, int size)
{
	if(size == 0) return 0;

	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(LGE_NVDATA_DYNAMIC_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_read( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't write dynamic NVDATA.\n");
			return ret;
		}

		sys_close(h_file);
	}
	else
	{
		printk("Can't open dynamic nvdata partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

int lge_dynamic_nvdata_raw_write(int offset, char* buf, int size)

{
	if(size == 0) return 0;

	int h_file = 0;
	int ret = 0;

	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(LGE_NVDATA_DYNAMIC_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_write( h_file, buf, size);

		if( ret != size )
		{
			printk("Can't write dynamic NVDATA.\n");
			return ret;
		}

		sys_close(h_file);
	}
	else
	{
		printk("Can't open dynamic NVDATA partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

int lge_dynamic_nvdata_read(enum lge_dynamic_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	return lge_dynamic_nvdata_raw_read(pos,buf,size);
}

int lge_dynamic_nvdata_write(enum lge_dynamic_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	return lge_dynamic_nvdata_raw_write(pos,buf,size);
}


void lge_clean_dynamic_nvdata_partition()
{
}

// ------------------------- For Static NV data -----------------------------------------------------//
int lge_static_nvdata_raw_read(int offset, char* buf, int size)
{
	if(size == 0) return 0;

	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(LGE_NVDATA_STATIC_PARTITION, O_RDWR,0);

			printk("lge_static_nvdata_raw_read : h_file = %d.\n",h_file);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_read( h_file, buf, size);

		sys_close(h_file);

		if( ret != size )
		{
			printk("Can't read static NVDATA.\n");
			return ret;
		}

		//sys_close(h_file);
	}
	else
	{
		printk("Can't open static nvdata partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(oldfs);

	return size;
}

int lge_static_nvdata_raw_write(int offset, char* buf, int size)

{
	if(size == 0) return 0;

	int h_file = 0;
	int ret = 0;
	mm_segment_t oldfs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(LGE_NVDATA_STATIC_PARTITION, O_RDWR,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_write( h_file, buf, size);

		sys_close(h_file);

		if( ret != size )
		{
			printk("Can't write static NVDATA.\n");
			return ret;
		}

		//sys_close(h_file);
	}
	else
	{
		printk("Can't open static NVDATA partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(oldfs);

	return size;
}



int lge_static_nvdata_read(enum lge_static_nvdata_offset offset, char* buf, int size)
{
	int pos = (int)offset;
	return lge_static_nvdata_raw_read(pos,buf,size);
}

int lge_static_nvdata_write(enum lge_static_nvdata_offset offset, char* buf, int size)

{
	int pos = (int)offset;
	return lge_static_nvdata_raw_write(pos,buf,size);
}
