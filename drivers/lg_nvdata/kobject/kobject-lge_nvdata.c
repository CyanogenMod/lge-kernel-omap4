/* LGE_CHANGE_S [woosock.yang@lge.com],  */
#include <linux/fs.h>
#include <linux/syscalls.h>

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <plat/lge_nvdata_handler.h>

#define MAX_SIZE 100

static int offset;
static char lge_dynamic_nvdata_raw_read_buf[MAX_SIZE];
static char lge_static_nvdata_raw_read_buf[MAX_SIZE];
static int size;

/***********************************
read funcs
	Concurrency issue possibly will occur... need to working on it.

	//ref_cnt for read funcs... -> NO.
	//lock...
***********************************/
static ssize_t lge_dynamic_nvdata_raw_read_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%s", lge_dynamic_nvdata_raw_read_buf);	
}

static ssize_t lge_dynamic_nvdata_raw_read_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d %d", &offset, &size);
	lge_dynamic_nvdata_raw_read(offset, lge_dynamic_nvdata_raw_read_buf, size);
	return count;
}

static ssize_t lge_static_nvdata_raw_read_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%s", lge_static_nvdata_raw_read_buf);
}


static ssize_t lge_static_nvdata_raw_read_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d %d", &offset, &size);
	lge_static_nvdata_raw_read(offset, lge_static_nvdata_raw_read_buf, size);	
	return count;
}


/***********************************
//write funcs	
***********************************/
static ssize_t lge_nvdata_write_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{	
	//write func doesn't allow to read.
	return sprintf(buf, "%s", "-1");
}

static ssize_t lge_dynamic_nvdata_raw_write_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	char bufTmp[MAX_SIZE];
	sscanf(buf, "%d %s %d", &offset, bufTmp, &size);
	lge_dynamic_nvdata_raw_write(offset, bufTmp, size);
	return count;
}

static ssize_t lge_static_nvdata_raw_write_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	char bufTmp[MAX_SIZE];
	sscanf(buf, "%d %s %d", &offset, bufTmp, &size);
	lge_static_nvdata_raw_write(offset, bufTmp, size);
	return count;
}



static struct kobj_attribute lge_dynamic_nvdata_raw_read_attribute =
	__ATTR(dynamic_nvdata_raw_read, 0666, lge_dynamic_nvdata_raw_read_show, lge_dynamic_nvdata_raw_read_store);
static struct kobj_attribute lge_dynamic_nvdata_raw_write_attribute =
	__ATTR(dynamic_nvdata_raw_write, 0666, lge_nvdata_write_show, lge_dynamic_nvdata_raw_write_store);
static struct kobj_attribute lge_static_nvdata_raw_read_attribute =
	__ATTR(static_nvdata_raw_read, 0666, lge_static_nvdata_raw_read_show, lge_static_nvdata_raw_read_store);
static struct kobj_attribute lge_static_nvdata_raw_write_attribute =
	__ATTR(static_nvdata_raw_write, 0666, lge_nvdata_write_show, lge_static_nvdata_raw_write_store);


static struct attribute *attrs[] = {
	&lge_dynamic_nvdata_raw_read_attribute.attr,
	&lge_dynamic_nvdata_raw_write_attribute.attr,
	&lge_static_nvdata_raw_read_attribute.attr,
	&lge_static_nvdata_raw_write_attribute.attr,	
	NULL,	
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *lge_nvdata_kobj;

static int __init lge_nvdata_init(void)
{
	int retval;

	lge_nvdata_kobj = kobject_create_and_add("kobject_lge_nvdata", kernel_kobj);
	if (!lge_nvdata_kobj)
		return -ENOMEM;

	retval = sysfs_create_group(lge_nvdata_kobj, &attr_group);
	if (retval)
		kobject_put(lge_nvdata_kobj);

	return retval;
}

static void __exit lge_nvdata_exit(void)
{
	kobject_put(lge_nvdata_kobj);
}

module_init(lge_nvdata_init);
module_exit(lge_nvdata_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("lge.com");
/* LGE_CHANGE_E [woosock.yang@lge.com] */
