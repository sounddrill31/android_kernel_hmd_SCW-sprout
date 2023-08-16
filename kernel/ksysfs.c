// SPDX-License-Identifier: GPL-2.0-only
/*
 * kernel/ksysfs.c - sysfs attributes in /sys/kernel, which
 * 		     are not related to any other subsystem
 *
 * Copyright (C) 2004 Kay Sievers <kay.sievers@vrfy.org>
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/profile.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/capability.h>
#include <linux/compiler.h>

#include <linux/rcupdate.h>	/* rcu_expedited and rcu_normal */

#ifdef TARGET_PRODUCT_PUNISHER
#include <linux/mmc/mmc.h>
#include <linux/mm.h>
#endif

#define KERNEL_ATTR_RO(_name) \
static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

#define KERNEL_ATTR_RW(_name) \
static struct kobj_attribute _name##_attr = \
	__ATTR(_name, 0644, _name##_show, _name##_store)

/* current uevent sequence number */
static ssize_t uevent_seqnum_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", (unsigned long long)uevent_seqnum);
}
KERNEL_ATTR_RO(uevent_seqnum);

#ifdef CONFIG_UEVENT_HELPER
/* uevent helper program, used during early boot */
static ssize_t uevent_helper_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", uevent_helper);
}
static ssize_t uevent_helper_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	if (count+1 > UEVENT_HELPER_PATH_LEN)
		return -ENOENT;
	memcpy(uevent_helper, buf, count);
	uevent_helper[count] = '\0';
	if (count && uevent_helper[count-1] == '\n')
		uevent_helper[count-1] = '\0';
	return count;
}
KERNEL_ATTR_RW(uevent_helper);
#endif

#ifdef CONFIG_PROFILING
static ssize_t profiling_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", prof_on);
}
static ssize_t profiling_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;

	if (prof_on)
		return -EEXIST;
	/*
	 * This eventually calls into get_option() which
	 * has a ton of callers and is not const.  It is
	 * easiest to cast it away here.
	 */
	profile_setup((char *)buf);
	ret = profile_init();
	if (ret)
		return ret;
	ret = create_proc_profile();
	if (ret)
		return ret;
	return count;
}
KERNEL_ATTR_RW(profiling);
#endif

#ifdef CONFIG_KEXEC_CORE
static ssize_t kexec_loaded_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !!kexec_image);
}
KERNEL_ATTR_RO(kexec_loaded);

static ssize_t kexec_crash_loaded_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", kexec_crash_loaded());
}
KERNEL_ATTR_RO(kexec_crash_loaded);

static ssize_t kexec_crash_size_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%zu\n", crash_get_memory_size());
}
static ssize_t kexec_crash_size_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long cnt;
	int ret;

	if (kstrtoul(buf, 0, &cnt))
		return -EINVAL;

	ret = crash_shrink_memory(cnt);
	return ret < 0 ? ret : count;
}
KERNEL_ATTR_RW(kexec_crash_size);

#endif /* CONFIG_KEXEC_CORE */

#ifdef CONFIG_CRASH_CORE

static ssize_t vmcoreinfo_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	phys_addr_t vmcore_base = paddr_vmcoreinfo_note();
	return sprintf(buf, "%pa %x\n", &vmcore_base,
			(unsigned int)VMCOREINFO_NOTE_SIZE);
}
KERNEL_ATTR_RO(vmcoreinfo);

#endif /* CONFIG_CRASH_CORE */

/* whether file capabilities are enabled */
static ssize_t fscaps_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", file_caps_enabled);
}
KERNEL_ATTR_RO(fscaps);

#ifndef CONFIG_TINY_RCU
int rcu_expedited;
static ssize_t rcu_expedited_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(rcu_expedited));
}
static ssize_t rcu_expedited_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	if (kstrtoint(buf, 0, &rcu_expedited))
		return -EINVAL;

	return count;
}
KERNEL_ATTR_RW(rcu_expedited);

int rcu_normal;
static ssize_t rcu_normal_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(rcu_normal));
}
static ssize_t rcu_normal_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	if (kstrtoint(buf, 0, &rcu_normal))
		return -EINVAL;

	return count;
}
KERNEL_ATTR_RW(rcu_normal);
#endif /* #ifndef CONFIG_TINY_RCU */


#ifdef CONFIG_SUPPORT_RESTART_MODEM
static ssize_t restart_modem_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "restart modem by write 1\n");
}

extern void restart_modem_by_sysnode(void);
static ssize_t restart_modem_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long cnt;

	if (kstrtoul(buf, 0, &cnt))
		return -EINVAL;

	if (cnt == 1)
		restart_modem_by_sysnode();

	return count;
}
KERNEL_ATTR_RW(restart_modem);
#endif


#ifdef TARGET_PRODUCT_PUNISHER
int wallpaper_ID;
static ssize_t wallpaper_ID_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "The wallpaper_ID :0x%x\n", READ_ONCE(wallpaper_ID));
}
static ssize_t wallpaper_ID_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	if (kstrtoint(buf, 0, &wallpaper_ID))
		return -EINVAL;

	return count;
}
KERNEL_ATTR_RW(wallpaper_ID);

static ssize_t info_ram_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	char ramsize[8] = "";
	struct sysinfo si;
    si_meminfo(&si);
    if(si.totalram > 1572864 )				   // 6G = 1572864 	(256 *1024)*6
   		strcpy(ramsize , "8G");
    else if(si.totalram > 1048576)			  // 4G = 786432 	(256 *1024)*4
    		strcpy(ramsize , "6G");
    else if(si.totalram > 786432)			 // 3G = 786432 	(256 *1024)*3
    		strcpy(ramsize , "4G");
    else if(si.totalram > 524288)			// 2G = 524288 	(256 *1024)*2
    		strcpy(ramsize , "3G");
    else if(si.totalram > 262144)               // 1G = 262144		(256 *1024)     4K page size
    		strcpy(ramsize , "2G");
    else if(si.totalram > 131072)               // 512M = 131072		(256 *1024/2)   4K page size
    		strcpy(ramsize , "1G");
    else
    		strcpy(ramsize , "512M");

	return sprintf(buf, "%s\n", ramsize);
}
static ssize_t info_ram_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}
KERNEL_ATTR_RW(info_ram);
#endif

/*
 * Make /sys/kernel/notes give the raw contents of our kernel .notes section.
 */
extern const void __start_notes __weak;
extern const void __stop_notes __weak;
#define	notes_size (&__stop_notes - &__start_notes)

static ssize_t notes_read(struct file *filp, struct kobject *kobj,
			  struct bin_attribute *bin_attr,
			  char *buf, loff_t off, size_t count)
{
	memcpy(buf, &__start_notes + off, count);
	return count;
}

static struct bin_attribute notes_attr __ro_after_init  = {
	.attr = {
		.name = "notes",
		.mode = S_IRUGO,
	},
	.read = &notes_read,
};

struct kobject *kernel_kobj;
EXPORT_SYMBOL_GPL(kernel_kobj);


static struct attribute * kernel_attrs[] = {
	&fscaps_attr.attr,
	#ifdef CONFIG_SUPPORT_RESTART_MODEM
	&restart_modem_attr.attr,
	#endif
	&uevent_seqnum_attr.attr,
#ifdef CONFIG_UEVENT_HELPER
	&uevent_helper_attr.attr,
#endif
#ifdef CONFIG_PROFILING
	&profiling_attr.attr,
#endif
#ifdef CONFIG_KEXEC_CORE
	&kexec_loaded_attr.attr,
	&kexec_crash_loaded_attr.attr,
	&kexec_crash_size_attr.attr,
#endif
#ifdef CONFIG_CRASH_CORE
	&vmcoreinfo_attr.attr,
#endif
#ifndef CONFIG_TINY_RCU
	&rcu_expedited_attr.attr,
	&rcu_normal_attr.attr,
#endif
#ifdef TARGET_PRODUCT_PUNISHER
	&wallpaper_ID_attr.attr,
	&info_ram_attr.attr,
#endif
	NULL
};

static const struct attribute_group kernel_attr_group = {
	.attrs = kernel_attrs,
};

static int __init ksysfs_init(void)
{
	int error;

	kernel_kobj = kobject_create_and_add("kernel", NULL);
	if (!kernel_kobj) {
		error = -ENOMEM;
		goto exit;
	}
	error = sysfs_create_group(kernel_kobj, &kernel_attr_group);
	if (error)
		goto kset_exit;

	if (notes_size > 0) {
		notes_attr.size = notes_size;
		error = sysfs_create_bin_file(kernel_kobj, &notes_attr);
		if (error)
			goto group_exit;
	}

	return 0;

group_exit:
	sysfs_remove_group(kernel_kobj, &kernel_attr_group);
kset_exit:
	kobject_put(kernel_kobj);
exit:
	return error;
}

core_initcall(ksysfs_init);
