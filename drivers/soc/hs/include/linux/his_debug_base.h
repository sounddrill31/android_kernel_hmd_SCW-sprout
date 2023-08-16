#ifndef __HIS_DEBUG_CONTROL_NODE_H__
#define __HIS_DEBUG_CONTROL_NODE_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/version.h>

#define PROTECTED_GPIO_NUM   12

enum {
	BOOT_NORMAL_MODE,
	BOOT_CHARGER_MODE,
	BOOT_FACTORY_MODE,
	BOOT_RECOVERY_MODE,
	BOOT_CALI_MODE,
	BOOT_SILENCE_MODE
};

struct device_bootinfo {
	u8 bootmode;
	u8 meid_is_null;
	u8 alarm_mode;
	u8 backlight_on;
	u32 sector_size;
	u64 sectors_num;
	u64 ddr_size;
	/* tz protected gpios num and array */
	u32 prot_num;
	u32 prot_gpios[PROTECTED_GPIO_NUM];
	const char *parti_path;
	u8 fused;
	u8 board_id;
	u8 phone_is_encrypt;
};
extern struct device_bootinfo dev_bi;

#define PRINT_OUT(m, x...) \
	do { \
		if (m) \
			seq_printf(m, x); \
		else \
			pr_err(x); \
	} while (0)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#define FILE_SYNC ksys_sync()
#else /*KERNEL_VERSION_4_19*/
#define FILE_SYNC sys_sync()
#endif /*KERNEL_VERSION_4_19*/


enum {
	DEBUG_ENABLE_BIT       = 1U << 1,
	PRINT_WAKELOCK_BIT     = 1U << 2,
	SERIAL_ENABLE_BIT      = 1U << 3,
	AUDIO_DEBUG_BIT        = 1U << 4,
	FS_DEBUG_BIT           = 1U << 5,
};

void set_debug_flag_bit(int set_bit);
void clear_debug_flag_bit(int set_bit);
bool get_debug_flag_bit(int get_bit);

  /* read/write file interface */
extern size_t his_read_file(const char *path, loff_t offset, void *buf, u32 size);
extern size_t his_write_file(const char *path, loff_t offset, void *buf, u32 size);
 
/* sysfs node interface */
extern int his_register_sysfs_attr(struct attribute *attr);
extern int his_register_sysfs_attr_group(struct attribute_group *attr_group);
extern struct kobject *his_register_sysfs_dir(const char *name);

/* debugfs node interface */
extern int his_register_debugfs_file(const char *name, umode_t mode, void *data,
		const struct file_operations *fops);
extern struct dentry *his_register_debugfs_dir(const char *name);

/* procfs node interface */
extern int his_create_procfs_file(const char *name, umode_t mode,
		const struct file_operations *fops);
extern struct proc_dir_entry *his_create_procfs_dir(const char *name);


/* device tree get u32 arrry */
extern int his_of_get_u32_array(struct device_node *np, const char *prop_name,
		u32 *out, int *len, u32 max_size);


/* create debug flag sysfs attr */
extern void debug_flag_control_init(void);

extern unsigned long get_hs_total_ram(void);

#endif	/* __HIS_DEBUG_CONTROL_NODE_H__ */
