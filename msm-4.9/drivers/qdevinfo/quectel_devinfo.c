/*********************************************************************************
 *      Copyright:  (C) 2019 quectel
 *                  All rights reserved.
 *
 *       Filename:  quectel_devinfo.c
 *    Description:  add this driver for get device information(AT+QDEVINFO)
 *
 *        Version:  1.0.0(20190227)
 *         Author:  Geoff Liu <geoff.liu@quectel.com>
 *      ChangeLog:  1, Release initial version on 20190227
 *                  Modify by Peeta Chen.
 ********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#define QUECTEL_QDEVINFO_CMD
#ifdef QUECTEL_QDEVINFO_CMD

#define EMMC_NAME_STR_LEN   32
#define DDR_STR_LEN         64
#define EXT_CSD_STR_LEN     1025
#define MMC_NAME            "/sys/class/mmc_host/mmc0/mmc0:0001/name"
#define MMC_EXT_CSD         "/sys/kernel/debug/mmc0/mmc0:0001/ext_csd"
#define MEMINFO             "/proc/meminfo"

/* read file */
int get_buf(const char *filename, char *buf, int size)
{
    int length;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;

    fp = filp_open(filename, O_RDONLY | O_LARGEFILE, 0);
    if (IS_ERR(fp)) {
        printk(KERN_ERR "create file error\n");
        return -1;
    }

    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    length = vfs_read(fp, buf, size, &pos);
    filp_close(fp, NULL);
    set_fs(fs);
    printk(KERN_INFO "length = %d", length);

    return length;
}

static int quec_emmc_ext_csd_proc_show(struct seq_file *m, void *v)
{
    char *kbuf;

    kbuf = kmalloc(EXT_CSD_STR_LEN + 1, GFP_KERNEL);
    memset(kbuf, 0, EXT_CSD_STR_LEN + 1);
    get_buf(MMC_EXT_CSD, kbuf, EXT_CSD_STR_LEN);
    seq_printf(m, "%s", kbuf);
    kfree(kbuf);

    return 0;
}

static int quec_emmc_ext_csd_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emmc_ext_csd_proc_show, inode->i_private);
}

static const struct file_operations quec_emmc_ext_csd_proc_fops = {
    .open       = quec_emmc_ext_csd_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

char change_char_hex(char c)
{
    if ((c >= '0') && (c <= '9'))
        return (c - '0');
    else if ((c >= 'a') && (c <= 'f'))
        return (c - 'a' + 10);
    else if ((c >= 'A') && (c <= 'F'))
        return (c - 'A' + 10);

    return 0;
}

char change_char_excsd(char c1, char c2)
{
    printk(KERN_DEBUG "%s:%d c1=0x%x, c2=0x%x\n", __func__, __LINE__, c1, c2);

    return change_char_hex(c1) * 16 +change_char_hex(c2);
}

static u32 quec_get_emmc_size_mb(void)
{
    char *kbuf;
    char mmc_size[4];
    u32 capacity = 0;

    kbuf = kmalloc(EXT_CSD_STR_LEN + 1, GFP_KERNEL);
    memset(kbuf, 0, EXT_CSD_STR_LEN + 1);
    get_buf(MMC_EXT_CSD, kbuf, EXT_CSD_STR_LEN);
    mmc_size[0] = change_char_excsd(kbuf[424], kbuf[425]);
    mmc_size[1] = change_char_excsd(kbuf[426], kbuf[427]);
    mmc_size[2] = change_char_excsd(kbuf[428], kbuf[429]);
    mmc_size[3] = change_char_excsd(kbuf[430], kbuf[431]);
    memcpy(&capacity, mmc_size, 4);
    kfree(kbuf);
    return capacity/2048;
}

static int quec_emmc_size_proc_show(struct seq_file *m, void *v)
{
    u32 capacity = 0;
    capacity = quec_get_emmc_size_mb();
    seq_printf(m, "%uM\n", capacity);
    return 0;
}

static int quec_emmc_size_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emmc_size_proc_show, inode->i_private);
}

static const struct file_operations quec_emmc_size_proc_fops = {
    .open       = quec_emmc_size_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int quec_emmc_eol_proc_show(struct seq_file *m, void *v)
{
    char *kbuf;
    char mmc_eol;

    kbuf = kmalloc(EXT_CSD_STR_LEN + 1, GFP_KERNEL);
    memset(kbuf, 0, EXT_CSD_STR_LEN + 1);
    get_buf(MMC_EXT_CSD, kbuf, EXT_CSD_STR_LEN);
    mmc_eol = change_char_excsd(kbuf[534], kbuf[535]);
    seq_printf(m, "emmc_eol[%04d]\n", mmc_eol);
    kfree(kbuf);
    return 0;
}

static int quec_emmc_eol_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emmc_eol_proc_show, inode->i_private);
}

static const struct file_operations quec_emmc_eol_proc_fops = {
    .open       = quec_emmc_eol_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int quec_emmc_life_proc_show(struct seq_file *m, void *v)
{
    char *kbuf;
    char mmc_life[2];

    kbuf = kmalloc(EXT_CSD_STR_LEN + 1, GFP_KERNEL);
    memset(kbuf, 0, EXT_CSD_STR_LEN + 1);
    get_buf(MMC_EXT_CSD, kbuf, EXT_CSD_STR_LEN);
    mmc_life[0] = change_char_excsd(kbuf[536], kbuf[537]);
    mmc_life[1] = change_char_excsd(kbuf[538], kbuf[539]);
    seq_printf(m, "emmc_life_time[%04x%04x]\n", mmc_life[0], mmc_life[1]);
    kfree(kbuf);

    return 0;
}

static int quec_emmc_life_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emmc_life_proc_show, inode->i_private);
}

static const struct file_operations quec_emmc_life_proc_fops = {
    .open       = quec_emmc_life_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int quec_emmc_health_proc_show(struct seq_file *m, void *v)
{
    char *kbuf;
    int i, j;
    char mmc_health;

    kbuf = kmalloc(EXT_CSD_STR_LEN +1, GFP_KERNEL);
    memset(kbuf, 0, EXT_CSD_STR_LEN + 1);
    get_buf(MMC_EXT_CSD, kbuf, EXT_CSD_STR_LEN);
    seq_printf(m, "mmc_health_factory[");
    for (i = 540; i < 572; i++) {
        j = i + 1;
        mmc_health = change_char_excsd(kbuf[i], kbuf[j]);
        seq_printf(m, "%2x", mmc_health);
        i++;
    }

    seq_printf(m, "]\n");
    seq_printf(m, "mmc_health_runtime[");
    for (i = 572; i < 604; i++) {
        j = i + 1;
        mmc_health = change_char_excsd(kbuf[i], kbuf[j]);
        seq_printf(m, "%02x", mmc_health);
        i++;
    }

    seq_printf(m, "]\n");
    kfree(kbuf);

    return 0;
}

static int quec_emmc_health_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emmc_health_proc_show, inode->i_private);
}

static const struct file_operations quec_emmc_health_proc_fops = {
    .open       = quec_emmc_health_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int quec_get_emmc_size_gb(void)
{
    u32 capacity_mb = 0;

    int size_gb;
    capacity_mb = quec_get_emmc_size_mb();
	if(capacity_mb >= 8192){
		if(capacity_mb % 8192 == 0)
			size_gb = (capacity_mb/8192) * 8; //GB
		else
			size_gb = (capacity_mb/8192 + 1) * 8; //GB
	}else{
		if(capacity_mb % 4096 == 0)
			size_gb = (capacity_mb/4096) * 4; //GB
		else
			size_gb = (capacity_mb/4096 + 1) * 4; //GB
	}

	if(size_gb <4 )
		size_gb = 4;

    return size_gb;
}

static int quec_get_ddr_size_mb(void)
{
    char *kbuf;
    int i, result = 0;

    kbuf = kmalloc(DDR_STR_LEN, GFP_KERNEL);
    memset(kbuf, 0, DDR_STR_LEN);
    get_buf(MEMINFO, kbuf, DDR_STR_LEN);

    for (i = 0; kbuf[i] != '\n'; i++) {
        if((kbuf[i] > '/') && (kbuf[i] < ':'))
            result = result * 10 + kbuf[i] - '0';
    }

    kfree(kbuf);
    return result/1024;
}

static int quec_emcp_info_proc_show(struct seq_file *m, void *v)
{
    char *kbuf;
    int ddr_size = 0;
    int i = 0;

    kbuf = kmalloc(EMMC_NAME_STR_LEN, GFP_KERNEL);
    memset(kbuf, 0, EMMC_NAME_STR_LEN);
    ddr_size = quec_get_ddr_size_mb();
    get_buf(MMC_NAME, kbuf, EMMC_NAME_STR_LEN);
    for (i = 0; i < EMMC_NAME_STR_LEN; i++) {
        if (kbuf[i] == '\n') {
            kbuf[i] = '\0';
            break;
        }
    }

    if (ddr_size <= 512)
        seq_printf(m, "%s,%dG,512M\n", kbuf, quec_get_emmc_size_gb());
    else {
        ddr_size = ddr_size / 1024 + 1;
        seq_printf(m, "%s,%dG,%dG\n", kbuf, quec_get_emmc_size_gb(), ddr_size);
    }

    kfree(kbuf);

    return 0;
}

static int quec_emcp_info_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_emcp_info_proc_show, inode->i_private);
}

static const struct file_operations quec_emcp_info_proc_fops = {
    .open       = quec_emcp_info_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

extern void quectel_get_pmic_info(char *buf);

static int quec_pmu_info_proc_show(struct seq_file *m, void *v)
{
    char pmu_info[64] = {'\0'};

    quectel_get_pmic_info(pmu_info);
    seq_printf(m, "%s\n", pmu_info);
    return 0;
}

static int quec_pmu_info_proc_open(struct inode *inode, struct file *filp)
{
    return single_open(filp, quec_pmu_info_proc_show, inode->i_private);
}

static const struct file_operations quec_pmu_info_proc_fops = {
    .open       = quec_pmu_info_proc_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int qdevinfo_proc_create(void)
{
    printk(KERN_INFO "proc create\n");
    proc_create("quec_emmc_ext_csd", 0444, NULL, &quec_emmc_ext_csd_proc_fops);
    proc_create("quec_emmc_size", 0444, NULL, &quec_emmc_size_proc_fops);
    proc_create("quec_emmc_eol", 0444, NULL, &quec_emmc_eol_proc_fops);
    proc_create("quec_emmc_life", 0444, NULL, &quec_emmc_life_proc_fops);
    proc_create("quec_emmc_health", 0444, NULL, &quec_emmc_health_proc_fops);
    proc_create("quec_emcp_info", 0444, NULL, &quec_emcp_info_proc_fops);
    proc_create("quec_pmu_info", 0444, NULL, &quec_pmu_info_proc_fops);
    return 0;
}

static int __init quec_devinfo_init(void)
{
    if (qdevinfo_proc_create())
        printk(KERN_ERR "quec_devinfo init failed!\n");
    else
        printk(KERN_INFO "quec_devinfo init success!\n");

    pr_info("fulinux I am here\n");

    return 0;
}

static void __exit quec_devinfo_exit(void)
{
    printk(KERN_DEBUG "quectel devinfo exit!");
}

module_init(quec_devinfo_init);
module_exit(quec_devinfo_exit);

MODULE_AUTHOR("geoff.liu@quectel.com");
MODULE_LICENSE("GPL");

#endif /* QUECTEL_QDEVINFO_CMD */
