#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#define VER_BUF_SIZE    32
static char mcu_ver[VER_BUF_SIZE] = {0};
static char board_id[VER_BUF_SIZE] = {0};
static char fpga_ver[VER_BUF_SIZE] = {0};
static const char *MCU_VER_FILE = "mcu_version";
static const char *FPGA_VER_FILE = "fpga_version";
static const char *board_id_file = "board_id";
static struct dentry *debugfs_base = 0;


struct did_data {
    unsigned int board_id;
    struct device dev;
    struct pinctrl *pctl;
};

static int mcu_ver_print(struct seq_file *m, void *v) {
    seq_printf(m, mcu_ver);
    return 0;
}

static int proc_mcu_info_open(struct inode *inode, struct  file *file) 
{
    return single_open(file, mcu_ver_print, NULL);
}

static ssize_t dev_mcu_info_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
    char ver[VER_BUF_SIZE] = {0};

    if (count > sizeof(mcu_ver)) {
        return -EFAULT;
    }

    if (copy_from_user(ver, buffer, count)) { 
        return -EFAULT;
    }
    memset(mcu_ver, 0, sizeof(mcu_ver));
    strncpy(mcu_ver, ver, count);

    return count; 
}

static int fpga_ver_print(struct seq_file *m, void *v) {
    seq_printf(m, fpga_ver);
    return 0;
}

static int proc_fpga_info_open(struct inode *inode, struct  file *file) 
{
    return single_open(file, fpga_ver_print, NULL);
}

static ssize_t dev_fpga_info_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
    char ver[VER_BUF_SIZE] = {0};

    if (count > sizeof(fpga_ver)) {
        return -EFAULT;
    }

    if (copy_from_user(ver, buffer, count)) { 
        return -EFAULT;
    }
    memset(fpga_ver, 0, sizeof(fpga_ver));
    strncpy(fpga_ver, ver, count);

    return count; 
}

static const struct file_operations proc_info_mcu_fops = {
    .owner      = THIS_MODULE,
	.open	    = proc_mcu_info_open,
	.read	    = seq_read,
    .write	    = dev_mcu_info_proc_write,
	.llseek	    = seq_lseek,
	.release    = single_release,
};

static const struct file_operations proc_info_fpga_fops = {
    .owner      = THIS_MODULE,
	.open	    = proc_fpga_info_open,
	.read	    = seq_read,
    .write	    = dev_fpga_info_proc_write,
	.llseek	    = seq_lseek,
	.release    = single_release,
};

static int bid_show(struct seq_file *m, void *v) {
    seq_printf(m, board_id);
    return 0;
}

static int proc_info_bid_open(struct inode *inode, struct  file *file) 
{
    return single_open(file, bid_show, NULL);
}

static const struct file_operations proc_info_bid_fops = {
    .owner      = THIS_MODULE,
	.open	    = proc_info_bid_open,
	.read	    = seq_read,
	.llseek	    = seq_lseek,
	.release    = single_release,
};

static int dev_info_debug_init(void)
{
    debugfs_base = debugfs_create_dir("dev_info", NULL);
    if (!debugfs_base)
        return -ENOMEM;

    if (!debugfs_create_file(MCU_VER_FILE, S_IRUGO | S_IWUGO, debugfs_base, NULL, &proc_info_mcu_fops))
        return -ENOMEM;

    if (!debugfs_create_file(FPGA_VER_FILE, S_IRUGO | S_IWUGO, debugfs_base, NULL, &proc_info_fpga_fops))
        return -ENOMEM;

    return 0;
}

static int dev_info_probe(struct platform_device *pdev)
{
    int err, id;
    struct did_data *pdid;
    struct device_node *np = pdev->dev.of_node;
    struct pinctrl_state *pctls;

    pdid = devm_kzalloc(&pdev->dev, sizeof(*pdid), GFP_KERNEL);
    if (!pdid)
        return -ENOMEM;

    pdid->board_id = 0;
    pdid->pctl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pdid->pctl)) {
        if (PTR_ERR(pdid->pctl) == -EPROBE_DEFER) {
            dev_err(&pdev->dev, "pin ctl critical error!\n");
            return -EPROBE_DEFER;
        }

        dev_notice(&pdev->dev, "pin control isn't used\n");
        pdid->pctl = 0;
    }

    if (pdid->pctl) {
        pctls = pinctrl_lookup_state(pdid->pctl, "dev_info_active");
        if (IS_ERR(pctls)) {
            dev_err(&pdev->dev, "failure to get pinctrl active state\n");
            return PTR_ERR(pctls);
        }
        err = pinctrl_select_state(pdid->pctl, pctls);
        if (err) {
            dev_err(&pdev->dev, "cannot set ts pinctrl active state\n");
            return err;
        }
        id = of_get_named_gpio(np,"mcn,board-id-0", 0);
        if (gpio_is_valid(id)) {
            err = devm_gpio_request(&pdev->dev, id, "board-id-0");
            if (err >= 0) {
                gpio_direction_input(id);
                gpio_export(id, 0);
                err = gpio_get_value(id);
                pdid->board_id |= err << 0;
            }
        }
        id = of_get_named_gpio(np,"mcn,board-id-1", 0);
        if (gpio_is_valid(id)) {
            err = devm_gpio_request(&pdev->dev, id, "board-id-1");
            if (err >= 0) {
                gpio_direction_input(id);
                gpio_export(id, 0);
                err = gpio_get_value(id);
                pdid->board_id |= err << 1;
            }
        }
        id = of_get_named_gpio(np,"mcn,board-id-2", 0);
        if (gpio_is_valid(id)) {
            err = devm_gpio_request(&pdev->dev, id, "board-id-2");
            if (err >= 0) {
                gpio_direction_input(id);
                gpio_export(id, 0);
                err = gpio_get_value(id);
                pdid->board_id |= err << 2;
            }
        }
    }

    sprintf(board_id, "%d", pdid->board_id);

    platform_set_drvdata(pdev, pdid);

    return 0;
}

static int dev_info_remove(struct platform_device *pdev)
{
	struct did_data *pd = platform_get_drvdata(pdev);

    if (pd) {
        kfree(pd);
    }

	platform_set_drvdata(pdev, 0);

	return 0;
}

static const struct of_device_id of_did_match[] = {
	{ .compatible = "mcn,device-info", },
	{},
};

static struct platform_driver did = {
	.probe    = dev_info_probe,
	.remove   = dev_info_remove,
    .driver   = {
        .owner	= THIS_MODULE,
        .name	= "dev_info",
        .bus	= &platform_bus_type,
        .of_match_table = of_match_ptr(of_did_match),
    },
};

int __init dev_info_init(void)
{
    int err;
    err = platform_driver_register(&did);

    if (0 != err) {
        pr_notice("failure to register\n");
    }

    proc_create(MCU_VER_FILE, S_IRUGO | S_IWUGO, NULL, &proc_info_mcu_fops);
    proc_create(FPGA_VER_FILE, S_IRUGO | S_IWUGO, NULL, &proc_info_fpga_fops);
    proc_create(board_id_file, S_IRUGO | S_IWUGO, NULL, &proc_info_bid_fops);

    dev_info_debug_init();

	return 0;
}

void __exit dev_info_exit(void)
{
    remove_proc_entry(MCU_VER_FILE, NULL);
    remove_proc_entry(FPGA_VER_FILE, NULL);

    if (debugfs_base)
        debugfs_remove_recursive(debugfs_base);
}

module_init(dev_info_init);
module_exit(dev_info_exit);

