
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>

#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/pinctrl/consumer.h>
#include <linux/syscore_ops.h>

struct rfkillpin_attr {
    struct device_attribute attr;
    char name[32];
};

struct watch_dog_pin_info{
	int toggle_pin;
	int port_det_pin;
	int usb_switch_pin;	
	int high_delay;
	int low_delay;
	struct delayed_work	toggle_work;
	int state; // high=1 low=0
    struct miscdevice mdev;
    int rf_kill_pin;
    int rf_state;
	int suspend_ind;
    unsigned long open;
    struct rfkillpin_attr attr_pin;
    struct rfkillpin_attr attr_toggle;
    spinlock_t rfkillpin_lock;
    unsigned long lock_flags;
    int suspend;
    int suspend_ind_a;
    struct device *dev;
    struct pinctrl *pctl;
    int	portable;
};

int proc_rf_kill_pin = -1;


ssize_t proc_rfkillpin_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size >= 1 && 0 == *ppos) {
		if (gpio_is_valid(proc_rf_kill_pin)) {
			if (0 == gpio_get_value(proc_rf_kill_pin))
				buf[0] = '0';	
			else
				buf[0] = '1';	
		}else
			buf[0] = 'e';

		if (size >= 2) {
			buf[1] = '\n';
			(*ppos) += 2;

			return 2;
		}

		(*ppos) ++;

		return 1;
	}

	return 0;
}

static const struct file_operations proc_rfkill_operations = {
	.read		= proc_rfkillpin_read,
};

static ssize_t rfkillpin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    //val = ('0' == *buf)?0:1;

    if(gpio_is_valid(wdi->rf_kill_pin)){
        spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
        gpio_set_value(wdi->rf_kill_pin, val);
        wdi->rf_state = val;
        spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);
        return count;
    }

	return -EINVAL;
}

static ssize_t rfkillpin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", wdi->rf_state);
}

static ssize_t toggle_active_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    pr_notice("%s\n", buf);
    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    pr_notice("%d\n", val);
    if (gpio_is_valid(wdi->toggle_pin)) {
        if (val == !wdi->suspend) {
            pr_notice("eq nothing to do\n");
            return count;
        }

        if (0 == wdi->suspend) {
            cancel_delayed_work_sync(&wdi->toggle_work);
        }

        spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
        wdi->suspend = !val;
        spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);

        schedule_delayed_work(&wdi->toggle_work, 0);	
        return count;
    }

	return -EINVAL;
}

static ssize_t toggle_active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    return sprintf(buf, "%d\n", (0 == wdi->suspend));
}

static ssize_t rfkillpin_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int val;
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

    if (1 == count) {
        val = ('0' == *buf)?0:1;

		if(gpio_is_valid(wdi->rf_kill_pin)){
			gpio_set_value(wdi->rf_kill_pin, val);
            wdi->rf_state = val;
            return count;
		}
	}

	return 0;
}

static ssize_t rfkillpin_read(struct file * file, char __user * buf, size_t count, loff_t *ppos)
{
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

    if (0 == wdi->open) {
        return -EINVAL;
    }

    if (1 != count) {
		printk("%s: buffer too small\n", __func__);
		return -EINVAL;
	}

    *buf = '0' + wdi->rf_state;

    if (ppos) {
        *ppos = 0;
    }

    return count;
}

static int rfkillpin_open(struct inode *inode, struct file *file)
{
    struct miscdevice *md =  file->private_data;
	struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

	if(test_and_set_bit(1, &wdi->open))
		return -EPERM;

	return 0;
}

static int rfkillpin_release(struct inode *inode, struct file *file)
{
    struct miscdevice *md =  file->private_data;
    struct watch_dog_pin_info *wdi = container_of(md, struct watch_dog_pin_info, mdev);

	clear_bit(1, &wdi->open);

	return 0;
}

static const struct file_operations rfkill_operations = {
    .owner      = THIS_MODULE,
	.read		= rfkillpin_read,
    .write      = rfkillpin_write,
    .open       = rfkillpin_open,
    .release    = rfkillpin_release,
};

static void watchdog_toggle_work(struct work_struct *work)
{
	struct watch_dog_pin_info *inf = container_of(work, struct watch_dog_pin_info, toggle_work.work);
    unsigned long d;

    if (0 != inf->suspend) {
        if (-1 == inf->suspend) {
            d = 0;
        } else {
            d = 1; //!inf->portable;
        }
        gpio_set_value(inf->toggle_pin, (int)d);
        return;
    }

    inf->state ^= 1; 
    d = (inf->state)?inf->high_delay:inf->low_delay;
    gpio_set_value(inf->toggle_pin, inf->state);

    schedule_delayed_work(&inf->toggle_work,msecs_to_jiffies(d));
}

static irqreturn_t port_det_handler(int irq, void *dev_id)
{
	struct watch_dog_pin_info *inf=dev_id;

    pr_notice("[%d]\n", gpio_get_value(inf->port_det_pin));
	if(0==gpio_get_value(inf->port_det_pin))
		gpio_direction_output(inf->usb_switch_pin,0);	
	else
		gpio_direction_output(inf->usb_switch_pin,1);	
	return IRQ_HANDLED;
}

static int watchdog_pin_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct device_node *np = op->dev.of_node;	
	struct watch_dog_pin_info *inf;
    struct pinctrl_state *pctls;
	int rc;
	int irq;
    const char *comp;

	if (!np) {
		dev_err(dev, "Can not find config!\n");
		return -ENOENT;
	}

	inf = devm_kzalloc(dev, sizeof(*inf), GFP_KERNEL);
	if (!inf) {
		dev_err(dev, "Memory exhausted!\n");
		return -ENOMEM;
	}


    comp = of_get_property(np, "compatible", NULL);
    if (comp && 0 == strncmp("mcn,fixed-watchdog-pin", comp, sizeof("mcn,fixed-watchdog-pin") - sizeof(char))) {
        inf->portable = 0;
    } else {
        inf->portable = 1;
    }

    pr_notice("TAB8 %s \n", (1 == inf->portable)?"portable":"fixed");

    inf->pctl = devm_pinctrl_get(dev);
    if (IS_ERR(inf->pctl)) {
        if (PTR_ERR(inf->pctl) == -EPROBE_DEFER) {
            dev_err(dev, "pin ctl critical error!\n");
            return -EPROBE_DEFER;
        }

        pr_notice("%s: pin control isn't used\n", __func__);
        inf->pctl = 0;
    }

    if (inf->pctl) {
        pctls = pinctrl_lookup_state(inf->pctl, "watchdog_pin_active");
        if (IS_ERR(pctls)) {
            dev_err(dev, "failure to get pinctrl active state\n");
            return PTR_ERR(pctls);
        }
        rc = pinctrl_select_state(inf->pctl, pctls);
        if (rc) {
            dev_err(dev, "cannot set ts pinctrl active state\n");
            return rc;
        }
    }


	inf->toggle_pin = of_get_named_gpio(np,"mcn,toggle-pin",0);
	if(inf->toggle_pin < 0){
		dev_err(dev, "Memory exhausted!\n");
		return -ENOMEM;		
	}
	rc=devm_gpio_request(dev,inf->toggle_pin, "watchdog_pin");
	if(rc < 0){
		dev_err(dev, "toggle pin is busy!\n");
		return -ENOMEM;			
	}

	inf->port_det_pin=
		of_get_named_gpio(np,"mcn,port-det-pin",0);
	if(gpio_is_valid(inf->port_det_pin)){
		rc=devm_gpio_request(dev,inf->port_det_pin,"port_det");	
		if(rc<0)
			dev_err(dev, "port det pin is busy!\n");		
		gpio_direction_input(inf->port_det_pin);	
	}
	
	inf->usb_switch_pin=
		of_get_named_gpio(np,"mcn,usb-switch-pin",0);
	if (gpio_is_valid(inf->usb_switch_pin) && gpio_is_valid(inf->port_det_pin)) {
		rc=devm_gpio_request(dev,inf->usb_switch_pin,"usb_switch");	
		if(rc<0)
			dev_err(dev, "usb switch pin is busy!\n");
		if(0==gpio_get_value(inf->port_det_pin))
			gpio_direction_output(inf->usb_switch_pin,0);	
		else
			gpio_direction_output(inf->usb_switch_pin,1);
        gpio_export(inf->usb_switch_pin, 0);
        gpio_export(inf->port_det_pin, 0);

        irq= gpio_to_irq(inf->port_det_pin);
		rc = devm_request_threaded_irq(dev, irq, NULL,
				port_det_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"port_det_handler", inf);
		if (rc < 0) {
			dev_err(dev,
				"request_irq for irq=%d  failed rc = %d\n",
				irq, rc);
		}	
	}	

	proc_rf_kill_pin = inf->rf_kill_pin = of_get_named_gpio(np,"mcn,rf-kill-pin", 0);

    if (gpio_is_valid(inf->rf_kill_pin)) {
        rc = devm_gpio_request(dev,inf->rf_kill_pin,"rf-kill-pin");
        if (rc < 0) {
            dev_err(dev, "rf-kill-pin is busy!\n");
            return -ENOMEM;			
        }
        gpio_direction_output(inf->rf_kill_pin, 1);
        gpio_export(inf->rf_kill_pin, 0);
        inf->rf_state = 1;
    }
	
	inf->suspend_ind = of_get_named_gpio(np,"mcn,suspend-ind-pin", 0);

	if (gpio_is_valid(inf->suspend_ind)) {
		rc = devm_gpio_request(dev, inf->suspend_ind, "suspend-ind-pin");
		if (rc < 0) {
			pr_err("suspend-ind-pin is busy\n");
		} else {
            inf->suspend_ind_a = 0;
			gpio_direction_output(inf->suspend_ind, inf->suspend_ind_a^1);
			gpio_export(inf->suspend_ind, 0);
		}
	} else {
        inf->suspend_ind = -1;
		pr_err("invalid suspend-ind-pin\n");
	}

	rc = of_property_read_u32(np, "mcn,high-delay", &inf->high_delay);

	if(rc<0)
		inf->high_delay=1000;

	rc = of_property_read_u32(np, "mcn,low-delay", &inf->low_delay);
	if(rc<0)
		inf->low_delay=1000;	

    inf->dev = dev;
    rc = device_init_wakeup(dev, 1);
    if (0 != rc) {
        pr_err("failure init wake source\n"); 
    }

	dev_set_drvdata(dev, inf);

    spin_lock_init(&inf->rfkillpin_lock);
	
    inf->state = 0;//!inf->portable;
    if (inf->high_delay == inf->low_delay) {
        inf->suspend = -1;
    } else {
        inf->suspend = 0;
    }
    gpio_direction_output(inf->toggle_pin, inf->state);
    gpio_export(inf->toggle_pin, 0);
    pr_notice("init toggle pin[%d]\n", inf->state);
    if (inf->high_delay || inf->low_delay) {
        pr_notice("launch toggle work[%d]\n", inf->state);
        INIT_DELAYED_WORK(&inf->toggle_work, watchdog_toggle_work);
        schedule_delayed_work(&inf->toggle_work, msecs_to_jiffies(inf->low_delay));	
    } else {
        inf->state = 1;
        gpio_set_value(inf->toggle_pin, inf->state^1);
        pr_notice("notify to cradle PON[%d]\n", inf->state);
    }

    inf->mdev.minor = MISC_DYNAMIC_MINOR;
    inf->mdev.name	= "rf_kill_pin";
    inf->mdev.fops	= &rfkill_operations;
	misc_register(&inf->mdev);

    snprintf(inf->attr_pin.name, sizeof(inf->attr_pin.name) - 1, "rf_kill_pin");
    inf->attr_pin.attr.attr.name = inf->attr_pin.name;
    inf->attr_pin.attr.attr.mode = 0444;
    inf->attr_pin.attr.show = rfkillpin_show;
    inf->attr_pin.attr.store = rfkillpin_store;
    sysfs_attr_init(&inf->attr_pin.attr.attr);
    device_create_file(dev, &inf->attr_pin.attr);

    snprintf(inf->attr_toggle.name, sizeof(inf->attr_toggle.name) - 1, "wd_toggle_active");
    inf->attr_toggle.attr.attr.name = inf->attr_toggle.name;
    inf->attr_toggle.attr.attr.mode = 0664;
    inf->attr_toggle.attr.show = toggle_active_show;
    inf->attr_toggle.attr.store = toggle_active_store;
    sysfs_attr_init(&inf->attr_toggle.attr.attr);
    device_create_file(dev, &inf->attr_toggle.attr);

    proc_create("rfkillpin", S_IRUSR | S_IRGRP | S_IROTH, 0, &proc_rfkill_operations);

	return 0;
}

static int watchdog_pin_remove(struct platform_device *op)
{
//    struct watch_dog_pin_info *wdi = platform_get_drvdata(op);

    if (device_may_wakeup(&op->dev))
        device_wakeup_disable(&op->dev);
    return 0;
}

static int watchdog_pin_prepare(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);
    unsigned long d;

    d = (unsigned long)ktime_to_ms(ktime_get());

    pr_notice("notify to mcu/cradle about suspend [%d] %lld\n", wdi->state, ktime_to_ms(ktime_get()));

    spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
    if (wdi->suspend != -1) {
        wdi->suspend = 1; 
    }
    spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);

    cancel_delayed_work(&wdi->toggle_work);

    if(gpio_is_valid(wdi->rf_kill_pin)){
        pr_notice("shut down rf %lld\n", ktime_to_ms(ktime_get()));
        gpio_set_value(wdi->rf_kill_pin, wdi->rf_state^1);
    }

    if (gpio_is_valid(wdi->toggle_pin)) {
        pr_notice("set toggle pin to inactive [%d] %lld\n", wdi->state, ktime_to_ms(ktime_get()));
        d = 1; //!wdi->portable;
        if (wdi->high_delay == wdi->low_delay) {
            //d = 1;
        }
        gpio_set_value(wdi->toggle_pin, d);
    }

	if(gpio_is_valid(wdi->suspend_ind)){
        pr_notice("notify to mcu[obc5] about suspend %lld\n", ktime_to_ms(ktime_get()));
        gpio_set_value(wdi->suspend_ind, wdi->suspend_ind_a);
    }

    pr_notice("prepared %lld\n", ktime_to_ms(ktime_get()));

    return 0;
}

static int watchdog_pin_suspend(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
    if (wdi->suspend != -1) {
        wdi->suspend = 1; 
    }
    spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);
    pr_notice("suspended %lld\n", ktime_to_ms(ktime_get()));

    return 0;
}

static int watchdog_pin_resume(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    if (gpio_is_valid(wdi->toggle_pin)) {
        pr_notice("restore toggling [%d] %lld\n", wdi->state, ktime_to_ms(ktime_get()));
        spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
        if (wdi->suspend != -1) {
            wdi->suspend = 0; 
        }
        spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);
    }

	if(gpio_is_valid(wdi->suspend_ind)){
		pr_notice("notify to mcu about resume %lld\n", ktime_to_ms(ktime_get()));
		gpio_set_value(wdi->suspend_ind, wdi->suspend_ind_a^1);
	}

    if(gpio_is_valid(wdi->rf_kill_pin)){
		pr_notice("restore rf-kill [%d] %lld\n", wdi->rf_state, ktime_to_ms(ktime_get()));
        gpio_set_value(wdi->rf_kill_pin, wdi->rf_state);
    }

	return 0;
}

static void watchdog_pin_complete(struct device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(dev);

    if (wdi->suspend != -1) {
        wdi->suspend = 0; 
    }

    pr_notice("complete and restart toggling %lld\n", ktime_to_ms(ktime_get()));

    schedule_delayed_work(&wdi->toggle_work, 0);	

    return;
}

static void watchdog_pin_shutdown(struct platform_device *dev)
{
    struct watch_dog_pin_info *wdi = dev_get_drvdata(&dev->dev);

    cancel_delayed_work(&wdi->toggle_work);

    spin_lock_irqsave(&wdi->rfkillpin_lock, wdi->lock_flags);
    wdi->suspend = 1;
    spin_unlock_irqrestore(&wdi->rfkillpin_lock, wdi->lock_flags);

    if(gpio_is_valid(wdi->rf_kill_pin)){
        pr_notice("shutdown rf-kill [%d] %lld\n", wdi->rf_state, ktime_to_ms(ktime_get()));
        gpio_set_value(wdi->rf_kill_pin, wdi->rf_state^1);
    }

    if (gpio_is_valid(wdi->toggle_pin)) {
        pr_notice("set toggle pin to inactive [%d] %lld\n", wdi->state, ktime_to_ms(ktime_get()));
        gpio_set_value(wdi->toggle_pin, 0);
    }

    return;
}

static const struct dev_pm_ops watchdog_pin_pm_ops =
{
	.prepare	= watchdog_pin_prepare,
    .complete   = watchdog_pin_complete,
    .suspend	= watchdog_pin_suspend,
    .resume		= watchdog_pin_resume,
};

static struct of_device_id watchdog_pin_match[] = {
	{ .compatible = "mcn,watchdog-pin", },
    { .compatible = "mcn,fixed-watchdog-pin", },
	{},
};

static struct platform_driver watchdog_pin_driver = {
	.probe		= watchdog_pin_probe,
	.remove		= watchdog_pin_remove,
    .shutdown   = watchdog_pin_shutdown,
	.driver		= {
		.name = "watchdog-pin",
		.owner = THIS_MODULE,
		.of_match_table = watchdog_pin_match,
        .pm = &watchdog_pin_pm_ops,
	},
};

module_platform_driver(watchdog_pin_driver);

MODULE_AUTHOR("skj at ehang Inc.");
MODULE_DESCRIPTION("WATCH DOG GPIO TOGGLE driver");
MODULE_LICENSE("GPL");


