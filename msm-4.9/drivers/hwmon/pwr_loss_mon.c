/**
 * pwr_loss_mon.c - Power Lost Monitoring Driver
 *
 * Copyright (C) 2013-2019 Micronet Inc.
 *
 * Written by Vladimir Zatulovsky 
 * <vladimir.zatulovsky@micronet-inc.com> 
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include "../../../../out/target/product/msm8953_64_c801/obj/kernel/msm-3.18/include/generated/autoconf.h"
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>

#include <linux/platform_device.h>

#include <linux/cpu.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/hwmon.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#if 0
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
//#include <linux/cpufreq.h>
#endif

struct pwr_loss_mon_attr {
    struct device_attribute attr;
    char name[32];
};

struct power_loss_monitor {
    struct device dev;
    struct device *hmd;
    struct pinctrl *pctl;
    struct power_supply *usb_psy;
    struct power_supply *bms_psy;
    struct delayed_work pwr_lost_work;
    struct pwr_loss_mon_attr attr_state;
    struct pwr_loss_mon_attr attr_inl;
    struct pwr_loss_mon_attr attr_wan_d;
    struct pwr_loss_mon_attr attr_wlan_d;
    struct pwr_loss_mon_attr attr_off_d;
    struct notifier_block pwr_loss_mon_cpu_notifier;
    struct notifier_block pwr_loss_mon_remount_notifier;
    struct notifier_block pwr_loss_mon_cradle_notifier;
    struct notifier_block pwr_loss_mon_vbus_notifier;
    struct wake_lock wlock;
	int    pwr_lost_irq;
    int    pwr_lost_pin;
    int    pwr_lost_pin_l;
	int    pwr_lost_off_cd;
    int    pwr_lost_off_d;
	int    pwr_lost_wan_cd;
    int    pwr_lost_wan_d;
    int    pwr_lost_wlan_cd;
    int    pwr_lost_wlan_d;
    int    sys_ready;
    int    cradle_attached;
    int    remount_complete;
    spinlock_t pwr_lost_lock;
    unsigned long lock_flags;
    long long pwr_lost_timer;
    int batt_is_scap;
    int portable;
};

extern int cradle_register_notifier(struct notifier_block *nb);
extern int power_ok_register_notifier(struct notifier_block *nb);

static RAW_NOTIFIER_HEAD(power_lost_chain);
static DEFINE_RAW_SPINLOCK(power_lost_chain_lock);

void power_loss_notify(struct power_loss_monitor *pwrl, unsigned long reason, void *arg)
{
	unsigned long flags;

    raw_spin_lock_irqsave(&power_lost_chain_lock, flags);
    raw_notifier_call_chain(&power_lost_chain, reason, 0);
    raw_spin_unlock_irqrestore(&power_lost_chain_lock, flags);

    // Vladimir:
    // TODO: make notifications available for user space
    // 
    //sysfs_notify(&pwrl->hmd->kobj, NULL, "state");
    //kobject_uevent(&pwrl->hmd->kobj, KOBJ_CHANGE);
}

static void adreno_suspend(struct power_loss_monitor *pwrl, int s)
{
#if 0
    int fd;
    char suspend[2];

    snprintf(suspend, sizeof(suspend) - 1, "%d", s);

    fd = sys_open("/sys/devices/soc.0/1c00000.qcom,kgsl-3d0/kgsl/kgsl-3d0/suspend", O_WRONLY, 0);
    if (fd < 0) {
        pr_err("/sys/devices/soc.0/1c00000.qcom,kgsl-3d0/kgsl/kgsl-3d0/suspend\n");
        return;
    }
    sys_write(fd, suspend, 1);
    sys_close(fd);
#endif
    pr_notice("[%d] not supported, camera must work\n", s);

    return;
}

static void wcnss_suspend(struct power_loss_monitor *pwrl, int s)
{
#if 0
    int fd;

    fd = sys_open("/sys/devices/soc.0/a000000.qcom,wcnss-wlan/power_loss", O_WRONLY, 0);
    if (fd < 0) {
        pr_err("/sys/devices/soc.0/a000000.qcom,wcnss-wlan/power_loss\n");
        return;
    }
    sys_write(fd, "1", 1);
    sys_close(fd);
#endif
    pr_notice("[%d] not supported, network must work\n", s);

    return;
}

void enable_irq_safe(int irq, int en)
{
    struct irq_desc *desc;

    desc = irq_to_desc(irq);

    if(en && desc->depth > 0) {
        enable_irq(irq);
    } else if (!en && 0 == desc->depth) {
        disable_irq_nosync(irq);
    }
}

// Vladimir
// TODO: implement remount completing wait
//

extern int emergency_remount_register_notifier(struct notifier_block *nb);

static void remount_ro(struct power_loss_monitor *pwrl)
{
    int fd, i = 0;

    fd = emergency_remount_register_notifier(&pwrl->pwr_loss_mon_remount_notifier);
    if (fd) {
        pr_err("failure to register remount notifier [%d]\n", fd);
        return;
    }

    fd = sys_open("/proc/sysrq-trigger", O_WRONLY, 0);
    if (fd < 0) {
        pr_notice("failure to open /proc/sysrq-trigger\n");
        return;
    }
    sys_write(fd, "u", 1);
    sys_close(fd);


    while (!pwrl->remount_complete && (i++ < 40)) {
        msleep(100);
    }

    return;
}

static void __ref pwr_loss_mon_work(struct work_struct *work)
{
    int val, err, usb_online;
    long long timer;
	struct power_loss_monitor *pwrl = container_of(work, struct power_loss_monitor, pwr_lost_work.work);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    timer = ktime_to_ms(ktime_get());
    val = gpio_get_value(pwrl->pwr_lost_pin);

    if (!pwrl->usb_psy) {
        pr_notice("usb power supply not ready %lld\n", ktime_to_ms(ktime_get()));
        pwrl->usb_psy = power_supply_get_by_name("usb");
    }

    if (pwrl->usb_psy) {
        usb_online = power_supply_is_system_supplied();
    } else {
        usb_online = 0;
    }

    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    if (pwrl->portable) {
        if (!pwrl->bms_psy) {
            pr_notice("bms power supply not ready %lld\n", ktime_to_ms(ktime_get()));
            pwrl->bms_psy = power_supply_get_by_name("bms");
        }

        if (!pwrl->bms_psy) {
            schedule_delayed_work(&pwrl->pwr_lost_work, msecs_to_jiffies(100));
            return;
        } else if (-1 == pwrl->batt_is_scap) {
            union power_supply_propval prop = {0,};

            err = pwrl->bms_psy->get_property(pwrl->bms_psy, POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
            if (err) {
                pr_notice("failure to get battery type %lld\n", ktime_to_ms(ktime_get()));
                enable_irq_safe(pwrl->pwr_lost_irq, 0);
                return;
            }
            pr_notice("battery type is %s %lld\n", prop.strval, ktime_to_ms(ktime_get()));
            err = strncmp(prop.strval, "Unknown Battery", strlen("Unknown Battery"));
            if (0 != err) {
                err = strncmp(prop.strval, "Disconnected Battery", strlen("Disconnected Battery"));
            }
            if (0 != err) {
                err = strncmp(prop.strval, "Loading Battery Data", strlen("Loading Battery Data"));
            }
            if (0 == err) {
                schedule_delayed_work(&pwrl->pwr_lost_work, msecs_to_jiffies(100));
                return;
            }
            err = strncmp(prop.strval, "c801_scap_4v2_135mah_30k", strlen("c801_scap_4v2_135mah_30k"));
            /*pwrl->cradle_attached = */pwrl->batt_is_scap = (0 == err);
        }
        if (0 == pwrl->batt_is_scap) {
            enable_irq_safe(pwrl->pwr_lost_irq, 0);
            return;
        }
    }

    if (pwrl->pwr_lost_timer <= 0) {
        spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pwrl->pwr_lost_wan_d = pwrl->pwr_lost_wlan_d = pwrl->pwr_lost_off_d = -1;
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        enable_irq_safe(pwrl->pwr_lost_irq, 1);

        return;
    }

    if (pwrl->pwr_lost_pin_l != val || (usb_online && (0 == pwrl->cradle_attached))) {
        enable_irq_safe(pwrl->pwr_lost_irq, 0);
        spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pwrl->pwr_lost_wan_d = pwrl->pwr_lost_wlan_d = pwrl->pwr_lost_off_d = -1;
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pr_notice("input power restored or usb charger attached %lld\n", timer);
        for (val = num_possible_cpus() - 1; val > 0; val--) {
            if (cpu_online(val))
                continue;
#if defined (CONFIG_SMP)
            pr_notice("voting for cpu%d up", val);
            err = cpu_up(val);
            if (err && err == notifier_to_errno(NOTIFY_BAD))
                pr_notice(" is declined\n");
            else if (err)
                pr_err(" failure. err:%d\n", err);
            else
                pr_notice("\n");
#endif
        }
        adreno_suspend(pwrl, 0);
        wcnss_suspend(pwrl, 0);
        power_loss_notify(pwrl, 0, 0);
        enable_irq_safe(pwrl->pwr_lost_irq, 1);
        wake_unlock(&pwrl->wlock);

        return;
    } else if (-1 == pwrl->pwr_lost_off_d) {
        pwrl->pwr_lost_wan_d = pwrl->pwr_lost_timer + pwrl->pwr_lost_wan_cd; 
        pwrl->pwr_lost_wlan_d = pwrl->pwr_lost_timer + pwrl->pwr_lost_wlan_cd;
        if (pwrl->pwr_lost_wlan_d <= pwrl->pwr_lost_wan_d) {
            pwrl->pwr_lost_wlan_d = pwrl->pwr_lost_wan_d + 1000;
        }
        pwrl->pwr_lost_off_d = pwrl->pwr_lost_timer + pwrl->pwr_lost_off_cd;
        if (pwrl->pwr_lost_off_d <= pwrl->pwr_lost_wlan_d) {
            pwrl->pwr_lost_off_d = pwrl->pwr_lost_wlan_d + 100;
        }
        pr_notice("input power lost %lld\n", timer);
        pr_notice("shutdown display not implemented yet %lld\n", ktime_to_ms(ktime_get()));
        power_loss_notify(pwrl, 1, 0);
        pr_notice("shudown adreno %lld\n", ktime_to_ms(ktime_get()));
        adreno_suspend(pwrl, 1);
        wake_lock(&pwrl->wlock);
        pr_notice("shutdown cores %lld\n", ktime_to_ms(ktime_get()));
        for (val = num_possible_cpus() - 1; val > 0; val--) {
            if (!cpu_online(val))
                continue;
#if defined (CONFIG_SMP)
            pr_notice("voting for cpu%d down", val);
            err = cpu_down(val);
            if (err)
                pr_err(" is failure [%d]\n", err);
            else
                pr_notice( "\n");
#endif
        }
        timer = (pwrl->pwr_lost_wan_d > ktime_to_ms(ktime_get()))?pwrl->pwr_lost_wan_d - ktime_to_ms(ktime_get()):0;
        pr_notice("wan will shutdown %lld + %lld\n", ktime_to_ms(ktime_get()), timer);
    }

    if (pwrl->pwr_lost_off_d < ktime_to_ms(ktime_get())) {
        spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pwrl->remount_complete = 0;
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

        pr_notice("urgent remount block devices ro %lld\n", ktime_to_ms(ktime_get()));

        sys_sync();
        if (1) {
            remount_ro(pwrl); 
        }
        pr_notice("urgent shutdown device %lld\n", ktime_to_ms(ktime_get()));
        if (1) {
            orderly_poweroff(1);
            return;
        }
        // temporrary for enhance cam bringup
        if (0) {
            timer = 300000; 
        }
    } else if (pwrl->pwr_lost_wlan_d < ktime_to_ms(ktime_get())) {
        pr_notice("wlan shutdown %lld\n", ktime_to_ms(ktime_get()));
        wcnss_suspend(pwrl, 1);

        timer = pwrl->pwr_lost_off_d - ktime_to_ms(ktime_get());
        pr_notice("device will shutdown %lld + %lld\n", ktime_to_ms(ktime_get()), timer);
    } else if (pwrl->pwr_lost_wan_d < ktime_to_ms(ktime_get())) {
        pr_notice("wan shutdown not supported %lld\n", ktime_to_ms(ktime_get()));

        timer = (pwrl->pwr_lost_wlan_d > ktime_to_ms(ktime_get()))?pwrl->pwr_lost_wlan_d - ktime_to_ms(ktime_get()):0;
        pr_notice("wlan will shutdown %lld + %lld\n", ktime_to_ms(ktime_get()), timer);
    }
    enable_irq_safe(pwrl->pwr_lost_irq, 1);

    schedule_delayed_work(&pwrl->pwr_lost_work, (timer)?msecs_to_jiffies(timer): 0);
}

static irqreturn_t pwr_loss_irq_handler(int irq, void *irq_data)
{
	struct power_loss_monitor *pwrl = irq_data;

	disable_irq_nosync(pwrl->pwr_lost_irq);
    cancel_delayed_work(&pwrl->pwr_lost_work);

    pr_notice("power loss is triggered %lld\n", ktime_to_ms(ktime_get()));

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    schedule_delayed_work(&pwrl->pwr_lost_work, 0); 

	return IRQ_HANDLED;
}

static int __ref pwr_loss_remount_callback(struct notifier_block *nfb, unsigned long a, void *arg)
{
    struct power_loss_monitor *pwrl = container_of(nfb, struct power_loss_monitor, pwr_loss_mon_remount_notifier);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->remount_complete = (unsigned int)a;
    pr_notice("remounted %d\n", pwrl->remount_complete);
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

	return NOTIFY_OK;
}

static int __ref pwr_loss_cpu_callback(struct notifier_block *nfb, unsigned long a, void *pcpu)
{
//	uint32_t cpu = (uintptr_t)pcpu;
    struct power_loss_monitor *pwrl = container_of(nfb, struct power_loss_monitor, pwr_loss_mon_cpu_notifier);

	if (a == CPU_UP_PREPARE || a == CPU_UP_PREPARE_FROZEN) {
		if (pwrl->pwr_lost_off_d != -1) {
			// pr_notice("prevent cpu%d up\n", cpu);
			return NOTIFY_BAD;
		}
        // pr_notice("voting for cpu%d up\n", cpu);
	}

	return NOTIFY_OK;
}

static int __ref pwr_loss_cradle_callback(struct notifier_block *nfb, unsigned long r, void *p)
{
    struct power_loss_monitor *pwrl = container_of(nfb, struct power_loss_monitor, pwr_loss_mon_cradle_notifier);

    pwrl->cradle_attached = r;
    pr_notice("cradle state %d\n", pwrl->cradle_attached);

	return NOTIFY_OK;
}

static int __ref pwr_loss_vbus_callback(struct notifier_block *nfb, unsigned long r, void *p)
{
    struct power_loss_monitor *pwrl = container_of(nfb, struct power_loss_monitor, pwr_loss_mon_vbus_notifier);

    pr_notice("vbus state %ld[%d]\n", r, pwrl->cradle_attached);

    if (!pwrl->cradle_attached) {
        enable_irq_safe(pwrl->pwr_lost_irq, 0); 

        cancel_delayed_work(&pwrl->pwr_lost_work);

        spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
        pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());
        spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

        schedule_delayed_work(&pwrl->pwr_lost_work, msecs_to_jiffies(1000)); 
    }

	return NOTIFY_OK;
}

int power_loss_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
    int err;

	raw_spin_lock_irqsave(&power_lost_chain_lock, flags);
	err = raw_notifier_chain_register(&power_lost_chain, nb);
	raw_spin_unlock_irqrestore(&power_lost_chain_lock, flags);

	return err;
}
EXPORT_SYMBOL(power_loss_register_notifier);

#if 0
// example notification using

static int power_lost_notifier(struct notifier_block *nb, unsigned long val, void *priv)
{
	struct <specific> *dev_specific = container_of(nb, struct <specific>, power_loss_notifier);

    // Do something
    //

	return NOTIFY_OK;
}

// In probe
power_loss_register_notifier(<specific> *dev_specific->power_loss_notifier)
#endif

static ssize_t pwr_loss_mon_show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "pwr-loss-mon");
}

static DEVICE_ATTR(name, 0444, pwr_loss_mon_show_name, 0);

static ssize_t pwr_loss_mon_in_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = -1, usb_online;
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);
    union power_supply_propval prop = {0,};

    if (!pwrl->usb_psy) {
        pr_notice("usb power supply not ready %lld\n", ktime_to_ms(ktime_get()));
        pwrl->usb_psy = power_supply_get_by_name("usb");
    }

    if (pwrl->usb_psy) {
        usb_online = power_supply_is_system_supplied();
    }

    if (gpio_is_valid(pwrl->pwr_lost_pin)) {
        val = gpio_get_value(pwrl->pwr_lost_pin);
    }

    if (0 != pwrl->bms_psy->get_property(pwrl->bms_psy, POWER_SUPPLY_PROP_BATTERY_TYPE, &prop) || !prop.strval) {
        prop.strval = "unknown";
    }

	return sprintf(buf, "Vbus is %s, Input power is %s, Config [%d, %d %s]\n", (usb_online)?"online":"offline", (val)?"plugged":"unplugged",
                   pwrl->portable, pwrl->batt_is_scap, prop.strval);
}

static ssize_t pwr_loss_mon_in_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_pin_l);
}

static ssize_t pwr_loss_mon_in_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_pin_l = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t pwr_loss_mon_offd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_off_cd);
}

static ssize_t pwr_loss_mon_offd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_off_cd = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t pwr_loss_mon_wand_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_wan_cd);
}

static ssize_t pwr_loss_mon_wand_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    if (kstrtos32(buf, 10, &val))
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_wan_cd = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static ssize_t pwr_loss_mon_wland_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwrl->pwr_lost_wlan_cd);
}

static ssize_t pwr_loss_mon_wland_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int val;
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    if (!pwrl->sys_ready) {
        return -EINVAL;
    }

    if (kstrtos32(buf, 10, &val)) 
        return -EINVAL;

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_wlan_cd = val;
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    return count;
}

static int pwr_loss_mon_probe(struct platform_device *pdev)
{
	int err, val;
	struct power_loss_monitor *pwrl;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    struct pinctrl_state *pctls;
    const char *c;

    np = dev->of_node; //of_find_compatible_node(0, 0, "a8-hm-power-lost");
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }

    pwrl = devm_kzalloc(dev, sizeof(struct power_loss_monitor), GFP_KERNEL);
    if (!pwrl) {
        pr_err("Add some memory\n");
        return -ENOMEM;
    }

    pwrl->sys_ready = 0;

    pwrl->batt_is_scap = -1;
    c = of_get_property(np, "compatible", 0);
    if (c) {
        pr_err("node is finded\n");
    }

    if (c && 0 == strncmp("mcn,pwr-loss-mon-scap", c, sizeof("mcn,pwr-loss-mon-scap") - sizeof(char))) {
        pwrl->portable = 1;
    } else {
        pwrl->portable = 0;
    }

    spin_lock_init(&pwrl->pwr_lost_lock);
    wake_lock_init(&pwrl->wlock, WAKE_LOCK_SUSPEND, "pwr_loss_suspend_lock");

    snprintf(pwrl->attr_state.name, sizeof(pwrl->attr_state.name) - 1, "state");
    pwrl->attr_state.attr.attr.name = pwrl->attr_state.name;
    pwrl->attr_state.attr.attr.mode = 0444;
    pwrl->attr_state.attr.show = pwr_loss_mon_in_show;
    sysfs_attr_init(&pwrl->attr_state.attr.attr);

    snprintf(pwrl->attr_inl.name, sizeof(pwrl->attr_inl.name) - 1, "level");
    pwrl->attr_inl.attr.attr.name = pwrl->attr_inl.name;
    pwrl->attr_inl.attr.attr.mode = 0666;
    pwrl->attr_inl.attr.show = pwr_loss_mon_in_l_show;
    pwrl->attr_inl.attr.store = pwr_loss_mon_in_l_store;
    sysfs_attr_init(&pwrl->attr_inl.attr.attr);

    snprintf(pwrl->attr_off_d.name, sizeof(pwrl->attr_off_d.name) - 1, "off_delay");
    pwrl->attr_off_d.attr.attr.name = pwrl->attr_off_d.name;
    pwrl->attr_off_d.attr.attr.mode = 0444;
    pwrl->attr_off_d.attr.show = pwr_loss_mon_offd_show;
    pwrl->attr_off_d.attr.store = pwr_loss_mon_offd_store;
    sysfs_attr_init(&pwrl->attr_off_d.attr.attr);

    snprintf(pwrl->attr_wan_d.name, sizeof(pwrl->attr_wan_d.name) - 1, "wan_off_delay");
    pwrl->attr_wan_d.attr.attr.name = pwrl->attr_wan_d.name;
    pwrl->attr_wan_d.attr.attr.mode = 0444;
    pwrl->attr_wan_d.attr.show = pwr_loss_mon_wand_show;
    pwrl->attr_wan_d.attr.store = pwr_loss_mon_wand_store;
    sysfs_attr_init(&pwrl->attr_wan_d.attr.attr);

    snprintf(pwrl->attr_wlan_d.name, sizeof(pwrl->attr_wlan_d.name) - 1, "wlan_off_delay");
    pwrl->attr_wlan_d.attr.attr.name = pwrl->attr_wlan_d.name;
    pwrl->attr_wlan_d.attr.attr.mode = 0444;
    pwrl->attr_wlan_d.attr.show = pwr_loss_mon_wland_show;
    pwrl->attr_wlan_d.attr.store = pwr_loss_mon_wland_store;
    sysfs_attr_init(&pwrl->attr_wlan_d.attr.attr);

    err = of_property_read_u32(np, "mcn,wan-off-delay", &val); 
    if (err < 0) {
        val = 0;
    }
    pwrl->pwr_lost_wan_cd = val;

    err = of_property_read_u32(np, "mcn,wlan-off-delay", &val); 
    if (err < 0) {
        val = 0;
    }
    pwrl->pwr_lost_wlan_cd = val;

    err = of_property_read_u32(np, "mcn,plat-off-delay", &val); 
    if (err < 0) {
        val = 300000;
    }
    pwrl->pwr_lost_off_cd = val;

    pr_notice("Initial parameters\n");
    pr_notice("shutdown after %d\n", pwrl->pwr_lost_off_cd);
    pr_notice("shutdown WAN after %d\n", pwrl->pwr_lost_wan_cd);
    pr_notice("shutdown WLAN after %d\n", pwrl->pwr_lost_wlan_cd);
    pwrl->pwr_lost_wan_d = pwrl->pwr_lost_wlan_d = pwrl->pwr_lost_off_d = -1;

    pwrl->pwr_loss_mon_cpu_notifier.notifier_call = pwr_loss_cpu_callback;
    pwrl->pwr_loss_mon_remount_notifier.notifier_call = pwr_loss_remount_callback;
    pwrl->pwr_loss_mon_cradle_notifier.notifier_call = pwr_loss_cradle_callback;
    pwrl->pwr_loss_mon_vbus_notifier.notifier_call = pwr_loss_vbus_callback;

    err = register_cpu_notifier(&pwrl->pwr_loss_mon_cpu_notifier);
    if (err) {
        pr_err("failure to register cpu notifier [%d]\n", err);
    }
    cradle_register_notifier(&pwrl->pwr_loss_mon_cradle_notifier);

    do {
        pwrl->pctl = devm_pinctrl_get(dev);
        if (IS_ERR(pwrl->pctl)) {
            if (PTR_ERR(pwrl->pctl) == -EPROBE_DEFER) {
                dev_err(dev, "pin ctl critical error!\n");
                err = -EPROBE_DEFER;
                break;
            }

            pr_notice("pin control isn't used\n");
            pwrl->pctl = 0;
        }

        if (pwrl->pctl) {
            pctls = pinctrl_lookup_state(pwrl->pctl, "pwr_loss_mon_active");
            if (IS_ERR(pctls)) {
                dev_err(dev, "failure to get pinctrl active state\n");
                err = PTR_ERR(pctls);
                break;
            }
            err = pinctrl_select_state(pwrl->pctl, pctls);
            if (err) {
                dev_err(dev, "failure to set pinctrl active state\n");
                break;
            }
        }

        val = of_get_named_gpio_flags(np, "mcn,pwr-loss-mon", 0, (enum of_gpio_flags *)&pwrl->pwr_lost_pin_l);
        if (!gpio_is_valid(val)) {
            pr_err("ivalid power lost detect pin\n");
            err = -EINVAL;
            break;
        }
        pwrl->pwr_lost_pin = val;
        pwrl->pwr_lost_pin_l = !pwrl->pwr_lost_pin_l;
        pr_notice("power loss indicator %d\n", pwrl->pwr_lost_pin);
        pr_notice("power loss active level %s\n", (pwrl->pwr_lost_pin_l)?"high":"low");



        pwrl->hmd = hwmon_device_register(dev);
    	if (IS_ERR(pwrl->hmd)) {
    		err = PTR_ERR(pwrl->hmd);
    		break;
    	}

        dev_set_drvdata(pwrl->hmd, pwrl);

        err = device_create_file(pwrl->hmd, &dev_attr_name);
    	if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_state.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_inl.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_off_d.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
        if (err) {
            err = -EINVAL;
            break;
        }
        err = device_create_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
    	if (err) {
            err = -EINVAL;
            break;
        }

        dev_set_drvdata(dev, pwrl);

        err = devm_gpio_request(dev, pwrl->pwr_lost_pin, "input-power-state");
        if (err < 0) {
            pr_err("failure to request the gpio[%d]\n", pwrl->pwr_lost_pin);
            break;
        }
		err = gpio_direction_input(pwrl->pwr_lost_pin);
		if (err < 0) {
            pr_err("failure to set direction of the gpio[%d]\n", pwrl->pwr_lost_pin);
            break;
		}
        gpio_export(pwrl->pwr_lost_pin, 0);

        INIT_DELAYED_WORK(&pwrl->pwr_lost_work, pwr_loss_mon_work);

		pwrl->pwr_lost_irq = gpio_to_irq(pwrl->pwr_lost_pin);
		if (pwrl->pwr_lost_irq < 0) {
            pr_err("failure to request gpio[%d] irq\n", pwrl->pwr_lost_pin);
		} else {
            err = devm_request_irq(dev, pwrl->pwr_lost_irq, pwr_loss_irq_handler,
                                   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                   pdev->name, pwrl);
            if (!err) {
                enable_irq_safe(pwrl->pwr_lost_irq, 0);
                device_init_wakeup(dev, 1);
            } else {
                pr_err("failure to request irq[%d] irq\n", pwrl->pwr_lost_irq);
            }
        }

		pwrl->pwr_lost_timer = -1;

        schedule_delayed_work(&pwrl->pwr_lost_work, (pwrl->pwr_lost_off_cd)?msecs_to_jiffies(pwrl->pwr_lost_off_cd): 0);
        power_ok_register_notifier(&pwrl->pwr_loss_mon_vbus_notifier);

        pwrl->sys_ready = 1;

		pr_notice("power lost detector registered (%d, %d)\n", pwrl->pwr_lost_pin, pwrl->pwr_lost_irq);

		return 0;
	}while(0);

    unregister_cpu_notifier(&pwrl->pwr_loss_mon_cpu_notifier);

    if (pwrl->pwr_lost_irq >= 0) {
        enable_irq_safe(pwrl->pwr_lost_irq, 0);
//        if (device_may_wakeup(dev))
//            disable_irq_wake(pwrl->pwr_lost_irq);
        device_wakeup_disable(dev);
        devm_free_irq(dev, pwrl->pwr_lost_irq, pwrl);
    }

    if(gpio_is_valid(pwrl->pwr_lost_pin))
        devm_gpio_free(dev, pwrl->pwr_lost_pin);

    dev_set_drvdata(dev, 0);
    dev_set_drvdata(pwrl->hmd, 0);

    if (!IS_ERR(pwrl->hmd)) {
        device_remove_file(pwrl->hmd, &dev_attr_name);
        device_remove_file(pwrl->hmd, &pwrl->attr_state.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_inl.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_off_d.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
        device_remove_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
        hwmon_device_unregister(pwrl->hmd);
    }

	devm_kfree(dev, pwrl);

	return err;
}

#if defined CONFIG_PM
static int pwr_loss_mon_suspend(struct device *dev)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    if (pwrl->pwr_lost_off_d != -1) {
        return -EINVAL;
    }

    enable_irq_safe(pwrl->pwr_lost_irq, 1);

    if (device_may_wakeup(dev))
        enable_irq_wake(pwrl->pwr_lost_irq);

    pr_notice("\n");

	return 0;
}

static int pwr_loss_mon_resume(struct device *dev)
{
    struct power_loss_monitor *pwrl = dev_get_drvdata(dev);

    enable_irq_safe(pwrl->pwr_lost_irq, 0);

    spin_lock_irqsave(&pwrl->pwr_lost_lock, pwrl->lock_flags);
    pwrl->pwr_lost_timer = ktime_to_ms(ktime_get());
    spin_unlock_irqrestore(&pwrl->pwr_lost_lock, pwrl->lock_flags);

    if (device_may_wakeup(dev))
        disable_irq_wake(pwrl->pwr_lost_irq);

    schedule_delayed_work(&pwrl->pwr_lost_work, 0);

	pr_notice("\n");

	return 0;
}
#else
#define pwr_loss_mon_suspend 0
#define pwr_loss_mon_resume 0
#endif

static const struct dev_pm_ops pwr_loss_mon_pm_ops =
{
	.suspend	= pwr_loss_mon_suspend,
	.resume		= pwr_loss_mon_resume,
};

static int pwr_loss_mon_remove(struct platform_device *pdev)
{
	struct power_loss_monitor *pwrl = platform_get_drvdata(pdev);

    cancel_delayed_work(&pwrl->pwr_lost_work);
    enable_irq_safe(pwrl->pwr_lost_irq, 0);
    wake_lock_destroy(&pwrl->wlock);
//    if (device_may_wakeup(&pdev->dev))
//        disable_irq_wake(pwrl->pwr_lost_irq);
    device_wakeup_disable(&pdev->dev);
    devm_free_irq(&pdev->dev, pwrl->pwr_lost_irq, pwrl);

    unregister_cpu_notifier(&pwrl->pwr_loss_mon_cpu_notifier);

    device_remove_file(pwrl->hmd, &dev_attr_name);
    device_remove_file(pwrl->hmd, &pwrl->attr_state.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_inl.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_off_d.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_wan_d.attr);
    device_remove_file(pwrl->hmd, &pwrl->attr_wlan_d.attr);
    dev_set_drvdata(pwrl->hmd, 0);
    hwmon_device_unregister(pwrl->hmd);

	if(gpio_is_valid(pwrl->pwr_lost_pin))
		devm_gpio_free(&pdev->dev, pwrl->pwr_lost_pin);
    dev_set_drvdata(&pdev->dev, 0);

	devm_kfree(&pdev->dev, pwrl);

	return 0;
}

static void pwr_loss_mon_shutdown(struct platform_device *pdev) {
    pwr_loss_mon_remove(pdev);
}

static const struct of_device_id of_pwr_loss_mon_match[] = {
	{ .compatible = "mcn,pwr-loss-mon", },
    { .compatible = "mcn,pwr-loss-mon-scap", },
	{},
};
MODULE_DEVICE_TABLE(of, of_pwr_loss_mon_match);

static struct platform_driver pwr_loss_mon_platform_driver = {
	.probe      = pwr_loss_mon_probe,
    .remove	    = pwr_loss_mon_remove,
    .shutdown   = pwr_loss_mon_shutdown,

	.driver = {
		.name = "pwr_loss_mon",
		.of_match_table = of_match_ptr(of_pwr_loss_mon_match),
        .pm = &pwr_loss_mon_pm_ops,
	},
};

static int __init pwr_loss_mon_init(void)
{
	return platform_driver_register(&pwr_loss_mon_platform_driver);
}

device_initcall(pwr_loss_mon_init);

MODULE_DESCRIPTION("Power Loss Hardware Monitor");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwr-loss-mon");

