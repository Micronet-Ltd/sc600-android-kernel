
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/dirent.h>
#include <linux/string.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include "../../misc/hi_3w/hi_3w.h"

#define VINPUTS_NAME	"vinputs"
#define	FIND_NAME 		"vgpio_in"

extern int32_t gpio_in_register_notifier(struct notifier_block *nb);
extern int32_t gpio_in_unregister_notifier(struct notifier_block *nb);
extern int cradle_register_notifier(struct notifier_block *nb);

struct gpio_set {
	int base;
	int ngpio;
    int ain[2];
    int ain_l[2];
    int ain_irq[2];
    int ain_irq_mask;
};

struct work_params {
	int reason;
	int args;
};

struct vinput_key {
	unsigned int	code;
	int 			val;
};
//
//#define ABS_HAT0X		0x10
//#define ABS_HAT0Y		0x11
//#define ABS_HAT1X		0x12
//#define ABS_HAT1Y		0x13
//#define ABS_HAT2X		0x14
//#define ABS_HAT2Y		0x15
//#define ABS_HAT3X		0x16
//#define ABS_HAT3Y		0x17

static struct vinput_key vinputs[] = {
	{ABS_HAT0X, 0}, //KEY_F1
	{ABS_HAT0Y, 0},
	{ABS_HAT1X, 0},
	{ABS_HAT1Y, 0},
	{ABS_HAT2X, 0},
	{ABS_HAT2Y, 0},
	{ABS_HAT3X, 0},
	{ABS_HAT3Y, 0},
};
struct virt_inputs {
	struct 	miscdevice      *mdev;
	struct 	input_dev       *input_dev;
	struct  work_struct     work;
    struct  delayed_work    virtual_input_init_work;
    struct  notifier_block  virtual_inputs_cradle_notifier;
	struct 	work_params     wparams; 
	struct 	vinput_key      *vmap;
	struct	gpio_set 	    gpios_in;
	struct 	notifier_block  notifier;
	int		                reinit;
    int                     cradle_attached;
    int                     tlmm_based;
    struct mutex lock;
    struct pinctrl          *pctl;
};

static struct virt_inputs* vdev;

static int vinputs_init_files(void);

//#define DBG_INIT	1
//#ifdef	DBG_INIT
//char dbg_buf[1024] = {0};//temp!!!
//#endif
//////////////
inline static int is_gpios_exists(struct gpio_set* gpios)
{
	if (gpios->base > 0 && gpios->ngpio > 0) {
		return 1;
	}
	pr_info("%s: gpio failed base %d, ngpio %d\n", __func__, gpios->base, gpios->ngpio);
	return 0;
}

static int vinputs_get_gpios(int f_update)
{
	int i, err, val = 0, v = 0;
	int num = vdev->gpios_in.base;
	int qty = min_t(size_t, (vdev->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

    if (vdev->tlmm_based) {
        if (gpio_is_valid(vdev->gpios_in.ain[0])) {
            val |= (vdev->gpios_in.ain_l[0] == gpio_get_value(vdev->gpios_in.ain[0]));
        }
        if (gpio_is_valid(vdev->gpios_in.ain[1])) {
            val |= (vdev->gpios_in.ain_l[1] == gpio_get_value(vdev->gpios_in.ain[1])) << 1;
        }

        return val;
    }

    if (!is_gpios_exists(&vdev->gpios_in)) {
		return -1;
	}

	for (i = 0; i < qty; i++) {
		err = gpio_request(num, 0);
		if (!err && gpio_is_valid(num)) {

			v = gpio_get_value(num);
			gpio_free(num);

			val |= (v << i);
			if (f_update) {
				vdev->vmap[i].val = v;
			}
		} else {
			val |= (vdev->vmap[i].val << i);//don't change
			pr_err("%s: gpio_request failed err %d\n", __func__, err);
		}

		num++;
	}
	return val;
}

static int vinputs_get_val(void)
{
	int i, val = 0;
	int qty = min_t(size_t, (vdev->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

	for (i = 0; i < qty; i++) {
		val |= (vdev->vmap[i].val << i);
	}
	return val;
}

static ssize_t show_in_all(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = vinputs_get_val();

	return sprintf(buf, "0x%02X\n", val);
}

static ssize_t show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct virt_inputs *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *sensor_attr = to_sensor_dev_attr(attr);
	int ix = sensor_attr->index;

	if( (ix > ARRAY_SIZE(vinputs) - 1) || ix < 0 ){
		return sprintf(buf, "in%d - bad parameter\n", ix);
	}
	pr_notice("%s: %d\n", __func__, vdev->vmap[ix].code);
	return sprintf(buf, "%d\n", vdev->vmap[ix].val);
}

static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO, show_in, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, show_in, NULL, 5);
static SENSOR_DEVICE_ATTR(in6_input, S_IRUGO, show_in, NULL, 6);
static SENSOR_DEVICE_ATTR(in7_input, S_IRUGO, show_in, NULL, 7);
static DEVICE_ATTR(in_all, S_IRUGO, show_in_all, NULL);

static struct attribute *in_attributes[] = {
	// INPUTS
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	&dev_attr_in_all.attr,
	NULL
};

static const struct attribute_group in_attr_group = {
	.attrs = in_attributes
};

static irqreturn_t ain_irq_handler(int irq, void *arg)
{
	struct virt_inputs *vinp = (struct virt_inputs *)arg;
    struct irq_desc *irq_desc;
    int i;

    for (i = 0; i < 2; i++) {
        if ( vinp->gpios_in.ain_irq[i] == irq ) {
            irq_desc = irq_to_desc(irq);
            if (irq_desc) {
                if (irq_desc->irq_data.chip->irq_mask) {
                    irq_desc->irq_data.chip->irq_mask(&irq_desc->irq_data);
                    pr_notice("%s: mask irq[%d]\n", __func__, irq);
                }
                if (irq_desc->irq_data.chip->irq_ack) {
                    irq_desc->irq_data.chip->irq_ack(&irq_desc->irq_data);
                    pr_notice("%s: ack irq[%d]\n", __func__, irq);
                }
            }

            disable_irq_nosync(irq);
            vinp->gpios_in.ain_irq_mask |= (1 << i);

            break;
        }
    }

    schedule_work(&vinp->work); 

	return IRQ_HANDLED;
}

static void vinputs_work_func(struct work_struct *work) 
{
	struct virt_inputs *vinp  = container_of(work, struct virt_inputs, work);
    struct device_node *np;
    struct irq_desc *desc;
    struct irq_desc *irq_desc;
	unsigned long val = -1;
	int i, v, sync = 0;
	int qty = min_t(size_t, (vinp->gpios_in.ngpio), (ARRAY_SIZE(vinputs)));

	if (vinp->reinit) {
		pr_notice("%s: init\n", __func__);

        mutex_lock(&vinp->lock);
        if (vinp->tlmm_based) {
            vinp->gpios_in.base = 0;
            vinp->gpios_in.ngpio = 0;
            np = of_find_compatible_node(NULL, NULL, "mcn,tlmm-based-vinputs");
            if (np) {
                val = of_get_named_gpio_flags(np, "mcn,ain-0", 0, (enum of_gpio_flags *)&v);
                if (gpio_is_valid(val)) {
                    vinp->gpios_in.ngpio++;
                    vinp->gpios_in.ain[0] = val;
                    vinp->gpios_in.ain_l[0] = !v;
                } else {
                    vinp->gpios_in.ain[0] = -1;
                    vinp->gpios_in.ain_l[0] = -1;
                }

                if (gpio_is_valid(vinp->gpios_in.ain[0])) {
                    val = devm_gpio_request(vinp->mdev->this_device, vinp->gpios_in.ain[0], "ain-0-state");
                    if (val >= 0) {
                        val = gpio_direction_input(vinp->gpios_in.ain[0]);
                        if (val >= 0) {
                            gpio_export(vinp->gpios_in.ain[0], 1);

                            vinp->gpios_in.ain_irq[0] = gpio_to_irq(vinp->gpios_in.ain[0]);
                            if (vinp->gpios_in.ain_irq[0] < 0) {
                                dev_err(vinp->mdev->this_device, "failure to request gpio[%d] irq\n", vinp->gpios_in.ain[0]);
                            } else {
                                val = devm_request_irq(vinp->mdev->this_device, vinp->gpios_in.ain_irq[0], ain_irq_handler,
                                                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                                       vinp->mdev->name, vinp);
                                if (!val) {
                                    disable_irq_nosync(vinp->gpios_in.ain_irq[0]);
                                } else {
                                    dev_err(vinp->mdev->this_device, "failure to request irq[%d] irq -- polling available\n",
                                            vinp->gpios_in.ain_irq[0]);
                                }
                            }
                        }
                    }
                }

                val = of_get_named_gpio_flags(np, "mcn,ain-1", 0, (enum of_gpio_flags *)&v);
                if (gpio_is_valid(val)) {
                    vinp->gpios_in.ngpio++;
                    vinp->gpios_in.ain[1] = val;
                    vinp->gpios_in.ain_l[1] = !v;
                } else {
                    vinp->gpios_in.ain[1] = -1;
                    vinp->gpios_in.ain_l[1] = -1;
                }

                if (gpio_is_valid(vinp->gpios_in.ain[1])) {
                    val = devm_gpio_request(vinp->mdev->this_device, vinp->gpios_in.ain[1], "ain-1-state");
                    if (val >= 0) {
                        val = gpio_direction_input(vinp->gpios_in.ain[1]);
                        if (val >= 0) {
                            gpio_export(vinp->gpios_in.ain[1], 1);

                            vinp->gpios_in.ain_irq[1] = gpio_to_irq(vinp->gpios_in.ain[1]);
                            if (vinp->gpios_in.ain_irq[1] < 0) {
                                dev_err(vinp->mdev->this_device, "failure to request gpio[%d] irq\n", vinp->gpios_in.ain[1]);
                            } else {
                                val = devm_request_irq(vinp->mdev->this_device, vinp->gpios_in.ain_irq[1], ain_irq_handler,
                                                       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                                       vinp->mdev->name, vinp);
                                if (!val) {
                                    disable_irq_nosync(vinp->gpios_in.ain_irq[1]);
                                } else {
                                    dev_err(vinp->mdev->this_device, "failure to request irq[%d] irq -- polling available\n",
                                            vinp->gpios_in.ain_irq[1]);
                                }
                            }
                        }
                    }
                }
                of_node_put(np);
            }

            qty = vinp->gpios_in.ngpio;

            if (vinp->gpios_in.ngpio > 0) {
                vinp->gpios_in.ain_irq_mask = 3;
                val = 0;
            } else {
                val = -1;
            }
        } else {
            val = vinputs_init_files();
        }
        if (0 == val) {
            vinp->reinit = 0; 
            pr_notice("%s: succeed\n", __func__);
        }
        mutex_unlock(&vinp->lock);
	}

    if (vinp->reinit) {
        pr_notice("%s: failure\n", __func__);
        schedule_work(&vinp->work);
        return;
    }

	val = vinputs_get_gpios(0); 

	pr_notice("%s: gpio 0x%X\n", __func__, (unsigned int)val);

	for (i = 0; i < qty; i++) {
		v = (test_bit(i, &val)) ? (KEY_F1 + i) : 0;

		if(vinp->vmap[i].val != v) {
			input_report_abs(vinp->input_dev, vinp->vmap[i].code, v);
			vinp->vmap[i].val = v;
			sync = 1;
			pr_notice("%s input%i = %d reported\n", __func__, i, (unsigned int)v);
		}
	}

	if (sync) {
		input_sync(vinp->input_dev);
        pr_notice("%s input synced\n", __func__);
	}

    if (vinp->tlmm_based) {
        for (i = 0; i < 2; i++) {
            if (vinp->gpios_in.ain_irq_mask & (1 << i)) {
                vinp->gpios_in.ain_irq_mask &= ~(1 << i);
                desc = irq_to_desc(vinp->gpios_in.ain_irq[i]);
                irq_desc = irq_to_desc(vinp->gpios_in.ain_irq[i]);
                if(desc->depth > 0) {
                    enable_irq(vinp->gpios_in.ain_irq[i]);
                    if (irq_desc) {
                        if (irq_desc && irq_desc->irq_data.chip->irq_ack) {
                            irq_desc->irq_data.chip->irq_ack(&irq_desc->irq_data);
                        }
                        if (irq_desc->irq_data.chip->irq_unmask) {
                            irq_desc->irq_data.chip->irq_unmask(&irq_desc->irq_data);
                        }
                    }
                }
            }
        }
    }
}

static void cradle_is_connected_work_fix(struct work_struct *work){
    uint32_t cmd = 0, tx_cmd = 0;
    static uint32_t inp_val[2] = {0}, vinp_code[2] = {0};
    unsigned int delay_val = 0;
    static int status_changed = 1;
    static int status_prev = 0;
    static int status_curr = 3;
    int cnt;
    int transmit_err;
    //int err_cnt = 0;

    struct virt_inputs *vinp = container_of(work, struct virt_inputs, virtual_input_init_work.work);
    
    if (status_changed) {
        for (cnt = 0; cnt<2; cnt++) {
            tx_cmd = cnt<<2;
            tx_cmd |=0x28;
            tx_cmd<<=24;
            if ((status_curr ^ status_prev) & (cnt+1)) {
                //pr_notice("request status%x\n", tx_cmd);
                if (vinp->cradle_attached & 0x10) {
                    msleep(20);
                    transmit_err = hi_3w_tx_cmd(&tx_cmd, 1);
                    /*while (transmit_err) {
                        pr_notice("transmit for input chnl-%d error %d\n", cnt, transmit_err);
                        tx_cmd = 0;
                        tx_cmd = cnt<<2;
                        tx_cmd |=0x28;
                        tx_cmd<<=24;
                        msleep(20);
                        transmit_err = hi_3w_tx_cmd(&tx_cmd, 1);
                        if (++err_cnt>=15) {
                            err_cnt = 0;
                            break;
                        }
                    }*/
                    inp_val[cnt] = tx_cmd;
                    inp_val[cnt] &=~(0xFF000000);
                    if (inp_val[cnt]>65535) {
                        pr_notice("voltage-%d of chnl-%d\n", inp_val[cnt], cnt);
                        inp_val[cnt] = 0;
                        pr_notice("was reset to 0 chnl-%d\n", cnt);
                    }
                } else {
                    inp_val[cnt] = 0;
                }
                vinp_code[cnt] = (inp_val[cnt] > 7000)?(KEY_F1 + cnt):0;
                input_report_abs(vinp->input_dev, vinp->vmap[cnt].code, vinp_code[cnt]);
                vinp->vmap[cnt].val = vinp_code[cnt];
                //pr_notice("input%d:code-%d, code in file-%d, voltage-%d\n", cnt, vinp->vmap[cnt].code, vinp->vmap[cnt].val, inp_val[cnt]);
            }
        }
        input_sync(vinp->input_dev);

        status_prev = status_curr;
        status_changed = 0;
        msleep(30);
    }

    if (vinp->cradle_attached & 0x10) {
        //pr_notice("input1 - %d, input2 - %d\n",inp_val[0], inp_val[1]);
        cmd = 0;
        //pr_notice("request status%x\n", cmd);
        hi_3w_tx_cmd(&cmd, 1);
        status_curr = (cmd & 0x18) >> 3;
        if (status_curr != status_prev) {
            status_changed = 1;
            delay_val = 1000;
            //pr_notice("status was changed from %d to %d\n",status_prev, status_curr);
        } else {
            delay_val = 2000;
        }
        //pr_notice("responce status%x\n", cmd);
        schedule_delayed_work(&vdev->virtual_input_init_work, delay_val?msecs_to_jiffies(delay_val):0);
    }
}

static int32_t __ref vinputs_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
    struct virt_inputs *vinp = container_of(nfb, struct virt_inputs, notifier);

    pr_notice("%s: [%lu]\n", __func__, reason);

	if (0 == reason) {
        mutex_lock(&vinp->lock);
		// vinp->reinit = 1;
	}
	schedule_work(&vinp->work);

    if (0 == reason) {
        mutex_unlock(&vinp->lock);
    }

    return NOTIFY_OK;
}

static int __ref virtual_inputs_cradle_callback(struct notifier_block *nfb, unsigned long reason, void *p)
{
    struct virt_inputs *vinputs = container_of(nfb, struct virt_inputs, virtual_inputs_cradle_notifier);

    pr_notice("%s: cradle state %lu\n", __func__, reason);
    vinputs->cradle_attached = reason;
    cancel_delayed_work(&vdev->virtual_input_init_work);

    if (vinputs->cradle_attached & 0x10) {
        schedule_delayed_work(&vdev->virtual_input_init_work, 100); 
    } else if (vinputs->cradle_attached & 0x20) {
        if (!vdev->work.func) {
            vdev->reinit = 1;
            INIT_WORK(&vdev->work, vinputs_work_func);
            schedule_work(&vdev->work);
        }

        if (!vdev->notifier.notifier_call) {
            vdev->notifier.notifier_call = vinputs_callback; 
            pr_notice("register vgpio notifications %lld\n", ktime_to_ms(ktime_get()));
            if (gpio_in_register_notifier(&vdev->notifier)) {
                pr_err("failure to register vgpio notificationsr [%lld]\n", ktime_to_ms(ktime_get()));
                vdev->notifier.notifier_call = 0;
            }
        }
    } else {
        if (vdev->notifier.notifier_call) {
            pr_notice("%s: deregister vgpio notifications %lld\n", __func__, ktime_to_ms(ktime_get()));
            //gpio_in_unregister_notifier(&vdev->notifier);
            //vdev->notifier.notifier_call = 0;
        }
        if (vdev->work.func) {
            pr_notice("%s: cancel net944 based vgpio's work %lld\n", __func__, ktime_to_ms(ktime_get()));
            //cancel_work_sync(&vdev->work);
        }
    }

	return NOTIFY_OK;
}

static int vinputs_open(struct inode *inode, struct file *file)
{
	file->private_data = vdev;
	return 0;
}

static int vinputs_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int gpiochip_lable_match(struct gpio_chip* chip, void* data)
{
	return !strcmp(chip->label, data);
}

static int find_gpios_by_label(char* find_name, struct gpio_set* gset)
{
	struct gpio_chip *chip;
	chip = gpiochip_find(find_name, gpiochip_lable_match);

	if (!chip) {
		pr_err("%s: gpiochip not found, label %s\n", __func__, find_name);
		return 0;
	}

	gset->base = chip->base;
	gset->ngpio= chip->ngpio;

	pr_notice("%s: found base %d, num %d\n", __func__, gset->base, gset->ngpio );
	return gset->ngpio;
}

static int vinputs_init_files(void)
{
	if(find_gpios_by_label(FIND_NAME, &vdev->gpios_in) > 0) {
        if(is_gpios_exists(&vdev->gpios_in)) {
            return 0;
        }
	}
	return -1;
}

static const struct file_operations vinputs_dev_fops = {
	.owner      = THIS_MODULE,
	.open       = vinputs_open,
	.release    = vinputs_release,
	.llseek		= no_llseek,
};

static struct miscdevice vinputs_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= VINPUTS_NAME,
	.fops		= &vinputs_dev_fops,
};

static int vinputs_create_input_dev(struct virt_inputs* vdev)
{
	int error, i;

	vdev->input_dev = input_allocate_device();
	if (!vdev->input_dev) {
		pr_err("input_allocate_device failed\n");
		return -ENOMEM;
	}

	kfree(vdev->input_dev->name);
	vdev->input_dev->name = kstrndup(VINPUTS_NAME, sizeof(VINPUTS_NAME), GFP_KERNEL);

	input_set_capability(vdev->input_dev, EV_ABS, vdev->vmap[0].code);

	for (i = 0; i < ARRAY_SIZE(vinputs); i++) {
		input_set_abs_params(vdev->input_dev, vdev->vmap[i].code, 0, KEY_F1 + i, 0, 0); 
	}

	error = input_register_device(vdev->input_dev);
	if (error) {
		pr_err("input_register_device failed\n");
		input_free_device(vdev->input_dev);
		return error;
	 }

	return 0;
}

static int __init virtual_inputs_init(void)
{
	int error;
    int fixed_mode = 0, tlmm_based = 0;
	struct device *dev;
    struct device_node *np;

    np = of_find_compatible_node(NULL, NULL, "mcn,fixed-vinputs");
    if (np) {
        fixed_mode = 1;
        pr_notice("vinputs are stm32/k20 based\n");
    } else {
        np = of_find_compatible_node(NULL, NULL, "mcn,tlmm-based-vinputs");
        if (np) {
            pr_notice("vinputs are tlmm based\n");
            tlmm_based = 1;
        }
    }
	pr_info("%s:\n", __func__);
	vdev = kzalloc(sizeof(struct virt_inputs), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	vdev->vmap = vinputs;
	vdev->reinit = 0;
    vdev->tlmm_based = tlmm_based;

    error = misc_register(&vinputs_dev); 
    if (error) {
        pr_err("%s: misc_register failed\n", vinputs_dev.name);
        return -EINVAL;
    }
    vdev->mdev = &vinputs_dev;
    dev = vdev->mdev->this_device;

    if (vdev->tlmm_based) {
        struct platform_device *pdev;
        struct pinctrl_state *pctls;

        pdev = of_find_device_by_node(np);
        if (pdev) {
            vdev->pctl = devm_pinctrl_get(&pdev->dev); 
            if (IS_ERR(vdev->pctl)) {
                if (PTR_ERR(vdev->pctl) == -EPROBE_DEFER) {
                    dev_err(dev, "pin ctl critical error!\n");
                    misc_deregister(&vinputs_dev);
                    kfree(vdev);
                    of_node_put(np);
                    return -EPROBE_DEFER;
                }

                pr_notice("pin control isn't used\n");
                vdev->pctl = 0;
            }

            if (vdev->pctl) {
                pctls = pinctrl_lookup_state(vdev->pctl, "automotive_inputs_active");
                if (IS_ERR(pctls)) {
                    dev_err(dev, "failure to lookup active pinctrl\n");
                    misc_deregister(&vinputs_dev);
                    kfree(vdev);
                    of_node_put(np);
                    return PTR_ERR(pctls);
                }
                error = pinctrl_select_state(vdev->pctl, pctls);
                if (error) {
                    dev_err(dev, "failure to set pinctrl active state\n");
                    misc_deregister(&vinputs_dev);
                    kfree(vdev);
                    of_node_put(np);
                    return error;
                }
            }
        } else {
            dev_err(dev, "failure to access platform device pinctrl\n");
        }
    } else {
        if (fixed_mode) {
            vdev->virtual_inputs_cradle_notifier.notifier_call = virtual_inputs_cradle_callback;
            cradle_register_notifier(&vdev->virtual_inputs_cradle_notifier);
        }

        if(!fixed_mode){
            vdev->notifier.notifier_call = vinputs_callback;
            error = gpio_in_register_notifier(&vdev->notifier);
            if (error) {
                pr_err("failure to register remount notifier [%d]\n", error);
                misc_deregister(&vinputs_dev);
                kfree(vdev);
                of_node_put(np);
                return error;
            }
        }
    }
    of_node_put(np);

	error = vinputs_create_input_dev(vdev);
	if (error) {
		misc_deregister(&vinputs_dev);
		kfree(vdev);
		return error;
	}

    error = sysfs_create_group(&dev->kobj, &in_attr_group);
    if (error) {
        pr_err("%s: could not create sysfs group\n", __func__);
        input_free_device(vdev->input_dev);
        misc_deregister(&vinputs_dev);
        kfree(vdev);
        return error;
    }

    mutex_init(&vdev->lock);
    vdev->reinit = 1;
    if(!fixed_mode){
        INIT_WORK(&vdev->work, vinputs_work_func);
        schedule_work(&vdev->work); 
    }else{
        INIT_DELAYED_WORK(&vdev->virtual_input_init_work, cradle_is_connected_work_fix);
        vdev->cradle_attached = 0;
        schedule_delayed_work(&vdev->virtual_input_init_work, 2000);
    }
    pr_notice("%s:-\n", __func__);

	return 0;
}

static void __exit virtual_inputs_exit(void)
{
	struct device *dev= vinputs_dev.this_device;

    cancel_work_sync(&vdev->work);
	sysfs_remove_group(&dev->kobj, &in_attr_group);
	input_free_device(vdev->input_dev);
	misc_deregister(&vinputs_dev);
	kfree(vdev);
}

late_initcall(virtual_inputs_init);
module_exit(virtual_inputs_exit);

MODULE_LICENSE("GPL");

