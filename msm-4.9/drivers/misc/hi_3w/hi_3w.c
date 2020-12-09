/*
 * Copyright (C) 2019 Micronet Inc, All Right Reserved.
 *
 * Athor: vladimir.zatulovsky@micronet-inc.com
 *        marina.shaevich@micronet-inc.com
 *
 */

#define pr_fmt(fmt) "%s: " fmt, __func__
#include <linux/kconfig.h>
//#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/switch.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/notifier.h>

#include "hi_3w.h"

struct hi_3w_attr {
    struct device_attribute attr;
    char name[32];
};

struct hi_3w_device {   
    struct  work_struct work;
    int     hi_3w_clock_pin;
    int     hi_3w_mosi_pin;
    int     hi_3w_miso_pin;
    int     hi_3w_clock_active_l;
    int     hi_3w_mosi_active_l;
    int     hi_3w_miso_active_l;
    struct  wake_lock wlock;
    struct  device*    pdev;
    struct  pinctrl *pctl;
    struct  hi_3w_attr attr_cmd2slv;
    struct  mutex lock;
};

struct hi_3w_device* hi_dev;

int wait_for(int time){
    long long timer;

    timer = ktime_to_us(ktime_get()) + time;

    while (ktime_to_us(ktime_get()) < timer);

    return 0;
}

int hi_3w_tx_cmd(uint32_t *cmd, bool wait_for_response)
{
    uint8_t msg;
    int response_bit_cnt = 0;
    int cmd_bit_cnt = 0;    
    uint32_t response;
    int ack_val; 
    int initial_val;
    int err = GENERAL_ERROR;
    int time_tresh;
    //int i; 

    if (!hi_dev || !cmd) {
        return GENERAL_ERROR;
    }

    if (!gpio_is_valid(hi_dev->hi_3w_clock_pin) || !gpio_is_valid(hi_dev->hi_3w_mosi_pin) || !gpio_is_valid(hi_dev->hi_3w_miso_pin)) {
        pr_err("one of the gpio is invalid %d\n", *cmd);
        return NOT_AVAILABLE;
    }

    //read the inital state of the input pin
    mutex_lock(&hi_dev->lock);
    time_tresh = ktime_to_us(ktime_get());
    /*time_tresh = ktime_to_us(ktime_get()) + 30000;
    while (ktime_to_us(ktime_get())<time_tresh) {
        initial_val = gpio_get_value(hi_dev->hi_3w_miso_pin);
        if (!initial_val) {
            time_tresh = ktime_to_us(ktime_get()) + 30000;
        }
    }*/
    initial_val = gpio_get_value(hi_dev->hi_3w_miso_pin);
    //mutex_unlock(&hi_dev->lock);

    //pr_notice("slave%shold miso\n", (initial_val)?" not ":" ");

    if (initial_val == GPIO_LOW) {
        pr_err("3w arbitration failure\n");
        // error, something went wrong, the input should be set to high
        mutex_unlock(&hi_dev->lock);
        return NOT_AVAILABLE;
    }

    //wake the MCU
    msg = (*cmd)>>24;
    //pr_notice("the command to send is: %x\n", msg);
    //pr_notice("wake slave\n");

    //mutex_lock(&hi_dev->lock);
    gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_LOW);    
    wait_for(WAIT_DELAY_INTERVAL<<3); 
    /*time_tresh = ktime_to_us(ktime_get()) + 30000;
    while (ktime_to_us(ktime_get())<time_tresh) {
        initial_val = gpio_get_value(hi_dev->hi_3w_miso_pin);
        if (initial_val) {
            time_tresh = ktime_to_us(ktime_get()) + 30000;
        }
    }*/
    gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_HIGH);   

    //send the command to MCU
    /*for (i = 0; i < CMD_LEN; i++) {  
        gpio_set_value(hi_dev->hi_3w_mosi_pin, (msg >> i)&1);        
        wait_for(WAIT_DELAY_INTERVAL);
        gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_LOW);
        wait_for(WAIT_DELAY_INTERVAL);
        gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_HIGH);
    }*/

    do {
        gpio_set_value(hi_dev->hi_3w_mosi_pin, msg & 1);        
        wait_for(WAIT_DELAY_INTERVAL);
        gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_LOW);
        wait_for(WAIT_DELAY_INTERVAL);
        gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_HIGH);

        msg >>= 1;
        cmd_bit_cnt++;
    }while (cmd_bit_cnt < CMD_LEN);

    //read the ack from MCU
    wait_for(WAIT_DELAY_INTERVAL);
    gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_LOW);
    wait_for(WAIT_DATA_VALID);
    ack_val = gpio_get_value(hi_dev->hi_3w_miso_pin);
    wait_for(WAIT_DATA_INVALID);
    gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_HIGH);

    if (ack_val == GPIO_LOW) {
         //ACK is received
        if (wait_for_response){
            //read the response
            //pr_notice("waiting for response...\n");
            response = 0;
            do {
                wait_for(WAIT_DELAY_INTERVAL);
                gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_LOW);
                wait_for(WAIT_DATA_VALID);
                response |= (gpio_get_value(hi_dev->hi_3w_miso_pin) << response_bit_cnt);
                wait_for(WAIT_DATA_INVALID);
                gpio_set_value(hi_dev->hi_3w_clock_pin, GPIO_HIGH);
                
                response_bit_cnt++; 
            }while (response_bit_cnt < RESPONSE_MSG_LEN);

            (*cmd) |= response;
            //pr_notice("the response is: %d\n", response);
        }
        err = NO_ERROR;
    } else {
        time_tresh = ktime_to_us(ktime_get()) - time_tresh;
        pr_notice("request NACKed on cmd %d [%d, %d]\n", *cmd, response, time_tresh);
        //NACK is received

        err = CMD_REJECTED;
    }
    mutex_unlock(&hi_dev->lock);

    return err;
}
EXPORT_SYMBOL_GPL(hi_3w_tx_cmd);


static ssize_t hi_3w_rx_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "not supported\n");
}

static ssize_t hi_3w_tx_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct hi_3w_device *hi3w = dev_get_drvdata(dev);
    uint32_t cmd, raw;
    int err = -EINVAL, w;

    if (count < sizeof(cmd) + 2*sizeof(char)) {
        return err;
    }

    mutex_lock(&hi3w->lock);
    err = kstrtou32(buf, 16, &raw);
    mutex_unlock(&hi3w->lock);

    pr_notice("tx [%d]\n", raw);
    if (0 == err) {
        mutex_lock(&hi3w->lock);
        cmd = (raw >> 8) << 24;
        w = raw & 0xFF;
        mutex_unlock(&hi3w->lock);

        err = hi_3w_tx_cmd(&cmd, w);

        if (!err) {
            err = count; 
        }
    }

    return err; 
}

static int hi_3w_probe(struct platform_device *pdev)
{
    int err = GENERAL_ERROR;
    //struct hi_3w_device *hi_dev;
    struct device *dev = &pdev->dev;
    struct device_node *np;
    struct pinctrl_state *pctls;

    np = dev->of_node;
    if (!np) {
        pr_err("failure to find device tree\n");
        return -EINVAL;
    }

    hi_dev = devm_kzalloc(dev, sizeof(struct hi_3w_device), GFP_KERNEL);
    if (!hi_dev)
        return -ENOMEM;

    do {
        hi_dev->pctl = devm_pinctrl_get(dev);
        if (IS_ERR(hi_dev->pctl)) {
            if (PTR_ERR(hi_dev->pctl) == -EPROBE_DEFER) {
                dev_err(dev, "pin ctl critical error!\n");
                err = -EPROBE_DEFER;
                break;
            }
 
            pr_notice("pin control isn't used\n");
            hi_dev->pctl = 0;
        } else {
            pr_notice("using pin control\n");
        }

        if (hi_dev->pctl) {
            pctls = pinctrl_lookup_state(hi_dev->pctl, "hi_3w_pins_active");
            if (IS_ERR(pctls)) {
                dev_err(dev, "failure to get pinctrl active state\n");
                err = PTR_ERR(pctls);
                break;
            }
            //set the pins as active
            err = pinctrl_select_state(hi_dev->pctl, pctls);
            if (err) {
                dev_err(dev, "failure to set pinctrl active state\n");
                break;
            }
            pr_notice("hi_3w_pins_active selected\n");
        }

        mutex_init(&hi_dev->lock);

        //INIT_WORK(&hi_dev->work, dock_switch_work_func);
        wake_lock_init(&hi_dev->wlock, WAKE_LOCK_SUSPEND, "hi_3w_wait_lock");

        //Init 3wire clock pin
        err = of_get_named_gpio_flags(np, "mcn,3w-clock-pin", 0, (enum of_gpio_flags *)&hi_dev->hi_3w_clock_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("invalid 3-wire clock pin\n");
            err = -EINVAL;
            break;
        }        

        hi_dev->hi_3w_clock_pin = err;
        hi_dev->hi_3w_clock_active_l = !hi_dev->hi_3w_clock_active_l;
        pr_notice("3-wire clock active level %s\n", (hi_dev->hi_3w_clock_active_l)? "high" : "low");

        //Init 3wire mosi pin
        err = of_get_named_gpio_flags(np, "mcn,3w-mosi-pin", 0, (enum of_gpio_flags *)&hi_dev->hi_3w_mosi_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("invalid 3-wire mosi pin\n");
            err = -EINVAL;
            break;
        }
        hi_dev->hi_3w_mosi_pin = err;
        hi_dev->hi_3w_mosi_active_l = !hi_dev->hi_3w_mosi_active_l;
        pr_notice("3-wire mosi active level %s\n", (hi_dev->hi_3w_mosi_active_l)? "high" : "low");

        //Init miso pin
        err = of_get_named_gpio_flags(np, "mcn,3w-miso-pin", 0, (enum of_gpio_flags *)&hi_dev->hi_3w_miso_active_l);
        if (!gpio_is_valid(err)) {
            pr_err("invalid 3-wire miso pin\n");
            err = -EINVAL;
            break;
        }
        hi_dev->hi_3w_miso_pin = err;
        hi_dev->hi_3w_miso_active_l = !hi_dev->hi_3w_miso_active_l;
        pr_notice("3-wire miso active level %s\n", (hi_dev->hi_3w_miso_active_l)? "high" : "low");

        if (gpio_is_valid(hi_dev->hi_3w_clock_pin)) {
            //Configure the clock pin as output
            err = devm_gpio_request(dev, hi_dev->hi_3w_clock_pin, "3w-clock");
            if (err < 0) {
                pr_err("failure to get the gpio[%d]\n", hi_dev->hi_3w_clock_pin);
                break;
            }
            err = gpio_direction_output(hi_dev->hi_3w_clock_pin, 1);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", hi_dev->hi_3w_clock_pin);
                break;
            }
            gpio_export(hi_dev->hi_3w_clock_pin, 0);
        }

        if (gpio_is_valid(hi_dev->hi_3w_mosi_pin)) {
            //Configure the mosi pin as output
            err = devm_gpio_request(dev, hi_dev->hi_3w_mosi_pin, "3w-mosi");
            if (err < 0) {
                pr_err("failure to get the gpio[%d]\n", hi_dev->hi_3w_mosi_pin);
                break;
            }
            err = gpio_direction_output(hi_dev->hi_3w_mosi_pin, 1);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", hi_dev->hi_3w_mosi_pin);
                break;
            }
            gpio_export(hi_dev->hi_3w_mosi_pin, 0);
        }

        if (gpio_is_valid(hi_dev->hi_3w_miso_pin)) {
            //Configure the miso pin as input
            err = devm_gpio_request(dev, hi_dev->hi_3w_miso_pin, "3w-miso");
            if (err < 0) {
                pr_err("failure to request the gpio[%d]\n", hi_dev->hi_3w_miso_pin);
                break;
            }
            err = gpio_direction_input(hi_dev->hi_3w_miso_pin);
            if (err < 0) {
                pr_err("failure to set direction of the gpio[%d]\n", hi_dev->hi_3w_miso_pin);
                break;
            }
            gpio_export(hi_dev->hi_3w_miso_pin, 1);
        }

        hi_dev->pdev = dev;               
        dev_set_drvdata(dev, hi_dev);
        //schedule_work(&hi_dev->work);

        //Init debug interface
        snprintf(hi_dev->attr_cmd2slv.name, sizeof(hi_dev->attr_cmd2slv.name) - 1, "cmd2slv");
        hi_dev->attr_cmd2slv.attr.attr.name = hi_dev->attr_cmd2slv.name;
        hi_dev->attr_cmd2slv.attr.attr.mode = S_IRUGO|S_IWUGO;
        hi_dev->attr_cmd2slv.attr.show = hi_3w_rx_show;
        hi_dev->attr_cmd2slv.attr.store = hi_3w_tx_store;
        sysfs_attr_init(&hi_dev->attr_cmd2slv.attr.attr);
        device_create_file(dev, &hi_dev->attr_cmd2slv.attr);

        return NO_ERROR;

    }while(0);

    if (gpio_is_valid(hi_dev->hi_3w_clock_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_clock_pin);
    if (gpio_is_valid(hi_dev->hi_3w_mosi_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_mosi_pin);
    if (gpio_is_valid(hi_dev->hi_3w_miso_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_miso_pin);
    devm_kfree(dev, hi_dev);
 
    pr_err("Error initializing\n");

    return err;
}

static int hi_3w_remove(struct platform_device *pdev)
{
    struct hi_3w_device *hi_dev = platform_get_drvdata(pdev);

    //cancel_work_sync(&hi_dev->work);   

    if (gpio_is_valid(hi_dev->hi_3w_clock_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_clock_pin);
    if (gpio_is_valid(hi_dev->hi_3w_mosi_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_mosi_pin);
    if (gpio_is_valid(hi_dev->hi_3w_miso_pin))
        devm_gpio_free(&pdev->dev, hi_dev->hi_3w_miso_pin);

    wake_lock_destroy(&hi_dev->wlock);

    dev_set_drvdata(&pdev->dev, 0);

    devm_kfree(&pdev->dev, hi_dev);

    return NO_ERROR;
}

static int hi_3w_suspend(struct device *dev)
{
    return NO_ERROR;
}

static int hi_3w_resume(struct device *dev)
{
    return NO_ERROR;
}

static const struct dev_pm_ops hi_3w_pm_ops = {
    .suspend    = hi_3w_suspend,
    .resume     = hi_3w_resume,
};

static struct of_device_id hi_3w_match[] = {
    { .compatible = "mcn,hi-3w", },
    {},
};

static struct platform_driver hi_3w_platform_driver = {
    .probe = hi_3w_probe,
    .remove = hi_3w_remove, 
    .driver = {
        .name = "hi3w",
        .owner = THIS_MODULE, 
        .of_match_table = hi_3w_match,
        .pm = &hi_3w_pm_ops,
    },
};

module_platform_driver(hi_3w_platform_driver);

MODULE_DESCRIPTION("3-wire host interface");
MODULE_AUTHOR("Vladimir Zatulovsky <vladimir.zatulovsky@micronet-inc.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tab8-hi-3w");

