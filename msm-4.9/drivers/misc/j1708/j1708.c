/*
 * MCU J1708 driver
 *
 * Copyright 2016 Micronet Inc., ruslan.sirota@micronet-inc.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
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

#include <linux/poll.h>
#include <linux/version.h>
#include <asm/uaccess.h>


MODULE_LICENSE("Dual BSD/GPL");

/////

#define J1708_MAX_MSG_EVENTS 128
#define J1708_MSG_PAYLOAD_SIZE 21
#define J1708_MSG_PACKET_SIZE 24
//#define J1708_DEBUG


/* NOTES:         dev_in = mcu_j1708  | dev_out = j1708
   (from MCU/iodriver) 	write_in    --|--> read_out   ex: cat /dev/j1708
	(to MCU/iodriver)   read_in    <--|--  write_out  ex: echo 'j1708packet' > /dev/j1708

   1. j1708 data comes from MCU to iodriver on /dev/ttyACM4. 
   2. j1708 frame is parsed and sent to /dev/mcu_j1708
   3. j1708 driver receives data at write_in, puts in a out_queue and read_out sends it /dev/j1708
   4. j1708 reads in packets via write_out and puts it in in_queue and read_in sends it to /dev/mcu_j1708  
 */

// TODO: make continous allocation
struct j1708_msg { /* 1 + 21 + 2 = 24 */ 
    uint8_t len;
	uint8_t data[J1708_MSG_PAYLOAD_SIZE];
	uint8_t padding[2];
};

// typical queue implementation
struct j1708_msg_queue {
	struct mutex lock;
	wait_queue_head_t wq;
	unsigned int head;
	unsigned int tail;
	struct j1708_msg msgs[J1708_MAX_MSG_EVENTS];
};

static inline int queue_empty(struct j1708_msg_queue * queue)
{
	return queue->head == queue->tail;
}

static inline struct j1708_msg * queue_get_msg(struct j1708_msg_queue * queue)
{
	queue->tail = (queue->tail + 1) % J1708_MAX_MSG_EVENTS;
	return &(queue->msgs[queue->tail]);
}

static void queue_add_msg(struct j1708_msg_queue * queue, struct j1708_msg * msg)
{
	queue->head = (queue->head + 1) % J1708_MAX_MSG_EVENTS;
	if(queue->head == queue->tail){
        pr_err("%s() q->head=q->tail %d\n", __func__, queue->head);
		queue->tail = (queue->tail + 1) % J1708_MAX_MSG_EVENTS;
	}
	//queue->msgs[queue->head] = msg; // ptr

    pr_err("%s() len %d, padding=%x, j1708msgSize=%d\n", __func__, msg->len, msg->padding[1], (unsigned int)sizeof(struct j1708_msg));
	memset(&queue->msgs[queue->head], 0, sizeof(struct j1708_msg));
	memcpy(&queue->msgs[queue->head], msg, sizeof(struct j1708_msg));

#ifdef J1708_DEBUG
	struct j1708_msg * msg_in_q;
	char * c_msg_in_q;
	int i;
	msg_in_q = &(queue->msgs[queue->head]);

	pr_err("%s() queue->msg[queue->head data], len = %d, padding[0]=%x, padding[1]=%x, sizeof(j1708_msg)=%d\n", 
			__func__, msg_in_q->len, msg_in_q->padding[0], msg_in_q->padding[1], (int)sizeof(struct j1708_msg));
	c_msg_in_q = (char *) msg_in_q;
	for (i = 0; i < sizeof(struct j1708_msg); i++){
	    pr_err("0x%02x, ", (uint8_t)*c_msg_in_q);
		c_msg_in_q++;
	}
	pr_err("\n");
#endif
}

struct virt_j1708 {
	struct j1708_msg_queue queue_in;
    struct j1708_msg_queue queue_out;
};

struct virt_j1708 g_virt_j1708_mcu_dev;

/////

static int j1708_dev_open(struct inode *inode, struct file *file)
{
	struct virt_j1708 * dev;

	dev = &g_virt_j1708_mcu_dev;

	file->private_data = dev;


	return 0;
}

static int j1708_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}


static unsigned int j1708_dev_poll_in(struct file * file,  poll_table * wait)
{
	struct virt_j1708 * dev = file->private_data;
	unsigned int mask = 0;

	mutex_lock(&dev->queue_out.lock);
	poll_wait(file, &dev->queue_out.wq, wait);
	if(!queue_empty(&dev->queue_out))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->queue_out.lock);

	return mask;
}

static ssize_t virt_j1708_chr_read_in(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_j1708 * dev = file->private_data;//&g_virt_j1708_mcu_dev;
    struct j1708_msg msg = {0};
	int read_count = 0;

	if(count < J1708_MSG_PAYLOAD_SIZE) {
		pr_err("%s() count < %d\n", __func__, J1708_MSG_PAYLOAD_SIZE);
		return -EINVAL;
	}

	if (count >= J1708_MSG_PAYLOAD_SIZE) {
		struct j1708_msg * pmsg = NULL;
        
		pr_err("%s(): got here1", __func__);
		mutex_lock(&dev->queue_out.lock);
		while(queue_empty(&dev->queue_out)) {
			mutex_unlock(&dev->queue_out.lock);
			if((file->f_flags &  O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->queue_out.wq, !queue_empty(&dev->queue_out));
			mutex_lock(&dev->queue_out.lock);
		}

		if( (pmsg = queue_get_msg(&dev->queue_out)) )
			memcpy(&msg, pmsg, sizeof(msg));

		mutex_unlock(&dev->queue_out.lock);
    	
		pr_err("%s(): got here2, msg_len=%d", __func__, msg.len);
		if(pmsg && msg.len) {
#ifdef J1708_DEBUG
        {
            int i;
            pr_err("%s() count %d\n", __func__, msg.len);
            pr_err("%s() data: ", __func__);
    		for (i = 0; i < msg.len; i++) {
                pr_err("0x%02x, ", (unsigned int)(msg.data[i]));
    		}
            pr_err("\n");
    	}
#endif
			if(copy_to_user(buf + read_count, msg.data, msg.len))
				return -EINVAL;

			read_count += msg.len;
		}
	}

	return read_count;
}

static ssize_t virt_j1708_chr_write_in(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_j1708 * dev = file->private_data;
	struct j1708_msg msg = {0};
	ssize_t w = 0;

	if(count > J1708_MSG_PAYLOAD_SIZE)
		return -EINVAL;

	// This will never block
    if(copy_from_user(msg.data, buf, count))
        return -EACCES;
	
	msg.len = (unsigned int)count;
#ifdef J1708_DEBUG
    {
        int i;
        pr_err("%s() msg.len %d\n", __func__, msg.len);
        pr_err("%s() msg.data: ", __func__);
		for (i = 0; i < count; i++) {
            pr_err("0x%02x ", (unsigned int)(msg.data[i]));
		}
        pr_err("\n");
	}
#endif

    mutex_lock(&dev->queue_in.lock);
    queue_add_msg(&dev->queue_in, &msg);
    mutex_unlock(&dev->queue_in.lock);

    wake_up_interruptible(&dev->queue_in.wq);
    w += J1708_MSG_PACKET_SIZE ;

	return w;
}

/// 
/* Transuctions to MCU */
static unsigned int j1708_dev_poll_out(struct file * file,  poll_table * wait)
{
	struct virt_j1708 * dev = file->private_data;
	unsigned int mask = 0;

	mutex_lock(&dev->queue_in.lock);
	poll_wait(file, &dev->queue_in.wq, wait);
	if(!queue_empty(&dev->queue_in))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->queue_in.lock);

	return mask;
}

static ssize_t virt_j1708_chr_read_out(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_j1708 * dev = file->private_data;//&g_virt_j1708_mcu_dev;
    struct j1708_msg msg = {0};
	int read_count = 0;

	if(count < J1708_MSG_PAYLOAD_SIZE) {
		pr_err("%s() count < %d\n", __func__, J1708_MSG_PAYLOAD_SIZE);
		return -EINVAL;
	}

	if (count >= J1708_MSG_PAYLOAD_SIZE) {
		struct j1708_msg * pmsg = NULL;

		mutex_lock(&dev->queue_in.lock);
		while(queue_empty(&dev->queue_in)) {
			mutex_unlock(&dev->queue_in.lock);
			if((file->f_flags &  O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->queue_in.wq, !queue_empty(&dev->queue_in));
			mutex_lock(&dev->queue_in.lock);
		}

		if( (pmsg = queue_get_msg(&dev->queue_in)) )
			memcpy(&msg, pmsg, sizeof(msg));

		mutex_unlock(&dev->queue_in.lock);

		if(pmsg && msg.len) {
#ifdef J1708_DEBUG
        {   
			char * c_msg = (char *)&msg;
            int i;
            pr_err("%s() msg.len=%d, msgSize=%d\n", __func__, msg.len, (unsigned int)sizeof(msg));
            pr_err("%s() data: ", __func__);
    		for (i = 0; i < sizeof(msg); i++) {
                pr_err("0x%02x,  ", (unsigned int)*c_msg);
				c_msg++;
    		}
            pr_err("\n");
    	}
#endif
			if(copy_to_user(buf + read_count, &msg, J1708_MSG_PACKET_SIZE))
				return -EINVAL;

			read_count += J1708_MSG_PACKET_SIZE;
		}
	}

	return read_count;
}

static ssize_t virt_j1708_chr_write_out(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_j1708 * dev = file->private_data;
	struct j1708_msg msg = {0};
	ssize_t w = 0;

	if(count > J1708_MSG_PAYLOAD_SIZE)
		return -EINVAL;

    if(copy_from_user(msg.data, buf, count))
        return -EACCES;

	msg.len = (unsigned int)count;
#ifdef J1708_DEBUG
    {
        int i;
        pr_err("%s() msg.len %d\n", __func__, msg.len);
        pr_err("%s() msg.data: ", __func__);
		for (i = 0; i < count; i++) {
            pr_err("0x%02x ", (unsigned int)(msg.data[i]));
		}
        pr_err("\n");
	}
#endif

    mutex_lock(&dev->queue_out.lock);
    queue_add_msg(&dev->queue_out, &msg);
    mutex_unlock(&dev->queue_out.lock);

    wake_up_interruptible(&dev->queue_out.wq);
    w += J1708_MSG_PACKET_SIZE;

	return w;
} 
/// 

static const struct file_operations j1708_dev_fops_in = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_j1708_chr_read_in,
	.write = virt_j1708_chr_write_in,
	.open = j1708_dev_open,
	.release = j1708_dev_release,
	.poll   = j1708_dev_poll_in,
};

static struct miscdevice j1708_dev_in = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "mcu_j1708",
	.fops		= &j1708_dev_fops_in,
};

static const struct file_operations j1708_dev_fops_out = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_j1708_chr_read_out,
	.write = virt_j1708_chr_write_out,
	.open = j1708_dev_open,
	.release = j1708_dev_release,
	.poll   = j1708_dev_poll_out,
};

static struct miscdevice j1708_dev_out = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "j1708",
	.fops		= &j1708_dev_fops_out,
};




/* old init function
static int __init virtual_j1708_init(void)
{
	struct virt_j1708 * dev = &g_virt_j1708_mcu_dev;
	int ret;

	init_waitqueue_head(&dev->queue_in.wq);
	mutex_init(&dev->queue_in.lock);

    init_waitqueue_head(&dev->queue_out.wq);
	mutex_init(&dev->queue_out.lock);


	ret = misc_register(&j1708_dev_in);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_in\n", __func__);
		return ret;
	}

    ret = misc_register(&j1708_dev_out);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_out\n", __func__);
        misc_deregister(&j1708_dev_in);
		return ret;
	}

	return 0;
}*/

static int virtual_j1708_probe(struct platform_device *op)
{
	struct virt_j1708 * dev = &g_virt_j1708_mcu_dev;
	int ret;

	init_waitqueue_head(&dev->queue_in.wq);
	mutex_init(&dev->queue_in.lock);

    init_waitqueue_head(&dev->queue_out.wq);
	mutex_init(&dev->queue_out.lock);


	ret = misc_register(&j1708_dev_in);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_in\n", __func__);
		return ret;
	}

    ret = misc_register(&j1708_dev_out);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_out\n", __func__);
        misc_deregister(&j1708_dev_in);
		return ret;
	}

	return 0;


}
/* old exit function (probaby bagged)
static void __exit virtual_j1708_exit(void)
{
	misc_register(&j1708_dev_in);
    misc_deregister(&j1708_dev_out);
}*/

static int virtual_j1708_remove(struct platform_device *op)
{
	int ret = 0;

  	ret = misc_deregister(&j1708_dev_in);
    ret += misc_deregister(&j1708_dev_out);

	return ret;
}  


/*
module_init(virtual_j1708_init);
module_exit(virtual_j1708_exit);
*/

static const struct of_device_id virtual_j1708_match[] = {
	{ .compatible = "virtual-j1708", },
	{},
};


static struct platform_driver virtual_j1708_driver = {
	.probe		= virtual_j1708_probe,
	.remove		= virtual_j1708_remove,
	.driver		= {
		.name	= "virtual_j1708",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(virtual_j1708_match),//right now (11/2018) the macro of_match_ptr() does nothing but 
                                                        	//it is always used in those structures in the kernel, 
															//probably to enable future changes to this mechanism so I went with the convention
	},
};


module_platform_driver(virtual_j1708_driver);


MODULE_AUTHOR("Barak Avigdory <barak.avigdory@micronet-inc.com>");
MODULE_DESCRIPTION("J1708 host interface");
MODULE_LICENSE("GPL");
