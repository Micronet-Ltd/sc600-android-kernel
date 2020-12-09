#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/module.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/hwmon-sysfs.h>
#include "../misc/hi_3w/hi_3w.h"

MODULE_LICENSE("Dual BSD/GPL");

/////

//#define VGPIO_USE_SPINLOCK
//#define MCU_GPIO_USE_SPINLOCK

////////////////////////////////////////////////////////////////////////
#define NUMBER_OF_MCU_GPIOS 160
#define NUMBER_OF_MCU_GROUPS 5 					/*Barak - each group posses 32 independent GPIO    \
												   as for now (3/2019) In the current MCU the groups \
												   are called A B C D and E*/
#define TURN_A9_GPIO_NUM_TO_MCU(x) (((x >> 5) << 8) | (x & 0x1f))
#define IS_CONNECTED(reason) (reason & 1u) //look in switch_dock.c for enum NOTIFICATION_REASONS 
										   //to understand the logic here

//I'll rather use the next macros instead of using the conventional kernel functions
//because those functions recieves a 64 bit mask address and in the case of an error
//in the input a wrong GPIO in another port might be affected/read while the casting
//to uint32_t in the next macros will nullify the value if the bit_index will exceed 31
#define TEST_BIT32(bit_map, bit_index) (bit_map & (((uint32_t)1)<<bit_index))
#define SET_BIT32(bit_map, bit_index) (bit_map |= (((uint32_t)1)<<bit_index))
#define CLEAR_BIT32(bit_map, bit_index) (bit_map &= (~(((uint32_t)1)<<bit_index)))

//should be identical to the same enum in control.h
typedef enum
{
	SYNC_INFO = 0,
	COMM_WRITE_REQ = 1,
	COMM_READ_REQ = 2,
	COMM_READ_RESP = 3,
	PING_REQ = 4,
	PING_RESP = 5,
	GPIO_INT_STATUS = 6,
	POWER_MGM_STATUS = 7,
	ONE_WIRE_DATA = 8,
} packet_type_enum;

typedef enum
{
	SET = COMM_WRITE_REQ,
	GET = COMM_READ_REQ,
} mcu_gpio_op_t;

//from the MCU
typedef enum _gpio_pin_direction
{
	kGpioDigitalInput = 0U,
	kGpioDigitalOutput = 1U
} gpio_pin_direction_t;

typedef struct
{
	int gpio_num;
	mcu_gpio_op_t op;
} op_info_t;

typedef enum mcu_status
{
	NO_REQUEST = 0,
	REQUEST_SENT = 1,
	ERROR_SENDING = 2
} mcu_response_t;

typedef void(*p_out_set)(struct gpio_chip *, unsigned, int);
struct mcu_bank
{
	wait_queue_head_t mcu_wq; //will be used to sleep and wait for response from the MCU
							  //in case a value needs to be retrieved

#ifdef VGPIO_USE_SPINLOCK //Will be used to wait for response from the MCU
	spinlock_t lock;
#else
	struct mutex lock;
#endif
    op_info_t gpio_info;

	unsigned long mcu_gpio_value[5];
	unsigned long mcu_gpio_mask[5];
	unsigned long mcu_gpio_dir[5];

	int returned_gpio_val;
	volatile mcu_response_t returned_flag; 
    p_out_set pntr_func_gpio_set;

};

extern int cradle_register_notifier(struct notifier_block *nb);

/*
//defined in switch dock and is used to inform
struct notifier_block gpio_notifier ;

 unsigned long is_device_connected_to_mcu = 0;

int32_t __ref connected_to_mcu_gpio_callback(struct notifier_block *nfb, unsigned long reason, void *arg)
{
	//this action should be atomic for cases in which the device is for some reason
	// connected and disconnected very rapidly (for example if the physical connection is not stable)
	// We don't want the concurrent callbacks that access this variable to mix
	IS_CONNECTED(reason)? set_bit(1, &is_device_connected_to_mcu):
						  clear_bit(1, &is_device_connected_to_mcu);
    return NOTIFY_OK;
}*/

/////////////////////////////////////////////////////////////////
struct vgpio_bank {
	unsigned long gpio_mask;
	unsigned long gpio_value;
	wait_queue_head_t wq;
#ifdef VGPIO_USE_SPINLOCK
	spinlock_t lock;
#else
	struct mutex lock;
#endif
};
unsigned long g_gpio;

#ifdef VGPIO_USE_SPINLOCK
#define DEFINE_LOCK_FLAGS(x) int x
#define LOCK_BANK(lock, flags) do { spin_lock_irqsave(&lock, flags); } while (0)
#define UNLOCK_BANK(lock, flags) do { spin_unlock_irqrestore(&lock, flags); } while (0)
#else
#define DEFINE_LOCK_FLAGS(x) // NOOP
#define LOCK_BANK(lock, flags) do { mutex_lock(&lock); } while (0)
#define UNLOCK_BANK(lock, flags) do { mutex_unlock(&lock); } while (0)
#endif

struct virt_gpio {
	struct gpio_chip gpiochip_out;
	struct gpio_chip gpiochip_in;
	struct gpio_chip gpiochip_mcu;

	unsigned long inuse;

	// atomic bitset
	unsigned long gpi_values;
	struct vgpio_bank gpo_bank;
	struct mcu_bank mcu_gpio_bank;
    struct notifier_block virt_gpio_cradle_notifier;
	unsigned long enabled_out;
	unsigned long enabled_in;
    int     cradle_attached;
};

struct virt_gpio * g_pvpgio;

/////
static RAW_NOTIFIER_HEAD(gpio_in_chain);
static DEFINE_RAW_SPINLOCK(gpio_in_chain_lock);

static void gpio_in_notify(unsigned long reason, void *arg)
{
    unsigned long flags;

    raw_spin_lock_irqsave(&gpio_in_chain_lock, flags);
    raw_notifier_call_chain(&gpio_in_chain, reason, arg);
    raw_spin_unlock_irqrestore(&gpio_in_chain_lock, flags);
}

int32_t gpio_in_register_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int32_t err;

    raw_spin_lock_irqsave(&gpio_in_chain_lock, flags);
    err = raw_notifier_chain_register(&gpio_in_chain, nb);
    raw_spin_unlock_irqrestore(&gpio_in_chain_lock, flags);
    pr_notice("%s: %d\n", __func__, err);

    return err;
}
EXPORT_SYMBOL(gpio_in_register_notifier);

int32_t gpio_in_unregister_notifier(struct notifier_block *nb)
{
    unsigned long flags;
    int32_t err;

    pr_notice("%s\n", __func__);
    raw_spin_lock_irqsave(&gpio_in_chain_lock, flags);
    err = raw_notifier_chain_unregister(&gpio_in_chain, nb);
    raw_spin_unlock_irqrestore(&gpio_in_chain_lock, flags);

    return err;
}
EXPORT_SYMBOL(gpio_in_unregister_notifier);

///////debug access
static ssize_t show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, val = 0;

	for(i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++) {
		val |= (test_bit(i, &g_pvpgio->gpi_values) << i);
	}
	return sprintf(buf, "%02X\n", val);
}

static ssize_t set_in(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char str[8] = {0};
	int i;
	unsigned long mask = 0, val = 0, changed_bits = 0;

	if (5 != count) {
		pr_err("error: format XXXX (XX - mask, XX - value)\n");
		return count;
	}
	strncpy(str, buf, 2);
	mask = simple_strtol(str, NULL, 16);
	strncpy(str, buf + 2, 2);
	val = simple_strtol(str, NULL, 16);

	pr_notice("%s, mask 0x%X, val 0x%X\n", __func__, (unsigned int)mask, (unsigned int)val);

	for(i = 0; i < g_pvpgio->gpiochip_in.ngpio; i++) {
		if(test_bit(i, &mask)) {
			if(test_bit(i, &val) != test_bit(i, &g_pvpgio->gpi_values)) {

				if(test_bit(i, &val)) {
					pr_info("%s set   INPUT%d\n", __func__, i);
					set_bit(i, &g_pvpgio->gpi_values);
				} else {
					pr_info("%s: clear INPUT%d\n", __func__, i);
					clear_bit(i, &g_pvpgio->gpi_values);
				}
				changed_bits |= (i<<i);
			}
		}
	}
	if (changed_bits) {
		g_gpio = changed_bits;
		pr_notice("%s: 0x%X\n", __func__, (unsigned int)g_gpio);
		gpio_in_notify(1, &g_gpio);
	}

	return count;
}

static DEVICE_ATTR(dbg_inputs, S_IWGRP|S_IWUSR|S_IRUSR|S_IRGRP, show_in, set_in);

static struct attribute *in_attributes[] = {
	&dev_attr_dbg_inputs.attr,
	NULL
};

static const struct attribute_group in_attr_group = {
	.attrs = in_attributes
};

/////
static int vgpio_dev_open(struct inode *inode, struct file *file)
{
	struct virt_gpio * dev;
	DEFINE_LOCK_FLAGS(flags);

	dev = g_pvpgio;

	if(test_and_set_bit(1, &dev->inuse))
		return -EPERM;

	//dev = container_of(inode->i_cdev, struct virt_gpio, cdev);
	file->private_data = dev;

	LOCK_BANK(dev->gpo_bank.lock, flags);

	// Set all the bits in the mask so all values are updated first time
	dev->gpo_bank.gpio_mask = 0xff;

	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	// Does this need to be done, or is it redundant
	wake_up_interruptible(&dev->gpo_bank.wq);

	return 0;
}

static int vgpio_dev_release(struct inode *inode, struct file *file)
{
	struct virt_gpio * dev = file->private_data;

	clear_bit(1, &dev->inuse);

	return 0;
}

// TODO: Test
static unsigned int vgpio_dev_poll(struct file * file,  poll_table * wait)
{
	struct virt_gpio * dev = file->private_data;
	unsigned int mask = 0;
	int i = 0;
	uint64_t should_connect = 0;

	DEFINE_LOCK_FLAGS(flags);

	LOCK_BANK(dev->gpo_bank.lock, flags);
	poll_wait(file, &dev->gpo_bank.wq, wait);

	LOCK_BANK(dev->mcu_gpio_bank.lock, flags);
	//check if the mask is 0
	for (i = 0; i < NUMBER_OF_MCU_GROUPS; ++i) {
		should_connect += dev->mcu_gpio_bank.mcu_gpio_mask[i];
	}

	if (dev->gpo_bank.gpio_mask || should_connect) {
		mask |= POLLIN | POLLRDNORM;
	}

	UNLOCK_BANK(dev->mcu_gpio_bank.lock, flags);
	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	return mask;
}

static ssize_t virt_gpio_chr_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct virt_gpio *dev = file->private_data;
	uint8_t output[6]; //maximal possible data size needed to transfer info
	int output_size = 4; //start with a minimal output size
	int port = 0;
	unsigned long offset = 0;
	uint8_t value;
	unsigned long out_mask;
	unsigned long out_values = 0;
	uint32_t gpio_mask = 0;

	//pr_notice("%s() READ\n", __func__);

	if (count < output_size) {
		pr_err("%s() count too small\n", __func__);
		return -EINVAL;
	}

	//note that after this loop port holds the following index to the
	//index of the first port that has a gpio that should be set/get
	// and gpio_mask holds the bit map of this port
	while (!gpio_mask && port < 5) {	
		gpio_mask = dev->mcu_gpio_bank.mcu_gpio_mask[port];
		
		++port;
	}

	--port;
	//pr_notice("GPIO MASK! %u %d",gpio_mask,port);
	if (gpio_mask) {
		//pr_notice("READ_OR_WRITE!");

		//clean the bit in the mask
		dev->mcu_gpio_bank.mcu_gpio_mask[port] &= ~gpio_mask;

		//extract the value from the value mask
		value = !!(gpio_mask & (dev->mcu_gpio_bank.mcu_gpio_value[port]));

		//use the direction to check whether this is an output or input and use it to set the packet size
		output_size = ((dev->mcu_gpio_bank.mcu_gpio_dir[port])&gpio_mask) == kGpioDigitalInput? 5/*read request size*/:
																							6/*write request size*/;
		//get the offset
		while (1 != (gpio_mask&1)) {
			gpio_mask>>=1;
			++offset;
		}

		//pr_notice("port offset value: %u %lu %u",port, offset,value);

		//iodriver will set the first three parameters
		output[0] = 0; 
		output[1] = 0;
		output[2] = 0;
		output[3] = port;
		output[4] = offset;
		output[5] = value;
	} else {
		//pr_notice("GPIO_INT_STATUS!");
		LOCK_BANK(dev->gpo_bank.lock, flags);
		while (!dev->gpo_bank.gpio_mask) {
			UNLOCK_BANK(dev->gpo_bank.lock, flags);
			if ((file->f_flags & O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->gpo_bank.wq, dev->gpo_bank.gpio_mask);
			LOCK_BANK(dev->gpo_bank.lock, flags);
		}

		out_mask = dev->gpo_bank.gpio_mask;
		out_values = dev->gpo_bank.gpio_value;
		dev->gpo_bank.gpio_mask = 0;

		UNLOCK_BANK(dev->gpo_bank.lock, flags);

    	if(out_mask) {
            output[0] = 0; //iodriver will set that
            output[1] = 0;
            output[2] = out_mask & 0xff;
            output[3] = out_values & 0xff;
        } else {
            pr_debug("%s() return 0 no io change\n", __func__);
        }
	}

	if (copy_to_user(buf, output, output_size))
		return -EINVAL;
	*ppos = 0;

	pr_debug("%s() return %d\n", __func__, output_size);

	return output_size;
}

static ssize_t virt_gpio_chr_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct virt_gpio * dev = file->private_data;
	uint8_t msg[4];
	int i;
	uint8_t val = 0;

	if(count != 4)
		return -EINVAL;

//	pr_notice("virt_gpio_chr_write!");

	if(copy_from_user(msg, buf, count))
		return -EACCES;

	if(msg[0] || msg[1]) {
		pr_err("%s() unsupported command %x,%x\n", __func__, msg[0], msg[1]);
		return -EINVAL;
	}

//	pr_notice("writing %02d %02d\n", msg[2], msg[3]);

	//Now check whether a user thread is waiting for a response
	if (REQUEST_SENT == dev->mcu_gpio_bank.returned_flag) {
		if (-1 == msg[2]) {
			dev->mcu_gpio_bank.returned_flag = ERROR_SENDING;
		} else {
			dev->mcu_gpio_bank.returned_gpio_val = msg[2];
			mb();
			dev->mcu_gpio_bank.returned_flag = NO_REQUEST;	
		}

		wake_up_interruptible(&(dev->mcu_gpio_bank.mcu_wq));
	} else {
	// TODO: use macro
		for (i = 0; i < dev->gpiochip_in.ngpio; i++) {
			if (msg[2] & (1 << i)) {
				if (msg[3] & (1 << i)) {
					//				printk("%s set   INPUT%d\n", __func__, i);
					set_bit(i, &dev->gpi_values);
				} else {
					//				printk("%s: clear INPUT%d\n", __func__, i);
					clear_bit(i, &dev->gpi_values);
				}
				val |= (1 << i);
			}
		}
		if (val) {
    		pr_notice("%s: %d\n", __func__, val);
    	    g_gpio = val;
    	    gpio_in_notify(1, &g_gpio);
		}
	}

	return count;
}

/////

static int virt_gpio_out_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_out);

	return 0;
}

static int virt_gpio_mcu_request(struct gpio_chip *chip, unsigned offset)
{
	//struct virt_gpio * dev = g_pvpgio;
	//pr_notice("%s() %d\n", __func__, offset);
	//set_bit(offset&0x1F/*modulo 32*/, (unsigned long *)&dev->enabled_mcu[offset>>5/*dividing by 32*/]);

	return 0;
}

static int virt_gpio_in_request(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
//	printk("%s: %d\n", __func__, offset);
	set_bit(offset, &dev->enabled_in);

	return 0;
}

static void virt_gpio_out_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_out);
}

static void virt_gpio_in_free(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;
	pr_debug("%s() %d\n", __func__, offset);
	clear_bit(offset, &dev->enabled_in);
}

static void virt_gpio_mcu_free(struct gpio_chip *chip, unsigned offset)
{
	//struct virt_gpio * dev = g_pvpgio;
	//pr_notice("%s() %d\n", __func__, offset);
	//clear_bit(offset&0x1F/*modulo 32*/, (unsigned long *)&dev->enabled_mcu[offset>>5/*dividing by 32*/]);
}

static int virt_gpio_out_get(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int virt_gpio_fix_out_get(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void virt_gpio_mcu_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio *dev = g_pvpgio;
	int port, bit_index;

	port = (offset >> 5);		 
	bit_index = (offset & 31u); 

	DEFINE_LOCK_FLAGS(flags); // make last

//	pr_notice("%s() bit_index %u, port %u offset %u \n", __func__, bit_index, port, offset);

	LOCK_BANK(dev->mcu_gpio_bank.lock, flags);

	//only in case this gpio is output
	if (kGpioDigitalInput != TEST_BIT32(dev->mcu_gpio_bank.mcu_gpio_dir[port], bit_index))
	{
		//pr_notice("setting bits at %s() offset %d value %d port %d bit_index %d \n", __func__, offset, value,port,bit_index);
		
		if (value)
			SET_BIT32(dev->mcu_gpio_bank.mcu_gpio_value[port], bit_index);
		else
			CLEAR_BIT32(dev->mcu_gpio_bank.mcu_gpio_value[port], bit_index);
		//set the mask the bit in the mask to signal poll/read and the value
		SET_BIT32(dev->mcu_gpio_bank.mcu_gpio_mask[port], bit_index);
		//pr_notice("mcu_gpio_value %s() %d %d %d %d %d\n", __func__,dev->mcu_gpio_bank.mcu_gpio_value[0],dev->mcu_gpio_bank.mcu_gpio_value[1],dev->mcu_gpio_bank.mcu_gpio_value[2],dev->mcu_gpio_bank.mcu_gpio_value[3],dev->mcu_gpio_bank.mcu_gpio_value[4] );
	}

	UNLOCK_BANK(dev->mcu_gpio_bank.lock, flags);
}

static int virt_gpio_mcu_get(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	unsigned int port, bit_index, ret;

	port = (offset >> 5);		// dividing by 32
	bit_index = (offset & 31u); // modolu 32

	DEFINE_LOCK_FLAGS(flags); // make last

	pr_notice("%s() bit_index %u, port %u offset %u \n", __func__, bit_index, port, offset);

	LOCK_BANK(dev->mcu_gpio_bank.lock, flags);

	//only if the gpio is input
	if (kGpioDigitalInput == TEST_BIT32(dev->mcu_gpio_bank.mcu_gpio_dir[port], bit_index))
	{
		//set the mask to signal poll and read to get this value
		SET_BIT32(dev->mcu_gpio_bank.mcu_gpio_mask[port], bit_index);
		//__set_bit(bit_index, (unsigned long *)&dev->mcu_gpio_bank.mcu_gpio_mask[port]);
		UNLOCK_BANK(dev->mcu_gpio_bank.lock, flags);
		//Wait for response
		//pr_notice("%s() before wait\n", __func__);
		dev->mcu_gpio_bank.returned_flag = REQUEST_SENT;
		wait_event_interruptible_exclusive(dev->mcu_gpio_bank.mcu_wq, REQUEST_SENT != dev->mcu_gpio_bank.returned_flag);
		//pr_notice("%s() after wait %d \n", __func__, dev->mcu_gpio_bank.returned_flag);
		
		if (NO_REQUEST != dev->mcu_gpio_bank.returned_flag) {
			dev->mcu_gpio_bank.returned_flag = NO_REQUEST;
			return -EBUSY;
		}

		LOCK_BANK(dev->mcu_gpio_bank.lock, flags);
		dev->mcu_gpio_bank.returned_gpio_val ? 
		SET_BIT32(dev->mcu_gpio_bank.mcu_gpio_value[port], bit_index)
		:
		CLEAR_BIT32(dev->mcu_gpio_bank.mcu_gpio_value[port], bit_index);
	}
	ret = test_bit(bit_index, &dev->mcu_gpio_bank.mcu_gpio_value[port]);
	
	UNLOCK_BANK(dev->mcu_gpio_bank.lock, flags);

	return (ret)?1:0;
}

static int virt_gpio_in_get(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio * dev = g_pvpgio;

//    printk("%s: %lx\n", __func__, dev->gpi_values);
	return test_bit(offset, &dev->gpi_values) ? 1 : 0;
}

static void virt_gpio_out_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio * dev = g_pvpgio;
	DEFINE_LOCK_FLAGS(flags); // make last

	pr_debug("%s() offset %d value %d\n", __func__, offset, value);

	LOCK_BANK(dev->gpo_bank.lock, flags);

	__set_bit(offset, &dev->gpo_bank.gpio_mask);
	if(value)
		__set_bit(offset, &dev->gpo_bank.gpio_value);
	else
		__clear_bit(offset, &dev->gpo_bank.gpio_value);

	UNLOCK_BANK(dev->gpo_bank.lock, flags);

	wake_up_interruptible(&dev->gpo_bank.wq);
}

static void virt_gpio_out_set_fix(struct gpio_chip *chip, unsigned offset, int value){
    struct virt_gpio * dev = g_pvpgio;
    int tx_cmd = 0;
    int en_send = 0;
	DEFINE_LOCK_FLAGS(flags); // make last
    LOCK_BANK(dev->gpo_bank.lock, flags);
    //pr_notice("offset %d value %d\n", offset, value);

    switch (offset) {
    case 0:
        if (value) {
            tx_cmd = 0x21;
        }else{
            tx_cmd = 0x20;
        }
        en_send = 1;
        break;
    case 1:
        if (value) {
            tx_cmd = 0x23;
        }else{
            tx_cmd = 0x22;
        }
        en_send = 1;
        break;
    default:break;
    }
    if (en_send) {
        tx_cmd <<= 24;
        //pr_notice("offset %x\n", tx_cmd);
        hi_3w_tx_cmd(&tx_cmd, 0);
        en_send = 0;
    }
    UNLOCK_BANK(dev->gpo_bank.lock, flags);
}

static void virt_gpio_fix_out_set(struct gpio_chip *chip, unsigned offset, int value){
    struct virt_gpio * dev = g_pvpgio;

    //LOCK_BANK(dev->gpo_bank.lock, flags);
    if (!dev->mcu_gpio_bank.pntr_func_gpio_set) {
        //UNLOCK_BANK(dev->gpo_bank.lock, flags);
        return;
    }
    //UNLOCK_BANK(dev->gpo_bank.lock, flags);

    dev->mcu_gpio_bank.pntr_func_gpio_set(chip, offset, value);
}

static int __ref virt_gpio_cradle_callback(struct notifier_block *nfb, unsigned long reason, void *p)
{
    struct virt_gpio * dev = container_of(nfb, struct virt_gpio, virt_gpio_cradle_notifier);
    DEFINE_LOCK_FLAGS(flags);

    dev->cradle_attached = reason;

    pr_notice("%s %d\n", __func__, dev->cradle_attached);

    //LOCK_BANK(dev->gpo_bank.lock, flags);

    if (dev->cradle_attached & 0x20) {
        dev->mcu_gpio_bank.pntr_func_gpio_set = virt_gpio_out_set;
    } else if (dev->cradle_attached & 0x10) {
        dev->mcu_gpio_bank.pntr_func_gpio_set = virt_gpio_out_set_fix;
    } else {
        dev->mcu_gpio_bank.pntr_func_gpio_set = 0;
    }

    //UNLOCK_BANK(dev->gpo_bank.lock, flags);

    return NOTIFY_OK;
}

static int virt_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	// set direction input
	pr_debug("%s() offset %d output\n", __func__, offset);
	return 0;
}

static int virt_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	pr_debug("%s() set output and offset %d to %d\n", __func__, offset, value);
	// set direction output and set value
	virt_gpio_out_set(chip, offset, value);
	// Set as output
	return 0;
}

static int virt_gpio_mcu_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct virt_gpio *dev = g_pvpgio;
	int port, bit_index;
	port = offset / 32;		 /*power of two so the compiler will turn this to a shift*/
	bit_index = offset % 32; /*power of two so the compiler will do this with a mask*/

	CLEAR_BIT32(dev->mcu_gpio_bank.mcu_gpio_dir[port], bit_index);

	return 0;
}

static int virt_gpio_mcu_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct virt_gpio *dev = g_pvpgio;
	int port, bit_index;
	port = offset / 32;		 /*power of two so the compiler will turn this to a shift*/
	bit_index = offset % 32; /*power of two so the compiler will do this with a mask*/

	SET_BIT32(dev->mcu_gpio_bank.mcu_gpio_dir[port], bit_index);

	return 0;
}

static const struct file_operations vgpio_dev_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_gpio_chr_read,
	.write = virt_gpio_chr_write,
	.open = vgpio_dev_open,
	.release = vgpio_dev_release,
	.poll   = vgpio_dev_poll,
};

static struct miscdevice vgpio_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "vgpio",
	.fops		= &vgpio_dev_fops,
};

static int __init virtual_gpio_init(void)
{
	struct virt_gpio * dev;
	int ret;
    int fixed_mode = 0;
    struct device_node *np;
    np = of_find_compatible_node(NULL, NULL, "mcn,fixed-vinputs");
    if (np) {
        fixed_mode = 1;
        pr_err("node is finded\n");
    }

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if(!dev)
		return -ENOMEM;

	g_pvpgio = dev;

	init_waitqueue_head(&dev->gpo_bank.wq);
	init_waitqueue_head(&dev->mcu_gpio_bank.mcu_wq);

	dev->mcu_gpio_bank.returned_flag = NO_REQUEST;

#ifdef VGPIO_USE_SPINLOCK
	spin_lock_init(&dev->gpo_bank.lock);
	spin_lock_init(&dev->mcu_gpio_bank.lock);
#else
	mutex_init(&dev->gpo_bank.lock);
	mutex_init(&dev->mcu_gpio_bank.lock);
#endif

	ret = misc_register(&vgpio_dev);
	if(ret) {
		pr_err("%s() Unable to register misc device \n", __func__);
		return ret;
	}

	dev->gpiochip_out.label = "vgpio_out"; //virtgpio
	dev->gpiochip_out.request = virt_gpio_out_request;
	dev->gpiochip_out.free = virt_gpio_out_free;
	dev->gpiochip_out.direction_output = virt_gpio_direction_output;
    if (!fixed_mode) {
        dev->gpiochip_out.set = virt_gpio_out_set;
        dev->gpiochip_out.get = virt_gpio_out_get;
    }else{
        dev->gpiochip_out.set = virt_gpio_fix_out_set;
        dev->gpiochip_out.get = virt_gpio_fix_out_get;
        dev->virt_gpio_cradle_notifier.notifier_call = virt_gpio_cradle_callback;
        cradle_register_notifier(&dev->virt_gpio_cradle_notifier);
    }
	dev->gpiochip_out.base = -1;
	dev->gpiochip_out.ngpio = 8;
#ifdef VGPIO_USE_SPINLOCK
	dev->gpiochip_out.can_sleep = 0;
#else
	dev->gpiochip_out.can_sleep = 1;
#endif

	dev->gpiochip_in.label = "vgpio_in";
	dev->gpiochip_in.request = virt_gpio_in_request;
	dev->gpiochip_in.free = virt_gpio_in_free;
	dev->gpiochip_in.direction_input = virt_gpio_direction_input;
	dev->gpiochip_in.get = virt_gpio_in_get;
	dev->gpiochip_in.base = -1;
	dev->gpiochip_in.ngpio = 8;
	dev->gpiochip_in.can_sleep = 0;

	dev->gpiochip_mcu.label = "vgpio_mcu";
	dev->gpiochip_mcu.request = virt_gpio_mcu_request;
	dev->gpiochip_mcu.free = virt_gpio_mcu_free;
	dev->gpiochip_mcu.direction_input = virt_gpio_mcu_direction_input;
	dev->gpiochip_mcu.direction_output = virt_gpio_mcu_direction_output;
	dev->gpiochip_mcu.get = virt_gpio_mcu_get; //returns data in the
	dev->gpiochip_mcu.set = virt_gpio_mcu_set;
	dev->gpiochip_mcu.base = -1;
	dev->gpiochip_mcu.ngpio = 160;
	dev->gpiochip_mcu.can_sleep = 1;

	gpiochip_add(&dev->gpiochip_out);
	pr_debug("out base %d\n", dev->gpiochip_out.base);

	gpiochip_add(&dev->gpiochip_in);
	pr_debug("in base %d\n", dev->gpiochip_in.base);

	gpiochip_add(&dev->gpiochip_mcu);
	pr_notice("in base %d\n", dev->gpiochip_mcu.base);

    gpio_in_notify(0, 0);
//
	ret = sysfs_create_group(&vgpio_dev.this_device->kobj, &in_attr_group);
	if (ret) {
		pr_err("%s: could not create sysfs group [%d]\n", __func__, ret);
	}
//
	return 0;
}

static void __exit virtual_gpio_exit(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	int ret;
#endif
	struct virt_gpio * dev;
	dev = g_pvpgio;

	if(dev->enabled_out) {
		int i;
		pr_info("some vgpio_in gpios are still enabled.\n");
		for(i = 0; i < dev->gpiochip_out.ngpio; i++) {
			if( (dev->enabled_out & (1<<i)) ) {
				pr_info("free vgpio_in %d.\n", i);
				gpio_free(dev->gpiochip_out.base + i);
			}
		}
	}

	if(dev->enabled_in) {
		int i;
		pr_info("some vgpio_out gpios are still enabled.\n");
		for(i = 0; i < dev->gpiochip_in.ngpio; i++) {
			if( (dev->enabled_in & (1<<i)) ) {
				pr_info("free vgpio_out %d.\n", i);
				gpio_free(dev->gpiochip_in.base + i);
			}
		}
	}

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	ret =
#endif
	gpiochip_remove(&dev->gpiochip_in);

	// gpiochip_remove returns void in 3.18
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
	ret =
#endif
	gpiochip_remove(&dev->gpiochip_out);

	misc_deregister(&vgpio_dev);

	kfree(dev);
}


module_init(virtual_gpio_init);
module_exit(virtual_gpio_exit);
