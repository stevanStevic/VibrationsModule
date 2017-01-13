#ifndef _VIBRATIONS_MODULE_H_INCLUDED_
#define _VIBRATIONS_MODULE_H_INCLUDED_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>	
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ioport.h>

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/uaccess.h>

/* Allowing to print kernel messages */
#define DEBUG

/* Documentation info */
#define DRIVER_AUTHOR "Stevan Stevic"
#define DRIVER_DESC "Simple driver that handles up to 4 vibration sensor (SW-420) or similar"


#define MAXDEVICES 4

#define DEVICE_NAME "vibrato"

/* Declaration of driver functions */
int vibration_driver_init(void);
void vibration_driver_exit(void);
static int vibration_driver_open(struct inode*, struct file*);
static int vibration_driver_release(struct inode*, struct file*);
static ssize_t vibration_driver_read(struct file*, char* buf, size_t , loff_t*);
static ssize_t vibration_driver_write(struct file*, const char* buf, size_t , loff_t*);

/* Structure that declares the usual file access functions. */
struct file_operations driver_fops =
{
    open    :   vibration_driver_open,
    release :   vibration_driver_release,
    read    :   vibration_driver_read,
    write   :   vibration_driver_write
};

typedef struct device_st {
	unsigned char *data;
	struct mutex dev_mutex; 
	struct cdev c_dev;
} Device;


static irqreturn_t h_irq_gpio3(int irq, void *data)
{
    static char value = -1;

    printk("Interrupt from IRQ 0x%x\n", irq);    
    
    value = GetGpioPinValue(GPIO_03);    
    
    printk("GPIO_03 level = 0x%x\n", value);    
        
    return IRQ_HANDLED;
}


#endif