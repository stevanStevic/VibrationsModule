#ifndef _VIBRATIONS_MODULE_H_INCLUDED_
#define _VIBRATIONS_MODULE_H_INCLUDED_

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>	
#include <linux/cdev.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>

#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/io.h>

/* To allow kernel messages printing uncomment this */
//#define DEBUG

/* Documentation info */
#define DRIVER_AUTHOR "Stevan Stevic"
#define DRIVER_DESC "Simple driver that handles up to 4 vibration sensor (SW-420) or similar"

#define MAX_DEVICES 4
#define DEVICE_NAME "vibrato"

#define BUFF_LEN 1024
#define BLOCK_LEN 512

/* mode. 
 * 0 is for setting frequency and sampling.
 * 1 is for returning period 
 * Used as index od data buff when sampling signal
 */
typedef struct device_st {
	unsigned char *data;
	unsigned int sample;
	unsigned int period;
	unsigned int timestamp;
	unsigned int frequency;
	struct hrtimer sampling_timer;	
	ktime_t sampling_kt;	
	struct mutex dev_mutex;
	int irq_gpio;
	int mode;
	struct cdev c_dev;
} Device;

/* Declaration of default driver functions */
int vibration_driver_init(void);
void vibration_driver_exit(void);
static int vibration_driver_open(struct inode*, struct file*);
static int vibration_driver_release(struct inode*, struct file*);
static ssize_t vibration_driver_read(struct file*, char* buf, size_t , loff_t*);
static ssize_t vibration_driver_write(struct file*, const char* buf, size_t , loff_t*);
static long vibration_driver_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

/* Declartaion of internal driver functions */
static int construct_device(Device* dev, int minor, struct class *class);
static enum hrtimer_restart sampling_timer_callback(struct hrtimer *param);
static enum hrtimer_restart period_timer_callback(struct hrtimer *param);
static irqreturn_t h_irq_gpio(int irq, void *data);

/* Structure that declares the usual file access functions. */
struct file_operations driver_fops =
{
    open			:   vibration_driver_open,
    release 		:   vibration_driver_release,
    read 			:   vibration_driver_read,
    write   		:   vibration_driver_write,
	unlocked_ioctl	:	vibration_driver_ioctl
};

#endif
