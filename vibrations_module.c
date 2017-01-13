#include "rpiGPIOManipulation.h"
#include "vibrations_module.h"

/* Global variables of the module*/
static Device* devices;
static struct class *cl;
static int major_number;
static int devices_to_destroy;

/* Timer variables */
static struct hrtimer blink_timer;
static ktime_t kt;
static long long current_time = 0;

#define TIMER_SEC    0
#define TIMER_NANO_SEC  1*1000*1000 /* 250ms */

/* Default GPIO pins if no arguments passed */
static int do_GPIO_pins[4] = {GPIO_02, GPIO_03, GPIO_18, GPIO_22};
static int argc =  0;

/*
* module_param_array(variable_name, data_type, num, perm);
* The first param is the parameter's (in this case the array's) name.
* The second param is the data type of the elements of the array.
* The third argument is a pointer to the variable that will store the number
* of elements of the array initialized by the user at module loading time.
* The fourth argument is the permission bits.
*/
module_param_array(do_GPIO_pins, int, &argc, 0000);
MODULE_PARM_DESC(do_GPIO_pins, "Array of GPIO pins were DO pin of SW-420 vibration sensor is connected.");

enum hrtimer_restart blink_timer_callback(struct hrtimer *param)
{
	current_time++;

    hrtimer_forward(&blink_timer, ktime_get(), kt);
    
    return HRTIMER_RESTART;
}


static irqreturn_t h_irq_gpio(int irq, void *data)
{
//  printk("Interrupt from IRQ 0x%x\n", irq);    
	int end_time;
	Device* dev;
	
	dev = (Device*)(data);

	/* Period is current time - prevoius time that interrupt occured */	
	end_time = current_time;
	dev->period = end_time - dev->timestamp;

#ifdef DEBUG	
	printk(KERN_INFO "PERIOD: %d\n", dev->period);
#endif
 
	dev->timestamp = end_time;
        
    return IRQ_HANDLED;
}

ssize_t vibration_driver_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	/* Driver has no functionality for writing */

	return 0;
}

ssize_t vibration_driver_read(struct file *filp, char __user *buf, size_t count, 
	loff_t *f_pos)
{
	Device* dev = (Device*)filp->private_data;
	ssize_t retval = 0;
	//int cnt = 0;
	//int temp;
	
	/*
	if (mutex_lock_killable(&dev->dev_mutex))
		return -EINTR;
	
	if (*f_pos >= BUFF_LEN) //EOF
		goto out;
	
	if (*f_pos + count > BUFF_LEN)
		count = BUFF_LEN - *f_pos;
	
	if (count > BLOCK_LEN)
		count = BLOCK_LEN;
	*/

	//printk(KERN_INFO "period: %d\n", dev->period);
	if (copy_to_user(buf, &dev->period, 4) != 0)
	{
		retval = -EFAULT;
		goto out;
	}
	
	*f_pos += count;
	retval = count;
	
out:
	mutex_unlock(&dev->dev_mutex);
	return retval;
}

int vibration_driver_release(struct inode* inode, struct file* filp) {

	return 0;
}

int vibration_driver_open(struct inode* inode, struct file* filp)
{
	int mjn;
	int mnn;
	Device *dev;

	mjn = imajor(inode);
	mnn = iminor(inode);

	/* <major, minor> validation */
	if(mjn != major_number || mnn < 0 || mnn > MAXDEVICES) {
		printk(KERN_WARNING "No device found with minor=%d and major=%d\n", mjn, mnn);
		return -ENODEV; /* No such device */
	}

	dev = &devices[mnn];
	filp->private_data = dev; 	//Saving device for other functions

	if (inode->i_cdev != &(dev->c_dev))
	{
		return -ENODEV; /* No such device */
	}

	dev->period = 0;
	dev->timestamp = 0;

	/* Memory allocation for data if first time opened */
	if (dev->data == NULL)
	{
		dev->data = (unsigned char*)kmalloc(BUFF_LEN, GFP_KERNEL);
		if (dev->data == NULL)
		{
			printk(KERN_WARNING "open(): out of memory\n");
			return -ENOMEM;
		}
	}

	return 0;
}

int construct_device(Device* dev, int minor, struct class *class)
{
	int err;
	dev_t devno;
	struct device* device = NULL;

	err = 0;
	devno = MKDEV(major_number, minor);

	/* Fields initialization */
	dev->data = NULL;		// Memory is allocated when device is first opened
	mutex_init(&(dev->dev_mutex));
	
	/* Device initialization and device adding */
	cdev_init(&(dev->c_dev), &driver_fops);
	dev->c_dev.owner = THIS_MODULE;

	err = cdev_add(&(dev->c_dev), devno, 1);
	if( err ) {
		printk(KERN_ALERT "Error %d while trying to add %s%d",
			err, DEVICE_NAME, minor);
		return err;
	}

	/* Creating device */
	device = device_create(class, NULL, devno, NULL, DEVICE_NAME "%d", minor);
	if(IS_ERR(device)) {
		err = PTR_ERR(device);
		printk(KERN_WARNING "Error %d while trying to create %s%d", err, DEVICE_NAME, minor);
		cdev_del(&(dev->c_dev));
	}
#ifdef DEBUG
	printk(KERN_INFO "Device files created\n");
#endif

	return 0;
}

int vibration_driver_init(void) 
{
	int i;
	int err;
	dev_t t_dev;

	devices_to_destroy = 0;
	err = 0;

	/* Asking to register driver at some available major device number
	 * starting with minor device number one, 
	 * and asking for a MAXDEVICES number of minor numbers.  
	 */
	err = alloc_chrdev_region(&t_dev, 0, MAXDEVICES, DEVICE_NAME);
	if(err) {
		printk( KERN_ALERT "Device Registration failed\n" );
        return -1;
	}

	/* Extracting major number for devices creation */
	major_number = MAJOR(t_dev);

#ifdef DEBUG
	printk( KERN_INFO "Device Registration successful with major number %d\n", major_number);
#endif

	cl = class_create(THIS_MODULE, DEVICE_NAME);
	if( IS_ERR(cl) ) {
		err = PTR_ERR(cl);
        printk( KERN_ALERT "Class creation failed\n" );
        goto error;
    }
	
	devices = (Device*)kmalloc(MAXDEVICES * sizeof(Device), GFP_KERNEL);
	if( devices == NULL) {
		printk( KERN_ALERT "Memory allocation for devices failed\n" );
		goto error;	
	}
	
	/* Creating devices one by one */
	for(i = 0; i < MAXDEVICES; i++) {
		err = construct_device(&devices[i], i, cl);
		if(err) {
			printk(KERN_ALERT "Creation of devices failed\n" );
			devices_to_destroy = i;
			goto error;
		}	
	}

	/* Initialize high resolution timer. */
    hrtimer_init(&blink_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    kt = ktime_set(TIMER_SEC, TIMER_NANO_SEC);    
    blink_timer.function = &blink_timer_callback;
    hrtimer_start(&blink_timer, kt, HRTIMER_MODE_REL); 

#ifdef DEBUG
	printk(KERN_INFO "Timer is set\n");
#endif   

	for(i = 0; i < MAXDEVICES; i++) {
		/* Setting pin direction to INPUT */
		SetInternalPullUpDown(do_GPIO_pins[i], PULL_UP);
		SetGpioPinDirection(do_GPIO_pins[i], GPIO_DIRECTION_IN);

		/* Enabling interrupts */
		gpio_request_one(do_GPIO_pins[i], GPIOF_IN, "irq_gpio");
		devices[i].irq_gpio = gpio_to_irq(do_GPIO_pins[i]);    
		err = request_irq(devices[i].irq_gpio, h_irq_gpio, IRQF_TRIGGER_FALLING, "irq_gpio", (void *)&devices[i]);	//devices[i] as data parametar for interrupt 
    	if( err ) {
       		printk("Error: ISR %d not registered!\n", do_GPIO_pins[i]);
   		}        
	}

#ifdef DEBUG
	printk(KERN_INFO "GPIO pins set successfully\n");
#endif   

	return 0;	//Success

error:
	
	vibration_driver_exit();
	return 1;
}

void vibration_driver_exit(void) 
{
	int i;
	int numOfIterations;

	/* Clear GPIO pins. */
	for(i = 0; i < MAXDEVICES; i++) {
		disable_irq(devices[i].irq_gpio);
    	free_irq(devices[i].irq_gpio, (void *)&devices[i]);
    	gpio_free(do_GPIO_pins[i]);

    	SetInternalPullUpDown(do_GPIO_pins[i], PULL_NONE);
	}

	/* Timer destroy */
	hrtimer_cancel(&blink_timer);    

	if (devices) {
		numOfIterations = devices_to_destroy ? devices_to_destroy : MAXDEVICES;
 
		printk(KERN_INFO "Devices to des: %d, num of iter: %d", devices_to_destroy, numOfIterations);
		for (i = 0; i < numOfIterations; i++) {
			device_destroy(cl, MKDEV(major_number, i));
			cdev_del(&(devices[i].c_dev));
			kfree(devices[i].data);
			mutex_destroy(&(devices[i].dev_mutex));
		}
	
		kfree(devices);
	}
#ifdef DEBUG
	printk(KERN_INFO "Devices are destroyed\n");
#endif

	if(cl)
		class_destroy(cl);

	/* vibration_driver_exit() is never called if registration fails */
	unregister_chrdev_region(MKDEV(major_number, 0), MAXDEVICES);
	
    printk(KERN_ALERT "Devices unregistered\n");
	return;
}

module_init(vibration_driver_init);
module_exit(vibration_driver_exit);

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR(DRIVER_AUTHOR);       /* Who wrote this module? */

MODULE_DESCRIPTION(DRIVER_DESC);    /* What does this module do */

/*
* The MODULE_SUPPORTED_DEVICE macro might be used in the future to help
* automatic configuration of modules, but is currently unused other than
* for documentation purposes.

MODULE_SUPPORTED_DEVICE("Supported device: devA, devB, etc.");
*/
