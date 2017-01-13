#include "rpiGPIOManipulation.h"
#include "vibrations_module.h"

/* Global variables of the module*/
static Device* devices;
static struct class *cl;
static int major_number;
static int devices_to_destroy;

/* IRQ number. */
static int irq_gpio3 = -1;

static int construct_device(Device* dev, int minor, struct class *class)
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
	
	/* creating devices one by one */
	for(i = 0; i < MAXDEVICES; i++) {
		err = construct_device(&devices[i], i, cl);
		if(err) {
			printk(KERN_ALERT "Creation of devices failed\n" );
			devices_to_destroy = i;
			goto error;
		}	
	}
	return 0;	//Success

error:
	
	vibration_driver_exit();
	return 1;
}

void vibration_driver_exit(void) 
{
	int i;

	if (devices) {
		for (i = 0; i < devices_to_destroy; i++) {
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

	/* vibration_driver_exit() is never called if registration failed */
	unregister_chrdev_region(MKDEV(major_number, 0), MAXDEVICES);
	
    printk(KERN_ALERT "Devices unregistered\n");
}

static int vibration_driver_open(struct inode *inode, struct file *filp) {return 0;}
static int vibration_driver_release(struct inode *inode, struct file *filp) {return 0;}
static ssize_t vibration_driver_read(struct file *filp, char *buf, size_t len, loff_t *f_pos) {return 0;}
static ssize_t vibration_driver_write(struct file *filp, const char *buf, size_t len, loff_t *f_pos) {return 0;}

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
