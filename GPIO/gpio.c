#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include<linux/delay.h>
// Define the GPIO pin number to which the LED is connected
#define LED_GPIO_PIN 12

// Declare variables for the character device
static dev_t dev_num;// for Getting major and minior number( vendor id) 
static struct cdev led_cdev;


static ssize_t led_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos);
static ssize_t led_read(struct file *filp, char __user *buf, size_t count, loff_t *pos);
static int __init led_init(void);
//static int __init led_exit(void);
static int led_open(struct inode *inode, struct file *filp);
static int led_release(struct inode *inode, struct file *filp);
static struct class *cl; // Global variable for the device class



static const struct file_operations led_fops = {
    .owner = THIS_MODULE,
    .read = led_read, 
    .write = led_write,
    .open = led_open,
    .release = led_release,
};



// Function to handle device file open events
static int led_open(struct inode *inode, struct file *filp)
{
    return 0;
}

// Function to handle device file close events
static int led_release(struct inode *inode, struct file *filp)
{
    return 0;
}


// Function to turn the LED on or off
static ssize_t led_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{

    int i;

    for(i=0;i<10;i++)
    {

        gpio_set_value(LED_GPIO_PIN, 0);
        msleep(2000);
        gpio_set_value(LED_GPIO_PIN, 1);
        msleep(2000);
    }
    return count;
}

static ssize_t led_read(struct file *filp,  char __user *buf, size_t count, loff_t *pos)
{
   int ret_read;
   ret_read=gpio_get_value(LED_GPIO_PIN);
    printk(KERN_INFO "Driver: read()%d\n",ret_read);
    return 0;

}

// Function to initialize the device driver
static int __init led_init(void)
{
    int err;
    struct device *dev_ret; //checking device is created properly or not 
    // Allocate a character device number
    err = alloc_chrdev_region(&dev_num, 0, 1, "led");
    if (err < 0) {
        printk(KERN_ERR "led: failed to allocate device number\n");
        return err;
    }

    // Initialize the character device
    cdev_init(&led_cdev, &led_fops);
    led_cdev.owner = THIS_MODULE;

    // Add the character device to the kernel
    err = cdev_add(&led_cdev, dev_num, 1);
    if (err < 0) {
        printk(KERN_ERR "led: failed to add character device\n");
        unregister_chrdev_region(dev_num, 1);
        return err;
    }

    // Character Device Files - Creation & Operations

    cl = class_create(THIS_MODULE, "LED_MODULE");//This one is create a node in dev file
    if (IS_ERR(dev_ret = device_create(cl, NULL, dev_num, NULL, "GPIO_LED"))) // inside LED_MODULE node GPIO_LED module is created 
    {
        class_destroy(cl);
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(dev_ret);
    }


    // Request the GPIO pin
    err = gpio_request(LED_GPIO_PIN, "led");
    if (err < 0) {
        printk(KERN_ERR "led: failed to request GPIO pin\n");
        cdev_del(&led_cdev);
        unregister_chrdev_region(dev_num, 1);
        return err;
    }

    // Set the direction of the GPIO pin to output
    gpio_direction_output(LED_GPIO_PIN, 0);

    printk(KERN_INFO "led: device driver initialized\n");
    return 0;
}

// Function to clean up the device driver
static void __exit led_exit(void)
{
    // Set the value of the GPIO pin to off
    gpio_set_value(LED_GPIO_PIN, 0);

    // Free the GPIO pin
    gpio_free(LED_GPIO_PIN);
    //device destroy 
    device_destroy(cl, dev_num); //Module destroy first
    class_destroy(cl);// Node is Destroy then
    // Remove the character device from the kernel
    cdev_del(&led_cdev);

    // Release the character device number
    unregister_chrdev_region(dev_num, 1);

    printk(KERN_INFO "led: device driver cleaned up\n");
}

// Register the device driver initialization and cleanup functions
module_init(led_init);
module_exit(led_exit);

// Declare the module license and description
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPIO LED character device driver");

