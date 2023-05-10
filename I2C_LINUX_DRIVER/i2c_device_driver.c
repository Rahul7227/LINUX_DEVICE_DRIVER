/*I2C DEVICE DRIVER*/

#include<linux/module.h>
#include<linux/init.h>
#include<linux/fs.h>
#include<linux/device.h>
#include<linux/i2c.h>
#include<linux/delay.h>
#include<linux/uaccess.h>
#include<linux/slab.h>
#include<linux/kernel.h>

#define LCD_SLAVE_ADDR 0x27
#define I2C_BUS_AVAILABLE 2
#define SLAVE_DEVICE_NAME "LCD_DISPLAY"


/* LCD Commands */

//Clears the display and sets the cursor to the home position
#define LCD_CMD_CLEAR 0x01

//Sets the cursor to the home position
#define LCD_CMD_HOME 0x02

//Sets the text entry mode, which determines how the cursor behaves when text is entered.
#define LCD_CMD_MODE 0x06

#define LCD_CMD_DISPLAY_OFF 0x08

#define LCD_CMD_DISPLAY_ON 0x0C

//Sets various display options, such as the number of lines and character size.
#define LCD_CMD_FUNCTION_SET 0x20

//Sets the address of the character generator RAM (CGRAM), which is used to define custom characters.
#define LCD_CMD_SET_CGRAM_ADDR 0x40

//Sets the address of the display data RAM (DDRAM), which is where characters are written to be displayed on the screen.
#define LCD_CMD_SET_DDRAM_ADDR 0x80


/* LCD Pins */

#define LCD_PIN_RS 0 //The pin used to control the register select (RS) signal on the LCD display. This signal determines whether data sent to the display is interpreted as a command or as text data.

#define LCD_PIN_RW 1 //The pin used to control the read/write (R/W) signal on the LCD display. This signal determines whether data is being read from the display (R/W = 1) or written to the display (R/W = 0).

#define LCD_PIN_EN 2 //The pin used to control the enable (EN) signal on the LCD display. This signal triggers the LCD to latch the data on its input pins when transitioning from low to high.

#define LCD_PIN_BACKLIGHT 3 //The pin used to control the backlight on the LCD display. This pin is typically used to turn the backlight on or off, or to adjust its brightness.
 
//The pins(D4-D7) used to transmit the 4-bit or 8-bit data bus between the controller and the LCD display. These pins carry the data that is being written to or read from the display, one nibble (4 bits) at a time.

#define LCD_PIN_D4 4
#define LCD_PIN_D5 5
#define LCD_PIN_D6 6
#define LCD_PIN_D7 7


/*structure declaration*/
static struct i2c_adapter *lcd_i2c_adapter=NULL;
static struct i2c_client *lcd_i2c_client=NULL;


static dev_t dev=0;
static struct class *dev_class;

static int lcd_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lcd_i2c_driver_remove(struct i2c_client *client);


//LCD_WRITE FUNCTION//

// Function to send commands to the LCD
static void lcd_send_command(struct i2c_client *client, u8 command)
{
    u8 data[4];

    data[0] = (command & 0xF0) | (1 << LCD_PIN_EN) | (1 << LCD_PIN_BACKLIGHT);
    data[1] = (command & 0xF0) | (1 << LCD_PIN_BACKLIGHT);
    data[2] = ((command & 0x0F) << 4) | (1 << LCD_PIN_EN) | (1 << LCD_PIN_BACKLIGHT);
    data[3] = ((command & 0x0F) << 4) | (1 << LCD_PIN_BACKLIGHT);

    i2c_master_send(client, data, 4);
    msleep(5);
}
// Function to send data to the LCD
static void lcd_send_data(struct i2c_client *client, u8 data)
{
    u8 data_[4];

    data_[0] = (data & 0xF0) | (1 << LCD_PIN_EN) | (1 << LCD_PIN_RS) | (1 << LCD_PIN_BACKLIGHT);
    data_[1] = (data & 0xF0) | (1 << LCD_PIN_RS) | (1 << LCD_PIN_BACKLIGHT);
    data_[2] = ((data & 0x0F) << 4) | (1 << LCD_PIN_EN) | (1 << LCD_PIN_RS) | (1 << LCD_PIN_BACKLIGHT);
    data_[3] = ((data & 0x0F) << 4) | (1 << LCD_PIN_RS) | (1 << LCD_PIN_BACKLIGHT);

    i2c_master_send(client, data_, 4);
    msleep(5);
}
    
static void lcd_init(struct i2c_client *client)
{
    lcd_send_command(client, LCD_CMD_FUNCTION_SET | (1 << 5));
    msleep(1000);
   lcd_send_command(client, LCD_CMD_FUNCTION_SET | (1 << 5));
   msleep(1000);
    lcd_send_command(client, LCD_CMD_FUNCTION_SET | (1 << 5));
    msleep(1000);
    lcd_send_command(client, LCD_CMD_FUNCTION_SET | (1 << 5) | (1 << 4));
    msleep(1000);
    lcd_send_command(client, LCD_CMD_MODE);
    msleep(1000);
    lcd_send_command(client, LCD_CMD_DISPLAY_ON | (1 << 2));
    msleep(1000);
    lcd_send_command(client, LCD_CMD_CLEAR);
    msleep(1000);
}

// Function to display a string on the LCD 
 static void lcd_display_string(struct i2c_client *client, const char *str)
{
       	while (*str)
	       	lcd_send_data(client, *str++);
} 



/*structure that has the slave device id*/

static const struct i2c_device_id lcd_i2c_driver_id[] ={
	{ SLAVE_DEVICE_NAME,0},
	{}
};

MODULE_DEVICE_TABLE(i2c ,lcd_i2c_driver_id);

/*I2C driver Structure that has to be added to linux*/

static struct i2c_driver lcd_i2c_driver = {
	.driver={
		.name = SLAVE_DEVICE_NAME,
		.owner= THIS_MODULE,
	},
	.probe    = lcd_i2c_driver_probe,
	.remove   = lcd_i2c_driver_remove,
	.id_table = lcd_i2c_driver_id,
};



/*I2c board info structre*/

static struct i2c_board_info lcd_i2c_board_info =
{
	I2C_BOARD_INFO(SLAVE_DEVICE_NAME ,LCD_SLAVE_ADDR)
}; 

/*This function getting called when the slave has been found
  This will be called only once when we load the driver.*/

static int lcd_i2c_driver_probe(struct i2c_client *client ,const struct i2c_device_id *id)
{
	//int len;
	//char str[]="HELLOWORLD";
	printk(KERN_INFO "The probe-invoked sucessfully \n");

	lcd_init(client);

	printk(KERN_INFO"LCD INIT IS DONE SUCCESFULLY\n");

	lcd_display_string(client,"INNOMINDS");

	return 0;

}


/*This function getting called when the slave has been removed 
  This will be called only once when we unload the driver.
  */

static int lcd_i2c_driver_remove(struct i2c_client *client)
{
	printk(KERN_INFO"The probe is removed sucessfully");
	return 0;
}


/*initialization*/

static int __init i2c_driver_init(void)
{
	int ret =-1;

	/*Allocating major number*/

	if((alloc_chrdev_region(&dev,0,1,"i2c_client_driver"))<0)

	{
		printk(KERN_INFO "cannot allocate the major number..\n");
		return -1;
	}
	printk(KERN_INFO"major =%d minor =%d..\n",MAJOR(dev),MINOR(dev));

	/*creating struct class*/
	if((dev_class = class_create(THIS_MODULE,"i2c_client_class"))==NULL)
	{
		printk(KERN_INFO "cannot create the struct class..\n");

	}
	/*creating device */

	if((device_create(dev_class,NULL,dev,NULL,"i2c_client_dev")) ==NULL)
	{
		printk(KERN_INFO "cannot create the device..\n");
	}
	/*i2c driver add*/
	lcd_i2c_adapter = i2c_get_adapter(I2C_BUS_AVAILABLE);

	if(lcd_i2c_adapter != NULL)

	{
		lcd_i2c_client=i2c_new_client_device(lcd_i2c_adapter,&lcd_i2c_board_info);

		if(lcd_i2c_client!=NULL)
		{
			/*the function i2c_add_driver should be called with a pointer to the struct i2c_driver*/
			i2c_add_driver(&lcd_i2c_driver);

			ret =0;

		}

		i2c_put_adapter(lcd_i2c_adapter);
	}
	printk(KERN_INFO"Driver Added\n");

	return ret;

}



static void __exit i2c_driver_exit(void)
{
	i2c_del_driver(&lcd_i2c_driver);
	i2c_unregister_device(lcd_i2c_client);
	device_destroy(dev_class,dev);
	class_destroy(dev_class);
	unregister_chrdev_region(dev,1);
	printk(KERN_INFO "Driver removed\n");
}


module_init(i2c_driver_init);
module_exit(i2c_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("i2c client driver");
MODULE_AUTHOR("Rahul");
