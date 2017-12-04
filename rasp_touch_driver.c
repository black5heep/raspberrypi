#include <linux/miscdevice.h>  
#include <linux/delay.h>  
#include <asm/irq.h>  
#include <linux/kernel.h>  
#include <linux/module.h>  
#include <linux/init.h>  

#include <linux/types.h>  
#include <linux/delay.h>  
#include <linux/moduleparam.h>  
#include <linux/slab.h>  
#include <linux/errno.h>  
#include <linux/ioport.h>
#include <linux/gpio.h>  
#define GPIO_Function 0x3f200000
#define GPIO_Pin_Output 0x3f20001c
  
//#include "bcm2835.h"  
  
// Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)  
#define PIN RPI_GPIO_P1_11  
#define BCM2835_GPIO_BASE 0x3f200000
int open_state = 0;         //文件打开状态  
  


volatile unsigned long *gpio_dir = NULL;
volatile unsigned long *gpio_out = NULL;
  
int major;
static struct class *led_class;
static struct class_device *led_class_dev;
static int __init leds_init(void)  
{  
	int err;
	struct resource *p;
    
    gpio_dir = (volatile unsigned long *)ioremap(GPIO_Function, 16);
	if(gpio_dir==NULL)  
	{
		printk("dir:ioremap err\r\n");
		return -1;
	}
    gpio_out = (volatile unsigned long *)ioremap(GPIO_Pin_Output, 16);
if(gpio_out==NULL) 
	{
		printk("out:ioremap err\r\n");
return -1;
	}
     printk("dir:%d\r\nout:%d\r\n",gpio_dir,gpio_out);
	*gpio_dir &= ~((0x7<<(2*3))|(0x7<<(3*3)));
	*gpio_dir |= ((0x1<<(2*3))|(0x1<<(3*3)));


    *gpio_out |= (1<<3);
    printk("ledsinit.\n");  
    return 0;  
}  
  
static void leds_exit(void)  
{  
	//gpio_free(3); 
	iounmap(gpio_dir);
	iounmap(gpio_out);
	release_mem_region(GPIO_Function,16);
     release_mem_region(GPIO_Pin_Output,16);
    printk("leds_exit\n");  
}  
  
module_init(leds_init);  
module_exit(leds_exit);  
  
MODULE_AUTHOR("Hu Chunxu");  
MODULE_LICENSE("GPL");  