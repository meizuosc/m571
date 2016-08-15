#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

#include <linux/delay.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#define MAX77821_I2C_BUSNUM  3//for I2C channel 0
#define I2C_ID_NAME "max77821"
#define MAX77821_ADDR 0x48
#ifndef LCD_BACKLIGHT_FULL
 #define LCD_BACKLIGHT_FULL 255//2047
#endif

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata max77821_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, MAX77821_ADDR)};
static struct i2c_client *max77821_i2c_client = NULL;
/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int max77821_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int max77821_remove(struct i2c_client *client);
static int max77821_write_bytes(unsigned char addr, unsigned char value);

/***************************************************************************** 
 * Data Structure
 *****************************************************************************/
 struct max77821_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id max77821_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver max77821_iic_driver = {
	.id_table	= max77821_id,
	.probe		= max77821_probe,
	.remove		= max77821_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "max77821",
	},
 
};

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int max77821_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "max77821_iic_probe\n");
	printk("max77821: info==>name=%s addr=0x%x\n",client->name,client->addr);
	max77821_i2c_client  = client;	
	return 0;      
}

static int max77821_remove(struct i2c_client *client)
{  	
  printk( "max77821_remove\n");
  max77821_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}

 static int max77821_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = max77821_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	{
	printk(KERN_EMERG"max77821 write data fail !!\n");	
}
else
{}
	//printk(KERN_EMERG"max77821 write data 0x%X=>0x%X !!\n",addr,value);
	return ret ;
}

static int __init max77821_iic_init(void)
{
   printk( "max77821_iic_init\n");
   if(i2c_register_board_info(MAX77821_I2C_BUSNUM, &max77821_board_info, 1))
   	{
   printk( "max77821_iic_init i2c_register_board_info fail\n");
   	}
   else
   	{
   	printk( "max77821_iic_init i2c_register_board_info success\n");
   	}
   if(i2c_add_driver(&max77821_iic_driver))
   	{
	   printk( "max77821_iic_init i2c_add_driver fail\n");
   	}
   else
   	{
   printk( "max77821_iic_init i2c_add_driver success\n");
   	}
   return 0;
}

static void __exit max77821_iic_exit(void)
{
  printk( "max77821_iic_exit\n");
  i2c_del_driver(&max77821_iic_driver);  
}

module_init(max77821_iic_init);
module_exit(max77821_iic_exit);

MODULE_AUTHOR("BSP MEIZU");
MODULE_DESCRIPTION("MTK max77821 I2C Driver");
MODULE_LICENSE("GPL"); 


static unsigned int last_level=0x0;
unsigned int Cust_SetBacklight(unsigned int level)
{
	unsigned char lm_lsb=0;
	unsigned char lm_msb=0;


	if(level>LCD_BACKLIGHT_FULL)
	{
		level=LCD_BACKLIGHT_FULL;
	}
	if (last_level==0)
	{
		max77821_write_bytes(0x98,0xF4);
	}
	if (level == 0)
	{
		max77821_write_bytes(0x98,0x04);
	}
	max77821_write_bytes(0x99, level);
	if((last_level==0)||(level==0))
	{
	printk(KERN_EMERG"Cust_SetBacklight level=%d lsb=%X msb=%X\n",level,lm_lsb,lm_msb);
	}
	last_level=level;
}

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	#ifndef MEIZU_M81 //2015-04-24 by Yin ShunQing, in order to enable button LED
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
	#else
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	#endif
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (long)disp_bls_set_backlight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

