/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "BU64245.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define LENS_I2C_BUSNUM 0
static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("BU64245", 0x0c)};
#define PLATFORM_DRIVER_NAME "lens_actuator_bu64245"
#define AF_DRIVER_CLASS_NAME "actuatordrv_bu64245"

#define BU64245_DRVNAME "BU64245"
#define BU64245_VCM_WRITE_ID           0x18

#define BU64245_DEBUG
#ifdef BU64245_DEBUG
#define BU64245DB printk
#else
#define BU64245DB(x,...)
#endif

static spinlock_t g_BU64245_SpinLock;

static struct i2c_client * g_pstBU64245_I2Cclient = NULL;

static dev_t g_BU64245_devno;
static struct cdev * g_pBU64245_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4BU64245_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4BU64245_INF = 0;
static unsigned long g_u4BU64245_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_out(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);

extern int af_inf_pos;
extern int af_macro_pos;

static int s4BU64245_ReadReg(unsigned short * a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

    mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN,GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,1);
    
    i4RetValue = i2c_master_recv(g_pstBU64245_I2Cclient, pBuff , 2);

    if (i4RetValue < 0) 
    {
        BU64245DB("[BU64245]I2C read failed!! \n");
        return -1;
    }
		*a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];

    return 0;
}

static int s4BU64245_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;
    char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xc0), (char)(a_u2Data & 0xff)};

    mt_set_gpio_mode(GPIO_CAMERA_AF_EN_PIN,GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CAMERA_AF_EN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,1);
    
    g_pstBU64245_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstBU64245_I2Cclient, puSendCmd, 2);
    if (i4RetValue < 0) 
    {
        BU64245DB("[BU64245]I2C send failed!! \n");
        return -1;
    }
    return 0;
}

inline static int getBU64245Info(__user stBU64245_MotorInfo * pstMotorInfo)
{
    stBU64245_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4BU64245_MACRO;
    stMotorInfo.u4InfPosition     = g_u4BU64245_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4BU64245_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stBU64245_MotorInfo)))
    {
        BU64245DB("[BU64245] copy to user failed when getting motor information \n");
    }

    return 0;
}

inline static int moveBU64245(unsigned long a_u4Position)
{
    int ret = 0;
    
    if((a_u4Position > g_u4BU64245_MACRO) || (a_u4Position < g_u4BU64245_INF))
    {
        BU64245DB("[BU64245] out of range \n");
        return -EINVAL;
    }

    if (g_s4BU64245_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4BU64245_ReadReg(&InitPos);
	    
        spin_lock(&g_BU64245_SpinLock);
        if(ret == 0)
        {
            BU64245DB("[BU64245] Init Pos %6d \n", InitPos);
            g_u4CurrPosition = (unsigned long)InitPos;
        }
        else
        {		
            g_u4CurrPosition = 0;
        }
        g_s4BU64245_Opened = 2;
        spin_unlock(&g_BU64245_SpinLock);
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_BU64245_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_BU64245_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_BU64245_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_BU64245_SpinLock);			
    }
    else{
	    return 0;
    }

    spin_lock(&g_BU64245_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_BU64245_SpinLock);	

    //BU64245DB("[BU64245] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

            spin_lock(&g_BU64245_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(&g_BU64245_SpinLock);	
		
            if(s4BU64245_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(&g_BU64245_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(&g_BU64245_SpinLock);				
            }
            else
            {
                BU64245DB("[BU64245] set I2C failed when moving the motor \n");			
                spin_lock(&g_BU64245_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(&g_BU64245_SpinLock);				
            }

    return 0;
}

inline static int setBU64245Inf(unsigned long a_u4Position)
{
    spin_lock(&g_BU64245_SpinLock);
    g_u4BU64245_INF = a_u4Position;
    spin_unlock(&g_BU64245_SpinLock);	
    return 0;
}

inline static int setBU64245Macro(unsigned long a_u4Position)
{
    spin_lock(&g_BU64245_SpinLock);
    g_u4BU64245_MACRO = a_u4Position;
    spin_unlock(&g_BU64245_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long BU64245_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case BU64245IOC_G_MOTORINFO :
            i4RetValue = getBU64245Info((__user stBU64245_MotorInfo *)(a_u4Param));
        break;

        case BU64245IOC_T_MOVETO :
            i4RetValue = moveBU64245(a_u4Param);
        break;
 
        case BU64245IOC_T_SETINFPOS :
            i4RetValue = setBU64245Inf(a_u4Param);
        break;

        case BU64245IOC_T_SETMACROPOS :
            i4RetValue = setBU64245Macro(a_u4Param);
        break;
		
        default :
      	    BU64245DB("[BU64245] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int BU64245_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    BU64245DB("[BU64245] BU64245_Open - Start\n");

    spin_lock(&g_BU64245_SpinLock);

    if(g_s4BU64245_Opened)
    {
        spin_unlock(&g_BU64245_SpinLock);
        BU64245DB("[BU64245] the device is opened \n");
        return -EBUSY;
    }

    g_s4BU64245_Opened = 1;
		
    spin_unlock(&g_BU64245_SpinLock);

    BU64245DB("[BU64245] BU64245_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int BU64245_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    BU64245DB("[BU64245] BU64245_Release - Start\n");

    if (g_s4BU64245_Opened)
    {
        BU64245DB("[BU64245] feee \n");
        g_sr = 5;
#if 0  // no need to move when release AF,for it will make noise
	s4BU64245_WriteReg(200);
        msleep(10);
	s4BU64245_WriteReg(100);
        msleep(10);
#endif
        spin_lock(&g_BU64245_SpinLock);
        g_s4BU64245_Opened = 0;
        spin_unlock(&g_BU64245_SpinLock);

    }
//    mt_set_gpio_out(GPIO_CAMERA_AF_EN_PIN,0);
    BU64245DB("[BU64245] BU64245_Release - End\n");

    return 0;
}

int BU64245_move_to_nature(void)
{
	int i;
	int step = 20;
	int af_nature_position = 10;
	
	BU64245DB("[BU64245] move AF to nature position before power off \n");
	for(i=g_u4CurrPosition;i>af_nature_position;i-=step){
		s4BU64245_WriteReg(i);
		msleep(1);
	}
	s4BU64245_WriteReg(af_nature_position);
	return 0;
}

/*Interface for factoryTest,move af to infinite position*/
int MoveAF_to_INF(int pos) 
{
	if(pos){
		BU64245DB("[BU64245] move AF to infinite(value=%d) position\n",af_inf_pos);
		s4BU64245_WriteReg(af_inf_pos);
	}
	else{
		BU64245DB("[BU64245] move AF to macro(value=%d) position\n",af_macro_pos);
		s4BU64245_WriteReg(af_macro_pos);
	}
	
	return 0;
}

static const struct file_operations g_stBU64245_fops = {
    .owner = THIS_MODULE,
    .open = BU64245_Open,
    .release = BU64245_Release,
    .unlocked_ioctl = BU64245_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = BU64245_Ioctl,
#endif
};

inline static int Register_BU64245_CharDrv(void)
{
    struct device* vcm_device = NULL;

    BU64245DB("[BU64245] Register_BU64245_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_BU64245_devno, 0, 1,BU64245_DRVNAME) )
    {
        BU64245DB("[BU64245] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pBU64245_CharDrv = cdev_alloc();

    if(NULL == g_pBU64245_CharDrv)
    {
        unregister_chrdev_region(g_BU64245_devno, 1);

        BU64245DB("[BU64245] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pBU64245_CharDrv, &g_stBU64245_fops);

    g_pBU64245_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pBU64245_CharDrv, g_BU64245_devno, 1))
    {

        unregister_chrdev_region(g_BU64245_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "actuatordrv");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        BU64245DB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_BU64245_devno, NULL, BU64245_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    return 0;
}

inline static void Unregister_BU64245_CharDrv(void)
{
    BU64245DB("[BU64245] Unregister_BU64245_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pBU64245_CharDrv);

    unregister_chrdev_region(g_BU64245_devno, 1);
    
    device_destroy(actuator_class, g_BU64245_devno);

    class_destroy(actuator_class);

    BU64245DB("[BU64245] Unregister_BU64245_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int BU64245_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int BU64245_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id BU64245_i2c_id[] = {{BU64245_DRVNAME,0},{}};   
struct i2c_driver BU64245_i2c_driver = {                       
    .probe = BU64245_i2c_probe,                                   
    .remove = BU64245_i2c_remove,                           
    .driver.name = BU64245_DRVNAME,                 
    .id_table = BU64245_i2c_id,                             
};  

#if 0 
static int BU64245_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, BU64245_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int BU64245_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int BU64245_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    BU64245DB("[BU64245] BU64245_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstBU64245_I2Cclient = client;
    
    g_pstBU64245_I2Cclient->addr = 0x0c;//g_pstBU64245_I2Cclient->addr >> 1;
    
    //Register char driver
    i4RetValue = Register_BU64245_CharDrv();

    if(i4RetValue){

        BU64245DB("[BU64245] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_BU64245_SpinLock);

    BU64245DB("[BU64245] Attached!! \n");

    return 0;
}

static int BU64245_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&BU64245_i2c_driver);
}

static int BU64245_remove(struct platform_device *pdev)
{
    i2c_del_driver(&BU64245_i2c_driver);
    return 0;
}

static int BU64245_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int BU64245_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stBU64245_Driver = {
    .probe		= BU64245_probe,
    .remove	= BU64245_remove,
    .suspend	= BU64245_suspend,
    .resume	= BU64245_resume,
    .driver		= {
        .name	= PLATFORM_DRIVER_NAME,
        .owner	= THIS_MODULE,
    }
};
static struct platform_device g_stAF_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init BU64245_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
 
	if(platform_device_register(&g_stAF_device)){
        BU64245DB("failed to register AF driver\n");
        return -ENODEV;
    }
	
    if(platform_driver_register(&g_stBU64245_Driver)){
        BU64245DB("failed to register BU64245 driver\n");
        return -ENODEV;
    }
    return 0;
}

static void __exit BU64245_i2C_exit(void)
{
	platform_driver_unregister(&g_stBU64245_Driver);
}

module_init(BU64245_i2C_init);
module_exit(BU64245_i2C_exit);

MODULE_DESCRIPTION("BU64245 lens module driver");
MODULE_AUTHOR("KY Chen <vend_james-cc.wu@Mediatek.com>");
MODULE_LICENSE("GPL");


