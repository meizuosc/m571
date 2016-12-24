/* drivers/i2c/chips/max77819_pmic.c - MAX77819_PMIC motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <linux/xlog.h>


#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include "cust_charging.h"
#include <mach/charging.h>


#include "max77819_pmic.h"


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define DEBUG 0
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define C_I2C_FIFO_SIZE         8
#define MAX77819_PMIC_I2C_NUM 3

#define MAX77819_PMIC_DEV_NAME        "MAX77819_PMIC"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id max77819_pmic_i2c_id[] = {{MAX77819_PMIC_DEV_NAME, 0}, {} };
/*the adapter id will be available in customization*/

static struct i2c_board_info i2c_max77819_pmic __initdata = { I2C_BOARD_INFO("MAX77819_PMIC", 0xCC>>1)};

kal_bool chargin_hw_init_done = KAL_FALSE;


/*----------------------------------------------------------------------------*/
static int max77819_pmic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int max77819_pmic_i2c_remove(struct i2c_client *client);

/*----------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*/
typedef enum {
    BLS_TRC_READ  = 0x01,
    BLS_TRC_RAWDATA = 0x02,
    BLS_TRC_IOCTL   = 0x04,
    BLS_TRC_FILTER = 0x08,
} BLS_TRC;
/*----------------------------------------------------------------------------*/

struct max77819_pmic_i2c_data {
    struct i2c_client *client;

    /*misc*/
    atomic_t                trace;
    atomic_t                suspend;
    /*data*/
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver max77819_pmic_i2c_driver = {
    .driver = {
	.name           = MAX77819_PMIC_DEV_NAME,
    },
	.probe		= max77819_pmic_i2c_probe,
	.remove			= max77819_pmic_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend            = max77819_pmic_suspend,
    .resume             = max77819_pmic_resume,
#endif
	.id_table = max77819_pmic_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *max77819_pmic_i2c_client;
static struct platform_driver max77819_pmic_backlight_driver;
static struct max77819_pmic_i2c_data *obj_i2c_data;

/*----------------------------------------------------------------------------*/
#define BLS_TAG                  "[MAX77819_PMIC] "
#define BLS_FUN(f)               printk(KERN_ERR BLS_TAG"%s\n", __func__)
#define BLS_ERR(fmt, args...)    printk(KERN_ERR BLS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define BLS_LOG(fmt, args...)    printk(KERN_ERR BLS_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/

/*-------------------- power control function----------------------------------*/

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int max77819_pmic_open(struct inode *inode, struct file *file)
{
	file->private_data = max77819_pmic_i2c_client;

	if (file->private_data == NULL) {
		BLS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int max77819_pmic_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int max77819_pmic_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct max77819_pmic_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	kal_uint8  dat = 0;

	GSE_FUN();

	return err;
}
/*----------------------------------------------------------------------------*/
static int max77819_pmic_resume(struct i2c_client *client)
{
	struct max77819_pmic_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		BLS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void max77819_pmic_early_suspend(struct early_suspend *h)
{
	struct max77819_pmic_i2c_data *obj = container_of(h, struct max77819_pmic_i2c_data, early_drv);
//	int err;
	BLS_FUN();

	if (obj == NULL) {
		BLS_ERR("null pointer!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void max77819_pmic_late_resume(struct early_suspend *h)
{
	struct max77819_pmic_i2c_data *obj = container_of(h, struct max77819_pmic_i2c_data, early_drv);
	int err;

	BLS_FUN();
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

static DEFINE_MUTEX(max77819_pmic_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write max77819]
  *
  *********************************************************/

int max77819_pmic_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf = 0;
    char     readData = 0;
    int      ret = 0;

    BLS_FUN();
    mutex_lock(&max77819_pmic_i2c_access);

    cmd_buf = cmd;

    max77819_pmic_i2c_client->addr = (max77819_pmic_i2c_client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
    ret = i2c_master_send(max77819_pmic_i2c_client, &cmd_buf, (1<<8 | 1));
    if (ret < 0) {
	BLS_ERR("send command error!!\n");
	mutex_unlock(&max77819_pmic_i2c_access);
	return ret;
    }

    readData = cmd_buf;
    *returnData = readData;

	max77819_pmic_i2c_client->addr = max77819_pmic_i2c_client->addr & I2C_MASK_FLAG;

    mutex_unlock(&max77819_pmic_i2c_access);
    return 0;
}

int max77819_pmic_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret = 0;

	BLS_FUN();
    mutex_lock(&max77819_pmic_i2c_access);

    write_data[0] = cmd;
    write_data[1] = writeData;
    ret = i2c_master_send(max77819_pmic_i2c_client, write_data, 2);
    if (ret < 0) {
	BLS_ERR("send command error!!\n");
	mutex_unlock(&max77819_pmic_i2c_access);
	return ret;
    }
    mutex_unlock(&max77819_pmic_i2c_access);
    return 0;
}

u32 max77819_pmic_read_interface(kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 max77819_reg = 0;
    int ret = 0;

    ret = max77819_pmic_read_byte(RegNum, &max77819_reg);

        #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
	BLS_LOG("[max77819_pmic_read_interface] Reg[%x]=0x%x\n", RegNum, max77819_reg);
	#endif

    max77819_reg &= (MASK << SHIFT);
    *val = (max77819_reg >> SHIFT);

        #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
	BLS_LOG("[max77819_pmic_read_interface] val=0x%x\n", *val);
	#endif

    return ret;
}

u32 max77819_pmic_config_interface(kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 max77819_pmic_reg = 0;
    int ret = 0;

    ret = max77819_pmic_read_byte(RegNum, &max77819_pmic_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_config_interface] Reg[%x]=0x%x\n", RegNum, max77819_pmic_reg);
    #endif

    max77819_pmic_reg &= ~(MASK << SHIFT);
    max77819_pmic_reg |= (val << SHIFT);

    ret = max77819_pmic_write_byte(RegNum, max77819_pmic_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_config_interface] write Reg[%x]=0x%x\n", RegNum, max77819_pmic_reg);
    #endif

    return ret;
}

static int max77819_pmic_read_byte_sr(struct i2c_client *client, kal_uint8 addr, kal_uint8 *data)
{
   kal_uint8 buf;
	int ret = 0;

	client->addr = (client->addr & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
	buf = addr;
	ret = i2c_master_send(client, (const char *)&buf, 1<<8 | 1);
	if (ret < 0) {
		BLS_ERR("send command error!!\n");
		return ret;
	}

	*data = buf;
	client->addr = client->addr & I2C_MASK_FLAG;
	return 0;
}


int max77819_pmic_read_block(struct i2c_client *client, kal_uint8 addr, kal_uint8 *data, kal_uint8 len)
{
    if (len == 1) {
	return max77819_pmic_read_byte_sr(client, addr, data);
    } else {
	kal_uint8 beg = addr;
	struct i2c_msg msgs[2] = {
	{
		.addr = client->addr,    .flags = 0,
		.len = 1,                .buf = &beg
	    },
	{
		.addr = client->addr,    .flags = I2C_M_RD,
		.len = len,             .buf = data,
	}
	};
	int err;

	if (!client)
	return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
	    BLS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
	return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
	    BLS_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
	    err = -EIO;
	} else {
	    err = 0;    /*no error*/
	}
	return err;
    }

}
/*----------------------------------------------------------------------------*/
EXPORT_SYMBOL_GPL(max77819_pmic_read_block);


/*----------------------------------------------------------------------------*/
int max77819_pmic_write_block(struct i2c_client *client, kal_uint8 addr, kal_uint8 *data, kal_uint8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];

    if (!client)
	return -EINVAL;
    else if (len >= C_I2C_FIFO_SIZE) {
	BLS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
	return -EINVAL;
    }

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
	buf[num++] = data[idx];

    err = i2c_master_send(client, buf, num);
    if (err < 0) {
	BLS_ERR("send command error!!\n");
	return -EFAULT;
    } else {
	err = 0;    /*no error*/
    }
    return err;
}
/*----------------------------------------------------------------------------*/
EXPORT_SYMBOL_GPL(max77819_pmic_write_block);

int max77819_pmic_reg_write(kal_uint8 reg, kal_uint8 val)
{
    kal_uint8 max77819_reg = 0;
    int ret = 0;

    max77819_pmic_read_byte(reg, &max77819_reg);

    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_write] Before Reg[%x]=0x%x\n", reg, max77819_reg);
    #endif

    if (val == max77819_reg) {
	#ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
	BLS_LOG("[max77819_pmic_reg_write] Set Reg[%x] same value 0x%x\n", reg, max77819_reg);
	#endif
	return 0;
    }

	ret = max77819_pmic_write_byte(reg, val);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_write] write Reg[%x]=0x%x\n", reg, val);
    #endif

    max77819_pmic_read_byte(reg, &max77819_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_write] After Reg[%x]=0x%x\n", reg, max77819_reg);
    #endif

    if (max77819_reg != val) {
	return -1;
    }
    return ret;
}
/*----------------------------------------------------------------------------*/
EXPORT_SYMBOL_GPL(max77819_pmic_reg_write);


int max77819_pmic_reg_read(kal_uint8 reg, kal_uint8 *val)
{
    int ret = 0;

    ret = max77819_pmic_read_byte(reg, val);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_read] Reg[%x]=0x%x\n", reg, *val);
    #endif

    return ret;

}
EXPORT_SYMBOL_GPL(max77819_pmic_reg_read);


int max77819_pmic_reg_update_bits(kal_uint8 reg, kal_uint8 mask, kal_uint8 val)
{
    kal_uint8 max77819_reg = 0;
    kal_uint8 old_reg = 0;
    kal_uint8 new_reg = 0;
    int ret = 0;

    max77819_pmic_read_byte(reg, &max77819_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_update_bits] Before Reg[%x]=0x%x\n", reg, max77819_reg);
    #endif

    old_reg = max77819_reg;
    max77819_reg &= ~(mask);
    max77819_reg |= val;
    if (old_reg == max77819_reg) {
	#ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
	BLS_LOG("[max77819_pmic_reg_update_bits] Set Reg[%x] same value 0x%x\n", reg, max77819_reg);
	#endif
	return 0;
    }

    new_reg = max77819_reg;

    ret = max77819_pmic_write_byte(reg, max77819_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_update_bits] write Reg[%x]=0x%x\n", reg, max77819_reg);
    #endif

	max77819_pmic_read_byte(reg, &max77819_reg);
    #ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
    BLS_LOG("[max77819_pmic_reg_update_bits] After Reg[%x]=0x%x\n", reg, max77819_reg);
    #endif

    if (max77819_reg != new_reg) {
	return -1;
    }
    return ret;

}
EXPORT_SYMBOL_GPL(max77819_pmic_reg_update_bits);


/*----------------------------------------------------------------------------*/
static int max77819_pmic_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct max77819_pmic_i2c_data *obj;

	int err = 0, i = 0;

	BLS_FUN();

	if (!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
		err = -1;
		goto exit;
	}

	memset(obj, 0, sizeof(struct max77819_pmic_i2c_data));

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

	max77819_pmic_i2c_client = new_client;
	chargin_hw_init_done = KAL_TRUE;



kal_uint8 reg_val;
		for (i = 0x30 ; i <= 0x4b; i++)
			 max77819_pmic_reg_read(i, &reg_val);

exit:

	return err;
}

/*----------------------------------------------------------------------------*/
static int max77819_pmic_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	max77819_pmic_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init max77819_pmic_init(void)
{
	BLS_FUN();
	i2c_register_board_info(/*2*/MAX77819_PMIC_I2C_NUM, &i2c_max77819_pmic, 1);
	if (i2c_add_driver(&max77819_pmic_i2c_driver)) {
		BLS_ERR("add driver error\n");
		return -1;
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit max77819_pmic_exit(void)
{
	BLS_FUN();
	i2c_del_driver(&max77819_pmic_i2c_driver);
}
/*----------------------------------------------------------------------------*/
module_init(max77819_pmic_init);
module_exit(max77819_pmic_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MAX77819_PMIC I2C driver");
MODULE_AUTHOR("Bing.Song@mediatek.com");
