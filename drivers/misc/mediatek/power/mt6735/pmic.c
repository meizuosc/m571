/*****************************************************************************
 *
 * Filename:
 * ---------
 *    pmic.c 
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines PMIC functions
 *
 * Author:
 * -------
 * WY Chuang
 *
 ****************************************************************************/
#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/earlysuspend.h>
#include <linux/seq_file.h>
#include <linux/of_fdt.h>


#include <linux/uaccess.h>

#include <linux/regulator/driver.h>   
#include <linux/regulator/machine.h>   


#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/pmic.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_gpio.h>
#include <mach/mtk_rtc.h>
#include <mach/mt_spm_mtcmos.h>

#include <mach/battery_common.h>
#include <linux/time.h>

#include <mach/mt_pm_ldo.h>
#include "pmic_dvt.h"

#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
#include <mach/mt_boot.h>
#include <mach/system.h>
#include "mach/mt_gpt.h"
#endif

#include <mach/battery_meter.h>
#include <mach/mt6311.h>
#include <cust_pmic.h>
#include <cust_eint.h>
#include <cust_battery_meter.h>

 
//==============================================================================
// PMIC extern functions
//==============================================================================
extern kal_uint16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern kal_uint16 pmic_get_register_value(PMU_FLAGS_LIST_ENUM flagname);
extern void pmic_auxadc_init(void);

//==============================================================================
// PMIC extern variable
//==============================================================================
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
extern void mt_power_off(void);
static kal_bool long_pwrkey_press = false;
static unsigned long timer_pre = 0; 
static unsigned long timer_pos = 0; 
#define LONG_PWRKEY_PRESS_TIME         500*1000000    //500ms
#endif


//==============================================================================
// Global variable
//==============================================================================
unsigned int g_eint_pmic_num = 206;
unsigned int g_cust_eint_mt_pmic_debounce_cn = 1; 
unsigned int g_cust_eint_mt_pmic_type = 4;
unsigned int g_cust_eint_mt_pmic_debounce_en = 1;

//==============================================================================
// PMIC related define
//==============================================================================
static DEFINE_MUTEX(pmic_lock_mutex);
#define PMIC_EINT_SERVICE

//==============================================================================
// PMIC read/write APIs
//==============================================================================
#if 0 //defined(CONFIG_MTK_FPGA)
    // no CONFIG_PMIC_HW_ACCESS_EN
#else
    #define CONFIG_PMIC_HW_ACCESS_EN
#endif

#ifndef CONFIG_MEIZU_CLOSE_MTK_LOG
#define PMICLOG(fmt, arg...)   printk(PMICTAG fmt,##arg)
#else
#define PMICLOG(fmt, arg...)   no_printk(PMICTAG fmt,##arg)
#endif



static DEFINE_MUTEX(pmic_access_mutex);

U32 pmic_read_interface (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    mutex_lock(&pmic_access_mutex);

    //mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_read_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_read_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= (MASK << SHIFT);
    *val = (pmic_reg >> SHIFT);
    //PMICLOG"[pmic_read_interface] val=0x%x\n", *val);

    mutex_unlock(&pmic_access_mutex);
#else
    //PMICLOG("[pmic_read_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    mutex_lock(&pmic_access_mutex);

    //1. mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= ~(MASK << SHIFT);
    pmic_reg |= (val << SHIFT);

    //2. mt_write_byte(RegNum, pmic_reg);
    return_value= pwrap_wacs2(1, (RegNum), pmic_reg, &rdata);
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_config_interface] write Reg[%x]=0x%x\n", RegNum, pmic_reg);

    #if 0
    //3. Double Check
    //mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap write data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    PMICLOG("[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);
    #endif

    mutex_unlock(&pmic_access_mutex);
#else
    //PMICLOG("[pmic_config_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

U32 pmic_read_interface_nolock (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

	  /* pmic wrapper has spinlock protection. pmic do not to do it again */
    //mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_read_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_read_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= (MASK << SHIFT);
    *val = (pmic_reg >> SHIFT);
    //PMICLOG"[pmic_read_interface] val=0x%x\n", *val);

#else
    //PMICLOG("[pmic_read_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

U32 pmic_config_interface_nolock (U32 RegNum, U32 val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    /* pmic wrapper has spinlock protection. pmic do not to do it again */

    //1. mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= ~(MASK << SHIFT);
    pmic_reg |= (val << SHIFT);

    //2. mt_write_byte(RegNum, pmic_reg);
    return_value= pwrap_wacs2(1, (RegNum), pmic_reg, &rdata);
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //PMICLOG"[pmic_config_interface] write Reg[%x]=0x%x\n", RegNum, pmic_reg);

    #if 0
    //3. Double Check
    //mt_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        PMICLOG("[pmic_config_interface] Reg[%x]= pmic_wrap write data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    PMICLOG("[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);
    #endif

#else
    //PMICLOG("[pmic_config_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

//==============================================================================
// PMIC lock/unlock APIs
//==============================================================================
void pmic_lock(void)
{
    mutex_lock(&pmic_lock_mutex);
}

void pmic_unlock(void)
{
    mutex_unlock(&pmic_lock_mutex);
}

kal_uint32 upmu_get_reg_value(kal_uint32 reg)
{
    U32 ret=0;
    U32 reg_val=0;
    
    ret=pmic_read_interface(reg, &reg_val, 0xFFFF, 0x0);
    
    return reg_val;
}
EXPORT_SYMBOL(upmu_get_reg_value);

void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val)
{
    U32 ret=0;
    
    ret=pmic_config_interface(reg, reg_val, 0xFFFF, 0x0);    
}

unsigned int get_pmic_mt6325_cid(void)
{
    return 0;
}

U32 get_mt6325_pmic_chip_version (void)
{
    return 0;
}
/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void mt6328_dump_register(void)
{
    kal_uint8 i=0;

    PMICLOG("dump PMIC 6328 register\n");

   for(i=0;i<=0x0fae;i=i+10)
   {
	PMICLOG("Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x\n",
		i,upmu_get_reg_value(i),i+1,upmu_get_reg_value(i+1),i+2,upmu_get_reg_value(i+2),i+3,upmu_get_reg_value(i+3),i+4,upmu_get_reg_value(i+4));

       PMICLOG("Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x Reg[0x%x]=0x%x\n",
      		i+5,upmu_get_reg_value(i+5),i+6,upmu_get_reg_value(i+6),i+7,upmu_get_reg_value(i+7),i+8,upmu_get_reg_value(i+8),i+9,upmu_get_reg_value(i+9));
   }

}

//==============================================================================
// workaround for vio18 drop issue in E1
//==============================================================================

const static unsigned char mt6328_VIO[] = {
	12,	13,	14,	15,	0,	1,	2,	3,	8,	8,	8,	8,	8,	9,	10,	11
};

static unsigned char vio18_cal;

void upmu_set_rg_vio18_cal(kal_uint32 en)
{
	U32 chip_version = 0;
	
	chip_version = pmic_get_register_value(PMIC_SWCID);

	if(chip_version==PMIC6328_E1_CID_CODE)
	{
		if(en==1)
		{
			pmic_set_register_value(PMIC_RG_VIO18_CAL,mt6328_VIO[vio18_cal]);
		}
		else
		{
			pmic_set_register_value(PMIC_RG_VIO18_CAL,vio18_cal);
		}
	}

}
EXPORT_SYMBOL(upmu_set_rg_vio18_cal);


const static unsigned char mt6328_VIO_1_84[] = {
	14,	15,	0,	1,	2,	3,	4,	5,	8,	8,	8,	9,	10,	11,	12,	13
};



void upmu_set_rg_vio18_184(void)
{
	PMICLOG("[upmu_set_rg_vio18_184] old cal=%d new cal=%d.\r\n", vio18_cal,mt6328_VIO_1_84[vio18_cal]);
	pmic_set_register_value(PMIC_RG_VIO18_CAL,mt6328_VIO_1_84[vio18_cal]);

}


const static unsigned char mt6328_VMC_1_86[] = {
	14,	15,	0,	1,	2,	3,	4,	5,	8,	8,	8,	9,	10,	11,	12,	13
};
	

static unsigned char vmc_cal;

void upmu_set_rg_vmc_184(kal_uint8 x)
{
    
	PMICLOG("[upmu_set_rg_vio18_184] old cal=%d new cal=%d.\r\n", vmc_cal,mt6328_VMC_1_86[vmc_cal]);
    if(x==0)
    {
    	pmic_set_register_value(PMIC_RG_VMC_CAL,vmc_cal);
		PMICLOG("[upmu_set_rg_vio18_184]:0 old cal=%d new cal=%d.\r\n", vmc_cal,vmc_cal);
    }
	else
	{
		pmic_set_register_value(PMIC_RG_VMC_CAL,mt6328_VMC_1_86[vmc_cal]);
		PMICLOG("[upmu_set_rg_vio18_184]:1 old cal=%d new cal=%d.\r\n", vmc_cal,mt6328_VMC_1_86[vmc_cal]);
	}
}


static kal_uint8 vcamd_cal=0;

const static unsigned char mt6328_vcamd[] = {
	1,	2,	3,	4,	5,	6,	7,	7,	9,	10,	11,	12,	13,	14,	15,	0
};


void upmu_set_rg_vcamd(kal_uint8 x)
{
    
	PMICLOG("[upmu_set_rg_vcamd] old cal=%d new cal=%d.\r\n", vcamd_cal,mt6328_vcamd[vcamd_cal]);
    if(x==0)
    {
    	pmic_set_register_value(PMIC_RG_VCAMD_CAL,vcamd_cal);
		PMICLOG("[upmu_set_rg_vcamd]:0 old cal=%d new cal=%d.\r\n", vcamd_cal,vcamd_cal);
    }
	else
	{
		pmic_set_register_value(PMIC_RG_VCAMD_CAL,mt6328_vcamd[vcamd_cal]);
		PMICLOG("[upmu_set_rg_vcamd]:1 old cal=%d new cal=%d.\r\n", vcamd_cal,mt6328_vcamd[vcamd_cal]);
	}
}




//==============================================================================
// workaround for VMC voltage list
//==============================================================================
int pmic_read_VMC_efuse(void)
{

	kal_uint32 val;
	pmic_set_register_value(PMIC_RG_EFUSE_CK_PDN_HWEN,0);
	pmic_set_register_value(PMIC_RG_EFUSE_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_OTP_RD_SW,1);

	pmic_set_register_value(PMIC_RG_OTP_PA,0xd*2);

	if(pmic_get_register_value(PMIC_RG_OTP_RD_TRIG)==0)
	{
		pmic_set_register_value(PMIC_RG_OTP_RD_TRIG,1);
	}
	else
	{
		pmic_set_register_value(PMIC_RG_OTP_RD_TRIG,0);
	}

	udelay(50); /*efuse hw bug, need to wait delay */

	while(pmic_get_register_value(PMIC_RG_OTP_RD_BUSY)==1)
	{
	}

	val=pmic_get_register_value(PMIC_RG_OTP_DOUT_SW);

	val=val*0x80;

	pmic_set_register_value(PMIC_RG_EFUSE_CK_PDN_HWEN,1);
	pmic_set_register_value(PMIC_RG_EFUSE_CK_PDN,1);

	return val;
}

//==============================================================================
// upmu_interrupt_chrdet_int_en
//==============================================================================
void upmu_interrupt_chrdet_int_en(kal_uint32 val)
{
    PMICLOG("[upmu_interrupt_chrdet_int_en] val=%d.\r\n", val);

    //mt6325_upmu_set_rg_int_en_chrdet(val);
    pmic_set_register_value(PMIC_RG_INT_EN_CHRDET,val);
}
EXPORT_SYMBOL(upmu_interrupt_chrdet_int_en);


//==============================================================================
// PMIC charger detection
//==============================================================================
kal_uint32 upmu_get_rgs_chrdet(void)
{
	kal_uint32 val=0;

	//val = mt6325_upmu_get_rgs_chrdet();
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
	PMICLOG("[upmu_get_rgs_chrdet] CHRDET status = %d\n", val);

	return val;
}

//==============================================================================
// mt-pmic dev_attr APIs
//==============================================================================
U32 g_reg_value=0;
static ssize_t show_pmic_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_pmic_access] 0x%x\n", g_reg_value);
    return sprintf(buf, "%u\n", g_reg_value);
}
static ssize_t store_pmic_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    U32 reg_value = 0;
    U32 reg_address = 0;
    PMICLOG("[store_pmic_access] \n");
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_pmic_access] buf is %s \n",buf);
        reg_address = simple_strtoul(buf,&pvalue,16);

        if(size > 5)
        {
            reg_value = simple_strtoul((pvalue+1),NULL,16);
            PMICLOG("[store_pmic_access] write PMU reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=pmic_config_interface(reg_address, reg_value, 0xFFFF, 0x0);
        }
        else
        {
            ret=pmic_read_interface(reg_address, &g_reg_value, 0xFFFF, 0x0);
            PMICLOG("[store_pmic_access] read PMU reg 0x%x with value 0x%x !\n",reg_address,g_reg_value);
            PMICLOG("[store_pmic_access] Please use \"cat pmic_access\" to get value\r\n");
        }
    }
    return size;
}
static DEVICE_ATTR(pmic_access, 0664, show_pmic_access, store_pmic_access); //664

//==============================================================================
// DVT entry
//==============================================================================
kal_uint8 g_reg_value_pmic=0;

static ssize_t show_pmic_dvt(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_pmic_dvt] 0x%x\n", g_reg_value_pmic);
    return sprintf(buf, "%u\n", g_reg_value_pmic);
}

static ssize_t store_pmic_dvt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int test_item = 0;
    
    PMICLOG("[store_pmic_dvt] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_pmic_dvt] buf is %s and size is %zu \n",buf,size);
        test_item = simple_strtoul(buf,&pvalue,10);
        PMICLOG("[store_pmic_dvt] test_item=%d\n", test_item);

        #ifdef MTK_PMIC_DVT_SUPPORT 
        pmic_dvt_entry(test_item);
        #else
        PMICLOG("[store_pmic_dvt] no define MTK_PMIC_DVT_SUPPORT\n");
        #endif
    }    
    return size;
}
static DEVICE_ATTR(pmic_dvt, 0664, show_pmic_dvt, store_pmic_dvt);


//==============================================================================
// PMIC6328 linux reguplator driver
//==============================================================================
extern PMU_FLAG_TABLE_ENTRY pmu_flags_table[];

static int mtk_regulator_enable(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;
	kal_uint32 add=0,val=0;

	mreg=container_of(rdesc,struct mtk_regulator,desc);
	if (mreg->en_reg!=0)
	{
		pmic_set_register_value(mreg->en_reg,1);
		add=pmu_flags_table[mreg->en_reg].offset;
		val=upmu_get_reg_value(pmu_flags_table[mreg->en_reg].offset);
	}

	PMICLOG("regulator_enable(name=%s id=%d en_reg=%x vol_reg=%x) [%x]=0x%x\n",rdesc->name, rdesc->id
		,mreg->en_reg,mreg->vol_reg,add,val);

	return 0;
}

static int mtk_regulator_disable(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;
	kal_uint32 add=0,val=0;

	mreg=container_of(rdesc,struct mtk_regulator,desc);
		
	if (rdev->use_count==0)
	{
		PMICLOG("regulator_disable fail (name=%s use_count=%d) \n",rdesc->name,rdev->use_count);
		return -1;
	}

	if (mreg->en_reg!=0)
	{
		pmic_set_register_value(mreg->en_reg,0);
		add=pmu_flags_table[mreg->en_reg].offset;
		val=upmu_get_reg_value(pmu_flags_table[mreg->en_reg].offset);		
	}

	PMICLOG("regulator_disable(name=%s id=%d en_reg=%x vol_reg=%x use_count=%d) [%x]=0x%x\n",rdesc->name, rdesc->id
		,mreg->en_reg,mreg->vol_reg,rdev->use_count,add,val);

	return 0;
}

static int mtk_regulator_is_enabled(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;
	int en;

	mreg=container_of(rdesc,struct mtk_regulator,desc);

	en=pmic_get_register_value(mreg->en_reg);
		
	PMICLOG("regulator_is_enabled(name=%s id=%d en_reg=%x vol_reg=%x en=%d)\n",rdesc->name, rdesc->id
		,mreg->en_reg,mreg->vol_reg,en);
	
	return en;
}

static int mtk_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;
	unsigned char regVal=0;
	const int *pVoltage;
	int voltage=0;
	kal_uint32 add=0,val=9;

	mreg=container_of(rdesc,struct mtk_regulator,desc);
	
	if(mreg->desc.n_voltages!=1)
	{
		if(mreg->vol_reg!=0)
		{
			regVal=pmic_get_register_value(mreg->vol_reg);
			if(mreg->pvoltages!=NULL)
			{
				pVoltage=(const int*)mreg->pvoltages;
				voltage=pVoltage[regVal];
				add=pmu_flags_table[mreg->en_reg].offset;
				val=upmu_get_reg_value(pmu_flags_table[mreg->en_reg].offset);
			}
			else
			{
				voltage=mreg->desc.min_uV+mreg->desc.uV_step*regVal;
			}
		}
		else
		{
			PMICLOG("regulator_get_voltage_sel bugl(name=%s id=%d en_reg=%x vol_reg=%x)\n",rdesc->name, rdesc->id
				,mreg->en_reg,mreg->vol_reg);		
		}
	}
	else
	{
		if(mreg->vol_reg!=0)
		{
			regVal=0;
			pVoltage=(const int*)mreg->pvoltages;
			voltage=pVoltage[regVal];
		}
		else
		{
			PMICLOG("regulator_get_voltage_sel bugl(name=%s id=%d en_reg=%x vol_reg=%x)\n",rdesc->name, rdesc->id
				,mreg->en_reg,mreg->vol_reg);		
		}
	}
	
	PMICLOG("regulator_get_voltage_sel(name=%s id=%d en_reg=%x vol_reg=%x reg/sel:%d voltage:%d [0x%x]=0x%x)\n",rdesc->name, rdesc->id
		,mreg->en_reg,mreg->vol_reg,regVal,voltage,add,val);

	return regVal;
}

static int mtk_regulator_set_voltage_sel(struct regulator_dev *rdev,
					      unsigned selector)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;

	mreg=container_of(rdesc,struct mtk_regulator,desc);
		
	PMICLOG("regulator_set_voltage_sel(name=%s id=%d en_reg=%x vol_reg=%x selector=%d)\n",rdesc->name, rdesc->id
		,mreg->en_reg,mreg->vol_reg,selector);
	

	if(strcmp(rdesc->name,"VCAMD")==0)
	{

		if(selector==3)
		{	
			upmu_set_rg_vcamd(1);
		}
		else
		{
			upmu_set_rg_vcamd(0);
		}
	}

	if(strcmp(rdesc->name,"VRF18_1")==0)
	 {
					 if(selector==4)
					 {				
									 pmic_config_interface(0xA86,0x0,0x1,2);
					 }
					 else
					 {
									pmic_config_interface(0xA86,0x1,0x1,2);
					 }
	 }

	
	if(strcmp(rdesc->name,"VEFUSE")==0)
	{
					if (mreg->vol_reg!=0)
					{
									pmic_set_register_value(mreg->vol_reg,selector+3);
					}				
	}
	else
	{
	if (mreg->vol_reg!=0)
	{
		pmic_set_register_value(mreg->vol_reg,selector);
	}
	}


	return 0;
}

static int mtk_regulator_list_voltage(struct regulator_dev *rdev,
					   unsigned selector)
{
	const struct regulator_desc *rdesc=rdev->desc;
	struct mtk_regulator *mreg;
	const int *pVoltage;
	int voltage=0;

	mreg=container_of(rdesc,struct mtk_regulator,desc);
		
	if(mreg->desc.n_voltages!=1)
	{
		if(mreg->vol_reg!=0)
		{
			if(mreg->pvoltages!=NULL)
			{
				pVoltage=(const int*)mreg->pvoltages;
				voltage=pVoltage[selector];
			}
			else
			{
				voltage=mreg->desc.min_uV+mreg->desc.uV_step*selector;
			}
		}
		else
		{
			PMICLOG("mtk_regulator_list_voltage bugl(name=%s id=%d en_reg=%x vol_reg=%x)\n",rdesc->name, rdesc->id
				,mreg->en_reg,mreg->vol_reg);		
		}
	}
	else
	{
				pVoltage=(const int*)mreg->pvoltages;
				voltage=pVoltage[0];
	}
	
//	PMICLOG("regulator_list_voltage(name=%s id=%d en_reg=%x vol_reg=%x selector=%d voltage=%d)\n",rdesc->name, rdesc->id
//		,mreg->en_reg,mreg->vol_reg,selector,voltage);

    return voltage;
}




static struct regulator_ops mtk_regulator_ops = {
	.enable		     = mtk_regulator_enable,
	.disable	     = mtk_regulator_disable,
	.is_enabled	     = mtk_regulator_is_enabled,
	.get_voltage_sel = mtk_regulator_get_voltage_sel,
	.set_voltage_sel = mtk_regulator_set_voltage_sel,
	.list_voltage	 = mtk_regulator_list_voltage,
};

static ssize_t show_LDO_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	unsigned int ret_value=0;

	mreg=container_of(attr,struct mtk_regulator,en_att);

	ret_value=pmic_get_register_value(mreg->en_reg);

	PMICLOG("[EM] LDO_%s_STATUS : %d\n",mreg->desc.name,ret_value);
	return sprintf(buf, "%u\n", ret_value);	
}

static ssize_t store_LDO_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    PMICLOG("[EM] Not Support Write Function\n");    
    return size;
}

static ssize_t show_LDO_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	const int *pVoltage;

	kal_uint16 regVal;
	unsigned int ret_value=0;

	mreg=container_of(attr,struct mtk_regulator,voltage_att);

	if(mreg->desc.n_voltages!=1)
	{
		if(mreg->vol_reg!=0)
		{
			regVal=pmic_get_register_value(mreg->vol_reg);
			if(mreg->pvoltages!=NULL)
			{
				pVoltage=(const int*)mreg->pvoltages;
				ret_value=pVoltage[regVal];
			}
			else
			{
				ret_value=mreg->desc.min_uV+mreg->desc.uV_step*regVal;
			}
		}
		else
		{
			PMICLOG("[EM][ERROR] LDO_%s_VOLTAGE : voltage=0 vol_reg=0\n", mreg->desc.name);
		}
	}
	else
	{
		pVoltage=(const int*)mreg->pvoltages;
		ret_value=pVoltage[0];
	}

	ret_value=ret_value/1000;
	PMICLOG("[EM] LDO_%s_VOLTAGE : %d\n", mreg->desc.name,ret_value);
	return sprintf(buf, "%u\n", ret_value);

}

static ssize_t store_LDO_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	PMICLOG("[EM] Not Support Write Function\n");    
	return size;
}

static ssize_t show_BUCK_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	unsigned int ret_value=0;

	mreg=container_of(attr,struct mtk_regulator,en_att);

	ret_value=pmic_get_register_value(mreg->qi_en_reg);

	PMICLOG("[EM] BUCK_%s_STATUS : %d\n",mreg->desc.name,ret_value);
	return sprintf(buf, "%u\n", ret_value);	
}
static ssize_t store_BUCK_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	PMICLOG("[EM] Not Support Write Function\n");    
	return size;
}

static ssize_t show_BUCK_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct mtk_regulator *mreg;
	const int *pVoltage;

	kal_uint16 regVal;
	unsigned int ret_value=0;

	mreg=container_of(attr,struct mtk_regulator,voltage_att);

	if(mreg->desc.n_voltages!=1)
	{
		if(mreg->qi_vol_reg!=0)
		{
			regVal=pmic_get_register_value(mreg->qi_vol_reg);
			if(mreg->pvoltages!=NULL)
			{
				pVoltage=(const int*)mreg->pvoltages;
				ret_value=pVoltage[regVal];
			}
			else
			{
				ret_value=mreg->desc.min_uV+mreg->desc.uV_step*regVal;
			}
		}
		else
		{
			PMICLOG("[EM][ERROR] buck_%s_VOLTAGE : voltage=0 vol_reg=0\n", mreg->desc.name);
		}
	}
	else
	{
		pVoltage=(const int*)mreg->pvoltages;	
		ret_value=pVoltage[0];	
	}

	ret_value=ret_value/1000;
	PMICLOG("[EM] BUCK_%s_VOLTAGE : %d\n", mreg->desc.name,ret_value);
	return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	PMICLOG("[EM] Not Support Write Function\n");    
	return size;
}

//LDO setting
const static int mt6328_VCAMA_voltages[] = {
	1500000,
	1800000,
	2500000,
	2800000,
};

const static int mt6328_VCN33_voltages[] = {
	3300000,
	3400000,
	3500000,
	3600000,
};

const static int mt6328_VEFUSE_voltages[] = {
	1800000,
	1900000,
	2000000,
	2100000,
	2200000,	
};

//VSIM1 & VSIM2
const static int mt6328_VSIM1_voltages[] = {
	1700000,
	1800000,
	1860000,
	2760000,
	3000000,	
	3100000,		
};

const static int mt6328_VEMC33_voltages[] = {
	1800000,
	2900000,	
	3000000,
	3300000,
};

const static int mt6328_VMCH_voltages[] = {
	2900000,
	3000000,		
	3300000,
};

const static int mt6328_VMC_voltages[] = {
	1800000,
	2900000,
	3000000,
	3300000,	
};

const static int mt6328_VMC_E1_1_voltages[] = {
	2500000,
	2900000,
	3000000,
	3300000,	
};

const static int mt6328_VMC_E1_2_voltages[] = {
	1300000,
	1800000,
	2900000,
	3300000,	
};


//VCAM_AF & VIBR
const static int mt6328_VCAM_AF_voltages[] = {
	1200000,
	1300000,
	1500000,
	1800000,	
	2000000,
	2800000,
	3000000,
	3300000,		
};

const static int mt6328_VGP1_voltages[] = {
	1200000,
	1300000,
	1500000,
	1800000,	
	2500000,
	2800000,
	3000000,
	3300000,		
};

const static int mt6328_VCAMD_voltages[] = {
	 900000,
	1000000,
	1100000,
	1200000,	
	1300000,
	1500000,	
};

const static int mt6328_VRF18_1_voltages[] = {
	1200000,
	1300000,
	1500000,
    1800000,	
	1825000,	
};

const static int mt6328_VCAM_IO_voltages[] = {
	1200000,
	1300000,
	1500000,
	1800000,	
};

const static int mt6328_VM_voltages[] = {
	1240000,
	1390000,
	1540000,
};

const static int mt6328_1v8_voltages[] = {
	1800000,
};

const static int mt6328_2v8_voltages[] = {
	2800000,
};

const static int mt6328_3v3_voltages[] = {
	3300000,
};

const static int mt6328_1v825_voltages[] = {
	1825000,
};

struct mtk_regulator mtk_ldos[] = {
	PMIC_LDO_GEN1(VAUX18,		PMIC_RG_VAUX18_EN, 		NULL,mt6328_1v8_voltages,	0,PMIC_EN),
	PMIC_LDO_GEN1(VTCXO_0,	PMIC_RG_VTCXO_0_EN, 	NULL,mt6328_2v8_voltages,	0,PMIC_EN),
	PMIC_LDO_GEN1(VTCXO_1,	PMIC_RG_VTCXO_1_EN,  	NULL,mt6328_2v8_voltages,	0,PMIC_EN),
	PMIC_LDO_GEN1(VAUD28,		PMIC_RG_VAUD28_EN, 		NULL,mt6328_2v8_voltages,	1,PMIC_EN),	
	PMIC_LDO_GEN1(VCN28,		PMIC_RG_VCN28_EN,		NULL,mt6328_2v8_voltages,	1,PMIC_EN),		
	PMIC_LDO_GEN1(VCAMA,		PMIC_RG_VCAMA_EN, 		PMIC_RG_VCAMA_VOSEL,mt6328_VCAMA_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VCN33_BT,	PMIC_RG_VCN33_EN_BT, 	PMIC_RG_VCN33_VOSEL,mt6328_VCN33_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VCN33_WIFI,	PMIC_RG_VCN33_EN_WIFI, 	PMIC_RG_VCN33_VOSEL,mt6328_VCN33_voltages,1,PMIC_EN_VOL),		
	PMIC_LDO_GEN1(VUSB33,		PMIC_RG_VUSB33_EN,		NULL,mt6328_3v3_voltages,	1,PMIC_EN),
	PMIC_LDO_GEN1(VEFUSE,		PMIC_RG_VEFUSE_EN, 		PMIC_RG_VEFUSE_VOSEL,mt6328_VEFUSE_voltages,0,PMIC_EN_VOL),		
	PMIC_LDO_GEN1(VSIM1,		PMIC_RG_VSIM1_EN, 		PMIC_RG_VSIM1_VOSEL,mt6328_VSIM1_voltages,0,PMIC_EN_VOL),	
	PMIC_LDO_GEN1(VSIM2,		PMIC_RG_VSIM2_EN, 		PMIC_RG_VSIM2_VOSEL,mt6328_VSIM1_voltages,0,PMIC_EN_VOL),	
	PMIC_LDO_GEN1(VEMC_3V3,	PMIC_RG_VEMC_3V3_EN, 	PMIC_RG_VEMC_3V3_VOSEL,mt6328_VEMC33_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VMCH,		PMIC_RG_VMCH_EN, 		PMIC_RG_VMCH_VOSEL,mt6328_VMCH_voltages,1,PMIC_EN_VOL),		
	PMIC_LDO_GEN1(TREF,		PMIC_RG_TREF_EN,		NULL,mt6328_1v8_voltages,	0,PMIC_EN),	
	PMIC_LDO_GEN1(VMC,		PMIC_RG_VMC_EN, 		PMIC_RG_VMC_VOSEL,mt6328_VMC_voltages,1,PMIC_EN_VOL),		
	PMIC_LDO_GEN1(VCAMAF,		PMIC_RG_VCAMAF_EN, 		PMIC_RG_VCAMAF_VOSEL,mt6328_VCAM_AF_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VIO28,		PMIC_RG_VIO28_EN, 		NULL,mt6328_2v8_voltages,	0,PMIC_EN),	
	PMIC_LDO_GEN1(VGP1,		PMIC_RG_VGP1_EN, 		PMIC_RG_VGP1_VOSEL,mt6328_VGP1_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VIBR,		PMIC_RG_VIBR_EN, 		PMIC_RG_VIBR_VOSEL,mt6328_VCAM_AF_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VCAMD,		PMIC_RG_VCAMD_EN, 		PMIC_RG_VCAMD_VOSEL,mt6328_VCAMD_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VRF18_0,		PMIC_RG_VRF18_0_EN,		NULL,mt6328_1v825_voltages,0,PMIC_EN),	
	PMIC_LDO_GEN1(VRF18_1,		PMIC_RG_VRF18_1_EN, 	PMIC_RG_VRF18_1_VOSEL,mt6328_VRF18_1_voltages,0,PMIC_EN_VOL),
	PMIC_LDO_GEN1(VIO18,		PMIC_RG_VIO18_EN,		NULL,mt6328_1v8_voltages,	0,PMIC_EN),	
	PMIC_LDO_GEN1(VCN18,		PMIC_RG_VCN18_EN,		NULL,mt6328_1v8_voltages,	1,PMIC_EN),	
	PMIC_LDO_GEN1(VCAMIO,		PMIC_RG_VCAMIO_EN, 		PMIC_RG_VCAMIO_VOSEL,mt6328_VCAM_IO_voltages,1,PMIC_EN_VOL),
	PMIC_LDO_GEN2(VSRAM,		PMIC_RG_VSRAM_EN, 		PMIC_RG_VSRAM_VOSEL,700000,1493750,6250,1,PMIC_EN_VOL),	
	PMIC_LDO_GEN1(VM,			PMIC_RG_VM_EN, 			PMIC_RG_VM_VOSEL,mt6328_VM_voltages,1,PMIC_EN_VOL),		
};

static struct mtk_regulator mtk_bucks[] = {
	PMIC_BUCK_GEN(VPA,			PMIC_QI_VPA_EN,		PMIC_NI_VPA_VOSEL, 		500000,	3650000,50000),
	PMIC_BUCK_GEN(VPROC,		PMIC_QI_VPROC_EN,	PMIC_NI_VPROC_VOSEL, 	600000,	1393750,6250),
	PMIC_BUCK_GEN(VCORE1,		PMIC_QI_VCORE1_EN,	PMIC_NI_VCORE1_VOSEL,  	600000,	1393750,6250),
	PMIC_BUCK_GEN(VSYS22,		PMIC_QI_VSYS22_EN,	PMIC_NI_VSYS22_VOSEL, 	1200000,	1993750,6250),	
	PMIC_BUCK_GEN(VLTE,		PMIC_QI_VLTE_EN,		PMIC_NI_VLTE_VOSEL,		600000,	1393750,6250),		
};

void mtk_regulator_init(struct platform_device *dev)
{
	int i=0;
	int ret=0;
	int isEn=0;
//workaround for VMC voltage	
	if(pmic_get_register_value(PMIC_SWCID)==PMIC6328_E1_CID_CODE)
	{
		if(pmic_read_VMC_efuse()!=0)
		{
			PMICLOG("VMC voltage use E1_2 voltage table  \n");
			mtk_ldos[MT6328_POWER_LDO_VMC].pvoltages=(void *)mt6328_VMC_E1_2_voltages;
		}
		else
		{
			PMICLOG("VMC voltage use E1_1 voltage table  \n");
			mtk_ldos[MT6328_POWER_LDO_VMC].pvoltages=(void *)mt6328_VMC_E1_1_voltages;
		}
	}

//workaround for VIO18
	vio18_cal=pmic_get_register_value(PMIC_RG_VIO18_CAL);
	vmc_cal=pmic_get_register_value(PMIC_RG_VMC_CAL);
	vcamd_cal=pmic_get_register_value(PMIC_RG_VCAMD_CAL);

	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		if (mtk_ldos[i].isUsedable==1)
		{
			mtk_ldos[i].config.dev=&(dev->dev);
			mtk_ldos[i].config.init_data=&mtk_ldos[i].init_data;
			if(mtk_ldos[i].desc.n_voltages!=1)
			{
				const int *pVoltage;
				
				if(mtk_ldos[i].vol_reg!=0)
				{
					if(mtk_ldos[i].pvoltages!=NULL)
					{
						pVoltage=(const int*)mtk_ldos[i].pvoltages;
						mtk_ldos[i].init_data.constraints.max_uV=pVoltage[mtk_ldos[i].desc.n_voltages-1];
						mtk_ldos[i].init_data.constraints.min_uV=pVoltage[0];		
					}
					else
					{
						mtk_ldos[i].init_data.constraints.max_uV=(mtk_ldos[i].desc.n_voltages-1)*mtk_ldos[i].desc.uV_step+mtk_ldos[i].desc.min_uV;
						mtk_ldos[i].init_data.constraints.min_uV=mtk_ldos[i].desc.min_uV;
						PMICLOG("test man_uv:%d min_uv:%d  \n", (mtk_ldos[i].desc.n_voltages-1)*mtk_ldos[i].desc.uV_step+mtk_ldos[i].desc.min_uV,
							mtk_ldos[i].desc.min_uV);
					}
				}
				PMICLOG("min_uv:%d max_uv:%d  \n", mtk_ldos[i].init_data.constraints.min_uV,mtk_ldos[i].init_data.constraints.max_uV);
			}

			mtk_ldos[i].desc.owner=THIS_MODULE;

			mtk_ldos[i].rdev= regulator_register(&mtk_ldos[i].desc,&mtk_ldos[i].config);
			if ( IS_ERR(mtk_ldos[i].rdev) ) {                                     
			    ret = PTR_ERR(mtk_ldos[i].rdev);
			    PMICLOG("[regulator_register] failed to register %s (%d)\n", mtk_ldos[i].desc.name, ret);
			} else {
			    PMICLOG("[regulator_register] pass to register %s\n", mtk_ldos[i].desc.name);
			}
			mtk_ldos[i].reg=regulator_get(&(dev->dev), mtk_ldos[i].desc.name);
			isEn=regulator_is_enabled(mtk_ldos[i].reg); 
			if(isEn!=0)
			{
				PMICLOG("[regulator] %s is default on\n", mtk_ldos[i].desc.name);
				//ret=regulator_enable(mtk_ldos[i].reg);
			}			
		}
	}

}



void pmic6328_regulator_test(void)
{
	int i=0,j;
	int ret1,ret2;
	struct regulator *reg;
	
	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		if (mtk_ldos[i].isUsedable==1)
		{
			reg=mtk_ldos[i].reg;
			PMICLOG("[regulator enable test] %s\n", mtk_ldos[i].desc.name);
			
			ret1=regulator_enable(reg);
			ret2=regulator_is_enabled(reg);

			if(ret2==pmic_get_register_value(mtk_ldos[i].en_reg))
			{
				PMICLOG("[enable test pass] \n");
			}
			else
			{
				PMICLOG("[enable test fail] ret=%d enable=%d addr:0x%x reg:0x%x\n", ret1,ret2,
				pmu_flags_table[mtk_ldos[i].en_reg].offset,
				pmic_get_register_value(mtk_ldos[i].en_reg));
			}

			ret1=regulator_disable(reg);
			ret2=regulator_is_enabled(reg);

			if(ret2==pmic_get_register_value(mtk_ldos[i].en_reg))
			{
				PMICLOG("[disable test pass] \n");
			}
			else
			{
				PMICLOG("[disable test fail] ret=%d enable=%d addr:0x%x reg:0x%x\n", ret1,ret2,
				pmu_flags_table[mtk_ldos[i].en_reg].offset,
				pmic_get_register_value(mtk_ldos[i].en_reg));
			}

		}
	}

	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		const int *pVoltage;
	
		reg=mtk_ldos[i].reg;
		if (mtk_ldos[i].isUsedable==1)
		{
			PMICLOG("[regulator voltage test] %s voltage:%d \n", mtk_ldos[i].desc.name,mtk_ldos[i].desc.n_voltages);

			if(mtk_ldos[i].pvoltages!=NULL)
			{
				pVoltage=(const int*)mtk_ldos[i].pvoltages;
		
				for(j=0;j<mtk_ldos[i].desc.n_voltages;j++)
				{
					int rvoltage;
					regulator_set_voltage(reg, pVoltage[j], pVoltage[j]);
					rvoltage=regulator_get_voltage(reg);


					if((j==pmic_get_register_value(mtk_ldos[i].vol_reg))&& (pVoltage[j]==rvoltage))
					{
						PMICLOG("[%d:%d]:pass  set_voltage:%d  rvoltage:%d \n",j,pmic_get_register_value(mtk_ldos[i].vol_reg),
						pVoltage[j] ,rvoltage);	
						
					}
					else
					{
						PMICLOG("[%d:%d]:fail  set_voltage:%d  rvoltage:%d \n",j,pmic_get_register_value(mtk_ldos[i].vol_reg),
						pVoltage[j] ,rvoltage);						
					}		
				}
			}
		}
	}


}

int getHwVoltage(MT65XX_POWER powerId)
{
	struct regulator *reg;
	
	
	if(powerId>=ARRAY_SIZE(mtk_ldos))
		return -100;

	if(mtk_ldos[powerId].isUsedable!=TRUE)
		return -101;
	
	reg=mtk_ldos[powerId].reg;

	return regulator_get_voltage(reg);
}
EXPORT_SYMBOL(getHwVoltage);

int isHwPowerOn(MT65XX_POWER powerId)
{
	struct regulator *reg;
	
	if(powerId>=ARRAY_SIZE(mtk_ldos))
		return -100;

	if(mtk_ldos[powerId].isUsedable!=TRUE)
		return -101;
	
	reg=mtk_ldos[powerId].reg;

	return regulator_is_enabled(reg);

}
EXPORT_SYMBOL(isHwPowerOn);

bool hwPowerOn(MT65XX_POWER powerId, int powerVolt, char *mode_name)
{
	struct regulator *reg;
	int ret1,ret2;
	
	if(powerId>=ARRAY_SIZE(mtk_ldos))
		return FALSE;

	if(mtk_ldos[powerId].isUsedable!=TRUE)
		return FALSE;
	
	reg=mtk_ldos[powerId].reg;

	ret2=regulator_set_voltage(reg, powerVolt, powerVolt);

	if (ret2!=0)
	{
		PMICLOG("hwPowerOn:%s set voltage %d fail",mtk_ldos[powerId].desc.name,powerVolt);
	}
	
	ret1=regulator_enable(reg);

	if (ret1!=0)
	{
		PMICLOG("hwPowerOn:%s enable fail",mtk_ldos[powerId].desc.name);
	}

	PMICLOG("hwPowerOn:%d:%s volt:%d name:%s cnt:%d",powerId,mtk_ldos[powerId].desc.name,powerVolt,mode_name,mtk_ldos[powerId].rdev->use_count);
	return TRUE;
}
EXPORT_SYMBOL(hwPowerOn);

bool hwPowerDown(MT65XX_POWER powerId, char *mode_name)
{
	struct regulator *reg;
	int ret1;
	
	if(powerId>=ARRAY_SIZE(mtk_ldos))
		return FALSE;
	
	if(mtk_ldos[powerId].isUsedable!=TRUE)
		return FALSE;
	
	reg=mtk_ldos[powerId].reg;
	ret1=regulator_disable(reg);

	if (ret1!=0)
	{
		PMICLOG("hwPowerOn err:ret1:%d ",ret1);
	}

	PMICLOG("hwPowerDown:%d:%s name:%s cnt:%d",powerId,mtk_ldos[powerId].desc.name,mode_name,mtk_ldos[powerId].rdev->use_count);
	return TRUE;
}
EXPORT_SYMBOL(hwPowerDown);

bool hwPowerSetVoltage(MT65XX_POWER powerId, int powerVolt, char *mode_name)
{
	struct regulator *reg;
	int ret1;
	
	if(powerId>=ARRAY_SIZE(mtk_ldos))
		return FALSE;
	
	reg=mtk_ldos[powerId].reg;

	ret1=regulator_set_voltage(reg, powerVolt, powerVolt);

	if (ret1!=0)
	{
		PMICLOG("hwPowerSetVoltage:%s set voltage %d fail",mtk_ldos[powerId].desc.name,powerVolt);
	}


	PMICLOG("hwPowerSetVoltage:%d:%s name:%s cnt:%d",powerId,mtk_ldos[powerId].desc.name,mode_name,mtk_ldos[powerId].rdev->use_count);
	return TRUE;
}
EXPORT_SYMBOL(hwPowerSetVoltage);




//==============================================================================
// Low battery call back function
//==============================================================================
#define LBCB_NUM 16

#ifndef DISABLE_LOW_BATTERY_PROTECT
#define LOW_BATTERY_PROTECT
#endif

int g_lowbat_int_bottom=0;

#ifdef LOW_BATTERY_PROTECT
// ex. 3400/5400*4096
#define BAT_HV_THD   (POWER_INT0_VOLT*4096/5400) //ex: 3400mV
#define BAT_LV_1_THD (POWER_INT1_VOLT*4096/5400) //ex: 3250mV
#define BAT_LV_2_THD (POWER_INT2_VOLT*4096/5400) //ex: 3000mV

int g_low_battery_level=0;
int g_low_battery_stop=0;

struct low_battery_callback_table
{
    void *lbcb;
};

struct low_battery_callback_table lbcb_tb[] ={
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}    
};

void (*low_battery_callback)(LOW_BATTERY_LEVEL);

void register_low_battery_notify( void (*low_battery_callback)(LOW_BATTERY_LEVEL), LOW_BATTERY_PRIO prio_val )
{
    PMICLOG("[register_low_battery_notify] start\n");

    lbcb_tb[prio_val].lbcb = low_battery_callback;
    
    PMICLOG("[register_low_battery_notify] prio_val=%d\n",prio_val);
}

void exec_low_battery_callback(LOW_BATTERY_LEVEL low_battery_level) //0:no limit
{
    int i=0;

    if(g_low_battery_stop==1)
    {
        PMICLOG("[exec_low_battery_callback] g_low_battery_stop=%d\n", g_low_battery_stop);
    }
    else
    {
        for(i=0 ; i<LBCB_NUM ; i++) 
        {
            if(lbcb_tb[i].lbcb != NULL)
            {
                low_battery_callback = lbcb_tb[i].lbcb;
                low_battery_callback(low_battery_level);
                PMICLOG("[exec_low_battery_callback] prio_val=%d,low_battery=%d\n",i,low_battery_level);
            }        
        }
    }
}

void lbat_min_en_setting(int en_val)
{
	pmic_set_register_value(PMIC_AUXADC_LBAT_EN_MIN,en_val);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MIN,en_val);    
	pmic_set_register_value(PMIC_RG_INT_EN_BAT_L,en_val);    	
}

void lbat_max_en_setting(int en_val)
{
	pmic_set_register_value(PMIC_AUXADC_LBAT_EN_MAX,en_val);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_IRQ_EN_MAX,en_val);    
	pmic_set_register_value(PMIC_RG_INT_EN_BAT_H,en_val);    	
}

void low_battery_protect_init(void)
{
	//default setting
	pmic_set_register_value(PMIC_AUXADC_LBAT_DEBT_MIN,0);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_DEBT_MAX,0);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_15_0,1);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_DET_PRD_19_16,0);    

	pmic_set_register_value(PMIC_AUXADC_LBAT_VOLT_MAX,BAT_HV_THD);    
	pmic_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,BAT_LV_1_THD);    	

	lbat_min_en_setting(1);
	lbat_max_en_setting(0);

	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
	        MT6328_AUXADC_LBAT3, upmu_get_reg_value(MT6328_AUXADC_LBAT3),
	        MT6328_AUXADC_LBAT4, upmu_get_reg_value(MT6328_AUXADC_LBAT4),
	        MT6328_INT_CON0, upmu_get_reg_value(MT6328_INT_CON0)
	        );

	PMICLOG("[low_battery_protect_init] %d mV, %d mV, %d mV\n",
	    POWER_INT0_VOLT, POWER_INT1_VOLT, POWER_INT2_VOLT);
	PMICLOG("[low_battery_protect_init] Done\n");

}

#endif //#ifdef LOW_BATTERY_PROTECT

void bat_h_int_handler(void)
{
	g_lowbat_int_bottom=0;
	
	PMICLOG("[bat_h_int_handler]....\n");

	//sub-task
#ifdef LOW_BATTERY_PROTECT
	g_low_battery_level=0;
	exec_low_battery_callback(LOW_BATTERY_LEVEL_0);

#if 0
	lbat_max_en_setting(0);
	mdelay(1);
	lbat_min_en_setting(1);
#else
	pmic_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,BAT_LV_1_THD); 

	lbat_min_en_setting(0);
	lbat_max_en_setting(0);
	mdelay(1);
	lbat_min_en_setting(1);   
#endif

	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
	        MT6328_AUXADC_LBAT3, upmu_get_reg_value(MT6328_AUXADC_LBAT3),
	        MT6328_AUXADC_LBAT4, upmu_get_reg_value(MT6328_AUXADC_LBAT4),
	        MT6328_INT_CON0, upmu_get_reg_value(MT6328_INT_CON0)
	        );
#endif 

}

void bat_l_int_handler(void)
{
	PMICLOG("[bat_l_int_handler]....\n");

	//sub-task
#ifdef LOW_BATTERY_PROTECT
	g_low_battery_level++;
	if(g_low_battery_level > 2)
	g_low_battery_level = 2; 

	if(g_low_battery_level==1)      
	exec_low_battery_callback(LOW_BATTERY_LEVEL_1);
	else if(g_low_battery_level==2) 
	{ 
	exec_low_battery_callback(LOW_BATTERY_LEVEL_2);
		g_lowbat_int_bottom=1;
	}
	else                            
	PMICLOG("[bat_l_int_handler]err,g_low_battery_level=%d\n", g_low_battery_level);

#if 0
	lbat_min_en_setting(0);
	mdelay(1);
	lbat_max_en_setting(1);
#else

	pmic_set_register_value(PMIC_AUXADC_LBAT_VOLT_MIN,BAT_LV_2_THD); 

	lbat_min_en_setting(0);
	lbat_max_en_setting(0);
	mdelay(1);
	if(g_low_battery_level<2)
	{
		lbat_min_en_setting(1);
	}
	lbat_max_en_setting(1);
#endif

	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
	        MT6328_AUXADC_LBAT3, upmu_get_reg_value(MT6328_AUXADC_LBAT3),
	        MT6328_AUXADC_LBAT4, upmu_get_reg_value(MT6328_AUXADC_LBAT4),
	MT6328_INT_CON0, upmu_get_reg_value(MT6328_INT_CON0)
	);
#endif 

}

//==============================================================================
// Battery OC call back function
//==============================================================================
#define OCCB_NUM 16

#ifndef DISABLE_BATTERY_OC_PROTECT
#define BATTERY_OC_PROTECT
#endif

// ex. Ireg = 65535 - (I * 950000uA / 2 / 158.122 / CAR_TUNE_VALUE * 100)
// (950000/2/158.122)*100~=300400

#define BAT_OC_H_THD   65535-((300400*POWER_BAT_OC_CURRENT_H/1000)/CAR_TUNE_VALUE)       //ex: 4670mA
#define BAT_OC_L_THD   65535-((300400*POWER_BAT_OC_CURRENT_L/1000)/CAR_TUNE_VALUE)       //ex: 5500mA

#define BAT_OC_H_THD_RE   65535-((300400*POWER_BAT_OC_CURRENT_H_RE/1000)/CAR_TUNE_VALUE) //ex: 3400mA
#define BAT_OC_L_THD_RE   65535-((300400*POWER_BAT_OC_CURRENT_L_RE/1000)/CAR_TUNE_VALUE) //ex: 4000mA

int g_battery_oc_level=0;
int g_battery_oc_stop=0;

struct battery_oc_callback_table
{
    void *occb;
};

struct battery_oc_callback_table occb_tb[] ={
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}    
};

void (*battery_oc_callback)(BATTERY_OC_LEVEL);

void register_battery_oc_notify( void (*battery_oc_callback)(BATTERY_OC_LEVEL), BATTERY_OC_PRIO prio_val )
{
    PMICLOG("[register_battery_oc_notify] start\n");

    occb_tb[prio_val].occb = battery_oc_callback;
    
    PMICLOG("[register_battery_oc_notify] prio_val=%d\n",prio_val);
}

void exec_battery_oc_callback(BATTERY_OC_LEVEL battery_oc_level) //0:no limit
{
    int i=0;

    if(g_battery_oc_stop==1)
    {
        PMICLOG("[exec_battery_oc_callback] g_battery_oc_stop=%d\n", g_battery_oc_stop);
    }
    else
    {
        for(i=0 ; i<OCCB_NUM ; i++) 
        {
            if(occb_tb[i].occb != NULL)
            {
                battery_oc_callback = occb_tb[i].occb;
                battery_oc_callback(battery_oc_level);
                PMICLOG("[exec_battery_oc_callback] prio_val=%d,battery_oc_level=%d\n",i,battery_oc_level);
            }        
        }
    }
}

void bat_oc_h_en_setting(int en_val)
{
  pmic_set_register_value(PMIC_RG_INT_EN_FG_CUR_H,en_val); // mt6325_upmu_set_rg_int_en_fg_cur_h(en_val);   
}

void bat_oc_l_en_setting(int en_val)
{
   pmic_set_register_value(PMIC_RG_INT_EN_FG_CUR_L,en_val); //mt6325_upmu_set_rg_int_en_fg_cur_l(en_val);
}

void battery_oc_protect_init(void)
{
    pmic_set_register_value(PMIC_FG_CUR_HTH,BAT_OC_H_THD);//mt6325_upmu_set_fg_cur_hth(BAT_OC_H_THD);
    pmic_set_register_value(PMIC_FG_CUR_LTH,BAT_OC_L_THD);//mt6325_upmu_set_fg_cur_lth(BAT_OC_L_THD);

    bat_oc_h_en_setting(0);
    bat_oc_l_en_setting(1);

    PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
        MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
        MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
        );

    PMICLOG("[battery_oc_protect_init] %d mA, %d mA\n",
        POWER_BAT_OC_CURRENT_H, POWER_BAT_OC_CURRENT_L);
    PMICLOG("[battery_oc_protect_init] Done\n");  
}

void battery_oc_protect_reinit(void)
{
#ifdef BATTERY_OC_PROTECT
    pmic_set_register_value(PMIC_FG_CUR_HTH,BAT_OC_H_THD_RE);//mt6325_upmu_set_fg_cur_hth(BAT_OC_H_THD_RE);
    pmic_set_register_value(PMIC_FG_CUR_LTH,BAT_OC_L_THD_RE);//mt6325_upmu_set_fg_cur_lth(BAT_OC_L_THD_RE);

    PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
        MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
        MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
        );

    PMICLOG("[battery_oc_protect_reinit] %d mA, %d mA\n",
        POWER_BAT_OC_CURRENT_H_RE, POWER_BAT_OC_CURRENT_L_RE);
    PMICLOG("[battery_oc_protect_reinit] Done\n");  
#else
    PMICLOG("[battery_oc_protect_reinit] no define BATTERY_OC_PROTECT\n");
#endif
}

void fg_cur_h_int_handler(void)
{
    PMICLOG("[fg_cur_h_int_handler]....\n");
     
    //sub-task
#ifdef BATTERY_OC_PROTECT
    g_battery_oc_level=0;
    exec_battery_oc_callback(BATTERY_OC_LEVEL_0);
    bat_oc_h_en_setting(0);
    bat_oc_l_en_setting(0);
    mdelay(1);
    bat_oc_l_en_setting(1);
    
    PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
        MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
        MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
        );
#endif 
}

void fg_cur_l_int_handler(void)
{
	PMICLOG("[fg_cur_l_int_handler]....\n");
    
    //sub-task
#ifdef BATTERY_OC_PROTECT
    g_battery_oc_level=1;
    exec_battery_oc_callback(BATTERY_OC_LEVEL_1);       
    bat_oc_h_en_setting(0);
    bat_oc_l_en_setting(0);
    mdelay(1);
    bat_oc_h_en_setting(1);
    
    PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
        MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
        MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
        );
#endif
}

//==============================================================================
// 15% notify service
//==============================================================================
static struct hrtimer bat_percent_notify_timer;
static struct task_struct *bat_percent_notify_thread = NULL;
static kal_bool bat_percent_notify_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(bat_percent_notify_waiter);
struct wake_lock bat_percent_notify_lock;
static DEFINE_MUTEX(bat_percent_notify_mutex);

extern kal_uint32 bat_get_ui_percentage(void);

#define BPCB_NUM 16

#ifndef DISABLE_BATTERY_PERCENT_PROTECT
#define BATTERY_PERCENT_PROTECT
#endif

int g_battery_percent_level=0;
int g_battery_percent_stop=0;

#define BAT_PERCENT_LINIT 15

struct battery_percent_callback_table
{
    void *bpcb;
};

struct battery_percent_callback_table bpcb_tb[] ={
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}    
};

void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL);

void register_battery_percent_notify( void (*battery_percent_callback)(BATTERY_PERCENT_LEVEL), BATTERY_PERCENT_PRIO prio_val )
{
    PMICLOG("[register_battery_percent_notify] start\n");

    bpcb_tb[prio_val].bpcb = battery_percent_callback;
    
    PMICLOG("[register_battery_percent_notify] prio_val=%d\n",prio_val);

    if( (g_battery_percent_stop==0) && (g_battery_percent_level==1) )
    {
		#ifdef DISABLE_DLPT_FEATURE
			PMICLOG("[register_battery_percent_notify] level l happen\n");
			battery_percent_callback(BATTERY_PERCENT_LEVEL_1);
		#else
			if(prio_val == BATTERY_PERCENT_PRIO_FLASHLIGHT)
			{
				PMICLOG("[register_battery_percent_notify at DLPT] level l happen\n");
				battery_percent_callback(BATTERY_PERCENT_LEVEL_1);
			}
		#endif
    }
}

void exec_battery_percent_callback(BATTERY_PERCENT_LEVEL battery_percent_level) //0:no limit
{
    int i=0;

    if(g_battery_percent_stop==1)
    {
        PMICLOG("[exec_battery_percent_callback] g_battery_percent_stop=%d\n", g_battery_percent_stop);
    }
    else
    {
      #ifdef DISABLE_DLPT_FEATURE		
        for(i=0 ; i<BPCB_NUM ; i++) 
        {
            if(bpcb_tb[i].bpcb != NULL)
            {
                battery_percent_callback = bpcb_tb[i].bpcb;
                battery_percent_callback(battery_percent_level);
                PMICLOG("[exec_battery_percent_callback] prio_val=%d,battery_percent_level=%d\n",i,battery_percent_level);
            }        
        }
      #else
        battery_percent_callback = bpcb_tb[BATTERY_PERCENT_PRIO_FLASHLIGHT].bpcb;
        battery_percent_callback(battery_percent_level);
        PMICLOG("[exec_battery_percent_callback at DLPT] prio_val=%d,battery_percent_level=%d\n",BATTERY_PERCENT_PRIO_FLASHLIGHT,battery_percent_level);
      #endif
    }
}

int bat_percent_notify_handler(void *unused)
{
    ktime_t ktime;
    int bat_per_val=0;

    do
    {
        ktime = ktime_set(10, 0);

        wait_event_interruptible(bat_percent_notify_waiter, (bat_percent_notify_flag == KAL_TRUE));

        wake_lock(&bat_percent_notify_lock);
        mutex_lock(&bat_percent_notify_mutex);

        bat_per_val=bat_get_ui_percentage();
        
        if( (upmu_get_rgs_chrdet()==0) && (g_battery_percent_level==0) && (bat_per_val<=BAT_PERCENT_LINIT) )
        {
            g_battery_percent_level=1;
            exec_battery_percent_callback(BATTERY_PERCENT_LEVEL_1);
        }
        else if( (g_battery_percent_level==1) && (bat_per_val>BAT_PERCENT_LINIT) )
        {
            g_battery_percent_level=0;
            exec_battery_percent_callback(BATTERY_PERCENT_LEVEL_0);
        }
        else
        {
        }
        bat_percent_notify_flag = KAL_FALSE;
        
        PMICLOG("bat_per_level=%d,bat_per_val=%d\n",g_battery_percent_level,bat_per_val);

        mutex_unlock(&bat_percent_notify_mutex);
        wake_unlock(&bat_percent_notify_lock);
       
        hrtimer_start(&bat_percent_notify_timer, ktime, HRTIMER_MODE_REL);    
        
    } while (!kthread_should_stop());
    
    return 0;
}

enum hrtimer_restart bat_percent_notify_task(struct hrtimer *timer)
{
    bat_percent_notify_flag = KAL_TRUE; 
    wake_up_interruptible(&bat_percent_notify_waiter);
    PMICLOG("bat_percent_notify_task is called\n");
    
    return HRTIMER_NORESTART;
}

void bat_percent_notify_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(20, 0);
    hrtimer_init(&bat_percent_notify_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bat_percent_notify_timer.function = bat_percent_notify_task;    
    hrtimer_start(&bat_percent_notify_timer, ktime, HRTIMER_MODE_REL);

    bat_percent_notify_thread = kthread_run(bat_percent_notify_handler, 0, "bat_percent_notify_thread");
    if (IS_ERR(bat_percent_notify_thread))
    {
        PMICLOG("Failed to create bat_percent_notify_thread\n");
    }
    else
    {
        PMICLOG("Create bat_percent_notify_thread : done\n");
    }
}

//==============================================================================
// DLPT service
//==============================================================================
 
kal_uint32 ptim_bat_vol=0;
kal_int32  ptim_R_curr=0;
int ptim_imix=0;
int ptim_rac_val_avg=0;


extern kal_int32 fgauge_read_IM_current(void *data);

kal_int32 pmic_ptimretest=0;

extern void pmic_auxadc_lock(void);
extern void pmic_auxadc_unlock(void);


kal_uint32 ptim_cnt=0;



void do_ptim(kal_bool isSuspend)
{
	kal_uint32 i;
	kal_uint32 vbat_reg;

	//PMICLOG("[do_ptim] start \n");
	if(isSuspend==KAL_FALSE)
		pmic_auxadc_lock();
	//pmic_set_register_value(PMIC_RG_AUXADC_RST,1);
	//pmic_set_register_value(PMIC_RG_AUXADC_RST,0);

	upmu_set_reg_value(0x0eac,0x0006);

	pmic_set_register_value(PMIC_AUXADC_IMP_AUTORPT_PRD,6);
	pmic_set_register_value(PMIC_RG_AUXADC_SMPS_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_AUXADC_SMPS_CK_PDN_HWEN,0);

	pmic_set_register_value(PMIC_RG_AUXADC_CK_PDN_HWEN,0);
	pmic_set_register_value(PMIC_RG_AUXADC_CK_PDN,0);

	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	   
	
	//restore to initial state
	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);	   
	
	//set issue interrupt
	//pmic_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,1);
	
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_CHSEL,0);
	pmic_set_register_value(PMIC_AUXADC_IMP_AUTORPT_EN,1);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_CNT,3);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_MODE,1);

//	PMICLOG("[do_ptim] end %d %d \n",pmic_get_register_value(PMIC_RG_AUXADC_SMPS_CK_PDN),pmic_get_register_value(PMIC_RG_AUXADC_SMPS_CK_PDN_HWEN));

	while(pmic_get_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_STATUS)==0)
{
		//PMICLOG("[do_ptim] PMIC_AUXADC_IMPEDANCE_IRQ_STATUS= %d \n",pmic_get_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_STATUS));
		mdelay(1);	
	}

	//disable
	pmic_set_register_value(PMIC_AUXADC_IMP_AUTORPT_EN,0);//typo 
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_MODE,0);

	//clear irq
	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);
	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);


	if(isSuspend==KAL_FALSE)
		pmic_auxadc_unlock();
	//PMICLOG("[do_ptim2] 0xee8=0x%x  0x2c6=0x%x\n", upmu_get_reg_value(0xee8),upmu_get_reg_value(0x2c6));

	
	//pmic_set_register_value(PMIC_RG_INT_STATUS_AUXADC_IMP,1);//write 1 to clear !
	//pmic_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,0);
	
	
	vbat_reg=pmic_get_register_value(PMIC_AUXADC_ADC_OUT_IMP_AVG);
	ptim_bat_vol=(vbat_reg*3*18000)/32768;

	fgauge_read_IM_current((void *)&ptim_R_curr);
	
}


void enable_dummy_load(kal_uint32 en)
{

	if(en==1)
	{
	    //Enable dummy load--------------------------------------------------
        //mt6325_upmu_set_rg_g_drv_2m_ck_pdn(0);
        //mt6325_upmu_set_rg_drv_32k_ck_pdn(0);

		//upmu_set_reg_value(0x23c,0xfeb0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK2_CK_PDN,0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK3_CK_PDN,0);

		//upmu_set_reg_value(0x25a,0x8a00);
		pmic_set_register_value(PMIC_RG_DRV_ISINK2_CK_CKSEL,0);
		pmic_set_register_value(PMIC_RG_DRV_ISINK3_CK_CKSEL,0);

			//upmu_set_reg_value(0x82a,0x0c00);
			//upmu_set_reg_value(0x81c,0x7000);
			pmic_set_register_value(PMIC_ISINK_CH2_STEP,0xc);
			
			//upmu_set_reg_value(0x81e,0x7000);
			pmic_set_register_value(PMIC_ISINK_CH3_STEP,0xc);
				
			//upmu_set_reg_value(0x820,0x0300);
			pmic_set_register_value(PMIC_RG_ISINK2_DOUBLE_EN,1);
			pmic_set_register_value(PMIC_RG_ISINK3_DOUBLE_EN,1);
		
	
		//upmu_set_reg_value(0x828,0x0ccc);
		pmic_set_register_value(PMIC_ISINK_CH2_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH3_EN,1);
		pmic_set_register_value(PMIC_ISINK_CHOP2_EN,1);
		pmic_set_register_value(PMIC_ISINK_CHOP3_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH2_BIAS_EN,1);
		pmic_set_register_value(PMIC_ISINK_CH3_BIAS_EN,1);
		//pmic_set_register_value(PMIC_RG_VIBR_EN,1);

		//PMICLOG("[enable dummy load]\n");
	}
	else
	{
		//upmu_set_reg_value(0x828,0x0cc0);
		pmic_set_register_value(PMIC_ISINK_CH2_EN,0);
		pmic_set_register_value(PMIC_ISINK_CH3_EN,0);	
		//pmic_set_register_value(PMIC_RG_VIBR_EN,0);
		//PMICLOG("[disable dummy load]\n");
	}
	
}




#ifndef DISABLE_DLPT_FEATURE
#define DLPT_FEATURE_SUPPORT
#endif

#ifdef DLPT_FEATURE_SUPPORT
static struct hrtimer dlpt_notify_timer;
static struct task_struct *dlpt_notify_thread = NULL;
static kal_bool dlpt_notify_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(dlpt_notify_waiter);
struct wake_lock dlpt_notify_lock;
static DEFINE_MUTEX(dlpt_notify_mutex);

#define DLPT_NUM 16


int g_dlpt_stop=0;
unsigned int g_dlpt_val=0;

int g_dlpt_start=0;


int g_imix_val=0;
int g_imix_val_pre=0;
int g_low_per_timer=0;
int g_low_per_timeout_val=60;


int g_lbatInt1=POWER_INT2_VOLT*10;

struct dlpt_callback_table
{
    void *dlpt_cb;
};

struct dlpt_callback_table dlpt_cb_tb[] ={
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},
    {NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL},{NULL}    
};
  
void (*dlpt_callback)(unsigned int);

void register_dlpt_notify( void (*dlpt_callback)(unsigned int), DLPT_PRIO prio_val)
{
    PMICLOG("[register_dlpt_notify] start\n");

    dlpt_cb_tb[prio_val].dlpt_cb = dlpt_callback;
    
    PMICLOG("[register_dlpt_notify] prio_val=%d\n",prio_val);

    if( (g_dlpt_stop==0) && (g_dlpt_val!=0) )
    {
        PMICLOG("[register_dlpt_notify] dlpt happen\n");
        dlpt_callback(g_dlpt_val);        
    }
}

void exec_dlpt_callback(unsigned int dlpt_val)
{
    int i=0;

    g_dlpt_val = dlpt_val;

    if(g_dlpt_stop==1)
    {
        PMICLOG("[exec_dlpt_callback] g_dlpt_stop=%d\n", g_dlpt_stop);
    }
    else
    {
        for(i=0 ; i<DLPT_NUM ; i++) 
        {
            if(dlpt_cb_tb[i].dlpt_cb != NULL)
            {
                dlpt_callback = dlpt_cb_tb[i].dlpt_cb;                
                dlpt_callback(g_dlpt_val);
                PMICLOG("[exec_dlpt_callback] g_dlpt_val=%d\n", g_dlpt_val);
            }        
        }
    }
}

extern kal_uint32 bat_get_ui_percentage(void);
extern kal_int32 fgauge_read_v_by_d(int d_val);
extern kal_int32 fgauge_read_r_bat_by_v(kal_int32 voltage);

/*
int get_dlpt_iavg(int is_use_zcv)
{
    int bat_cap_val=0;
    int zcv_val=0;        
    int vsys_min_2_val=POWER_INT2_VOLT;
    int rbat_val=0;
    int rdc_val=0;    
    int iavg_val=0;

    bat_cap_val=bat_get_ui_percentage();

    if(is_use_zcv==1)
    {
        zcv_val=fgauge_read_v_by_d(100-bat_cap_val);
    }
    else
    {
        #if defined(SWCHR_POWER_PATH)
        zcv_val=PMIC_IMM_GetOneChannelValue(MT6328_AUX_ISENSE_AP,5,1);	
        #else
        zcv_val=PMIC_IMM_GetOneChannelValue(MT6328_AUX_BATSNS_AP,5,1);
        #endif
    }
    rbat_val=fgauge_read_r_bat_by_v(zcv_val);
    rdc_val=CUST_R_FG_OFFSET+R_FG_VALUE+rbat_val;

    if(rdc_val==0) 
        rdc_val=1;    
    iavg_val=((zcv_val-vsys_min_2_val)*1000)/rdc_val; // ((mV)*1000)/m-ohm
    
    PMICLOG("[dlpt] SOC=%d,ZCV=%d,vsys_min_2=%d,rpcb=%d,rfg=%d,rbat=%d,rdc=%d,iavg=%d(mA),is_use_zcv=%d\n", 
        bat_cap_val,zcv_val,vsys_min_2_val,CUST_R_FG_OFFSET,R_FG_VALUE,rbat_val,rdc_val,iavg_val,is_use_zcv 
        );
    PMICLOG("[dlpt_Iavg] %d,%d,%d,%d,%d,%d,%d,%d,%d\n", 
        bat_cap_val,zcv_val,vsys_min_2_val,CUST_R_FG_OFFSET,R_FG_VALUE,rbat_val,rdc_val,iavg_val,is_use_zcv
        );
    return iavg_val;
}

int get_real_volt(int val) //0.1mV
{
    int ret=0;
    ret = val&0x7FFF; 
    ret = (ret*4*1800*10)/32768; 
    return ret;
}

int get_real_curr(int val) //0.1mA
{
    int ret=0;

    if( val > 32767 ) // > 0x8000
    {
        ret = val-65535;
        ret = ret-(ret*2);
    }
    else
    {
        ret = val;
    }
    ret = ret*158122;    
    do_div(ret, 100000);
    ret = (ret*20)/R_FG_VALUE;
    ret = ((ret*CAR_TUNE_VALUE)/100);

    return ret;
}
*/
int get_rac_val(void)
{

    int volt_1=0;
    int volt_2=0;
    int curr_1=0;
    int curr_2=0;
    int rac_cal=0;
    int ret=0;
    kal_bool retry_state = KAL_FALSE;
	int retry_count=0;

    do
	{        
        //adc and fg--------------------------------------------------------
		do_ptim(KAL_TRUE);

		pmic_spm_crit2("[1,Trigger ADC PTIM mode] volt1=%d, curr_1=%d\n", ptim_bat_vol, ptim_R_curr);
		volt_1=ptim_bat_vol;
		curr_1=ptim_R_curr;
	
        pmic_spm_crit2("[2,enable dummy load]");
        enable_dummy_load(1);
        mdelay(50);
		//Wait --------------------------------------------------------------
        
        //adc and fg--------------------------------------------------------
		do_ptim(KAL_TRUE);

		pmic_spm_crit2("[3,Trigger ADC PTIM mode again] volt2=%d, curr_2=%d\n", ptim_bat_vol, ptim_R_curr);
		volt_2=ptim_bat_vol;
		curr_2=ptim_R_curr;

        //Disable dummy load-------------------------------------------------
        enable_dummy_load(0);

        //Calculate Rac------------------------------------------------------
        if( (curr_2-curr_1) >= 700 && (curr_2-curr_1) <= 1200 && (volt_1-volt_2)>=80 ) //40.0mA
        {
            rac_cal=((volt_1-volt_2)*1000)/(curr_2-curr_1); //m-ohm

            if(rac_cal<0) 
			{	
				ret = (rac_cal-(rac_cal*2))*1;
        }
            else
        {
				ret = rac_cal*1;
            }
        }
        else if( (curr_1-curr_2) >= 700 && (curr_2-curr_1) <= 1200 & (volt_2-volt_1)>=80 ) //40.0mA
        {
            rac_cal=((volt_2-volt_1)*1000)/(curr_1-curr_2); //m-ohm

            if(rac_cal<0) 
			{
				ret = (rac_cal-(rac_cal*2))*1;
            }
            else
            {
				ret = rac_cal*1;
            }
        }
        else
        {
            ret=-1;
            pmic_spm_crit2("[4,Calculate Rac] bypass due to (curr_x-curr_y) < 40mA\n");
        }  

        pmic_spm_crit2("[5,Calculate Rac] volt_1=%d,volt_2=%d,curr_1=%d,curr_2=%d,rac_cal=%d,ret=%d,retry_count=%d\n",
            volt_1,volt_2,curr_1,curr_2,rac_cal,ret,retry_count);

        pmic_spm_crit2("[6,Calculate Rac] %d,%d,%d,%d,%d,%d,%d\n",
            volt_1,volt_2,curr_1,curr_2,rac_cal,ret,retry_count);

    
		//------------------------
		retry_count++;
        
		if((retry_count < 3) && (ret == -1)) retry_state = KAL_TRUE;
        else                                 retry_state = KAL_FALSE;

	}while(retry_state == KAL_TRUE);
      
    return ret;
}

extern PMU_ChargerStruct BMT_status;

int get_dlpt_imix_spm(void)
{
    int rac_val[5],rac_val_avg;
	int volt[5],curr[5],volt_avg=0,curr_avg=0;
	int imix;
	int i;
	static kal_uint32 pre_ui_soc=101;
	kal_uint32 ui_soc;

	ui_soc=bat_get_ui_percentage();

	if(ui_soc!=pre_ui_soc)
	{
		pre_ui_soc=ui_soc;
	}
	else
	{
		pmic_spm_crit2("[dlpt_R] pre_SOC=%d SOC=%d skip \n",pre_ui_soc,ui_soc);
		return 0;
	}
	

	for(i=0;i<2;i++)
	{	
		rac_val[i]=get_rac_val();
		if(rac_val[i]==-1)
        return -1;
	}

    //rac_val_avg=rac_val[0]+rac_val[1]+rac_val[2]+rac_val[3]+rac_val[4];	
	//rac_val_avg=rac_val_avg/5;
	//PMICLOG("[dlpt_R] %d,%d,%d,%d,%d %d\n",rac_val[0],rac_val[1],rac_val[2],rac_val[3],rac_val[4],rac_val_avg);
	
    rac_val_avg=rac_val[0]+rac_val[1];	
	rac_val_avg=rac_val_avg/2;
	pmic_spm_crit2("[dlpt_R] %d,%d,%d\n",rac_val[0],rac_val[1],rac_val_avg);

	if(rac_val_avg>100)
		ptim_rac_val_avg=rac_val_avg;

/*
	for(i=0;i<5;i++)
	{		 
		do_ptim();

		volt[i]=ptim_bat_vol;
		curr[i]=ptim_R_curr;
		volt_avg+=ptim_bat_vol;
		curr_avg+=ptim_R_curr;
	}
		
	volt_avg=volt_avg/5;
	curr_avg=curr_avg/5;

	imix=curr_avg+(volt_avg-g_lbatInt1)*1000/ptim_rac_val_avg;

		pmic_spm_crit2("[dlpt_Imix] %d,%d,%d,%d,%d,%d,%d\n",volt_avg,curr_avg,g_lbatInt1,ptim_rac_val_avg,imix,BMT_status.SOC,bat_get_ui_percentage());

	ptim_imix=imix;
*/
	return 0;

		}


int get_dlpt_imix(void)
{
    int rac_val[5],rac_val_avg;
	int volt[5],curr[5],volt_avg=0,curr_avg=0;
	int imix;
	int i;

	for(i=0;i<5;i++)
	{		 
		//adc and fg--------------------------------------------------------
		do_ptim(KAL_FALSE);

		volt[i]=ptim_bat_vol;
		curr[i]=ptim_R_curr;
		volt_avg+=ptim_bat_vol;
		curr_avg+=ptim_R_curr;
		
	}

	volt_avg=volt_avg/5;
	curr_avg=curr_avg/5;
	
	imix=(curr_avg+(volt_avg-g_lbatInt1)*1000/ptim_rac_val_avg)/10;

		PMICLOG("[get_dlpt_imix] %d,%d,%d,%d,%d,%d,%d\n",volt_avg,curr_avg,g_lbatInt1,ptim_rac_val_avg,imix,BMT_status.SOC,bat_get_ui_percentage());

	ptim_imix=imix;

	return ptim_imix;

}



int get_dlpt_imix_charging(void)
{

    int zcv_val=0;        
    int vsys_min_1_val=POWER_INT2_VOLT;
    int imix_val=0;
#if defined(SWCHR_POWER_PATH)
	zcv_val=PMIC_IMM_GetOneChannelValue(MT6328_AUX_ISENSE_AP,5,1);	
#else
	zcv_val=PMIC_IMM_GetOneChannelValue(MT6328_AUX_BATSNS_AP,5,1);
#endif

	imix_val=(zcv_val-vsys_min_1_val)*1000/ptim_rac_val_avg*9/10;
    PMICLOG("[dlpt] get_dlpt_imix_charging %d %d %d %d\n",
        imix_val,zcv_val,vsys_min_1_val,ptim_rac_val_avg);

    return imix_val;
}


int dlpt_check_power_off(void)
{   
    int ret=0;

    ret=0;
    if(g_dlpt_start==0)
    {
        PMICLOG("[dlpt_check_power_off] not start\n");        
    }
    else
    {
        if (!upmu_get_rgs_chrdet())
        {
            if(g_low_battery_level==2 && g_lowbat_int_bottom==1) {
                ret=1;
            } else {
                ret=0;
            }

            PMICLOG("[dlpt_check_power_off] ptim_imix=%d,POWEROFF_BAT_CURRENT=%d,g_low_battery_level=%d,ret=%d,g_lowbat_int_bottom=%d\n", 
                ptim_imix, POWEROFF_BAT_CURRENT, g_low_battery_level, ret, g_lowbat_int_bottom
                );
        }
    }

    return ret;    
}



int dlpt_notify_handler(void *unused)
{
    ktime_t ktime;
    int pre_ui_soc=0;
    int cur_ui_soc=0;
    int diff_ui_soc=1;
    pre_ui_soc=bat_get_ui_percentage();
    cur_ui_soc=pre_ui_soc;
    
    do
    {

//		if(BMT_status.bat_vol>4000)
//		{
//			ktime = ktime_set(20, 0);
//		}
//		else
//		{
        ktime = ktime_set(10, 0);
//		}
	
        

        wait_event_interruptible(dlpt_notify_waiter, (dlpt_notify_flag == KAL_TRUE));

        wake_lock(&dlpt_notify_lock);
        mutex_lock(&dlpt_notify_mutex);
        //---------------------------------

        cur_ui_soc=bat_get_ui_percentage();

        if(cur_ui_soc <= 1)
        {
            g_low_per_timer+=10;
            if(g_low_per_timer > g_low_per_timeout_val)
            g_low_per_timer=0;
            PMICLOG("[DLPT] g_low_per_timer=%d,g_low_per_timeout_val=%d\n", g_low_per_timer, g_low_per_timeout_val);
        }
        else
        {
            g_low_per_timer=0;
        }
		
		PMICLOG("[dlpt_notify_handler] %d %d %d %d %d\n",pre_ui_soc,cur_ui_soc,g_imix_val,g_low_per_timer,g_low_per_timeout_val);
/*
        if( ((pre_ui_soc-cur_ui_soc)>=diff_ui_soc) || 
            ((cur_ui_soc-pre_ui_soc)>=diff_ui_soc) ||
            (g_imix_val == 0)                      ||
            (g_imix_val == -1)                     ||
            ( (cur_ui_soc <= 1) && (g_low_per_timer >= g_low_per_timeout_val) )

          )
*/
       {   
        
            PMICLOG("[DLPT] is running \n");
			if(ptim_rac_val_avg==0)
			{
				 PMICLOG("[DLPT] ptim_rac_val_avg=0 , skip\n");
			}
			else
        {   
            if (upmu_get_rgs_chrdet())
            {   
	                g_imix_val=get_dlpt_imix_charging();
            }
            else
            {
	                g_imix_val=get_dlpt_imix();
                
/*
                //sync value
	                if(g_imix_val != -1)
                {
	                    if(g_imix_val_pre <= 0)
                    {
	                        g_imix_val_pre=g_imix_val;
                    }
	                    if(g_imix_val > g_imix_val_pre)
                    {
	                        PMICLOG("[DLPT] g_imix_val=%d,g_imix_val_pre=%d\n", g_imix_val, g_imix_val_pre);
	                        g_imix_val=g_imix_val_pre;
                    }
                    else
                    {
	                        g_imix_val_pre=g_imix_val;
                    }
                }
*/
            }
            
            //Notify
	            if(g_imix_val >= 1)
            {
	                if(g_imix_val > IMAX_MAX_VALUE)
                {                    
	                    g_imix_val = IMAX_MAX_VALUE;
                }
	                exec_dlpt_callback(g_imix_val);                
            }
            else
            {
                exec_dlpt_callback(1);
				PMICLOG("[DLPT] return 1\n");
            }

            pre_ui_soc=cur_ui_soc;

	            PMICLOG("[DLPT_final] %d,%d,%d,%d,%d,%d\n", 
					g_imix_val,g_imix_val_pre,pre_ui_soc,cur_ui_soc,diff_ui_soc, IMAX_MAX_VALUE);			
			}

        }
        
        g_dlpt_start=1;
        dlpt_notify_flag = KAL_FALSE;

        //---------------------------------
        mutex_unlock(&dlpt_notify_mutex);
        wake_unlock(&dlpt_notify_lock);
       
        hrtimer_start(&dlpt_notify_timer, ktime, HRTIMER_MODE_REL);    
        
    } while (!kthread_should_stop());
    
    return 0;
}

enum hrtimer_restart dlpt_notify_task(struct hrtimer *timer)
{
    dlpt_notify_flag = KAL_TRUE; 
    wake_up_interruptible(&dlpt_notify_waiter);
    PMICLOG("dlpt_notify_task is called\n");
    
    return HRTIMER_NORESTART;
}

int get_system_loading_ma(void)
{
    int fg_val=0;
    
    if(g_dlpt_start==0) {    
        PMICLOG("get_system_loading_ma not ready\n");
    } else {
        fg_val = battery_meter_get_battery_current();
        fg_val = fg_val/10;
        if(battery_meter_get_battery_current_sign()==1)
        {
            fg_val = 0-fg_val; // charging
        }        
        PMICLOG("[get_system_loading_ma] fg_val=%d\n", fg_val);
    }
    
    return fg_val;
}


void dlpt_notify_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(30, 0);
    hrtimer_init(&dlpt_notify_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    dlpt_notify_timer.function = dlpt_notify_task;    
    hrtimer_start(&dlpt_notify_timer, ktime, HRTIMER_MODE_REL);

    dlpt_notify_thread = kthread_run(dlpt_notify_handler, 0, "dlpt_notify_thread");
    if (IS_ERR(dlpt_notify_thread))
    {
        PMICLOG("Failed to create dlpt_notify_thread\n");
    }
    else
    {
        PMICLOG("Create dlpt_notify_thread : done\n");
    }

pmic_set_register_value(PMIC_RG_UVLO_VTHL,0);

    //re-init UVLO volt
    switch(POWER_UVLO_VOLT_LEVEL){       
        case 2500: pmic_set_register_value(PMIC_RG_UVLO_VTHL,0); break;
        case 2550: pmic_set_register_value(PMIC_RG_UVLO_VTHL,1); break;
        case 2600: pmic_set_register_value(PMIC_RG_UVLO_VTHL,2); break;
        case 2650: pmic_set_register_value(PMIC_RG_UVLO_VTHL,3); break;
        case 2700: pmic_set_register_value(PMIC_RG_UVLO_VTHL,4);break;
        case 2750: pmic_set_register_value(PMIC_RG_UVLO_VTHL,5); break;
        case 2800: pmic_set_register_value(PMIC_RG_UVLO_VTHL,6); break;
        case 2850: pmic_set_register_value(PMIC_RG_UVLO_VTHL,7); break;
        case 2900: pmic_set_register_value(PMIC_RG_UVLO_VTHL,8); break;        
        default:
            PMICLOG("Invalid value(%d)\n", POWER_UVLO_VOLT_LEVEL);
            break;
    }    
    PMICLOG("POWER_UVLO_VOLT_LEVEL=%d, [0x%x]=0x%x\n", 
        POWER_UVLO_VOLT_LEVEL, MT6328_CHR_CON17, upmu_get_reg_value(MT6328_CHR_CON17));
}

#else
int get_dlpt_imix_spm(void)
{}


#endif //#ifdef DLPT_FEATURE_SUPPORT

//==============================================================================
// interrupt Setting 
//==============================================================================
static struct pmic_interrupt_bit interrupt_status0[] = {
	PMIC_S_INT_GEN(RG_INT_STATUS_PWRKEY),
	PMIC_S_INT_GEN(RG_INT_STATUS_HOMEKEY),
	PMIC_S_INT_GEN(RG_INT_STATUS_PWRKEY_R),
	PMIC_S_INT_GEN(RG_INT_STATUS_HOMEKEY_R),
	PMIC_S_INT_GEN(RG_INT_STATUS_THR_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_THR_L),
	PMIC_S_INT_GEN(RG_INT_STATUS_BAT_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_BAT_L),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(RG_INT_STATUS_RTC),
	PMIC_S_INT_GEN(RG_INT_STATUS_AUDIO),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(RG_INT_STATUS_ACCDET),
	PMIC_S_INT_GEN(RG_INT_STATUS_ACCDET_EINT),
	PMIC_S_INT_GEN(RG_INT_STATUS_ACCDET_NEGV),
	PMIC_S_INT_GEN(RG_INT_STATUS_NI_LBAT_INT),
};

static struct pmic_interrupt_bit interrupt_status1[] = {
	PMIC_S_INT_GEN(RG_INT_STATUS_VPROC_OC),
	PMIC_S_INT_GEN(RG_INT_STATUS_VSYS_OC),
	PMIC_S_INT_GEN(RG_INT_STATUS_VLTE_OC),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(RG_INT_STATUS_VCORE_OC),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(NO_USE),
	PMIC_S_INT_GEN(RG_INT_STATUS_VPA_OC),
	PMIC_S_INT_GEN(RG_INT_STATUS_LDO_OC),
	PMIC_S_INT_GEN(RG_INT_STATUS_BAT2_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_BAT2_L),
	PMIC_S_INT_GEN(RG_INT_STATUS_VISMPS0_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_VISMPS0_L),
	PMIC_S_INT_GEN(RG_INT_STATUS_AUXADC_IMP),
	PMIC_S_INT_GEN(NO_USE),
};

static struct pmic_interrupt_bit interrupt_status2[] = {
	PMIC_S_INT_GEN(RG_INT_STATUS_OV),
	PMIC_S_INT_GEN(RG_INT_STATUS_BVALID_DET),
	PMIC_S_INT_GEN(RG_INT_STATUS_VBATON_HV),
	PMIC_S_INT_GEN(RG_INT_STATUS_VBATON_UNDET),
	PMIC_S_INT_GEN(RG_INT_STATUS_WATCHDOG),
	PMIC_S_INT_GEN(RG_INT_STATUS_PCHR_CM_VDEC),
	PMIC_S_INT_GEN(RG_INT_STATUS_CHRDET),
	PMIC_S_INT_GEN(RG_INT_STATUS_PCHR_CM_VINC),
	PMIC_S_INT_GEN(RG_INT_STATUS_FG_BAT_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_FG_BAT_L),
	PMIC_S_INT_GEN(RG_INT_STATUS_FG_CUR_H),
	PMIC_S_INT_GEN(RG_INT_STATUS_FG_CUR_L),
	PMIC_S_INT_GEN(RG_INT_STATUS_FG_ZCV),
	PMIC_S_INT_GEN(RG_INT_STATUS_SPKL_D),
	PMIC_S_INT_GEN(RG_INT_STATUS_SPKL_AB),
	PMIC_S_INT_GEN(NO_USE),
};

static struct pmic_interrupts interrupts[] = {
	PMIC_M_INTS_GEN(MT6328_INT_STATUS0,MT6328_INT_CON0,MT6328_INT_CON0_SET,MT6328_INT_CON0_CLR,interrupt_status0),
	PMIC_M_INTS_GEN(MT6328_INT_STATUS1,MT6328_INT_CON1,MT6328_INT_CON1_SET,MT6328_INT_CON1_CLR,interrupt_status1),
	PMIC_M_INTS_GEN(MT6328_INT_STATUS2,MT6328_INT_CON2,MT6328_INT_CON2_SET,MT6328_INT_CON2_CLR,interrupt_status2),	
};



extern void kpd_pwrkey_pmic_handler(unsigned long pressed);
void pwrkey_int_handler(void)
{

	PMICLOG("[pwrkey_int_handler] Press pwrkey %d\n",pmic_get_register_value(PMIC_PWRKEY_DEB));

#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
	{
	    timer_pre = sched_clock();
	}
#endif
#if defined(CONFIG_MTK_FPGA)
#else
	kpd_pwrkey_pmic_handler(0x1);
#endif
}

void pwrkey_int_handler_r(void)
{
	PMICLOG("[pwrkey_int_handler_r] Release pwrkey %d\n",pmic_get_register_value(PMIC_PWRKEY_DEB));
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && timer_pre != 0)
	{
	        timer_pos = sched_clock();
	        if(timer_pos - timer_pre >= LONG_PWRKEY_PRESS_TIME)
	        {
	            long_pwrkey_press = true;
	        }
	        PMICLOG("[pmic_thread_kthread] timer_pos = %ld, timer_pre = %ld, timer_pos-timer_pre = %ld, long_pwrkey_press = %d\r\n",timer_pos, timer_pre, timer_pos-timer_pre, long_pwrkey_press);
	        if(long_pwrkey_press)   //500ms
	        {
	            PMICLOG("[pmic_thread_kthread] Power Key Pressed during kernel power off charging, reboot OS\r\n");
	            //arch_reset(0, NULL);
	        }
	}        
#endif
#if defined(CONFIG_MTK_FPGA)
#else
	kpd_pwrkey_pmic_handler(0x0);
#endif
}

//==============================================================================
// PMIC Interrupt callback
//==============================================================================
extern void kpd_pmic_rstkey_handler(unsigned long pressed);
void homekey_int_handler(void)
{
	PMICLOG("[homekey_int_handler] Press homekey %d \n",pmic_get_register_value(PMIC_HOMEKEY_DEB));
#if defined(CONFIG_MTK_FPGA)
#else
	kpd_pmic_rstkey_handler(0x1);   
#endif
}

void homekey_int_handler_r(void)
{
	PMICLOG("[homekey_int_handler_r] Release homekey %d \n",pmic_get_register_value(PMIC_HOMEKEY_DEB));
#if defined(CONFIG_MTK_FPGA)
#else
	kpd_pmic_rstkey_handler(0x0);
#endif
}

void chrdet_int_handler(void)
{
	PMICLOG("[chrdet_int_handler]CHRDET status = %d....\n",pmic_get_register_value(PMIC_RGS_CHRDET));

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (!upmu_get_rgs_chrdet())
	{
	    int boot_mode = 0;
	    boot_mode = get_boot_mode();
	    
	    if(boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || boot_mode == LOW_POWER_OFF_CHARGING_BOOT || boot_mode == META_BOOT || boot_mode == ADVMETA_BOOT)
	    {
	        PMICLOG("[chrdet_int_handler] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
	        mt_power_off();
	    }
	}
#endif
	pmic_set_register_value(PMIC_RG_USBDL_RST,1);
	do_chrdet_int_task();
}

void auxadc_imp_int_handler_r(void)
{
	PMICLOG("auxadc_imp_int_handler_r() =%d \n",pmic_get_register_value(PMIC_AUXADC_ADC_OUT_IMP));
	//clear IRQ
	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,1);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,1);	

	//restore to initial state
	pmic_set_register_value(PMIC_AUXADC_CLR_IMP_CNT_STOP,0);
	pmic_set_register_value(PMIC_AUXADC_IMPEDANCE_IRQ_CLR,0);		

	//turn off interrupt
	pmic_set_register_value(PMIC_RG_INT_EN_AUXADC_IMP,0);

}

//==============================================================================
// PMIC Interrupt service
//==============================================================================
static DEFINE_MUTEX(pmic_mutex);
static struct task_struct *pmic_thread_handle = NULL;
struct wake_lock pmicThread_lock;

void wake_up_pmic(void)
{
    PMICLOG("[wake_up_pmic]\r\n");
    wake_up_process(pmic_thread_handle);
    wake_lock(&pmicThread_lock);    
}
EXPORT_SYMBOL(wake_up_pmic);

void mt_pmic_eint_irq(void)
{
    PMICLOG("[mt_pmic_eint_irq] receive interrupt\n");
    wake_up_pmic();
    return ;
}

void pmic_enable_interrupt(kal_uint32 intNo,kal_uint32 en,char *str)
{
	kal_uint32 shift,no;
	shift=intNo/PMIC_INT_WIDTH;
	no=intNo%PMIC_INT_WIDTH;

	if(shift >=ARRAY_SIZE(interrupts))
	{
		PMICLOG("[pmic_enable_interrupt] fail intno=%d \r\n",intNo);
		return;
	}

 	PMICLOG("[pmic_enable_interrupt] intno=%d en=%d str=%s shf=%d no=%d [0x%x]=0x%x\r\n",intNo,en,str,shift,no
		,interrupts[shift].en,upmu_get_reg_value(interrupts[shift].en));

	if(en==1)
	{
		pmic_config_interface(interrupts[shift].set,0x1,0x1,no);  
	}
	else if(en==0)
	{
		pmic_config_interface(interrupts[shift].clear,0x1,0x1,no);  
	}

	PMICLOG("[pmic_enable_interrupt] after [0x%x]=0x%x\r\n",
		interrupts[shift].en,upmu_get_reg_value(interrupts[shift].en));	
	
} 

void pmic_register_interrupt_callback(kal_uint32 intNo,void (EINT_FUNC_PTR)(void))
{
	kal_uint32 shift,no;
	shift=intNo/PMIC_INT_WIDTH;
	no=intNo%PMIC_INT_WIDTH;

	if(shift >=ARRAY_SIZE(interrupts))
	{
		PMICLOG("[pmic_register_interrupt_callback] fail intno=%d \r\n",intNo);
		return;
	}

	PMICLOG("[pmic_register_interrupt_callback] intno=%d \r\n",intNo);

	interrupts[shift].interrupts[no].callback=EINT_FUNC_PTR;

}

void PMIC_EINT_SETTING(void)
{
	upmu_set_reg_value(MT6328_INT_CON0, 0);	
       upmu_set_reg_value(MT6328_INT_CON1, 0);
       upmu_set_reg_value(MT6328_INT_CON2, 0);

	//enable pwrkey/homekey interrupt
	upmu_set_reg_value(MT6328_INT_CON0_SET, 0xf);

	//for all interrupt events, turn on interrupt module clock
	pmic_set_register_value(PMIC_RG_INTRP_CK_PDN,0);    

	//For BUCK OC related interrupt, please turn on pwmoc_6m_ck (6MHz)
	pmic_set_register_value(PMIC_RG_PWMOC_6M_CK_PDN,0);    

	pmic_register_interrupt_callback(0,pwrkey_int_handler);
	pmic_register_interrupt_callback(1,homekey_int_handler);
	pmic_register_interrupt_callback(2,pwrkey_int_handler_r);
	pmic_register_interrupt_callback(3,homekey_int_handler_r);	

	pmic_register_interrupt_callback(6,bat_h_int_handler);
	pmic_register_interrupt_callback(7,bat_l_int_handler);

	pmic_register_interrupt_callback(38,chrdet_int_handler);

	pmic_register_interrupt_callback(42,fg_cur_h_int_handler);
	pmic_register_interrupt_callback(43,fg_cur_l_int_handler);	

	pmic_enable_interrupt(0,1,"PMIC");
	pmic_enable_interrupt(1,1,"PMIC");
	pmic_enable_interrupt(2,1,"PMIC");
	pmic_enable_interrupt(3,1,"PMIC");

	#ifdef LOW_BATTERY_PROTECT
	// move to lbat_xxx_en_setting
	#else
	pmic_enable_interrupt(6,1,"PMIC");
	pmic_enable_interrupt(7,1,"PMIC");
	#endif

	//pmic_enable_interrupt(30,1,"PMIC");

	pmic_enable_interrupt(38,1,"PMIC");

	#ifdef BATTERY_OC_PROTECT
	// move to bat_oc_x_en_setting
	#else
	pmic_enable_interrupt(42,1,"PMIC");
	pmic_enable_interrupt(43,1,"PMIC");
	#endif

	//mt_eint_set_hw_debounce(g_eint_pmic_num, g_cust_eint_mt_pmic_debounce_cn);    
	mt_eint_registration(g_eint_pmic_num,g_cust_eint_mt_pmic_type,mt_pmic_eint_irq,0);
	mt_eint_unmask(g_eint_pmic_num);    

	PMICLOG("[CUST_EINT] CUST_EINT_MT_PMIC_MT6325_NUM=%d\n", g_eint_pmic_num);
	PMICLOG("[CUST_EINT] CUST_EINT_PMIC_DEBOUNCE_CN=%d\n", g_cust_eint_mt_pmic_debounce_cn);
	PMICLOG("[CUST_EINT] CUST_EINT_PMIC_TYPE=%d\n", g_cust_eint_mt_pmic_type);
	PMICLOG("[CUST_EINT] CUST_EINT_PMIC_DEBOUNCE_EN=%d\n", g_cust_eint_mt_pmic_debounce_en);

}

static void pmic_int_handler(void)
{
	kal_uint8 i,j;
	kal_uint32 ret;

	for (i = 0; i < ARRAY_SIZE(interrupts); i++) 
	{
		kal_uint32 int_status_val=0;
		int_status_val=upmu_get_reg_value(interrupts[i].address);
    		PMICLOG("[PMIC_INT] addr[0x%x]=0x%x\n", interrupts[i].address,int_status_val);

		for(j=0;j<PMIC_INT_WIDTH;j++)
		{
			if((int_status_val)&(1<<j))
			{
				PMICLOG("[PMIC_INT][%s]  \n", interrupts[i].interrupts[j].name);
				if (interrupts[i].interrupts[j].callback!=NULL)
				{
					interrupts[i].interrupts[j].callback();
					interrupts[i].interrupts[j].times++;
				}
				ret=pmic_config_interface(interrupts[i].address,0x1,0x1,j);    
			}
		}
	}	
}


int pmic_rdy=0,usb_rdy=0;
void pmic_enable_charger_detection_int(int x)
{
	
	if(x==0)
	{
		pmic_rdy=1;
		PMICLOG("[pmic_enable_charger_detection_int] PMIC\n");
	}
	else if (x==1)
	{
		usb_rdy=1;
		PMICLOG("[pmic_enable_charger_detection_int] USB\n");
	}


	PMICLOG("[pmic_enable_charger_detection_int] pmic_rdy=%d usb_rdy=%d\n",pmic_rdy,usb_rdy);
	if(pmic_rdy==1 && usb_rdy==1)
	{
		wake_up_bat();  
		PMICLOG("[pmic_enable_charger_detection_int] enable charger detection interrupt\n");
	}
	
}

kal_bool is_charger_detection_rdy(void)
{

	if(pmic_rdy==1 && usb_rdy==1)
	{
		return KAL_TRUE;
	}
	else
	{
		return KAL_FALSE;
	}
}


static int pmic_thread_kthread(void *x)
{
	kal_uint32 i;
	kal_uint32 int_status_val=0;
	U32 pwrap_eint_status=0;
	struct sched_param param = { .sched_priority = 98 };

	sched_setscheduler(current, SCHED_FIFO, &param);
	set_current_state(TASK_INTERRUPTIBLE);

	PMICLOG("[PMIC_INT] enter\n");

        pmic_enable_charger_detection_int(0);

	/* Run on a process content */
	while (1) {
		mutex_lock(&pmic_mutex);

		pwrap_eint_status = pmic_wrap_eint_status();    
		PMICLOG("[PMIC_INT] pwrap_eint_status=0x%x\n", pwrap_eint_status);

		pmic_int_handler();

		pmic_wrap_eint_clr(0x0);
		//PMICLOG("[PMIC_INT] pmic_wrap_eint_clr(0x0);\n");

		for (i = 0; i < ARRAY_SIZE(interrupts); i++) 
		{
			int_status_val=upmu_get_reg_value(interrupts[i].address);
			PMICLOG("[PMIC_INT] after ,int_status_val[0x%x]=0x%x\n", interrupts[i].address,int_status_val);		
		}	


		mdelay(1);

		mutex_unlock(&pmic_mutex);
		wake_unlock(&pmicThread_lock);

		set_current_state(TASK_INTERRUPTIBLE);        
		mt_eint_unmask(g_eint_pmic_num);
		schedule();
	}

	return 0;
}



int is_ext_buck2_exist(void)
{
#if defined(CONFIG_MTK_FPGA)
        return 0;
#else
        return mt_get_gpio_in(130);
#endif 
}

int is_ext_vbat_boost_exist(void)
{
        return 0;    
}

int is_ext_swchr_exist(void)
{
	return 0;
}

/*
//==============================================================================
// Enternal SWCHR
//==============================================================================
#ifdef MTK_BQ24261_SUPPORT
extern int is_bq24261_exist(void);
#endif

int is_ext_swchr_exist(void)
{
    #ifdef MTK_BQ24261_SUPPORT
        if( (is_bq24261_exist()==1) )
            return 1;    
        else
            return 0;    
    #else
        PMICLOG("[is_ext_swchr_exist] no define any HW\n");
        return 0;
    #endif    
}


//==============================================================================
// Enternal VBAT Boost status
//==============================================================================
extern int is_tps6128x_sw_ready(void);
extern int is_tps6128x_exist(void);

int is_ext_vbat_boost_sw_ready(void)
{    
    if( (is_tps6128x_sw_ready()==1) )
        return 1;    
    else
        return 0;    
}

int is_ext_vbat_boost_exist(void)
{
    if( (is_tps6128x_exist()==1) )
        return 1;
    else
        return 0;    
}

*/

//==============================================================================
// Enternal BUCK status
//==============================================================================
extern int is_mt6311_sw_ready(void);
extern int is_mt6311_exist(void);
extern int get_mt6311_i2c_ch_num(void);

int get_ext_buck_i2c_ch_num(void)
{
    if(is_mt6311_exist()==1)
    {
        return get_mt6311_i2c_ch_num();
    }
    else
    {
        return -1;
    }
}

int is_ext_buck_sw_ready(void)
{    
    if( (is_mt6311_sw_ready()==1) )
        return 1;    
    else
        return 0;    
}

int is_ext_buck_exist(void)
{
    if( (is_mt6311_exist()==1) )
        return 1;
    else
        return 0;    
}

//==============================================================================
// FTM 
//==============================================================================
#define PMIC_DEVNAME "pmic_ftm"
#define Get_IS_EXT_BUCK_EXIST _IOW('k', 20, int)
#define Get_IS_EXT_VBAT_BOOST_EXIST _IOW('k', 21, int)
#define Get_IS_EXT_SWCHR_EXIST _IOW('k', 22, int)
#define Get_IS_EXT_BUCK2_EXIST _IOW('k', 23, int)


static struct class *pmic_class = NULL;
static struct cdev *pmic_cdev;
static int pmic_major = 0;
static dev_t pmic_devno;

static long pmic_ftm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int *user_data_addr;
    int ret = 0;
	int adc_in_data[2] = {1,1};
	int adc_out_data[2] = {1,1};

    switch(cmd)
    {
        //#if defined(FTM_EXT_BUCK_CHECK)
            case Get_IS_EXT_BUCK_EXIST:
                user_data_addr = (int *)arg;
                ret = copy_from_user(adc_in_data, user_data_addr, 8);
                adc_out_data[0] = is_ext_buck_exist();
                ret = copy_to_user(user_data_addr, adc_out_data, 8); 
                PMICLOG("[pmic_ftm_ioctl] Get_IS_EXT_BUCK_EXIST:%d\n", adc_out_data[0]);
            break;
        //#endif

        //#if defined(FTM_EXT_VBAT_BOOST_CHECK)
            case Get_IS_EXT_VBAT_BOOST_EXIST:
                user_data_addr = (int *)arg;
                ret = copy_from_user(adc_in_data, user_data_addr, 8);
                adc_out_data[0] = is_ext_vbat_boost_exist();
                ret = copy_to_user(user_data_addr, adc_out_data, 8); 
                PMICLOG("[pmic_ftm_ioctl] Get_IS_EXT_VBAT_BOOST_EXIST:%d\n", adc_out_data[0]);
            break;
        //#endif
        
        //#if defined(FEATURE_FTM_SWCHR_HW_DETECT)
            case Get_IS_EXT_SWCHR_EXIST:
                user_data_addr = (int *)arg;
                ret = copy_from_user(adc_in_data, user_data_addr, 8);
                adc_out_data[0] = is_ext_swchr_exist();
                ret = copy_to_user(user_data_addr, adc_out_data, 8); 
                PMICLOG("[pmic_ftm_ioctl] Get_IS_EXT_SWCHR_EXIST:%d\n", adc_out_data[0]);
            break;
        //#endif
            case Get_IS_EXT_BUCK2_EXIST:
                user_data_addr = (int *)arg;
                ret = copy_from_user(adc_in_data, user_data_addr, 8);
                adc_out_data[0] = is_ext_buck2_exist();
                ret = copy_to_user(user_data_addr, adc_out_data, 8); 
                PMICLOG("[pmic_ftm_ioctl] Get_IS_EXT_BUCK2_EXIST:%d\n", adc_out_data[0]);
            break;        
        default:
            PMICLOG("[pmic_ftm_ioctl] Error ID\n");
            break;
    }
    
    return 0;
}

static int pmic_ftm_open(struct inode *inode, struct file *file)
{ 
   return 0;
}

static int pmic_ftm_release(struct inode *inode, struct file *file)
{
    return 0;
}


static struct file_operations pmic_ftm_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = pmic_ftm_ioctl,
    .open           = pmic_ftm_open,
    .release        = pmic_ftm_release,    
};

void pmic_ftm_init(void)
{
    struct class_device *class_dev = NULL;
    int ret=0;
    
    ret = alloc_chrdev_region(&pmic_devno, 0, 1, PMIC_DEVNAME);
    if (ret) 
        PMICLOG("[pmic_ftm_init] Error: Can't Get Major number for pmic_ftm\n");
    
    pmic_cdev = cdev_alloc();
    pmic_cdev->owner = THIS_MODULE;
    pmic_cdev->ops = &pmic_ftm_fops;

    ret = cdev_add(pmic_cdev, pmic_devno, 1);
    if(ret)
        PMICLOG("[pmic_ftm_init] Error: cdev_add\n");
    
    pmic_major = MAJOR(pmic_devno);
    pmic_class = class_create(THIS_MODULE, PMIC_DEVNAME);
    
    class_dev = (struct class_device *)device_create(pmic_class, 
                                                   NULL, 
                                                   pmic_devno, 
                                                   NULL, 
                                                   PMIC_DEVNAME);
    
    PMICLOG("[pmic_ftm_init] Done\n");
}




//==============================================================================
// HW Setting 
//==============================================================================
kal_uint16 is_battery_remove=0;
kal_uint16 is_wdt_reboot_pmic=0;

kal_uint16 is_battery_remove_pmic(void)
{
	return is_battery_remove;
}

extern bool crystal_exist_status(void);

extern void mt6311_hw_component_detect(void);

void PMIC_INIT_SETTING_V1(void)
{
    U32 chip_version = 0;
    U32 ret = 0;
    
    chip_version = pmic_get_register_value(PMIC_SWCID);

	is_battery_remove=!pmic_get_register_value(PMIC_STRUP_PWROFF_SEQ_EN);
	is_wdt_reboot_pmic=pmic_get_register_value(PMIC_WDTRSTB_STATUS);
	pmic_set_register_value(PMIC_WDTRSTB_STATUS_CLR,1);
	

    //--------------------------------------------------------

        PMICLOG("[Kernel_PMIC_INIT_SETTING_V1] 6328 PMIC Chip = 0x%x\n",chip_version);
	PMICLOG("[Kernel_PMIC_INIT_SETTING_V1] is_battery_remove =%d is_wdt_reboot=%d 6311=%d\n",is_battery_remove,is_wdt_reboot_pmic,is_ext_buck_exist());

   if(is_ext_buck_exist()==1)
   {
           PMICLOG("[Kernel_PMIC_INIT_SETTING_V1] 2015-04-13 for turbo...\n");
ret = pmic_config_interface(0x4,0x1,0x1,4); // [4:4]: RG_EN_DRVSEL; Ricky
ret = pmic_config_interface(0xA,0x1,0x1,0); // [0:0]: DDUVLO_DEB_EN; Ricky
ret = pmic_config_interface(0xA,0x1,0x1,11); // [11:11]: BIAS_GEN_EN_SEL; Luke, in pre-load for SMPS
ret = pmic_config_interface(0xC,0x1,0x1,0); // [0:0]: VPROC_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,1); // [1:1]: VAUX18_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,4); // [4:4]: VCORE1_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,5); // [5:5]: VSYS22_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,6); // [6:6]: VLTE_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,7); // [7:7]: VIO18_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,8); // [8:8]: VAUD28_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,9); // [9:9]: VTCXO_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,10); // [10:10]: VUSB_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,11); // [11:11]: VSRAM_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,12); // [12:12]: VIO28_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,13); // [13:13]: VM_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xE,0x1,0x1,0); // [0:0]: VPROC_PG_ENB; disable PG ,0514 Luke
ret = pmic_config_interface(0x10,0x1,0x1,5); // [5:5]: UVLO_L2H_DEB_EN; Ricky
ret = pmic_config_interface(0x16,0x1,0x1,0); // [0:0]: STRUP_PWROFF_SEQ_EN; Ricky
ret = pmic_config_interface(0x16,0x1,0x1,1); // [1:1]: STRUP_PWROFF_PREOFF_EN; Ricky
ret = pmic_config_interface(0x1E,0x0,0x1,11); // [11:11]: RG_TESTMODE_SWEN; CC: Test mode, first command
ret = pmic_config_interface(0x32,0x1,0x1,15); // [15:15]: VPROC_OC_ENB; disable vproc OC, 0514 Luke
ret = pmic_config_interface(0x40,0x1,0x1,12); // [12:12]: RG_RST_DRVSEL; Ricky
ret = pmic_config_interface(0x204,0x1,0x1,4); // [4:4]: RG_SRCLKEN_IN0_HW_MODE; Juinn-Ting
ret = pmic_config_interface(0x204,0x1,0x1,5); // [5:5]: RG_SRCLKEN_IN1_HW_MODE; Juinn-Ting
ret = pmic_config_interface(0x204,0x0,0x1,6); // [6:6]: RG_OSC_SEL_HW_MODE; Luke, for Vsys PFM issue,0204
ret = pmic_config_interface(0x226,0x1,0x1,0); // [0:0]: RG_SMT_WDTRSTB_IN; Ricky
ret = pmic_config_interface(0x226,0x1,0x1,2); // [2:2]: RG_SMT_SRCLKEN_IN0; Ricky
ret = pmic_config_interface(0x226,0x1,0x1,3); // [3:3]: RG_SMT_SRCLKEN_IN1; Ricky
ret = pmic_config_interface(0x242,0x1,0x1,2); // [2:2]: RG_RTC_75K_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x242,0x1,0x1,3); // [3:3]: RG_RTCDET_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x248,0x1,0x1,13); // [13:13]: RG_RTC_EOSC32_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x248,0x1,0x1,14); // [14:14]: RG_TRIM_75K_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x25A,0x1,0x1,9); // [9:9]: RG_75K_32K_SEL; Angela
ret = pmic_config_interface(0x40E,0x0,0x3,2); // [3:2]: VPROC_OC_WND; update OC debounce timing , 0527, Luke
ret = pmic_config_interface(0x412,0x0,0x3,2); // [3:2]: VLTE_OC_WND; update OC debounce timing , 0527, Luke
ret = pmic_config_interface(0x420,0x1,0x1,4); // [4:4]: VPA_EN_OC_SDN_SEL; disable VPA OC,Luke 0424
ret = pmic_config_interface(0x422,0x0,0x1,0); // [0:0]: VSRAM_TRACK_SLEEP_CTRL; turn off SRAM Tracking,Luke ,04/10
ret = pmic_config_interface(0x422,0x0,0x1,1); // [1:1]: VSRAM_TRACK_ON_CTRL; turn off SRAM Tracking,Luke ,04/10
ret = pmic_config_interface(0x422,0x0,0x1,2); // [2:2]: VPROC_TRACK_ON_CTRL; turn off SRAM Tracking,Luke ,04/10
ret = pmic_config_interface(0x424,0x0,0x7F,0); // [6:0]: VSRAM_VOSEL_DELTA; SRAM Tracking,Fandy
ret = pmic_config_interface(0x424,0x10,0x7F,8); // [14:8]: VSRAM_VOSEL_OFFSET; SRAM Tracking,Fandy
ret = pmic_config_interface(0x426,0x48,0x7F,0); // [6:0]: VSRAM_VOSEL_ON_LB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x426,0x78,0x7F,8); // [14:8]: VSRAM_VOSEL_ON_HB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x428,0x28,0x7F,0); // [6:0]: VSRAM_VOSEL_SLEEP_LB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x42E,0x1,0x1FF,0); // [8:0]: RG_SMPS_TESTMODE_B; 
ret = pmic_config_interface(0x446,0x2,0x7,8); // [10:8]: RG_VCORE1_PFM_RIP; for PFM ripple, Luke
ret = pmic_config_interface(0x44C,0x0,0x3,7); // [8:7]: RG_VSYS22_ZX_OS; Luke,0114 for VSYS damage
ret = pmic_config_interface(0x450,0x80,0xFF,0); // [7:0]: RG_VSYS22_RSV; Luke,0114 for VSYS damage
ret = pmic_config_interface(0x452,0x8,0x3F,3); // [8:3]: RG_VSYS22_TRAN_BST; Luke,0204 for VSYS not sleep
ret = pmic_config_interface(0x462,0x3,0x3,10); // [11:10]: RG_VPA_SLP; Seven Stability
ret = pmic_config_interface(0x470,0x2,0x7,8); // [10:8]: RG_VLTE_PFM_RIP; for PFM ripple, Luke
ret = pmic_config_interface(0x486,0x1,0x1,0); // [0:0]: VPROC_EN; For D-3T, turn on ,Luke 0413; D-3T Vproc OC issue
ret = pmic_config_interface(0x488,0x11,0x7F,0); // [6:0]: VPROC_SFCHG_FRATE; 11/20 DVFS raising slewrate,SY
ret = pmic_config_interface(0x488,0x0,0x1,7); // [7:7]: VPROC_SFCHG_FEN; VSRAM tracking,Fandy
ret = pmic_config_interface(0x488,0x4,0x7F,8); // [14:8]: VPROC_SFCHG_RRATE; 11/20 DVFS raising slewrate,SY
ret = pmic_config_interface(0x488,0x0,0x1,15); // [15:15]: VPROC_SFCHG_REN; VSRAM tracking,Fandy
ret = pmic_config_interface(0x498,0x1,0x1,8); // [8:8]: VPROC_VSLEEP_EN; 11/20 sleep mode by SRCLKEN
ret = pmic_config_interface(0x49A,0x0,0x3,4); // [5:4]: VPROC_OSC_SEL_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x49A,0x0,0x3,8); // [9:8]: VPROC_R2R_PDN_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x49A,0x0,0x3,14); // [15:14]: VPROC_VSLEEP_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x4AA,0x0,0x1,1); // [1:1]: VSRAM_VOSEL_CTRL; SRAM tracking by SW mode, Luke 0305
ret = pmic_config_interface(0x4B0,0x8,0x7F,0); // [6:0]: VSRAM_SFCHG_FRATE; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x1,0x1,7); // [7:7]: VSRAM_SFCHG_FEN; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x8,0x7F,8); // [14:8]: VSRAM_SFCHG_RRATE; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x1,0x1,15); // [15:15]: VSRAM_SFCHG_REN; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B4,0x38,0x7F,0); // [6:0]: VSRAM_VOSEL_ON; SRAM tracking,by SPM request
ret = pmic_config_interface(0x4B6,0x28,0x7F,0); // [6:0]: VSRAM_VOSEL_SLEEP; SRAM tracking,by SPM request
ret = pmic_config_interface(0x4D2,0x1,0x1,1); // [1:1]: VLTE_VOSEL_CTRL; ShangYing
ret = pmic_config_interface(0x4D8,0x11,0x7F,0); // [6:0]: VLTE_SFCHG_FRATE; 11/20 DVFS falling slewrate
ret = pmic_config_interface(0x4D8,0x4,0x7F,8); // [14:8]: VLTE_SFCHG_RRATE; 11/20 DVFS raising slewrate
ret = pmic_config_interface(0x4DE,0x28,0x7F,0); // [6:0]: VLTE_VOSEL_SLEEP; 11/20 Sleep mode 0.85V
ret = pmic_config_interface(0x4E8,0x3,0x3,0); // [1:0]: VLTE_TRANS_TD; ShangYing
ret = pmic_config_interface(0x4E8,0x1,0x1,8); // [8:8]: VLTE_VSLEEP_EN; 11/20 sleep mode by SRCLKEN
ret = pmic_config_interface(0x4EA,0x0,0x3,4); // [5:4]: VLTE_OSC_SEL_SRCLKEN_SEL; 
ret = pmic_config_interface(0x4EA,0x0,0x3,8); // [9:8]: VLTE_R2R_PDN_SRCLKEN_SEL; 
ret = pmic_config_interface(0x4EA,0x0,0x3,14); // [15:14]: VLTE_VSLEEP_SRCLKEN_SEL; 
ret = pmic_config_interface(0x60E,0x1,0x1,1); // [1:1]: VCORE1_VOSEL_CTRL; ShangYing
ret = pmic_config_interface(0x614,0x11,0x7F,0); // [6:0]: VCORE1_SFCHG_FRATE; 11/20 DVS falling slewrate
ret = pmic_config_interface(0x614,0x4,0x7F,8); // [14:8]: VCORE1_SFCHG_RRATE; 11/20 DVS rising slewrate
ret = pmic_config_interface(0x61A,0x28,0x7F,0); // [6:0]: VCORE1_VOSEL_SLEEP; 11/20 sleep mode 0.85V
ret = pmic_config_interface(0x624,0x3,0x3,0); // [1:0]: VCORE1_TRANS_TD; ShangYing
ret = pmic_config_interface(0x624,0x1,0x1,8); // [8:8]: VCORE1_VSLEEP_EN; 11/20 sleep mode control by SRCLKEN
ret = pmic_config_interface(0x626,0x0,0x3,4); // [5:4]: VCORE1_OSC_SEL_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x626,0x0,0x3,8); // [9:8]: VCORE1_R2R_PDN_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x626,0x0,0x3,14); // [15:14]: VCORE1_VSLEEP_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x646,0x5,0x7,0); // [2:0]: VSYS22_BURST; Seven
ret = pmic_config_interface(0x64C,0x0,0x1,8); // [8:8]: VSYS22_VSLEEP_EN; VSYS22 sleep mode in 0105,Luke
ret = pmic_config_interface(0x664,0x2,0x7F,0); // [6:0]: VPA_SFCHG_FRATE; Seven
ret = pmic_config_interface(0x664,0x0,0x1,7); // [7:7]: VPA_SFCHG_FEN; Seven
ret = pmic_config_interface(0x664,0x2,0x7F,8); // [14:8]: VPA_SFCHG_RRATE; Seven
ret = pmic_config_interface(0x664,0x0,0x1,15); // [15:15]: VPA_SFCHG_REN; Seven
ret = pmic_config_interface(0x674,0x3,0x3,4); // [5:4]: VPA_TRANS_CTRL; Seven for BW extention
ret = pmic_config_interface(0x67E,0x2,0x3,4); // [5:4]: VPA_DVS_TRANS_CTRL; Seven for BW extention
ret = pmic_config_interface(0xA00,0x1,0x1,3); // [3:3]: RG_VTCXO_0_ON_CTRL; 12/15, Fandy HW mode
ret = pmic_config_interface(0xA02,0x1,0x1,3); // [3:3]: RG_VTCXO_1_ON_CTRL; by RF request 11/02,Luke
ret = pmic_config_interface(0xA06,0x1,0x1,6); // [6:6]: RG_VAUX18_AUXADC_PWDB_EN; Chuan-Hung
ret = pmic_config_interface(0xA30,0x0,0x1,0); // [0:0]: RG_VEFUSE_MODE_SET; Fandy:Disable VEFUSE
ret = pmic_config_interface(0xA44,0x1,0x1,1); // [1:1]: RG_TREF_EN; Tim
ret = pmic_config_interface(0xA46,0x0,0x1,14); // [14:14]: QI_VM_STB; Fandy, disable
ret = pmic_config_interface(0xA64,0x2,0x3,4); // [5:4]: RG_VEMC_3V3_VOSEL; 
ret = pmic_config_interface(0xA88,0x68,0x7F,0); // [6:0]: RG_VSRAM_VOSEL; update for D-3 T VSRAM initial setting,Luke
ret = pmic_config_interface(0xC14,0x1,0x1,0); // [0:0]: RG_SKIP_OTP_OUT; Fandy: for CORE power(VDVFS1x, VCOREx, VSRAM_DVFS) max voltage limitation.
ret = pmic_config_interface(0xCBC,0x1,0x1,8); // [8:8]: FG_SLP_EN; Ricky
ret = pmic_config_interface(0xCBC,0x1,0x1,9); // [9:9]: FG_ZCV_DET_EN; Ricky
ret = pmic_config_interface(0xCC0,0x24,0xFFFF,0); // [15:0]: FG_SLP_CUR_TH; Ricky
ret = pmic_config_interface(0xCC2,0x14,0xFF,0); // [7:0]: FG_SLP_TIME; Ricky
ret = pmic_config_interface(0xCC4,0xFF,0xFF,8); // [15:8]: FG_DET_TIME; Ricky
ret = pmic_config_interface(0xE94,0x0,0x1,13); // [13:13]: AUXADC_CK_AON_GPS; YP Niou, sync with golden setting
ret = pmic_config_interface(0xE94,0x0,0x1,14); // [14:14]: AUXADC_CK_AON_MD; YP Niou, sync with golden setting
ret = pmic_config_interface(0xE94,0x0,0x1,15); // [15:15]: AUXADC_CK_AON; YP Niou, sync with golden setting
ret = pmic_config_interface(0xEA4,0x1,0x3,4); // [5:4]: AUXADC_TRIM_CH2_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,6); // [7:6]: AUXADC_TRIM_CH3_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,8); // [9:8]: AUXADC_TRIM_CH4_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,10); // [11:10]: AUXADC_TRIM_CH5_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,12); // [13:12]: AUXADC_TRIM_CH6_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x2,0x3,14); // [15:14]: AUXADC_TRIM_CH7_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,0); // [1:0]: AUXADC_TRIM_CH8_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,2); // [3:2]: AUXADC_TRIM_CH9_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,4); // [5:4]: AUXADC_TRIM_CH10_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,6); // [7:6]: AUXADC_TRIM_CH11_SEL; Ricky
ret = pmic_config_interface(0xEB8,0x1,0x1,14); // [14:14]: AUXADC_START_SHADE_EN; Chuan-Hung
ret = pmic_config_interface(0xF4A,0xB,0xF,4); // [7:4]: RG_VCDT_HV_VTH; Tim:VCDT_HV_th=7V
ret = pmic_config_interface(0xF54,0x0,0x7,1); // [3:1]: RG_VBAT_OV_VTH; Tim:for 4.35 battery
ret = pmic_config_interface(0xF62,0x3,0xF,0); // [3:0]: RG_CHRWDT_TD; Tim:WDT=32s
ret = pmic_config_interface(0xF6C,0x2,0x1F,0); // [4:0]: RG_LBAT_INT_VTH; Ricky: E1 only
ret = pmic_config_interface(0xF70,0x1,0x1,1); // [1:1]: RG_BC11_RST; Tim:Disable BC1.1 timer
ret = pmic_config_interface(0xF74,0x0,0x7,4); // [6:4]: RG_CSDAC_STP_DEC; Tim:Reduce ICHG current ripple (align 6323)
ret = pmic_config_interface(0xF7A,0x1,0x1,2); // [2:2]: RG_CSDAC_MODE; Tim:Align 6323
ret = pmic_config_interface(0xF7A,0x1,0x1,6); // [6:6]: RG_HWCV_EN; Tim:Align 6323
ret = pmic_config_interface(0xF7A,0x1,0x1,7); // [7:7]: RG_ULC_DET_EN; Tim:Align 6323
   }
   else
   {
   	PMICLOG("[Kernel_PMIC_INIT_SETTING_V1] 2015-04-10...\n");
ret = pmic_config_interface(0x4,0x1,0x1,4); // [4:4]: RG_EN_DRVSEL; Ricky
ret = pmic_config_interface(0xA,0x1,0x1,0); // [0:0]: DDUVLO_DEB_EN; Ricky
ret = pmic_config_interface(0xA,0x1,0x1,11); // [11:11]: BIAS_GEN_EN_SEL; Luke, in pre-load for SMPS
ret = pmic_config_interface(0xC,0x1,0x1,0); // [0:0]: VPROC_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,1); // [1:1]: VAUX18_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,4); // [4:4]: VCORE1_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,5); // [5:5]: VSYS22_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,6); // [6:6]: VLTE_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,7); // [7:7]: VIO18_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,8); // [8:8]: VAUD28_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,9); // [9:9]: VTCXO_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,10); // [10:10]: VUSB_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,11); // [11:11]: VSRAM_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,12); // [12:12]: VIO28_PG_H2L_EN; Ricky
ret = pmic_config_interface(0xC,0x1,0x1,13); // [13:13]: VM_PG_H2L_EN; Ricky
ret = pmic_config_interface(0x10,0x1,0x1,5); // [5:5]: UVLO_L2H_DEB_EN; Ricky
ret = pmic_config_interface(0x16,0x1,0x1,0); // [0:0]: STRUP_PWROFF_SEQ_EN; Ricky
ret = pmic_config_interface(0x16,0x1,0x1,1); // [1:1]: STRUP_PWROFF_PREOFF_EN; Ricky
ret = pmic_config_interface(0x1E,0x0,0x1,11); // [11:11]: RG_TESTMODE_SWEN; CC: Test mode, first command
ret = pmic_config_interface(0x40,0x1,0x1,12); // [12:12]: RG_RST_DRVSEL; Ricky
ret = pmic_config_interface(0x204,0x1,0x1,4); // [4:4]: RG_SRCLKEN_IN0_HW_MODE; Juinn-Ting
ret = pmic_config_interface(0x204,0x1,0x1,5); // [5:5]: RG_SRCLKEN_IN1_HW_MODE; Juinn-Ting
ret = pmic_config_interface(0x204,0x0,0x1,6); // [6:6]: RG_OSC_SEL_HW_MODE; Luke, for Vsys PFM issue,0204
ret = pmic_config_interface(0x226,0x1,0x1,0); // [0:0]: RG_SMT_WDTRSTB_IN; Ricky
ret = pmic_config_interface(0x226,0x1,0x1,2); // [2:2]: RG_SMT_SRCLKEN_IN0; Ricky
ret = pmic_config_interface(0x226,0x1,0x1,3); // [3:3]: RG_SMT_SRCLKEN_IN1; Ricky
ret = pmic_config_interface(0x242,0x1,0x1,2); // [2:2]: RG_RTC_75K_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x242,0x1,0x1,3); // [3:3]: RG_RTCDET_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x248,0x1,0x1,13); // [13:13]: RG_RTC_EOSC32_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x248,0x1,0x1,14); // [14:14]: RG_TRIM_75K_CK_PDN; Juinn-Ting
ret = pmic_config_interface(0x25A,0x1,0x1,9); // [9:9]: RG_75K_32K_SEL; Angela
ret = pmic_config_interface(0x40E,0x0,0x3,2); // [3:2]: VPROC_OC_WND; update OC debounce timing , 0527, Luke
ret = pmic_config_interface(0x412,0x0,0x3,2); // [3:2]: VLTE_OC_WND; update OC debounce timing , 0527, Luke
ret = pmic_config_interface(0x420,0x1,0x1,4); // [4:4]: VPA_EN_OC_SDN_SEL; Disable VPA OC shutdown, Luke 0421
ret = pmic_config_interface(0x422,0x1,0x1,0); // [0:0]: VSRAM_TRACK_SLEEP_CTRL; SRAM Tracking,Fandy
ret = pmic_config_interface(0x422,0x1,0x1,1); // [1:1]: VSRAM_TRACK_ON_CTRL; SRAM Tracking,Fandy
ret = pmic_config_interface(0x422,0x1,0x1,2); // [2:2]: VPROC_TRACK_ON_CTRL; SRAM Tracking,Fandy
ret = pmic_config_interface(0x424,0x0,0x7F,0); // [6:0]: VSRAM_VOSEL_DELTA; SRAM Tracking,Fandy
ret = pmic_config_interface(0x424,0x10,0x7F,8); // [14:8]: VSRAM_VOSEL_OFFSET; SRAM Tracking,Fandy
ret = pmic_config_interface(0x426,0x48,0x7F,0); // [6:0]: VSRAM_VOSEL_ON_LB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x426,0x78,0x7F,8); // [14:8]: VSRAM_VOSEL_ON_HB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x428,0x28,0x7F,0); // [6:0]: VSRAM_VOSEL_SLEEP_LB; SRAM Tracking,Fandy
ret = pmic_config_interface(0x42E,0x1,0x1FF,0); // [8:0]: RG_SMPS_TESTMODE_B; 
ret = pmic_config_interface(0x446,0x2,0x7,8); // [10:8]: RG_VCORE1_PFM_RIP; for FPM ripple,0312
ret = pmic_config_interface(0x44C,0x0,0x3,7); // [8:7]: RG_VSYS22_ZX_OS; Luke,0114 for VSYS damage
ret = pmic_config_interface(0x450,0x80,0xFF,0); // [7:0]: RG_VSYS22_RSV; Luke,0114 for VSYS damage
ret = pmic_config_interface(0x452,0x8,0x3F,3); // [8:3]: RG_VSYS22_TRAN_BST; Luke,0204 for VSYS not sleep
ret = pmic_config_interface(0x45A,0x2,0x7,8); // [10:8]: RG_VPROC_PFM_RIP; for FPM ripple,0312
ret = pmic_config_interface(0x462,0x3,0x3,10); // [11:10]: RG_VPA_SLP; Seven Stability
ret = pmic_config_interface(0x470,0x2,0x7,8); // [10:8]: RG_VLTE_PFM_RIP; for FPM ripple,0312
ret = pmic_config_interface(0x482,0x1,0x1,1); // [1:1]: VPROC_VOSEL_CTRL; ShangYing
ret = pmic_config_interface(0x488,0x11,0x7F,0); // [6:0]: VPROC_SFCHG_FRATE; 11/20 DVFS raising slewrate,SY
ret = pmic_config_interface(0x488,0x1,0x1,7); // [7:7]: VPROC_SFCHG_FEN; VSRAM tracking,Fandy
ret = pmic_config_interface(0x488,0x4,0x7F,8); // [14:8]: VPROC_SFCHG_RRATE; 11/20 DVFS raising slewrate,SY
ret = pmic_config_interface(0x488,0x1,0x1,15); // [15:15]: VPROC_SFCHG_REN; VSRAM tracking,Fandy
ret = pmic_config_interface(0x48E,0x28,0x7F,0); // [6:0]: VPROC_VOSEL_SLEEP; 11/20 Sleep mode 0.85V
ret = pmic_config_interface(0x498,0x3,0x3,0); // [1:0]: VPROC_TRANS_TD; ShangYing
ret = pmic_config_interface(0x498,0x1,0x3,4); // [5:4]: VPROC_TRANS_CTRL; ShangYing
ret = pmic_config_interface(0x498,0x1,0x1,8); // [8:8]: VPROC_VSLEEP_EN; 11/20 sleep mode by SRCLKEN
ret = pmic_config_interface(0x49A,0x0,0x3,4); // [5:4]: VPROC_OSC_SEL_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x49A,0x0,0x3,8); // [9:8]: VPROC_R2R_PDN_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x49A,0x0,0x3,14); // [15:14]: VPROC_VSLEEP_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x4AA,0x1,0x1,1); // [1:1]: VSRAM_VOSEL_CTRL; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x8,0x7F,0); // [6:0]: VSRAM_SFCHG_FRATE; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x1,0x1,7); // [7:7]: VSRAM_SFCHG_FEN; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x8,0x7F,8); // [14:8]: VSRAM_SFCHG_RRATE; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B0,0x1,0x1,15); // [15:15]: VSRAM_SFCHG_REN; SRAM tracking,Fandy
ret = pmic_config_interface(0x4B4,0x40,0x7F,0); // [6:0]: VSRAM_VOSEL_ON; SRAM tracking,Fandy
ret = pmic_config_interface(0x4D2,0x1,0x1,1); // [1:1]: VLTE_VOSEL_CTRL; ShangYing
ret = pmic_config_interface(0x4D8,0x11,0x7F,0); // [6:0]: VLTE_SFCHG_FRATE; 11/20 DVFS falling slewrate
ret = pmic_config_interface(0x4D8,0x4,0x7F,8); // [14:8]: VLTE_SFCHG_RRATE; 11/20 DVFS raising slewrate
ret = pmic_config_interface(0x4DE,0x28,0x7F,0); // [6:0]: VLTE_VOSEL_SLEEP; 11/20 Sleep mode 0.85V
ret = pmic_config_interface(0x4E8,0x3,0x3,0); // [1:0]: VLTE_TRANS_TD; ShangYing
ret = pmic_config_interface(0x4E8,0x1,0x1,8); // [8:8]: VLTE_VSLEEP_EN; 11/20 sleep mode by SRCLKEN
ret = pmic_config_interface(0x4EA,0x0,0x3,4); // [5:4]: VLTE_OSC_SEL_SRCLKEN_SEL; 
ret = pmic_config_interface(0x4EA,0x0,0x3,8); // [9:8]: VLTE_R2R_PDN_SRCLKEN_SEL; 
ret = pmic_config_interface(0x4EA,0x0,0x3,14); // [15:14]: VLTE_VSLEEP_SRCLKEN_SEL; 
ret = pmic_config_interface(0x60E,0x1,0x1,1); // [1:1]: VCORE1_VOSEL_CTRL; ShangYing
ret = pmic_config_interface(0x614,0x11,0x7F,0); // [6:0]: VCORE1_SFCHG_FRATE; 11/20 DVS falling slewrate
ret = pmic_config_interface(0x614,0x4,0x7F,8); // [14:8]: VCORE1_SFCHG_RRATE; 11/20 DVS rising slewrate
ret = pmic_config_interface(0x61A,0x28,0x7F,0); // [6:0]: VCORE1_VOSEL_SLEEP; 11/20 sleep mode 0.85V
ret = pmic_config_interface(0x624,0x3,0x3,0); // [1:0]: VCORE1_TRANS_TD; ShangYing
ret = pmic_config_interface(0x624,0x1,0x1,8); // [8:8]: VCORE1_VSLEEP_EN; 11/20 sleep mode control by SRCLKEN
ret = pmic_config_interface(0x626,0x0,0x3,4); // [5:4]: VCORE1_OSC_SEL_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x626,0x0,0x3,8); // [9:8]: VCORE1_R2R_PDN_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x626,0x0,0x3,14); // [15:14]: VCORE1_VSLEEP_SRCLKEN_SEL; ShangYing
ret = pmic_config_interface(0x646,0x5,0x7,0); // [2:0]: VSYS22_BURST; Seven
ret = pmic_config_interface(0x64C,0x0,0x1,8); // [8:8]: VSYS22_VSLEEP_EN; VSYS22 sleep mode in 0105,Luke
ret = pmic_config_interface(0x664,0x2,0x7F,0); // [6:0]: VPA_SFCHG_FRATE; Seven
ret = pmic_config_interface(0x664,0x0,0x1,7); // [7:7]: VPA_SFCHG_FEN; Seven
ret = pmic_config_interface(0x664,0x2,0x7F,8); // [14:8]: VPA_SFCHG_RRATE; Seven
ret = pmic_config_interface(0x664,0x0,0x1,15); // [15:15]: VPA_SFCHG_REN; Seven
ret = pmic_config_interface(0x674,0x3,0x3,4); // [5:4]: VPA_TRANS_CTRL; Seven for BW extention
ret = pmic_config_interface(0x67E,0x2,0x3,4); // [5:4]: VPA_DVS_TRANS_CTRL; Seven for BW extention
ret = pmic_config_interface(0xA00,0x1,0x1,3); // [3:3]: RG_VTCXO_0_ON_CTRL; 12/15, Fandy HW mode
ret = pmic_config_interface(0xA02,0x1,0x1,3); // [3:3]: RG_VTCXO_1_ON_CTRL; by RF request 11/02,Luke
ret = pmic_config_interface(0xA06,0x1,0x1,6); // [6:6]: RG_VAUX18_AUXADC_PWDB_EN; Chuan-Hung
ret = pmic_config_interface(0xA30,0x0,0x1,0); // [0:0]: RG_VEFUSE_MODE_SET; Fandy:Disable VEFUSE
ret = pmic_config_interface(0xA44,0x1,0x1,1); // [1:1]: RG_TREF_EN; Tim
ret = pmic_config_interface(0xA46,0x0,0x1,14); // [14:14]: QI_VM_STB; Fandy, disable
ret = pmic_config_interface(0xA64,0x2,0x3,4); // [5:4]: RG_VEMC_3V3_VOSEL; 
ret = pmic_config_interface(0xC14,0x1,0x1,0); // [0:0]: RG_SKIP_OTP_OUT; Fandy: for CORE power(VDVFS1x, VCOREx, VSRAM_DVFS) max voltage limitation.
ret = pmic_config_interface(0xCBC,0x1,0x1,8); // [8:8]: FG_SLP_EN; Ricky
ret = pmic_config_interface(0xCBC,0x1,0x1,9); // [9:9]: FG_ZCV_DET_EN; Ricky
ret = pmic_config_interface(0xCC0,0x24,0xFFFF,0); // [15:0]: FG_SLP_CUR_TH; Ricky
ret = pmic_config_interface(0xCC2,0x14,0xFF,0); // [7:0]: FG_SLP_TIME; Ricky
ret = pmic_config_interface(0xCC4,0xFF,0xFF,8); // [15:8]: FG_DET_TIME; Ricky
ret = pmic_config_interface(0xE94,0x0,0x1,13); // [13:13]: AUXADC_CK_AON_GPS; YP Niou, sync with golden setting
ret = pmic_config_interface(0xE94,0x0,0x1,14); // [14:14]: AUXADC_CK_AON_MD; YP Niou, sync with golden setting
ret = pmic_config_interface(0xE94,0x0,0x1,15); // [15:15]: AUXADC_CK_AON; YP Niou, sync with golden setting
ret = pmic_config_interface(0xEA4,0x1,0x3,4); // [5:4]: AUXADC_TRIM_CH2_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,6); // [7:6]: AUXADC_TRIM_CH3_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,8); // [9:8]: AUXADC_TRIM_CH4_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,10); // [11:10]: AUXADC_TRIM_CH5_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x1,0x3,12); // [13:12]: AUXADC_TRIM_CH6_SEL; Ricky
ret = pmic_config_interface(0xEA4,0x2,0x3,14); // [15:14]: AUXADC_TRIM_CH7_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,0); // [1:0]: AUXADC_TRIM_CH8_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,2); // [3:2]: AUXADC_TRIM_CH9_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,4); // [5:4]: AUXADC_TRIM_CH10_SEL; Ricky
ret = pmic_config_interface(0xEA6,0x1,0x3,6); // [7:6]: AUXADC_TRIM_CH11_SEL; Ricky
ret = pmic_config_interface(0xEB8,0x1,0x1,14); // [14:14]: AUXADC_START_SHADE_EN; Chuan-Hung
ret = pmic_config_interface(0xF4A,0xB,0xF,4); // [7:4]: RG_VCDT_HV_VTH; Tim:VCDT_HV_th=7V
ret = pmic_config_interface(0xF54,0x0,0x7,1); // [3:1]: RG_VBAT_OV_VTH; Tim:for 4.35 battery
ret = pmic_config_interface(0xF62,0x3,0xF,0); // [3:0]: RG_CHRWDT_TD; Tim:WDT=32s
ret = pmic_config_interface(0xF6C,0x2,0x1F,0); // [4:0]: RG_LBAT_INT_VTH; Ricky: E1 only
ret = pmic_config_interface(0xF70,0x1,0x1,1); // [1:1]: RG_BC11_RST; Tim:Disable BC1.1 timer
ret = pmic_config_interface(0xF74,0x0,0x7,4); // [6:4]: RG_CSDAC_STP_DEC; Tim:Reduce ICHG current ripple (align 6323)
ret = pmic_config_interface(0xF7A,0x1,0x1,2); // [2:2]: RG_CSDAC_MODE; Tim:Align 6323
ret = pmic_config_interface(0xF7A,0x1,0x1,6); // [6:6]: RG_HWCV_EN; Tim:Align 6323
ret = pmic_config_interface(0xF7A,0x1,0x1,7); // [7:7]: RG_ULC_DET_EN; Tim:Align 6323
   
   }



    //--------------------------------------------------------

	if(crystal_exist_status()==0)
	{
		PMICLOG("32k-less VTCXO always on...\n");
		pmic_set_register_value(PMIC_RG_VTCXO_0_ON_CTRL,0);		
		pmic_set_register_value(PMIC_RG_VTCXO_0_EN,1);	
		
	}

    //--------------------------------------------------------
    #if defined(PMIC_HW_USE_4L_SS_LAYOUT)
    ret = pmic_config_interface(0x494,0x0,0x3,0); // [1:0]: VPROC slow slew rate
    ret = pmic_config_interface(0x496,0x0,0x3,0); // [1:0]: VPROC slow slew rate
    ret = pmic_config_interface(0x620,0x0,0x3,0); // [1:0]: VCORE slow slew rate
    ret = pmic_config_interface(0x622,0x0,0x3,0); // [1:0]: VCORE slow slew rate
    ret = pmic_config_interface(0x4E4,0x0,0x3,0); // [1:0]: VLTE slow slew rate
    ret = pmic_config_interface(0x4E6,0x0,0x3,0); // [1:0]: VLTE slow slew rate
    ret = pmic_config_interface(0x648,0x0,0x3,0); // [1:0]: VSYS slow slew rate
    ret = pmic_config_interface(0x64A,0x0,0x3,0); // [1:0]: VSYS slow slew rate
    #endif
}

extern void pmu_drv_tool_customization_init(void);

void PMIC_CUSTOM_SETTING_V1(void)
{
    #if defined(CONFIG_MTK_FPGA)
    #else    
    pmu_drv_tool_customization_init(); //DCT
    #endif
}


//==============================================================================
// Dump all LDO status 
//==============================================================================
void dump_ldo_status_read_debug(void)
{
	int i,j;
	int en=0;
	int voltage_reg=0;
	int voltage=0;
	const int *pVoltage;

	printk("********** BUCK/LDO status dump [1:ON,0:OFF]**********\n");

	for (i = 0; i < ARRAY_SIZE(mtk_bucks); i++) 
	{
		if(mtk_bucks[i].qi_en_reg!=0)
		{
			en=pmic_get_register_value(mtk_bucks[i].qi_en_reg);
		}
		else
		{
			en=-1;
		}

		if(mtk_bucks[i].qi_vol_reg!=0)
		{
			voltage_reg=pmic_get_register_value(mtk_bucks[i].qi_vol_reg);
			voltage=mtk_bucks[i].desc.min_uV+mtk_bucks[i].desc.uV_step*voltage_reg;
		}
		else
		{
			voltage_reg=-1;
			voltage=-1;
		}
		PMICLOG("%s   status:%d     voltage:%duv    voltage_reg:%d\n",mtk_bucks[i].desc.name,en,voltage,voltage_reg);
	}	

	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		if(mtk_ldos[i].en_reg!=0)
		{
			en=pmic_get_register_value(mtk_ldos[i].en_reg);
		}
		else
		{
			en=-1;
		}

		if(mtk_ldos[i].desc.n_voltages!=1)
		{
			if(mtk_ldos[i].vol_reg!=0)
			{
				voltage_reg=pmic_get_register_value(mtk_ldos[i].vol_reg);
				if(mtk_ldos[i].pvoltages!=NULL)
				{
					pVoltage=(const int*)mtk_ldos[i].pvoltages;
					voltage=pVoltage[voltage_reg];
				}
				else
				{
					voltage=mtk_ldos[i].desc.min_uV+mtk_ldos[i].desc.uV_step*voltage_reg;
				}
			}
			else
			{
				voltage_reg=-1;
				voltage=-1;
			}
		}
		else
		{
			pVoltage=(const int*)mtk_ldos[i].pvoltages;
			voltage=pVoltage[0];
		}

		PMICLOG("%s   status:%d     voltage:%duv    voltage_reg:%d\n",mtk_ldos[i].desc.name,en,voltage,voltage_reg);
	}	


	for (i = 0; i < ARRAY_SIZE(interrupts); i++) 
	{
		for(j=0;j<PMIC_INT_WIDTH;j++)
		{

			PMICLOG("[PMIC_INT][%s] interrupt issue times: %d\n", interrupts[i].interrupts[j].name,
				interrupts[i].interrupts[j].times);
		}
	}	

	PMICLOG("Power Good Status=0x%x.\n", upmu_get_reg_value(0x21c));
	PMICLOG("OC Status=0x%x.\n", upmu_get_reg_value(0x214));
	PMICLOG("Thermal Status=0x%x.\n", upmu_get_reg_value(0x21e));
}

static int proc_utilization_show(struct seq_file *m, void *v)
{
	int i,j;
	int en=0;
	int voltage_reg=0;
	int voltage=0;
	const int *pVoltage;

	seq_printf(m,"********** BUCK/LDO status dump [1:ON,0:OFF]**********\n");

	for (i = 0; i < ARRAY_SIZE(mtk_bucks); i++) 
	{
		if(mtk_bucks[i].qi_en_reg!=0)
		{
			en=pmic_get_register_value(mtk_bucks[i].qi_en_reg);
		}
		else
		{
			en=-1;
		}

		if(mtk_bucks[i].qi_vol_reg!=0)
		{
			voltage_reg=pmic_get_register_value(mtk_bucks[i].qi_vol_reg);
			voltage=mtk_bucks[i].desc.min_uV+mtk_bucks[i].desc.uV_step*voltage_reg;
		}
		else
		{
			voltage_reg=-1;
			voltage=-1;
		}
		seq_printf(m,"%s   status:%d     voltage:%duv    voltage_reg:%d\n",mtk_bucks[i].desc.name,en,voltage,voltage_reg);
	}	

	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		if(mtk_ldos[i].en_reg!=0)
		{
			en=pmic_get_register_value(mtk_ldos[i].en_reg);
		}
		else
		{
			en=-1;
		}

		if(mtk_ldos[i].desc.n_voltages!=1)
		{
			if(mtk_ldos[i].vol_reg!=0)
			{
				voltage_reg=pmic_get_register_value(mtk_ldos[i].vol_reg);
				if(mtk_ldos[i].pvoltages!=NULL)
				{
					pVoltage=(const int*)mtk_ldos[i].pvoltages;
					voltage=pVoltage[voltage_reg];
				}
				else
				{
					voltage=mtk_ldos[i].desc.min_uV+mtk_ldos[i].desc.uV_step*voltage_reg;
				}
			}
			else
			{
				voltage_reg=-1;
				voltage=-1;
			}
		}
		else
		{
			pVoltage=(const int*)mtk_ldos[i].pvoltages;
			voltage=pVoltage[0];
		}
		seq_printf(m,"%s   status:%d     voltage:%duv    voltage_reg:%d\n",mtk_ldos[i].desc.name,en,voltage,voltage_reg);
	}	

	for (i = 0; i < ARRAY_SIZE(interrupts); i++) 
	{
		for(j=0;j<PMIC_INT_WIDTH;j++)
		{

			seq_printf(m,"[PMIC_INT][%s] interrupt issue times: %d\n", interrupts[i].interrupts[j].name,
				interrupts[i].interrupts[j].times);
		}
	}	
	seq_printf(m,"Power Good Status=0x%x.\n", upmu_get_reg_value(0x21c));
	seq_printf(m,"OC Status=0x%x.\n", upmu_get_reg_value(0x214));
	seq_printf(m,"Thermal Status=0x%x.\n", upmu_get_reg_value(0x21e));
    
    return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations pmic_debug_proc_fops = { 
    .open  = proc_utilization_open, 
    .read  = seq_read,
};

static int proc_dump_register_show(struct seq_file *m, void *v)
{
	int i;

	seq_printf(m,"********** dump PMIC registers**********\n");

   for(i=0;i<=0x0fae;i=i+10)
   {
	seq_printf(m,"Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x\n",
		i,upmu_get_reg_value(i),i+1,upmu_get_reg_value(i+1),i+2,upmu_get_reg_value(i+2),i+3,upmu_get_reg_value(i+3),i+4,upmu_get_reg_value(i+4));




       seq_printf(m,"Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x Reg[%x]=0x%x\n",
      		i+5,upmu_get_reg_value(i+5),i+6,upmu_get_reg_value(i+6),i+7,upmu_get_reg_value(i+7),i+8,upmu_get_reg_value(i+8),i+9,upmu_get_reg_value(i+9));
   }
    
    return 0;
}

static int proc_dump_register_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_dump_register_show, NULL);
}

static const struct file_operations pmic_dump_register_proc_fops = { 
    .open  = proc_dump_register_open, 
    .read  = seq_read,
};

void pmic_debug_init(void)
{
    struct proc_dir_entry *mt_pmic_dir;

    mt_pmic_dir = proc_mkdir("mt_pmic", NULL);
    if (!mt_pmic_dir) {
        PMICLOG("fail to mkdir /proc/mt_pmic\n" );
        return;
    }

    proc_create("dump_ldo_status", S_IRUGO | S_IWUSR, mt_pmic_dir, &pmic_debug_proc_fops);
    PMICLOG("proc_create pmic_debug_proc_fops\n" );

    proc_create("dump_pmic_reg", S_IRUGO | S_IWUSR, mt_pmic_dir, &pmic_dump_register_proc_fops);
    PMICLOG("proc_create pmic_dump_register_proc_fops\n" );



}

static kal_bool pwrkey_detect_flag = KAL_FALSE;
static struct hrtimer pwrkey_detect_timer;
static struct task_struct *pwrkey_detect_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(pwrkey_detect_waiter);

#define BAT_MS_TO_NS(x) (x * 1000 * 1000)

enum hrtimer_restart pwrkey_detect_sw_workaround(struct hrtimer *timer)
{
	kal_uint32 val=0;
	pwrkey_detect_flag = KAL_TRUE;
	wake_up_interruptible(&pwrkey_detect_waiter);
	

	return HRTIMER_NORESTART;
}

int pwrkey_detect_sw_thread_handler(void *unused)
{
	ktime_t ktime;
	do {
		ktime = ktime_set(3, BAT_MS_TO_NS(1000));



		wait_event_interruptible(pwrkey_detect_waiter,
					 (pwrkey_detect_flag == KAL_TRUE));

	//PMICLOG("=>charger_hv_detect_sw_workaround \n");
	if (pmic_get_register_value(PMIC_RG_STRUP_75K_CK_PDN)==1)
	{
		PMICLOG("charger_hv_detect_sw_workaround =0x%x \n",upmu_get_reg_value(0x24e));
		pmic_set_register_value(PMIC_RG_STRUP_75K_CK_PDN,0);
	}


		hrtimer_start(&pwrkey_detect_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;

}



void pwrkey_sw_workaround_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(0, BAT_MS_TO_NS(2000));
	hrtimer_init(&pwrkey_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pwrkey_detect_timer.function = pwrkey_detect_sw_workaround;
	hrtimer_start(&pwrkey_detect_timer, ktime, HRTIMER_MODE_REL);

	pwrkey_detect_thread =
	    kthread_run(pwrkey_detect_sw_thread_handler, 0,
			"mtk pwrkey_sw_workaround_init");
	
	if (IS_ERR(pwrkey_detect_thread)) {
		PMICLOG("[%s]: failed to create pwrkey_detect_thread thread\n",
				    __func__);
	}

}

#ifdef LOW_BATTERY_PROTECT
//==============================================================================
// low battery protect UT
//==============================================================================
static ssize_t show_low_battery_protect_ut(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_low_battery_protect_ut] g_low_battery_level=%d\n", g_low_battery_level);
    return sprintf(buf, "%u\n", g_low_battery_level);
}
static ssize_t store_low_battery_protect_ut(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_low_battery_protect_ut] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_low_battery_protect_ut] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if(val<=2)
        {
            PMICLOG("[store_low_battery_protect_ut] your input is %d\n", val);
            exec_low_battery_callback(val);
        }
        else
        {
            PMICLOG("[store_low_battery_protect_ut] wrong number (%d)\n", val);
        }
    }
    return size;
}
static DEVICE_ATTR(low_battery_protect_ut, 0664, show_low_battery_protect_ut, store_low_battery_protect_ut); //664

//==============================================================================
// low battery protect stop
//==============================================================================
static ssize_t show_low_battery_protect_stop(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_low_battery_protect_stop] g_low_battery_stop=%d\n", g_low_battery_stop);
    return sprintf(buf, "%u\n", g_low_battery_stop);
}
static ssize_t store_low_battery_protect_stop(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_low_battery_protect_stop] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_low_battery_protect_stop] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if( (val!=0) && (val!=1) )
            val=0;
        g_low_battery_stop = val;
        PMICLOG("[store_low_battery_protect_stop] g_low_battery_stop=%d\n", g_low_battery_stop);
    }
    return size;
}
static DEVICE_ATTR(low_battery_protect_stop, 0664, show_low_battery_protect_stop, store_low_battery_protect_stop); //664

//==============================================================================
// low battery protect level
//==============================================================================
static ssize_t show_low_battery_protect_level(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_low_battery_protect_level] g_low_battery_level=%d\n", g_low_battery_level);
    return sprintf(buf, "%u\n", g_low_battery_level);
}
static ssize_t store_low_battery_protect_level(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    PMICLOG("[store_low_battery_protect_level] g_low_battery_level=%d\n", g_low_battery_level);
   
    return size;
}
static DEVICE_ATTR(low_battery_protect_level, 0664, show_low_battery_protect_level, store_low_battery_protect_level); //664
#endif

#ifdef BATTERY_OC_PROTECT
//==============================================================================
// battery OC protect UT
//==============================================================================
static ssize_t show_battery_oc_protect_ut(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_oc_protect_ut] g_battery_oc_level=%d\n", g_battery_oc_level);
    return sprintf(buf, "%u\n", g_battery_oc_level);
}
static ssize_t store_battery_oc_protect_ut(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_battery_oc_protect_ut] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_battery_oc_protect_ut] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if(val<=1)
        {
            PMICLOG("[store_battery_oc_protect_ut] your input is %d\n", val);
            exec_battery_oc_callback(val);
        }
        else
        {
            PMICLOG("[store_battery_oc_protect_ut] wrong number (%d)\n", val);
        }
    }
    return size;
}
static DEVICE_ATTR(battery_oc_protect_ut, 0664, show_battery_oc_protect_ut, store_battery_oc_protect_ut); //664

//==============================================================================
// battery OC protect stop
//==============================================================================
static ssize_t show_battery_oc_protect_stop(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_oc_protect_stop] g_battery_oc_stop=%d\n", g_battery_oc_stop);
    return sprintf(buf, "%u\n", g_battery_oc_stop);
}
static ssize_t store_battery_oc_protect_stop(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_battery_oc_protect_stop] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_battery_oc_protect_stop] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if( (val!=0) && (val!=1) )
            val=0;
        g_battery_oc_stop = val;
        PMICLOG("[store_battery_oc_protect_stop] g_battery_oc_stop=%d\n", g_battery_oc_stop);
    }
    return size;
}
static DEVICE_ATTR(battery_oc_protect_stop, 0664, show_battery_oc_protect_stop, store_battery_oc_protect_stop); //664

//==============================================================================
// battery OC protect level
//==============================================================================
static ssize_t show_battery_oc_protect_level(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_oc_protect_level] g_battery_oc_level=%d\n", g_battery_oc_level);
    return sprintf(buf, "%u\n", g_battery_oc_level);
}
static ssize_t store_battery_oc_protect_level(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    PMICLOG("[store_battery_oc_protect_level] g_battery_oc_level=%d\n", g_battery_oc_level);
   
    return size;
}
static DEVICE_ATTR(battery_oc_protect_level, 0664, show_battery_oc_protect_level, store_battery_oc_protect_level); //664
#endif

#ifdef BATTERY_PERCENT_PROTECT
//==============================================================================
// battery percent protect UT
//==============================================================================
static ssize_t show_battery_percent_protect_ut(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_percent_protect_ut] g_battery_percent_level=%d\n", g_battery_percent_level);
    return sprintf(buf, "%u\n", g_battery_percent_level);
}
static ssize_t store_battery_percent_protect_ut(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_battery_percent_protect_ut] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_battery_percent_protect_ut] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if(val<=1)
        {
            PMICLOG("[store_battery_percent_protect_ut] your input is %d\n", val);
            exec_battery_percent_callback(val);
        }
        else
        {
            PMICLOG("[store_battery_percent_protect_ut] wrong number (%d)\n", val);
        }
    }
    return size;
}
static DEVICE_ATTR(battery_percent_protect_ut, 0664, show_battery_percent_protect_ut, store_battery_percent_protect_ut); //664

//==============================================================================
// battery percent protect stop
//==============================================================================
static ssize_t show_battery_percent_protect_stop(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_percent_protect_stop] g_battery_percent_stop=%d\n", g_battery_percent_stop);
    return sprintf(buf, "%u\n", g_battery_percent_stop);
}
static ssize_t store_battery_percent_protect_stop(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_battery_percent_protect_stop] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_battery_percent_protect_stop] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if( (val!=0) && (val!=1) )
            val=0;
        g_battery_percent_stop = val;
        PMICLOG("[store_battery_percent_protect_stop] g_battery_percent_stop=%d\n", g_battery_percent_stop);
    }
    return size;
}
static DEVICE_ATTR(battery_percent_protect_stop, 0664, show_battery_percent_protect_stop, store_battery_percent_protect_stop); //664

//==============================================================================
// battery percent protect level
//==============================================================================
static ssize_t show_battery_percent_protect_level(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_battery_percent_protect_level] g_battery_percent_level=%d\n", g_battery_percent_level);
    return sprintf(buf, "%u\n", g_battery_percent_level);
}
static ssize_t store_battery_percent_protect_level(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    PMICLOG("[store_battery_percent_protect_level] g_battery_percent_level=%d\n", g_battery_percent_level);
   
    return size;
}
static DEVICE_ATTR(battery_percent_protect_level, 0664, show_battery_percent_protect_level, store_battery_percent_protect_level); //664
#endif

#ifdef DLPT_FEATURE_SUPPORT
//==============================================================================
// DLPT UT
//==============================================================================
static ssize_t show_dlpt_ut(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_dlpt_ut] g_dlpt_val=%d\n", g_dlpt_val);
    return sprintf(buf, "%u\n", g_dlpt_val);
}
static ssize_t store_dlpt_ut(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_dlpt_ut] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_dlpt_ut] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,10);
        
        PMICLOG("[store_dlpt_ut] your input is %d\n", val);
        exec_dlpt_callback(val);        
    }
    return size;
}
static DEVICE_ATTR(dlpt_ut, 0664, show_dlpt_ut, store_dlpt_ut); //664

//==============================================================================
// DLPT stop
//==============================================================================
static ssize_t show_dlpt_stop(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_dlpt_stop] g_dlpt_stop=%d\n", g_dlpt_stop);
    return sprintf(buf, "%u\n", g_dlpt_stop);
}
static ssize_t store_dlpt_stop(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    U32 val = 0;
    
    PMICLOG("[store_dlpt_stop] \n");
    
    if(buf != NULL && size != 0)
    {
        PMICLOG("[store_dlpt_stop] buf is %s and size is %zu \n",buf,size);
        val = simple_strtoul(buf,&pvalue,16);
        if( (val!=0) && (val!=1) )
            val=0;
        g_dlpt_stop = val;
        PMICLOG("[store_dlpt_stop] g_dlpt_stop=%d\n", g_dlpt_stop);
    }
    return size;
}
static DEVICE_ATTR(dlpt_stop, 0664, show_dlpt_stop, store_dlpt_stop); //664

//==============================================================================
// DLPT level
//==============================================================================
static ssize_t show_dlpt_level(struct device *dev,struct device_attribute *attr, char *buf)
{
    PMICLOG("[show_dlpt_level] g_dlpt_val=%d\n", g_dlpt_val);
    return sprintf(buf, "%u\n", g_dlpt_val);
}
static ssize_t store_dlpt_level(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    PMICLOG("[store_dlpt_level] g_dlpt_val=%d\n", g_dlpt_val);
   
    return size;
}
static DEVICE_ATTR(dlpt_level, 0664, show_dlpt_level, store_dlpt_level); //664
#endif

//==============================================================================
// system function 
//==============================================================================

static unsigned long pmic_node = 0;

static int fb_early_init_dt_get_chosen(unsigned long node, const char *uname, int depth, void *data)
{
        if (depth != 1 ||
            (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
                return 0;
        pmic_node = node;
        return 1;
}


static int pmic_mt_probe(struct platform_device *dev)
{
	int ret_device_file = 0,i;    
	kal_uint8 *pimix;
	int len=0;
#ifdef MEIZU_M81
	U32 ret=0;
	U32 reg_val=0;
#endif

	if(of_scan_flat_dt(fb_early_init_dt_get_chosen, NULL) > 0)
	{
		pimix = of_get_flat_dt_prop(pmic_node, "atag,imix_r", &len);
	}


	if(pimix==NULL)
	{
		PMICLOG(" pimix==NULL len=%d \n",len);
	}
	else
	{
		PMICLOG(" pimix=%d \n",*pimix);
		ptim_rac_val_avg=*pimix;
	}

	PMICLOG("******** MT pmic driver probe!! ********%d \n",ptim_rac_val_avg);
	

	//get PMIC CID
	PMICLOG("PMIC CID=0x%x PowerGoodStatus=0x%x OCStatus=0x%x ThermalStatus=0x%x rsvStatus=0x%x\n", pmic_get_register_value(PMIC_SWCID),
	upmu_get_reg_value(0x21c),upmu_get_reg_value(0x214),upmu_get_reg_value(0x21e),upmu_get_reg_value(0x2a6));

	upmu_set_reg_value(0x2a6,0xff);

        //dump_ldo_status_read_debug();

#if defined(CONFIG_ARCH_MT6753)
		PMICLOG("[PMIC_INIT_SETTING_V1] delay to MT6311 init\n");
#else
		PMIC_INIT_SETTING_V1();
		PMICLOG("[PMIC_INIT_SETTING_V1] Done\n");
#endif 

	PMIC_CUSTOM_SETTING_V1();
	PMICLOG("[PMIC_CUSTOM_SETTING_V1] Done\n");

//#if defined(CONFIG_MTK_FPGA)
#if 0
	PMICLOG("[PMIC_EINT_SETTING] disable when CONFIG_MTK_FPGA\n");
#else
	//PMIC Interrupt Service
	pmic_thread_handle = kthread_create(pmic_thread_kthread, (void *) NULL, "pmic_thread");
	if (IS_ERR(pmic_thread_handle)) 
	{
	    pmic_thread_handle = NULL;
	    PMICLOG("[pmic_thread_kthread_mt6325] creation fails\n");        
	}
	else
	{
	    wake_up_process(pmic_thread_handle);
	    PMICLOG("[pmic_thread_kthread_mt6325] kthread_create Done\n");
	} 

	PMIC_EINT_SETTING();
	PMICLOG("[PMIC_EINT_SETTING] Done\n");
#endif

	#ifdef 	MEIZU_M81
	ret=pmic_read_interface(0xA7C,&reg_val,0xF,8);

	switch(reg_val){
     		case 0x0: ret=pmic_config_interface(0xA7C, 0xD, 0xF, 8); break;
     		case 0x1: ret=pmic_config_interface(0xA7C, 0xE, 0xF, 8); break;
     		case 0x2: ret=pmic_config_interface(0xA7C, 0xF, 0xF, 8); break;
		case 0x3: ret=pmic_config_interface(0xA7C, 0x0, 0xF, 8); break;
     		case 0x4: ret=pmic_config_interface(0xA7C, 0x1, 0xF, 8); break;
     		case 0x5: ret=pmic_config_interface(0xA7C, 0x2, 0xF, 8); break;
     		case 0x6: ret=pmic_config_interface(0xA7C, 0x3, 0xF, 8); break;
     		case 0x7: ret=pmic_config_interface(0xA7C, 0x4, 0xF, 8); break;
     		case 0x8:   break;
     		case 0x9:   break;
     		case 0xa:   break;
     		case 0xb:  ret=pmic_config_interface(0xA7C, 0x8, 0xF, 8); break;
     		case 0xc:  ret=pmic_config_interface(0xA7C, 0x9, 0xF, 8); break;
     		case 0xd:  ret=pmic_config_interface(0xA7C, 0xA, 0xF, 8); break;
     		case 0xe:  ret=pmic_config_interface(0xA7C, 0xB, 0xF, 8); break;
     
      		default:
         		PMICLOG("set VRF18 : argument invalid!!\r\n");
           		break;
    	}
	#endif

	mtk_regulator_init(dev);

#ifdef LOW_BATTERY_PROTECT
    low_battery_protect_init();
#else
	PMICLOG("[PMIC] no define LOW_BATTERY_PROTECT\n" );
#endif

#ifdef BATTERY_OC_PROTECT
    battery_oc_protect_init();
#else
	PMICLOG("[PMIC] no define BATTERY_OC_PROTECT\n" );
#endif

#ifdef BATTERY_PERCENT_PROTECT
    bat_percent_notify_init();
#else
    PMICLOG("[PMIC] no define BATTERY_PERCENT_PROTECT\n" );
#endif

#ifdef DLPT_FEATURE_SUPPORT
    dlpt_notify_init();
#else
    PMICLOG("[PMIC] no define DLPT_FEATURE_SUPPORT\n" );
#endif

#if 1 
	pmic_set_register_value(PMIC_AUXADC_CK_AON,1); 
	pmic_set_register_value(PMIC_RG_CLKSQ_EN_AUX_AP_MODE,1);
	PMICLOG("[PMIC] auxadc 26M test : Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
	        MT6328_AUXADC_CON0, upmu_get_reg_value(MT6328_AUXADC_CON0),
	        MT6328_TOP_CLKSQ, upmu_get_reg_value(MT6328_TOP_CLKSQ)	        
	        );
#endif

	pmic_debug_init();
	PMICLOG("[PMIC] pmic_debug_init : done.\n" );

	pmic_ftm_init();



	//EM BUCK voltage & Status
	for (i = 0; i < ARRAY_SIZE(mtk_bucks); i++) 
	{
		//PMICLOG("[PMIC] register buck id=%d \n",i);
		ret_device_file = device_create_file(&(dev->dev),&mtk_bucks[i].en_att);
		ret_device_file = device_create_file(&(dev->dev),&mtk_bucks[i].voltage_att);
	}	

	//EM ldo voltage & Status
	for (i = 0; i < ARRAY_SIZE(mtk_ldos); i++) 
	{
		//PMICLOG("[PMIC] register ldo id=%d \n",i);
		ret_device_file = device_create_file(&(dev->dev),&mtk_ldos[i].en_att);
		ret_device_file = device_create_file(&(dev->dev),&mtk_ldos[i].voltage_att);
	}	

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_pmic_access);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_pmic_dvt);

#ifdef LOW_BATTERY_PROTECT
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_low_battery_protect_ut);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_low_battery_protect_stop);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_low_battery_protect_level);
#endif	

#ifdef BATTERY_OC_PROTECT
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_oc_protect_ut);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_oc_protect_stop);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_oc_protect_level);
#endif	

#ifdef BATTERY_PERCENT_PROTECT
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_percent_protect_ut);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_percent_protect_stop);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_battery_percent_protect_level);
#endif	

#ifdef DLPT_FEATURE_SUPPORT
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_dlpt_ut);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_dlpt_stop);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_dlpt_level);
#endif	

	PMICLOG("[PMIC] device_create_file for EM : done.\n" );

	//pwrkey_sw_workaround_init();
	return 0;
}

static int pmic_mt_remove(struct platform_device *dev)
{
    PMICLOG("******** MT pmic driver remove!! ********\n" );

    return 0;
}

static void pmic_mt_shutdown(struct platform_device *dev)
{
    PMICLOG("******** MT pmic driver shutdown!! ********\n" );
}

static int pmic_mt_suspend(struct platform_device *dev, pm_message_t state)
{

    PMICLOG("******** MT pmic driver suspend!! ********\n" );

#ifdef LOW_BATTERY_PROTECT
	lbat_min_en_setting(0);
	lbat_max_en_setting(0);
	
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
		MT6328_AUXADC_LBAT3, upmu_get_reg_value(MT6328_AUXADC_LBAT3),
		MT6328_AUXADC_LBAT4, upmu_get_reg_value(MT6328_AUXADC_LBAT4),
		MT6328_INT_CON0, upmu_get_reg_value(MT6328_INT_CON0)
	);
#endif
	
#ifdef BATTERY_OC_PROTECT
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);
	
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
        MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
        MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
    );
#endif

    return 0;
}

static int pmic_mt_resume(struct platform_device *dev)
{

	PMICLOG("******** MT pmic driver resume!! ********\n" );

#ifdef LOW_BATTERY_PROTECT
	lbat_min_en_setting(0);
	lbat_max_en_setting(0);
	mdelay(1);

	if(g_low_battery_level==1)
	{
		lbat_min_en_setting(1);
		lbat_max_en_setting(1);
	}
	else if(g_low_battery_level==2)
	{
		//lbat_min_en_setting(0);
		lbat_max_en_setting(1);
	}
	else //0
	{
		lbat_min_en_setting(1);
		//lbat_max_en_setting(0);
	}
	
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
        MT6328_AUXADC_LBAT3, upmu_get_reg_value(MT6328_AUXADC_LBAT3),
        MT6328_AUXADC_LBAT4, upmu_get_reg_value(MT6328_AUXADC_LBAT4),
        MT6328_INT_CON0, upmu_get_reg_value(MT6328_INT_CON0)
    );
#endif
	
#ifdef BATTERY_OC_PROTECT
	bat_oc_h_en_setting(0);
	bat_oc_l_en_setting(0);
	mdelay(1);

	if(g_battery_oc_level==1)
		bat_oc_h_en_setting(1);
	else
		bat_oc_l_en_setting(1);
	
	PMICLOG("Reg[0x%x]=0x%x, Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\n", 
		MT6328_FGADC_CON23, upmu_get_reg_value(MT6328_FGADC_CON23),
		MT6328_FGADC_CON24, upmu_get_reg_value(MT6328_FGADC_CON24),
		MT6328_INT_CON2, upmu_get_reg_value(MT6328_INT_CON2)
	);
#endif

	return 0;
}

struct platform_device pmic_mt_device = {
    .name   = "mt-pmic",
    .id        = -1,
};

static struct platform_driver pmic_mt_driver = {
    .probe        = pmic_mt_probe,
    .remove       = pmic_mt_remove,
    .shutdown     = pmic_mt_shutdown,
    //#ifdef CONFIG_PM
    .suspend      = pmic_mt_suspend,
    .resume       = pmic_mt_resume,
    //#endif
    .driver       = {
        .name = "mt-pmic",
    },
};
static DEFINE_MUTEX(pmic_efuse_lock_mutex);
u32 pmic_Read_Efuse_HPOffset(int i)
{
    U32 ret = 0;
    U32 reg_val = 0;
    U32 efusevalue;

    printk("pmic_Read_Efuse_HPOffset(+)\n");
	mutex_lock(&pmic_efuse_lock_mutex);
    //1. enable efuse ctrl engine clock
    ret = pmic_config_interface(0x027C, 0x0040, 0xFFFF, 0);
    ret = pmic_config_interface(0x0252, 0x0004, 0xFFFF, 0);

    //2.
    ret = pmic_config_interface(0x0C16, 0x1, 0x1, 0);


    //3. set row to read
    ret = pmic_config_interface(0x0C00, i, 0x1F, 1);

    //4. Toggle
    ret = pmic_read_interface(0x0C10, &reg_val, 0x1, 0);
    if (reg_val == 0)
        {
            ret = pmic_config_interface(0x0C10, 1, 0x1, 0);
        }
        else
        {
            ret = pmic_config_interface(0x0C10, 0, 0x1, 0);
        }

     //5. polling Reg[0x61A]
     reg_val = 1;
     while (reg_val == 1)
        {
            ret = pmic_read_interface(0x0C1A, &reg_val, 0x1, 0);
            printk("pmic_Read_Efuse_HPOffset polling 0x61A=0x%x\n", reg_val);
        }

        udelay(1000);//Need to delay at least 1ms for 0x61A and than can read 0xC18

    //6. read data
    efusevalue = upmu_get_reg_value(0x0C18);
    printk("HPoffset : efuse=0x%x\n", efusevalue);
    //7. Disable efuse ctrl engine clock
    ret = pmic_config_interface(0x0250, 0x0004, 0xFFFF, 0);
    ret = pmic_config_interface(0x027A, 0x0040, 0xFFFF, 0);
	mutex_unlock(&pmic_efuse_lock_mutex);
    return efusevalue;
}

//==============================================================================
// PMIC mudule init/exit
//==============================================================================
static int __init pmic_mt_init(void)
{
	int ret;

	wake_lock_init(&pmicThread_lock, WAKE_LOCK_SUSPEND, "pmicThread_lock_mt6328 wakelock");
	wake_lock_init(&bat_percent_notify_lock, WAKE_LOCK_SUSPEND,"bat_percent_notify_lock wakelock");

#ifdef DLPT_FEATURE_SUPPORT
	wake_lock_init(&dlpt_notify_lock, WAKE_LOCK_SUSPEND,"dlpt_notify_lock wakelock");
#endif //#ifdef DLPT_FEATURE_SUPPORT

	// PMIC device driver register
	ret = platform_device_register(&pmic_mt_device);
	if (ret) {
		PMICLOG("****[pmic_mt_init] Unable to device register(%d)\n", ret);
		return ret;
	}
	ret = platform_driver_register(&pmic_mt_driver);
	if (ret) {
		PMICLOG("****[pmic_mt_init] Unable to register driver (%d)\n", ret);
		return ret;
	}

#if 0//#ifdef CONFIG_HAS_EARLYSUSPEND
	//register_early_suspend(&pmic_early_suspend_desc);
#endif

	pmic_auxadc_init();

	PMICLOG("****[pmic_mt_init] Initialization : DONE !!\n");

	return 0;
}

static void __exit pmic_mt_exit (void)
{}

fs_initcall(pmic_mt_init);

//module_init(pmic_mt_init);
module_exit(pmic_mt_exit);

MODULE_AUTHOR("WY Chuang");
MODULE_DESCRIPTION("MT PMIC Device Driver");
MODULE_LICENSE("GPL");

