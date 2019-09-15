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

#include <linux/uaccess.h>

#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_gpio.h>
#include <mach/mtk_rtc.h>
#include <mach/mt_spm_mtcmos.h>

#include <mach/battery_common.h>
#include <linux/time.h>
//#include <mach/pmic_mt6328_sw.h>

#include <cust_pmic.h>
#include <cust_battery_meter.h>


//==============================================================================
// Extern
//==============================================================================
extern kal_uint16 mt6328_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern kal_uint16 mt6328_get_register_value(PMU_FLAGS_LIST_ENUM flagname);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val);

//==============================================================================
// PMIC-AUXADC related define
//==============================================================================
#define VOLTAGE_FULL_RANGE     	1800
#define VOLTAGE_FULL_RANGE_6311	3200
#define ADC_PRECISE		32768 	// 15 bits
#define ADC_PRECISE_CH7		131072 	// 17 bits
#define ADC_PRECISE_6311	4096 	// 12 bits

//==============================================================================
// PMIC-AUXADC global variable
//==============================================================================

#define PMICTAG                "[Auxadc] "
#define PMICLOG(fmt, arg...)   printk(PMICTAG fmt,##arg)

kal_int32 count_time_out=15;
struct wake_lock pmicAuxadc_irq_lock;
//static DEFINE_SPINLOCK(pmic_adc_lock);
static DEFINE_MUTEX(pmic_adc_mutex);

void pmic_auxadc_init(void)
{
	kal_int32 adc_busy;
	wake_lock_init(&pmicAuxadc_irq_lock, WAKE_LOCK_SUSPEND, "pmicAuxadc irq wakelock");

	mt6328_set_register_value(PMIC_AUXADC_AVG_NUM_LARGE,6); // 1.3ms
	mt6328_set_register_value(PMIC_AUXADC_AVG_NUM_SMALL,2); // 0.8ms	

	mt6328_set_register_value(PMIC_AUXADC_AVG_NUM_SEL,0x83); // 0.8ms

	mt6328_set_register_value(PMIC_RG_VBUF_EN,0x1);

	PMICLOG("****[pmic_auxadc_init] DONE\n");
}

void pmic_auxadc_lock(void)
{
	wake_lock(&pmicAuxadc_irq_lock);
	mutex_lock(&pmic_adc_mutex);
}

void pmic_auxadc_unlock(void)
{
	wake_unlock(&pmicAuxadc_irq_lock);
	mutex_unlock(&pmic_adc_mutex);
}


extern kal_int32 g_I_SENSE_offset;
kal_int32 PMIC_IMM_GetCurrent(void)
{
	kal_int32 ret=0;
	int count=0;
	kal_int32 batsns,isense;
	kal_int32 ADC_I_SENSE=1;   // 1 measure time
	kal_int32 ADC_BAT_SENSE=1;	// 1 measure time
	kal_int32 ICharging=0;

	mt6328_set_register_value(PMIC_AUXADC_CK_AON,1);	
	mt6328_set_register_value(PMIC_RG_VBUF_EN,1);
	mt6328_set_register_value(PMIC_RG_CLKSQ_EN_AUX_AP_MODE,1);

	wake_lock(&pmicAuxadc_irq_lock);
	mutex_lock(&pmic_adc_mutex);
	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,0x3,0xffff,0);

	while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH0_BY_AP) != 1 )
	{
		msleep(1);
		if( (count++) > count_time_out)
		{
		PMICLOG("[PMIC_IMM_GetCurrent] batsns Time out!\n");
		break;
		}			
	}
	batsns = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH0_BY_AP); 	

	while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH1_BY_AP) != 1 )
	{
		msleep(1);
		if( (count++) > count_time_out)
		{
		PMICLOG("[PMIC_IMM_GetCurrent] isense Time out!\n");
		break;
		}			
	}
	isense = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH1_BY_AP);


	 ADC_BAT_SENSE=(batsns*3*VOLTAGE_FULL_RANGE)/32768;
	 ADC_I_SENSE=(isense*3*VOLTAGE_FULL_RANGE)/32768;

	 ICharging = (ADC_I_SENSE - ADC_BAT_SENSE + g_I_SENSE_offset)*1000/CUST_R_SENSE;


	wake_unlock(&pmicAuxadc_irq_lock);
	mutex_unlock(&pmic_adc_mutex);

	PMICLOG("[PMIC_IMM_GetCurrent(share lock)] raw batsns:%d raw isense:%d batsns:%d isense:%d iCharging:%d cnt:%d\n", batsns,isense,ADC_BAT_SENSE,ADC_I_SENSE,ICharging,count);

	return ICharging;

}



//==============================================================================
// PMIC-AUXADC 
//==============================================================================
kal_uint32 PMIC_IMM_GetOneChannelValue(mt6328_adc_ch_list_enum dwChannel, int deCount, int trimd)
{
	kal_int32 ret=0;
	kal_int32 ret_data;    
	kal_int32 r_val_temp=0;   
	kal_int32 adc_result=0;   
	int count=0;
	kal_uint32 busy;
	/*
	CH0: BATSNS
	CH1: ISENSE
	CH2: VCDT
	CH3: BAT ON
	CH4: PMIC TEMP
	CH5: ACCDET
	CH6:
	CH7: TSX
	CH8:
	CH9:
	CH10:
	CH11:
	CH12:
	CH13:
	CH14:
	CH15:
	BATSNS 3v-4.5v
	ISENSE 1.5-4.5v
	BATON  0-1.8v
	VCDT   4v-14v
	ACCDET 1.8v 
	GPS    1.8v
	
	*/

	if(dwChannel>15)
		return -1;

        //upmu_set_reg_value(0x28c,0x0002);

	mt6328_set_register_value(PMIC_AUXADC_CK_AON,1);	
        mt6328_set_register_value(PMIC_RG_VBUF_EN,1);
        mt6328_set_register_value(PMIC_RG_CLKSQ_EN_AUX_AP_MODE,1);

	//upmu_set_reg_value(0x0a44,0x010a);
	

	wake_lock(&pmicAuxadc_irq_lock);
	mutex_lock(&pmic_adc_mutex);
	//ret=pmic_config_interface(MT6328_TOP_CLKSQ_SET,(1<<2),0xffff,0);
	ret=pmic_config_interface(MT6328_AUXADC_RQST0_SET,(1<<dwChannel),0xffff,0);
	

	busy=upmu_get_reg_value(0E84);
	udelay(50);

	switch(dwChannel){         
	case 0:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH0_BY_AP) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH0_BY_AP); 	
	break;
	case 1:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH1_BY_AP) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH1_BY_AP);		
	break;
	case 2:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH2) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH2);				
	break;
	case 3:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH3) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH3);			
	break;
	case 4:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH4) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH4);				
	break;
	case 5:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH5) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH5);			
	break;
	case 6:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH6) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH6);				
	break;
	case 7:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_AP) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_AP);				
	break;
	case 8:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH8) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH8);				
	break;
	case 9:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH9) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH9);			
	break;
	case 10:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH10) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH10);			
	break;
	case 11:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH11) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH11);			
	break;
	case 12:    
	case 13:    
	case 14:    
	case 15:    		
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH12_15) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH12_15);				
	break;
	

	default:
		PMICLOG("[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
		wake_unlock(&pmicAuxadc_irq_lock);
		mutex_unlock(&pmic_adc_mutex);
	return -1;
	break;
	}

    switch(dwChannel){         
        case 0:                
            r_val_temp = 3;           
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;
        case 1:    
            r_val_temp = 3;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;
        case 2:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 3:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 4:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 5:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 6:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 7:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;    
        case 8:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;    			
	case 9:    
	case 10:  
	case 11:  
	case 12:
	case 13:    
	case 14:  
	case 15:  
	case 16:		
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;  
        default:
            PMICLOG("[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
            wake_unlock(&pmicAuxadc_irq_lock);
			mutex_unlock(&pmic_adc_mutex);
            return -1;
            break;
    }

	wake_unlock(&pmicAuxadc_irq_lock);
	mutex_unlock(&pmic_adc_mutex);

       PMICLOG("[AUXADC] ch=%d raw=%d data=%d \n", dwChannel, ret_data,adc_result);

	//return ret_data;
	return adc_result;

}

kal_uint32 PMIC_IMM_GetOneChannelValueMD(kal_uint8 dwChannel, int deCount, int trimd)
{
	kal_int32 ret=0;
	kal_int32 ret_data;    
	kal_int32 r_val_temp=0;   
	kal_int32 adc_result=0;   
	int count=0;
	/*
	CH0: BATSNS
	CH1: ISENSE
	CH4: PMIC TEMP
	CH7: TSX by MD
	CH8: TSX by GPS

	*/

	if(dwChannel!=0 && dwChannel!=1 && dwChannel!=4 && dwChannel!=7 && dwChannel!=8)
		return -1;


	wake_lock(&pmicAuxadc_irq_lock);

	mutex_lock(&pmic_adc_mutex);
	ret=pmic_config_interface(MT6328_TOP_CLKSQ_SET,(1<<3),0xffff,0);
	ret=pmic_config_interface(MT6328_AUXADC_RQST1_SET,(1<<dwChannel),0xffff,0);
	mutex_unlock(&pmic_adc_mutex);

	udelay(10);

	switch(dwChannel){         
	case 0:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH0_BY_MD) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[PMIC_IMM_GetOneChannelValueMD] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH0_BY_MD); 		
	break;
	case 1:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH1_BY_MD) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[PMIC_IMM_GetOneChannelValueMD] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH1_BY_MD); 			
	break;
	case 4:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH4_BY_MD) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[PMIC_IMM_GetOneChannelValueMD] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH4_BY_MD); 				
	break;
	case 7:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_MD) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[PMIC_IMM_GetOneChannelValueMD] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_MD); 				
	break;
	case 8:    
		while(mt6328_get_register_value(PMIC_AUXADC_ADC_RDY_CH7_BY_GPS) != 1 )
		{
			msleep(1);
			if( (count++) > count_time_out)
			{
			PMICLOG("[PMIC_IMM_GetOneChannelValueMD] (%d) Time out!\n", dwChannel);
			break;
			}			
		}
		ret_data = mt6328_get_register_value(PMIC_AUXADC_ADC_OUT_CH7_BY_GPS); 				
	break;


	default:
		PMICLOG("[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
		wake_unlock(&pmicAuxadc_irq_lock);
	return -1;
	break;
	}

    switch(dwChannel){         
        case 0:                
            r_val_temp = 3;           
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;
        case 1:    
            r_val_temp = 3;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;
        case 4:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/4096;
            break;
        case 7:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;    
        case 8:    
            r_val_temp = 1;
            adc_result = (ret_data*r_val_temp*VOLTAGE_FULL_RANGE)/32768;
            break;    			
        default:
            PMICLOG("[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
            wake_unlock(&pmicAuxadc_irq_lock);
            return -1;
            break;
    }

	wake_unlock(&pmicAuxadc_irq_lock);

       PMICLOG("[AUXADC] PMIC_IMM_GetOneChannelValueMD ch=%d raw=%d data=%d\n", dwChannel, ret_data,adc_result);

	return ret_data;
	//return adc_result;

}


