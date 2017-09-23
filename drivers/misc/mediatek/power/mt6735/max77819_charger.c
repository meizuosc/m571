 /*
  * MAX77819 charger driver
  *
  * Copyright (C) 2013 Maxim Integrated Product
  * Gyungoh Yoo <jack.yoo@maximintegrated.com>
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 and
  * only version 2 as published by the Free Software Foundation.
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include "max77819_pmic.h"

 // =========================Maxim77819=================================== //
 /* Register */
#define MAX77819_CHGINT			0x30
#define MAX77819_CHGINT_MASK		0x31
#define MAX77819_CHG_STAT		0x32
#define MAX77819_DC_BATT_DTLS		0x33
#define MAX77819_CHG_DTLS		0x34
#define MAX77819_DETAILS3		0x35
#define MAX77819_BAT2SOC_CTL		0x36
#define MAX77819_CHGCTL1		0x37
#define MAX77819_FCHGCRNT		0x38
#define MAX77819_TOPOFF			0x39
#define MAX77819_BATREG			0x3A
#define MAX77819_DCCRNT			0x3B
#define MAX77819_AICLCNTL		0x3C
#define MAX77819_RBOOST_CTL1		0x3D
#define MAX77819_CHGCTL2		0x3E
#define MAX77819_BATDET			0x3F
#define MAX77819_USBCHGCTL		0x40
#define MAX77819_MBATREGMAX		0x41
#define MAX77819_CHGCCMAX		0x42
#define MAX77819_RBOOST_CTL2		0x43
#define MAX77819_CHG_WDT_INT		0x44
#define MAX77819_CHG_WDT_INT_MASK	0x45
#define MAX77819_CHG_WDTC		0x46
#define MAX77819_CHG_WDT_CTL		0x47
#define MAX77819_CHG_WDT_DTLS		0x48

 /* MAX77819_CHGINT */
#define MAX77819_THM_I			(1 << 1)
#define MAX77819_BAT_I			(1 << 2)
#define MAX77819_CHG_I			(1 << 3)
#define MAX77819_DC_UVP_I		(1 << 4)
#define MAX77819_OVP_I			(1 << 5)
#define MAX77819_TOPOFF_I		(1 << 6)
#define MAX77819_AICLOTG_I		(1 << 7)

 /* MAX77819_CHGINT_MASK */
#define MAX77819_THM_M			(1 << 1)
#define MAX77819_BAT_M			(1 << 2)
#define MAX77819_CHG_M			(1 << 3)
#define MAX77819_DC_UVP_M		(1 << 4)
#define MAX77819_OVP_M			(1 << 5)
#define MAX77819_TOPOFF_M		(1 << 6)
#define MAX77819_AICLOTG_M		(1 << 7)

 /* MAX77819_CHGINT_STAT */
#define MAX77819_DC_V			(1 << 0)
#define MAX77819_THM_NOK		(1 << 1)
#define MAX77819_BAT_NOK		(1 << 2)
#define MAX77819_CHG_NOK		(1 << 3)
#define MAX77819_DCUVP_NOK		(1 << 4)
#define MAX77819_OVP_NOK		(1 << 5)
#define MAX77819_DCI_NOK		(1 << 6)
#define MAX77819_AICL_NOK		(1 << 7)

 /* MAX77819_DC_BATT_DTLS */
#define MAX77819_DC_AICL		(1 << 7)
#define MAX77819_DC_I			(1 << 6)
#define MAX77819_DC_OVP		(1 << 5)
#define MAX77819_DC_UVP			(1 << 4)
#define MAX77819_BAT_DTLS		(3 << 2)
#define MAX77819_BATDET_DTLS		(3 << 0)

 /* MAX77819_CHG_DTLS */
#define MAX77819_THM_DTLS		(7 << 5)
#define MAX77819_TOPOFF_DTLS		(1 << 4)
#define MAX77819_CHG_DTL		(0x0F << 0)

 /* MAX77819_DETAILS3 */
#define MAX77819_BAT2SYS		(1 << 4)
#define MAX77819_VPQUTH			(1 << 3)

 /* MAX77819_BAT2SOC_CTL */
#define MAX77819_BAT2SOC		(3 << 3)
#define MAX77819_BAT2SOCEN		(1 << 2)
#define MAX77819_TBAT2SOC		(3 << 0)

 /* MAX77819_CHGCTL1 */
#define MAX77819_SFO_DEBOUNCE_EN	(1 << 7)
#define MAX77819_SFO_DEBOUNCE_TMR	(3 << 5)
#define MAX77819_THM_DIS		(1 << 4)
#define MAX77819_JEITA_EN		(1 << 3)
#define MAX77819_BUCK_EN		(1 << 2)
#define MAX77819_CHGPROT		(3 << 0)

 /* MAX77819_FCHGCRNT */
#define MAX77819_FCHGTIME		(7 << 5)
#define MAX77819_CHGCC			(0x1F << 0)

 /* MAX77819_TOPOFF */
#define MAX77819_TOPOFFTIME		(7 << 5)
#define MAX77819_IFST2P8		(1 << 4)
#define MAX77819_ITOPOFF		(7 << 0)

 /* MAX77819_BATREG */
#define MAX77819_REGTEMP		(3 << 6)
#define MAX77819_CHGRSTRT		(1 << 5)
#define MAX77819_MBATREG		(0x0F << 1)
#define MAX77819_VICHG_GAIN		(1 << 0)

 /* MAX77819_DCCRNT */
#define MAX77819_DCILMT			(0x3F << 0)

 /* MAX77819_AICLCNTL */
#define MAX77819_AICL_RESET		(1 << 5)
#define MAX77819_AICL			(0x0F << 1)
#define MAX77819_DCMON_DIS		(1 << 0)

 /* MAX77819_RBOOST_CTL1 */
#define MAX77819_BSTSLEWRATE		(7 << 5)
#define MAX77819_RBFORCEPWM		(1 << 4)
#define MAX77819_RBOOSTEN		(1 << 0)

 /* MAX77819_CHGCTL2 */
#define MAX77819_DCILIM_EN		(1 << 7)
#define MAX77819_PREQCUR		(3 << 5)
#define MAX77819_CEN			(1 << 4)
#define MAX77819_QBATEN			(1 << 3)
#define MAX77819_VSYSREG		(7 << 0)

 /* MAX77819_BATDET */
#define MAX77819_STRONGPUENB		(1 << 7)
#define MAX77819_BATDETENB		(1 << 6)
#define MAX77819_TDEB_BATREM		(0x1F << 1)
#define MAX77819_TDEB_BATREMS		(1 << 0)

 /* MAX77819_USBCHGCTL */
#define MAX77819_DIS_TIMER0		(1 << 7)
#define MAX77819_USB_HICURRENT		(1 << 3)
#define MAX77819_USB_SUSPEND		(1 << 2)
#define MAX77819_LOW_BAT		(1 << 0)

 /* MAX77819_MBATREGMAX */
#define MAX77819_MBATREGMX		(0x0F << 0)

 /* MAX77819_CHGCCMAX */
#define MAX77819_CHGCCMX		(0x1F << 0)

 /* MAX77819_RBOOST_CTL2 */
#define MAX77819_VBYPSET		(0x7F << 0)

 /* MAX77819_CHG_WDT_INT */
#define MAX77819_DC_V_I			(1 << 7)
#define MAX77819_CHG_WDT_I		(1 << 4)
#define MAX77819_CHG_WDT_WRN_I		(1 << 0)

 /* MAX77819_CHG_WDT_INT_MASK */
#define MAX77819_DC_V_M			(1 << 7)
#define MAX77819_CHG_WDT_M		(1 << 4)
#define MAX77819_CHG_WDT_WRN_M		(1 << 0)

 /* MAX77819_CHG_WDTC */
#define MAX77819_CHG_WDTC_		(3 << 0)
#define MAX77819_CHG_WDT_I		(1 << 4)
#define MAX77819_CHG_WDT_WRN_I		(1 << 0)

 /* MAX77819_CHG_WDT_CTL */
#define MAX77819_CHG_WDT		(3 << 6)
#define MAX77819_CHG_CHG_WDT_EN		(1 << 0)

 /* MAX77819_CHG_WDT_DTLS */
#define MAX77819_CHG_WDT_STAT		(1 << 4)
#define MAX77819_CHG_WDT_WRN_STAT	(1 << 0)
 /*Charger initialization valute*/


extern int battery_charger_dis;
extern void max77819_charger_dump_register(void);

#define M2SH(m) ((m) & 0x0F ? ((m) & 0x03 ? ((m) & 0x01 ? 0 : 1) : ((m) & 0x04 ? 2 : 3)) : \
					((m) & 0x30 ? ((m) & 0x10 ? 4 : 5) : ((m) & 0x40 ? 6 : 7)))

struct max77819_charger_platform_data {
	int fast_charge_timer;			/* One of 0, 4, 5, 6, 7, 8, 9, 16Hr */
	int restart_threshold;			/* 150000uV or 200000uV */
	int current_limit;			/* 100000uA ~ 1875000uA */
	int topoff_current_threshold;		/* 50000uA ~ 400000uA */
	int topoff_timer;			/* 0min to 60min, -1 is for not-done */
	int voltage;				/* 3550000uV ~ 4400000uV */
	/* AICL control */
	bool enable_aicl;
	int aicl_voltage;			/* 3900000uV ~ 4800000uV */
	int aicl_threshold;			/* 100000uV or 200000uV */
	int prequal_current;			/* one of 100000, 200000, 300000, 400000uA */
};


static struct max77819_charger_platform_data max77819_charger_pdata = {
		.fast_charge_timer = 0,
					.restart_threshold = 150000,
					.current_limit = 1500000,
					.topoff_current_threshold = 150000, /* modified by lyf*/
					.topoff_timer	= 0,
					.voltage = 4350000,
					.enable_aicl = 0,       /* modified by liujin*/
					.aicl_voltage = 4500000,
					.aicl_threshold = 100000,
					.prequal_current = 400000,	/* modified by lyf*/
 };

static kal_uint32 max77819_charger_inited = KAL_FALSE;
//static kal_uint32 max77819_charger_enabled = KAL_FALSE;

static kal_uint32 last_chgcc;
static kal_uint32 last_inputcc;
static kal_uint32 last_cv_voltage;

kal_bool max77819_charger_is_locked(void)
{
    kal_uint8 val = 0;

    max77819_pmic_reg_read(MAX77819_CHGCTL1, &val);
    if ((val & MAX77819_CHGPROT) == MAX77819_CHGPROT) {
	return KAL_FALSE;
    } else
	return KAL_TRUE;
}
void max77819_otg_enable(bool on)
{
	if (on) {
		max77819_pmic_reg_update_bits(MAX77819_RBOOST_CTL1, (1 << 0), 1);//enable reverse boost
		max77819_pmic_reg_update_bits(MAX77819_BAT2SOC_CTL, (1 << 5), (1 << 5));//enable otg
	} else{
		max77819_pmic_reg_update_bits(MAX77819_BAT2SOC_CTL, (1 << 5), 0);//disable otg
		max77819_pmic_reg_update_bits(MAX77819_RBOOST_CTL1, (1 << 0), 0);//disable reverse boost
	}
}

void max77819_buck_enable(bool on)
{
	if (on) {
		max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, (1 << 2), (1 << 2));//enable buck
	} else{
		max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, (1 << 2), 0);//disable buck

	}

}
/*charger unlock*/
static int max77819_charger_unlock(void)
{
    int rc;

    rc = max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, MAX77819_CHGPROT, 3);
	printk("max77819_charger_unlock [%d]\n", rc);

    return rc;
}
/*charger lock*/
static int max77819_charger_lock(void)
{
    int rc;

    rc = max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, MAX77819_CHGPROT, 0);
	printk("max77819_charger_lock [%d]\n", rc);

    return rc;
}


 /*set charging cc current, the input value should be less than 2.0A*/
 int max77819_charger_set_current(void *data)
 {
	 kal_int32 chgcc = *(kal_int32 *)(data);
	 kal_uint8 val = 0;
	 int ret;
    //    chgcc = 150000;
	 printk("max77819_charger_set_current: %d\n", chgcc);

     if (last_chgcc == chgcc) {
	printk("max77819_charger_set_current: same chgcc, ignore it\n");
	return 0;
     }
	 /*set maximal charging current to */
	 max77819_pmic_reg_write(MAX77819_CHGCCMAX, 0x1F);


	if (chgcc < 25000)
		   val = 0;
	 else if (chgcc >= 25000 && chgcc <= 155000)
		   val = ((chgcc - 25000)/5000 + 0x1)<<(M2SH(MAX77819_CHGCC));
	 else if ((chgcc > 155000) && (chgcc < 180000))
		   val = 0x1b;
	 else if ((chgcc >= 180000) && (chgcc < 186700))
		   val = 0x1c;
	 else if ((chgcc >= 186700) && (chgcc < 193300))
		   val = 0x1d;
     else if ((chgcc >= 193300) && (chgcc < 200000))
		   val = 0x1e;
	else if (chgcc == 200000)
		   val = 0x1f;
     else
	      val = 0x06; //default 500mA

    // val |= 0 << M2SH(MAX77819_FCHGTIME);//disable timer


#if 0
	if (chgcc == 50000)
	val |= 0x06; /* 500mA */
     else
	val |= 0x1A; /* 1500mA */
#endif
    ret = max77819_charger_unlock();
    ret |= max77819_pmic_reg_update_bits(MAX77819_FCHGCRNT, MAX77819_CHGCC, val);
    ret |= max77819_charger_lock();

     last_chgcc = chgcc;
	if (ret < 0) {
		   printk(KERN_INFO "Failed to set FCHGCRNT: %d\n", ret);
		   last_chgcc = 0;
		return -1;
	}

     return ret;
 }



 /*get charging cc current*/
 int max77819_charger_get_current(void *data)
 {
	 kal_uint8 chgcc = 0;
	 int ret;

/* charger lock protection, start */
	 max77819_charger_unlock();
	 ret = max77819_pmic_reg_read(MAX77819_FCHGCRNT, &chgcc);
	 max77819_charger_lock();
/* charger lock protection, end */
	if (chgcc == 0)
		 *(kal_int32 *)(data) = 0;
	else if (chgcc >= 0x01 && chgcc <= 0x1b)
		 *(kal_int32 *)(data) = 25000 + 5000*(chgcc - 1);
	else if (chgcc == 0x1c)
		 *(kal_int32 *)(data) = 180000;
	else if (chgcc == 0x1d)
		 *(kal_int32 *)(data) = 186700;
	else if (chgcc == 0x1e)
		 *(kal_int32 *)(data) = 193300;
	else if (chgcc == 0x1f)
		 *(kal_int32 *)(data) = 200000;
	if (ret < 0) {
		 printk("Failed to set FCHGCRNT: %d\n", ret);
		return -1;
	}

     return ret;
 }
 /*set DC input current limitation,input would be mA*/
 int max77819_charger_set_input_current(void *data)
 {
	 kal_int32 uA = *(kal_int32 *)(data);
	 kal_uint8 val = 0;
     int ret;
    //uA = 150000;

	 printk("max77819_charger_set_input_current: %d\n", uA);

     if (last_inputcc == uA) {
	printk("max77819_charger_set_input_current: same inputcc, ignore it\n");
	return 0;
     }

  //      uA = 100000;

	if (uA == 10000)
		 val = 0 << M2SH(MAX77819_DCILMT);
	else if (uA >= 27500 && uA <= 150000)
		   val = ((0x3+((uA - 27500) / 2500)) << M2SH(MAX77819_DCILMT));
     else if (uA > 150000 && uA < 170900)
		   val = 0x34;
     else if (uA >= 170900 && uA < 175000)
		   val = 0x35;
     else if (uA >= 175000 && uA < 179200)
		   val = 0x36;
     else if (uA >= 179200 && uA < 183400)
		   val = 0x37;
	 else if (uA >= 183400 && uA < 187500)
		   val = 0x38;
	 else if (uA >= 187500 && uA < 191700)
		   val = 0x39;
     else if (uA >= 191700 && uA < 195900)
		   val = 0x3A;
     else if (uA >= 195900 && uA < 200000)
		   val = 0x3B;
	else if (uA >= 200000 && uA < 204200)
		   val = 0x3C;
	else if (uA >= 204200 && uA < 208400)
				 val = 0x3D;
	else if (uA >= 208400)
				 val = 0x3E;
     else if (uA == -1)
	      val = 0x3F << M2SH(MAX77819_DCILMT);



#if 0
     if (uA == 50000)
	val = 0x0C; /* 500mA */
     else
	val = 0x34; /* 1500mA */
#endif
/*  charger lock protection, start */
	   max77819_charger_unlock();
     ret = max77819_pmic_reg_write(MAX77819_DCCRNT, val);
	  max77819_charger_lock();
 /*charger lock protection, end */
      if (ret < 0) {
		printk(KERN_INFO "Failed to set input current: %d\n", ret);
		last_inputcc = 0;
		return -1;
	}

     last_inputcc = uA;
     return ret;
 }

 /*disable or enable charger */
int max77819_charger_enable(void *data)
 {
	 int ret;
     kal_uint32 enable = *(kal_uint32 *)(data);

	 printk("max77819_charger_enable: %d\n", enable);

	 max77819_charger_unlock();
	 ret = max77819_pmic_reg_update_bits(MAX77819_CHGCTL2, MAX77819_CEN, enable ? MAX77819_CEN : 0);
	 max77819_charger_lock();

	if (battery_charger_dis) {
	   printk(KERN_EMERG "max77819_charger_enable: battery_charger_dis == 1\n");
	    max77819_buck_enable(false);
	//    max77819_charger_inited = KAL_FALSE;
	//    last_chgcc = 0;
	//   last_inputcc = 0;
	//   last_cv_voltage = 0;
	} else {
	 printk(KERN_EMERG "max77819_charger_enable: battery_charger_dis == 0\n");
	    max77819_buck_enable(true);
	}
	    max77819_charger_dump_register();
     if (enable == KAL_FALSE) {
	max77819_charger_inited = KAL_FALSE;
	last_chgcc = 0;
	last_inputcc = 0;
	last_cv_voltage = 0;
     }
	return ret;
 }
 /*set battery regulation voltage*/
int max77819_charger_set_cv_voltage(void *data)
 {
	 int ret;
	 kal_uint32 vol = *(kal_uint32 *)(data);
	 kal_uint8 val = 0;

	 printk("max77819_charger_set_cv_voltage: %d\n", vol);
	 /*set battery regulation control register-->register address:0x3A*/
     if (last_cv_voltage == vol) {
	printk("max77819_charger_set_cv_voltage: same cv_voltage, ignore it\n");
	return 0;
     }

	switch (vol) {
	case 3550000:
		 val = 0 << M2SH(MAX77819_MBATREG);
		break;
	case 3700000 ... 4400000:
		 val = ((vol - 3700000) / 50000 + 1) << M2SH(MAX77819_MBATREG);
		break;
	default:
		 printk("Invalid voltage: %d\n", vol);
		return -1;
	}

	switch (max77819_charger_pdata.restart_threshold) {
	case 150000:
		val |= 0 << M2SH(MAX77819_CHGRSTRT);
		break;
	case 200000:
		val |= 1 << M2SH(MAX77819_CHGRSTRT);
		break;
	default:
		printk("Invalid restart_threshold: %d\n", max77819_charger_pdata.restart_threshold);
		return -1;
	}
/*  charger lock protection, start */
	 max77819_charger_unlock();
	 ret = max77819_pmic_reg_write(MAX77819_BATREG, val);
	 max77819_charger_lock();

 /* charger lock protection, end */
    if (ret < 0) {
		   printk(KERN_INFO "Failed to set cv: %d\n", ret);
		   last_cv_voltage = 0;
		return -1;
	}
    last_cv_voltage = vol;
	return ret;
 }

 int max77819_charger_get_charging_status(void *data)
 {
	int ret = 0;
	int value = 0;


	ret = max77819_pmic_reg_read(MAX77819_CHG_DTLS, &value);
	if (ret < 0)
		 printk("cannot read CHG_DTLS : %d\n", ret);
	 printk("max77819_charger_get_charging_status:0x%x\n", (value & MAX77819_CHG_DTL) >> M2SH(MAX77819_CHG_DTL));
	switch ((value & MAX77819_CHG_DTL) >> M2SH(MAX77819_CHG_DTL)) {
	case 0x1:	 /* prequal */
	case 0x2:	 /* CC */
	case 0x3:	 /* CV */
		return 0;
	case 0x4:	 /* topoff */
	case 0x5:	 /* done */
		return 1;
	case 0x0:	 /* dead batter */
	case 0x6:	 /* timer fault */
	case 0x7:	 /* temperature suspend */
	case 0x8:	 /* buck,charger off */
	case 0xB:	 /* USB suspend */
		return 0;
	case 0xA:	 /* OTG debounce time-out */
	case 0x9:	 /* thermal loop */
	default:
		return 0;
	}
 }
 /*init charger with inital parameters */
 int max77819_charger_hw_init(void *data)
 {
	  struct max77819_charger_platform_data *pdata = &max77819_charger_pdata;
	  int ret;
	  kal_uint8 val = 0, status = 0;

	  printk("max77819_charger_hw_init: start\n");
    printk("max77819_charger_hw_init: firstly check charging status,if in charging state, just do nothing\n");
    ret = max77819_pmic_reg_read(MAX77819_CHG_DTLS, &status);
    if (ret < 0)
       return -1;
    //jelphi
    val = (status & MAX77819_CHG_DTL) >> M2SH(MAX77819_CHG_DTL);

	if ((max77819_charger_inited == KAL_TRUE) && ((val == 0x01) || (val == 0x02) || (val == 0x03) || (val == 0x04) || (val == 0x05))) {

      printk(KERN_INFO "max77819_charger is in charging state ,init done already, return\n");
      return 0;
    }

     max77819_charger_inited == KAL_FALSE;

	 /* Unlock protected registers-->resister address :0x37 */
	 ret = max77819_pmic_reg_update_bits(MAX77819_CHGCTL1,
			 MAX77819_CHGPROT | MAX77819_THM_DIS | MAX77819_JEITA_EN,
			 MAX77819_CHGPROT | MAX77819_THM_DIS | MAX77819_JEITA_EN);


	 /*configure int mask -->register address:	0x31*/
	 val = MAX77819_THM_M | MAX77819_BAT_M | /* MAX77819_CHG_M | */
		 /*MAX77819_DC_UVP_M | */ MAX77819_OVP_M | /*MAX77819_TOPOFF_M | */
		 MAX77819_AICLOTG_M;
	 ret = max77819_pmic_reg_write(MAX77819_CHGINT_MASK, val);
	 /*configure automatic input current limitation--> 0x3C*/


	if (pdata->enable_aicl) {
	   /* check voltage,it should be 3.9v--4.8v*/
		if (unlikely(pdata->aicl_voltage < 3900000 || pdata->aicl_voltage > 4800000)) {
			 printk("Invalid aicl_voltage: %duA\n", pdata->aicl_voltage);
			return -1;
		}
	 /* check reset threshold below the voltage setting: eitther 100mv or 200mv*/
		if (unlikely(pdata->aicl_threshold != 100000 && pdata->aicl_threshold != 200000)) {
			 printk("Invalid aicl_threshold: %duA\n", pdata->aicl_threshold);
			return -1;
		}

		 val = ((((pdata->aicl_voltage - 3900000) / 100000) << M2SH(MAX77819_AICL)) |
				 (((pdata->aicl_threshold - 100000) / 100000) << M2SH(MAX77819_AICL_RESET)));
	} else
		 val = MAX77819_DCMON_DIS;
	 ret = max77819_pmic_reg_write(MAX77819_AICLCNTL, val);
	if (ret < 0) {
		 printk("Failed to set AICLCNTL: %d\n", ret);
		return -1;
	}

   /*set topoff curent threshold and topoff timer -->register address: 0x39*/

	if (unlikely(pdata->topoff_current_threshold < 50000 ||
			 pdata->topoff_current_threshold > 400000)) {
		 printk("Invalid topoff_current_threshold: %d\n",
				 pdata->topoff_current_threshold);
		return -1;
	}
	 /*topoff current threshold*/
	 val = ((pdata->topoff_current_threshold - 50000) / 50000) << M2SH(MAX77819_ITOPOFF);
	if (unlikely(pdata->topoff_timer < -1 || pdata->topoff_timer > 60)) {
		 printk("Invalid topoff_timer: %d\n", pdata->topoff_timer);
		return -1;
	} else if (pdata->topoff_timer == -1) /*never enter DONE state*/
		 val |= 7 << M2SH(MAX77819_TOPOFFTIME);
	else
		 val |= (pdata->topoff_timer / 10) << M2SH(MAX77819_TOPOFFTIME);
 /* charger lock protection, start */
     max77819_charger_unlock();
	 ret = max77819_pmic_reg_update_bits(MAX77819_TOPOFF, MAX77819_ITOPOFF | MAX77819_TOPOFFTIME, val);
	 max77819_charger_lock();
/* charger lock protection, end */
	if (ret < 0) {
		 printk("Failed to set TOPOFF: %d\n", ret);
		return -1;
	}

#if 1
   /*set battery regulation control register-->register address:0x3A*/
	switch (pdata->voltage) {
	case 3550000:
		 val = 0 << M2SH(MAX77819_MBATREG);
		break;
	case 3700000 ... 4400000:
		 val = ((pdata->voltage - 3700000) / 50000 + 1) << M2SH(MAX77819_MBATREG);
		break;
	default:
		 printk("Invalid voltage: %d\n", pdata->voltage);
		return -1;
	}
	 /*restart threshold should be 150000 uV or 200000uV only*/
	switch (pdata->restart_threshold) {
	case 150000:
		 val |= 0 << M2SH(MAX77819_CHGRSTRT);
		break;
	case 200000:
		 val |= 1 << M2SH(MAX77819_CHGRSTRT);
		break;
	default:
		 printk("Invalid restart_threshold: %d\n", pdata->restart_threshold);
		return -1;
	}
	 max77819_charger_unlock();
	 ret = max77819_pmic_reg_update_bits(MAX77819_BATREG, MAX77819_MBATREG | MAX77819_CHGRSTRT, val);
	 max77819_charger_lock();
	if (ret < 0) {
		 printk("Failed to set MAX77819_CHG_CNFG_04: %d\n", ret);
		return -1;
	}
#endif

	 /*set CHGCTL2-->register address:0x3e*/
	 /*set prequal current, enable input charge limitataton*/
	if (unlikely(pdata->prequal_current < 100000 || pdata->prequal_current > 400000)) {
		 printk("Invalid prequal_current: %d\n", pdata->prequal_current);
		return -1;
	}
	 val = ((pdata->prequal_current / 100000 - 1) << M2SH(MAX77819_PREQCUR)) | MAX77819_DCILIM_EN | 0x04; /* min Vsys 3.4V*/
/* charger lock protection, start */
	 max77819_charger_unlock();
	 ret = max77819_pmic_reg_update_bits(MAX77819_CHGCTL2,
			 MAX77819_PREQCUR | MAX77819_DCILIM_EN | MAX77819_QBATEN | MAX77819_VSYSREG, val);
	 max77819_charger_lock();
/* charger lock protection, end */
	if (ret < 0) {
		 printk("Failed to set MAX77819_CHG_CNFG_04: %d\n", ret);
		return -1;
	}
	    max77819_pmic_reg_write(0x3d, 0x20);
	 max77819_pmic_reg_write(0x36, 0x00);
	 max77819_pmic_reg_write(0x40, 0x00);
	    max77819_pmic_reg_write(0x41, 0x0f);
	 max77819_pmic_reg_write(0x42, 0x1f);
	    max77819_pmic_reg_write(0x43, 0x54);


	 #if 0
	 //test-------------------------------------------
	 max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, MAX77819_JEITA_EN, 1<<3);
	 max77819_pmic_reg_update_bits(MAX77819_CHGCTL1, MAX77819_THM_DIS, 1<<4);
	 max77819_pmic_reg_write(0x38, 0x50);
	 max77819_pmic_reg_write(0x3a, 0x16);
	 max77819_pmic_reg_write(0x3b, 0x34);
	 max77819_pmic_reg_write(0x3f, 0x6e);
	 max77819_pmic_reg_write(0x40, 0x08);
	 max77819_pmic_reg_write(0x41, 0x0f);
	 max77819_pmic_reg_write(0x42, 0x1f);
	 //test-------------------------------------------
	 #endif

     max77819_charger_inited = KAL_TRUE;
     last_chgcc = 0;
     last_inputcc = 0;
     last_cv_voltage = 0;
	   printk("max77819_charger_hw_init: end\n");

     return 0;
 }
/*get aicl flag status*/
bool  max77819_charger_get_aicl_status(void)
 {
	 kal_uint8 flag = 0;
	    bool aicl_on = false;
	 int ret;

	 ret = max77819_pmic_reg_read(MAX77819_CHG_STAT, &flag);
	if (ret < 0) {
		 printk("Failed to get AICL flag status: %d\n", ret);
		return -1;
	}

	  aicl_on = flag &  0x80;

	  printk(KERN_EMERG "max77819_charger_get_aicl_status:  aici_on = %d\n", aicl_on);

	return aicl_on;
 }

void max77819_adjust_input_current(void)
{
    kal_uint32 reg_inputcc = 0;
    kal_uint32 current_inputcc = 0;

    max77819_pmic_reg_read(MAX77819_DCCRNT, &reg_inputcc);

    current_inputcc = reg_inputcc & 0x3f;  //bit: 0-5
    current_inputcc -= 12; //300mA
    printk(KERN_EMERG "max77819_adjust_input_current: current_inputcc=%x\n", current_inputcc);
    if (current_inputcc <= 0X0c)//threshold :500mA
	current_inputcc = 0x0c;

    reg_inputcc = (reg_inputcc & (!0x3f))|current_inputcc;
    printk(KERN_EMERG "max77819_adjust_input_current: reg_inputcc(0x3b)=%x\n", reg_inputcc);
    max77819_pmic_reg_write(MAX77819_DCCRNT, reg_inputcc);

}

void max77819_aicl_control(void)
{
	      kal_uint32 reg_chgcc = 0;
	      kal_uint32 reg_inputcc = 0;
	      kal_uint32 current_chgcc = 0;
	      kal_uint32 current_inputcc = 0;
		if (!max77819_charger_get_aicl_status())
		return  -1;


	     max77819_charger_unlock();
	 max77819_pmic_reg_read(MAX77819_FCHGCRNT, &reg_chgcc);
	    max77819_pmic_reg_read(MAX77819_DCCRNT, &reg_inputcc);
	    printk(KERN_EMERG " reg_chgcc(0x38) = %x, reg_inputcc(0x3b)=%x\n", reg_chgcc, reg_inputcc);
	    current_chgcc = reg_chgcc & 0x1f;  //bit: 0-4
	    current_inputcc = reg_inputcc & 0x3f;  //bit: 0-5
	    if ((current_chgcc <= 0x06) || (current_inputcc <= 0x0c))
		return;
	do {
		current_chgcc -= 1;
		current_inputcc -= 2;
		if (current_chgcc <= 0X06)
		       current_chgcc = 0x06;   //threshold :500mA
		if (current_inputcc <= 0X0c)//threshold :500mA
		       current_inputcc = 0x0c;

		 reg_chgcc = (reg_chgcc & (!0x1f))|current_chgcc;
		 reg_inputcc = (reg_inputcc & (!0x3f))|current_inputcc;
	   //      max77819_pmic_reg_write(MAX77819_FCHGCRNT, reg_chgcc);
		 max77819_pmic_reg_write(MAX77819_DCCRNT, reg_inputcc);
		 msleep(50);
	} while (max77819_charger_get_aicl_status() &&  !(current_inputcc = 0x0c));

	    max77819_adjust_input_current();

	      max77819_charger_lock();
}

void max77819_vol_control(void)
{
	      kal_uint32 reg_chgcc = 0;
	      kal_uint32 reg_inputcc = 0;
	      kal_uint32 current_chgcc = 0;
	      kal_uint32 current_inputcc = 0;


	     max77819_charger_unlock();
	 max77819_pmic_reg_read(MAX77819_FCHGCRNT, &reg_chgcc);
	    max77819_pmic_reg_read(MAX77819_DCCRNT, &reg_inputcc);
	    printk(KERN_EMERG " reg_chgcc(0x38) = %x, reg_inputcc(0x3b)=%x\n", reg_chgcc, reg_inputcc);
	    current_chgcc = reg_chgcc & 0x1f;  //bit: 0-4
	    current_inputcc = reg_inputcc & 0x3f;  //bit: 0-5
	if ((current_chgcc <= 0x06) || (current_inputcc <= 0x0c))
		return;

		current_chgcc = 0x10; // 1A
		current_inputcc = 0x20;// 1A

		 reg_chgcc = (reg_chgcc & (!0x1f))|current_chgcc;
		 reg_inputcc = (reg_inputcc & (!0x3f))|current_inputcc;
		//max77819_pmic_reg_write(MAX77819_FCHGCRNT, reg_chgcc);
		 max77819_pmic_reg_write(MAX77819_DCCRNT, reg_inputcc);

	      max77819_charger_lock();
}

 void max77819_charger_dump_register(void)
 {
		  kal_uint8 reg_val = 0;
		  int i = 0;
		  int ret;

		for (i = 0x30 ; i <= 0x4b; i++)
				ret = max77819_pmic_reg_read(i, &reg_val);
 }

 // =========================Maxim77819=================================== //




