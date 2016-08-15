
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 meizu_otp.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *       meizu add for otp struct define
 *       author:lcz
 ****************************************************************************/
#ifndef _MEIZU_OTP_H
#define _MEIZU_OTP_H

typedef struct sunny_otp_struct{
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int lens_id;
	int IR_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}sunny_otp_struct;

typedef struct film_otp_struct {
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}ofilm_otp_struct;

typedef struct tech_otp_struct {
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int IR_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}qtech_otp_struct;

typedef struct avc_otp_struct {
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}avc_otp_struct;
#endif
