/*****************************************************************************
*
* Filename:
* ---------
*   max77819.h
*
* Project:
* --------
*   Android
*
* Description:
* ------------
*   max77819 header file
*
* Author:
* -------
*
****************************************************************************/

#ifndef _max77819_SW_H_
#define _max77819_SW_H_

//#define HIGH_BATTERY_VOLTAGE_SUPPORT

extern void max77819_otg_enable(bool on);
extern kal_bool max77819_charger_is_locked(void);
extern int max77819_charger_set_current(void *data);
extern int max77819_charger_get_current(void *data);
extern int max77819_charger_set_input_current(void *data);
extern int max77819_charger_enable(void *data);
extern int max77819_charger_set_cv_voltage(void *data);
extern int max77819_charger_get_charging_status(void *data);
extern int max77819_charger_hw_init(void *data);
extern void max77819_charger_dump_register(void);



#endif // _max77819_SW_H_

