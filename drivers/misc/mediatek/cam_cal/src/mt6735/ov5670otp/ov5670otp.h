/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ov5670otp.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   Ronnie Lai (MTK01420)
 *
 *============================================================================*/
#ifndef __CAM_CAL_H
#define __CAM_CAL_H

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define OV5670_SUNNY_DEVICE_ID	       0x6C
#define OV5670_OFILM_DEVICE_ID	       0x6C
#define OV5670_QTECH_DEVICE_ID	       0x6C
#define OV5670_AVC_DEVICE_ID	       0x6C
#define SUNNY_MODULE_ID	               0x01
#define OFILM_MODULE_ID	      	       0x05
#define QTECH_MODULE_ID	      	       0x06
#define AVC_MODULE_ID_A	      	       0x04
#define AVC_MODULE_ID_B      	       0x18
#define I2C_UNIT_SIZE                   1 //in byte
#define OTP_START_ADDR                 0x0


#endif /* __CAM_CAL_H */

