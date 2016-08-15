/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S-24CS64A.h
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
#define S5K3L2XXOTP_LITEON_DEVICE_ID	       0xA0
#define S5K3L2XXOTP_PRIMAX_DEVICE_ID	       0xA0
#define S5K3L2XXOTP_AVC_DEVICE_ID	       0xA0
#define I2C_UNIT_SIZE                            1 //in byte
#define OTP_START_ADDR                            0x0
#define PRIMAX_LSC_SIZE				1508
#define PRIMAX_LSC_START_ADDR			0x2C
#define LITEON_LSC_SIZE				932
#define LITEON_LSC_START_ADDR			0x1C
#define AVC_LSC_SIZE				1508
#define AVC_LSC_START_ADDR			0x2F


#endif /* __CAM_CAL_H */

