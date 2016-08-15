/* linux/drivers/hwmon/lis33de.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * MAX77819_CH driver for MT6516
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef MAX77819_CH_H
#define MAX77819_CH_H 
	 
#include <linux/ioctl.h>
#include <mach/mt_typedefs.h>

#define MAX77819_CH_I2C_SLAVE_ADDR		0x90


#define WLED_MAX_BRIGHTNESS     0xFF

/* registers */
#define MAX77819_CH_WLED_INT		0x9B
#define MAX77819_CH_WLED_INT_MASK	0x9C
#define MAX77819_CH_WLEDBSTCNTL	0x98
#define MAX77819_CH_IWLED			0x99

/* MAX77819_CH_WLED_INT */
#define MAX77819_CH_WLEDOL			0x10
#define MAX77819_CH_WLEDOVP_I		0x80

/* MAX77819_CH_WLED_INT_MASK */
#define MAX77819_CH_WLEDOL_M		0x10
#define MAX77819_CH_WLEDOVP_M		0x80

/* MAX77819_CH_WLEDBSTCNTL */
#define MAX77819_CH_WLEDOVP		0x02
#define MAX77819_CH_WLEDFOSC		0x0C
#define MAX77819_CH_WLEDPWM2EN		0x10
#define MAX77819_CH_WLEDPWM1EN		0x20
#define MAX77819_CH_WLED2EN		0x40
#define MAX77819_CH_WLED1EN		0x80

/* MAX77819_CH_IWLED */
#define MAX77819_CH_CUR			0xFF

	 
	 
#define MAX77819_CH_SUCCESS						0
#define MAX77819_CH_ERR_I2C						-1
#define MAX77819_CH_ERR_STATUS					-3
#define MAX77819_CH_ERR_SETUP_FAILURE			-4
#define MAX77819_CH_ERR_GETGSENSORDATA			-5
#define MAX77819_CH_ERR_IDENTIFICATION			-6
	 
	 
	 
#define MAX77819_CH_BUFSIZE				256

extern int max77819_pmic_reg_write(kal_uint8 reg, kal_uint8 val);
extern int max77819_pmic_reg_read(kal_uint8 reg, kal_uint8 *val);
extern int max77819_pmic_reg_update_bits(kal_uint8 reg, kal_uint8 mask, kal_uint8 val);

extern void max77819_flash_deinit(void);
extern void max77819_flash_disable(void);
extern void max77819_flash_enable(void);
extern void max77819_flash_init(void);
extern void max77819_flash_preOn(void);
extern void max77819_flash_set_duty(unsigned char duty);

#endif

