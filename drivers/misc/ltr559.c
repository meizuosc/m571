/* Lite-On LTR-559ALS Android / Linux Driver
 *
 * Copyright (C) 2013 Lite-On Technology Corp (Singapore)
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <asm/setup.h>
#include <linux/version.h>

#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <cust_eint.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <asm/unaligned.h>

#include <linux/meizu-sys.h>
#include <linux/meizu-sensors.h>

//#define LTR559_DEBUG 1
#ifdef LTR559_DEBUG
#define LOG_TAG_LTR559 "[ltr559]"
#define pr_info(format, arg...)         printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#define dev_err(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#define dev_info(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#define dev_dbg(dev, format, arg...)    printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#define dev_warn(dev, format, arg...)   printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#define dev_notice(dev, format, arg...) printk(KERN_EMERG LOG_TAG_LTR559 format , ## arg)
#endif

//#define LTR559_DEBUG_PUSH_TIME 0
#ifdef LTR559_DEBUG_PUSH_TIME
static struct timespec ltr559_time_before, ltr559_time_after;
static int ltr559_push_time_debug;
#endif

/* LTR-559 Registers */
#define LTR559_ALS_CONTR	0x80
#define LTR559_PS_CONTR		0x81
#define LTR559_PS_LED		0x82
#define LTR559_PS_N_PULSES	0x83
#define LTR559_PS_MEAS_RATE	0x84
#define LTR559_ALS_MEAS_RATE	0x85
#define LTR559_PART_ID		0x86
#define LTR559_MANUFACTURER_ID	0x87
#define LTR559_ALS_DATA_CH1_0	0x88
#define LTR559_ALS_DATA_CH1_1	0x89
#define LTR559_ALS_DATA_CH0_0	0x8A
#define LTR559_ALS_DATA_CH0_1	0x8B
#define LTR559_ALS_PS_STATUS	0x8C
#define LTR559_PS_DATA_0	0x8D
#define LTR559_PS_DATA_1	0x8E
#define LTR559_INTERRUPT	0x8F
#define LTR559_PS_THRES_UP_0	0x90
#define LTR559_PS_THRES_UP_1	0x91
#define LTR559_PS_THRES_LOW_0	0x92
#define LTR559_PS_THRES_LOW_1	0x93
#define LTR559_PS_OFFSET_1	0x94
#define LTR559_PS_OFFSET_0	0x95
#define LTR559_ALS_THRES_UP_0	0x97
#define LTR559_ALS_THRES_UP_1	0x98
#define LTR559_ALS_THRES_LOW_0	0x99
#define LTR559_ALS_THRES_LOW_1	0x9A
#define LTR559_INTERRUPT_PRST	0x9E
/* LTR-559 Registers */

#define SET_BIT 1
#define CLR_BIT 0

#define ALS 0
#define PS 1
#define ALSPS 2

//#define PS_W_SATURATION_BIT	3

/* address 0x80 */
#define ALS_MODE_ACTIVE	(1 << 0)
#define ALS_MODE_STDBY		(0 << 0)
#define ALS_SW_RESET		(1 << 1)
#define ALS_SW_NRESET		(0 << 1)
#define ALS_GAIN_1x		(0 << 2)
#define ALS_GAIN_2x		(1 << 2)
#define ALS_GAIN_4x		(2 << 2)
#define ALS_GAIN_8x		(3 << 2)
#define ALS_GAIN_48x	(6 << 2)
#define ALS_GAIN_96x	(7 << 2)
#define ALS_MODE_RDBCK			0
#define ALS_SWRT_RDBCK			1
#define ALS_GAIN_RDBCK			2
#define ALS_CONTR_RDBCK		3

/* address 0x81 */
#define PS_MODE_ACTIVE		(3 << 0)
#define PS_MODE_STDBY		(0 << 0)
#define PS_GAIN_16x			(0 << 2)
#define PS_GAIN_32x			(2 << 2)
#define PS_GAIN_64x			(3 << 2)
#define PS_SATUR_INDIC_EN	(1 << 5)
#define PS_SATU_INDIC_DIS	(0 << 5)
#define PS_MODE_RDBCK		0
#define PS_GAIN_RDBCK		1
#define PS_SATUR_RDBCK		2
#define PS_CONTR_RDBCK		3

/* address 0x82 */
#define LED_CURR_5MA		(0 << 0)
#define LED_CURR_10MA		(1 << 0)
#define LED_CURR_20MA		(2 << 0)
#define LED_CURR_50MA		(3 << 0)
#define LED_CURR_100MA		(4 << 0)
#define LED_CURR_DUTY_25PC		(0 << 3)
#define LED_CURR_DUTY_50PC		(1 << 3)
#define LED_CURR_DUTY_75PC		(2 << 3)
#define LED_CURR_DUTY_100PC	(3 << 3)
#define LED_PUL_FREQ_30KHZ		(0 << 5)
#define LED_PUL_FREQ_40KHZ		(1 << 5)
#define LED_PUL_FREQ_50KHZ		(2 << 5)
#define LED_PUL_FREQ_60KHZ		(3 << 5)
#define LED_PUL_FREQ_70KHZ		(4 << 5)
#define LED_PUL_FREQ_80KHZ		(5 << 5)
#define LED_PUL_FREQ_90KHZ		(6 << 5)
#define LED_PUL_FREQ_100KHZ	(7 << 5)
#define LED_CURR_RDBCK			0
#define LED_CURR_DUTY_RDBCK	1
#define LED_PUL_FREQ_RDBCK		2
#define PS_LED_RDBCK			3

/* address 0x84 */
#define PS_MEAS_RPT_RATE_50MS		(0 << 0)
#define PS_MEAS_RPT_RATE_70MS		(1 << 0)
#define PS_MEAS_RPT_RATE_100MS	(2 << 0)
#define PS_MEAS_RPT_RATE_200MS	(3 << 0)
#define PS_MEAS_RPT_RATE_500MS	(4 << 0)
#define PS_MEAS_RPT_RATE_1000MS	(5 << 0)
#define PS_MEAS_RPT_RATE_2000MS	(6 << 0)
#define PS_MEAS_RPT_RATE_10MS		(8 << 0)

/* address 0x85 */
#define ALS_MEAS_RPT_RATE_50MS	(0 << 0)
#define ALS_MEAS_RPT_RATE_100MS	(1 << 0)
#define ALS_MEAS_RPT_RATE_200MS	(2 << 0)
#define ALS_MEAS_RPT_RATE_500MS	(3 << 0)
#define ALS_MEAS_RPT_RATE_1000MS	(4 << 0)
#define ALS_MEAS_RPT_RATE_2000MS	(5 << 0)
#define ALS_INTEG_TM_100MS		(0 << 3)
#define ALS_INTEG_TM_50MS			(1 << 3)
#define ALS_INTEG_TM_200MS		(2 << 3)
#define ALS_INTEG_TM_400MS		(3 << 3)
#define ALS_INTEG_TM_150MS		(4 << 3)
#define ALS_INTEG_TM_250MS		(5 << 3)
#define ALS_INTEG_TM_300MS		(6 << 3)
#define ALS_INTEG_TM_350MS		(7 << 3)
#define ALS_MEAS_RPT_RATE_RDBCK	0
#define ALS_INTEG_TM_RDBCK			1
#define ALS_MEAS_RATE_RDBCK		2

/* address 0x86 */
#define PART_NUM_ID_RDBCK		0
#define REVISION_ID_RDBCK		1
#define PART_ID_REG_RDBCK		2

/* address 0x8C */
#define PS_DATA_STATUS_RDBCK		0
#define PS_INTERR_STATUS_RDBCK	1
#define ALS_DATA_STATUS_RDBCK		2
#define ALS_INTERR_STATUS_RDBCK	3
#define ALS_GAIN_STATUS_RDBCK		4
#define ALS_VALID_STATUS_RDBCK	5
#define ALS_PS_STATUS_RDBCK		6

/* address 0x8F */
#define INT_MODE_00					(0 << 0)
#define INT_MODE_PS_TRIG			(1 << 0)
#define INT_MODE_ALS_TRIG			(2 << 0)
#define INT_MODE_ALSPS_TRIG		(3 << 0)
#define INT_POLAR_ACT_LO			(0 << 2)
#define INT_POLAR_ACT_HI			(1 << 2)
#define INT_MODE_RDBCK				0
#define INT_POLAR_RDBCK			1
#define INT_INTERRUPT_RDBCK		2

/* address 0x9E */
#define ALS_PERSIST_SHIFT	0
#define PS_PERSIST_SHIFT	4
#define ALS_PRST_RDBCK		0
#define PS_PRST_RDBCK		1
#define ALSPS_PRST_RDBCK	2

#define PON_DELAY		600

#define ALS_MIN_MEASURE_VAL	0
#define ALS_MAX_MEASURE_VAL	65535
#define ALS_VALID_MEASURE_MASK	ALS_MAX_MEASURE_VAL
#define PS_MIN_MEASURE_VAL	0
#define PS_MAX_MEASURE_VAL	2047
#define PS_VALID_MEASURE_MASK   PS_MAX_MEASURE_VAL
#define LO_LIMIT			0
#define HI_LIMIT			1
#define LO_N_HI_LIMIT	2
#define PS_OFFSET_MIN_VAL		0
#define PS_OFFSET_MAX_VAL		1023
#define	FAR_VAL		1
#define	NEAR_VAL		0

#define DRIVER_VERSION "1.14"
#define PARTID 0x92
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "LTR559ALSPS"

#define ACT_INTERRUPT 1
#define AGC		1
#define PS_Dyn_Calib	0

/*
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define LTR559_IOCTL_MAGIC      'c'

/* IOCTLs for LTR559 device */
#define LTR559_IOCTL_PS_ENABLE		_IOR(LTR559_IOCTL_MAGIC, 1, int *)
#define LTR559_IOCTL_PS_GET_ENABLED	_IOW(LTR559_IOCTL_MAGIC, 2, int *)
#define LTR559_IOCTL_ALS_ENABLE		_IOR(LTR559_IOCTL_MAGIC, 3, int *)
#define LTR559_IOCTL_ALS_GET_ENABLED	_IOW(LTR559_IOCTL_MAGIC, 4, int *)

struct mz_light_ops
{
	int (*enable_device)(struct mz_light_data *mz_light);
	int (*disable_device)(struct mz_light_data *mz_light);
	int (*set_range)(struct mz_light_data *mz_light, int range);
	int (*get_range)(struct mz_light_data *mz_light);
	int (*set_pollrate)(struct mz_light_data *mz_light, int pollrate);
	int (*get_pollrate)(struct mz_light_data *mz_light);
	int (*get_data)(struct mz_light_data *mz_light, uint16_t *data_buf);
	int (*self_test)(struct mz_light_data *mz_light);
};

struct mz_light_data {

	struct device *dev;
	struct mz_light_ops *ops;

	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
	u8    *iio_buffer_data;
	struct workqueue_struct *iio_workq;
	struct delayed_work iio_work;
	atomic_t enabled;

	int      pollrate;
	int      odr;
	int      min_pollrate;
	int      samples_to_discard;
	void *   priv;
};


struct mz_proximity_ops
{
	int (*enable_device)(struct mz_proximity_data *mz_proximity);
	int (*disable_device)(struct mz_proximity_data *mz_proximity);
	int (*set_range)(struct mz_proximity_data *mz_proximity, int range);
	int (*get_range)(struct mz_proximity_data *mz_proximity);
	int (*set_pollrate)(struct mz_proximity_data *mz_proximity, int pollrate);
	int (*get_pollrate)(struct mz_proximity_data *mz_proximity);
	int (*get_data)(struct mz_proximity_data *mz_proximity, uint16_t *data_buf);
	int (*self_test)(struct mz_proximity_data *mz_proximity);
};

struct mz_proximity_data {

	struct device *dev;
	struct mz_proximity_ops *ops;

	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
	int trig_state;

	struct iio_dev *indio_dev_wk;
	struct iio_trigger *trig_wk;
	int trig_wk_state;

	u8    *iio_buffer_data;
	struct workqueue_struct *iio_workq;
	struct delayed_work iio_work;
	atomic_t enabled;
	int      bias;
	int      pollrate;
	int      odr;
	int      min_pollrate;
	int      irq_gpio;
	void *   priv;
};

struct LTR559_data {
	/* Device */
	struct i2c_client *i2c_client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct workqueue_struct *workqueue;
	struct early_suspend early_suspend;
	struct wake_lock ps_wake_lock;
	struct mutex bus_lock;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_suspend_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;
	struct delayed_work als_dwork;

	/* PS */
	uint8_t ps_enable_flag;
	uint8_t ps_suspend_enable_flag;
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;
	struct work_struct ps_work;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int irq;
	int gpio_int_no;
	int is_suspend;

	/* iio */
	int ps_state;
	int sensor_phone_calling;
	struct mz_light_data mz_light;
	struct mz_proximity_data mz_proximity;
};

struct LTR559_data *sensor_info;

#define	PS_MAX_INIT_KEPT_DATA_COUNTER		8
#define	PS_MAX_MOV_AVG_KEPT_DATA_CTR		7

uint16_t winfac1 = 85;
uint16_t winfac2 = 80;
uint16_t winfac3 = 31;
uint8_t eqn_prev = 0;
uint8_t ratio_old = 0;
uint16_t ps_init_kept_data[PS_MAX_INIT_KEPT_DATA_COUNTER];
uint16_t ps_ct_avg;
uint8_t ps_grabData_stage = 0;
uint32_t ftn_init;
uint32_t ftn_final;
uint32_t ntf_final;
uint16_t lux_val_prev = 0;
uint8_t ps_kept_data_counter = 0;
uint16_t ps_movavg_data[PS_MAX_MOV_AVG_KEPT_DATA_CTR];
uint8_t ps_movavg_data_counter = 0;
uint16_t ps_movct_avg;
//uint16_t ps_thresh_hi, ps_thresh_lo;

/* I2C Read */
// take note ---------------------------------------
// for i2c read, need to send the register address follwed by buffer over to register.
// There should not be a stop in between register address and buffer.
// There should not be release of lock in between register address and buffer.
// take note ---------------------------------------
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}

	return 0;
}

/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;

		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}

/* Set register bit */
static int8_t _LTR559_set_bit(struct i2c_client *client, uint8_t set,
		uint8_t cmd, uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}

static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc, uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	uint16_t ch0_coeff_i = 0;
	uint16_t ch1_coeff_i = 0;
	uint16_t ch0_coeff_f = 0;
	uint16_t ch1_coeff_f = 0;
	int8_t ret;
	uint8_t gain = 1, als_int_fac;
	uint8_t buffer[2];
	uint16_t win_fac = 0;
	int8_t fac = 1;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0) {
		als_int_fac = 10;
	} else if (als_int_fac == 1) {
		als_int_fac = 5;
	} else if (als_int_fac == 2) {
		als_int_fac = 20;
	} else if (als_int_fac == 3) {
		als_int_fac = 40;
	} else if (als_int_fac == 4) {
		als_int_fac = 15;
	} else if (als_int_fac == 5) {
		als_int_fac = 25;
	} else if (als_int_fac == 6) {
		als_int_fac = 30;
	} else if (als_int_fac == 7) {
		als_int_fac = 35;
	}

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = winfac1;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
				(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
				(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));
		//luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 696;
		win_fac = winfac2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) -
					(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) -
					(ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) -
				(ch1_adc * ch1_coeff_i)) * win_fac;
		//luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));
		//luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1300;
		fac = 1;
		win_fac = winfac3;
		luxval_i = ((ch0_adc * ch0_coeff_i) +
				(ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) +
				(ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));
		//luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		//luxval = 0;
	}

	if (fac < 0) {
		luxval = (luxval_i  - (luxval_f / 100)) / (gain * als_int_fac);
	} else {
		luxval = (luxval_i  + (luxval_f / 100)) / (gain * als_int_fac);
	}

	return luxval;
}

static uint16_t ratioHysterisis (uint16_t ch0_adc, uint16_t ch1_adc)
{
#define	RATIO_HYSVAL	10
	int ratio;
	uint8_t buffer[2], eqn_now;
	int8_t ret;
	uint16_t ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20) {
		ch0_calc = ch0_adc - ch1_adc;
	}

	if ((ch1_adc + ch0_calc) == 0) {
		ratio = 100;
	} else {
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);
	}

	if (ratio < 45) {
		eqn_now = 1;
	} else if ((ratio >= 45) && (ratio < 68)) {
		eqn_now = 2;
	} else if ((ratio >= 68) && (ratio < 99)) {
		eqn_now = 3;
	} else if (ratio >= 99) {
		eqn_now = 4;
	}

	if (eqn_prev == 0) {
		luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0) {
				abs_ratio_now_old *= (-1);
			}
			if (abs_ratio_now_old > RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_prev);
			}
		}
	}

	return luxval;
}

#if 0
void setWinFac (uint16_t *winfac, uint8_t *param_temp, size_t count)
{
	int winfacVal = 0;
	uint8_t u_ctr, deci_ctr = 0;

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { 				// 1 character
		if (param_temp[0] == '.') {
			param_temp[0] = 0;
		} else {
			param_temp[0] -= 48;
		}
		winfacVal = param_temp[0] * 10;
	} else if (count == 3) { 				// 2 characters
		for (u_ctr = 0; u_ctr < 2; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				winfacVal = param_temp[1] - 48;
			} else if (param_temp[1] == '.') {
				winfacVal = param_temp[0] - 48;
				winfacVal *= 10;
			} else {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[1];
				winfacVal *= 10;
			}
		}
	} else if (count == 4) { 				// 3 characters
		for (u_ctr = 0; u_ctr < 3; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				if (param_temp[2] >= 5) {
					if ((param_temp[1] >=0) && (param_temp[1] <= 8)) {
						param_temp[1]++;
						winfacVal = param_temp[1];
					} else {
						winfacVal = 10;
					}
				} else {
					winfacVal = param_temp[1];
				}
			} else if (param_temp[1] == '.') {
				param_temp[0] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[2];
			} else if (param_temp[2] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[1];
				winfacVal *= 10;
			}
		}
	} else if (count == 5) { 				// 4 characters
		for (u_ctr = 0; u_ctr < 3; u_ctr++) {
			if (param_temp[u_ctr] == '.')
				deci_ctr++;
		}

		if (deci_ctr <= 1) {
			if (param_temp[0] == '.') {
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				if (param_temp[2] >= 5) {
					if ((param_temp[1] >=0) && (param_temp[1] <= 8)) {
						param_temp[1]++;
						winfacVal = param_temp[1];
					} else {
						winfacVal = 10;
					}
				} else {
					winfacVal = param_temp[1];
				}
			} else if (param_temp[1] == '.') {
				param_temp[0] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 10) + param_temp[2];
			} else if (param_temp[2] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				param_temp[3] -= 48;
				winfacVal = (param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[3];
			} else if (param_temp[3] == '.') {
				param_temp[0] -= 48;
				param_temp[1] -= 48;
				param_temp[2] -= 48;
				winfacVal = (param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2];
			}
		}
	}

	*winfac = winfacVal;
}
#endif

/* Read ADC Value */
#if 0
static uint16_t read_adc_value(struct LTR559_data *LTR559)
{

	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD  	5000
#define AGC_HYS					15

	switch (LTR559->mode) {
	case 0 :
		/* ALS */
		buffer[0] = LTR559_ALS_DATA_CH1_0;

		/* read data bytes from data regs */
		ret = I2C_Read(buffer, 4);
		break;

	case 1 :
	case 3 : /* PS with saturation bit */
		/* PS */
		buffer[0] = LTR559_PS_DATA_0;

		/* read data bytes from data regs */
		ret = I2C_Read(buffer, 2);
		break;
	}

	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	switch (LTR559->mode) {
	case 0 :
		/* ALS Ch0 */
		ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
		dev_dbg(&LTR559->i2c_client->dev,
				"%s | als_ch0 value = 0x%04X\n", __func__,
				ch0_val);

		if (ch0_val > ALS_MAX_MEASURE_VAL) {
			dev_err(&LTR559->i2c_client->dev,
					"%s: ALS Value Error: 0x%X\n", __func__,
					ch0_val);
		}
		ch0_val &= ALS_VALID_MEASURE_MASK;
		input_report_abs(LTR559->als_input_dev, ABS_MISC, ch0_val);
		input_sync(LTR559->als_input_dev);

		/* ALS Ch1 */
		ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&LTR559->i2c_client->dev,
				"%s | als_ch1 value = 0x%04X\n", __func__,
				ch1_val);

		if (ch1_val > ALS_MAX_MEASURE_VAL) {
			dev_err(&LTR559->i2c_client->dev,
					"%s: ALS Value Error: 0x%X\n", __func__,
					ch1_val);
		}
		ch1_val &= ALS_VALID_MEASURE_MASK;
		input_report_abs(LTR559->als_input_dev, ABS_MISC, ch1_val);
		input_sync(LTR559->als_input_dev);

		buffer[0] = LTR559_ALS_PS_STATUS;
		ret = I2C_Read(buffer, 1);
		if (ret < 0) {
			dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
			return ret;
		}

		value_temp = buffer[0];
		temp = buffer[0];
		gain = (value_temp & 0x70);
		gain >>= 4;

		if (gain == 0) {			//gain 1
			gain = 1;
		} else if (gain == 1) {		//gain 2
			gain = 2;
		} else if (gain == 2) {		//gain 4
			gain = 4;
		} else if (gain == 3) {		//gain 8
			gain = 8;
		} else if (gain == 6) {		//gain 48
			gain = 48;
		} else if (gain == 7) {		//gain 96
			gain = 96;
		}

		buffer[0] = LTR559_ALS_CONTR;
		ret = I2C_Read(buffer, 1);
		if (ret < 0) {
			dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
			return ret;
		}
		value_temp = buffer[0];
		value_temp &= 0xE3;

		if ((ch0_val == 0) && (ch1_val > 50 ))
		{
			value = lux_val_prev;
		}
		else
		{
			if(AGC==1)  //Default is AGC =1 (enable)
			{
				if (gain == 1) {
					if ((ch0_val + ch1_val) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
						value = ratioHysterisis(ch0_val, ch1_val);
						value_temp |= ALS_GAIN_8x;
						gain_chg_req = 1;
					} else {
						value = ratioHysterisis(ch0_val, ch1_val);
					}
				} else if (gain == 8) {
					if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
						value = ratioHysterisis(ch0_val, ch1_val);
						value_temp |= ALS_GAIN_1x;
						gain_chg_req = 1;
					} else {
						value = ratioHysterisis(ch0_val, ch1_val);
					}
				} else {
					value = ratioHysterisis(ch0_val, ch1_val);
				}

				if (gain_chg_req) {
					buffer[0] = LTR559_ALS_CONTR;
					buffer[1] = value_temp;
					ret = I2C_Write(buffer, 2);
					if (ret < 0) {
						dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
						return ret;
					}
				}
			}
			else
			{
				value = ratioHysterisis(ch0_val, ch1_val);
			}

			/* ALS Lux Conversion */
			//value = lux_formula(ch0_val, ch1_val);
		}

		if ((value > 50000) || (((ch0_val + ch1_val) > 50000) && (temp & 0x80))) {
			value = 50000;
		}
		lux_val_prev = value;

		break;

	case 1 :
	case 3 : /* PS with saturation bit */
		/* PS */
		ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&LTR559->i2c_client->dev,
				"%s | ps value = 0x%04X\n", __func__,
				ps_val);

		if (LTR559->mode == 1) {
			if (ps_val > PS_MAX_MEASURE_VAL) {
				dev_err(&LTR559->i2c_client->dev,
						"%s: PS Value Error: 0x%X\n", __func__,
						ps_val);
			}
			ps_val &= PS_VALID_MEASURE_MASK;
		} else if (LTR559->mode == 3) {
			ps_val &= 0x87FF;
		}

		value = ps_val;

		break;

	}

	return value;
}
#endif

static uint16_t read_als_adc_value(struct LTR559_data *LTR559)
{

	int8_t ret = -99;
	uint16_t value = -99;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD	5000
#define AGC_HYS					15
#define MAX_VAL					32767 // (1 << 15) - 1

	/* ALS */
	buffer[0] = LTR559_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);

	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	/* ALS Ch0 */
	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
	#if 0
	dev_dbg(&LTR559->i2c_client->dev,
			"%s | als_ch0 value = 0x%04X\n", __func__,
			ch0_val);
	#endif
	if (ch0_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: ALS Value Error: 0x%X\n", __func__,
				ch0_val);
	}
	ch0_val &= ALS_VALID_MEASURE_MASK;
	//input_report_abs(LTR559->als_input_dev, ABS_MISC, ch0_val);
	//input_sync(LTR559->als_input_dev);

	/* ALS Ch1 */
	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	#if 0
	dev_dbg(&LTR559->i2c_client->dev,
			"%s | als_ch1 value = 0x%04X\n", __func__,
			ch1_val);
	#endif
	if (ch1_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: ALS Value Error: 0x%X\n", __func__,
				ch1_val);
	}
	ch1_val &= ALS_VALID_MEASURE_MASK;
	//input_report_abs(LTR559->als_input_dev, ABS_MISC, ch1_val);
	//input_sync(LTR559->als_input_dev);

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value_temp = buffer[0];
	temp = buffer[0];
	gain = (value_temp & 0x70);
	gain >>= 4;

	if (gain == 0) {		//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	value_temp = buffer[0];
	value_temp &= 0xE3;

	if ((ch0_val == 0) && (ch1_val > 50 )) {
		value = lux_val_prev;
	} else {
		if (gain == 1) {
			if ((ch0_val + ch1_val) <
					((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_8x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else if (gain == 8) {
			if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_1x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else {
			value = ratioHysterisis(ch0_val, ch1_val);
		}
		if (gain_chg_req) {
			buffer[0] = LTR559_ALS_CONTR;
			buffer[1] = value_temp;
			ret = I2C_Write(buffer, 2);
			if (ret < 0) {
				dev_err(&LTR559->i2c_client->dev,
						"%s | 0x%02X", __func__, buffer[0]);

				return ret;
			}
		}

	}

	if ((value > MAX_VAL) || (((ch0_val + ch1_val) > MAX_VAL) && (temp & 0x80))) {
		value = MAX_VAL;
	}

	if (value > 80) {
		if ((lux_val_prev > 15)
			&& (value < (lux_val_prev+15))
			&& (value > (lux_val_prev-15))) {
			value = lux_val_prev;
		}
	}
	lux_val_prev = value;

	return value;
}

static uint16_t read_ps_adc_value(struct LTR559_data *LTR559)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint8_t buffer[4];

	buffer[0] = LTR559_PS_DATA_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 2);

	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	dev_dbg(&LTR559->i2c_client->dev,
			"%s | ps value = 0x%04X\n", __func__,
			ps_val);

	if (ps_val > PS_MAX_MEASURE_VAL) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS Value Error: 0x%X\n", __func__,
				ps_val);
	}
	ps_val &= PS_VALID_MEASURE_MASK;

	value = ps_val;

	return value;
}

static int8_t als_mode_setup (uint8_t alsMode_set_reset, struct LTR559_data *LTR559)
{
	int8_t ret = 0;

	ret = _LTR559_set_bit(LTR559->i2c_client, alsMode_set_reset,
			LTR559_ALS_CONTR, ALS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s ALS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_sw_reset_setup(uint8_t alsSWReset_set_reset,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;

	ret = _LTR559_set_bit(LTR559->i2c_client, alsSWReset_set_reset,
			LTR559_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s ALS sw reset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_gain_setup (uint8_t alsgain_range, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE3;

	if (alsgain_range == 1) {
		value |= ALS_GAIN_1x;
	} else if (alsgain_range == 2) {
		value |= ALS_GAIN_2x;
	} else if (alsgain_range == 4) {
		value |= ALS_GAIN_4x;
	} else if (alsgain_range == 8) {
		value |= ALS_GAIN_8x;
	} else if (alsgain_range == 48) {
		value |= ALS_GAIN_48x;
	} else if (alsgain_range == 96) {
		value |= ALS_GAIN_96x;
	}

	buffer[0] = LTR559_ALS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s ALS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_contr_setup(uint8_t als_contr_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_CONTR;

	/* Default settings used for now. */
	buffer[1] = als_contr_val;
	buffer[1] &= 0x1F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | ALS_CONTR (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}

static int8_t als_contr_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MODE_RDBCK) {
		*retVal = value & 0x01;
	} else if (rdbck_type == ALS_SWRT_RDBCK) {
		*retVal = (value & 0x02) >> 1;
	} else if (rdbck_type == ALS_GAIN_RDBCK) {
		*retVal = (value & 0x1C) >> 2;
	} else if (rdbck_type == ALS_CONTR_RDBCK) {
		*retVal = value & 0x1F;
	}

	return ret;
}

static int8_t ps_mode_setup (uint8_t psMode_set_reset, struct LTR559_data *LTR559)
{
	int8_t ret = 0;

	ret = _LTR559_set_bit(LTR559->i2c_client, psMode_set_reset,
			LTR559_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s PS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_gain_setup (uint8_t psgain_range, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF3;

	if (psgain_range == 16) {
		value |= PS_GAIN_16x;
	} else if (psgain_range == 32) {
		value |= PS_GAIN_32x;
	} else if (psgain_range == 64) {
		value |= PS_GAIN_64x;
	}

	buffer[0] = LTR559_PS_CONTR;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s PS gain setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_satu_indica_setup(uint8_t pssatuindica_enable,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;

	ret = _LTR559_set_bit(LTR559->i2c_client, pssatuindica_enable,
			LTR559_PS_CONTR, PS_SATUR_INDIC_EN);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s PS saturation indicator setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_contr_setup(uint8_t ps_contr_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s | PS_CONTR (0x%02X) setup fail...",
				__func__, buffer[0]);
	}

	return ret;
}

static int8_t ps_contr_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PS_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == PS_GAIN_RDBCK) {
		*retVal = (value & 0x0C) >> 2;
	} else if (rdbck_type == PS_SATUR_RDBCK) {
		*retVal = (value & 0x20) >> 5;
	} else if (rdbck_type == PS_CONTR_RDBCK) {
		*retVal = value & 0x2F;
	}

	return ret;
}

static int8_t ps_ledCurrent_setup (uint8_t psledcurr_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (psledcurr_val == 5) {
		value |= LED_CURR_5MA;
	} else if (psledcurr_val == 10) {
		value |= LED_CURR_10MA;
	} else if (psledcurr_val == 20) {
		value |= LED_CURR_20MA;
	} else if (psledcurr_val == 50) {
		value |= LED_CURR_50MA;
	} else if (psledcurr_val == 100) {
		value |= LED_CURR_100MA;
	}

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s PS LED current setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_ledCurrDuty_setup (uint8_t psleddutycycle_val,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xE7;

	if (psleddutycycle_val == 25) {
		value |= LED_CURR_DUTY_25PC;
	} else if (psleddutycycle_val == 50) {
		value |= LED_CURR_DUTY_50PC;
	} else if (psleddutycycle_val == 75) {
		value |= LED_CURR_DUTY_75PC;
	} else if (psleddutycycle_val == 100) {
		value |= LED_CURR_DUTY_100PC;
	}

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s PS LED current duty setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_ledPulseFreq_setup (uint8_t pspulreq_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0x1F;

	if (pspulreq_val == 30) {
		value |= LED_PUL_FREQ_30KHZ;
	} else if (pspulreq_val == 40) {
		value |= LED_PUL_FREQ_40KHZ;
	} else if (pspulreq_val == 50) {
		value |= LED_PUL_FREQ_50KHZ;
	} else if (pspulreq_val == 60) {
		value |= LED_PUL_FREQ_60KHZ;
	} else if (pspulreq_val == 70) {
		value |= LED_PUL_FREQ_70KHZ;
	} else if (pspulreq_val == 80) {
		value |= LED_PUL_FREQ_80KHZ;
	} else if (pspulreq_val == 90) {
		value |= LED_PUL_FREQ_90KHZ;
	} else if (pspulreq_val == 100) {
		value |= LED_PUL_FREQ_100KHZ;
	}

	buffer[0] = LTR559_PS_LED;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s PS LED pulse frequency setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | PS_LED (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t ps_led_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_LED;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == LED_CURR_RDBCK) {
		*retVal = (value & 0x07);
	} else if (rdbck_type == LED_CURR_DUTY_RDBCK) {
		*retVal = (value & 0x18) >> 3;
	} else if (rdbck_type == LED_PUL_FREQ_RDBCK) {
		*retVal = (value & 0xE0) >> 5;
	} else if (rdbck_type == PS_LED_RDBCK) {
		*retVal = value;
	}

	return ret;
}

static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val > 15) {
		pspulsecount_val = 15;
	}
	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | PS_LED_COUNT (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t ps_ledPulseCount_readback (uint8_t *retVal, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR559_PS_N_PULSES;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}

static int8_t ps_meas_rate_setup(uint16_t meas_rate_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50) {
		value |= PS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 70) {
		value |= PS_MEAS_RPT_RATE_70MS;
	} else if (meas_rate_val == 100) {
		value |= PS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= PS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= PS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= PS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= PS_MEAS_RPT_RATE_2000MS;
	} else if (meas_rate_val == 10) {
		value |= PS_MEAS_RPT_RATE_10MS;
	}

	buffer[0] = LTR559_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s PS measurement rate setup fail...\n", __func__);

		return ret;
	}

	return ret;
}

static int8_t ps_meas_rate_readback (uint8_t *retVal, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3], value;

	buffer[0] = LTR559_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = (value & 0x0F);

	return ret;
}

static int8_t als_meas_rate_setup(uint16_t meas_rate_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xF8;

	if (meas_rate_val == 50) {
		value |= ALS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 100) {
		value |= ALS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= ALS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= ALS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= ALS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= ALS_MEAS_RPT_RATE_2000MS;
	}

	buffer[0] = LTR559_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s ALS measurement rate setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_integ_time_setup(uint16_t integ_time_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xC7;

	if (integ_time_val == 100) {
		value |= ALS_INTEG_TM_100MS;
	} else if (integ_time_val == 50) {
		value |= ALS_INTEG_TM_50MS;
	} else if (integ_time_val == 200) {
		value |= ALS_INTEG_TM_200MS;
	} else if (integ_time_val == 400) {
		value |= ALS_INTEG_TM_400MS;
	} else if (integ_time_val == 150) {
		value |= ALS_INTEG_TM_150MS;
	} else if (integ_time_val == 250) {
		value |= ALS_INTEG_TM_250MS;
	} else if (integ_time_val == 300) {
		value |= ALS_INTEG_TM_300MS;
	} else if (integ_time_val == 350) {
		value |= ALS_INTEG_TM_350MS;
	}

	buffer[0] = LTR559_ALS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s ALS integration time setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_meas_rate_reg_setup(uint8_t als_meas_rate_reg_val,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_MEAS_RATE;

	buffer[1] = als_meas_rate_reg_val;
	buffer[1] &= 0x3F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | ALS_MEAS_RATE (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t als_meas_rate_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == ALS_MEAS_RPT_RATE_RDBCK) {
		*retVal = (value & 0x07);
	} else if (rdbck_type == ALS_INTEG_TM_RDBCK) {
		*retVal = (value & 0x38) >> 3;
	} else if (rdbck_type == ALS_MEAS_RATE_RDBCK) {
		*retVal = (value & 0x3F);
	}

	return ret;
}

static int8_t part_ID_reg_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR559_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == PART_NUM_ID_RDBCK) {
		*retVal = (value & 0xF0) >> 4;
	} else if (rdbck_type == REVISION_ID_RDBCK) {
		*retVal = value & 0x0F;
	} else if (rdbck_type == PART_ID_REG_RDBCK) {
		*retVal = value;
	}

	return ret;
}

static int8_t manu_ID_reg_readback (uint8_t *retVal, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[1], value;

	buffer[0] = LTR559_MANUFACTURER_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	*retVal = value;

	return ret;
}

static int8_t als_ps_status_reg (uint8_t data_status_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (data_status_type == PS_DATA_STATUS_RDBCK) {
		*retVal = (value & 0x01);
	} else if (data_status_type == PS_INTERR_STATUS_RDBCK) {
		*retVal = (value & 0x02) >> 1;
	} else if (data_status_type == ALS_DATA_STATUS_RDBCK) {
		*retVal = (value & 0x04) >> 2;
	} else if (data_status_type == ALS_INTERR_STATUS_RDBCK) {
		*retVal = (value & 0x08) >> 3;
	} else if (data_status_type == ALS_GAIN_STATUS_RDBCK) {
		*retVal = (value & 0x70) >> 4;
	} else if (data_status_type == ALS_VALID_STATUS_RDBCK) {
		*retVal = (value & 0x80) >> 7;
	} else if (data_status_type == ALS_PS_STATUS_RDBCK) {
		*retVal = value;
	}

	return ret;
}

static int8_t als_ch0ch1raw_calc_readback (uint16_t *retVal1, uint16_t *retVal2,
		uint16_t *retVal3, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[11];
	uint16_t value1, value2, value3;

	buffer[0] = LTR559_ALS_DATA_CH1_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value1 = ((int)buffer[2]) + ((int)buffer[3] << 8); // CH0
	value2 = ((int)buffer[0]) + ((int)buffer[1] << 8); // CH1

	value3 = ratioHysterisis(value1, value2);

	*retVal1 = value1;
	*retVal2 = value2;
	*retVal3 = value3;

	return ret;
}

static int8_t interrupt_mode_setup (uint8_t interr_mode_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFC;

	if (interr_mode_val == 0) {
		value |= INT_MODE_00;
	} else if (interr_mode_val == 1) {
		value |= INT_MODE_PS_TRIG;
	} else if (interr_mode_val == 2) {
		value |= INT_MODE_ALS_TRIG;
	} else if (interr_mode_val == 3) {
		value |= INT_MODE_ALSPS_TRIG;
	}

	buffer[0] = LTR559_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s Interrupt mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t interrupt_polarity_setup (uint8_t interr_polar_val,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value &= 0xFB;

	if (interr_polar_val == 0) {
		value |= INT_POLAR_ACT_LO;
	} else if (interr_polar_val == 1) {
		value |= INT_POLAR_ACT_HI;
	}

	buffer[0] = LTR559_INTERRUPT;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s Interrupt polarity setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t interrupt_setup(uint8_t interrupt_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_INTERRUPT;

	/* Default settings used for now. */
	buffer[1] = interrupt_val;
	buffer[1] &= 0x07;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s |Interrupt (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t interrupt_readback (uint8_t rdbck_type, uint8_t *retVal,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_INTERRUPT;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (rdbck_type == INT_MODE_RDBCK) {
		*retVal = (value & 0x03);
	} else if (rdbck_type == INT_POLAR_RDBCK) {
		*retVal = (value & 0x04) >> 2;
	} else if (rdbck_type == INT_INTERRUPT_RDBCK) {
		*retVal = (value & 0x07);
	}

	return ret;
}

static int8_t ps_offset_setup (uint16_t ps_offset_val, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_OFFSET_1;
	buffer[1] = (ps_offset_val >> 8) & 0x03;
	buffer[2] = (ps_offset_val & 0xFF);

	ret = I2C_Write(buffer, 3);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s PS offset setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_offset_readback (uint16_t *offsetval, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2];
	uint16_t value;

	buffer[0] = LTR559_PS_OFFSET_1;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];
	value <<= 8;
	value += buffer[1];

	*offsetval = value;

	return ret;
}

static int8_t interrupt_persist_setup (uint8_t interr_persist_val,
		struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	value = interr_persist_val;

	buffer[0] = LTR559_INTERRUPT_PRST;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s Interrupt persist setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t interrupt_prst_readback (uint8_t *retVal, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_INTERRUPT_PRST;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	*retVal = value;

	return ret;
}

/* Set ALS range */
static int8_t set_als_range(uint16_t lt, uint16_t ht, uint8_t lo_hi)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0xFF;
		num_data = 5;
	}

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set als range:0x%04x"
			" - 0x%04x\n", __func__, lt, ht);

	return ret;
}

static int8_t als_range_readback (uint16_t *lt, uint16_t *ht, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo, value_hi;

	buffer[0] = LTR559_ALS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}

/* Set PS range */
static int8_t set_ps_range(uint16_t lt, uint16_t ht, uint8_t lo_hi,
		struct LTR559_data *LTR559)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
	}

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	dev_dbg(&LTR559->i2c_client->dev, "%s Set ps range:0x%04x"
			" - 0x%04x\n", __func__, lt, ht);

	return ret;
}

static int8_t ps_range_readback (uint16_t *lt, uint16_t *ht, struct LTR559_data *LTR559)
{
	int8_t ret = 0;
	uint8_t buffer[5];
	uint16_t value_lo, value_hi;

	buffer[0] = LTR559_PS_THRES_UP_0;
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value_lo = buffer[3];
	value_lo <<= 8;
	value_lo += buffer[2];
	*lt = value_lo;

	value_hi = buffer[1];
	value_hi <<= 8;
	value_hi += buffer[0];
	*ht = value_hi;

	return ret;
}

static uint16_t discardMinMax_findCTMov_Avg (uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA1		PS_MAX_MOV_AVG_KEPT_DATA_CTR
#define STARTING_PS_INDEX1		0
#define ENDING_PS_INDEX1		5
#define NUM_AVG_DATA1			5

	uint8_t i_ctr, i_ctr2, maxIndex, minIndex;
	uint16_t maxVal, minVal, _ps_val[MAX_NUM_PS_DATA1];
	uint16_t temp = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		_ps_val[i_ctr] = ps_val[i_ctr];
	}

	maxVal = ps_val[STARTING_PS_INDEX1];
	maxIndex = STARTING_PS_INDEX1;
	minVal = ps_val[STARTING_PS_INDEX1];
	minIndex = STARTING_PS_INDEX1;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] > maxVal) {
			maxVal = ps_val[i_ctr];
			maxIndex = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] < minVal) {
			minVal = ps_val[i_ctr];
			minIndex = i_ctr;
		}
	}

	i_ctr2 = 0;

	if (minIndex != maxIndex) {
		for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
			if ((i_ctr != minIndex) && (i_ctr != maxIndex)) {
				ps_val[i_ctr2] = _ps_val[i_ctr];
				i_ctr2++;
			}
		}
	}
	ps_val[MAX_NUM_PS_DATA1 - 1] = 0;
	ps_val[MAX_NUM_PS_DATA1 - 2] = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < ENDING_PS_INDEX1; i_ctr++) {
		temp += ps_val[i_ctr];
	}

	temp = (temp / NUM_AVG_DATA1);

	return temp;
}

static uint16_t findCT_Avg (uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA2		PS_MAX_INIT_KEPT_DATA_COUNTER
#define STARTING_PS_INDEX2		3
#define NUM_AVG_DATA2			3

	uint8_t i_ctr, min_Index, max_Index;
	uint16_t max_val, min_val;
	uint16_t temp = 0;
	//struct LTR559_data *LTR559 = sensor_info;

	max_val = ps_val[STARTING_PS_INDEX2];
	max_Index = STARTING_PS_INDEX2;
	min_val = ps_val[STARTING_PS_INDEX2];
	min_Index = STARTING_PS_INDEX2;

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] > max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val) {
		// all values are the same
		temp = ps_val[STARTING_PS_INDEX2];
	} else {
		for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index)) {
				temp += ps_val[i_ctr];
			}
		}
		temp = (temp / NUM_AVG_DATA2);
	}

	//temp = (temp / NUM_AVG_DATA2);

	return temp;
}

// take note ------------------------------------------
// This function should be called in the function which is called when the CALL button is pressed.
// take note ------------------------------------------
static void setThrDuringCall (void)
{
	int8_t ret;
	struct LTR559_data *LTR559 = sensor_info;

	// set ps measurement rate to 10ms
	ret = ps_meas_rate_setup(10, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;
	ps_movavg_data_counter = 0;

	ret = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
	}
}

/* Report PS input event */
static void report_ps_input_event(struct LTR559_data *LTR559)
{
	int8_t ret;
	uint16_t adc_value;

	adc_value = read_ps_adc_value (LTR559);

	pr_err("[LTR559] adc value %d, ps_grabData_stage %d\n", adc_value,
			ps_grabData_stage);
	if(PS_Dyn_Calib==1) //Default is PS_Dyn_Calib=1 (enable)
	{
		if (ps_grabData_stage == 0) {
			if (ps_kept_data_counter < PS_MAX_INIT_KEPT_DATA_COUNTER) {
				if (adc_value != 0) {
					ps_init_kept_data[ps_kept_data_counter]
						= adc_value;
					ps_kept_data_counter++;
				}
			}

			if (ps_kept_data_counter >= PS_MAX_INIT_KEPT_DATA_COUNTER) {
				ps_ct_avg = findCT_Avg(ps_init_kept_data);
				ftn_init = ps_ct_avg * 17;
				ps_grabData_stage = 1;
			}
		}

		if (ps_grabData_stage == 1) {
			if ((ftn_init - (ps_ct_avg * 10)) < 1400) {
				ftn_final = (ps_ct_avg * 10) + 1400;
			} else {
				if ((ftn_init - (ps_ct_avg * 10)) > 1800) {
					ftn_final = (ps_ct_avg * 10) + 1800;
				} else {
					ftn_final = ftn_init;
				}
			}
			ntf_final = (ftn_final - (ps_ct_avg * 10));
			ntf_final *= 4;
			ntf_final /= 100;
			ntf_final += ps_ct_avg;
			ftn_final /= 10;
			if (ntf_final >= PS_MAX_MEASURE_VAL) {
				ntf_final = PS_MAX_MEASURE_VAL;
			}
			if (ftn_final >= PS_MAX_MEASURE_VAL) {
				ftn_final = PS_MAX_MEASURE_VAL;
			}

			ret = ps_meas_rate_setup(50, LTR559);
			if (ret < 0) {
				dev_err(&LTR559->i2c_client->dev,
						"%s: PS MeasRate Setup Fail...\n",
						__func__);
			}

			ps_grabData_stage = 2;
		}
	}
	else
	{
		ps_grabData_stage = 2;
		ftn_final= 400;
		ntf_final= 200;
	}

	if (ps_grabData_stage == 2) {
		input_report_abs(LTR559->ps_input_dev, ABS_DISTANCE, adc_value);
		input_sync(LTR559->ps_input_dev);

		/* report NEAR or FAR to the user layer */
		if ((adc_value > ftn_final) || (adc_value < ntf_final)) {
			// FTN
			if (adc_value > ftn_final) {
				input_report_abs(LTR559->ps_input_dev, ABS_DISTANCE, NEAR_VAL);
				input_sync(LTR559->ps_input_dev);
			}
			// FTN

			// NTF
			if (adc_value < ntf_final) {
				input_report_abs(LTR559->ps_input_dev, ABS_DISTANCE, FAR_VAL);
				input_sync(LTR559->ps_input_dev);
			}
			// NTF
		}
		/* report NEAR or FAR to the user layer */

		if (ps_movavg_data_counter < PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			if (adc_value != 0) {
				ps_movavg_data[ps_movavg_data_counter] = adc_value;
				ps_movavg_data_counter++;
			}
		}

		if (ps_movavg_data_counter >= PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			ps_movct_avg = discardMinMax_findCTMov_Avg(ps_movavg_data);

			if (ps_movct_avg < ps_ct_avg) {
				ps_ct_avg = ps_movct_avg;
				ftn_init = ps_ct_avg * 17;
				ps_grabData_stage = 1;
			}
			ps_movavg_data_counter = 5;
		}

	}

}

/* Report ALS input event */
static void LTR559_als_schedwork(struct work_struct *work)
{
	struct LTR559_data *LTR559 = sensor_info;
	uint16_t adc_value;

	adc_value = read_als_adc_value (LTR559);

	input_report_abs(LTR559->als_input_dev, ABS_MISC, adc_value);
	input_sync(LTR559->als_input_dev);

	pr_err("[LTR559] report lux %d\n", adc_value);

	if(LTR559->als_enable_flag)
		queue_delayed_work(LTR559->workqueue, &LTR559->als_dwork, HZ / 2);
}

static void mz_proximity_iio_push_data(struct mz_proximity_data *mz_proximity,
					struct iio_dev *indio_dev,
					int ps_state);
/* Work when interrupt */
static void LTR559_ps_schedwork(struct work_struct *work)
{
#if 0
	int8_t ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct LTR559_data *LTR559 = sensor_info;
	uint8_t buffer[2];

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0A;
	newdata = status & 0x05;

	pr_err("[LTR559] ps eint %02x, %02x\n", interrupt_stat, newdata);
	// PS interrupt and PS with new data
	if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
		LTR559->ps_irq_flag = 1;
		report_ps_input_event(LTR559);
		LTR559->ps_irq_flag = 0;
	}
	/*
	// ALS interrupt and ALS with new data
	if ((interrupt_stat & 0x08) && (newdata & 0x04)) {
		LTR559->als_irq_flag = 1;
		report_als_input_event(LTR559);
		LTR559->als_irq_flag = 0;
	}
	*/
#else
	struct LTR559_data *LTR559 = sensor_info;
	struct mz_proximity_data *mz_proximity;
	mz_proximity = &LTR559->mz_proximity;

	if (mz_proximity->irq_gpio == 0)
		mz_proximity->irq_gpio = 1;

	int gpio = mt_get_gpio_in(GPIO_ALS_EINT_PIN);

#if 0
	pr_info("[LTR559] dump cur ps_data %d\n", gpio);
	input_report_abs(LTR559->ps_input_dev, ABS_DISTANCE, gpio);
	input_sync(LTR559->ps_input_dev);
#endif

	/* report 0 or 4 cm */
	if (mz_proximity->trig_state) {
		dev_info(mz_proximity->dev, "push non wk ps state: %d\n", gpio << 2);
		mz_proximity_iio_push_data(mz_proximity, mz_proximity->indio_dev, gpio << 2);
	}

	if (mz_proximity->trig_wk_state) {
		dev_info(mz_proximity->dev, "push wk ps state: %d\n", gpio << 2);
		mz_proximity_iio_push_data(mz_proximity, mz_proximity->indio_dev_wk, gpio << 2);
	}

	mt_eint_set_polarity(CUST_EINT_ALS_NUM, !gpio);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
}

/* IRQ Handler */
static void LTR559_irq_handler(void)
{
	mt_eint_mask(CUST_EINT_ALS_NUM);
	schedule_work(&sensor_info->ps_work);
}

static int LTR559_gpio_irq(struct LTR559_data *LTR559)
{
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_eint_set_sens(CUST_EINT_ALS_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, 0);
	mt_eint_registration(CUST_EINT_ALS_NUM,
			EINTF_TRIGGER_LOW, LTR559_irq_handler, 0);
	mt_eint_unmask(CUST_EINT_ALS_NUM);

	return 0;
}

/* PS Enable */
static int8_t ps_enable_init(struct LTR559_data *LTR559)
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read

	//setThrDuringCall();

	if (LTR559->ps_enable_flag) {
		dev_info(&LTR559->i2c_client->dev, "%s: already enabled\n", __func__);
		return 0;
	}

	/* Set thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	//rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
	//rc = set_ps_range(PS_MIN_MEASURE_VAL, 400, LO_N_HI_LIMIT);
	rc = set_ps_range(PS_MIN_MEASURE_VAL, 200, LO_N_HI_LIMIT, LTR559);
#else
	rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, LO_N_HI_LIMIT, LTR559);
#endif
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
		return rc;
	}

#if 0
	/* Allows this interrupt to wake the system */
	//rc = irq_set_irq_wake(LTR559->irq, 1);
	rc = set_irq_wake(LTR559->irq, 1);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: IRQ-%d WakeUp Enable Fail...\n", __func__, LTR559->irq);
		return rc;
	}
#endif

	rc = ps_led_setup(0x7F, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_ledPulseCount_setup(0x04, LTR559); // pulses = 4
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
	}

	rc = ps_meas_rate_setup(70, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	/*
	rc = ps_contr_setup(0x03, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	// dummy read
	buffer[0] = LTR559_PS_CONTR;
	I2C_Read(buffer, 1);
	// dumy read

	LTR559->ps_enable_flag = 1;
	*/

	return rc;
}

/* PS Disable */
static int8_t ps_set_enable(struct LTR559_data *LTR559, int en)
{
	int8_t rc = 0;

	if (LTR559->ps_enable_flag == !!en) {
		dev_info(&LTR559->i2c_client->dev, "%s: dupulicate action, abort\n", __func__);
		return 0;
	}

#if 0
	/* Don't allow this interrupt to wake the system anymore */
	//rc = irq_set_irq_wake(LTR559->irq, 0);
	rc = set_irq_wake(LTR559->irq, 0);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: IRQ-%d WakeUp Disable Fail...\n", __func__, LTR559->irq);
		return rc;
	}
#endif

	//rc = _LTR559_set_bit(LTR559->i2c_client, CLR_BIT, LTR559_PS_CONTR, PS_MODE);
	rc = ps_mode_setup(!!en, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	LTR559->ps_enable_flag = !!en;

	return rc;
}

#if 0
/* PS open fops */
ssize_t ps_open(struct inode *inode, struct file *file)
{
	struct LTR559_data *LTR559 = sensor_info;

	if (LTR559->ps_opened)
		return -EBUSY;

	LTR559->ps_opened = 1;

	return 0;
}

/* PS release fops */
ssize_t ps_release(struct inode *inode, struct file *file)
{
	struct LTR559_data *LTR559 = sensor_info;

	LTR559->ps_opened = 0;

	return ps_disable(LTR559);
}

/* PS IOCTL */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int ps_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct LTR559_data *LTR559 = sensor_info;

	pr_info("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LTR559_IOCTL_PS_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		rc = val ? ps_enable_init(LTR559) : ps_disable(LTR559);

		break;
	case LTR559_IOCTL_PS_GET_ENABLED:
		rc = put_user(LTR559->ps_enable_flag, (unsigned long __user *)arg);

		break;
	default:
		pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = ps_ioctl
#else
		.unlocked_ioctl = ps_ioctl
#endif
};

struct miscdevice ps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "LTR559_ps",
	.fops = &ps_fops
};
#endif

static int8_t als_enable_init(struct LTR559_data *LTR559)
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read

	/* if device not enabled, enable it */
	if (LTR559->als_enable_flag) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS already enabled...\n", __func__);
		return rc;
	}

	rc = als_meas_rate_reg_setup(0x03, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
		return rc;
	}

	/* Set minimummax thresholds where interrupt will *not* be generated */
#if ACT_INTERRUPT
	//rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
	rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, LO_N_HI_LIMIT);
#else
	rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, LO_N_HI_LIMIT);
#endif
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
		return rc;
	}

	/*
	rc = als_contr_setup(0x0D, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS Enable Fail...\n", __func__);
		return rc;
	}

	// dummy read
	buffer[0] = LTR559_ALS_CONTR;
	I2C_Read(buffer, 1);
	// dumy read

	LTR559->als_enable_flag = 1;
	*/

	return rc;
}

static int8_t als_set_enable(struct LTR559_data *LTR559, int en)
{
	int8_t rc = 0;

	if (LTR559->als_enable_flag == !!en) {
		dev_err(&LTR559->i2c_client->dev, "%s : dupulicate action\n", __func__);
		return rc;
	}

	//rc = _LTR559_set_bit(LTR559->i2c_client, CLR_BIT, LTR559_ALS_CONTR, ALS_MODE);
	rc = als_mode_setup(!!en, LTR559);
	if (rc < 0) {
		dev_err(&LTR559->i2c_client->dev,"%s: ALS Disable Fail...\n", __func__);
		return rc;
	}
	LTR559->als_enable_flag = !!en;

	als_meas_rate_setup(50, LTR559);
	als_integ_time_setup(50, LTR559);

	return rc;
}

#if 0
ssize_t als_open(struct inode *inode, struct file *file)
{
	struct LTR559_data *LTR559 = sensor_info;
	int8_t rc = 0;

	if (LTR559->als_opened) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS already Opened...\n", __func__);
		rc = -EBUSY;
	}
	LTR559->als_opened = 1;

	return rc;
}

ssize_t als_release(struct inode *inode, struct file *file)
{
	struct LTR559_data *LTR559 = sensor_info;

	LTR559->als_opened = 0;

	//return 0;
	return als_disable(LTR559);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int als_ioctl (struct inode *ino, struct file *file, unsigned int cmd, unsigned long arg)
#else
static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int rc = 0, val = 0;
	struct LTR559_data *LTR559 = sensor_info;

	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LTR559_IOCTL_ALS_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		/*pr_info("%s value = %d\n", __func__, val);*/
		rc = val ? als_enable_init(LTR559) : als_disable(LTR559);

		break;
	case LTR559_IOCTL_ALS_GET_ENABLED:
		val = LTR559->als_enable_flag;
		/*pr_info("%s enabled %d\n", __func__, val);*/
		rc = put_user(val, (unsigned long __user *)arg);

		break;
	default:
		pr_err("%s: INVALID COMMAND %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.release = als_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.ioctl = als_ioctl
#else
		.unlocked_ioctl = als_ioctl
#endif
};

static struct miscdevice als_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "LTR559_ls",
	.fops = &als_fops
};
#endif

static ssize_t als_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct LTR559_data *LTR559 = sensor_info;

	//LTR559->mode = ALS;
	//value = read_adc_value(LTR559);
	value = read_als_adc_value(LTR559);
	//input_report_abs(LTR559->als_input_dev, ABS_MISC, value);
	//input_sync(LTR559->als_input_dev);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static ssize_t ps_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct LTR559_data *LTR559 = sensor_info;

	//LTR559->mode = PS;
	//value = read_adc_value(LTR559);
	value = read_ps_adc_value(LTR559);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static ssize_t als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR559_data *LTR559 = sensor_info;

	ret = als_contr_readback(ALS_MODE_RDBCK, &rdback_val, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", rdback_val);

	return ret;
}

static ssize_t als_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct LTR559_data *LTR559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR559->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = als_set_enable(LTR559, param);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS mode setup Fail...\n", __func__);
		return (-1);
	}

	if(LTR559->als_enable_flag)
		queue_delayed_work(LTR559->workqueue, &LTR559->als_dwork, 0);

	return count;
}

static ssize_t ps_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct LTR559_data *LTR559 = sensor_info;

	ret = ps_contr_readback(PS_MODE_RDBCK, &rdback_val, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS_MODE_RDBCK Fail...\n", __func__);
		return (-1);
	}

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", rdback_val);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct LTR559_data *LTR559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&LTR559->i2c_client->dev, "%s: store value = %d\n", __func__, param);

	ret = ps_set_enable(LTR559, param);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS mode setup Fail...\n", __func__);
		return (-1);
	}

	return count;

}

static ssize_t ps_interrupt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int ps_gpio = mt_get_gpio_in(GPIO_ALS_EINT_PIN);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", ps_gpio);

	return ret;
}

static ssize_t ps_interrupt_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if(value)
		mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	else
		mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_DISABLE);

	return count;
}

static ssize_t ps_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val;
	struct LTR559_data *LTR559 = sensor_info;

	ret = ps_offset_readback(&rdback_val, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS offset readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static ssize_t ps_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t ps_offset = 0;
	struct LTR559_data *LTR559 = sensor_info;

	ret = kstrtou16(buf, 10, &ps_offset);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "kstrtoint error\n");
		return ret;
	}

	if (ps_offset > 1023) {
		ps_offset = 1023;
	}
	dev_dbg(&LTR559->i2c_client->dev, "%s: store value = %d\n",
			__func__, ps_offset);

	ret = ps_offset_setup(ps_offset, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: set ps offset Fail...\n",
				__func__);
		return (-1);
	}

	return count;
}

static ssize_t LTR559_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, count = 0;
	u8 buffer;

	for(i = 0x80; i < 0x9f; i++)
	{
		if(i > 0x9a && i < 0x9e) continue;
		buffer = i;
		if(!I2C_Read(&buffer, 1)) {
			count += scnprintf(buf + count, PAGE_SIZE,
					"[%02x] = %02x\n", i, buffer);
		} else {
			count += scnprintf(buf + count, PAGE_SIZE, "[%02x] = ERR\n", i);
		}
		pr_info("[LTR559] i2c read 0x%02x, 0x%02x\n", i, buffer);
	}

	return count;
}

static ssize_t LTR559_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 buffer[2];
	unsigned int tmp[2];
#if 1
	if(2 != sscanf(buf, "%x %x", &tmp[0], &tmp[1])) {
		return 0;
	}
	buffer[0] = tmp[0];
	buffer[1] = tmp[1];
	I2C_Write(buffer, 2);
#endif
	return count;
}


static ssize_t ltr559_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, count = 0;
	u8 buffer;

	count += scnprintf(buf, PAGE_SIZE, "%d\n", 1);

	return count;
}

static ssize_t ltr559_self_test_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 buffer[2];
	return count;
}
static DEVICE_ATTR(als_data, 0444, als_data_show, NULL);
static DEVICE_ATTR(als_enable, 0664, als_enable_show, als_enable_store);
static DEVICE_ATTR(ps_enable, 0664, ps_enable_show, ps_enable_store);
static DEVICE_ATTR(ps_data, 0444, ps_data_show, NULL);
static DEVICE_ATTR(ps_interrupt, 0664, ps_interrupt_show, ps_interrupt_store);
static DEVICE_ATTR(ps_offset, 0664, ps_offset_show, ps_offset_store);
static DEVICE_ATTR(reg, 0664, LTR559_reg_show, LTR559_reg_store);
static DEVICE_ATTR(self_test, 0664, ltr559_self_test_show, ltr559_self_test_store);

static struct attribute *LTR559_attributes[] = {
	&dev_attr_als_data,
	&dev_attr_als_enable,
	&dev_attr_ps_enable,
	&dev_attr_ps_data,
	&dev_attr_ps_interrupt,
	&dev_attr_ps_offset,
	&dev_attr_reg,
	NULL
};

static struct attribute_group LTR559_attribute_group = {
	.attrs = LTR559_attributes,
};

static int als_setup(struct LTR559_data *LTR559)
{
	int ret;

	LTR559->als_input_dev = input_allocate_device();
	if (!LTR559->als_input_dev) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: ALS Input Allocate Device Fail...\n",
				__func__);
		return -ENOMEM;
	}
	LTR559->als_input_dev->name = "LTR559_als";
	set_bit(EV_ABS, LTR559->als_input_dev->evbit);
	input_set_abs_params(LTR559->als_input_dev, ABS_MISC,
			ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(LTR559->als_input_dev);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: ALS Register Input Device Fail...\n",
				__func__);
		goto err_als_register_input_device;
	}

	/*
	ret = misc_register(&als_misc);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: ALS Register Misc Device Fail...\n",
				__func__);
		goto err_als_register_misc_device;
	}

	return ret;

err_als_register_misc_device:
	input_unregister_device(LTR559->als_input_dev);
	*/
err_als_register_input_device:
	input_free_device(LTR559->als_input_dev);

	return ret;
}

static int ps_setup(struct LTR559_data *LTR559)
{
	int ret;

	LTR559->ps_input_dev = input_allocate_device();
	if (!LTR559->ps_input_dev) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS Input Allocate Device Fail...\n",
				__func__);
		return -ENOMEM;
	}
	LTR559->ps_input_dev->name = "LTR559_ps";
	set_bit(EV_ABS, LTR559->ps_input_dev->evbit);
	input_set_abs_params(LTR559->ps_input_dev, ABS_DISTANCE,
			PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(LTR559->ps_input_dev);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS Register Input Device Fail...\n",
				__func__);
		goto err_ps_register_input_device;
	}

	/*
	ret = misc_register(&ps_misc);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS Register Misc Device Fail...\n",
				__func__);
		goto err_ps_register_misc_device;
	}

	return ret;

err_ps_register_misc_device:
	input_unregister_device(LTR559->ps_input_dev);
	*/
err_ps_register_input_device:
	input_free_device(LTR559->ps_input_dev);

	return ret;
}

static uint8_t _check_part_id(struct LTR559_data *LTR559)
{
	uint8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTR559_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: Read failure :0x%02X",
				__func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&LTR559->i2c_client->dev, "%s: Part failure miscompare"
				" act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}

	return 0;
}

static int LTR559_setup(struct LTR559_data *LTR559)
{
	int ret = 0;

	/* Reset the devices */
	ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _LTR559_set_bit(LTR559->i2c_client, CLR_BIT, LTR559_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&LTR559->i2c_client->dev, "%s: Reset LTR559 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(LTR559) < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

#if 1
	ret = LTR559_gpio_irq(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: GPIO Request Fail...\n", __func__);
		goto err_out1;
	}
	dev_dbg(&LTR559->i2c_client->dev, "%s Requested interrupt\n", __func__);
#endif

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&LTR559->i2c_client->dev, "%s: Set LTR559 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
#if ACT_INTERRUPT
	//ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_INTERRUPT, INT_MODE_ALSPS_TRIG);
	ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_INTERRUPT, INT_MODE_PS_TRIG);
#else
	ret = _LTR559_set_bit(LTR559->i2c_client, SET_BIT, LTR559_INTERRUPT, INT_MODE_00);
#endif
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&LTR559->i2c_client->dev, "%s Enabled interrupt to device\n", __func__);

	/* als initializing ... */
	ret = als_enable_init(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s Unable to enable ALS", __func__);
		goto err_out2;
	}

	/* ps initializing ... */
	ret = ps_enable_init(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s Unable to enable PS", __func__);
		goto err_out2;
	}

	return ret;

err_out2:
err_out1:
	dev_err(&LTR559->i2c_client->dev, "%s Unable to setup device\n", __func__);

	return ret;
}

static inline int mz_light_iio_enable(struct mz_light_data *mz_light);
static inline int mz_light_iio_disable(struct mz_light_data *mz_light);
static void LTR559_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	struct LTR559_data *LTR559 = sensor_info;

	if (LTR559->is_suspend != 0) {
		dev_err(&LTR559->i2c_client->dev, "%s Asked to suspend when already suspended\n", __func__);
		return;
	}
	LTR559->is_suspend = 1;

	/* Save away the state of the devices at suspend point */
	LTR559->als_suspend_enable_flag = atomic_read(&LTR559->mz_light.enabled);

	ret = mz_light_iio_disable(&LTR559->mz_light);

	if (ret) {
		dev_err(&LTR559->i2c_client->dev, "%s Unable to complete suspend\n", __func__);
	} else {
		dev_info(&LTR559->i2c_client->dev, "%s Suspend completed\n", __func__);
	}

	/* we need to keep proximity working required by flyme */
	if (LTR559->sensor_phone_calling || LTR559->mz_proximity.trig_wk_state) {
		/* when phone calling, we enable proximity to wake up the phone */
		mt_eint_unmask(CUST_EINT_ALS_NUM);
	} else {
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
}

static void LTR559_late_resume(struct early_suspend *h)
{
	struct LTR559_data *LTR559 = sensor_info;
	int ret = 0;

	if (LTR559->is_suspend != 1) {
		dev_err(&LTR559->i2c_client->dev, "%s Asked to resume when not suspended\n", __func__);
		return;
	}
	LTR559->is_suspend = 0;

	/* If ALS was enbled before suspend, enable during resume */
	if (LTR559->als_suspend_enable_flag) {
		ret += mz_light_iio_enable(&LTR559->mz_light);
		LTR559->als_suspend_enable_flag = 0;
	}

	if (ret) {
		dev_err(&LTR559->i2c_client->dev, "%s Unable to complete resume\n", __func__);
	} else {
		dev_info(&LTR559->i2c_client->dev, "%s Resume completed\n", __func__);
	}

	mt_eint_unmask(CUST_EINT_ALS_NUM);

}


/* iio device interfaces */
#define ALS_SAMPLES_TO_DISCARD 80
static int mz_light_ltr559_enable(struct mz_light_data *mz_light)
{
	struct LTR559_data *LTR559 = mz_light->priv;
	mz_light->samples_to_discard = ALS_SAMPLES_TO_DISCARD;
	return als_set_enable(LTR559, 1);
}

static int mz_light_ltr559_disable(struct mz_light_data *mz_light)
{
	struct LTR559_data *LTR559 = mz_light->priv;
	// we keep the als always enable, so that we can push als data as soon as possible after resume.
	//return als_set_enable(LTR559, 0);
	return 0;
}

static int mz_light_ltr559_get_data(struct mz_light_data *mz_light, uint16_t *data_buf)
{

	struct LTR559_data *LTR559 = mz_light->priv;
	data_buf[0] = read_als_adc_value(LTR559);
	return 1;
}

struct mz_light_ops ltr559_ligth_ops = {
	.enable_device  = mz_light_ltr559_enable,
	.disable_device = mz_light_ltr559_disable,
	.get_data       = mz_light_ltr559_get_data,
};

static inline int mz_light_iio_enable(struct mz_light_data *mz_light)
{
	int ret = 0;

	if (!atomic_cmpxchg(&mz_light->enabled, 0, 1)) {

		if (mz_light->ops->enable_device)
			ret = mz_light->ops->enable_device(mz_light);

		if (ret < 0) {
			atomic_set(&mz_light->enabled, 0);
			return ret;
		}

		dev_dbg(mz_light->dev, "light iio work start\n");
		mz_light->pollrate = 400;
		queue_delayed_work(mz_light->iio_workq, &mz_light->iio_work,
			0);
	}

	return 0;
}

static inline int mz_light_iio_disable(struct mz_light_data *mz_light)
{
	if (atomic_cmpxchg(&mz_light->enabled, 1, 0)) {

		dev_dbg(mz_light->dev, "light iio work stop\n");
		cancel_delayed_work_sync(&mz_light->iio_work);

		if (mz_light->ops->disable_device)
			mz_light->ops->disable_device(mz_light);
	}

	return 0;
}


#define MZ_LIGHT_IIO_NUMBER_DATA_CHANNELS		1
#define MZ_LIGHT_IIO_BYTE_FOR_CHANNEL		2
static void mz_light_iio_func(struct work_struct *work)
{
	int ret;
	int i, n = 0;
	struct mz_light_data *mz_light;
	uint16_t light_data[3];
	int64_t timestamp;
	struct timespec ts;

	mz_light = container_of((struct delayed_work *)work,
					struct mz_light_data, iio_work);

	//dev_dbg(mz_light->dev, "%s: enter\n", __func__);

	if (mz_light->ops->get_data)
		ret = mz_light->ops->get_data(mz_light, light_data);
	else
		ret = -1;

	if (ret > 0) {
		#if 0
		dev_dbg(mz_light->dev, "iio_buffer_data: ================\n");
		#endif
		for (i = 0; i < MZ_LIGHT_IIO_NUMBER_DATA_CHANNELS; i++) {

			if (mz_light->indio_dev->active_scan_mask == NULL) {
				printk(KERN_EMERG "ltr559 bug: als active_scan_mask: %p\n",
					mz_light->indio_dev->active_scan_mask);
				return;
			}

			if (test_bit(i, mz_light->indio_dev->active_scan_mask)) {
				memcpy(&mz_light->iio_buffer_data[n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL],
						&light_data[i],
						MZ_LIGHT_IIO_BYTE_FOR_CHANNEL);
				#if 0
				dev_dbg(mz_light->dev, "ltr559 lux: %d ",
					*(int16_t *)&mz_light->iio_buffer_data[n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL]);

				if (*(int16_t *)&mz_light->iio_buffer_data[n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL] == 0) {
					printk(KERN_EMERG "%d ",
						*(int16_t *)&mz_light->iio_buffer_data[n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL]);
				}
				#endif
				n++;
			}
		}

		/* timestamp = iio_get_time_ns(); */
		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);
		if (mz_light->indio_dev->scan_timestamp) {
			*(s64 *)((u8 *)mz_light->iio_buffer_data +
				ALIGN(n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;
				#if 0
				dev_dbg(mz_light->dev, "%lld ",
					*(s64 *)((u8 *)mz_light->iio_buffer_data +
					ALIGN(n * MZ_LIGHT_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))));
				#endif
		}
		#if 0
		dev_dbg(mz_light->dev, "\niio_buffer_data ============= \n");
		dev_dbg(mz_light->dev, "%s: push buffer\n", __func__);
		#endif
		iio_push_to_buffers(mz_light->indio_dev, mz_light->iio_buffer_data);
		#if LTR559_DEBUG_PUSH_TIME
		if (ltr559_push_time_debug) {
			ltr559_push_time_debug = 0;
			get_monotonic_boottime(&ltr559_time_after);
			printk(KERN_EMERG "first push time [%d]: %lu.%03lu\n", samples_discarded,
				ltr559_time_after.tv_sec - ltr559_time_before.tv_sec,
				ltr559_time_after.tv_nsec / NSEC_PER_MSEC - ltr559_time_before.tv_nsec / NSEC_PER_MSEC);
		}
		#endif

	}

out:
	queue_delayed_work(mz_light->iio_workq, &mz_light->iio_work,
					msecs_to_jiffies(mz_light->pollrate));
	//dev_dbg(mz_light->dev, "%s: leave\n", __func__);
}


int mz_light_trig_set_state(struct iio_trigger *trig, bool state)
{
	int ret;
	struct mz_light_data *mz_light = iio_device_get_drvdata(
		iio_trigger_get_drvdata(trig));

	dev_info(mz_light->dev, "%s: enter, state: %d\n", __func__, state);
	if (state) {
		#if LTR559_DEBUG_PUSH_TIME
		ltr559_push_time_debug = 1;
		get_monotonic_boottime(&ltr559_time_before);
		#endif
		ret = mz_light_iio_enable(mz_light);
	} else {
		ret = mz_light_iio_disable(mz_light);
	}

	return ret;
}


static const struct iio_trigger_ops mz_light_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &mz_light_trig_set_state,
};

static inline irqreturn_t mz_light_iio_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}


static int mz_light_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	return iio_sw_buffer_preenable(indio_dev);
}

static int mz_light_iio_buffer_postenable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_light_data *mz_light = iio_device_get_drvdata(indio_dev);
	dev_info(mz_light->dev, "%s: enter\n", __func__);

	err = iio_triggered_buffer_postenable(indio_dev);
	if (err < 0) {
		dev_err(mz_light->dev,
			"failed to call iio_triggered_buffer_postenable\n");
		return err;
	}

	dev_info(mz_light->dev, "%s: leave\n", __func__);
	return 0;
}

static int mz_light_iio_buffer_predisable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_light_data *mz_light = iio_device_get_drvdata(indio_dev);
	dev_info(mz_light->dev, "%s: enter\n", __func__);
	err = iio_triggered_buffer_predisable(indio_dev);
	if (err < 0) {
		dev_err(mz_light->dev,
			"failed to call iio_triggered_buffer_predisable\n");
		return err;
	}

	dev_info(mz_light->dev, "%s: leave\n", __func__);
	return 0;
}


static int mz_light_iio_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_light_data *mz_light = iio_device_get_drvdata(indio_dev);
	dev_info(mz_light->dev, "%s: enter\n", __func__);
	mz_light_iio_disable(mz_light);
	dev_info(mz_light->dev, "%s: leave\n", __func__);
	return 0;
}

static const struct iio_buffer_setup_ops mz_light_iio_buffer_setup_ops = {
	.preenable  = &mz_light_iio_buffer_preenable,
	.postenable = &mz_light_iio_buffer_postenable,
	.predisable = &mz_light_iio_buffer_predisable,
	.postdisable = &mz_light_iio_buffer_postdisable,
};

static int mz_light_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	u8 outdata[2];
	struct mz_light_data *mz_light = iio_device_get_drvdata(indio_dev);
	dev_dbg(mz_light->dev, "%s: enter\n", __func__);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			dev_dbg(mz_light->dev, "%s: leave\n", __func__);
			return -EBUSY;
		}

		/* get data from device */
		/* outdata = xxx */

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);
		dev_dbg(mz_light->dev, "%s: leave\n", __func__);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* the scale is always 1 */
		*val = 1;
		*val2 = 0;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
	dev_dbg(mz_light->dev, "%s: leave\n", __func__);
	return 0;
}


#define MZ_LIGHT_IIO_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 16 - bits, \
		.storagebits = 16, \
		.endianness = endian, \
	}, \
}

#define MZ_LIGHT_X_L_ADDR		0x28
static const struct iio_chan_spec mz_light_iio_ch[] = {
	MZ_LIGHT_IIO_CHANNELS(IIO_LIGHT, 0, IIO_MOD_X, IIO_LE,
					16, MZ_LIGHT_X_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};


ssize_t mz_iio_sysfs_get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

ssize_t mz_iio_sysfs_flush_fifo(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t mz_iio_sysfs_sampling_frequency_avail(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
				"%d\n", 10);
}

static ssize_t mz_iio_sysfs_get_sampling_frequency(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_info("%s: enter\n", __func__);
	return sprintf(buf, "%d\n", 10);
}

static ssize_t mz_iio_sysfs_set_sampling_frequency(
			struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	pr_info("%s: enter\n", __func__);

	pr_info("%s: leave\n", __func__);
	return size;
}

static ssize_t mz_iio_sysfs_get_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i, count = 0;
	u8 buffer;

	for(i = 0x80; i < 0x9f; i++)
	{
		if(i > 0x9a && i < 0x9e) continue;
		buffer = i;
		if(!I2C_Read(&buffer, 1)) {
			count += scnprintf(buf + count, PAGE_SIZE,
					"[%02x] = %02x\n", i, buffer);
		} else {
			count += scnprintf(buf + count, PAGE_SIZE, "[%02x] = ERR\n", i);
		}
		pr_info("[LTR559] i2c read 0x%02x, 0x%02x\n", i, buffer);
	}

	return count;
}

static ssize_t mz_iio_sysfs_set_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 buffer[2];
	unsigned int tmp[2];
#if 1
	if(2 != sscanf(buf, "%x %x", &tmp[0], &tmp[1])) {
		return 0;
	}
	buffer[0] = tmp[0];
	buffer[1] = tmp[1];
	I2C_Write(buffer, 2);
#endif
	return count;
}


static ssize_t mz_iio_sysfs_get_ps_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val;
	struct LTR559_data *LTR559 = sensor_info;

	ret = ps_offset_readback(&rdback_val, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,
				"%s: PS offset readback Fail...\n", __func__);
		return (-1);
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

static ssize_t mz_iio_sysfs_set_ps_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t ps_offset = 0;
	struct LTR559_data *LTR559 = sensor_info;

	ret = kstrtou16(buf, 10, &ps_offset);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "kstrtoint error\n");
		return ret;
	}

	if (ps_offset > 1023) {
		ps_offset = 1023;
	}
	dev_info(&LTR559->i2c_client->dev, "%s: store value = %d\n",
			__func__, ps_offset);

	ret = ps_offset_setup(ps_offset, LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: set ps offset Fail...\n",
				__func__);
		return (-1);
	}

	return count;
}

static ssize_t mz_iio_sysfs_get_als_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct LTR559_data *LTR559 = sensor_info;

	//LTR559->mode = ALS;
	//value = read_adc_value(LTR559);
	value = read_als_adc_value(LTR559);
	//input_report_abs(LTR559->als_input_dev, ABS_MISC, value);
	//input_sync(LTR559->als_input_dev);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}

static ssize_t  mz_iio_sysfs_get_ps_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct LTR559_data *LTR559 = sensor_info;

	//LTR559->mode = PS;
	//value = read_adc_value(LTR559);
	value = read_ps_adc_value(LTR559);
	//ret = sprintf(buf, "%d\n", value);
	ret = sprintf(buf, "%d\n", value);

	return ret;
}


static ssize_t mz_iio_sysfs_get_sensor_phone_calling(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct LTR559_data *LTR559 = sensor_info;

	return sprintf(buf, "%d\n", LTR559->sensor_phone_calling);
}

static ssize_t mz_iio_sysfs_set_sensor_phone_calling(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int flag;
	struct LTR559_data *LTR559 = sensor_info;

	err = kstrtoint(buf, 10, &flag);
	if (err < 0)
		return err;

	LTR559->sensor_phone_calling = !!flag;

	return err < 0 ? err : size;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
			mz_iio_sysfs_get_sampling_frequency,
			mz_iio_sysfs_set_sampling_frequency);

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(mz_iio_sysfs_sampling_frequency_avail);
static IIO_DEVICE_ATTR(hw_fifo_lenght, S_IRUGO, mz_iio_sysfs_get_hw_fifo_lenght, NULL, 0);
static IIO_DEVICE_ATTR(flush, S_IWUSR, NULL, mz_iio_sysfs_flush_fifo, 0);
static IIO_DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, mz_iio_sysfs_get_reg, mz_iio_sysfs_set_reg, 0);
static IIO_DEVICE_ATTR(ps_offset, S_IWUSR | S_IRUGO, mz_iio_sysfs_get_ps_offset, mz_iio_sysfs_set_ps_offset, 0);
static IIO_DEVICE_ATTR(ps_data, S_IRUGO, mz_iio_sysfs_get_ps_data, NULL, 0);
static IIO_DEVICE_ATTR(als_data, S_IRUGO, mz_iio_sysfs_get_als_data, NULL, 0);
static IIO_DEVICE_ATTR(sensor_phone_calling, S_IWUSR | S_IRUGO,
				mz_iio_sysfs_get_sensor_phone_calling,
				mz_iio_sysfs_set_sensor_phone_calling, 0);

static struct attribute *mz_light_iio_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	NULL,
};

static const struct attribute_group mz_light_iio_attribute_group = {
	.attrs = mz_light_iio_attributes,
};


static const struct iio_info mz_light_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &mz_light_iio_attribute_group,
	.read_raw = &mz_light_iio_read_raw,
};


static int mz_sensors_ltr559_als_get_name(struct device *dev, char **name)
{
	*name = "ltr559-als";

	dev_info(dev, "ltr559_als_get_name succeeded\n");
	return 0;
}

struct meizu_sensors_ops meizu_ltr559_als_ops = {
	.get_name = &mz_sensors_ltr559_als_get_name,
};

static int mz_light_iio_init(struct mz_light_data *mz_light)
{
	int ret = -1;

	mz_light->iio_buffer_data = kmalloc(32, GFP_KERNEL);
	if (mz_light->iio_buffer_data == NULL) {
		dev_err(mz_light->dev, "failed to call kmalloc\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&mz_light->iio_work, mz_light_iio_func);

	mz_light->iio_workq = create_singlethread_workqueue("mz_light_iio");
	if (!mz_light->iio_workq) {
		dev_err(mz_light->dev,
			"failed to call create_singlethread_workqueue\n");
		goto free_iio_buffer_data;
	}

	mz_light->indio_dev = iio_device_alloc(0);
	if (!mz_light->indio_dev) {
		dev_err(mz_light->dev, "failed to call iio_device_alloc\n");
		goto wq_destory;
	}

	mz_light->indio_dev->name = kasprintf(GFP_KERNEL, "%s_%s", "lsm6ds3", "light");
	mz_light->indio_dev->info = &mz_light_iio_info;
	mz_light->indio_dev->channels = (struct iio_chan_spec const	*)&mz_light_iio_ch;
	mz_light->indio_dev->num_channels = ARRAY_SIZE(mz_light_iio_ch);
	mz_light->indio_dev->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(mz_light->indio_dev, mz_light);

	ret = iio_triggered_buffer_setup(mz_light->indio_dev,
				&mz_light_iio_handler_empty, NULL,
				&mz_light_iio_buffer_setup_ops);
	if (ret < 0) {
		dev_err(mz_light->dev, "failed to setup triggered buffer\n");
		goto iio_device_free;
	}

	mz_light->trig = iio_trigger_alloc("%s-trigger", "light");
	if (!mz_light->trig) {
		dev_err(mz_light->dev, "failed to call iio_trigger_alloc\n");
		goto iio_deallocate_buffer;
	}

	mz_light->trig->ops = &mz_light_trigger_ops;
	mz_light->trig->dev.parent = mz_light->dev;
	iio_trigger_set_drvdata(mz_light->trig, mz_light->indio_dev);

	ret = iio_trigger_register(mz_light->trig);
	if (ret < 0) {
		dev_err(mz_light->dev, "failed to register iio trigger.\n");
		goto iio_deallocate_trigger;
	}
	mz_light->indio_dev->trig = mz_light->trig;

	ret = iio_device_register(mz_light->indio_dev);
	if (ret < 0) {
		dev_err(mz_light->dev, "failed to register iio device.\n");
		goto iio_deallocate_trigger;
	}
#if 0
	ret = meizu_sysfslink_register_name(&mz_light->indio_dev->dev, "light");
	if (ret < 0) {
		dev_err(mz_light->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_deallocate_trigger;
	}
#endif
	ret = meizu_sensor_register(MEIZU_SENSOR_ID_ALS,
		&mz_light->indio_dev->dev, &meizu_ltr559_als_ops);
	if (ret < 0) {
		dev_err(mz_light->dev, "failed to meizu_sensor_register.\n");
		goto iio_deallocate_trigger;
	}

	return 0;


iio_deallocate_trigger:
	iio_trigger_free(mz_light->trig);
iio_deallocate_buffer:
	iio_triggered_buffer_cleanup(mz_light->indio_dev);
iio_device_free:
	iio_device_free(mz_light->indio_dev);
wq_destory:
	destroy_workqueue(mz_light->iio_workq);
free_iio_buffer_data:
	kfree(mz_light->iio_buffer_data);
err_kmalloc:
	return ret;
}





/* iio device interfaces */
static int mz_proximity_ltr559_enable(struct mz_proximity_data *mz_proximity)
{
	struct LTR559_data *LTR559 = mz_proximity->priv;
	pr_info("%s\n", __func__);
	return ps_set_enable(LTR559, 1);
}

static int mz_proximity_ltr559_disable(struct mz_proximity_data *mz_proximity)
{
	struct LTR559_data *LTR559 = mz_proximity->priv;
	pr_info("%s\n", __func__);
	return ps_set_enable(LTR559, 0);
}

static int mz_proximity_ltr559_get_data(struct mz_proximity_data *mz_proximity, uint16_t *data_buf)
{

	struct LTR559_data *LTR559 = mz_proximity->priv;
	pr_info("%s\n", __func__);
	data_buf[0] = mt_get_gpio_in(GPIO_ALS_EINT_PIN);
	return 1;
}

struct mz_proximity_ops ltr559_proximity_ops = {
	.enable_device  = mz_proximity_ltr559_enable,
	.disable_device = mz_proximity_ltr559_disable,
	.get_data       = mz_proximity_ltr559_get_data,
};

static inline int mz_proximity_iio_enable(struct mz_proximity_data *mz_proximity)
{
	int ret = 0;

	dev_info(mz_proximity->dev, "%s: enter\n", __func__);
	if (atomic_add_return(1, &mz_proximity->enabled) == 1) {

		if (mz_proximity->ops->enable_device)
			ret = mz_proximity->ops->enable_device(mz_proximity);

		if (ret < 0) {
			atomic_sub(1, &mz_proximity->enabled);
			return ret;
		}
#if 0
		dev_dbg(mz_proximity->dev, "proximity iio work start\n");
		queue_delayed_work(mz_proximity->iio_workq, &mz_proximity->iio_work,
			msecs_to_jiffies(mz_proximity->pollrate));
#endif
		mt_eint_unmask(CUST_EINT_ALS_NUM);
	}

	return 0;
}

static inline int mz_proximity_iio_disable(struct mz_proximity_data *mz_proximity)
{
	dev_info(mz_proximity->dev, "%s: enter\n", __func__);
	struct LTR559_data *LTR559 = mz_proximity->priv;
	if (atomic_sub_return(1, &mz_proximity->enabled) == 0) {
#if 0
		dev_dbg(mz_proximity->dev, "proximity iio work stop\n");
		cancel_delayed_work_sync(&mz_proximity->iio_work);
#endif
		mt_eint_mask(CUST_EINT_ALS_NUM);
		cancel_work_sync(&LTR559->ps_work);

		if (mz_proximity->ops->disable_device)
			mz_proximity->ops->disable_device(mz_proximity);
	}

	return 0;
}



#define MZ_PROXIMITY_IIO_NUMBER_DATA_CHANNELS		1
#define MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL		2

static void mz_proximity_iio_push_data(struct mz_proximity_data *mz_proximity,
					struct iio_dev *indio_dev,
					int ps_state)
{
	int ret;
	int i, n = 0;
	uint16_t proximity_data[3] = {0,0,0};
	int64_t timestamp;
	struct timespec ts;

	proximity_data[0] = ps_state;

	dev_info(mz_proximity->dev, "[ps_state]: %d \n", ps_state);
	for (i = 0; i < MZ_PROXIMITY_IIO_NUMBER_DATA_CHANNELS; i++) {

		if (indio_dev->active_scan_mask == NULL) {
			printk(KERN_EMERG "ltr559 bug: ps active_scan_mask: %p\n",
				indio_dev->active_scan_mask);
			return;
		}

		if (test_bit(i, indio_dev->active_scan_mask)) {
			memcpy(&mz_proximity->iio_buffer_data[n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL],
					&proximity_data[i],
					MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL);
			#if 0
			dev_dbg(mz_proximity->dev, "%d ",
				*(int16_t *)&mz_proximity->iio_buffer_data[n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL]);
			#endif
			n++;
		}
	}

	/* timestamp = iio_get_time_ns(); */
	get_monotonic_boottime(&ts);
	timestamp = timespec_to_ns(&ts);
	if (indio_dev->scan_timestamp) {
		*(s64 *)((u8 *)mz_proximity->iio_buffer_data +
			ALIGN(n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL,
					sizeof(s64))) = timestamp;
			#if 0
			dev_dbg(mz_proximity->dev, "%lld ",
				*(s64 *)((u8 *)mz_proximity->iio_buffer_data +
				ALIGN(n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL,
					sizeof(s64))));
			#endif
	}
	//dev_dbg(mz_proximity->dev, "\niio_buffer_data ============= \n");
	//dev_dbg(mz_proximity->dev, "%s: push buffer\n", __func__);

	dev_dbg(mz_proximity->dev, "[push]: %d \n", *(uint16_t*)mz_proximity->iio_buffer_data);
	iio_push_to_buffers(indio_dev, mz_proximity->iio_buffer_data);

	//dev_dbg(mz_proximity->dev, "%s: leave\n", __func__);
}

static void mz_proximity_iio_func(struct work_struct *work)
{
	int ret;
	int i, n = 0;
	struct mz_proximity_data *mz_proximity;
	uint16_t proximity_data[3];
	int64_t timestamp;
	struct timespec ts;

	mz_proximity = container_of((struct delayed_work *)work,
					struct mz_proximity_data, iio_work);

	dev_dbg(mz_proximity->dev, "%s: enter\n", __func__);

	if (mz_proximity->ops->get_data)
		ret = mz_proximity->ops->get_data(mz_proximity, proximity_data);
	else
		ret = -1;

	if (ret > 0) {
		dev_dbg(mz_proximity->dev, "iio_buffer_data: ================\n");
		for (i = 0; i < MZ_PROXIMITY_IIO_NUMBER_DATA_CHANNELS; i++) {

			if (mz_proximity->indio_dev->active_scan_mask == NULL) {
				printk(KERN_EMERG "ltr559 bug: ps active_scan_mask: %p\n",
					mz_proximity->indio_dev->active_scan_mask);
				return;
			}

			if (test_bit(i, mz_proximity->indio_dev->active_scan_mask)) {
				memcpy(&mz_proximity->iio_buffer_data[n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL],
						&proximity_data[i],
						MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL);
				#if 1
				dev_dbg(mz_proximity->dev, "%d ",
					*(int16_t *)&mz_proximity->iio_buffer_data[n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL]);
				#endif
				n++;
			}
		}

		/* timestamp = iio_get_time_ns(); */
		get_monotonic_boottime(&ts);
		timestamp = timespec_to_ns(&ts);
		if (mz_proximity->indio_dev->scan_timestamp) {
			*(s64 *)((u8 *)mz_proximity->iio_buffer_data +
				ALIGN(n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))) = timestamp;
				#if 1
				dev_dbg(mz_proximity->dev, "%lld ",
					*(s64 *)((u8 *)mz_proximity->iio_buffer_data +
					ALIGN(n * MZ_PROXIMITY_IIO_BYTE_FOR_CHANNEL,
						sizeof(s64))));
				#endif
		}
		dev_dbg(mz_proximity->dev, "\niio_buffer_data ============= \n");
		dev_dbg(mz_proximity->dev, "%s: push buffer\n", __func__);
		iio_push_to_buffers(mz_proximity->indio_dev, mz_proximity->iio_buffer_data);
	}
#if 0
	queue_delayed_work(mz_proximity->iio_workq, &mz_proximity->iio_work,
					msecs_to_jiffies(mz_proximity->pollrate));
#endif
	dev_dbg(mz_proximity->dev, "%s: leave\n", __func__);
}


int mz_proximity_trig_set_state(struct iio_trigger *trig, bool state)
{
	int ret;
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(
		iio_trigger_get_drvdata(trig));

	dev_info(mz_proximity->dev, "%s: enter, state: %d\n", __func__, state);

	mz_proximity->trig_state = state;
	if (state) {
		schedule_work(&sensor_info->ps_work);
	}

	return ret;
}


static const struct iio_trigger_ops mz_proximity_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &mz_proximity_trig_set_state,
};


int mz_proximity_wk_trig_set_state(struct iio_trigger *trig, bool state)
{
	int ret;
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(
		iio_trigger_get_drvdata(trig));

	dev_info(mz_proximity->dev, "%s: enter, state: %d\n", __func__, state);

	mz_proximity->trig_wk_state = state;
	if (state) {
		schedule_work(&sensor_info->ps_work);
	}

	return ret;
}


static const struct iio_trigger_ops mz_proximity_wk_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &mz_proximity_wk_trig_set_state,
};

static inline irqreturn_t mz_proximity_iio_handler_empty(int irq, void *p)
{
	return IRQ_HANDLED;
}


static int mz_proximity_iio_buffer_preenable(struct iio_dev *indio_dev)
{
	int ret;
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(indio_dev);

	dev_info(mz_proximity->dev, "%s: enter\n", __func__);

	ret = mz_proximity_iio_enable(mz_proximity);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "mz_proximity_iio_enable failed\n");
		return ret;
	}

	ret = iio_sw_buffer_preenable(indio_dev);

	dev_info(mz_proximity->dev, "%s: leave\n", __func__);

	return ret;
}

static int mz_proximity_iio_buffer_postdisable(struct iio_dev *indio_dev)
{
	int err;
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(indio_dev);
	dev_info(mz_proximity->dev, "%s: enter\n", __func__);
	mz_proximity_iio_disable(mz_proximity);
	dev_info(mz_proximity->dev, "%s: leave\n", __func__);
	return 0;
}

static const struct iio_buffer_setup_ops mz_proximity_iio_buffer_setup_ops = {
	.preenable  = &mz_proximity_iio_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &mz_proximity_iio_buffer_postdisable,
};



#define LTR559_CALIBBIAS_COUNT 20
static int mz_proximity_ltr559_get_calibbias(struct mz_proximity_data *mz_proximity)
{
	int i, ret;
	uint16_t raw;
	uint32_t sum = 0;

	struct LTR559_data *LTR559 = mz_proximity->priv;

	mz_proximity_iio_disable(mz_proximity);
	mz_proximity_iio_enable(mz_proximity);

	for (i=0; i<LTR559_CALIBBIAS_COUNT; i++) {
		raw = read_ps_adc_value(LTR559);

		sum += raw;
		msleep(70);
	}

	mz_proximity->bias = sum / LTR559_CALIBBIAS_COUNT;
	return 0;
}

static int mz_proximity_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *ch, int *val, int *val2, long mask)
{
	int err = 0;
	u8 outdata[2];
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(indio_dev);
	dev_dbg(mz_proximity->dev, "%s: enter\n", __func__);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);

		if (indio_dev->currentmode == INDIO_BUFFER_TRIGGERED) {
			mutex_unlock(&indio_dev->mlock);
			dev_dbg(mz_proximity->dev, "%s: leave\n", __func__);
			return -EBUSY;
		}

		/* get data from device */
		/* outdata = xxx */

		*val = (s16)get_unaligned_le16(outdata);
		*val = *val >> ch->scan_type.shift;

		mutex_unlock(&indio_dev->mlock);
		dev_dbg(mz_proximity->dev, "%s: leave\n", __func__);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/* the scale is always 1 */
		*val = 1;
		*val2 = 0;

		return IIO_VAL_INT_PLUS_MICRO;


	case IIO_CHAN_INFO_CALIBBIAS:
		dev_dbg(mz_proximity->dev, "get channel[%d] calibbiaa.\n", ch->channel);

		mutex_lock(&indio_dev->mlock);

		if (!mz_proximity->bias) {
			err = mz_proximity_ltr559_get_calibbias(mz_proximity);
			if (err < 0) {
				mutex_unlock(&indio_dev->mlock);
				return err;
			}
		}

		mutex_unlock(&indio_dev->mlock);
		*val = mz_proximity->bias;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
	dev_dbg(mz_proximity->dev, "%s: leave\n", __func__);
	return 0;
}


static int mz_proximity_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	int err = 0;
	struct mz_proximity_data *mz_proximity = iio_device_get_drvdata(indio_dev);
	struct LTR559_data *LTR559 = mz_proximity->priv;
	dev_dbg(mz_proximity->dev, "%s: enter\n", __func__);

	switch (mask) {

	case IIO_CHAN_INFO_CALIBBIAS:

		err = ps_offset_setup(val, LTR559);
		if (err < 0)
			return -EINVAL;

		mz_proximity->bias = val;
		break;

	default:
		return -EINVAL;
	}

	return err;
}

#define MZ_PROXIMITY_IIO_CHANNELS(device_type, index, mod, endian, bits, addr) \
{ \
	.type = device_type, \
	.modified = 1, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
			BIT(IIO_CHAN_INFO_OFFSET) |\
			BIT(IIO_CHAN_INFO_CALIBBIAS), \
	.scan_index = index, \
	.channel2 = mod, \
	.address = addr, \
	.scan_type = { \
		.sign = 's', \
		.realbits = bits, \
		.shift = 16 - bits, \
		.storagebits = 16, \
		.endianness = endian, \
	}, \
}

#define MZ_PROXIMITY_X_L_ADDR		0x28
static const struct iio_chan_spec mz_proximity_iio_ch[] = {
	MZ_PROXIMITY_IIO_CHANNELS(IIO_PROXIMITY, 0, IIO_MOD_X, IIO_LE,
					16, MZ_PROXIMITY_X_L_ADDR),
	IIO_CHAN_SOFT_TIMESTAMP(3)
};

static struct attribute *mz_proximity_iio_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_hw_fifo_lenght.dev_attr.attr,
	&iio_dev_attr_flush.dev_attr.attr,
	&iio_dev_attr_reg.dev_attr.attr,
	&iio_dev_attr_ps_data.dev_attr.attr,
	&iio_dev_attr_als_data.dev_attr.attr,
	&iio_dev_attr_ps_offset.dev_attr.attr,
	&iio_dev_attr_sensor_phone_calling.dev_attr.attr,
	NULL,
};

static const struct attribute_group mz_proximity_iio_attribute_group = {
	.attrs = mz_proximity_iio_attributes,
};


static const struct iio_info mz_proximity_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &mz_proximity_iio_attribute_group,
	.read_raw = &mz_proximity_iio_read_raw,
	.write_raw = &mz_proximity_iio_write_raw,
};

static const struct iio_info mz_proximity_wk_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &mz_proximity_iio_attribute_group,
	.read_raw = &mz_proximity_iio_read_raw,
	.write_raw = &mz_proximity_iio_write_raw,
};


static int mz_sensors_ltr559_ps_calibrate(struct device *dev)
{
	int err;
	struct LTR559_data *LTR559 = sensor_info;

	ps_offset_setup(0, LTR559);

	err = mz_proximity_ltr559_get_calibbias(&LTR559->mz_proximity);
	if (err < 0) {
		dev_info(dev, "ltr559_ps_calibrate failed\n");
		return err;
	}

	ps_offset_setup(LTR559->mz_proximity.bias, LTR559);

	dev_info(dev, "ltr559_ps_calibrate succeeded\n");
	return 0;
}

static int mz_sensors_ltr559_ps_get_calibbias(struct device *dev, int32_t calibbias[3])
{
	struct LTR559_data *LTR559 = sensor_info;

	calibbias[0] = LTR559->mz_proximity.bias;

	dev_info(dev, "ltr559_ps_get_calibbias succeeded\n");
	return 0;
}

static int mz_sensors_ltr559_ps_get_name(struct device *dev, char **name)
{
	*name = "ltr559-ps";

	dev_info(dev, "ltr559_ps_get_name succeeded\n");
	return 0;
}


static int mz_sensors_ltr559_ps_get_irq_gpio(struct device *dev, int **state)
{
	struct LTR559_data *LTR559 = sensor_info;

	if (LTR559->mz_proximity.irq_gpio == 1)
		*state = 1;
	else
		*state = 0;

	dev_info(dev, "ltr559_ps_get_irq_gpio succeeded\n");
	return 0;
}

static int mz_sensors_ltr559_ps_set_enable(struct device *dev, int state)
{
	int ret;
	struct LTR559_data *LTR559 = sensor_info;

	if (state) {
		ret = mz_proximity_iio_enable(&LTR559->mz_proximity);
		LTR559->mz_proximity.irq_gpio = 0;
	} else {
		ret = mz_proximity_iio_disable(&LTR559->mz_proximity);
	}

	dev_info(dev, "ltr559_ps_set_enable ret: %d, state: %d\n", ret, state);
	return ret;
}


static int mz_sensors_ltr559_ps_get_enable(struct device *dev, int **state)
{
	struct LTR559_data *LTR559 = sensor_info;

	if (!!atomic_read(&LTR559->mz_proximity.enabled))
		*state = 1;
	else
		*state = 0;

	dev_info(dev, "ltr559_ps_get_irq_gpio succeeded\n");
	return 0;
}

struct meizu_sensors_ops meizu_ltr559_ps_ops = {
	.calibrate = &mz_sensors_ltr559_ps_calibrate,
	.get_calibbias = &mz_sensors_ltr559_ps_get_calibbias,
	.get_name = &mz_sensors_ltr559_ps_get_name,
	.get_irq_gpio = &mz_sensors_ltr559_ps_get_irq_gpio,
	.set_enable = &mz_sensors_ltr559_ps_set_enable,
	.get_enable = &mz_sensors_ltr559_ps_get_enable
};

static int mz_proximity_iio_init(struct mz_proximity_data *mz_proximity)
{
	int ret = -1;

	mz_proximity->irq_gpio = -1;
	mz_proximity->iio_buffer_data = kmalloc(32, GFP_KERNEL);
	if (mz_proximity->iio_buffer_data == NULL) {
		dev_err(mz_proximity->dev, "failed to call kmalloc\n");
		return -ENOMEM;
	}

	mz_proximity->indio_dev = iio_device_alloc(0);
	if (!mz_proximity->indio_dev) {
		dev_err(mz_proximity->dev, "failed to call iio_device_alloc\n");
		goto free_iio_buffer_data;
	}

	mz_proximity->indio_dev->name = kasprintf(GFP_KERNEL, "%s_%s", "lsm6ds3", "proximity");
	mz_proximity->indio_dev->info = &mz_proximity_iio_info;
	mz_proximity->indio_dev->channels = (struct iio_chan_spec const	*)&mz_proximity_iio_ch;
	mz_proximity->indio_dev->num_channels = ARRAY_SIZE(mz_proximity_iio_ch);
	mz_proximity->indio_dev->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(mz_proximity->indio_dev, mz_proximity);

	ret = iio_triggered_buffer_setup(mz_proximity->indio_dev,
				&mz_proximity_iio_handler_empty, NULL,
				&mz_proximity_iio_buffer_setup_ops);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to setup triggered buffer\n");
		goto iio_device_free;
	}

	mz_proximity->trig = iio_trigger_alloc("%s-trigger", "proximity");
	if (!mz_proximity->trig) {
		dev_err(mz_proximity->dev, "failed to call iio_trigger_alloc\n");
		goto iio_deallocate_buffer;
	}

	mz_proximity->trig->ops = &mz_proximity_trigger_ops;
	mz_proximity->trig->dev.parent = mz_proximity->dev;
	iio_trigger_set_drvdata(mz_proximity->trig, mz_proximity->indio_dev);

	ret = iio_trigger_register(mz_proximity->trig);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to register iio trigger.\n");
		goto iio_deallocate_trigger;
	}
	mz_proximity->indio_dev->trig = mz_proximity->trig;

	ret = iio_device_register(mz_proximity->indio_dev);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to register iio device.\n");
		goto iio_deallocate_trigger;
	}
#if 0
	ret = meizu_sysfslink_register_name(&mz_proximity->indio_dev->dev, "proximity");
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_deallocate_trigger;
	}
#endif
	ret = meizu_sensor_register(MEIZU_SENSOR_ID_PS,
		&mz_proximity->indio_dev->dev, &meizu_ltr559_ps_ops);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to meizu_sensor_register.\n");
		goto iio_deallocate_trigger;
	}

	return 0;


iio_deallocate_trigger:
	iio_trigger_free(mz_proximity->trig);
iio_deallocate_buffer:
	iio_triggered_buffer_cleanup(mz_proximity->indio_dev);
iio_device_free:
	iio_device_free(mz_proximity->indio_dev);
free_iio_buffer_data:
	kfree(mz_proximity->iio_buffer_data);
	return ret;
}


static int mz_proximity_wk_iio_init(struct mz_proximity_data *mz_proximity)
{
	int ret;

	mz_proximity->indio_dev_wk = iio_device_alloc(0);
	if (!mz_proximity->indio_dev_wk) {
		dev_err(mz_proximity->dev, "failed to call iio_device_alloc\n");
		return -1;
	}

	mz_proximity->indio_dev_wk->name = kasprintf(GFP_KERNEL, "%s_%s", "lsm6ds3", "ps_wk");
	mz_proximity->indio_dev_wk->info = &mz_proximity_iio_info;
	mz_proximity->indio_dev_wk->channels = (struct iio_chan_spec const	*)&mz_proximity_iio_ch;
	mz_proximity->indio_dev_wk->num_channels = ARRAY_SIZE(mz_proximity_iio_ch);
	mz_proximity->indio_dev_wk->modes = INDIO_DIRECT_MODE;
	iio_device_set_drvdata(mz_proximity->indio_dev_wk, mz_proximity);

	ret = iio_triggered_buffer_setup(mz_proximity->indio_dev_wk,
				&mz_proximity_iio_handler_empty, NULL,
				&mz_proximity_iio_buffer_setup_ops);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to setup triggered buffer\n");
		goto iio_device_free;
	}

	mz_proximity->trig_wk = iio_trigger_alloc("%s-trigger", "proximity_wk");
	if (!mz_proximity->trig_wk) {
		dev_err(mz_proximity->dev, "failed to call iio_trigger_alloc\n");
		goto iio_deallocate_buffer;
	}

	mz_proximity->trig_wk->ops = &mz_proximity_wk_trigger_ops;
	mz_proximity->trig_wk->dev.parent = mz_proximity->dev;
	iio_trigger_set_drvdata(mz_proximity->trig_wk, mz_proximity->indio_dev_wk);

	ret = iio_trigger_register(mz_proximity->trig_wk);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to register iio trigger.\n");
		goto iio_deallocate_trigger;
	}
	mz_proximity->indio_dev_wk->trig = mz_proximity->trig_wk;

	ret = iio_device_register(mz_proximity->indio_dev_wk);
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to register iio device.\n");
		goto iio_deallocate_trigger;
	}

#if 0
	ret = meizu_sysfslink_register_name(&mz_proximity->indio_dev_wk->dev, "proximity_wk");
	if (ret < 0) {
		dev_err(mz_proximity->dev, "failed to meizu_sysfslink_register_name.\n");
		goto iio_deallocate_trigger;
	}
#endif

	return 0;


iio_deallocate_trigger:
	iio_trigger_free(mz_proximity->trig_wk);
iio_deallocate_buffer:
	iio_triggered_buffer_cleanup(mz_proximity->indio_dev_wk);
iio_device_free:
	iio_device_free(mz_proximity->indio_dev_wk);
	return ret;
}


static int LTR559_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct LTR559_data *LTR559;

	pr_info("%s: enter probe ...\n", __func__);
	LTR559 = kzalloc(sizeof(struct LTR559_data), GFP_KERNEL);
	if (!LTR559)
	{
		dev_err(&LTR559->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = LTR559;

	/* Set initial defaults */
	LTR559->als_enable_flag = 0;
	LTR559->ps_enable_flag = 0;

	LTR559->i2c_client = client;
	LTR559->irq = client->irq;

	i2c_set_clientdata(client, LTR559);

	if (_check_part_id(LTR559) < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev,"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Create the workqueue for the interrup handler */
	LTR559->workqueue = create_singlethread_workqueue("LTR559_wq");
	if (!LTR559->workqueue) {
		dev_err(&LTR559->i2c_client->dev, "%s: Create WorkQueue Fail...\n", __func__);
		ret = -ENOMEM;
		goto err_out;
	}

	INIT_WORK(&LTR559->ps_work, LTR559_ps_schedwork);
	INIT_DELAYED_WORK(&LTR559->als_dwork, LTR559_als_schedwork);

	/* Wake lock option for promity sensor */
	wake_lock_init(&(LTR559->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	/* Setup and configure both the ALS and PS on the LTR559 device */
	ret = LTR559_setup(LTR559);
	if (ret < 0) {
		dev_err(&LTR559->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto err_LTR559_setup;
	}

	/* Setup the suspend and resume functionality */
	LTR559->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	LTR559->early_suspend.suspend = LTR559_early_suspend;
	LTR559->early_suspend.resume = LTR559_late_resume;
	register_early_suspend(&LTR559->early_suspend);

	/* Register the sysfs files */
	//sysfs_create_group(&client->dev.kobj, &LTR559_attribute_group);
	//sysfs_register_als_device(client, &LTR559->als_input_dev->dev);
	//sysfs_register_ps_device(client, &LTR559->ps_input_dev->dev);

	LTR559->mz_light.dev  = &client->dev;
	LTR559->mz_light.priv = LTR559;
	LTR559->mz_light.ops  = &ltr559_ligth_ops;
	mz_light_iio_init(&LTR559->mz_light);

	LTR559->mz_proximity.dev  = &client->dev;
	LTR559->mz_proximity.priv = LTR559;
	LTR559->mz_proximity.ops  = &ltr559_proximity_ops;
	mz_proximity_iio_init(&LTR559->mz_proximity);
	mz_proximity_wk_iio_init(&LTR559->mz_proximity);

	dev_dbg(&LTR559->i2c_client->dev, "%s: probe complete\n", __func__);

	als_set_enable(LTR559, 1);
	return ret;

err_LTR559_setup:
	destroy_workqueue(LTR559->workqueue);
err_out:
	kfree(LTR559);

	return ret;
}

static const struct i2c_device_id LTR559_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

static struct i2c_driver LTR559_driver = {
	.probe = LTR559_probe,
	.id_table = LTR559_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

static struct i2c_board_info i2c_ltr559[] = {
	{
		I2C_BOARD_INFO(DEVICE_NAME, 0x23)
	}
};

static int __init LTR559_init(void)
{
	pr_info("ltr559 init\n");
	i2c_register_board_info(2, i2c_ltr559, 1);
	return 0;
}

//module_init(LTR559_init)
//module_exit(LTR559_exit)

postcore_initcall(LTR559_init);
module_i2c_driver(LTR559_driver);

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-559ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

