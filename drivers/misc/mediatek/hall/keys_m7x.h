#ifndef __KEYS_M7X_H__
#define __KEYS_M7X_H__

#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <mach/mt_reg_base.h>
#include <mach/eint.h>
#include <mach/irqs.h>
#include <mach/sync_write.h>

/*kpd.h file path: mediatek/kernel/include/linux */
#include <linux/kpd.h>
#include "cust_gpio_usage.h"
#include "cust_kpd.h"
#include "cust_eint.h"

/*
 * DELAYED_WORK: use delay work
 * KEYPAD_DEBUG: contrl log print
 * KEY_HALL_DEBUG: contrl sysfs show key_hall_press
 */
#define DELAYED_WORK
#define KEYPAD_DEBUG
#define KEY_HALL_DEBUG


/*
 * Redefine hall pin
 * According to DCT config, modify them.
 */
/* Pin */
#define M7X_HALL_KEY_GPIO	GPIO_HALL_SWITH_EINT_PIN
/* EINT mode */
#define M7X_HALL_KEY_PIN_EINT_MODE	GPIO_HALL_SWITH_EINT_PIN_M_EINT
/* IRQ num */
#define M7X_HALL_IRQ_NUM	CUST_EINT_HALL_INT_NUM
/* IRQ type */
#define M7X_HALL_IRQ_TYPE	CUST_EINT_HALL_INT_TYPE
/* IRQ debounce  */
#define M7X_HALL_IRQ_DEBOUNCE_CN	CUST_EINT_HALL_INT_DEBOUNCE_CN


#define RELEASED (0)
#define PRESSED  (1)
#define CLOSED   (2)
#define REMOVED  (3)

#define FALLING MT_EINT_POL_NEG
#define RISING  MT_EINT_POL_POS
#define LOW     MT_EINT_POL_NEG
#define HIGH    MT_EINT_POL_POS

typedef enum {
	GPIO_HALL = 0,
	GPIO_MAX
} GPIO_INDEX;

struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
#ifdef DELAYED_WORK
	struct delayed_work delayed_work;
#endif
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	struct mutex key_mutex;
	bool disabled;
	atomic_t key_pressed;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};

extern unsigned int mt_eint_get_polarity(unsigned int eint_num);
void set_gpio_bdata(struct gpio_button_data *bdata, GPIO_INDEX which_gpio);
void meizu_hall_init(void);
void m7x_gpio_keys_init(void);

#endif /* __KEYS_M7X_H__ */
