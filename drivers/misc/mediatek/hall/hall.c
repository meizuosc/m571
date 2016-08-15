/*
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "keys_m7x.h"

static struct gpio_button_data *gpio_bdata[GPIO_MAX] = {NULL,};

struct gpio_button_data **get_gpio_bdata(void)
{
	return gpio_bdata;
}

struct gpio_button_data *get_gpio_sbdata(GPIO_INDEX which_gpio)
{
	return gpio_bdata[which_gpio];
}

void set_gpio_bdata(struct gpio_button_data *bdata, GPIO_INDEX which_gpio)
{
	struct gpio_button_data **gbdata = get_gpio_bdata();
	/*
	 * related to enum GPIO_INDEX
	 */
#ifdef KEYPAD_DEBUG
	printk("[Hall] %s: key:%s, gpio:%d\n", __func__, bdata->button->desc, which_gpio);
#endif
	switch( which_gpio ) {
	case GPIO_HALL:
		gbdata[which_gpio] = bdata;
		break;
	default:
		printk("[Hall] %s: index:%d is not within the scope of [0,%d]\n",
				__func__, which_gpio, GPIO_MAX);
		break;
	}
}

static void kpd_hall_irq_handler(void)
{
	struct gpio_button_data *bdata = get_gpio_sbdata(GPIO_HALL);

	if(!bdata) {
		mt_eint_unmask(M7X_HALL_IRQ_NUM);
		printk("[Hall] %s error: hall key irq is not init, bdata==NULL!\n",__func__);
	} else {
#ifdef DELAYED_WORK
		mt_eint_set_polarity(bdata->button->irq, atomic_read(&bdata->key_pressed)==CLOSED?LOW:HIGH);
		schedule_work(&bdata->work);
#else
		mod_timer(&bdata->timer,
					jiffies + msecs_to_jiffies(bdata->timer_debounce));
#endif
	}
}

/*
 * HALL:
 * Set EINT mode, input, enable pull up
 * set irq debounce
 * Register and enable interrupt(trigger by low level)
 */
void meizu_hall_init(void)
{
	/* set hall EINT mode, input, enable pull up */
	mt_set_gpio_mode(M7X_HALL_KEY_GPIO, M7X_HALL_KEY_PIN_EINT_MODE);
	mt_set_gpio_dir(M7X_HALL_KEY_GPIO, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(M7X_HALL_KEY_GPIO, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(M7X_HALL_KEY_GPIO, GPIO_PULL_UP);

	/* set hall irq debounce */
	mt_eint_set_sens(M7X_HALL_IRQ_NUM, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(M7X_HALL_IRQ_NUM, M7X_HALL_IRQ_DEBOUNCE_CN);

	/* register hall irq */
	mt_eint_registration(M7X_HALL_IRQ_NUM, M7X_HALL_IRQ_TYPE, kpd_hall_irq_handler, 0);
	mt_eint_unmask(M7X_HALL_IRQ_NUM);
#ifdef KEYPAD_DEBUG
	printk("[Hall] %s\n", __func__);
#endif
}
