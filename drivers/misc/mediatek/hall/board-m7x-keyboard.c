/*
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/i2c.h>

#include "keys_m7x.h"

/*
 * Only hall key here
 */
static struct gpio_keys_button M7x_gpio_keys_tables[] = {
	{
		.code			= SW_LID, // 0x15, //KEY_HALL_CLOSED, //KEY_HALL_REMOVED, //KEY_POWER,
		.desc			= "gpio-keys: KEY_HALL",
		.type			= EV_SW,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 500,
		.gpio           	= M7X_HALL_KEY_GPIO,
		.irq            	= M7X_HALL_IRQ_NUM,
	},
};

static struct gpio_keys_platform_data M7x_gpio_keys_data = {
	.buttons		= M7x_gpio_keys_tables,
	.nbuttons		= ARRAY_SIZE(M7x_gpio_keys_tables),
};

static struct platform_device m7x_gpio_keys = {
	.name			= "gpio-keys",
	.dev			= {
		.platform_data	= &M7x_gpio_keys_data,
	},
};

void m7x_gpio_keys_init(void)
{
	platform_device_register(&m7x_gpio_keys);
}
