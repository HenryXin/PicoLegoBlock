/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>

#define SW0_NODE	DT_ALIAS(sw0)

static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
static const struct pwm_dt_spec pwm_led3 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led3));
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});


#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U)

static struct gpio_callback button_cb_data;

uint8_t led_selected = 0;

void button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
	led_selected++;
	if(led_selected>2) {
		led_selected = 0;
	}
}


int main(void)
{
	uint32_t max_period;
	uint32_t period;
	uint8_t dir = 0U;
	int ret;

	printk("PWM-based blinky\n");

	if (!pwm_is_ready_dt(&pwm_led1)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led1.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&pwm_led2)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led2.dev->name);
		return 0;
	}
	if (!pwm_is_ready_dt(&pwm_led3)) {
		printk("Error: PWM device %s is not ready\n",
		       pwm_led3.dev->name);
		return 0;
	}

	
	if (!gpio_is_ready_dt(&button)) {
		printk("Error: button device %s is not ready\n",
		       button.port->name);
		return 0;
	}
	
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
	
	/*
	 * In case the default MAX_PERIOD value cannot be set for
	 * some PWM hardware, decrease its value until it can.
	 *
	 * Keep its value at least MIN_PERIOD * 4 to make sure
	 * the sample changes frequency at least once.
	 */
	printk("Calibrating for channel %d...\n", pwm_led1.channel);
	max_period = MAX_PERIOD;
	while (pwm_set_dt(&pwm_led1, max_period, max_period / 2U)) {
		max_period /= 2U;
		if (max_period < (4U * MIN_PERIOD)) {
			printk("Error: PWM device "
			       "does not support a period at least %lu\n",
			       4U * MIN_PERIOD);
			return 0;
		}
	}

	printk("Done calibrating; maximum/minimum periods %u/%lu nsec\n",
	       max_period, MIN_PERIOD);

	period = max_period;
	while (1) {
		if(led_selected == 0) {
			ret = pwm_set_dt(&pwm_led1, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led2, period, 0);
			ret = pwm_set_dt(&pwm_led3, period, 0);
			printk("Using period %d\n", period);
		} 
		else if(led_selected == 1) 
		{
			ret = pwm_set_dt(&pwm_led2, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led1, period, 0);
			ret = pwm_set_dt(&pwm_led3, period, 0);
		}
		else {
			ret = pwm_set_dt(&pwm_led3, period, period / 2U);
			if (ret) {
				printk("Error %d: failed to set pulse width\n", ret);
				return 0;
			}
			ret = pwm_set_dt(&pwm_led1, period, 0);
			ret = pwm_set_dt(&pwm_led2, period, 0);
		}
		period = dir ? (period * 2U) : (period / 2U);
		if (period > max_period) {
			period = max_period / 2U;
			dir = 0U;
		} else if (period < MIN_PERIOD) {
			period = MIN_PERIOD * 2U;
			dir = 1U;
		}

		int val = gpio_pin_get_dt(&button);

		if (val >= 0) {
			//gpio_pin_set_dt(&led, val);
			printk("Button value: %d\n", val);
		}
		k_sleep(K_SECONDS(4U));
	}
	return 0;
}
