/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

int main(void)
{
	while(1) {
		printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
