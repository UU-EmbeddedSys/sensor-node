#include <stdio.h>
#include <zephyr/kernel.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"

void main(void)
{
	while (true) {
		printk("Hello World from the app!\n");

		printk("%d\n", test_bme680());

		printk("%d\n", test_adxl());

		greet();

		k_sleep(K_MSEC(1000));
	}
}
