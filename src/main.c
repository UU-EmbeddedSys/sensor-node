#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"


#define MY_STACK_SIZE 5000
#define MY_PRIORITY 5

#define REFRESH_TIME 1000

LOG_MODULE_REGISTER(main);


extern void my_entry_point(void *, void *, void *);

K_THREAD_STACK_DEFINE(my_stack_area, MY_STACK_SIZE);
struct k_thread polling_thread, i2c_thread;





void sensor_polling_fake(void* p1, void* p2, void* p3){
	while (true) {
		printk("Hello World from the app!\n");

		printk("JELLOW, MAI NEIM IS RODRIGO, AND AI AM BERI STIUPID :D\n");

		k_sleep(K_MSEC(1000));
	}
}


void sensor_polling(void* p1, void* p2, void* p3){
	bme680_dev_t bme680_device;
	bme680_dev_t_init(&bme680_device);
    bme680_config_init(&bme680_device);
	
	
	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		bme680_read_temperature(&bme680_device);
		LOG_INF("I'm doing something");
	}


}


void i2c_communication(void *p1, void *p2, void *p3){
	
	while(true){
		printf("I2C loop \n");
		k_sleep(K_MSEC(1000));
	}
}

void main(void)
{

	sensor_polling(NULL, NULL, NULL);
	k_tid_t polling_thread_tid = k_thread_create(&polling_thread, my_stack_area,
                                 K_THREAD_STACK_SIZEOF(my_stack_area),
                                 sensor_polling,
                                 NULL, NULL, NULL,
                                 MY_PRIORITY, 0, K_FOREVER);
	
	k_tid_t i2c_thread_tid = k_thread_create(&i2c_thread, my_stack_area,
                                 K_THREAD_STACK_SIZEOF(my_stack_area),
                                 i2c_communication,
                                 NULL, NULL, NULL,
                                 MY_PRIORITY, 0, K_FOREVER);

	k_thread_cpu_pin( polling_thread_tid, 0 );
	k_thread_cpu_pin( i2c_thread_tid    , 1 );

	k_thread_start( polling_thread_tid );
	k_thread_start( i2c_thread_tid );

	
}
