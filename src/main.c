#include <stdio.h>
#include <zephyr/kernel.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"


#define MY_STACK_SIZE 500
#define MY_PRIORITY 5

#define REFRESH_TIME 1000


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
	 
	const struct device* i2c_dev = init_bme680();
	
	
	
	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		uint8_t reg_ctrl_meas = 0b01001001; 
		poll_bme680(i2c_dev, reg_ctrl_meas);
	}


}


void i2c_communication(void *p1, void *p2, void *p3){
	
	printf("TEST");
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
                                 sensor_polling_fake,
                                 NULL, NULL, NULL,
                                 MY_PRIORITY, 0, K_FOREVER);

	k_thread_cpu_pin( polling_thread_tid, 0 );
	k_thread_cpu_pin( i2c_thread_tid    , 1 );

	k_thread_start( polling_thread_tid );
	k_thread_start( i2c_thread_tid );
	while(true){
		printf("TEST");
	}

	
}
