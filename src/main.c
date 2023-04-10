#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"
#include <zephyr/log_ctrl.h>


#define MY_STACK_SIZE 5000
#define MY_PRIORITY 5

#define REFRESH_TIME 1000

LOG_MODULE_REGISTER(main);



K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_polling_stack, MY_STACK_SIZE);

struct k_thread polling_thread, i2c_thread;

typedef struct sensor_tree_t{
	bme680_manager_t bme680_device;
	adxl345_manager_t adxl345_device;
} sensor_tree_t;

sensor_tree_t sensor_tree;


void sensor_polling_fake(void* p1, void* p2, void* p3){
	while (true) {
		LOG_INF("Hello World from the app!\n");

		LOG_INF("JELLOW, MAI NEIM IS RODRIGO, AND AI AM BERI STIUPID :D\n");

		k_sleep(K_MSEC(1000));
	}
}


void sensor_polling(void* p1, void* p2, void* p3){
	log_thread_set(k_current_get());
	bme680_constructor(&(sensor_tree.bme680_device));
	
	
	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		bme680_read_temperature(&(sensor_tree.bme680_device));
		LOG_INF("I'm doing something\n");
	}


}


void i2c_communication(void *p1, void *p2, void *p3){
	log_thread_set(k_current_get());
	LOG_INF("I2C thread started\n");
	while(true){
		LOG_INF("Temperature: %f\n", sensor_tree.bme680_device.last_temperature); //TODO add mutex
		k_sleep(K_MSEC(1000));
	}
}

void main(void)
{
	k_tid_t i2c_thread_tid = k_thread_create(&i2c_thread, i2c_stack,
                                 K_THREAD_STACK_SIZEOF(i2c_stack),
                                 i2c_communication,
                                 NULL, NULL, NULL,
                                 4, 0, K_FOREVER);
	k_tid_t polling_thread_tid = k_thread_create(&polling_thread, sensor_polling_stack,
                                 K_THREAD_STACK_SIZEOF(sensor_polling_stack),
                                 sensor_polling,
                                 NULL, NULL, NULL,
                                 MY_PRIORITY, 0, K_FOREVER);
	

	k_thread_cpu_pin( polling_thread_tid, 0 );
	k_thread_cpu_pin( i2c_thread_tid    , 1 );

	k_thread_start( i2c_thread_tid );
	k_thread_start( polling_thread_tid );

	
}
