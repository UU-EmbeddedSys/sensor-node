#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define REFRESH_TIME 400

LOG_MODULE_REGISTER(main);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_polling_stack, MY_STACK_SIZE);

struct k_thread polling_thread, i2c_thread;

struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

typedef struct sensor_tree_t {
	bme680_manager_t bme680_device;
	adxl345_manager_t adxl345_device;
} sensor_tree_t;

sensor_tree_t sensor_tree;

void sensor_polling(void *p1, void *p2, void *p3)
{
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	bme680_constructor(&(sensor_tree.bme680_device));

	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		bme680_read_sensors(&(sensor_tree.bme680_device));

		gpio_pin_toggle_dt(&led0);
	}
}

void i2c_communication(void *p1, void *p2, void *p3)
{
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

	LOG_INF("I2C thread started\n");
	while (true) {
		printf("Temperature: %f\n",
			sensor_tree.bme680_device.last_temperature);
		printf("Pressure: %f\n",
			sensor_tree.bme680_device.last_pressure);
		printf("Humidity: %f\n",
			sensor_tree.bme680_device.last_humidity);
		gpio_pin_toggle_dt(&led1);
		k_sleep(K_MSEC(1000));
	}
}

void main(void)
{
	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication, NULL,
		NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);
	k_tid_t polling_thread_tid = k_thread_create(
		&polling_thread, sensor_polling_stack, K_THREAD_STACK_SIZEOF(sensor_polling_stack),
		sensor_polling, NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_cpu_pin(&polling_thread_tid, 0);
	k_thread_cpu_pin(&i2c_thread_tid, 1);

	k_thread_start(i2c_thread_tid);
	k_thread_start(polling_thread_tid);
}