#include "sample.h"
#include "i2c_slave.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define REFRESH_TIME 500

LOG_MODULE_REGISTER(main);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_polling_stack, MY_STACK_SIZE);

struct k_thread polling_thread, i2c_thread;

struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

sensor_tree_t sensor_tree;

void print_sensors_value()
{
	LOG_INF("T: %f P: %f H: %f", sensor_tree.bme680_device.last_temperature, sensor_tree.bme680_device.last_pressure, sensor_tree.bme680_device.last_humidity);
	LOG_INF("X: %f Y: %f Z: %f ", sensor_tree.adxl345_device.x_acceleration, sensor_tree.adxl345_device.y_acceleration, sensor_tree.adxl345_device.z_acceleration);
	LOG_INF("Distance: %f", sensor_tree.ultrasonic_device.distance);
}

void sensor_polling(void *p1, void *p2, void *p3)
{
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	adxl345_constructor(&(sensor_tree.adxl345_device));
	adxl345_set_fifo_mode(&(sensor_tree.adxl345_device), STREAM);
	adxl345_set_range(&(sensor_tree.adxl345_device), R_16G);
	adxl345_set_frequency(&(sensor_tree.adxl345_device), HZ_25);
	adxl345_set_measurement_mode(&(sensor_tree.adxl345_device));
	adxl345_chip_id(&(sensor_tree.adxl345_device));

	bme680_constructor(&(sensor_tree.bme680_device));
	ultrasonic_init(&(sensor_tree.ultrasonic_device));
	while (true) {
		print_sensors_value();
		gpio_pin_toggle_dt(&led0);
		k_sleep(K_MSEC(REFRESH_TIME));

		bme680_read_sensors(&(sensor_tree.bme680_device));
		adxl345_read_xyz_axis(&(sensor_tree.adxl345_device));
		ultrasonic_duration(&(sensor_tree.ultrasonic_device));

		gpio_pin_toggle_dt(&led0);
	}
}

void i2c_communication_target(void *p1, void *p2, void *p3)
{
	int err = 0;
	err = slave_init(&sensor_tree);
	if (err != 0) {
		LOG_ERR("Failed initilize slave %d", err);
		return;
	}
	while (true) { // busy waiting thread
		k_sleep(K_MSEC(REFRESH_TIME));
	}
}

void main(void)
{
	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_target,
		NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);
	k_tid_t polling_thread_tid = k_thread_create(
		&polling_thread, sensor_polling_stack, K_THREAD_STACK_SIZEOF(sensor_polling_stack),
		sensor_polling, NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_cpu_pin((k_tid_t)&polling_thread_tid, 0);
	k_thread_cpu_pin((k_tid_t)&i2c_thread_tid, 1);

	k_thread_start(i2c_thread_tid);
	k_thread_start(polling_thread_tid);
}