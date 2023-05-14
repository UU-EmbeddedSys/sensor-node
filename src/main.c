#include "sample.h"
#include "i2c_slave.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <stdlib.h>

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

sensor_tree_t sensor_tree = {
	.int_src = NONE
};

void print_sensors_value()
{
	LOG_INF("T: %f P: %f H: %f", sensor_tree.bme680_device.last_temperature,
		sensor_tree.bme680_device.last_pressure, sensor_tree.bme680_device.last_humidity);
	LOG_INF("X: %f Y: %f Z: %f ", sensor_tree.adxl345_device.x_acceleration,
		sensor_tree.adxl345_device.y_acceleration,
		sensor_tree.adxl345_device.z_acceleration);
	LOG_INF("Distance: %f", sensor_tree.ultrasonic_device.distance);
}

void sensor_polling(void *p1, void *p2, void *p3)
{
	bme680_manager_t *bme680 = &sensor_tree.bme680_device;
	adxl345_manager_t *adxl345 = &sensor_tree.adxl345_device;
	ultrasonic_manager_t *ultr = &sensor_tree.ultrasonic_device;
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	adxl345_constructor(adxl345);
	adxl345_set_fifo_mode(adxl345, STREAM);
	adxl345_set_range(adxl345, R_16G);
	adxl345_set_frequency(adxl345, HZ_25);
	adxl345_set_measurement_mode(adxl345);
	adxl345_chip_id(adxl345);

	bme680_constructor(bme680);
	ultrasonic_init(ultr);
	while (true) {
		print_sensors_value();
		gpio_pin_toggle_dt(&led0);
		k_sleep(K_MSEC(REFRESH_TIME));

		bme680_read_sensors(bme680);
		adxl345_read_xyz_axis(adxl345);
		ultrasonic_duration(ultr);
		// check the values to trigger an interrupt on the driver
		if (sensor_tree.int_src == NONE) {
			if (bme680->last_temperature > bme680->temp_thresh) {
				k_sem_give(&(bme680->bme_sem));
			}
			if (bme680->last_humidity > bme680->hum_thresh) {
				k_sem_give(&(bme680->bme_sem));
			}
			if (bme680->last_pressure > bme680->press_thresh) {
				k_sem_give(&(bme680->bme_sem));
			}
			if (ultr->distance < ultr->trigger_threshold) {
				k_sem_give(&(ultr->us_sem));
			}
			if (abs(adxl345->x_acceleration) > adxl345->acceleration_threshold ||
			    abs(adxl345->y_acceleration) > adxl345->acceleration_threshold ||
			    abs(adxl345->z_acceleration) > adxl345->acceleration_threshold) {
				k_sem_give(&(adxl345->adxl_sem));
			}
		}
		gpio_pin_toggle_dt(&led0);
	}
}

void i2c_communication_target(void *p1, void *p2, void *p3)
{
	bme680_manager_t *bme680 = &sensor_tree.bme680_device;
	adxl345_manager_t *adxl345 = &sensor_tree.adxl345_device;
	ultrasonic_manager_t *ultr = &sensor_tree.ultrasonic_device;
	int err = 0;
	err = slave_init(&sensor_tree);
	if (err != 0) {
		LOG_ERR("Failed initilize slave %d", err);
		return;
	}
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE); // INTERRUPT PIN
	while (true) {
		if (k_sem_take(&(bme680->bme_sem), K_NO_WAIT) == 0) {
			// Interrupt from BME
			if (bme680->last_temperature > bme680->temp_thresh) {
				LOG_WRN("Temp above threshold {%f} [%f]", bme680->last_temperature, bme680->temp_thresh);
				sensor_tree.int_src = TEMPERATURE;
			} else if (bme680->last_humidity > bme680->hum_thresh) {
				LOG_WRN("Hum above threshold {%f} [%f]", bme680->last_humidity, bme680->hum_thresh);
				sensor_tree.int_src = HUMIDITY;
			} else if (bme680->last_pressure > bme680->press_thresh) {
				LOG_WRN("Press above threshold {%f} [%f]", bme680->last_pressure, bme680->press_thresh);
				sensor_tree.int_src = PRESSURE;
			}
			gpio_pin_set_dt(&led1, 1);
			continue;
		}
		if (k_sem_take(&(adxl345->adxl_sem), K_NO_WAIT) == 0) {
			LOG_WRN("ADXL above threshold");
			if (abs(adxl345->x_acceleration) > adxl345->acceleration_threshold ||
			    abs(adxl345->y_acceleration) > adxl345->acceleration_threshold ||
			    abs(adxl345->z_acceleration) > adxl345->acceleration_threshold) {
				sensor_tree.int_src = ACCELERATION;
			}
			gpio_pin_set_dt(&led1, 1);
			continue;
		}
		if (k_sem_take(&(ultr->us_sem), K_NO_WAIT) == 0){
			LOG_WRN("DIST above threshold");
			if (ultr->distance < ultr->trigger_threshold) {
				sensor_tree.int_src = DISTANCE;
			}
			gpio_pin_set_dt(&led1, 1);
			continue;
		}
		k_sleep(K_MSEC(REFRESH_TIME));
		gpio_pin_set_dt(&led1, 0);
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