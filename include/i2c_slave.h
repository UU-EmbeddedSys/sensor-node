#ifndef I2C_SLAVE_H
#define I2C_SLAVE_H

#include "i2c_registers.h"
#include "bme680.h"
#include "adxl345.h"
#include "ultrasonic.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <stdint.h>

#define SENSOR_NODE_ADDR 0x40

typedef struct i2c_slave_manager_t
{
	bool first_write;
	bool read_started;
	uint8_t start_address;
	uint8_t remaining_bytes;
	uint8_t* buffer;
} i2c_slave_manager_t;

typedef struct sensor_tree_t {
	bme680_manager_t bme680_device;
	adxl345_manager_t adxl345_device;
	ultrasonic_manager_t ultrasonic_device;
} sensor_tree_t;

int slave_init(sensor_tree_t *sensor_tree);

#endif
