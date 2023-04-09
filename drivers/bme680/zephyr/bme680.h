#ifndef BME680_H
#define BME680_H

#include "bme680_registers.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <stdint.h>

#define BME680_ADDR 0x77

typedef struct bme680_calib_data_t {
	/*! Variable to store calibrated temperature data */
	uint16_t par_t1;
	/*! Variable to store calibrated temperature data */
	int16_t par_t2;
	/*! Variable to store calibrated temperature data */
	int16_t par_t3;
	/*! Variable to store t_fine size */
	int32_t t_fine;
} bme680_calib_data_t;

void bme680_soft_reset(const struct device *i2c_dev);

void bme680_chip_id(const struct device *i2c_dev);

bme680_calib_data_t bme680_calib_data(const struct device *i2c_dev);

void bme680_config_init(const struct device *i2c_dev);

void bme680_read_temperature(const struct device *i2c_dev);

#endif /* BME680_H */
