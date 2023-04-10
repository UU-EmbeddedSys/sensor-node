#ifndef BME680_H
#define BME680_H

#include "bme680_registers.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>


#include <stdint.h>

#define BME680_ADDR 0x77
// FIXME
enum Oversampling {
    NO_OVERSAMPLING = 1,
    OVERSAMPLING_2X = 2,
    OVERSAMPLING_4X = 4,
    OVERSAMPLING_8X = 8,
    OVERSAMPLING_16X = 16,
};

typedef struct bme680_manager_t{
	const struct device *i2c_dev;
	uint8_t temp_oversampling;
	uint8_t forced_mode;


	float last_temperature;
	float last_humidity;
	float last_pressure;
} bme680_manager_t;


typedef struct bme680_temp_calib_data_t {
	/*! Variable to store calibrated temperature data */
	uint16_t par_t1;
	/*! Variable to store calibrated temperature data */
	int16_t par_t2;
	/*! Variable to store calibrated temperature data */
	int16_t par_t3;
	/*! Variable to store t_fine size */
	int32_t t_fine;
} bme680_temp_calib_data_t;

typedef struct bme680_pressure_calib_data_t {
	uint16_t par_p1;
	int16_t par_p2;
	int16_t par_p3;
	int16_t par_p4;
	int16_t par_p5;
	int16_t par_p6;
	int16_t par_p7;
	int16_t par_p8;
	int16_t par_p9;
	uint8_t par_p10;
	int32_t press_comp;
} bme680_pressure_calib_data_t;

typedef struct bme680_humidity_calib_data_t {
	uint16_t par_h1;
	int16_t par_h2;
	uint8_t par_h3;
	int8_t par_h4;
	int8_t par_h5;
	uint8_t par_h6;
	int8_t par_h7;
	int32_t hum_comp;
} bme680_humidity_calib_data_t;

typedef struct bme680_gas_calib_data_t {
	uint8_t par_g1;
	uint16_t par_g2;
	uint8_t par_g3;
	uint8_t res_heat_range;
	uint8_t res_heat_val;
    uint8_t res_heat; // store the resistance
	uint8_t gas_range;
	uint8_t range_switching_error;
    int32_t gas_res; // compensated gas sensor resistance output data in Ohms.
} bme680_gas_calib_data_t;

typedef struct bme680_calib_data_t {
	bme680_temp_calib_data_t temp;
	bme680_pressure_calib_data_t press;
	bme680_humidity_calib_data_t hum;
    bme680_gas_calib_data_t gas;
} bme680_calib_data_t;


void bme680_constructor(bme680_manager_t* bme680_device);

void bme680_soft_reset(bme680_manager_t* bme680_device);

void bme680_chip_id(bme680_manager_t* bme680_device);

// TODO modify for getting ALL the parameters
bme680_temp_calib_data_t bme680_calib_data(bme680_manager_t* bme680_device);

void bme680_config_init(bme680_manager_t* bme680_device);

void bme680_read_temperature(bme680_manager_t* bme680_device);

#endif /* BME680_H */
