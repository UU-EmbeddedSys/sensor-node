#ifndef ADXL345_H
#define ADXL345_H

#include "adxl345_registers.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <stdint.h>

#define ADXL345_ADDR 0x53

#define SENSITIVITY   32	  // LSB/g
#define SCALE_FACTOR  31.2 / 1000 // mg/LSB
#define EARTH_GRAVITY 9.81f

#define ADXL345_COMPLEMENT 0xfc00

typedef enum FIFO_MODE {
	BYPASS = (0 << 7) | (0 << 6),
	FIFO = (0 << 7) | (1 << 6),
	STREAM = (1 << 7) | (0 << 6),
	TRIGGER = (1 << 7) | (1 << 6)
} FIFO_MODE;

typedef enum RANGE {
	R_2G = (0 << 1) | (0 << 0),
	R_4G = (0 << 1) | (1 << 0),
	R_8G = (1 << 1) | (0 << 0),
	R_16G = (1 << 1) | (1 << 0)
} RANGE;

typedef enum FREQUENCY {
	HZ_3200 = 0b1111,
	HZ_1600 = 0b1110,
	HZ_800 = 0b1101,
	HZ_400 = 0b1100,
	HZ_200 = 0b1011,
	HZ_100 = 0b1010,
	HZ_50 = 0b1001,
	HZ_25 = 0b1000,
	HZ_12_5 = 0b0111,
	HZ_6_25 = 0b0110
} FREQUENCY;

typedef struct adxl345_manager_t {
	const struct device *i2c_dev;
	float x_acceleration;
	float y_acceleration;
	float z_acceleration;

	float acceleration_threshold;
	struct k_sem adxl_sem;
} adxl345_manager_t;

void adxl345_constructor(adxl345_manager_t *adxl345_device);

void adxl345_read_xyz_axis(adxl345_manager_t *adxl345_device);

void adxl345_chip_id(adxl345_manager_t *bme680_device);

void adxl345_set_measurement_mode(adxl345_manager_t *adxl345_device);
void adxl345_set_fifo_mode(adxl345_manager_t *adxl345_device, FIFO_MODE mode);
void adxl345_constructor(adxl345_manager_t *adxl345_device);
void adxl345_set_frequency(adxl345_manager_t *adxl345_device, FREQUENCY freq);
void adxl345_set_range(adxl345_manager_t *adxl345_device, RANGE range);

#endif /* ADXL345_H */
