#ifndef ADXL345_H
#define ADXL345_H

#include "adxl345_registers.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <stdint.h>

#define ADXL345_ADDR 0x53


typedef struct adxl345_manager_t{
	const struct device *i2c_dev;
	
} adxl345_manager_t;


void adxl345_constructor(adxl345_manager_t* adxl345_device);

void adxl345_read_x_axis(adxl345_manager_t* adxl345_device);

void adxl345_chip_id(adxl345_manager_t* bme680_device);


#endif /* ADXL345_H */
