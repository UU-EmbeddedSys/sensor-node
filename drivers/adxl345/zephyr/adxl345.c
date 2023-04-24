#include "adxl345.h"

void adxl345_constructor(adxl345_manager_t *adxl345_device)
{
	adxl345_device->i2c_dev = device_get_binding("I2C_0");
}

void adxl345_read_x_axis(adxl345_manager_t *adxl345_device)
{
}