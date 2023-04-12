#include "adxl345.h"

LOG_MODULE_REGISTER(adxl345);

static int adxl345_read_reg(const struct device *i2c_dev, uint8_t *read_buf, uint8_t num_bytes,
			       uint8_t start_address)
{
	int err = 0;
	struct i2c_msg msg[2];
	// Write on the I2C bus the internal address of the sensor we want to read to
	msg[0].buf = &start_address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;
	err = i2c_transfer(i2c_dev, msg, 2, ADXL345_ADDR);


	//int ret = i2c_reg_read_byte(i2c_dev, ADXL345_ADDR, start_address, read_buf);


	return err;
}

/**
 * @brief Write an internal register of the adxl345 sensor
 *
 * @param i2c_dev
 * @param write_buf
 * @param num_bytes
 * @param start_address
 * @return int
 */
static int adxl345_write_reg(const struct device *i2c_dev, uint8_t *write_buf, uint8_t num_bytes,
				uint8_t start_address)
{
	int err = 0;
	struct i2c_msg msg[2];
	// Write on the I2C bus the internal address of the sensor we want to read to
	msg[0].buf = &start_address;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE;
	// Read the data from the bus
	msg[1].buf = (uint8_t *)write_buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	err = i2c_transfer(i2c_dev, msg, 2, ADXL345_ADDR);
	return err;
}

void adxl345_constructor(adxl345_manager_t* adxl345_device){
	if (adxl345_device == NULL) {
		LOG_INF("Failed to allocate memory for adxl345_device\n");
		return;
	}
	/* Initialize I2C device */
    adxl345_device->i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!(adxl345_device->i2c_dev)) {
        LOG_INF("Failed to get I2C device\n");
        return;
    }
    else{
        LOG_INF("I2C device found\n");
    }
	

	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int err = i2c_configure(adxl345_device->i2c_dev, i2c_cfg);
	if(err != 0)
	{
		LOG_ERR("i2c_configure\n");
	}
	LOG_INF("I2C device configured\n");
	

	uint8_t data =  (1 << 3);

	//ACTIVATE MEASURING BIT OF POWER_CTL
	adxl345_write_reg(adxl345_device->i2c_dev, &data, 1, 0x2D);
	LOG_INF("Measuring mode activated\n");
	

	
}


/**
 * @brief Read the chip ID to verify the communication with the device
 *
 * @param i2c_dev
 */
void adxl345_chip_id(adxl345_manager_t* adxl345_device)
{
	int err = 0;
	uint8_t chip_id = 0;
	err = adxl345_read_reg(adxl345_device->i2c_dev, &chip_id, 1, ADXL345_DEVID);
	if (err != 0) {
		LOG_ERR("adxl345_chip_id\n");
	}
	LOG_INF("Chip ID 0x%02x\n", chip_id);
	if (chip_id != 0xE5) // reset state of a read only register
	{
		LOG_ERR("Wrong chip ID");
	}
}


void adxl345_read_x_axis(adxl345_manager_t* adxl345_device){
	int err = 0;
	uint8_t axis_data[6];
	err = adxl345_read_reg(adxl345_device->i2c_dev, axis_data, sizeof(axis_data), ADXL345_DATAX0);
	if(err != 0)
	{
		LOG_ERR("error reading axis");
	}
	uint16_t x, y, z;
	x = axis_data[1] << 8 | axis_data[0];
	y = axis_data[3] << 8 | axis_data[2];
	z = axis_data[5] << 8 | axis_data[3];

	LOG_INF("[RAW] X: %u Y: %u Z: %u\n", x, y, z);
}