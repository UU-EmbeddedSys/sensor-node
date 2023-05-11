#include "adxl345.h"

LOG_MODULE_REGISTER(adxl345, LOG_LEVEL_INF);

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

void adxl345_constructor(adxl345_manager_t *adxl345_device)
{
	if (adxl345_device == NULL) {
		LOG_INF("Failed to allocate memory for adxl345_device\n");
		return;
	}
	/* Initialize I2C device */
	adxl345_device->i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	if (!(adxl345_device->i2c_dev)) {
		LOG_INF("Failed to get I2C device\n");
		return;
	} else {
		LOG_INF("I2C device found\n");
	}

	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int err = i2c_configure(adxl345_device->i2c_dev, i2c_cfg);
	if (err != 0) {
		LOG_ERR("i2c_configure\n");
	}
	LOG_INF("I2C device configured\n");
}

/**
 * @brief any register can be written to or read from to configure the part while
 * the device is in standby mode.
 *
 * @param adxl345_device
 */
void adxl345_set_measurement_mode(adxl345_manager_t *adxl345_device)
{
	int err = 0;
	uint8_t data = (1 << POWER_CTL_MEASURE);

	// ACTIVATE MEASURING BIT OF POWER_CTL
	err = adxl345_write_reg(adxl345_device->i2c_dev, &data, sizeof(data), ADXL345_POWER_CTL);
	if (err != 0) {
		LOG_ERR("err set measurement");
	}
	LOG_INF("Measuring mode activated\n");
}

void adxl345_set_standby_mode(adxl345_manager_t *adxl345_device)
{
	int err = 0;
	uint8_t data = (0 << POWER_CTL_MEASURE);

	// ACTIVATE MEASURING BIT OF POWER_CTL
	err = adxl345_write_reg(adxl345_device->i2c_dev, &data, sizeof(data), ADXL345_POWER_CTL);
	if (err != 0) {
		LOG_ERR("err set measurement");
	}
	LOG_INF("Measuring mode activated\n");
}

void adxl345_set_range(adxl345_manager_t *adxl345_device, RANGE range)
{
	int err = 0;
	uint8_t data = range;

	// ACTIVATE MEASURING BIT OF POWER_CTL
	err = adxl345_write_reg(adxl345_device->i2c_dev, &data, sizeof(data), ADXL345_DATA_FORMAT);
	if (err != 0) {
		LOG_ERR("err set resolution");
	}
	LOG_INF("+-16g resolution set");
}

/**
 * @brief Read the chip ID to verify the communication with the device
 *
 * @param i2c_dev
 */
void adxl345_chip_id(adxl345_manager_t *adxl345_device)
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

static void adxl345_read_xyz_offset(adxl345_manager_t *adxl345_device)
{
	int err = 0;
	uint8_t data[8];
	err = adxl345_read_reg(adxl345_device->i2c_dev, data, sizeof(data), ADXL345_OFSX);
	if (err != 0) {
		LOG_ERR("offsert");
	}
	LOG_INF("x_off %02X y_off %02X z_off %02X\n", data[0], data[1], data[2]);
}

void adxl345_read_xyz_axis(adxl345_manager_t *adxl345_device)
{
	int err = 0;
	// adxl345_read_xyz_offset(adxl345_device);
	uint8_t axis_data[6];
	err = adxl345_read_reg(adxl345_device->i2c_dev, axis_data, sizeof(axis_data),
			       ADXL345_DATAX0);
	if (err != 0) {
		LOG_ERR("error reading axis");
	}
	int16_t x, y, z;

	x = (axis_data[1] << 8 | axis_data[0]);
	LOG_HEXDUMP_DBG(&x, sizeof(x), "x");
	y = (axis_data[3] << 8 | axis_data[2]);
	LOG_HEXDUMP_DBG(&y, sizeof(x), "y");
	z = (axis_data[5] << 8 | axis_data[4]);
	LOG_HEXDUMP_DBG(&z, sizeof(x), "z");

	if (x & BIT(9)) {
		x |= ADXL345_COMPLEMENT;
	}
	if (y & BIT(9)) {
		y |= ADXL345_COMPLEMENT;
	}
	if (z & BIT(9)) {
		z |= ADXL345_COMPLEMENT;
	}

	LOG_DBG("[RAW] X: %u Y: %u Z: %u", x, y, z);

	adxl345_device->x_acceleration = ((x)*EARTH_GRAVITY / SENSITIVITY);// / EARTH_GRAVITY;
	adxl345_device->y_acceleration = ((y)*EARTH_GRAVITY / SENSITIVITY);// / EARTH_GRAVITY;
	adxl345_device->z_acceleration = ((z)*EARTH_GRAVITY / SENSITIVITY);// / EARTH_GRAVITY;
}

void adxl345_read_active_thresh(adxl345_manager_t *adxl345_device)
{
	int err = 0;
	uint8_t read_buf[8] = {0};
	err = adxl345_read_reg(adxl345_device->i2c_dev, read_buf, sizeof(read_buf),
			       ADXL345_THRESH_ACT);
	if (err != 0) {
		LOG_ERR("%d", err);
	}
}

void adxl345_set_active_thresh(adxl345_manager_t *adxl345_device)
{
	// TODO
}

void adxl345_set_fifo_mode(adxl345_manager_t *adxl345_device, FIFO_MODE mode)
{
	int err = 0;
	uint8_t data = mode;
	err = adxl345_write_reg(adxl345_device->i2c_dev, &data, sizeof(data), ADXL345_FIFO_CTL);
	if (err != 0) {
		LOG_ERR("%d", err);
	}
}

void adxl345_set_frequency(adxl345_manager_t *adxl345_device, FREQUENCY freq)
{
	int err = 0;
	uint8_t data = freq;
	err = adxl345_write_reg(adxl345_device->i2c_dev, &data, sizeof(data), ADXL345_BW_RATE);
	if (err != 0) {
		LOG_ERR("%d", err);
	}
}