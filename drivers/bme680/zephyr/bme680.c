#include <bme680.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


LOG_MODULE_REGISTER(bme680);

/** Master -> write -> Slave
 * 1. Broadcast START
 * 2. Broadcast TARGET ADDR (7bit) + R/W bit set to WRITE
 * 3. Send the byte indicating the internal slave address
 * 4. <- ACK
 * 5. Send byte of data
 */

static bme680_temp_calib_data_t calib_data = {};

const int32_t const_array1_int[] = {2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2126008810, 2147483647, 2130303777, 2147483647, 2147483647, 2143188679, 2136746228, 2147483647, 2126008810, 2147483647, 2147483647};
const int32_t const_array2_int[] = {4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064, 16016016, 8000000, 4000000, 2000000, 1000000, 500000, 250000, 125000};
const float const_array1[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.99f, 1.0f, 0.992f, 1.0f, 1.0f, 0.998f, 0.995f, 1.0f, 0.99f, 1.0f, 1.0f};
const float const_array2[] = {8000000.0f, 4000000.0f, 2000000.0f, 1000000.0f, 499500.4995f, 248262.1648f, 125000.0f, 63004.03226f, 31281.28128f, 15625.0f, 7812.5f, 3906.25f, 1953.125f, 976.5625f, 488.28125f, 244.140625f};

static float calc_temperature(uint32_t temp_adc);

static int set_forced_mode(bme680_manager_t* bme680_device);


/**
 * @brief Read an internal register of the BME680 sensor
 *
 * @param i2c_dev I2C device controller
 * @param read_buf buffer to store the data read on the bus
 * @param num_bytes quantity of data to be read
 * @param start_address Internal address of the sensor
 * @return int
 */
static int bme680_read_reg(const struct device *i2c_dev, uint8_t *read_buf, uint8_t num_bytes,
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
	err = i2c_transfer(i2c_dev, msg, 2, BME680_ADDR);
	return err;
}

/**
 * @brief Write an internal register of the BME680 sensor
 *
 * @param i2c_dev
 * @param write_buf
 * @param num_bytes
 * @param start_address
 * @return int
 */
static int bme680_write_reg(const struct device *i2c_dev, uint8_t *write_buf, uint8_t num_bytes,
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
	err = i2c_transfer(i2c_dev, msg, 2, BME680_ADDR);
	return err;
}

void bme680_constructor(bme680_manager_t* bme680_device)
{
	if (bme680_device == NULL) {
		LOG_INF("Failed to allocate memory for bme680_device\n");
		return;
	}
	/* Initialize I2C device */
    bme680_device->i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!(bme680_device->i2c_dev)) {
        LOG_INF("Failed to get I2C device\n");
        return;
    }
    else{
        LOG_INF("I2C device found\n");
    }
	bme680_device->forced_mode = 0x01;
	bme680_device->temp_oversampling = (0 << 7) | (0 << 6) | (1 << 5);

	// if (!device_is_ready(bme680_device->i2c_dev)) {
	// 	LOG_INF("I2C: Device is not ready.\n");
	// 	return NULL;
	// }

	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int err = i2c_configure(bme680_device->i2c_dev, i2c_cfg);
	if(err != 0)
	{
		LOG_ERR("i2c_configure\n");
	}
	
	bme680_config_init(bme680_device);

	bme680_device->last_humidity = -1;
	bme680_device->last_pressure = -1;
	bme680_device->last_temperature = -1;
}


/**
 * @brief Read the chip ID to verify the communication with the device
 *
 * @param i2c_dev
 */
void bme680_chip_id(bme680_manager_t* bme680_device)
{
	int err = 0;
	uint8_t chip_id = 0;
	err = bme680_read_reg(bme680_device->i2c_dev, &chip_id, 1, BME680_ID);
	if (err != 0) {
		LOG_ERR("bme680_chip_id\n");
	}
	LOG_INF("Chip ID 0x%02x\n", chip_id);
	if (chip_id != 0x61) // reset state of a read only register
	{
		LOG_ERR("Wrong chip ID");
	}
}

bme680_temp_calib_data_t bme680_calib_data(bme680_manager_t* bme680_device)
{
	int err = 0;
	uint8_t read_buf[8] = {0};
	// par_t1
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 2, 0xE9);
	if (err != 0) {
		LOG_ERR("bme680_calib_data\n");
	}
	calib_data.par_t1 = (read_buf[0]) | (read_buf[1] << 8);
	// par_t2 & par_t3
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 3, 0x8A);
	if (err != 0) {
		LOG_ERR("bme680_calib_data\n");
	}
	calib_data.par_t2 = (read_buf[0]) | (read_buf[1] << 8);
	calib_data.par_t3 = (read_buf[2]);
	printf("PARAMS: t1=0x%02x t2=0x%02x t3=0x%02x\n", calib_data.par_t1, calib_data.par_t2,
	       calib_data.par_t3);
	return calib_data;
}

void bme680_soft_reset(bme680_manager_t* bme680_device){
	int err = 0;
	uint8_t soft_rst_cmd = 0xB6;

	err = bme680_write_reg(bme680_device->i2c_dev, &soft_rst_cmd, 1, BME680_RESET);
	if (err != 0) {
		LOG_ERR("bme680_soft_reset\n");
	}
}

void bme680_config_init(bme680_manager_t* bme680_device)
{
	LOG_INF("Configuring BME680");
	int err = 0;
	uint8_t iir_filter = 0;
	// Set the IIR filter
	err = bme680_write_reg(bme680_device->i2c_dev, &iir_filter, 1, BME680_CONFIG);
	if (err != 0) {
		LOG_ERR("bme680_config_init\n");
	}
	LOG_INF("IIR filter set");
	err = set_forced_mode(bme680_device);
	if (err != 0) {
		LOG_ERR("bme680_config_init\n");
	}
	bme680_calib_data(bme680_device);
}

void bme680_read_temperature(bme680_manager_t* bme680_device)
{
	int err = 0;
	err = set_forced_mode(bme680_device);
	if (err != 0) {
		LOG_ERR("bme680_read_temperature\n");
	}
	uint8_t read_buf[8] = {0};
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 3, BME680_TEMP_MSB);
	if (err != 0) {
		LOG_ERR("read_temp\n");
	}
	uint32_t raw_temp = (read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4);
	//printf("RAW: %d - %02x %02x %02x\n", raw_temp, read_buf[0], read_buf[1], read_buf[2]);
	// hexdump(read_buf, 8);
	bme680_device->last_temperature = calc_temperature(raw_temp);
}

static int set_forced_mode(bme680_manager_t* bme680_device)
{
	int err = 0;
	uint8_t byte = bme680_device->temp_oversampling | bme680_device->forced_mode;
	err = bme680_write_reg(bme680_device->i2c_dev, &byte, 1, BME680_CTRL_MEAS);
	return err;
}

/**
 * @brief calc_temperature function got from the Bosch repository
 *
 * @param temp_adc
 * @param data
 * @return float
 */
static float calc_temperature(uint32_t temp_adc)
{
	float var1;
	float var2;
	float calc_temp;

	/* calculate var1 data */
	var1 = ((((float)temp_adc / 16384.0f) - ((float)calib_data.par_t1 / 1024.0f)) *
		((float)calib_data.par_t2));

	/* calculate var2 data */
	var2 = (((((float)temp_adc / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f)) *
		 (((float)temp_adc / 131072.0f) - ((float)calib_data.par_t1 / 8192.0f))) *
		((float)calib_data.par_t3 * 16.0f));

	/* t_fine value*/
	calib_data.t_fine = (var1 + var2);

	/* compensated temperature data*/
	calc_temp = ((calib_data.t_fine) / 5120.0f);

	return calc_temp;
}