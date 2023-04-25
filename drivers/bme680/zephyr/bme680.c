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

static bme680_temp_calib_data_t calib_data_temperature = {};
static bme680_humidity_calib_data_t calib_data_humidity = {};
static bme680_pressure_calib_data_t calib_data_pressure = {};

const int32_t const_array1_int[] = {2147483647, 2147483647, 2147483647, 2147483647,
				    2147483647, 2126008810, 2147483647, 2130303777,
				    2147483647, 2147483647, 2143188679, 2136746228,
				    2147483647, 2126008810, 2147483647, 2147483647};
const int32_t const_array2_int[] = {
	4096000000, 2048000000, 1024000000, 512000000, 255744255, 127110228, 64000000, 32258064,
	16016016,   8000000,	4000000,    2000000,   1000000,	  500000,    250000,   125000};
const float const_array1[] = {1.0f, 1.0f, 1.0f,	  1.0f,	  1.0f, 0.99f, 1.0f, 0.992f,
			      1.0f, 1.0f, 0.998f, 0.995f, 1.0f, 0.99f, 1.0f, 1.0f};
const float const_array2[] = {8000000.0f,   4000000.0f,	  2000000.0f, 1000000.0f,
			      499500.4995f, 248262.1648f, 125000.0f,  63004.03226f,
			      31281.28128f, 15625.0f,	  7812.5f,    3906.25f,
			      1953.125f,    976.5625f,	  488.28125f, 244.140625f};

static int set_forced_mode(bme680_manager_t *bme680_device);

static float calc_temperature(uint32_t temp_adc);
static float calc_pressure(uint32_t press_adc);
static float calc_humidity(uint32_t press_adc, uint32_t last_temperature);
void bme680_read_temperature(bme680_manager_t* bme680_device);
void bme680_read_humidity(bme680_manager_t* bme680_device);
void bme680_read_pressure(bme680_manager_t* bme680_device);


static int bme680_read_temperature(bme680_manager_t *bme680_device);
static int bme680_read_pressure(bme680_manager_t *bme680_device);
static int bme680_read_humidity(bme680_manager_t *bme680_device);

void bme680_constructor(bme680_manager_t *bme680_device)
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
	} else {
		LOG_INF("I2C device found\n");
	}
	bme680_device->forced_mode = 0x01;
	bme680_device->temp_oversampling = X1 << TEMP_SHIFT;//(0 << 7) | (0 << 6) | (1 << 5);
	bme680_device->hum_oversampling = X2 << HUM_SHIFT;//(0 << 7) | (0 << 6) | (1 << 5);
	bme680_device->press_oversampling = X1 << PRESS_SHIFT;//(0 << 7) | (0 << 6) | (1 << 5);

	// if (!device_is_ready(bme680_device->i2c_dev)) {
	// 	LOG_INF("I2C: Device is not ready.\n");
	// 	return NULL;
	// }

	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
	int err = i2c_configure(bme680_device->i2c_dev, i2c_cfg);
	if (err != 0) {
		LOG_ERR("i2c_configure\n");
	}

	bme680_config_init(bme680_device);

	bme680_device->last_humidity = -1;
	bme680_device->last_pressure = -1;
	bme680_device->last_temperature = -1;
}

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

/**
 * @brief Read the chip ID to verify the communication with the device
 *
 * @param i2c_dev
 */
void bme680_chip_id(bme680_manager_t *bme680_device)
{
	int err = 0;
	uint8_t chip_id = 0;
	err = bme680_read_reg(bme680_device->i2c_dev, &chip_id, 1, BME680_ID);
	if (err != 0) {
		LOG_ERR("bme680_chip_id\n");
	}
	printf("Chip ID 0x%02x\n", chip_id);
	if (chip_id != 0x61) // reset state of a read only register
	{
		LOG_ERR("Wrong chip ID");
	}
}

bme680_temp_calib_data_t bme680_calib_data_temperature(bme680_manager_t *bme680_device)
{
	int err = 0;
	uint8_t read_buf[8] = {0};
	// par_t1
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 2, BME680_PAR_T1_LSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_temperature\n");
	}
	calib_data_temperature.par_t1 = (read_buf[0]) | (read_buf[1] << 8);
	// par_t2 & par_t3
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 3, BME680_PAR_T2_LSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_temperature\n");
	}
	calib_data_temperature.par_t2 = (read_buf[0]) | (read_buf[1] << 8);
	calib_data_temperature.par_t3 = (read_buf[2]);
	printf("PARAMS: t1=0x%02x t2=0x%02x t3=0x%02x\n", calib_data_temperature.par_t1,
	       calib_data_temperature.par_t2, calib_data_temperature.par_t3);
	return calib_data_temperature;
}

bme680_humidity_calib_data_t bme680_calib_data_humidity(bme680_manager_t *bme680_device)
{
	int err = 0;
	uint8_t read_buf[8] = {0};
	// par_t1
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 2, BME680_PAR_H1_LSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_temperature\n");
	}
	calib_data_humidity.par_h1 =
		(read_buf[0] & 0x0F) | (read_buf[1] >> 4); // TODO get only 3 bits

	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 2, BME680_PAR_H2_MSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_temperature\n");
	}
	calib_data_humidity.par_h2 = (read_buf[0] >> 4) | (read_buf[1] << 4); // TODO get only 3
									      // bits

	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 5, BME680_PAR_H3);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_temperature\n");
	}
	calib_data_humidity.par_h3 = (read_buf[0]);
	calib_data_humidity.par_h4 = (read_buf[1]);
	calib_data_humidity.par_h5 = (read_buf[2]);
	calib_data_humidity.par_h6 = (read_buf[3]);
	calib_data_humidity.par_h7 = (read_buf[4]);
	// uint32_t raw_temp = (read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4);

	return calib_data_humidity;
}

bme680_pressure_calib_data_t bme680_calib_data_pressure(bme680_manager_t *bme680_device)
{
	int err = 0;
	uint8_t read_buf[16] = {0};
	// reads all the consecutive bytes from Par P1 to Par P7
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf, (BME680_PAR_P7 - BME680_PAR_P1_LSB),
			      BME680_PAR_P1_LSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_pressure p1-7\n");
	}
	calib_data_pressure.par_p1 = (read_buf[1] << 8) | read_buf[0];
	calib_data_pressure.par_p2 = (read_buf[3] << 8) | read_buf[2];
	calib_data_pressure.par_p3 = read_buf[4];
	calib_data_pressure.par_p4 = (read_buf[6] << 8) | read_buf[5];
	calib_data_pressure.par_p5 = (read_buf[8] << 8) | read_buf[7];
	calib_data_pressure.par_p6 = read_buf[8];
	calib_data_pressure.par_p7 = read_buf[9];
	// let's clean the memory of the buffer
	memset(read_buf, 0, sizeof(read_buf));
	// reads all the consecutive bytes from PAR P8 to PAR 10
	err = bme680_read_reg(bme680_device->i2c_dev, read_buf,
			      (BME680_PAR_P10 - BME680_PAR_P8_LSB), BME680_PAR_P8_LSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_pressure p8-10\n");
	}
	calib_data_pressure.par_p8 = (read_buf[1] << 8) | read_buf[0];
	calib_data_pressure.par_p9 = (read_buf[3] << 8) | read_buf[2];
	calib_data_pressure.par_p10 = read_buf[4];

	return calib_data_pressure;
}

void bme680_soft_reset(bme680_manager_t *bme680_device)
{
	int err = 0;
	uint8_t soft_rst_cmd = 0xB6;

	err = bme680_write_reg(bme680_device->i2c_dev, &soft_rst_cmd, 1, BME680_RESET);
	if (err != 0) {
		LOG_ERR("bme680_soft_reset\n");
	}
}

void bme680_config_init(bme680_manager_t *bme680_device)
{
	LOG_INF("Configuring BME680");
	int err = 0;
	// FIXME the selection of the IIR filter should be done in the measurement
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

	bme680_calib_data_temperature(bme680_device);
	bme680_calib_data_humidity(bme680_device);
	bme680_calib_data_pressure(bme680_device);

}

void bme680_read_sensors(bme680_manager_t* bme680_device)
{
	int err = 0;
	err = set_forced_mode(bme680_device);
	if (err != 0) {
		LOG_ERR("bme680_read_temperature\n");
	}
	bme680_read_temperature(bme680_device);
	bme680_read_humidity(bme680_device);
	bme680_read_pressure(bme680_device);

}



static int set_forced_mode(bme680_manager_t* bme680_device)
{
	int err = 0;
	uint8_t byte = bme680_device->temp_oversampling | bme680_device->forced_mode | bme680_device->press_oversampling;
	err = bme680_write_reg(bme680_device->i2c_dev, &byte, 1, BME680_CTRL_MEAS);
	if (err != 0) {
		LOG_ERR("set_forced_mode\n");
		return err;
	}
	byte = bme680_device->hum_oversampling;
	err = bme680_write_reg(bme680_device->i2c_dev, &byte, 1, BME680_CTRL_HUM);
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
	var1 = ((((float)temp_adc / 16384.0f) - ((float)calib_data_temperature.par_t1 / 1024.0f)) *
		((float)calib_data_temperature.par_t2));

	/* calculate var2 data */
	var2 = (((((float)temp_adc / 131072.0f) -
		  ((float)calib_data_temperature.par_t1 / 8192.0f)) *
		 (((float)temp_adc / 131072.0f) -
		  ((float)calib_data_temperature.par_t1 / 8192.0f))) *
		((float)calib_data_temperature.par_t3 * 16.0f));

	/* t_fine value*/
	calib_data_temperature.t_fine = (var1 + var2);

	/* compensated temperature data*/
	calc_temp = ((calib_data_temperature.t_fine) / 5120.0f);

	return calc_temp;
}

/**
 * @brief calc_humidity function from Bosch datasheet
 *
 * @param hum_adc raw humidity temperature from adc conversion
 * @param last_temperature last read temperature
 * @return float
 */
static float calc_humidity(uint32_t hum_adc, uint32_t last_temperature)
{
	double var1, var2, var3, var4, hum_comp;
	//print calibration data
	printf("par_h1: %d\n", calib_data_humidity.par_h1);
	printf("par_h2: %d\n", calib_data_humidity.par_h2);
	printf("par_h3: %d\n", calib_data_humidity.par_h3);
	printf("par_h4: %d\n", calib_data_humidity.par_h4);
	printf("par_h5: %d\n", calib_data_humidity.par_h5);
	printf("par_h6: %d\n", calib_data_humidity.par_h6);
	printf("par_h7: %d\n", calib_data_humidity.par_h7);

	var1 = hum_adc - (((double)calib_data_humidity.par_h1 * 16.0) +
			  (((double)calib_data_humidity.par_h3 / 2.0) * last_temperature));
	var2 = var1 * (((double)calib_data_humidity.par_h2 / 262144.0) *
		       (1.0 + (((double)calib_data_humidity.par_h4 / 16384.0) * last_temperature) +
			(((double)calib_data_humidity.par_h5 / 1048576.0) * last_temperature *
			 last_temperature)));
	var3 = (double)calib_data_humidity.par_h6 / 16384.0;
	var4 = (double)calib_data_humidity.par_h7 / 2097152.0;
	hum_comp = var2 + ((var3 + (var4 * last_temperature)) * var2 * var2);

	return hum_comp;
}

/**
 * @brief
 *
 * @param press_adc
 * @return float
 */
static float calc_pressure(uint32_t press_adc)
{
 
	double var1, var2, var3, press_comp;
	//print all calib data
	// printf("par_p1: %d\n", calib_data_pressure.par_p1);
	// printf("par_p2: %d\n", calib_data_pressure.par_p2);
	// printf("par_p3: %d\n", calib_data_pressure.par_p3);
	// printf("par_p4: %d\n", calib_data_pressure.par_p4);
	// printf("par_p5: %d\n", calib_data_pressure.par_p5);
	// printf("par_p6: %d\n", calib_data_pressure.par_p6);
	// printf("par_p7: %d\n", calib_data_pressure.par_p7);
	// printf("par_p8: %d\n", calib_data_pressure.par_p8);
	// printf("par_p9: %d\n", calib_data_pressure.par_p9);
	// printf("par_p10: %d\n", calib_data_pressure.par_p10);


	var1 = ((double) calib_data_temperature.t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double) calib_data_pressure.par_p6 / 131072.0);
	var2 = var2 + (var1 * (double) calib_data_pressure.par_p5 * 2.0);
	var2 = (var2 / 4.0) + ((double) calib_data_pressure.par_p4 * 65536.0);
	var1 = ((((double) calib_data_pressure.par_p3 * var1 * var1) / 16384.0) +
	((double) calib_data_pressure.par_p2 * var1)) / 524288.0;
	var1 = (1.0 + (var1 / 32768.0)) * (double) calib_data_pressure.par_p1;
	press_comp = 1048576.0 - (double) press_adc;
	press_comp = ((press_comp - (var2 / 4096.0)) * 6250.0) / var1;
	var1 =
	((double) calib_data_pressure.par_p9 * press_comp * press_comp) /
	2147483648.0;
	var2 = press_comp * ((double) calib_data_pressure.par_p8 / 32768.0);
	var3 = (press_comp / 256.0) * (press_comp / 256.0) *
	(press_comp / 256.0) * (calib_data_pressure.par_p10 / 131072.0);
	press_comp = press_comp + (var1 + var2 + var3 +
			((double) calib_data_pressure.par_p7 * 128.0)) /
	16.0;

	return press_comp;
	}

static int bme680_read_temperature(bme680_manager_t *bme680_device)
{
	uint8_t read_buf[8] = {0};
	int err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 3, BME680_TEMP_MSB);
	if (err != 0) {
		LOG_ERR("read_temp\n");
	}
	uint32_t raw_temp = (read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4);
	LOG_DBG("raw pressure: %u", raw_temp);
	LOG_HEXDUMP_DBG(read_buf, sizeof(read_buf), "pressure");
	bme680_device->last_temperature = calc_temperature(raw_temp);
	return err;
}

static int bme680_read_humidity(bme680_manager_t *bme680_device)
{
	uint8_t read_buf[8] = {0};

	int err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 2, BME680_HUM_MSB);
	if (err != 0) {
		LOG_ERR("bme680_calib_data_humidity\n");
	}

	uint32_t raw_humid =  (read_buf[0] << 8) | (read_buf[1]);
	printf("RAW HUMID: %d - %02x %02x %02x\n", raw_humid, read_buf[0], read_buf[1], read_buf[2]);

	bme680_device->last_humidity = calc_humidity(raw_humid, bme680_device->last_humidity);
	LOG_DBG("raw humidity: %u", raw_humid);
	LOG_HEXDUMP_DBG(read_buf, sizeof(read_buf), "pressure");
	return err;
}

static int bme680_read_pressure(bme680_manager_t *bme680_device)
{
	uint8_t read_buf[8] = {0};
	int err = bme680_read_reg(bme680_device->i2c_dev, read_buf, 3, BME680_PRESS_ADC_MSB);
	if (err != 0) {
		LOG_ERR("read_temp\n");
	}
	uint32_t raw_press = (read_buf[0] << 12) | (read_buf[1] << 4) | (read_buf[2] >> 4);
	LOG_DBG("raw pressure: %u", raw_press);
	LOG_HEXDUMP_DBG(read_buf, sizeof(read_buf), "pressure");
	bme680_device->last_pressure = calc_pressure(raw_press);

	return err;
}

/**
 * @brief
 * Single TPHG cycle is performed
 * Sensor automatically returns to sleep mode afterwards
 * Gas sensor heater only operates during gas measurement
 * @param bme680_device
 */
void bme680_read_sensors(bme680_manager_t *bme680_device)
{
	int err = 0;
	err = set_forced_mode(bme680_device);
	if (err != 0) {
		LOG_ERR("error setting forced mode %d", err);
	}
	err = bme680_read_temperature(bme680_device);
	if (err != 0) {
		LOG_ERR("error reading temperature %d", err);
	}
	err = bme680_read_pressure(bme680_device);
	if (err != 0) {
		LOG_ERR("error reading pressure %d", err);
	}
	err = bme680_read_humidity(bme680_device);
	if (err != 0) {
		LOG_ERR("erro reading humidity %d", err);
	}
}