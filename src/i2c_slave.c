#include "i2c_slave.h"

LOG_MODULE_REGISTER(slave, LOG_LEVEL_INF);

static sensor_tree_t *sensor_tree;

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
uint32_t i2c_speed_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD);

uint8_t send_buffer = 0x00;
uint8_t left_to_send = 0;
uint8_t *address_to_next = NULL;

uint8_t register_writing_address = 0xFF;
bool last_instruction_is_write = false;
uint8_t writing_mode = 0;

double test_value_double = 0.5; // this is because I don't have a real sensor yet
uint64_t test_value_scale = 0x1122334455667788;

static void i2c_load_next_streamed_value()
{
	if (left_to_send > 0) {
		send_buffer = *((uint8_t *)address_to_next);
		address_to_next++;
		left_to_send--;
	} else {
		send_buffer = 0xFF;
	}
}

static void i2c_write_register(uint8_t value)
{
	LOG_DBG("Writing register %02x WITH VALUE %02x", register_writing_address, value);

	switch (register_writing_address) {
	case BME680_CONFIG_HUMIDITY:
		sensor_tree->bme680_device.hum_oversampling = value << HUM_SHIFT;
		break;
	case BME680_CONFIG_TEMP:
		sensor_tree->bme680_device.temp_oversampling = value << TEMP_SHIFT;
		LOG_INF("Writing %d", value);
		break;
	case BME680_CONFIG_PRESSURE:
		sensor_tree->bme680_device.press_oversampling = value << PRESS_SHIFT;
		break;
	default:
		LOG_DBG("UNKNOWN REGISTER");
		break;
	}
	writing_mode += 1;
}

// typedef int (*i2c_target_write_requested_cb_t)(struct i2c_target_config *config);
static int our_i2c_write_requested(struct i2c_target_config *config)
{
	LOG_INF("Write requested ");

	if (!last_instruction_is_write && writing_mode) {
		LOG_DBG("PREPARE FOR WRITING");
	} else if (last_instruction_is_write && writing_mode) {
		LOG_DBG("WE'VE FINISHED PREVIOUS WRITE STREAK");
		last_instruction_is_write = false;
		writing_mode = 0;
	}
	if (!last_instruction_is_write && !writing_mode) {
		LOG_DBG("WE COME FROM READING");
	}

	return 0;
}
// typedef int (*i2c_target_read_requested_cb_t)(struct i2c_target_config *config, uint8_t *val);
static int our_i2c_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	LOG_INF("READ_REQUESTED");
	*val = send_buffer;
	LOG_DBG("Read requested (left to send %d): Answering with 0x%02x", left_to_send, *val);
	last_instruction_is_write = false;
	i2c_load_next_streamed_value();
	return 0;
}
// typedef int (*i2c_target_write_received_cb_t)( struct i2c_target_config *config, uint8_t val);
static int our_i2c_write_received(struct i2c_target_config *config, uint8_t address)
{
	LOG_INF("Write received %02x", address);
	if (writing_mode && !last_instruction_is_write) {
		i2c_write_register(address);
		writing_mode = 2;
		last_instruction_is_write = true;
		return 0;
	}

	last_instruction_is_write = true;

	if (writing_mode == 2 && last_instruction_is_write) {
		register_writing_address = address;
		last_instruction_is_write = false;
	}
	switch (address) {
	case TEST_READ_DOUBLE:
		// setup
		address_to_next = (uint8_t *)&test_value_double;
		left_to_send = 8;
		break;
	case TEST_READ_SCALE:
		// setup
		address_to_next = (uint8_t *)&test_value_scale;
		left_to_send = 8;
		break;
	case BME680_READ_TEMP:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.last_temperature);
		left_to_send = 8;
		break;
	case BME680_READ_PRESSURE:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.last_pressure);
		left_to_send = 8;
		break;

	case BME680_READ_HUMIDITY:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.last_humidity);
		left_to_send = 8;
		break;

	case ULTRASONIC_READ:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->ultrasonic_device.distance);
		left_to_send = 4;
		break;

	case ADXL345_READ_X:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->adxl345_device.x_acceleration);
		left_to_send = 4;
		break;

	case ADXL345_READ_Y:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->adxl345_device.y_acceleration);
		left_to_send = 4;
		break;

	case ADXL345_READ_Z:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->adxl345_device.z_acceleration);
		left_to_send = 4;
		break;

	case BME680_CONFIG_HUMIDITY:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.hum_oversampling);
		left_to_send = 1;
		break;

	case BME680_CONFIG_PRESSURE:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.press_oversampling);
		left_to_send = 1;
		break;

	case BME680_CONFIG_TEMP:
		// setup
		address_to_next = (uint8_t *)&(sensor_tree->bme680_device.temp_oversampling);
		left_to_send = 1;
		break;

	case CLEAR_I2C:
		// setup
		address_to_next = NULL;
		left_to_send = 0;
		break;

	default:
		address_to_next = NULL;
		left_to_send = 0;
		break;
	}
	register_writing_address = address;
	i2c_load_next_streamed_value();

	return 0;
}
// typedef int (*i2c_target_read_processed_cb_t)(struct i2c_target_config *config, uint8_t *val);
static int our_i2c_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	LOG_INF("Read processed");
	*val = 0x00;
	return 0;
}
// typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
static int our_i2c_stop(struct i2c_target_config *config)
{
	LOG_INF("Stop");
	address_to_next = NULL;
	left_to_send = 0;
	send_buffer = 0x00;

	if (last_instruction_is_write) {
		writing_mode = 1;
		last_instruction_is_write = false;
	} else {
		register_writing_address = 0xFF;
	}

	return 0;
}

/* SLAVE INITIALIZATION */

static struct i2c_target_callbacks sn_i2c_cbs = {
	.write_requested =
		our_i2c_write_requested, // typedef int (*i2c_target_write_requested_cb_t)(struct
					 // i2c_target_config *config);
	.read_requested =
		our_i2c_read_requested, // typedef int (*i2c_target_read_requested_cb_t)(struct
					// i2c_target_config *config, uint8_t *val);
	.write_received = our_i2c_write_received, // typedef int (*i2c_target_write_received_cb_t)(
						  // struct i2c_target_config *config, uint8_t val);
	.read_processed =
		our_i2c_read_processed, // typedef int (*i2c_target_read_processed_cb_t)(struct
					// i2c_target_config *config, uint8_t *val);
	.stop = our_i2c_stop // typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config
			     // *config);
};

static struct i2c_target_config i2c_cfg = {
	.address = SENSOR_NODE_ADDR, /* The target address on the bus */
	.flags = 0,		     /* No flags */
	.callbacks = &sn_i2c_cbs};

int slave_init(sensor_tree_t *sensors)
{
	int ret = 0;
	if (!device_is_ready(i2c_dev)) {
		printk("I2C device not ready");
		return ret;
	}

	ret = i2c_configure(i2c_dev, i2c_speed_cfg);
	if (ret < 0) {
		printk("Failed to configure I2C: %d", ret);
		return ret;
	}

	LOG_INF("Speed: %d", I2C_SPEED_GET(i2c_speed_cfg));

	/* Enable I2C target mode for the bus driver */
	ret = i2c_target_register(i2c_dev, &i2c_cfg);
	if (ret < 0) {
		printk("Failed to enable I2C target mode: %d", ret);
		return ret;
	}
	// add sensor reference
	if (sensors != NULL) {
		sensor_tree = sensors;
	}
	return ret;
}