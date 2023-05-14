#include "i2c_slave.h"

#include <errno.h>

LOG_MODULE_REGISTER(slave, LOG_LEVEL_INF);

const uint8_t SENSOR_NODE_ID = 0xAA;

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

static i2c_slave_manager_t slave_manager = {.first_write = false,
					    .read_started = false,
					    .remaining_bytes = 0,
					    .buffer = NULL,
					    .w_idx = 0,
					    .write_buffer = {0}};

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
		LOG_DBG("Writing %d", value);
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
/**
 * @brief Write initiate by the Master, R/W bit = 0. An address will be received.
 *
 * @param config
 * @return int
 */
static int our_i2c_write_requested(struct i2c_target_config *config)
{
	LOG_DBG("i2c_write_requested");

	slave_manager.first_write = true;

	return 0;
}

// typedef int (*i2c_target_write_received_cb_t)( struct i2c_target_config *config, uint8_t val);
/**
 * @brief The Master send the value to write. If first, it's the register address that want to
 * access.
 *
 * @param config
 * @param val
 * @return int
 */
static int our_i2c_write_received(struct i2c_target_config *config, uint8_t val)
{
	LOG_DBG("i2c_write_received [0x%02X]", val);
	int err = 0;
	if (slave_manager.first_write) {
		slave_manager.first_write = false;
		slave_manager.start_address = val;
	} else // it's a write
	{
		LOG_DBG("W 0x%02X to 0x%02X", val, slave_manager.start_address);
		switch (slave_manager.start_address) {
		case BME680_CONFIG_HUMIDITY:
			sensor_tree->bme680_device.hum_oversampling = val << HUM_SHIFT;
			break;
		case BME680_CONFIG_TEMP:
			sensor_tree->bme680_device.temp_oversampling = val << TEMP_SHIFT;
			break;
		case BME680_CONFIG_PRESSURE:
			sensor_tree->bme680_device.press_oversampling = val << PRESS_SHIFT;
			break;
		case INT_SOURCE:
			sensor_tree->int_src = val;
			break;
		case TEMP_UP_TRIGGER:
			slave_manager.write_buffer[slave_manager.w_idx++] = val;
			break;
		case HUM_UP_TRIGGER:
			slave_manager.write_buffer[slave_manager.w_idx++] = val;
			break;
		case PRES_UP_TRIGGER:
			slave_manager.write_buffer[slave_manager.w_idx++] = val;
			break;
		case ACCEL_UP_TRIGGER:
			slave_manager.write_buffer[slave_manager.w_idx++] = val;
			break;
		// TODO Add more
		default:
			LOG_DBG("UNKNOWN REGISTER");
			err = -ENXIO;
			break;
		}
	}
	return err;
}

void load_data(i2c_slave_manager_t *slave_manager)
{
	// LOG_INF("addr: [0x%02X]", slave_manager->start_address)
	switch (slave_manager->start_address) {
	case SENSOR_ID:
		slave_manager->buffer = (uint8_t *)&SENSOR_NODE_ID;
		slave_manager->remaining_bytes = sizeof(SENSOR_NODE_ID);
		break;
	case INT_ENABLE: // FIXME
		slave_manager->buffer = (uint8_t *)&SENSOR_NODE_ID;
		slave_manager->remaining_bytes = sizeof(SENSOR_NODE_ID);
		break;
	case INT_SOURCE:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->int_src);
		slave_manager->remaining_bytes = sizeof(sensor_tree->int_src);
		break;
	case BME680_READ_TEMP:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.last_temperature);
		slave_manager->remaining_bytes =
			sizeof(sensor_tree->bme680_device.last_temperature);
		break;
	case BME680_READ_PRESSURE:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.last_pressure);
		slave_manager->remaining_bytes = sizeof(sensor_tree->bme680_device.last_pressure);
		break;
	case BME680_READ_HUMIDITY:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.last_humidity);
		slave_manager->remaining_bytes = sizeof(sensor_tree->bme680_device.last_humidity);
		break;

	case ULTRASONIC_READ:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->ultrasonic_device.distance);
		slave_manager->remaining_bytes = sizeof(sensor_tree->ultrasonic_device.distance);
		break;

	case ADXL345_READ_X:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->adxl345_device.x_acceleration);
		slave_manager->remaining_bytes = sizeof(sensor_tree->adxl345_device.x_acceleration);
		break;

	case ADXL345_READ_Y:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->adxl345_device.y_acceleration);
		slave_manager->remaining_bytes = sizeof(sensor_tree->adxl345_device.y_acceleration);
		break;

	case ADXL345_READ_Z:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->adxl345_device.z_acceleration);
		slave_manager->remaining_bytes = sizeof(sensor_tree->adxl345_device.z_acceleration);
		break;

	case BME680_CONFIG_HUMIDITY:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.hum_oversampling);
		slave_manager->remaining_bytes =
			sizeof(sensor_tree->bme680_device.hum_oversampling);
		break;

	case BME680_CONFIG_PRESSURE:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.press_oversampling);
		slave_manager->remaining_bytes =
			sizeof(sensor_tree->bme680_device.press_oversampling);
		break;

	case BME680_CONFIG_TEMP:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.temp_oversampling);
		slave_manager->remaining_bytes =
			sizeof(sensor_tree->bme680_device.temp_oversampling);
		break;

	case TEMP_UP_TRIGGER:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.temp_thresh);
		slave_manager->remaining_bytes = sizeof(sensor_tree->bme680_device.temp_thresh);
		break;
	case HUM_UP_TRIGGER:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.hum_thresh);
		slave_manager->remaining_bytes = sizeof(sensor_tree->bme680_device.hum_thresh);
		break;
	case PRES_UP_TRIGGER:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->bme680_device.press_thresh);
		slave_manager->remaining_bytes = sizeof(sensor_tree->bme680_device.press_thresh);
		break;
	case ACCEL_UP_TRIGGER:
		slave_manager->buffer = (uint8_t *)&(sensor_tree->adxl345_device.acceleration_threshold);
		slave_manager->remaining_bytes = sizeof(sensor_tree->adxl345_device.acceleration_threshold);
		break;
	default:
		break;
	}
	LOG_HEXDUMP_DBG(slave_manager->buffer, slave_manager->remaining_bytes, "mngr");
}

// typedef int (*i2c_target_read_requested_cb_t)(struct i2c_target_config *config, uint8_t *val);
static int our_i2c_read_requested(struct i2c_target_config *config, uint8_t *val)
{
	LOG_DBG("i2c_read_requested");
	if (slave_manager.read_started == false) {
		slave_manager.read_started = true;
		load_data(&slave_manager);
	}
	*val = *(slave_manager.buffer);
	LOG_DBG("0x%02X", *(slave_manager.buffer));
	return 0;
}

// typedef int (*i2c_target_read_processed_cb_t)(struct i2c_target_config *config, uint8_t *val);
static int our_i2c_read_processed(struct i2c_target_config *config, uint8_t *val)
{
	LOG_DBG("i2c_read_processed");
	LOG_DBG("0x%02X", *(slave_manager.buffer));
	if (slave_manager.remaining_bytes == 0) {
		*val = *(slave_manager.buffer);
		return 0;
	}
	*val = *(slave_manager.buffer);
	slave_manager.buffer++;
	slave_manager.remaining_bytes--;
	return 0;
}
// typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
static int our_i2c_stop(struct i2c_target_config *config)
{
	LOG_DBG("i2c_stop\n");
	LOG_DBG("Stop op on [0x%02X]", slave_manager.start_address);
	LOG_HEXDUMP_DBG(slave_manager.write_buffer, slave_manager.w_idx, "");
	switch (slave_manager.start_address) {
	case TEMP_UP_TRIGGER:
		memcpy(&(sensor_tree->bme680_device.temp_thresh), slave_manager.write_buffer,
		       slave_manager.w_idx);
		LOG_INF("New T threshold: %f", sensor_tree->bme680_device.temp_thresh);
		break;
	case HUM_UP_TRIGGER:
		memcpy(&(sensor_tree->bme680_device.hum_thresh), slave_manager.write_buffer,
		       slave_manager.w_idx);
		LOG_INF("New H threshold: %f", sensor_tree->bme680_device.hum_thresh);
		break;
	case PRES_UP_TRIGGER:
		memcpy(&(sensor_tree->bme680_device.press_thresh), slave_manager.write_buffer,
		       slave_manager.w_idx);
		LOG_INF("New P threshold: %f", sensor_tree->bme680_device.press_thresh);
		break;
	case ACCEL_UP_TRIGGER:
		memcpy(&(sensor_tree->adxl345_device.acceleration_threshold),
		       slave_manager.write_buffer, slave_manager.w_idx);
		LOG_INF("New ACC threshold: %f",
			sensor_tree->adxl345_device.acceleration_threshold);
		break;
	default:
		break;
	}
	LOG_DBG("Completed %s on addr [0x%02X]", (slave_manager.read_started ? "READ" : "WRITE"),
		slave_manager.start_address);
	slave_manager.w_idx = 0;
	slave_manager.first_write = true;
	slave_manager.read_started = false;
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
		LOG_ERR("I2C device not ready");
		return ret;
	}

	ret = i2c_configure(i2c_dev, i2c_speed_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure I2C: %d", ret);
		return ret;
	}

	LOG_DBG("Speed: %d", I2C_SPEED_GET(i2c_speed_cfg));

	/* Enable I2C target mode for the bus driver */
	ret = i2c_target_register(i2c_dev, &i2c_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to enable I2C target mode: %d", ret);
		return ret;
	}
	// add sensor reference
	if (sensors != NULL) {
		sensor_tree = sensors;
	}
	return ret;
}