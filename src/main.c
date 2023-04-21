#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "bme680.h"
#include "adxl345.h"
#include "sample.h"

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define REFRESH_TIME 1000

LOG_MODULE_REGISTER(main);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_polling_stack, MY_STACK_SIZE);

struct k_thread polling_thread, i2c_thread;

struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

typedef struct sensor_tree_t {
	bme680_manager_t bme680_device;
	adxl345_manager_t adxl345_device;
} sensor_tree_t;

sensor_tree_t sensor_tree;

void sensor_polling(void *p1, void *p2, void *p3)
{
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	bme680_constructor(&(sensor_tree.bme680_device));

	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		bme680_read_temperature(&(sensor_tree.bme680_device));

		LOG_INF("I'm doing something\n");
		gpio_pin_toggle_dt(&led0);
	}
}


// typedef int (*i2c_target_write_requested_cb_t)(struct i2c_target_config *config);
int our_i2c_write_requested(struct i2c_target_config *config){
	printf("Write requested\n");
	return 0;
}
// typedef int (*i2c_target_read_requested_cb_t)(struct i2c_target_config *config, uint8_t *val);
int our_i2c_read_requested(struct i2c_target_config *config, uint8_t *val){
	printf("Read requested\n");
	return 0;
}

// typedef int (*i2c_target_write_received_cb_t)( struct i2c_target_config *config, uint8_t val);
int our_i2c_write_received(struct i2c_target_config *config, uint8_t val){
	printf("Write received\n");
	return 0;
}

// typedef int (*i2c_target_read_processed_cb_t)(struct i2c_target_config *config, uint8_t *val);
int our_i2c_read_processed(struct i2c_target_config *config, uint8_t *val){
	printf("Read processed\n");
	return 0;
}
// typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
int our_i2c_stop(struct i2c_target_config *config){
	printf("Stop\n");
	return 0;
}



/* Create a static initializer for a struct i2c_dt_spec */

static struct i2c_target_callbacks sn_i2c_cbs = {
		.write_requested = our_i2c_write_requested ,  // typedef int (*i2c_target_write_requested_cb_t)(struct i2c_target_config *config);
		.read_requested = our_i2c_read_requested,   // typedef int (*i2c_target_read_requested_cb_t)(struct i2c_target_config *config, uint8_t *val);
		.write_received = our_i2c_write_received,   // typedef int (*i2c_target_write_received_cb_t)( struct i2c_target_config *config, uint8_t val);
		.read_processed = our_i2c_read_processed,   // typedef int (*i2c_target_read_processed_cb_t)(struct i2c_target_config *config, uint8_t *val);
		.stop = our_i2c_stop // typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
};


/* Create a static initializer for a struct i2c_target_config */
static struct i2c_target_config i2c_cfg = {
    .address = 0x40, /* The target address on the bus */
    .flags = 0, /* No flags */
	.callbacks = &sn_i2c_cbs
};

const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
uint32_t i2c_speed_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD);


void i2c_communication_target(void *p1, void *p2, void *p3)
{
	int ret = 0;
	if(!device_is_ready(i2c_dev)){
		printk("I2C device not ready");
		return;
	}

	ret  = i2c_configure(i2c_dev, i2c_speed_cfg);
	if (ret < 0) {
		printk("Failed to configure I2C: %d\n", ret);
		return;
	}

	LOG_INF("Speed: %d\n", I2C_SPEED_GET(i2c_speed_cfg));

	/* Enable I2C target mode for the bus driver */
    ret = i2c_target_register (i2c_dev, &i2c_cfg);
    if (ret < 0) {
        printk("Failed to enable I2C target mode: %d\n", ret);
        return;
    }
	while(true){
		k_sleep(K_MSEC(REFRESH_TIME));
	}
}

const struct device *const i2c_dev_source = DEVICE_DT_GET(DT_NODELABEL(i2c0));
uint32_t i2c_cfg_source = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

void i2c_communication_source(void *p1, void *p2, void *p3){
	
	int err;
    if (!device_is_ready(i2c_dev_source)) {
        LOG_ERR("I2C: Device is not ready.\n");
        return;
    }

    err = i2c_configure(i2c_dev_source, i2c_cfg_source);
    if (err != 0) {
        LOG_ERR("i2c_configure\n");
    }

    uint8_t dummy = 0;
    while(true){
        LOG_INF("writing\n");
        // err = sensor_node_write_reg(i2c_dev, &dummy, 1, 0x01);
        err = i2c_reg_read_byte(i2c_dev_source, 0x40, 0x01, &dummy);
        if(err != 0)
        {
            LOG_ERR("I2C:%d", err);
        }
        k_sleep(K_MSEC(1000));
    }
}



void main(void)
{
	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_target, NULL,
		NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);
	k_tid_t polling_thread_tid = k_thread_create(
		&polling_thread, sensor_polling_stack, K_THREAD_STACK_SIZEOF(sensor_polling_stack),
		i2c_communication_source, NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_cpu_pin(&polling_thread_tid, 0);
	k_thread_cpu_pin(&i2c_thread_tid, 1);

	k_thread_start(i2c_thread_tid);
	k_thread_start(polling_thread_tid);
}