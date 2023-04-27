#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "bme680.h"
#include "adxl345.h"
#include "ultrasonic.h"
#include "sample.h"
#include "i2c_registers.h"

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define MY_STACK_SIZE 5000
#define MY_PRIORITY   5

#define REFRESH_TIME 500

LOG_MODULE_REGISTER(main);

K_THREAD_STACK_DEFINE(i2c_stack, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(sensor_polling_stack, MY_STACK_SIZE);

struct k_thread polling_thread, i2c_thread;



struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

//SENSOR POLLING
typedef struct sensor_tree_t {
	bme680_manager_t bme680_device;
	adxl345_manager_t adxl345_device;
	ultrasonic_manager_t ultrasonic_device;
} sensor_tree_t;

sensor_tree_t sensor_tree;

void sensor_polling(void *p1, void *p2, void *p3)
{
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);

	bme680_constructor(&(sensor_tree.bme680_device));
	//ultrasonic_init(&(sensor_tree.ultrasonic_device));
	while (true) {
		k_sleep(K_MSEC(REFRESH_TIME));
		
		bme680_read_sensors(&(sensor_tree.bme680_device));
		//ultrasonic_duration(&(sensor_tree.ultrasonic_device));

		gpio_pin_toggle_dt(&led0);

		// LOG_INF("\nT: %lf, P: %lf, H: %lf\n", 
		// 	sensor_tree.bme680_device.last_temperature,
		// 	sensor_tree.bme680_device.last_pressure,
		// 	sensor_tree.bme680_device.last_humidity);

		printk("\n");
	}
}


//I2C

/*
Explanation for tomorrow:
basically it supports multibyte read but only single byte write, but thats fine we dont need anything else.

the thing is that when the driver sends one read (of X bytes) and then one write and then one read again,
everything worked as expected, but then I tested two writes following each other and thats the problem
I've tried modifying Alessios write function, ive tried using the stock writing a single byte function
nothing seems to work as expected because what it does is that:

write requests
write receives (address of register)
stop
write requests
write receives (contents of register)
write receives (address of the second register to write)
write receives (contents of the second register )


so the problem there was that I cannot know when to start writing the second register
now i've solved it but yeah we cannot write two bytes to a single memory address
only single byte writes to each register, but thats all we need
to configure the sensor node as the sampling rates all fit within one byte for each configuration


*/

uint8_t send_buffer = 0x00;
uint8_t left_to_send = 0;
uint8_t* address_to_next = NULL;

uint8_t register_writing_address = 0xFF;
bool last_instruction_is_write = false;
uint8_t writing_mode = 0;

void i2c_load_next_streamed_value(){
	if(left_to_send > 0){
		send_buffer = *((uint8_t*)address_to_next);
		address_to_next++;
		left_to_send--;
	}
	else{
		send_buffer = 0x00;
	}
}

double test_value_double = 0.5; //this is because I don't have a real sensor yet
uint64_t test_value_scale = 0x1122334455667788;

void i2c_write_register(uint8_t value){
	
	switch(register_writing_address){
		case BME680_CONFIG_HUMIDITY:
			sensor_tree.bme680_device.hum_oversampling = value;
			break;
		case BME680_CONFIG_TEMP:
			sensor_tree.bme680_device.temp_oversampling = value;
			break;
		case BME680_CONFIG_PRESSURE:
			sensor_tree.bme680_device.press_oversampling = value;
			break;
		default:
			printf("Writing register %02x WITH VALUE %02x\n", register_writing_address, value);
			break;
	}
	writing_mode += 1;

}


// typedef int (*i2c_target_write_requested_cb_t)(struct i2c_target_config *config);
int our_i2c_write_requested(struct i2c_target_config *config){
	printf("Write requested \n");

	if(!last_instruction_is_write && writing_mode){
		printf("PREPARE FOR WRITING\n");
	}
	else if(last_instruction_is_write && writing_mode){
		printf("WE'VE FINISHED PREVIOUS WRITE STREAK\n");
		last_instruction_is_write = false;
		writing_mode = 0;
	}
	if(!last_instruction_is_write && !writing_mode){
		printf("WE COME FROM READING\n");
	}
	

	return 0;
}
// typedef int (*i2c_target_read_requested_cb_t)(struct i2c_target_config *config, uint8_t *val);
int our_i2c_read_requested(struct i2c_target_config *config, uint8_t *val){
	*val = send_buffer;
	printf("Read requested (left to send %d): Answering with 0x%02x\n", left_to_send, *val);	
	last_instruction_is_write = false;	
	i2c_load_next_streamed_value();
	return 0;
}
// typedef int (*i2c_target_write_received_cb_t)( struct i2c_target_config *config, uint8_t val);
int our_i2c_write_received(struct i2c_target_config *config, uint8_t address){
	printf("Write received %02x\n", address);
	if(writing_mode && !last_instruction_is_write){
		i2c_write_register(address);
		writing_mode = 2;
		last_instruction_is_write = true;
		return 0;
	}
	
	last_instruction_is_write = true;
	
	if (writing_mode == 2 && last_instruction_is_write){
		register_writing_address = address;
		last_instruction_is_write = false;
	}
	switch(address){
		case TEST_READ_DOUBLE:
			//setup
			address_to_next = (uint8_t*)&test_value_double;
			left_to_send = 8;
			break;
		case TEST_READ_SCALE:
			//setup
			address_to_next = (uint8_t*)&test_value_scale;
			left_to_send = 8;
			break;
		case BME680_READ_TEMP:
			//setup
			address_to_next = (uint8_t*)&(sensor_tree.bme680_device.last_temperature);
			left_to_send = 8;
			break;
		case BME680_READ_PRESSURE:
			//setup
			address_to_next = (uint8_t*)&(sensor_tree.bme680_device.last_pressure);
			left_to_send = 8;
			break;
	
		case BME680_READ_HUMIDITY:
			//setup
			address_to_next = (uint8_t*)&(sensor_tree.bme680_device.last_humidity);
			left_to_send = 8;
			break;
	
		case ULTRASONIC_READ:
			//setup
			address_to_next = (uint8_t*)&(sensor_tree.ultrasonic_device.distance);
			left_to_send = 4;
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
int our_i2c_read_processed(struct i2c_target_config *config, uint8_t *val){
	printf("Read processed\n");
	*val = 0x00;
	return 0;
}
// typedef int (*i2c_target_stop_cb_t)(struct i2c_target_config *config);
int our_i2c_stop(struct i2c_target_config *config){
	printf("Stop\n");
	address_to_next = NULL;
	left_to_send = 0;
	send_buffer = 0x00;
	
	if(last_instruction_is_write) {
		writing_mode = 1;
		last_instruction_is_write = false;
	}
	else register_writing_address = 0xFF;

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


void main(void)
{
	k_tid_t i2c_thread_tid = k_thread_create(
		&i2c_thread, i2c_stack, K_THREAD_STACK_SIZEOF(i2c_stack), i2c_communication_target, NULL,
		NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);
	k_tid_t polling_thread_tid = k_thread_create(
		&polling_thread, sensor_polling_stack, K_THREAD_STACK_SIZEOF(sensor_polling_stack),
		sensor_polling, NULL, NULL, NULL, MY_PRIORITY, K_INHERIT_PERMS, K_FOREVER);

	k_thread_cpu_pin((k_tid_t) &polling_thread_tid, 0);
	k_thread_cpu_pin((k_tid_t) &i2c_thread_tid, 1);

	k_thread_start(i2c_thread_tid);
	k_thread_start(polling_thread_tid);
}