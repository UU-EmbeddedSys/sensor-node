#include <ultrasonic.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// #include <zephyr/drivers/gpio.h>
#define TIMEOUT 1000000L

LOG_MODULE_REGISTER(ultrasonic);

const struct gpio_dt_spec us0 = GPIO_DT_SPEC_GET(DT_NODELABEL(my_us), gpios);

static uint32_t MicrosDiff(uint32_t begin, uint32_t end) {
    return end - begin;
}

void ultrasonic_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	static uint32_t begin, end;
	static bool edge = false; // low
	if(edge == false) // is rising
	{
		begin = k_cycle_get_32();
		edge ^= true;
	}
	else // is falling
	{
		end = k_cycle_get_32();
		edge ^= true;
		LOG_INF("INT: %u", (end - begin)/ 29 / 2);
	}
}

static uint32_t pulseIn(ultrasonic_manager_t* ultrasonic_device) {
    uint32_t begin = k_cycle_get_32();

    // wait for any previous pulse to end
    while (gpio_pin_get(ultrasonic_device->signal->port, ultrasonic_device->signal->pin)) {
        if (MicrosDiff(begin, k_cycle_get_32()) >= TIMEOUT) {
            return 0;
        }
    }

    // wait for the pulse to start
    while (!gpio_pin_get(ultrasonic_device->signal->port, ultrasonic_device->signal->pin)) {
        if (MicrosDiff(begin, k_cycle_get_32()) >= TIMEOUT) {
            return 0;
        }
    }
    uint32_t pulseBegin = k_cycle_get_32();

    // wait for the pulse to stop
    while (gpio_pin_get(ultrasonic_device->signal->port, ultrasonic_device->signal->pin)) {
        if (MicrosDiff(begin, k_cycle_get_32()) >= TIMEOUT) {
            return 0;
        }
    }
    uint32_t pulseEnd = k_cycle_get_32();

    return MicrosDiff(pulseBegin, pulseEnd);
}

static struct gpio_callback ultrasonic_cb_data;


void ultrasonic_init(ultrasonic_manager_t* ultrasonic_device) {
	ultrasonic_device->signal = &us0;
    gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_OUTPUT);
    gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 0);
    k_sleep(K_MSEC(2));
    gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 1);
    k_sleep(K_MSEC(5));
    gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 0);
    gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_INPUT);


	int err = gpio_pin_interrupt_configure_dt(ultrasonic_device->signal, GPIO_INT_EDGE_BOTH);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on pin %u\n",
			 err, ultrasonic_device->signal->pin);
		return;
	}

	gpio_init_callback(&ultrasonic_cb_data, ultrasonic_isr, BIT(ultrasonic_device->signal->pin));
	err = gpio_add_callback(ultrasonic_device->signal->port, &ultrasonic_cb_data);
	if (err != 0) {
		LOG_ERR("Error %d: failed to add callback on pin %u\n",
			 err, ultrasonic_device->signal->pin);
		return;
	}

}

uint32_t ultrasonic_duration(ultrasonic_manager_t* ultrasonic_device) {
	gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_OUTPUT_LOW);
    k_busy_wait(2000); // 2 microseconds
    gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_OUTPUT_HIGH);
    k_busy_wait(5000); // 5 microseconds
    gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_OUTPUT_LOW);
    gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, GPIO_INPUT);
    long duration;
    duration = pulseIn(ultrasonic_device);
    return duration;
    // return pulseIn(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 1, timeout);
}

/* The measured distance from the range 0 to 400 Centimeters */
uint32_t ultrasonic_measure_in_centimeters(ultrasonic_manager_t* ultrasonic_device) {
    return ultrasonic_duration(ultrasonic_device) / 29 / 2;
}

/* The measured distance from the range 0 to 4000 Millimeters */
uint32_t ultrasonic_measure_in_millimeters(ultrasonic_manager_t* ultrasonic_device, uint32_t timeout) {
    return ultrasonic_duration(ultrasonic_device) * (10 / 2) / 29;
}

/* The measured distance from the range 0 to 157 Inches */
uint32_t ultrasonic_measure_in_inches(ultrasonic_manager_t* ultrasonic_device, uint32_t timeout) {
    return ultrasonic_duration(ultrasonic_device) / 74 / 2;
}



void ultrasonic_constructor(ultrasonic_manager_t* ultrasonic_device){
    int err;
	ultrasonic_device->signal = &us0; // GPIO_DT_SPEC_GET(DT_NODELABEL(my_us), gpios);


    if(ultrasonic_device->signal == NULL)
    {
        LOG_ERR("Failed to get the device from DTS");
    }

	err = gpio_pin_interrupt_configure_dt(ultrasonic_device->signal, GPIO_INT_EDGE_BOTH);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on pin %u\n",
			 err, ultrasonic_device->signal->pin);
		return;
	}

	gpio_init_callback(&ultrasonic_cb_data, ultrasonic_isr, BIT(ultrasonic_device->signal->pin));
	err = gpio_add_callback(ultrasonic_device->signal->port, &ultrasonic_cb_data);
	if (err != 0) {
		LOG_ERR("Error %d: failed to add callback on pin %u\n",
			 err, ultrasonic_device->signal->pin);
		return;
	}

	// check if it's an input pin
	// int pin_type = gpio_pin_is_input(ultrasonic_device->signal->port, ultrasonic_device->signal->pin);
	// pin_type = (pin_type << 1) | gpio_pin_is_output(ultrasonic_device->signal->port, ultrasonic_device->signal->pin);
	// LOG_INF("PIN TYPE: %02x\n", pin_type);

}

