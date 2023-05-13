#include <ultrasonic.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// #include <zephyr/drivers/gpio.h>
#define TIMEOUT 10000000L

LOG_MODULE_REGISTER(ultrasonic, LOG_LEVEL_INF);

struct gpio_dt_spec us0 = GPIO_DT_SPEC_GET(DT_NODELABEL(my_us), gpios);

ultrasonic_manager_t *ultrasonic_device_global = NULL;
static bool start_read = false;

void ultrasonic_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	static uint32_t begin, end;
	static bool edge = false; // low
	if (start_read) {
		if (edge == false) // is rising
		{
			begin = k_cycle_get_32();
			edge ^= true;
		} else // is falling
		{
			end = k_cycle_get_32();
			edge ^= true;
			ultrasonic_device_global->distance =
				k_cyc_to_us_floor64(end - begin) / 29 / 2;
		}
	}
}

static struct gpio_callback ultrasonic_cb_data;

void ultrasonic_init(ultrasonic_manager_t *ultrasonic_device)
{
	ultrasonic_device->signal = &us0;
	// gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
	// GPIO_OUTPUT); gpio_pin_set(ultrasonic_device->signal->port,
	// ultrasonic_device->signal->pin, 0); k_sleep(K_MSEC(2));
	// gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 1);
	// k_sleep(K_MSEC(5));
	// gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 0);
	// gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
	// GPIO_INPUT);

	int err = gpio_pin_interrupt_configure_dt(ultrasonic_device->signal, GPIO_INT_EDGE_BOTH);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on pin %u\n", err,
			ultrasonic_device->signal->pin);
		return;
	}

	gpio_init_callback(&ultrasonic_cb_data, ultrasonic_isr,
			   BIT(ultrasonic_device->signal->pin));
	err = gpio_add_callback(ultrasonic_device->signal->port, &ultrasonic_cb_data);
	if (err != 0) {
		LOG_ERR("Error %d: failed to add callback on pin %u\n", err,
			ultrasonic_device->signal->pin);
		return;
	}
	ultrasonic_device_global = ultrasonic_device;
}

void ultrasonic_duration(ultrasonic_manager_t *ultrasonic_device)
{
	start_read = false;
	gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
			   GPIO_OUTPUT_LOW);
	k_busy_wait(2000); // 2 microseconds
	gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
			   GPIO_OUTPUT_HIGH);
	k_busy_wait(5000); // 5 microseconds
	gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
			   GPIO_OUTPUT_LOW);
	gpio_pin_configure(ultrasonic_device->signal->port, ultrasonic_device->signal->pin,
			   GPIO_INPUT);
	start_read = true;
}
