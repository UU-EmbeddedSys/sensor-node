#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>

typedef struct ultrasonic_manager_t {
	float distance;
	struct gpio_dt_spec *signal;
} ultrasonic_manager_t;

void ultrasonic_constructor(ultrasonic_manager_t *ultrasonic_device);

void ultrasonic_init(ultrasonic_manager_t *ultrasonic_device);
void ultrasonic_duration(ultrasonic_manager_t *ultrasonic_device);

#endif /* ULTRASONIC_H */