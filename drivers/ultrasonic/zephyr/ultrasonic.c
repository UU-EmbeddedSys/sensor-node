#include <ultrasonic.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


LOG_MODULE_REGISTER(ultrasonic);

// static uint32_t MicrosDiff(uint32_t begin, uint32_t end) {
//     return end - begin;
// }

// static uint32_t pulseIn(uint32_t pin, uint32_t state, uint32_t timeout = 1000000L) {
//     uint32_t begin = micros();

//     // wait for any previous pulse to end
//     while (digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
//             return 0;
//         }

//     // wait for the pulse to start
//     while (!digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
//             return 0;
//         }
//     uint32_t pulseBegin = micros();

//     // wait for the pulse to stop
//     while (digitalRead(pin)) if (MicrosDiff(begin, micros()) >= timeout) {
//             return 0;
//         }
//     uint32_t pulseEnd = micros();

//     return MicrosDiff(pulseBegin, pulseEnd);
// }

// long Ultrasonic::duration(uint32_t timeout) {
//     pinMode(_pin, OUTPUT);
//     digitalWrite(_pin, LOW);
//     delayMicroseconds(2);
//     digitalWrite(_pin, HIGH);
//     delayMicroseconds(5);
//     digitalWrite(_pin, LOW);
//     pinMode(_pin, INPUT);
//     long duration;
//     duration = pulseIn(_pin, HIGH, timeout);
//     return duration;
// }

// /*The measured distance from the range 0 to 400 Centimeters*/
// long Ultrasonic::MeasureInCentimeters(uint32_t timeout) {
//     long RangeInCentimeters;
//     RangeInCentimeters = duration(timeout) / 29 / 2;
//     return RangeInCentimeters;
// }

// /*The measured distance from the range 0 to 4000 Millimeters*/
// long Ultrasonic::MeasureInMillimeters(uint32_t timeout) {
//     long RangeInMillimeters;
//     RangeInMillimeters = duration(timeout) * (10 / 2) / 29;
//     return RangeInMillimeters;
// }

// /*The measured distance from the range 0 to 157 Inches*/
// long Ultrasonic::MeasureInInches(uint32_t timeout) {
//     long RangeInInches;
//     RangeInInches = duration(timeout) / 74 / 2;
//     return RangeInInches;
// }

static struct gpio_callback ultrasonic_cb_data;

void ultrasonic_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
	LOG_INF("Echo pin interrupt there should be 3 of this");
}

void ultrasonic_constructor(ultrasonic_manager_t* ultrasonic_device){
	/*//indicative_led

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button.port->name, button.pin);
		return;
	}

    ret = gpio_pin_interrupt_configure_dt(&button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);*/
    int err;
	ultrasonic_device->signal = GPIO_DT_SPEC_GET(DT_PATH(sensors, us0), gpio_signal);
	

    if(ultrasonic_device->signal == NULL)
    {
        LOG_ERR("Failed to get the device from DTS");
    }

	//err = gpio_pin_interrupt_configure_dt(ultrasonic_device->signal, GPIO_INT_EDGE_BOTH);
	if (err != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on pin %u\n",
			 err, ultrasonic_device->signal->pin);
		return;
	}

	gpio_init_callback(&ultrasonic_cb_data, ultrasonic_isr, BIT(ultrasonic_device->signal->pin));
	gpio_add_callback(ultrasonic_device->signal->port, &ultrasonic_cb_data);

}

void ultrasonic_read_distance(ultrasonic_manager_t* ultrasonic_device){
	gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 1);
	k_sleep(K_MSEC(10));
	gpio_pin_set(ultrasonic_device->signal->port, ultrasonic_device->signal->pin, 0);
}