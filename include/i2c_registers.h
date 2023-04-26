#ifndef I2C_REGISTERS_H
#define I2C_REGISTERS_H

//TEST REGISTER
//read: 1x8bytes(double)
#define TEST_READ 0x10


//BME680
//read: 3x8 bytes (double) temperature, pressure, humidity
//configuration: 3x1byte (uint8_t) sampling rate for temperature, pressure, humidity
#define BME680_READ_TEMP 0x50 //until 0x57
#define BME680_READ_PRESSURE 0x60 //until 0x67
#define BME680_READ_HUMIDITY 0x70 //until 0x77

#define BME680_CONFIG_TEMP 0x5A
#define BME680_CONFIG_PRESSURE 0x6A
#define BME680_CONFIG_HUMIDITY 0x7A

//ULTRASONIC
//read: 1x4bytes (float) distance in cm
//configuration 1x1byte (uint8_t) polling rate (not implemented)
#define ULTRASONIC_READ 0x80 //until 0x83
#define ULTRASONIC_POLLING 0x8A 

#endif