#ifndef I2C_REGISTERS_H
#define I2C_REGISTERS_H

#define SENSOR_ID  0x03 // 1 byte RO
#define INT_ENABLE 0x04 // 1 byte RW
#define INT_SOURCE 0x05 // 1 byte RW

// TEST REGISTER
// read: 1x8bytes(double)
#define TEST_READ_DOUBLE 0x10
#define TEST_READ_SCALE	 0x20

// ADXL345
// BME680
// read: 3x4 bytes (float) xyz
#define ADXL345_READ_X 0xA0 // until 0x53
#define ADXL345_READ_Y 0xB0 // until 0x63
#define ADXL345_READ_Z 0xC0 // until 0x73

// BME680
// read: 3x8 bytes (double) temperature, pressure, humidity
// configuration: 3x1byte (uint8_t) sampling rate for temperature, pressure, humidity
#define BME680_READ_TEMP     0x50 // until 0x57
#define BME680_READ_PRESSURE 0x60 // until 0x67
#define BME680_READ_HUMIDITY 0x70 // until 0x77

#define BME680_CONFIG_TEMP     0x5A
#define BME680_CONFIG_PRESSURE 0x6A
#define BME680_CONFIG_HUMIDITY 0x7A

#define TEMP_UP_TRIGGER 0x30 // (8 bytes) 30-31-32-33-34-35-36-37
#define HUM_UP_TRIGGER 0x38 // (8 bytes) 38-39-3A-3B-3C-3D-3E-3F
#define PRES_UP_TRIGGER 0x40 // (8 bytes) 40-41-42-43-44-45-46-47
#define ACCEL_UP_TRIGGER 0x48 // (4 bytes) 48-49-4A-4B

// ULTRASONIC
// read: 1x4bytes (float) distance in cm
// configuration 1x1byte (uint8_t) polling rate (not implemented)
#define ULTRASONIC_READ	   0x80 // until 0x83

#endif