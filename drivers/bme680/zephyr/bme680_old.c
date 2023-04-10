#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdio.h>
#include "bme680.h"


/* BME680 device address */
#define BME680_ADDR 0x77
#define BME680_REG_WAI 0xD0


const struct device* init_bme680(void){
	const struct device *i2c_dev;
	/* Initialize I2C device */
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!i2c_dev) {
        printk("Failed to get I2C device\n");
        return;
    }
    else{
        printk("I2C device found\n");
    }
    
    uint8_t data;
    int ret = i2c_reg_read_byte(i2c_dev, BME680_ADDR, BME680_REG_WAI, &data);
    if(ret){
        printk("Failed to read from BME680\n");
    }
    printk("BME680 WAI: 0x%02x", data);

	return i2c_dev;

}


int poll_bme680(const struct device* i2c_dev, uint8_t reg_ctrl_meas)
{
    int ret = 0;

	/* Configure BME680 */
	uint8_t tx_buf[2] = {BME680_CTRL_MEAS, reg_ctrl_meas};
	ret = i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), BME680_ADDR);
	if (ret) {
		printk("Failed to write to BME680\n");
		return;
	}

	/* Read temp_adc, pressure, and humidity from BME680 */
	uint8_t rx_buf[8];
	tx_buf[0] = BME680_PRESS_MSB;
	ret = i2c_write_read(i2c_dev, BME680_ADDR, tx_buf, 1, rx_buf, sizeof(rx_buf));
	if (ret) {
		printk("Failed to read from BME680\n");
		return;
	}

	/* Parse data */
	int32_t temp_adc = ((int32_t)rx_buf[3] << 12) | ((int32_t)rx_buf[4] << 4) | (rx_buf[5] >> 4);
	int32_t pressure = ((int32_t)rx_buf[0] << 12) | ((int32_t)rx_buf[1] << 4) | (rx_buf[2] >> 4);
	int32_t humidity = (rx_buf[6] << 8) | rx_buf[7];

			

	
	uint8_t par_t2_t3_buffer[3];
	tx_buf[0] = 0x8A;
	ret = i2c_write_read(i2c_dev, BME680_ADDR, tx_buf, 1, par_t2_t3_buffer, sizeof(par_t2_t3_buffer));
	if (ret) {
		printk("Failed to read from BME680\n");
		return;
	}
	int32_t par_t2 = (int32_t)par_t2_t3_buffer[0] | ((int32_t)par_t2_t3_buffer[1] << 8);
	int32_t par_t3 = (int32_t)par_t2_t3_buffer[2];


	uint8_t par_t1_buffer[2];
	tx_buf[0] = 0xE9;
	ret = i2c_write_read(i2c_dev, BME680_ADDR, tx_buf, 1, par_t1_buffer, sizeof(par_t1_buffer));
	if (ret) {
		printk("Failed to read from BME680\n");
		return;
	}
	int32_t par_t1 = (int32_t)par_t1_buffer[0] | ((int32_t)par_t1_buffer[1] << 8);


	double var1 = (((double)temp_adc / 16384.0) - ((double)par_t1 / 1024.0)) * ((double)par_t2);
	double var2 = ((((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0)) * (((double)temp_adc / 131072.0) - ((double)par_t1 / 8192.0))) * ((double)par_t3 * 16.0);

	double t_fine = var1 + var2;
	double temp_deg_c = t_fine/ 5120.0;

	printf("\ntemp_adc: %f ºC\n", temp_deg_c);
	//printf("Pressure: %f Pa\n", pressure_pa);
	//printf("Humidity: %f %%\n", humidity_pct);
}
