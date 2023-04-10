#include "adxl345.h"


const struct device* init_adxl345(void){
	return 0;
}

int poll_adxl345(const struct device* i2c_dev)
{
	return ADXL345_BW_RATE;
}
