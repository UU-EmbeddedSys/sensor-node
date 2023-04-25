#ifndef BME680_REGISTERS_H
#define BME680_REGISTERS_H

/* BME680 REGISTERS */

#define BME680_STATUS	  0x73
#define BME680_RESET	  0xE0
#define BME680_ID	  0xD0
#define BME680_CONFIG	  0x75
#define BME680_CTRL_MEAS  0x74
#define BME680_CTRL_HUM	  0x72
#define BME680_CTRL_GAS_1 0x71
#define BME680_CTRL_GAS_0 0x70

#define BME680_GAS_WAIT_0 0x64
#define BME680_GAS_WAIT_1 0x65
#define BME680_GAS_WAIT_2 0x66
#define BME680_GAS_WAIT_3 0x67
#define BME680_GAS_WAIT_4 0x68
#define BME680_GAS_WAIT_5 0x69
#define BME680_GAS_WAIT_6 0x6A
#define BME680_GAS_WAIT_7 0x6B
#define BME680_GAS_WAIT_8 0x6C
#define BME680_GAS_WAIT_9 0x6D

#define BME680_RES_HEAT_0 0x5A
#define BME680_RES_HEAT_1 0x5B
#define BME680_RES_HEAT_2 0x5C
#define BME680_RES_HEAT_3 0x5D
#define BME680_RES_HEAT_4 0x5E
#define BME680_RES_HEAT_5 0x5F
#define BME680_RES_HEAT_6 0x60
#define BME680_RES_HEAT_7 0x61
#define BME680_RES_HEAT_8 0x62
#define BME680_RES_HEAT_9 0x63

#define BME680_IDAC_HEAD_0 0x50
#define BME680_IDAC_HEAD_1 0x51
#define BME680_IDAC_HEAD_2 0x52
#define BME680_IDAC_HEAD_3 0x53
#define BME680_IDAC_HEAD_4 0x54
#define BME680_IDAC_HEAD_5 0x55
#define BME680_IDAC_HEAD_6 0x56
#define BME680_IDAC_HEAD_7 0x57
#define BME680_IDAC_HEAD_8 0x58
#define BME680_IDAC_HEAD_9 0x59

#define BME680_GAS_R_LSB    0x2B
#define BME680_GAS_R_MSB    0x2A
#define BME680_HUM_LSB	    0x26
#define BME680_HUM_MSB	    0x25
#define BME680_TEMP_XLSB    0x24
#define BME680_TEMP_LSB	    0x23
#define BME680_TEMP_MSB	    0x22
#define BME680_PRESS_XLSB   0x21
#define BME680_PRESS_LSB    0x20
#define BME680_PRESS_MSB    0x1F
#define BME680_EAS_STATUS_0 0x1D

#define BME680_TEMP(byte) \
         byte - BME680_TEMP_MSB

// Parameters for temperature sensor
#define BME680_PAR_T1_LSB 0xE9
#define BME680_PAR_T1_MSB 0xEA
#define BME680_PAR_T2_LSB 0x8A
#define BME680_PAR_T2_MSB 0x8B
#define BME680_PAR_T3	  0x8C

// Parameters for the pressure sensor
#define BME680_PAR_P1_LSB     0x8E
#define BME680_PAR_P1_MSB     0x8F
#define BME680_PAR_P2_LSB     0x90
#define BME680_PAR_P2_MSB     0x91
#define BME680_PAR_P3	      0x92
#define BME680_PAR_P4_LSB     0x94
#define BME680_PAR_P4_MSB     0x95
#define BME680_PAR_P5_LSB     0x96
#define BME680_PAR_P5_MSB     0x97
#define BME680_PAR_P6	      0x99
#define BME680_PAR_P7	      0x98
#define BME680_PAR_P8_LSB     0x9C
#define BME680_PAR_P8_MSB     0x9D
#define BME680_PAR_P9_LSB     0x9E
#define BME680_PAR_P9_MSB     0x9F
#define BME680_PAR_P10	      0xA0

#define PRESS_PARAM(byte) \
        byte - BME680_PAR_P1_LSB

// ADC registers for pressure measurement
#define BME680_PRESS_ADC_XLSB 0x21
#define BME680_PRESS_ADC_LSB  0x20
#define BME680_PRESS_ADC_MSB  0x1F

#define BME680_PRESS(byte) \
         byte - BME680_PRESS_ADC_MSB

// Parameters for the Humidity sensor
#define BME680_PAR_H1_LSB  0xE2
#define BME680_PAR_H1_MSB  0xE3
#define BME680_PAR_H2_LSB  0xE2
#define BME680_PAR_H2_MSB  0xE1
#define BME680_PAR_H3	   0xE4
#define BME680_PAR_H4	   0xE5
#define BME680_PAR_H5	   0xE6
#define BME680_PAR_H6	   0xE7
#define BME680_PAR_H7	   0xE8

#define HUM_PARAM(byte) \
    byte - BME680_PAR_H2_MSB

// ADC registers for humidy measurements
#define BME680_HUM_ADC_LSB 0x26
#define BME680_HUM_ADC_MSB 0x25

// Parameters for the Gas sensor
#define BME680_PAR_G1	      0xED
#define BME680_PAR_G2_LSB     0xEB
#define BME680_PAR_G2_MSB     0xEC
#define BME680_PAR_G3	      0xEE
#define BME680_RES_HEAT_RANGE 0x02
#define BME680_RES_HEAT_VAL   0x00
#define BME680_GAS_ADC_LSB    0x2B
#define BME680_GAS_ADC_MSB    0x2A
#define BME680_GAS_RANGE      0x2B
#define BME680_RANGE_SW_ERR   0x04

#endif /* BME680_REGISTERS_H */
