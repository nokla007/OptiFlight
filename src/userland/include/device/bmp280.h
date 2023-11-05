#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>
#include <i2c.h>
#include <math.h>


/*
 *  BMP280 I2c address
 */
#define BMP280_AD0_LOW     0xEC //address pin low (GND)
#define BMP280_AD0_HIGH    0xEE //address pin high (VCC)
#define BMP280_ADDR        BMP280_AD0_HIGH     // default I2C address  
 /*
  *  BMP280 register address
  */
#define BMP280_REGISTER_DIG_T1      0x88
#define BMP280_REGISTER_DIG_T2      0x8A
#define BMP280_REGISTER_DIG_T3      0x8C

#define BMP280_REGISTER_DIG_P1      0x8E
#define BMP280_REGISTER_DIG_P2      0x90
#define BMP280_REGISTER_DIG_P3      0x92
#define BMP280_REGISTER_DIG_P4      0x94
#define BMP280_REGISTER_DIG_P5      0x96
#define BMP280_REGISTER_DIG_P6      0x98
#define BMP280_REGISTER_DIG_P7      0x9A
#define BMP280_REGISTER_DIG_P8      0x9C
#define BMP280_REGISTER_DIG_P9      0x9E 

#define BMP280_REGISTER_CHIPID      0xD0
#define BMP280_REGISTER_VERSION     0xD1
#define BMP280_REGISTER_SOFTRESET   0xE0
#define BMP280_REGISTER_STATUS      0xF3
#define BMP280_REGISTER_CONTROL     0xF4
#define BMP280_REGISTER_CONFIG      0xF5

#define BMP280_TEMP_XLSB_REG        0xFC	    /*Temperature XLSB Register */
#define BMP280_TEMP_LSB_REG         0xFB        /*Temperature LSB Register  */ 
#define BMP280_TEMP_MSB_REG         0xFA        /*Temperature LSB Register  */  
#define BMP280_PRESS_XLSB_REG       0xF9		/*Pressure XLSB  Register   */
#define BMP280_PRESS_LSB_REG        0xF8		/*Pressure LSB Register     */
#define BMP280_PRESS_MSB_REG        0xF7		/*Pressure MSB Register     */	

  /*calibration parameters */
#define BMP280_DIG_T1_LSB_REG                0x88  
#define BMP280_DIG_T1_MSB_REG                0x89  
#define BMP280_DIG_T2_LSB_REG                0x8A  
#define BMP280_DIG_T2_MSB_REG                0x8B  
#define BMP280_DIG_T3_LSB_REG                0x8C  
#define BMP280_DIG_T3_MSB_REG                0x8D  
#define BMP280_DIG_P1_LSB_REG                0x8E  
#define BMP280_DIG_P1_MSB_REG                0x8F  
#define BMP280_DIG_P2_LSB_REG                0x90  
#define BMP280_DIG_P2_MSB_REG                0x91  
#define BMP280_DIG_P3_LSB_REG                0x92  
#define BMP280_DIG_P3_MSB_REG                0x93  
#define BMP280_DIG_P4_LSB_REG                0x94  
#define BMP280_DIG_P4_MSB_REG                0x95  
#define BMP280_DIG_P5_LSB_REG                0x96  
#define BMP280_DIG_P5_MSB_REG                0x97  
#define BMP280_DIG_P6_LSB_REG                0x98  
#define BMP280_DIG_P6_MSB_REG                0x99  
#define BMP280_DIG_P7_LSB_REG                0x9A  
#define BMP280_DIG_P7_MSB_REG                0x9B  
#define BMP280_DIG_P8_LSB_REG                0x9C  
#define BMP280_DIG_P8_MSB_REG                0x9D  
#define BMP280_DIG_P9_LSB_REG                0x9E  
#define BMP280_DIG_P9_MSB_REG                0x9F



bool BMP280_Init(void);

double BMP280_get_temp(void);
double BMP280_get_pressure(void);
double BMP280_get_altitude(void);

#endif /* __BMP280_H */