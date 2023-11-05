#include <device/bmp280.h>

static bool BMP_STATUS;

void __BMP_Reset(void);

/******
 * Calibration Values
*/
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int32_t t_fine;



/*****
 * Function definitions
*/
void BMP280_ReadReg(uint8_t RegAddr, uint8_t size, uint8_t* pBuffer)
{
    BMP_STATUS = _I2C_MEM_READ(I2C1, BMP280_ADDR, RegAddr, pBuffer, size);
    if (!BMP_STATUS) {
        BMP280_Reset();
    }
}
void BMP280_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    BMP_STATUS = _I2C_MEM_WRITE(I2C1, BMP280_ADDR, RegAddr, Val);
    if (!BMP_STATUS) {
        BMP280_Reset();
    }
}

/****
 * @brief
 * Read calibration values of bmp280
*/
void BMP280_Read_Calibration(void)
{
    uint8_t lsb, msb;

    /* read the temperature calibration parameters */
    BMP280_ReadReg(BMP280_DIG_T1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T1_MSB_REG, 1, &msb);
    dig_T1 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_T2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T2_MSB_REG, 1, &msb);
    dig_T2 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_T3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_T3_MSB_REG, 1, &msb);
    dig_T3 = msb << 8 | lsb;

    /* read the pressure calibration parameters */
    BMP280_ReadReg(BMP280_DIG_P1_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P1_MSB_REG, 1, &msb);
    dig_P1 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P2_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P2_MSB_REG, 1, &msb);
    dig_P2 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P3_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P3_MSB_REG, 1, &msb);
    dig_P3 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P4_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P4_MSB_REG, 1, &msb);
    dig_P4 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P5_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P5_MSB_REG, 1, &msb);
    dig_P5 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P6_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P6_MSB_REG, 1, &msb);
    dig_P6 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P7_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P7_MSB_REG, 1, &msb);
    dig_P7 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P8_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P8_MSB_REG, 1, &msb);
    dig_P8 = msb << 8 | lsb;
    BMP280_ReadReg(BMP280_DIG_P9_LSB_REG, 1, &lsb);
    BMP280_ReadReg(BMP280_DIG_P9_MSB_REG, 1, &msb);
    dig_P9 = msb << 8 | lsb;

}

/* Returns temperature in DegC, double precision. Output value of "1.23"equals 51.23 DegC. */
double BMP280_Compensate_Temperature(int32_t adc_T)
{
    double var1, var2, temperature;
    var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0
        - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;

    return temperature;
}


/* Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa */
double BMP280_Compensate_Pressure(int32_t adc_P)
{
    double var1, var2, pressure;

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);

    if (var1 == 0.0) {
        return 0; // avoid exception caused by division by zero  
    }

    pressure = 1048576.0 - (double)adc_P;
    pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)dig_P9) * pressure * pressure / 2147483648.0;
    var2 = pressure * ((double)dig_P8) / 32768.0;
    pressure = pressure + (var1 + var2 + ((double)dig_P7)) / 16.0;

    return pressure;
}


/***
   * @brief
   * Read temperature in Celsius from BMP280
***/
double BMP280_get_temp() {
    int32_t adc_T;
    uint8_t regs[3];
    BMP280_ReadReg(BMP280_TEMP_MSB_REG, 3, regs);
    adc_T = (regs[0] << 12) | (regs[1] << 4) | (regs[0] >> 4);
    //adc_T = 415148;
    double temp = BMP280_Compensate_Temperature(adc_T);
    return temp;
}

/*****
 * @brief
 * Read pressure in Pascals from BMP280
*/
double BMP280_get_pressure() {
    int32_t adc_P;
    uint8_t regs[3];
    BMP280_ReadReg(BMP280_PRESS_MSB_REG, 1, regs);
    adc_P = (regs[0] << 12) | (regs[1] << 4) | (regs[0] >> 4);
    //adc_P = 51988;
    double pressure = BMP280_Compensate_Pressure(adc_P);
    return pressure;
}

/******
 * @brief
 * Read altitude in meters from BMP280
*/
double BMP280_get_altitude() {

    double pressure = BMP280_get_pressure();
    double temperature = BMP280_get_temp();

    const double P0 = 101325; // sea level pressure in pascals
    const double L = 0.0065; // temperature lapse rate in K/m
    const double T0 = 288.15; // sea level temperature in Kelvin
    //double altitude = ((P0 / pressure) - pow((temperature + 273.15) / T0, 1 / 5.257)) / L;
    double altitude = (1 - pow(pressure / P0, 0.190284)) * 13451.6;

    return altitude;
}


void BMP280_Config(uint8_t osrs_p, uint8_t osrs_t, uint8_t t_sb, uint8_t filter)
{
    uint8_t config = (t_sb << 5) | (filter << 2);
    BMP280_WriteReg(BMP280_REGISTER_CONFIG, config);

    uint8_t ctrl = (osrs_t << 5) | (osrs_p << 2) | (3 << 0);
    BMP280_WriteReg(BMP280_REGISTER_CONTROL, ctrl);
}

bool BMP280_Init()
{
    bool status = true;
    uint8_t u8ChipID, u8CtrlMod, u8Status;

    BMP280_ReadReg(BMP280_REGISTER_CHIPID, 1, &u8ChipID);
    BMP280_ReadReg(BMP280_REGISTER_CONTROL, 1, &u8CtrlMod);
    BMP280_ReadReg(BMP280_REGISTER_STATUS, 1, &u8Status);

    if (u8ChipID == 0x58)
    {
        BMP280_Config(5, 2, 0, 4);
        BMP280_Read_Calibration();

    }
    else
    {
        status = false;
    }
    return  status;
}



void BMP280_Reset(void)
{
    BMP280_WriteReg(BMP280_REGISTER_SOFTRESET, 0xB6);
    BMP280_Config(5, 2, 0, 4);
}
