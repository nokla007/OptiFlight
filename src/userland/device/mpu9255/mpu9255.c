#include <device/mpu9255.h>

#define ACCEL_ALPHA 0.1
#define GYRO_ALPHA 0.8
#define MAG_ALPHA 0.6


static bool MPU_STATUS;

uint8_t BUF[10];
// MPU9255_TypeDef MPU9255_Offset = { 0 };
// MPU9255_TypeDef_Off MPU9255_Magn_Offset = { 0 };


/***
 * Offset Values
*/
double GyroOffset[3] = { 0 };
uint8_t MagOffest[3] = { 0 };

double GyroScaleFactor, AccelScaleFactor, MagnScaleFactor;

/***
 * private functions
*/
void MPU9255_CalAvgValue(uint8_t* pIndex, int16_t* pAvgBuffer, int16_t InVal, int32_t* pOutVal);
void MPU9255_InitGyrOffset(void);
void MPU9255_InitMagnOffset(void);
void __MPU_Reset(void);
bool MPU9255_Check(void);
static void lowPassFilter(double* new_data, double* previous_output, double alpha);

bool MPU_ReadReg(uint8_t RegAddr, uint8_t* pBuffer, uint8_t size)
{
    MPU_STATUS = _I2C_MEM_READ(I2C1, GYRO_ADDRESS, RegAddr, pBuffer, size);
    if (!MPU_STATUS) {
        __MPU_Reset();
        return false;
    }
    return true;
}
bool MPU_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    MPU_STATUS = _I2C_MEM_WRITE(I2C1, GYRO_ADDRESS, RegAddr, Val);
    if (!MPU_STATUS) {
        __MPU_Reset();
        return false;
    }
    return true;
}
bool MAG_ReadReg(uint8_t RegAddr, uint8_t* pBuffer, uint8_t size)
{
    MPU_STATUS = _I2C_MEM_READ(I2C1, MAG_ADDRESS, RegAddr, pBuffer, size);
    if (!MPU_STATUS) {
        __MPU_Reset();
        return false;
    }
    return true;
}
bool MAG_WriteReg(uint8_t RegAddr, uint8_t Val)
{
    MPU_STATUS = _I2C_MEM_WRITE(I2C1, MAG_ADDRESS, RegAddr, Val);
    if (!MPU_STATUS) {
        __MPU_Reset();
        return false;
    }
    return true;
}


bool MPU9255_Init()
{
    MPU_WriteReg(PWR_MGMT_1, 0x00); //init
    delay_ms(45);
    MPU_WriteReg(PWR_MGMT_1, 0x01); //select best available clock source
    delay_ms(45);
    MPU_WriteReg(CONFIG, 0x00); //bandwidth=250hz
    MPU_WriteReg(SMPLRT_DIV, 0x04); //sample rate = 1+4
    MPU_WriteReg(GYRO_CONFIG, 0x00); //250dps
    // GyroScaleFactor = 32.8;
    GyroScaleFactor = 131;
    MPU_WriteReg(ACCEL_CONFIG, 0x18); //16g
    // AccelScaleFactor = 4096;
    AccelScaleFactor = 2048;
    MagnScaleFactor = 0.6;
    // delay_ms(1);



    MPU_WriteReg(INT_PIN_CFG, 0x02); 	// Bypass enable for magnetometer.
    delay_ms(50);


    // Get gyro and mag offset
    MPU9255_InitGyrOffset();
    MPU9255_InitMagnOffset();

    bool imucheck = MPU9255_Check();

    return true;
}

static void lowPassFilter(double* new_data, double* previous_output, double alpha) {
    for (int i = 0; i < 3; i++) {
        new_data[i] = alpha * new_data[i] + (1.0 - alpha) * previous_output[i];
        previous_output[i] = new_data[i];
    }
}

/**
 * @brief Read all IMU data, low pass filtered and normalized
 *
 * @param gyroData
 * @param accelData
 * @param magData
 * @return true
 * @return false
 */
bool MPU9255_READ_ALL(double* gyroData, double* accelData, double* magData) {
    // previous data for low pass filter
    static double accelPrev[3] = { 0 };
    static double gyroPrev[3] = { 0 };
    static double magPrev[3] = { 0 };

    // read all data
    if (!MPU9255_READ_ACCEL(accelData)) return false;
    if (!MPU9255_READ_GYRO(gyroData)) return false;
    if (!MPU9255_READ_MAG(magData)) return false;

    // low pass filter
    lowPassFilter(accelData, accelPrev, ACCEL_ALPHA);
    lowPassFilter(gyroData, gyroPrev, GYRO_ALPHA);
    lowPassFilter(magData, magPrev, MAG_ALPHA);

    return true;
}



/**
  * @brief Get accelerometer data
  * @param  None
  * @retval None
  */
bool MPU9255_READ_ACCEL(double* accelData)
{
    uint8_t data[6];
    int16_t InBuffer[3] = { 0 };

    if (!MPU_ReadReg(ACCEL_XOUT_H, data, 6)) return false;

    InBuffer[0] = (data[0] << 8) | data[1];
    InBuffer[1] = (data[2] << 8) | data[3];
    InBuffer[2] = (data[4] << 8) | data[5];

    accelData[0] = InBuffer[0] / AccelScaleFactor;
    accelData[1] = InBuffer[1] / AccelScaleFactor;
    accelData[2] = InBuffer[2] / AccelScaleFactor;
    return true;

}

/**
  * @brief Get gyroscope readings
  * @param  buffer
  * @retval None
  */
bool MPU9255_READ_GYRO(double* gyroData)
{
    uint8_t data[6];
    int16_t InBuffer[3] = { 0 };

    if (!MPU_ReadReg(GYRO_XOUT_H, data, 6))return false;

    InBuffer[0] = (data[0] << 8) | data[1];
    InBuffer[1] = (data[2] << 8) | data[3];
    InBuffer[2] = (data[4] << 8) | data[5];

    gyroData[0] = InBuffer[0] / GyroScaleFactor - GyroOffset[0];
    gyroData[1] = InBuffer[1] / GyroScaleFactor - GyroOffset[1];
    gyroData[2] = InBuffer[2] / GyroScaleFactor - GyroOffset[2];

    return true;

}
/**
  * @brief Get compass datas
  * @param  None
  * @retval None
  */
bool MPU9255_READ_MAG(double* magData)
{
    uint8_t data[6];
    int16_t InBuffer[3] = { 0 };

    if (!MAG_WriteReg(MAG_CNTL, 0x01))return false;
    // delay_ms(1);

    uint8_t datardy = 0;
    while ((datardy & 0x01) == 0) {
        if (!MAG_ReadReg(0x02, &datardy, 1)) return false;
    }


    if (!MAG_ReadReg(MAG_XOUT_L, data, 6)) return false;
    InBuffer[0] = (data[1] << 8) | data[0];
    InBuffer[1] = (data[3] << 8) | data[2];
    InBuffer[2] = (data[5] << 8) | data[4];
    InBuffer[2] = -InBuffer[2];

    magData[0] = InBuffer[0] * (((double)MagOffest[0] - 128) / 256.0f + 1.0f);
    magData[1] = InBuffer[1] * (((double)MagOffest[1] - 128) / 256.0f + 1.0f);
    magData[2] = InBuffer[2] * (((double)MagOffest[2] - 128) / 256.0f + 1.0f);

    return true;
}

/**
  * @brief  Check MPU9255,ensure communication succeed
  * @param  None
  * @retval true: communicate succeed
  *               false: communicate fualt
  */
bool MPU9255_Check(void)
{
    uint8_t whoami;
    if (!MPU_ReadReg(WHO_AM_I, &whoami, 1)) return false;
    delay_ms(1);
    if (!MAG_ReadReg(0x00, &whoami, 1))return false;

    return true;
}

/**
  * @brief  Digital filter
  * @param *pIndex:
  * @param *pAvgBuffer:
  * @param InVal:
  * @param pOutVal:
  *
  * @retval None
  *
  */
void MPU9255_CalAvgValue(uint8_t* pIndex, int16_t* pAvgBuffer, int16_t InVal, int32_t* pOutVal)
{
    uint8_t i;

    *(pAvgBuffer + ((*pIndex)++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}
/**
  * @brief  Initializes gyroscopes offset
  * @param  None
  * @retval None
  */
void MPU9255_InitGyrOffset(void)
{
    uint8_t i;
    double	TempGx = 0, TempGy = 0, TempGz = 0;

    double gyroData[3];

    for (i = 0; i < 32; i++)
    {
        MPU9255_READ_GYRO(gyroData);

        TempGx += gyroData[0];
        TempGy += gyroData[1];
        TempGz += gyroData[2];

        delay_ms(1);
    }

    GyroOffset[0] = TempGx / 32;
    GyroOffset[1] = TempGy / 32;
    GyroOffset[2] = TempGz / 32;

}

void MPU9255_InitMagnOffset(void)
{
    MAG_WriteReg(MAG_CNTL, 0x01);
    delay_ms(10);
    MAG_ReadReg(MAG_XOUT_L, MagOffest, 6);
}

void __MPU_Reset(void) {
    delay_ms(10);
    _I2C_Reset(I2C1);
    delay_ms(10);
    MPU_WriteReg(PWR_MGMT_1, 0x80);
    delay_ms(45);
    MPU_WriteReg(PWR_MGMT_1, 0x00);
    delay_ms(45);
    MPU_WriteReg(CONFIG, 0x00); //bandwidth=250hz
    MPU_WriteReg(SMPLRT_DIV, 0x04); //sample rate = 1+4
    MPU_WriteReg(GYRO_CONFIG, 0x00); //250dps
    MPU_WriteReg(ACCEL_CONFIG, 0x18); //16g
    delay_ms(10);
    MPU_WriteReg(INT_PIN_CFG, 0x02);
    delay_ms(50);
}



