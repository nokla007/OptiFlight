
#include <kstdio.h>
#include <ktimes.h>
#include <math.h>
#include <device/bmp280.h>
#include <device/mpu9255.h>

void device_init() {
    BMP280_Init();
    MPU9255_Init();
}

double gyroData[3], accelData[3], magData[3];
double pressure, temperature, altitude;


int main() {
    device_init();



    while (1) {
        pressure = BMP280_get_pressure();
        temperature = BMP280_get_temp();
        altitude = BMP280_get_altitude();
        // kprintf("p = %f, t = %f, h = %f\n", pressure, temperature, altitude);
        delay_ms(500);

        MPU9255_READ_ACCEL(accelData);
        // kprintf("ax: %f\t ay: %f\t az: %f \n", accelData[0], accelData[1], accelData[2]);
        MPU9255_READ_GYRO(gyroData);
        // kprintf("gx: %f\t gy: %f\t gz: %f \n", gyroData[0], gyroData[1], gyroData[2]);
        MPU9255_READ_MAG(magData);
        // kprintf("mx: %f\t my: %f\t mz: %f \n", magData[0], magData[1], magData[2]);

        delay_ms(500);
    }

    return 0;
}



