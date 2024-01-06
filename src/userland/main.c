#include <kstdio.h>
#include <ktimes.h>
#include <math.h>
#include <device/bmp280.h>
#include <device/mpu9255.h>

#include <modules/ekf.h>

bool READY = false;

void device_init() {
    BMP280_Init();
    MPU9255_Init();
}

// static double gyroData[3], accelData[3], magData[3];
static double pressure, temperature, altitude;
static Angle angles = { 0 };
static double height = 0;
static bool imustatus = false;
static uint32_t interval = 0;

extern Angle angle2;

uint32_t __c_time;  // for debugging purpose

int main() {
    device_init();

    READY = true;

    uint32_t i = 0;
    while (1) {
        // pressure = BMP280_get_pressure();
        // temperature = BMP280_get_temp();
        // altitude = BMP280_get_altitude();
        // kprintf("p = %f, t = %f, h = %f\n", pressure, temperature, altitude);
        // delay_ms(500);

        // MPU9255_READ_ACCEL(accelData);
        // kprintf("ax: %f\t ay: %f\t az: %f \n", accelData[0], accelData[1], accelData[2]);
        // MPU9255_READ_GYRO(gyroData);
        // kprintf("gx: %f\t gy: %f\t gz: %f \n", gyroData[0], gyroData[1], gyroData[2]);
        // MPU9255_READ_MAG(magData);
        // kprintf("mx: %f\t my: %f\t mz: %f \n", magData[0], magData[1], magData[2]);

        // double yaw = atan2(magData[1], magData[0]) * 57.2958;

        // kprintf("yaw: %f\n", yaw);

        // delay_ms(500);

        uint32_t time = __getTime();
        imustatus = __getAngles(&angles);
        height = __getAltitude(&angles);
        interval = __getTime() - time;
        __c_time = getTime() % 100000;

        // print angles
        if (i++ == 100) {
            i = 0;
            kprintf("roll: %f\t pitch: %f\t yaw: %f\n", angles.roll, angles.pitch, angles.yaw);
        }
    }

    return 0;
}


/***
 * Systick Routine
*/
void SysTick_Routine() {
    // if (READY)
    //     __getAngles(&angles);
}
