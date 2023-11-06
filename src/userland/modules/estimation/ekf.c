#include <modules/ekf.h>
#include <device/bmp280.h>


void __getAngles(Angles* angles) {
    angles->x = 0;
    angles->y = 0;
    angles->z = 0;
}













double __getAltitude(void) {
    double altitude = BMP280_get_altitude();
    return altitude;
}