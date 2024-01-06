
#ifndef __EKFILTER_H
#define __EKFILTER_H
#include <stdint.h>
#include <stdbool.h>
#include <cm4.h>

typedef struct {
    double roll;
    double pitch;
    double yaw;
} Angle;

typedef struct {
    double w;
    double x;
    double y;
    double z;
} Quaternion;


bool __getAngles(Angle* angles);
double __getAltitude(Angle* angles);

#endif

