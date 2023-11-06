
#ifndef __EKFILTER_H
#define __EKFILTER_H
#include <stdint.h>

typedef struct {
    double x;
    double y;
    double z;
} Angles;


void __getAngles(Angles* angles);
double __getAltitude(void);

#endif