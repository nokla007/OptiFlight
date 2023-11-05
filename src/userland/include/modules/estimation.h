
#ifndef __ESTIMATION_H
#define __ESTIMATION_H
#include <stdint.h>

typedef struct {
    double x;
    double y;
    double z;
} Angles;


void __getAngles(Angles* angles);
double __getAltitude(void);

#endif
