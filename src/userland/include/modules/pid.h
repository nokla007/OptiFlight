
#ifndef __PID_H
#define __PID_H
#include <stdint.h>
#include <math.h>

#include "timer.h"
#include "gpio.h"

double PID_Controller(double set_value, double mea_value, double max_value, double min_value, double Kp, double Ki, double Kd, double Ts);


void motorDrive(int n);

#endif