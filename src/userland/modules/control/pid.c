#include <modules/pid.h>

double PID_Controller(double set_value, double mea_value, double max_value, double min_value, double Kp, double Ki, double Kd, double Ts)
{
    double output, error;
    static double pre_error = 0;    // previous error
    static double integral = 0;
    double derivative;

    error = set_value - mea_value;
    integral = integral + error * Ts;
    derivative = (error - pre_error) / Ts;
    output = Kp * error + Ki * integral + Kd * derivative;

    if (output > max_value)
    {
        output = max_value;
    }
    else if (output < min_value)
    {
        output = min_value;
    }

    pre_error = error;

    return output;
}


void motorDrive(int n) {

}
