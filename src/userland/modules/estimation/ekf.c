#include <modules/ekf.h>
#include <device/bmp280.h>
#include <device/mpu9255.h>

#include <math.h>

static bool status = true;

#define RAD_TO_DEG 57.2958
#define G 9.81  // Acceleration due to gravity in m/s^2

// Kalman filter structure
typedef struct
{
    double Q_angle; //angle prediction noise
    double Q_bias;  //bias noise
    double R_measure;   //measurement noise
    double angle;   //angle estimate
    double bias;    //bias estimate
    double P[2][2]; // error covariance matrix
} Kalman_t;

double dt = 0.01;

double k1, k2;

double m_height;
double measured_angles[3];  // roll pitch yaw (measurement)
double gyroData[3], accelData[3], magData[3];

Quaternion quat;
Angle angle_q;

/**
 Private function declaration
 */
void kalman(double* gyrodata, double* acceldata, double* magdata, Angle* angles);
void kalman_filter(double* gyrodata, double* acceldata, double* magdata, Angle* angles);
double Kalman_getAngle(Kalman_t* Kalman, double newAngle, double newRate);
void kalman_quaternion(double gyrodata[3], double acceldata[3], double magdata[3], Quaternion* q);
double kalman_altitude(Angle* angles, double accelZ, double altitude);
void __measurement(double* acceldata, double* magdata, double* angles);
void getMeasuremenetQuaternion(double* acceldata, double* magdata, Quaternion* q);
void euler_to_quaternion(double roll, double pitch, double yaw, Quaternion* q);
static void normalize(double* data, uint8_t size);
static double lowPassFilter(double new_data, double previous_output, double alpha);
void quaternion_to_euler(Quaternion* q, Angle* angle);




static double lowPassFilter(double new_data, double previous_output, double alpha) {
    return alpha * new_data + (1.0 - alpha) * previous_output;
}

static void normalize(double* data, uint8_t size) {
    double square = 0;
    for (int i = 0; i < size; i++) {
        square += data[i] * data[i];
    }
    double norm = sqrt(square);
    if (norm == 0) return;
    for (int i = 0; i < size; i++) {
        data[i] /= norm;
    }
}

/**
 * @brief get angle estimation by sensor fusion
 *
 * @param angles
 * @return true
 * @return false
 */

bool __getAngles(Angle* angles) {
    // static double gyroData[3], accelData[3], magData[3];
    // previous timestamp
    static uint32_t prev_t = 0;
    if (prev_t == 0) prev_t = __getTime();

    // read sensor data
    // bool accel_status = MPU9255_READ_ACCEL(accelData);
    // bool gyro_status = MPU9255_READ_GYRO(gyroData);
    // bool mag_status = MPU9255_READ_MAG(magData);

    bool status = MPU9255_READ_ALL(gyroData, accelData, magData);
    kalman_filter(gyroData, accelData, magData, angles);


    // if (status) {
    //     // Sensor Fusion
    //     // TODO call appropriate sensor fusion algorithm.
    //     // options: kalman filter or quaternion kalman filter        
    //     // kalman(gyroData, accelData, magData, angles);
    //     kalman_filter(gyroData, accelData, magData, angles);
    //     // kalman_quaternion(gyroData, accelData, magData, &quat);
    //     // quaternion_to_euler(&quat, &angle_q);
    // }
    // else {
    //     angles->roll = 0;
    //     angles->pitch = 0;
    //     angles->yaw = 0;
    //     return false;
    // }

    // record time interval
    dt = __getTime() - prev_t;
    dt = dt / 1000;
    prev_t = __getTime();

    return true;
}


/**
 * @brief Naive Kalman filter
 *
 * @param gyrodata
 * @param acceldata
 * @param angles
 */
void kalman(double* gyrodata, double* acceldata, double* magdata, Angle* angles) {
    // Covariances
    static double Q = 0.001; // Predict noise covariance (gyro)
    static double R = 0.005;  // Measurement noise covariance (accel)
    static double P = 1.0;  // Estimate error covariance
    static double P_roll = 1.0;
    static double P_pitch = 1.0;
    static double P_yaw = 1.0;

    // previous accelerometer data
    static double accelPrev[3] = { 0 };

    // Predict step
    angles->roll += gyrodata[0] * dt;
    angles->pitch += gyrodata[1] * dt;
    angles->yaw += gyrodata[2] * dt;

    // error covariance update
    P_roll = P_roll + Q * dt;
    P_pitch = P_pitch + Q * dt;
    P_yaw = P_yaw + Q * dt;

    // // Kalman gain
    double K_roll = P_roll / (P_roll + R);
    double K_pitch = P_pitch / (P_pitch + R);
    double K_yaw = P_yaw / (P_yaw + R);

    k1 = K_roll;
    k2 = K_pitch;

    // Update state estimate using accelerometer data
    __measurement(acceldata, magdata, measured_angles);
    double accel_roll = measured_angles[0];
    double accel_pitch = measured_angles[1];
    double mag_yaw = measured_angles[2];

    angles->roll = (1 - K_roll) * angles->roll + K_roll * accel_roll;
    angles->pitch = (1 - K_pitch) * angles->pitch + K_pitch * accel_pitch;
    angles->yaw = (1 - K_yaw) * angles->yaw + K_yaw * mag_yaw;


    // Update prediction error covariance
    P_roll = (1 - K_roll) * P_roll;
    P_pitch = (1 - K_pitch) * P_pitch;
}

/**
 * @brief Kalman Filter updated
 *
 * @param gyrodata
 * @param acceldata
 * @param magdata
 * @param angles
 */
void kalman_filter(double* gyrodata, double* acceldata, double* magdata, Angle* angles) {
    // kalman filter structure for each angle
    static Kalman_t KalmanRoll = {
        .Q_angle = 0.001,
        .Q_bias = 0.003,
        .R_measure = 0.005 };
    static Kalman_t KalmanPitch = {
        .Q_angle = 0.001,
        .Q_bias = 0.003,
        .R_measure = 0.005 };
    static Kalman_t KalmanYaw = {
        .Q_angle = 0.001,
        .Q_bias = 0.003,
        .R_measure = 0.005 };

    // get measurement data
    __measurement(acceldata, magdata, measured_angles);

    angles->roll = Kalman_getAngle(&KalmanRoll, measured_angles[0], gyrodata[0]);
    angles->pitch = Kalman_getAngle(&KalmanPitch, measured_angles[1], gyrodata[1]);
    angles->yaw = Kalman_getAngle(&KalmanYaw, measured_angles[2], gyrodata[2]);
}

double Kalman_getAngle(Kalman_t* Kalman, double newAngle, double newRate)
{
    // TODO handle angle as x,y coordinate
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};





void __measurement(double* acceldata, double* magdata, double* angles) {
    // normalize mag data
    // double magnorm = sqrt(magdata[0] * magdata[0] + magdata[1] * magdata[1] + magdata[2] * magdata[2]);
    // if (magnorm == 0) 
    double magnorm = 1;
    // roll
    // angles[0] = atan2(-acceldata[0], sqrt(acceldata[1] * acceldata[1] + acceldata[2] * acceldata[2])) * RAD_TO_DEG;
    angles[1] = atan2(acceldata[1], sqrt((acceldata[0] * acceldata[0]) + (acceldata[2] * acceldata[2]))) * RAD_TO_DEG;
    // pitch
    // angles[1] = atan2(acceldata[1], acceldata[2]) * RAD_TO_DEG;
    // angles[1] = atan2(-acceldata[0], acceldata[2]) * RAD_TO_DEG;
    angles[0] = atan2(acceldata[0], sqrt((acceldata[1] * acceldata[1]) + (acceldata[2] * acceldata[2]))) * RAD_TO_DEG;
    // yaw
    // angles[2] = -atan2(magdata[1] / magnorm, magdata[0] / magnorm) * RAD_TO_DEG; // TODO risky
    angles[2] = 0;
}



/**
 * @brief get altitude estimation by fusing acceleration and pressure data
 *
 * @param angle
 * @return double
 */

double __getAltitude(Angle* angle) {
    double z = 0;
    // previous accelData
    static double prevAccelZ = 0;

    // previous timestamp
    static uint32_t prev_t = 0;
    if (prev_t == 0) prev_t = __getTime();

    //read sensor data
    double altitude = BMP280_get_altitude();    // TODO add safety here

    m_height = altitude;

    double accelZ = accelData[2];
    z = kalman_altitude(angle, accelZ, altitude);

    // record time interval
    dt = __getTime() - prev_t;
    dt = dt / 1000;
    prev_t = __getTime();

    // return value
    return z;
}

extern double normal_accel;
double kalman_altitude(Angle* angles, double accelZ, double altitude) {
    static double z = 0;   // initial height = 0
    static double Vz = 0;     // initial velocity_Z = 0
    static double prev_Vz = 0;  // previous V_z
    // Covariances
    static double Q = 0.005; // Process noise covariance (accelerometer)
    static double R = 0.01;  // Measurement noise covariance (barometer)
    static double Pz = 1.0;  // Height estimate error covariance
    static double Pvz = 1.0; //Vertical velocity error covariance

    // get Accelration_Z in inertial frame
    double aZ = (accelZ - normal_accel) * 9.81;            // net acceleration in m/s^2
    aZ = aZ * cos(angles->roll / RAD_TO_DEG) * cos(angles->pitch / RAD_TO_DEG);  // acceleration in inertial frame

    //******** Estimate velocity_z  ***************
    Vz += aZ * dt;                      // velocity_Z predict
    Pvz = Pvz + Q * dt;
    double Kvz = Pvz / (Pvz + R);   // kalman gain
    double Vz_measure = (altitude - z) / dt;    // velocity measurement
    Vz = Vz + Kvz * (Vz_measure - Vz);  // update velocity
    Pvz = (1 - Kvz) * Pvz;


    //***** Estimate height  ***************
    z += Vz * dt + 0.5 * aZ * dt * dt;  // height predict
    Pz = Pz + Q * dt;
    double Kz = Pz / (Pz + R);  // kalman gain
    z = z + Kz * (altitude - z); //update
    Pz = (1 - Kz) * Pz;

    // return height ***********
    return z;
}





/******************************************************************
 * Quaternion based Kalman Filter
*/

Quaternion multiply_quat(Quaternion q1, Quaternion q2)
{
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

void normalize_quat(Quaternion* q) {
    double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0) return;
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

/**
 * @brief convert euler to quaternion representation
 * eauler angles in radians.
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param q
 */
void euler_to_quaternion(double roll, double pitch, double yaw, Quaternion* q) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q->w = cy * cp * cr + sy * sp * sr;
    q->x = cy * cp * sr - sy * sp * cr;
    q->y = sy * cp * cr + cy * sp * sr;
    q->z = sy * cp * sr - cy * sp * cr;

    normalize_quat(q);
}

void quaternion_to_euler(Quaternion* q, Angle* angle) {
    double roll, pitch, yaw;
    // Calculate roll (rotation around x-axis)
    roll = atan2(2 * (q->w * q->x + q->y * q->z), 1 - 2 * (q->x * q->x + q->y * q->y));
    // Calculate pitch (rotation around y-axis)
    pitch = asin(2 * (q->w * q->y - q->z * q->x));
    // Calculate yaw (rotation around z-axis)
    yaw = atan2(2 * (q->w * q->z + q->x * q->y), 1 - 2 * (q->y * q->y + q->z * q->z));
    // Assign calculated angles in degrees
    angle->roll = roll * RAD_TO_DEG;
    angle->pitch = pitch * RAD_TO_DEG;
    angle->yaw = yaw * RAD_TO_DEG;
}



/**
 * @brief Get the Measuremenet Quaternion from acceldata and magdata
 *
 * @param acceldata
 * @param magdata
 * @param q
 */
void getMeasuremenetQuaternion(double* acceldata, double* magdata, Quaternion* q) {
    // roll pitch yaw in degrees
    double roll = atan2(-acceldata[0], sqrt(acceldata[1] * acceldata[1] + acceldata[2] * acceldata[2]));
    double pitch = atan2(acceldata[1], acceldata[2]);
    double yaw = atan2(magdata[1], magdata[0]);

    // covert euler to quaternion
    euler_to_quaternion(roll, pitch, yaw, q);
}

void kalman_quaternion(double gyrodata[3], double acceldata[3], double magdata[3], Quaternion* q) {
    static double Q = 0.001; // Predict noise covariance (gyro)
    static double R = 0.005;  // Measurement noise covariance (accel)
    static double P_qw = 1.0;   // Estimate error covariance
    static double P_qx = 1.0;
    static double P_qy = 1.0;
    static double P_qz = 1.0;

    // previous accelerometer data for lowpassfilter
    static double accelPrev[3] = { 0 };

    // gyro data needs to be converted to radian/s from degree/s
    double gx = gyrodata[0] / RAD_TO_DEG;
    double gy = gyrodata[1] / RAD_TO_DEG;
    double gz = gyrodata[2] / RAD_TO_DEG;

    // predict step
    Quaternion deltaQ;
    deltaQ.w = 0.5 * (-q->x * gyrodata[0] - q->y * gyrodata[1] - q->z * gyrodata[2]) * dt;
    deltaQ.x = 0.5 * (q->w * gyrodata[0] + q->y * gyrodata[2] - q->z * gyrodata[1]) * dt;
    deltaQ.y = 0.5 * (q->w * gyrodata[1] - q->x * gyrodata[2] + q->z * gyrodata[0]) * dt;
    deltaQ.z = 0.5 * (q->w * gyrodata[2] + q->x * gyrodata[1] - q->y * gyrodata[0]) * dt;

    q->w += deltaQ.w;
    q->x += deltaQ.x;
    q->y += deltaQ.y;
    q->z += deltaQ.z;

    normalize_quat(q);


    // error covariance
    P_qw += Q * dt;
    P_qx += Q * dt;
    P_qy += Q * dt;
    P_qz += Q * dt;

    // kalman gain
    double K_qw = P_qw / (P_qw + R);
    double K_qx = P_qx / (P_qx + R);
    double K_qy = P_qy / (P_qy + R);
    double K_qz = P_qz / (P_qz + R);


    // measurement angle and update
    Quaternion measurement;
    getMeasuremenetQuaternion(acceldata, magdata, &measurement);

    q->w += K_qw * (measurement.w - q->w);
    q->x += K_qx * (measurement.x - q->x);
    q->y += K_qy * (measurement.y - q->y);
    q->z += K_qz * (measurement.z - q->z);


    // Update prediction error covariance
    P_qw = (1 - K_qw) * P_qw;
    P_qx = (1 - K_qx) * P_qx;
    P_qy = (1 - K_qy) * P_qy;
    P_qz = (1 - K_qz) * P_qz;
}