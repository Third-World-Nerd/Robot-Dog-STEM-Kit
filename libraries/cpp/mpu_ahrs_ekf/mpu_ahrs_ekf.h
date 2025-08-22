#include "ahrs_ekf.h"

#include "MPU9250.h"





#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET 0.0191751
#define GYRO_Y_OFFSET -0.0767002
#define GYRO_Z_OFFSET 0.0199740

#define ACCEL_X_OFFSET 0.0047886
#define ACCEL_Y_OFFSET 0.4345697
#define ACCEL_Z_OFFSET -9.9436264

const float magn_ellipsoid_center[3] = {-16.444, 123.131, -60.0703};
const float magn_ellipsoid_transform[3][3] = {{0.887227, 0.0186386, 0.0175599}, {0.0186386, 0.878481, 0.0445495}, {0.0175599, 0.0445495, 0.978254}};


const double g[3] = {0, 0, -1};
// magnetic inclination angle symbol = delta, magnetic declination symbol = gamma
const double theta = 56.5919*PI/180.0;
const double beta = -4.1934*PI/180.0;
const double r[3] = {cos(theta) * cos(beta), cos(theta) * sin(beta), sin(theta)};


const float Ts = 0.004;
const double P[4] = {0.0,0.0,0.0,0.0}; //uncertainty of quaternion
//const double P[4] = {0.1,0.1,0.1,0.1}; //uncertainty of quaternion
//const float q[4] = {1,0,0,0}; //initial quaternion

// Covariance matrices
const double Q[3] = {0.0000005924204051, 0.0000007749305595, 0.0000004315204792};  // covariance for gyro

const double R[6] = {0.0001175380701662, 0.0001196885607162, 0.0003229845822837, 0.2635248528243216, 0.2635248528243216, 0.2635248528243216}; // covariance for accel and mag

//extern double accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//extern double magnetom[3];
//extern double gyro[3];


extern double* q;

struct YPR {
    float yaw, pitch, roll;
  };


extern YPR ypr;


void imu_read();
void compensate_imu_errors();
void normalize_AM();
void Matrix_Vector_Multiply(const float a[3][3], const double b[3], double out[3]);
void imu_setup();
void ahrs_setup();
void ahrs_update();
void toYPR();
