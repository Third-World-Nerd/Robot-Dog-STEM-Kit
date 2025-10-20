# Robot-Dog

https://github.com/user-attachments/assets/f82f04b2-c392-450a-b499-d7d90e3e596e

#### Run cmd prompt as administrator and run:
```
mklink /D "C:\Users\sakar\OneDrive\Documents\Arduino\libraries\LegControl" "E:\THIRD WORLD NERD\robotDog\libraries\cpp\LegControl"
```
> do the same for rest of the library folders as well.

#### Calibration Data:
*Sampling Frequency: 100Hz*
**Magnetometer:**
```
const float magn_ellipsoid_center[3] = {-16.444, 123.131, -60.0703};
const float magn_ellipsoid_transform[3][3] = {{0.887227, 0.0186386, 0.0175599}, {0.0186386, 0.878481, 0.0445495}, {0.0175599, 0.0445495, 0.978254}};
```
**Accelerometer and Gyroscope:**
```
#define GYRO_X_OFFSET 0.0003547
#define GYRO_Y_OFFSET 0.0002904
#define GYRO_Z_OFFSET 0.0000442

#define ACCEL_X_OFFSET -1.7587781
#define ACCEL_Y_OFFSET -8.6502850
#define ACCEL_Z_OFFSET -9.2377464
```
#### Requirements:
> Install Bolder Flight Systems Eigen Library (can be downloaded directly from arduino ide libraries section)
> More details here: https://docs.arduino.cc/libraries/bolder-flight-systems-eigen/

#### Extra Calibration Data For Extended Kalman Filter
```
    const double Q[3] = {0.0000005924204051, 0.0000007749305595, 0.0000004315204792};  // covariance for gyro
    const double R[6] = {0.0001175380701662, 0.0001196885607162, 0.0003229845822837, 0.2635248528243216, 0.2635248528243216, 0.2635248528243216}; // covariance for accel and mag
```
> The Extended Kalman Filter maths applied is based on the documentation provided here: https://ahrs.readthedocs.io/en/latest/filters/ekf.html
