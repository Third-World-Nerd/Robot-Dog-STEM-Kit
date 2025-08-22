#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);

float accel[3];
float gyro[3];

unsigned long Time;

void read_sensors() {
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();
  
  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}

void setup() {
  Serial.begin(2000000);
  while (!Serial) yield();

  // start communication with IMU 
  IMU.begin();
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
   IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 9 for a 100 Hz update rate
  IMU.setSrd(9);
}

void loop() {
  Time = millis();
  
  read_sensors();
  
  Serial.print(Time);
  Serial.print(",");
  Serial.print(accel[0], 7);
  Serial.print(",");
  Serial.print(accel[1], 7);
  Serial.print(",");
  Serial.print(accel[2], 7);
  Serial.print(",");
  Serial.print(gyro[0], 7);
  Serial.print(",");
  Serial.print(gyro[1], 7);
  Serial.print(",");
  Serial.println(gyro[2], 7);
  
  while(millis()-Time<10);
  
}
