#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);

float magnetom[3];

unsigned long Time;

void read_sensors() {
  IMU.readSensor();
  magnetom[0] = IMU.getMagX_uT();
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();
}

void setup() {
  Serial.begin(2000000);
  // start communication with IMU 
  IMU.begin();
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 3 for 250Hz sampling rate
  IMU.setSrd(9);
}

void loop() {
  Time = millis();
  
  read_sensors();

  Serial.print(Time);
  Serial.print(",");
  Serial.print(magnetom[0], 7);
  Serial.print(",");
  Serial.print(magnetom[1], 7);
  Serial.print(",");
  Serial.println(magnetom[2], 7);

  while(millis()-Time<10); 
}
