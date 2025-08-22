#include <mpu_madgwick.h>

unsigned long tstart;

void setup() {

  imu_setup();
  // nrf_setup();
  delay(2000);

  Serial.begin(2000000);
  //time_former = micros();
}


// -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz
void loop() {
  tstart = millis();
  update_data();
  /*
  Serial.print(eul.roll_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.pitch_e * 180.0 / PI);
  Serial.print(" ");
  Serial.print(eul.yaw_e * 180.0 / PI);
  Serial.println(" ");
*/  
// /*
  Serial.print("ypr\t");
  Serial.print(ypr.yaw * 180.0 / PI);
  Serial.print("\t");
  Serial.print(ypr.pitch * 180.0 / PI);
  Serial.print("\t");
  Serial.println(ypr.roll * 180.0 / PI);
// */
  // Serial.print("quat\t");
  // Serial.print(qua.w);
  // Serial.print("\t");
  // Serial.print(qua.x);
  // Serial.print("\t");
  // Serial.print(qua.y);
  // Serial.print("\t");
  // Serial.println(qua.z);

    // for python visualization

    // sendData(ypr.yaw * 180.0 / PI,ypr.pitch * 180.0 / PI,ypr.roll * 180.0 / PI);
  //sendData(eul.roll_e, eul.pitch_e, eul.yaw_e);
//sendData((float)millis()/1000,magnetom[0], magnetom[1], magnetom[2]);
  //readButtonState();
  //Serial.println(buttonState);
/*

  while(buttonState == 0)
     {
         update_data();
         sendData(eul.roll_e, eul.pitch_e, eul.yaw_e);
         //Serial.println("button pressed");
         ESC.write(1095);
     }
   */     
  sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);*/
  //delayMicroseconds(500);
  while(millis()-tstart<10);
}
