#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50 // Analog servos run at 50 Hz updates

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Format: "0:300"
    int sep = input.indexOf(':'); 
    if (sep != -1) {
      int servoNum = input.substring(0, sep).toInt();
      int pulse = input.substring(sep + 1).toInt();
      pwm.setPWM(servoNum, 0, pulse);
      
    }
  }
}
