#include "LegControl.h"

Adafruit_PWMServoDriver servoBoard = Adafruit_PWMServoDriver(0x40);

void initializeLeg() {
  servoBoard.begin();
  servoBoard.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  delay(10); // Allow time for the servo board to initialize


  //Initalize all legs to midpoint
  // for (int leg = 0; leg < NUM_LEGS; leg++) {
  //   for (int joint = 0; joint < NUM_JOINTS; joint++) {
  //     int pulse = (PULSE_MIN + PULSE_MAX) / 2;
  //     servoBoard.setPWM((leg) * 3 + joint, 0, pulse);
  //   }
  // }
}

void moveLeg(int leg, int joint, int angle) {
  int angleMin, angleMax;
  int pulseMin, pulseMax;
  int phase;


  switch (joint) {
    case KNEE_JOINT:
      angleMin = KNEE_JOINT_ANGLE_MIN;
      angleMax = KNEE_JOINT_ANGLE_MAX;
      phase=-180;

      break;

    case HIP_LEG_JOINT:
      angleMin = HIP_LEG_JOINT_ANGLE_MIN;
      angleMax = HIP_LEG_JOINT_ANGLE_MAX;
      phase=150;
      break;

    case HIP_BODY_JOINT:
      angleMin = HIP_BODY_JOINT_ANGLE_MIN;
      angleMax = HIP_BODY_JOINT_ANGLE_MAX;
      phase=90;
      break;

    default:
      return;
  }

  // Constrain and map

  angle = constrain(angle, angleMin, angleMax);
  int pulse = map(abs(angle+phase), 0, 180, PULSE_MIN, PULSE_MAX);
  if (leg == LEG_2 || leg == LEG_4) {
    pulse = PULSE_MIN + PULSE_MAX - pulse;   // Invert within range
  }

  if(leg==LEG_3 && joint==HIP_LEG_JOINT){
    pulse=map(pulse,PULSE_MIN,PULSE_MAX,PULSE_MIN_210,PULSE_MAX_210);
  }

  if(leg == LEG_2 && joint == HIP_LEG_JOINT){
    pulse = map(pulse, PULSE_MIN, PULSE_MAX, PULSE_MIN_320, PULSE_MAX_320);
  }


  int channel=((leg) * 3) + joint;

  pulse=pulse+pulse_offset[channel];
  // if(channel==2 && pulse<PULSE_MIN_2){
  //   pulse=PULSE_MIN_2;
  // }
  //   if(channel==5 && pulse>PULSE_MAX_5){
  //   pulse=PULSE_MAX_5;
  // }
  //   if(channel==8 && pulse<PULSE_MIN_8){
  //   pulse=PULSE_MIN_8;
  // }
  //   if(channel==11 && pulse>PULSE_MAX_11){
  //   pulse=PULSE_MAX_11;
  // }
  servoBoard.setPWM(channel, 0 ,pulse);

}
