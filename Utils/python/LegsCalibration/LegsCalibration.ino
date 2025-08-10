#include <LegControl.h>

#define NUM_LEGS 4
#define NUM_JOINTS 3

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait efor serial on Leonardooo-type boards
  initializeLeg();  // Setup servos
}

void loop() {
  static bool receiving = false;
  static String inputString = "";

  while (Serial.available()) {
    char ch = Serial.read();

    if (ch == '<') {
      receiving = true;
      inputString = "";
    }


    else if (ch == '>' && receiving) {
      receiving = false;

      int angles[NUM_LEGS][NUM_JOINTS];
      int values[NUM_LEGS * NUM_JOINTS];
      int index = 0;

      char* token = strtok((char*)inputString.c_str(), ",");


      while (token != NULL && index < NUM_LEGS * NUM_JOINTS) {
        values[index++] = atoi(token);
        token = strtok(NULL, ",");
      }

      if (index == NUM_LEGS * NUM_JOINTS) {
        int k = 0;
        for (int i = 0; i < NUM_LEGS; i++) {
          angles[i][KNEE_JOINT] = values[k++];
          angles[i][HIP_LEG_JOINT] = values[k++];
          angles[i][HIP_BODY_JOINT] = values[k++];
        }

        for (int leg = 0; leg < NUM_LEGS; leg++) {
          moveLeg(leg, KNEE_JOINT, angles[leg][KNEE_JOINT]);
          moveLeg(leg, HIP_LEG_JOINT, angles[leg][HIP_LEG_JOINT]);
          moveLeg(leg, HIP_BODY_JOINT, angles[leg][HIP_BODY_JOINT]);
          
        }
      } else {
        Serial.println("Invalid angle data.");
      }
    }

    else if (receiving) {
      inputString += ch;
    }
  }
}