import copy
import os
import sys
import time
import numpy as np
from datetime import datetime

# Add paths
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)
libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import FOOT_POSITIONS_WALK
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND
from config import STEP_LENGTH_X
from config import STEP_LENGTH_Y
from config import HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH
from serial_comm import SerialComm
from kinematics import inverse_kinematics


try:
    ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
    time.sleep(2)
except Exception as e:
    ser = None
    print(f"⚠️ Could not connect to motor serial port: {e}")



class WalkingMechanism:

    def __init__(self, arm1, arm2, start, step_lengthX, step_lengthY, offset):
        self.arm1 = arm1
        self.arm2 = arm2
        self.x1, self.y1 = start[0], start[1]
        self.step_lengthX = step_lengthX
        self.step_lengthY = step_lengthY
        self.swing_steps = 8
        self.stance_steps = 24
        self.gait_cycle_len = self.swing_steps + self.stance_steps
        self.offset = offset

    
    def elliptical_path(self):
        
        swing_theta = np.linspace(np.pi, 0, self.swing_steps)
        swing_x = self.x1 + (self.step_lengthX / 2) * np.cos(swing_theta)
        swing_y = self.y1 + self.step_lengthY * np.sin(swing_theta)

        stance_x = np.linspace(self.x1 + self.step_lengthX / 2,self.x1 - self.step_lengthX / 2, self.stance_steps)
        stance_y = np.full(self.stance_steps, self.y1)

        x = np.concatenate([swing_x, stance_x])
        y = np.concatenate([swing_y, stance_y])

        print(x)

        return x, y
    

    def ik(self, x, y):
        b, a = inverse_kinematics(x, y)
        return int(b.item()), int(a.item())


class LegMovement():

    def __init__(self, legs, fp):
        self.legs = legs
        self.foot_positions = fp


    def start(self):
        command = [None] * 4

        for i in range(4):
            x, y = self.foot_positions[i]
            b, a = self.legs[i].ik(x, y)
            command[i] = [f"{int(b)}", f"{int(a)}", "0"]

        print(command)
        command_flat = [angle for leg_angles in command for angle in leg_angles]
        command = f"<{','.join(command_flat)}>"
        print(f"{command}")


        if ser and ser.is_open:
            try:
                ser.write_line(command.encode())
                now = datetime.now()
                minute = now.strftime("%M")
                second = now.strftime("%S")
                millisecond = int(now.microsecond / 1000)
                print(f"Sent: {minute}:{second}:{millisecond}:- {command}")
            except Exception as e:
                print(f"Serial error: {e}")
        else:

            print(f"Error: Serial port {SERIAL_PORT_SEND} not available")

        time.sleep(4)

        
        steps = 0

        while True:
            command = [None] * 4

            current_leg = steps
            leg_trajectories = [leg.elliptical_path() for leg in self.legs]

            leg_positions = []
            for i, (x_path, y_path) in enumerate(leg_trajectories):
                idx = (steps + self.legs[i].offset * self.legs[i].swing_steps) % self.legs[i].gait_cycle_len
                
                xP, yP = x_path[idx], y_path[idx]
                leg_positions.append((xP, yP))

            for i in range(4):
                x, y = leg_positions[i]
                b, a = self.legs[i].ik(x, y)
                command[i] = [f"{int(b)}", f"{int(a)}", "0"]


            command_flat = [angle for leg_angles in command for angle in leg_angles]
            command = f"<{','.join(command_flat)}>"
            print(f"{command}")


            if ser and ser.is_open:
                try:
                    ser.write_line(command.encode())
                    now = datetime.now()
                    minute = now.strftime("%M")
                    second = now.strftime("%S")
                    millisecond = int(now.microsecond / 1000)
                    print(f"Sent: {minute}:{second}:{millisecond}:- {command}")
                except Exception as e:
                    print(f"Serial error: {e}")
            else:

                print(f"Error: Serial port {SERIAL_PORT_SEND} not available")


            time.sleep(0.1)
            steps += 1

wm = [WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[0], STEP_LENGTH_X, STEP_LENGTH_Y, 0),
      WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[1], STEP_LENGTH_X, STEP_LENGTH_Y, 2),
      WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[2], STEP_LENGTH_X, STEP_LENGTH_Y, 1),
      WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[3], STEP_LENGTH_X, STEP_LENGTH_Y, 3),
    ]


Lm = LegMovement(wm, FOOT_POSITIONS_WALK)
Lm.start()


