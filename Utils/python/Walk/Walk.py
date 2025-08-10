import os
import sys
import time
from datetime import datetime
from math import comb

import numpy as np

# Add paths
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)
libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from kinematics import inverse_kinematics
from serial_comm import SerialComm

from config import BAUD_RATE_SEND
from config import DELAY
from config import FOOT_POSITIONS_WALK
from config import INTERPOLATION_STEPS
from config import SERIAL_PORT_SEND
from config import STEP_LENGTH_X
from config import STEP_LENGTH_Y

ser = SerialComm(port=SERIAL_PORT_SEND, baudrate=BAUD_RATE_SEND)


class Walk:
    def __init__(self):
        self.current_theta_index = 0  # current index in the theta array for the first (leg 0) leg
        swing_theta = np.linspace(np.pi, 0, INTERPOLATION_STEPS // 2)
        stance_theta = np.linspace(0, -np.pi, INTERPOLATION_STEPS // 2)
        self.thetas = np.concatenate((swing_theta, stance_theta))
        self.halfStepLengthsX = [STEP_LENGTH_X / 2] * 4  # Step lengths for each leg in X direction
        self.halfStepLengthsY = [STEP_LENGTH_Y / 2] * 4  # Step lengths for each leg in Y direction

    def reset(self):
        theta_knee = []
        theta_hip = []
        for leg in range(4):
            knee, hip = inverse_kinematics(FOOT_POSITIONS_WALK[leg][0], FOOT_POSITIONS_WALK[leg][1])
            if hip is None or knee is None:
                print(f"Leg {leg} is out of reach: x={FOOT_POSITIONS_WALK[leg][0]}, y={FOOT_POSITIONS_WALK[leg][1]}")
            else:
                theta_knee.append(knee)
                theta_hip.append(hip)
        command_values = []
        for leg in range(4):
            command_values.extend([str(int(theta_knee[leg])), str(int(theta_hip[leg])), "0"])
        command_str = "<" + ",".join(command_values) + ">"
        ser.write_line(command_str)

    def walk_one_interpolation_step(self):
        leg_x = []
        leg_y = []
        theta_hip = []
        theta_knee = []

        for leg in range(4):
            leg_x.append(
                FOOT_POSITIONS_WALK[leg][0] + self.halfStepLengthsX[leg] * np.cos(self.thetas[(self.current_theta_index + INTERPOLATION_STEPS // 2 * (leg == 1 or leg == 2)) % INTERPOLATION_STEPS])
            )
            leg_y.append(
                FOOT_POSITIONS_WALK[leg][1] + self.halfStepLengthsY[leg] * np.sin(self.thetas[(self.current_theta_index + INTERPOLATION_STEPS // 2 * (leg == 1 or leg == 2)) % INTERPOLATION_STEPS])
            )
        self.current_theta_index += 1
        self.current_theta_index %= INTERPOLATION_STEPS

        for leg in range(4):
            knee, hip = inverse_kinematics(leg_x[leg], leg_y[leg])
            if hip is None or knee is None:
                print(f"Leg {leg} is out of reach: x={leg_x[leg]}, y={leg_y[leg]}")
            else:
                theta_hip.append(hip)
                theta_knee.append(knee)

        command_values = []
        for leg in range(4):
            command_values.extend([str(int(theta_knee[leg])), str(int(theta_hip[leg])), "0"])
        command_str = "<" + ",".join(command_values) + ">"

        ser.write_line(command_str)

    def set_step_lengthX(self, leftX, rightX):
        self.halfStepLengthsX = [leftX / 2, rightX / 2, leftX / 2, rightX / 2]

    def walk(self, duration=10):
        start_time = datetime.now()
        while (datetime.now() - start_time).total_seconds() < duration:
            self.walk_one_interpolation_step()
            time.sleep(DELAY)


walk_instance = Walk()
walk_instance.reset()
time.sleep(3)  # Wait for the reset command to take effect

# walk_instance.stepLengthsX = [STEP_LENGTH_X / 2, STEP_LENGTH_X / 2 - 10, STEP_LENGTH_X / 2, STEP_LENGTH_X / 2 - 10]  # Step lengths for each leg in X direction

walk_instance.set_step_lengthX(-120, -120)  # walk forward
walk_instance.walk(10)  # Walk for 5 seconds
walk_instance.reset()  # Reset the robot to the initial position

time.sleep(2)  # Wait for the walk to complete

walk_instance.set_step_lengthX(120, 120)  # walk backward
walk_instance.walk(10)  # Walk for 5 seconds
walk_instance.reset()  # Reset the robot to the initial position
time.sleep(2)  # Wait for the walk to complete

walk_instance.set_step_lengthX(80, -80)  # walk left
walk_instance.walk(10)  # Walk for 5 seconds
walk_instance.reset()  # Reset the robot to the initial position
time.sleep(2)  # Wait for the walk to complete

walk_instance.set_step_lengthX(-80, 80)  # walk right
walk_instance.walk(10)  # Walk for 5 seconds
walk_instance.reset()  # Reset the robot to the initial position
