import os
import sys
import time

import numpy as np
from kinematics import inverse_kinematics
from serial_comm import SerialComm

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../.."))
sys.path.append(config_dir)

from config import DELAY
from config import FOOT_POSITIONS_WALK
from config import INTERPOLATION_STEPS
from config import STEP_LENGTH_X
from config import STEP_LENGTH_Y


class Walk:
    def __init__(self, ser_write_command: SerialComm):
        self._ser_write_command = ser_write_command
        self.current_theta_index = 0  # current index in the theta array for the first (leg 0) leg
        swing_theta = np.linspace(np.pi, 0, INTERPOLATION_STEPS // 2)
        stance_theta = np.linspace(0, -np.pi, INTERPOLATION_STEPS // 2)
        self.thetas = np.concatenate((swing_theta, stance_theta))
        self.halfStepLengthsX = [STEP_LENGTH_X / 2] * 4  # Step lengths for each leg in X direction
        self.halfStepLengthsY = [STEP_LENGTH_Y / 2] * 4  # Step lengths for each leg in Y direction
        self.foot_positions = FOOT_POSITIONS_WALK
        self.should_stop=True

    def reset(self, leg_deltas=[[0, 0], [0, 0], [0, 0], [0, 0]], foot_positions=None):
        self.should_stop=False
        if foot_positions is None:
            foot_positions = self.foot_positions
        theta_knee = []
        theta_hip = []
        new_leg_positions = []
        for leg in range(4):
            new_leg_positions.append([foot_positions[leg][0] + leg_deltas[leg][0], foot_positions[leg][1] + leg_deltas[leg][1]])
            knee, hip = inverse_kinematics(new_leg_positions[leg][0], new_leg_positions[leg][1])
            if hip is None or knee is None:
                print(f"Leg {leg} is out of reach: x={foot_positions[leg][0]}, y={foot_positions[leg][1]}")
                return
            else:
                theta_knee.append(knee)
                theta_hip.append(hip)
        command_values = []
        for leg in range(4):
            command_values.extend([str(int(theta_knee[leg])), str(int(theta_hip[leg])), "0"])
        command_str = "<" + ",".join(command_values) + ">"
        self._ser_write_command.write_line(command_str)
        return new_leg_positions

    def walk_one_interpolation_step(self, leg_deltas):
        leg_x = []
        leg_y = []
        theta_hip = []
        theta_knee = []
        for leg in range(4):
            leg_x.append(
                self.foot_positions[leg][0]
                + self.halfStepLengthsX[leg] * np.cos(self.thetas[(self.current_theta_index + INTERPOLATION_STEPS // 2 * (leg == 1 or leg == 2)) % INTERPOLATION_STEPS])
                + leg_deltas[leg][0]
            )
            leg_y.append(
                self.foot_positions[leg][1]
                + self.halfStepLengthsY[leg] * np.sin(self.thetas[(self.current_theta_index + INTERPOLATION_STEPS // 2 * (leg == 1 or leg == 2)) % INTERPOLATION_STEPS])
                + leg_deltas[leg][1]
            )
        self.current_theta_index += 1
        self.current_theta_index %= INTERPOLATION_STEPS

        for leg in range(4):
            knee, hip = inverse_kinematics(leg_x[leg], leg_y[leg])
            if hip is None or knee is None:
                print(f"Leg {leg} is out of reach: x={leg_x[leg]}, y={leg_y[leg]}")
                return
            else:
                theta_hip.append(hip)
                theta_knee.append(knee)

        command_values = []
        for leg in range(4):
            command_values.extend([str(int(theta_knee[leg])), str(int(theta_hip[leg])), "0"])
        command_str = "<" + ",".join(command_values) + ">"

        self._ser_write_command.write_line(command_str)

    def set_step_lengthX(self, leftX, rightX):
        self.halfStepLengthsX = [leftX / 2, rightX / 2, leftX / 2, rightX / 2]

    def set_step_lengthY(self, stepLength):
        self.halfStepLengthsY = [stepLength / 2] * 4

    def walk(self, leg_deltas=[[0, 0], [0, 0], [0, 0], [0, 0]], duration=10):
        start_time = time.perf_counter()
        while (time.perf_counter() - start_time) < duration:
            start_time_2 = time.perf_counter()
            self.walk_one_interpolation_step(leg_deltas)
            time.sleep(max(0, DELAY - (time.perf_counter() - start_time_2)))

    def reset_dynamic(self, leg_deltas, duration=5, update_foot_position_walk=False):
        start_time = time.perf_counter()
        new_leg_positions = []
        while (time.perf_counter() - start_time) < duration and not self.should_stop:
            start_time_2 = time.perf_counter()
            new_leg_positions = self.reset(leg_deltas)
            time.sleep(max(0, DELAY - (time.perf_counter() - start_time_2)))

        if update_foot_position_walk:
            self.foot_positions = new_leg_positions

    def stop(self):
        self.should_stop=True
