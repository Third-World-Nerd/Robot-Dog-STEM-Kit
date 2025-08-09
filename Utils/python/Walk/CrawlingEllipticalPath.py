import copy
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

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import DELAY
from config import FOOT_POSITIONS_WALK
from config import HIP_KNEE_LENGTH
from config import INTERPOLATION_STEPS_X
from config import INTERPOLATION_STEPS_Y
from config import KNEE_FOOT_LENGTH
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND
from config import STEP_LENGTH_X
from config import STEP_LENGTH_Y

try:
    ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
    time.sleep(2)
except Exception as e:
    ser = None
    print(f"⚠️ Could not connect to motor serial port: {e}")


class WalkingMechanism:

    def __init__(
        self,
        arm1,
        arm2,
        start,
        step_lengthX,
        step_lengthY,
        offset,
        inter_stepsX,
        inter_stepsY,
    ):
        self.arm1 = arm1
        self.arm2 = arm2
        self.x1, self.y1 = start[0], start[1]
        self.step_lengthX = step_lengthX
        self.step_lengthY = step_lengthY
        self.swing_steps = inter_stepsX
        self.stance_steps = inter_stepsY
        self.gait_cycle_len = self.swing_steps + self.stance_steps
        self.offset = offset
        self.trajectory = self.elliptical_path()

    def bezier_path(self):
        """
        Generates a gait trajectory using a 7th-degree Bezier curve for swing,
        followed by a linear stance phase.
        """

        def bezier_n(t, control_points):
            n = len(control_points) - 1
            point = np.zeros(2)
            for i, P in enumerate(control_points):
                bern = comb(n, i) * (t**i) * ((1 - t) ** (n - i))
                point += bern * P
            return point

        # Original 8 control points for 7th-degree Bézier (swing)
        control_points = np.array(
            [[0, 0], [-2, 0.3], [2, 3], [4, 2], [5, 0], [4.5, -0.5], [2, -0.3], [0, 0]]
        )

        # Scale to match desired step length
        scale_x = self.step_lengthX / (
            np.max(control_points[:, 0]) - np.min(control_points[:, 0])
        )
        scale_y = self.step_lengthY / (
            np.max(control_points[:, 1]) - np.min(control_points[:, 1])
        )
        control_points_scaled = np.copy(control_points)
        control_points_scaled[:, 0] *= scale_x
        control_points_scaled[:, 1] *= scale_y

        # Offset to desired start location
        control_points_scaled[:, 0] += self.x1
        control_points_scaled[:, 1] += self.y1

        # Time steps
        ts_swing = np.linspace(0.0, 1.0, self.swing_steps)
        ts_stance = np.linspace(0.0, 1.0, self.stance_steps)

        # Swing trajectory: Bézier curve
        swing_path = np.array([bezier_n(t, control_points_scaled) for t in ts_swing])

        # Stance trajectory: straight line from end of swing to start
        stance_path = np.linspace(swing_path[-1], swing_path[0], self.stance_steps)

        # Combine both phases
        foot_path = np.vstack((swing_path, stance_path))
        x = foot_path[:, 0]
        y = foot_path[:, 1]
        return x, y

    def elliptical_path(self):
        """
        This is for elliptical swing and linear stance
        """
        # swing_theta = np.linspace(np.pi, 0, self.swing_steps)
        # swing_x = self.x1 + (self.step_lengthX / 2) * np.cos(swing_theta)
        # swing_y = self.y1 + self.step_lengthY * np.sin(swing_theta)

        # stance_x = np.linspace(
        #     self.x1 + self.step_lengthX / 2,
        #     self.x1 - self.step_lengthX / 2,
        #     self.stance_steps,
        # )
        # stance_y = np.full(self.stance_steps, self.y1)

        # x = np.concatenate([swing_x, stance_x])
        # y = np.concatenate([swing_y, stance_y])

        # # print(x)

        # return x, y

        #     """
        #     This is for elliptical swing and stance
        #     """

        a = self.step_lengthX / 2
        b = self.step_lengthY

        swing_theta = np.linspace(np.pi, 0, self.swing_steps)

        # print(swing_theta)
        # print(self.x1)
        swing_x = self.x1 + a * np.cos(swing_theta)
        swing_y = self.y1 + b * np.sin(swing_theta)

        # print(swing_x)

        stance_theta = np.linspace(0, -np.pi, self.stance_steps)
        stance_x = self.x1 + a * np.cos(stance_theta)
        stance_y = self.y1 + b * np.sin(stance_theta)

        x = np.concatenate([swing_x, stance_x])
        y = np.concatenate([swing_y, stance_y])

        return x, y

    def ik(self, x, y, pX=0, pY=0):
        # print(f"X : {x}, Y : {y}")
        d = np.hypot(x, y)
        if d > (self.arm1 + self.arm2) or d < abs(self.arm1 - self.arm2):
            return None

        cos_theta2 = (x**2 + y**2 - self.arm1**2 - self.arm2**2) / (
            2 * self.arm1 * self.arm2
        )
        # if abs(cos_theta2) > 1:
        #     return None
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        # if not elbow_down:
        sin_theta2 = -sin_theta2

        theta2 = np.arctan2(sin_theta2, cos_theta2)
        k1 = self.arm1 + self.arm2 * cos_theta2
        k2 = self.arm2 * sin_theta2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        theta1 = int(np.degrees(theta1))
        theta2 = int(180 + np.degrees(theta2))

        return theta1, theta1 + theta2


class LegMovement:

    def __init__(self, legs, fp, steps):
        self.legs = legs
        self.foot_positions = fp
        self.steps = steps * 2
        self.generate_angles()
        # self.txt = ""
        # print(type(self.angles))
        # exit()

    def generate_angles(self):
        self.angles = []

        for steps in range(self.steps):
            # calcStart = time.time()
            command = [None] * 4

            current_leg = steps
            leg_trajectories = [leg.trajectory for leg in self.legs]
            # print(leg_trajectories)
            leg_positions = []
            for i, (x_path, y_path) in enumerate(leg_trajectories):
                idx = (
                    steps + self.legs[i].offset * self.legs[i].swing_steps
                ) % self.legs[i].gait_cycle_len

                xP, yP = x_path[idx], y_path[idx]
                leg_positions.append((xP, yP))

            for i in range(4):
                x, y = leg_positions[i]
                print(x, y)
                a, b = self.legs[i].ik(x, y)
                command[i] = [f"{int(b)}", f"{int(a)}", "0"]

            command_flat = [angle for leg_angles in command for angle in leg_angles]
            command = f"<{','.join(command_flat)}>"
            self.angles.append(command)

        # print(type(self.angles))
        # self.txt = f"{self.angles}"
        # with open("angles.txt", 'w') as f:
        #     f.write(self.txt)
        # print(self.angles)

        # print(command_flat)
        # print(command)
        # return self.angles
        # print(f"{command}")

    def start(self, delay):
        command = [None] * 4

        for i in range(4):
            x, y = self.foot_positions[i]
            a, b = self.legs[i].ik(x, y)
            command[i] = [f"{int(b)}", f"{int(a)}", "0"]

        # print(command)
        command_flat = [angle for leg_angles in command for angle in leg_angles]
        command = f"<{','.join(command_flat)}>"
        # command = f"<99,-50,0,99,-50,0,-120,-129,0,-129,-129,0>"
        # print(f"{command}")

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

        self.generate_angles()

        time.sleep(4)

        steps = 0

        # print()
        while True:
            calcStart = time.time()
            command = [None] * 4

            command = self.angles[steps % self.steps]
            print(time.time() - calcStart)

            # current_leg = steps
            # leg_trajectories = [leg.trajectory for leg in self.legs]
            # # print(leg_trajectories)
            # leg_positions = []
            # for i, (x_path, y_path) in enumerate(leg_trajectories):
            #     idx = (steps + self.legs[i].offset * self.legs[i].swing_steps) % self.legs[i].gait_cycle_len

            #     xP, yP = x_path[idx], y_path[idx]
            #     leg_positions.append((xP, yP))

            # for i in range(4):
            #     x, y = leg_positions[i]
            #     a, b = self.legs[i].ik(x, y)
            #     command[i] = [f"{int(b)}", f"{int(a)}", "0"]

            # command_flat = [angle for leg_angles in command for angle in leg_angles]
            # command = f"<{','.join(command_flat)}>"
            # print(f"{command}")

            if ser and ser.is_open:
                try:
                    command = command.encode()
                    print(time.time() - calcStart)
                    ser.write_line(command)
                    print(time.time() - calcStart)
                    now = datetime.now()
                    minute = now.strftime("%M")
                    second = now.strftime("%S")
                    millisecond = int(now.microsecond / 1000)
                    print(f"Sent: {minute}:{second}:{millisecond}:- {command}")
                except Exception as e:
                    print(f"Serial error: {e}")
            else:

                print(f"Error: Serial port {SERIAL_PORT_SEND} not available")
            time.sleep(0.01)
            print(f"Time taken : {time.time() - calcStart}")
            # time.sleep(0.01)
            steps += 1
            # if steps % 24 == 0:
            #     exit()


wm = [
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[0],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        0,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[1],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        1,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),  # 2
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[2],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        1,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),  # 1
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[3],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        0,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
]


Lm = LegMovement(wm, FOOT_POSITIONS_WALK, INTERPOLATION_STEPS_X)
Lm.start(DELAY)
