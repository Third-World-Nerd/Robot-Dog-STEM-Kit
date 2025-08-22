from collections import deque

from serial_comm import SerialComm


class PIDController:

    def __init__(self, ser_read_imu: SerialComm):
        self._ser_read_imu = ser_read_imu

        self._kp_yaw = 0.0
        self._ki_yaw = 0.0
        self._kd_yaw = 0.0

        self._kp_pitch = 0.4
        self._ki_pitch = 0.04
        self._kd_pitch = 0.04

        self._kp_roll = 0.1
        self._ki_roll = 0.01
        self._kd_roll = 0.01

        self._dt = 1 / 25
        self._prev_error = 0.0
        self._integral = 0.0

        self._setpoint_yaw = 0.0
        self._setpoint_pitch = 0.0
        self._setpoint_roll = 0.0

        self._yaw_error_history = deque(maxlen=100)
        self._pitch_error_history = deque(maxlen=100)
        self._roll_error_history = deque(maxlen=100)

        # append initial previous error for each axis to compute derivatives
        self._yaw_error_history.append(0)
        self._pitch_error_history.append(0)
        self._roll_error_history.append(0)

    def set_dt(self, dt):
        self._dt = dt

    def get_dt(self):
        return self._dt

    def set_setpoint_yaw(self, setpoint):
        self._setpoint_yaw = setpoint

    def set_setpoint_pitch(self, setpoint):
        self._setpoint_pitch = setpoint

    def set_setpoint_roll(self, setpoint):
        self._setpoint_roll = setpoint

    def set_gains_yaw(self, kp, ki, kd):
        self.kp_yaw = kp
        self.ki_yaw = ki
        self.kd_yaw = kd

    def set_gains_pitch(self, kp, ki, kd):
        self.kp_pitch = kp
        self.ki_pitch = ki
        self.kd_pitch = kd

    def set_gains_roll(self, kp, ki, kd):
        self.kp_roll = kp
        self.ki_roll = ki
        self.kd_roll = kd

    def compute_leg_deltas(self):
        yaw, pitch, roll = self._ser_read_imu.read_imu()

        if yaw is None or pitch is None or roll is None:
            return [[0, 0], [0, 0], [0, 0], [0, 0]]

        yaw_error = self._setpoint_yaw - yaw
        pitch_error = self._setpoint_pitch - pitch
        roll_error = self._setpoint_roll - roll

        yaw_integral = sum(self._yaw_error_history) + yaw_error
        pitch_integral = sum(self._pitch_error_history) + pitch_error
        roll_integral = sum(self._roll_error_history) + roll_error

        # get the prev error from the history deque
        yaw_derivative = (yaw_error - self._yaw_error_history[-1]) / self._dt
        pitch_derivative = (pitch_error - self._pitch_error_history[-1]) / self._dt
        roll_derivative = (roll_error - self._roll_error_history[-1]) / self._dt

        self._yaw_error_history.append(yaw_error)
        self._pitch_error_history.append(pitch_error)
        self._roll_error_history.append(roll_error)

        yaw_output = self._kp_yaw * yaw_error + self._ki_yaw * yaw_integral * self._dt + self._kd_yaw * yaw_derivative
        pitch_output = self._kp_pitch * pitch_error + self._ki_pitch * pitch_integral * self._dt + self._kd_pitch * pitch_derivative
        roll_output = self._kp_roll * roll_error + self._ki_roll * roll_integral * self._dt + self._kd_roll * roll_derivative

        delta_y_lf = -pitch_output + roll_output
        delta_y_rf = -pitch_output - roll_output
        delta_y_lb = +pitch_output + roll_output
        delta_y_rb = +pitch_output - roll_output

        delta_x_l = -yaw_output
        delta_x_r = yaw_output

        return [[delta_x_l, delta_y_lf], [delta_x_r, delta_y_rf], [delta_x_l, delta_y_lb], [delta_x_r, delta_y_rb]]
        return [[delta_x_l, delta_y_lf], [delta_x_r, delta_y_rf], [delta_x_l, delta_y_lb], [delta_x_r, delta_y_rb]]
