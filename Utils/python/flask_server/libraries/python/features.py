import os
import sys
import time

from serial_comm import SerialComm

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../.."))
sys.path.append(config_dir)

from walk import Walk

from config import FOOT_POSITIONS_PEE


def pee(motorPWM: int, duration: float, ser_write_command: SerialComm):
    walk = Walk(ser_write_command)
    walk.reset(foot_positions=FOOT_POSITIONS_PEE)
    time.sleep(1)
    ser_write_command.write_line(f"<{motorPWM}>")
    time.sleep(duration)
    ser_write_command.write_line("<0>")
    walk.reset()
