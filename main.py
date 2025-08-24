import os
import sys
import threading
import time

import numpy as np

# Add paths
libraries_dir = os.path.abspath(os.path.join(__file__, "../libraries/python"))
sys.path.append(libraries_dir)

import threading
import time

from features import pee
from pid_controller import PIDController
from serial_comm import SerialComm
from walk import Walk

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND


def pid_loop(pid, leg_deltas, leg_deltas_lock):
    while True:
        loop_start = time.perf_counter()
        with leg_deltas_lock:
            new_deltas = pid.compute_leg_deltas()
            for i in range(4):
                leg_deltas[i] = new_deltas[i]
        loop_elapsed = time.perf_counter() - loop_start
        time.sleep(max(0, pid.get_dt() - loop_elapsed))


def main():
    ser_write_command = SerialComm(port=SERIAL_PORT_SEND, baudrate=BAUD_RATE_SEND)
    ser_read_imu = SerialComm(port=SERIAL_PORT_REC, baudrate=BAUD_RATE_REC)
    pid = PIDController(ser_read_imu)
    walk_instance = Walk(ser_write_command)

    leg_deltas = [[0, 0] for _ in range(4)]
    leg_deltas_lock = threading.Lock()

    threading.Thread(target=pid_loop, args=(pid, leg_deltas, leg_deltas_lock), daemon=True).start()

    walk_instance.reset_dynamic(leg_deltas, duration=5, update_foot_position_walk=True)

    walk_instance.reset()
    # time.sleep(5)

    # walk_instance.set_step_lengthX(-120.0, -120.0)  # forward
    # walk_instance.set_step_lengthY(20)  # forward
    # walk_instance.walk(duration=20)

    pee(motorPWM=150, duration=3, ser_write_command=ser_write_command)

    # walk_instance.reset()


if __name__ == "__main__":
    main()
