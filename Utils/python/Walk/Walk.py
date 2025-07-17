import copy
import os
import sys
import time

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
from kinematics import inverse_kinematics
from pid_leg_stabilizer import PIDLegStabilizer
from serial_comm import SerialComm

def pid_loop(foot_positions, foot_positions_lock, pitch_roll_setpoint, pitch_roll_setpoint_lock, swinging_legs, swinging_legs_lock):


    try:
        imu_uc = SerialComm(SERIAL_PORT_REC, BAUD_RATE_REC, timeout=0.01)
        time.sleep(2)
    except Exception as e:
        imu_uc = None
        print(f"⚠️ Could not connect to IMU serial port: {e}")

    try:
        motor_uc = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
        time.sleep(2)
    except Exception as e:
        motor_uc = None
        print(f"⚠️ Could not connect to motor serial port: {e}")

    pid_leg = PIDLegStabilizer(
        kp_pitch=0.4,
        ki_pitch=0.04,
        kd_pitch=0.04,
        kp_roll=0.1,
        ki_roll=0.01,
        kd_roll=0.01,
        integral_window=100,
    )
    with pitch_roll_setpoint_lock:
        pid_leg.set_setpoint(pitch_roll_setpoint[0], pitch_roll_setpoint[1])

    sample_freq = 25.0
    loop_counter = 0
    last_print_time = time.time()

    # Initial stand-up position
    angle_values = []
    for leg in range(NUM_LEGS):
        foot_positions[leg][0] = FOOT_POSITIONS_WALK[leg][0]
        foot_positions[leg][1] = FOOT_POSITIONS_WALK[leg][1]
        knee, hip = inverse_kinematics(foot_positions[leg][0], foot_positions[leg][1])
        angle_values.extend([str(int(knee)), str(int(hip)), "0"])

    try:
        command = "<" + ",".join(angle_values) + ">"
        motor_uc.write_line(command)
        print(f"[INIT] Sent walking position: {command}")
    except Exception as e:
        print(f"[ERROR] Send failed: {e}")
    time.sleep(1)

    def read_imu():
        try:
            imu_uc.ser.reset_input_buffer()
            time.sleep(0.001)
            ypr_data = imu_uc.read_line().strip()
            if ypr_data.startswith("<") and ypr_data.endswith(">"):
                yaw, pitch, roll = map(float, ypr_data[1:-1].split(","))
                return yaw, pitch, roll
        except:
            pass
        return None, None, None

    while True:
        loop_start = time.perf_counter()

        yaw, pitch, roll = read_imu()
        if pitch is None or roll is None:
            continue
        print(f"[IMU] Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}")

        # For testing without motion:
        # pitch = 0.0
        # roll = 0.0
        with pitch_roll_setpoint_lock:
            pid_leg.set_setpoint(pitch_roll_setpoint[0], pitch_roll_setpoint[1])
        deltas = pid_leg.compute_leg_deltas(pitch, roll, 1 / sample_freq)
        angle_values = []
        ik_failed = False

        with foot_positions_lock:
            with swinging_legs_lock:
                current_swinging_legs = swinging_legs[:]

            for leg in range(NUM_LEGS):
                if not current_swinging_legs[leg]:
                    foot_positions[leg][1] += deltas[leg]
            foot_positions_local = copy.deepcopy(foot_positions)

        # Inverse Kinematics
        for leg in range(NUM_LEGS):
            try:
                knee, hip = inverse_kinematics(
                    foot_positions_local[leg][0], foot_positions_local[leg][1]
                )
                if knee is None or hip is None:
                    raise ValueError("IK failed")
                angle_values.extend([str(int(knee)), str(int(hip)), "0"])
            except Exception as e:
                print(f"[WARN] IK leg {leg+1} failed: {e}")
                ik_failed = True
                break

        if not ik_failed:
            try:
                command = "<" + ",".join(angle_values) + ">"
                motor_uc.write_line(command)
                # print(f"[SEND] {command}")
                None
            except Exception as e:
                print(f"[WARN] Failed to send angles: {e}")

        # FPS Monitor
        loop_counter += 1
        now = time.time()
        if now - last_print_time >= 1.0:
            print(f"[PID] Running at {loop_counter / (now - last_print_time):.2f} Hz")
            loop_counter = 0
            last_print_time = now

        # Maintain frequency
        loop_elapsed = time.perf_counter() - loop_start
        time.sleep(max(0, (1 / sample_freq) - loop_elapsed))


# def walk(foot_positions, foot_lock):
#     delay = 3.0
#     steps = [
#         ([0], 0, STEP_LENGTH_Y, pitch, roll),
#         ([0], -STEP_LENGTH_X, 0),
#         ([0], 0, -STEP_LENGTH_Y),
#         ([1], 0, STEP_LENGTH_Y),
#         ([1], -STEP_LENGTH_X, 0),
#         ([1], 0, -STEP_LENGTH_Y),
#         ([0, 1, 2, 3], STEP_LENGTH_X, 0),
#         ([2], 0, STEP_LENGTH_Y),
#         ([2], -STEP_LENGTH_X, 0),
#         ([2], 0, -STEP_LENGTH_Y),
#         ([3], 0, STEP_LENGTH_Y),
#         ([3], -STEP_LENGTH_X, 0),
#         ([3], 0, -STEP_LENGTH_Y),
#     ]
#     for legs, dx, dy in steps:
#         with foot_lock:
#             for leg in legs:
#                 foot_positions[leg][0] += dx
#                 foot_positions[leg][1] += dy
#         time.sleep(delay)


def set_swing_legs(swing_legs, swing_legs_lock, swing_leg_indices):
    with swing_legs_lock:
        for i in range(len(swing_legs)):
            swing_legs[i] = i in swing_leg_indices


def walk(foot_positions, legs, dx, dy, foot_lock):
    with foot_lock:
        for leg in legs:
            foot_positions[leg][0] += dx
            foot_positions[leg][1] += dy

def tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, pitch_setpoint, roll_setpoint):
    with pitch_roll_setpoint_lock:
        pitch_roll_setpoint[0] = pitch_setpoint
        pitch_roll_setpoint[1] = roll_setpoint

def maw_walk(foot_positions, leg , dx, dy):


import threading

if __name__ == "__main__":
    foot_positions = [list(pos) for pos in FOOT_POSITIONS_WALK]
    pitch_roll_setpoint = [0.0, 0.0]
    foot_positions_lock = threading.Lock()
    pitch_roll_setpoint_lock = threading.Lock()
    swinging_legs = [False for _ in range(NUM_LEGS)]
    swinging_legs_lock = threading.Lock()


    # Start PID loop in a separate thread
    pid_thread = threading.Thread(
        target=pid_loop, args=(foot_positions, foot_positions_lock, pitch_roll_setpoint, pitch_roll_setpoint_lock, swinging_legs, swinging_legs_lock)
    )
    pid_thread.daemon = True  # Thread exits when main thread exits
    pid_thread.start()



    tilt_angle = 5
    delay = 3
    walk_delay = 1

    time.sleep(5)


    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, tilt_angle, -tilt_angle)
    # time.sleep(delay)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [0])  # If leg 0 is swinging
    walk(foot_positions, [0], 0, STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [0], -STEP_LENGTH_X, 0,foot_positions_lock)
    walk(foot_positions, [1,2,3], STEP_LENGTH_X/4, 0,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [0], 0, -STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, 0, 0)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [])
    time.sleep(delay)

    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, -tilt_angle, +tilt_angle)
    # time.sleep(delay)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [3])  # If leg 3 is swinging
    walk(foot_positions, [3], 0, STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [3], -STEP_LENGTH_X, 0,foot_positions_lock)
    walk(foot_positions, [0,1,2], STEP_LENGTH_X/4, 0,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [3], 0, -STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, 0, 0)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [])
    time.sleep(delay)

    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, +tilt_angle, +tilt_angle)   #HAVE TO CHECK CONFUSED  
    # time.sleep(delay)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [1])  # If leg 1 is swinging
    walk(foot_positions, [1], 0, STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [1], -STEP_LENGTH_X, 0,foot_positions_lock)
    walk(foot_positions, [0,2,3], STEP_LENGTH_X/4, 0,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [1], 0, -STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, 0, 0)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [])
    time.sleep(delay)


    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, -tilt_angle, -tilt_angle)
    # time.sleep(delay)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [2])  # If leg 2 is swinging
    walk(foot_positions, [2], 0, STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [2], -STEP_LENGTH_X, 0,foot_positions_lock)
    walk(foot_positions, [0,1,3], STEP_LENGTH_X/4, 0,foot_positions_lock)
    time.sleep(walk_delay)
    walk(foot_positions, [2], 0, -STEP_LENGTH_Y,foot_positions_lock)
    time.sleep(walk_delay)self
    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, 0, 0)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [])
    # time.sleep(delay)


    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, -tilt_angle, -tilt_angle)
    # time.sleep(delay)
    # set_swing_legs(swinging_legs, swinging_legs_lock, [2])  # If leg 0 is swinging
    # walk(foot_positions, [2], 0, STEP_LENGTH_Y,foot_positions_lock)
    # # time.sleep(walk_delay)
    # walk(foot_positions, [2], STEP_LENGTH_X, 0,foot_positions_lock)
    # walk(foot_positions, [0,1,3], -STEP_LENGTH_X/4, 0,foot_positions_lock)
    # time.sleep(walk_delay)
    # walk(foot_positions, [2], 0, -STEP_LENGTH_Y,foot_positions_lock)
    # time.sleep(walk_delay)
    # tilt_body(pitch_roll_setpoint, pitch_roll_setpoint_lock, 0, 0)
    # time.sleep(delay)

    time.sleep(10)

    # # Now you can walk while the PID thread is running
    # for _ in range(3):
    #     walk(foot_positions, foot_positions_lock)
    #     time.sleep(1)

    # Main thread exits → PID thread exits too
