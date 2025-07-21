# config.py

# Serial communication settings
SERIAL_PORT_REC = "/dev/ttyACM0"  # or "/dev/ttyUSB0" for Linux/macOS
SERIAL_PORT_SEND = "/dev/ttyUSB0"  # or "/dev/ttyUSB1" for Linux/macOS
# Baud rates for receiving and sending data
BAUD_RATE_REC = 2000000  # Baud rate for receiving MPU data from Arduino/Teensy
BAUD_RATE_SEND = 115200  # Baud rate for sending Motor Angle commands to Arduino

# ==== Robot Leg Parameters ====

NUM_LEGS = 4
NUM_JOINTS = 3

JOINT_NAMES = ["knee", "Hip-Leg", "Hip-Body"]
JOINT_RANGES = [(70, 180), (-150, 30), (-20, 20)]
# Joint angles initial position
JOINT_ANGLES_INIT = [
    (JOINT_RANGES[i][1] + JOINT_RANGES[i][0]) / 2 for i in range(NUM_JOINTS)
]

HIP_KNEE_LENGTH = 95  # Hip to knee
KNEE_FOOT_LENGTH = 100  # Knee to foot

INTERPOLATION_STEPS_X = 12
INTERPOLATION_STEPS_Y = 12


a = 20
b = 50
c = -195
d = -175


FOOT_POSITIONS_REST = [[-95, -195], [-95, -195], [95, -195], [95, -195]]    
# FOOT_POSITIONS_WALK = [[a, c-10], [a, d-10], [b, c-10], [b, d-10]]
FOOT_POSITIONS_WALK = [[0, -162], [0, -160], [0, -162], [0, -160]]
# FOOT_POSITIONS_WALK = [[95, -100]] * 4
STEP_LENGTH_X = -40  # Step length in X direction
STEP_LENGTH_Y = 20 # Step length in Y direction

DELAY = 0.03


