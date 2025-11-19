# config.py

# # Serial communication settings
# SERIAL_PORT_REC = "/dev/ttyACM0"  # or "/dev/ttyUSB0" for Linux/macOS
# SERIAL_PORT_SEND = "/dev/ttyUSB0"#"COM18"  # or "/dev/ttyUSB1" for Linux/macOS
# # Baud rates for receiving ansending data
# BAUD_RATE_REC = 2000000  # Baud rate for receiving MPU data from Arduino/Teensy
# BAUD_RATE_SEND = 115200  # Baud rate for sending Motor Angle commands to Arduino

SERIAL_PORT_REC = "COM13"  # or "/dev/ttyUSB0" for Linux/macOS
SERIAL_PORT_SEND = "COM25"  # or "/dev/ttyUSB1" for Linux/macOS
# Baud rates for receiving and sending data
BAUD_RATE_REC = 115200  # Baud rate for receiving MPU data from Arduino/Teensy
BAUD_RATE_SEND = 115200  # Baud rate for sending Motor Angle commands to Arduino

# ==== Robot Leg Parameters ====

NUM_LEGS = 4
NUM_JOINTS = 3

JOINT_NAMES = ["knee", "Hip-Leg", "Hip-Body"]
JOINT_RANGES = [(70, 180), (-150, 30), (-20, 20)]
# Joint angles initial position
JOINT_ANGLES_INIT = [(JOINT_RANGES[i][1] + JOINT_RANGES[i][0]) / 2 for i in range(NUM_JOINTS)]

HIP_KNEE_LENGTH = 95  # Hip to knee
KNEE_FOOT_LENGTH = 100  # Knee to foot

INTERPOLATION_STEPS = 16  # even number

FOOT_POSITIONS_REST = [[-95, -195], [-95, -195], [95, -195], [95, -195]]


FOOT_POSITIONS_WALK = [[0, -165], [0, -160], [10, -173], [10, -170]]

FOOT_POSITIONS_PEE = [[0, -120], [0, -190], [10, -150], [100, -70]]

STEP_LENGTH_X = -120  # even number
STEP_LENGTH_Y = 10  # even number

DELAY = 0.025

PID_SAMPLE_FREQUENCY = 25  # Hz
