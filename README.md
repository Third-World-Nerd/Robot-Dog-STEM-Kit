# Robot‑Dog STEM Kit (Open Source)

A practical, step‑by‑step quadruped robot project. It includes Fusion 360 CAD, CoppeliaSim simulation, Arduino firmware libraries, and Python tools for walking, stabilization, and tuning.

# Robot-Dog
https://github.com/user-attachments/assets/f82f04b2-c392-450a-b499-d7d90e3e596e

## Table of Contents
- [1. Overview](#overview)
- [2. What’s in This Repo](#whats-in-this-repo)
- [3. Prerequisites](#prerequisites)
- [4. Install & Setup (Windows)](#install--setup-windows)
- [5. Configure COM Ports](#configure-com-ports)
- [6. Run Motion Control](#run-motion-control)
- [7. PID Tuner UI](#pid-tuner-ui)
- [8. Setpoint Tuner UI](#setpoint-tuner-ui)
- [9. Simulation in CoppeliaSim](#simulation-in-coppeliasim)
- [10. CAD: Final Design & Screenshots](#cad-final-design--screenshots)
- [11. Firmware Protocol (Arduino)](#firmware-protocol-arduino)
- [12. Calibration Data](#calibration-data)
- [13. Contributing](#contributing)
- [14. License](#license)
- [15. Troubleshooting](#troubleshooting)

## Overview
Open-source quadruped robot with:
- 3D design: Fusion 360 file `3D design/final_dog_body_design.f3d` (final design)
- Simulation: `CoppeliaSim/Robot_Dog.ttt`
- Control: Python scripts `main.py`, `pidTuner.py`, `setpointTuner.py`
- Firmware libraries (Arduino/C++): `libraries/cpp/…`
- License: MIT (see `LICENSE`)

## What’s in This Repo
- `3D design/` — Fusion 360 models (final: `final_dog_body_design.f3d`)
- `CoppeliaSim/` — Scene `Robot_Dog.ttt`
- `libraries/python/` — control modules: `walk.py`, `pid_controller.py`, `serial_comm.py`, etc.
- `libraries/cpp/` — Arduino libraries: `LegControl`, `MPU9250`, `mpu_ahrs_ekf`, `mpu_madgwick`
- `Utils/python/` — calibration, IK, stabilization, walking, Flask server examples
- Entry points: `main.py` (walk + stabilization), `pidTuner.py` (gain tuning UI), `setpointTuner.py` (setpoint tuning UI)
- Config: `config.py` (COM ports, baud, geometry, gait params)

## Prerequisites
- Windows 10/11 (commands below), Python 3.12+
- Arduino IDE (install Bolder Flight Systems Eigen via Library Manager)
- CoppeliaSim (to open `.ttt` scene)

## Install & Setup (Windows)
1) Clone or download this repo.
2) Create a virtual environment and install dependencies:
```
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```
3) Verify Python tools are found (optional):
```
python --version
pip --version
```

## Configure COM Ports
Edit `config.py` and set the exact serial ports and baud:
- `SERIAL_PORT_SEND` — COM port that receives motor commands from Python (Arduino controlling servos)
- `SERIAL_PORT_REC`  — COM port that streams IMU yaw/pitch/roll from Arduino
- Default baud rates: `BAUD_RATE_SEND = 115200`, `BAUD_RATE_REC = 115200`
Tip: Use Windows Device Manager → Ports (COM & LPT) to find your board’s COM numbers.

## Run Motion Control
1) Connect Arduino and IMU, power servos safely.
2) Ensure your firmware implements the [Firmware Protocol](#firmware-protocol-arduino).
3) Start the controller:
```
python main.py
```
What it does:
- Opens two serial ports: send (motor commands) and receive (IMU)
- Starts a PID loop to compute leg deltas from IMU
- Resets leg positions, triggers a demo feature, then idles

## PID Tuner UI
The UI lets you live-tune Pitch/Roll gains while the robot walks slowly.
Steps:
1) Confirm COM ports in `config.py`.
2) Run:
```
python pidTuner.py
```
3) Use sliders/entries to adjust `Kp`, `Ki`, `Kd` for Pitch and Roll.

![PID Tuner UI](https://github.com/user-attachments/assets/9739330c-3a76-429f-a666-bded4085f7b8)

4) Observe stability and gait; lower gains if oscillations occur.

## Setpoint Tuner UI
Tune desired Pitch/Roll setpoints interactively.
Steps:
1) Confirm COM ports in `config.py`.
2) Run:
```
python setpointTuner.py
```
3) Move sliders or type exact setpoints; observe posture change.

![Setpoint Tuner UI](https://github.com/user-attachments/assets/0250d53e-08e5-4088-9cb0-e5dd8fefd549)

## Simulation in CoppeliaSim
1) Launch CoppeliaSim.
2) Open scene: `CoppeliaSim/Robot_Dog.ttt`.
3) Press Play to run the scene. Use CoppeliaSim tools to inspect kinematics.
Note: Simulation is independent of Arduino; use it for planning and visualization.

## CAD: Final Design & Screenshots

Find the Final Fusion 360 file at `3D design/final_dog_body_design.f3d`

![Top View](3D%20design/images/top_view.png)
*Top view of the robot dog body design*

![Side View](3D%20design/images/front_view.png)
*Side view of the robot dog body design*

## Firmware Protocol (Arduino)
Your firmware should:
- Stream IMU data as: `<yaw,pitch,roll>` at `BAUD_RATE_REC` (example: `<0.5,-1.2,0.3>`)
- Receive motor commands as:
  - Leg angles packet: `<knee0,hip0,0,knee1,hip1,0,knee2,hip2,0,knee3,hip3,0>` (integers)
  - Feature PWM packet (used by `features.pee`): `<motorPWM>` (e.g., `<150>`, and `<0>` to stop)
Arduino Libraries:
- Install Eigen via Arduino Library Manager.
- Link (or copy) each library into Arduino’s `libraries` folder. Example symlink (run CMD as Administrator):
```
mklink /D "C:\Users\<your-user>\Documents\Arduino\libraries\LegControl" "<repo-path>\libraries\cpp\LegControl"
mklink /D "C:\Users\<your-user>\Documents\Arduino\libraries\MPU9250" "<repo-path>\libraries\cpp\MPU9250"
mklink /D "C:\Users\<your-user>\Documents\Arduino\libraries\mpu_ahrs_ekf" "<repo-path>\libraries\cpp\mpu_ahrs_ekf"
mklink /D "C:\Users\<your-user>\Documents\Arduino\libraries\mpu_madgwick" "<repo-path>\libraries\cpp\mpu_madgwick"
```
Upload:
- Open Arduino IDE, select your board and COM port, integrate the protocol above into your sketch, and upload.

## Calibration Data
Sampling frequency: 100 Hz
- Magnetometer:
```
const float magn_ellipsoid_center[3] = {-16.444, 123.131, -60.0703};
const float magn_ellipsoid_transform[3][3] = {{0.887227, 0.0186386, 0.0175599}, {0.0186386, 0.878481, 0.0445495}, {0.0175599, 0.0445495, 0.978254}};
```
- Accelerometer & Gyroscope offsets:
```
#define GYRO_X_OFFSET 0.0003547
#define GYRO_Y_OFFSET 0.0002904
#define GYRO_Z_OFFSET 0.0000442
#define ACCEL_X_OFFSET -1.7587781
#define ACCEL_Y_OFFSET -8.6502850
#define ACCEL_Z_OFFSET -9.2377464
```
- EKF covariance (example):
```
const double Q[3] = {0.0000005924204051, 0.0000007749305595, 0.0000004315204792};
const double R[6] = {0.0001175380701662, 0.0001196885607162, 0.0003229845822837, 0.2635248528243216, 0.2635248528243216, 0.2635248528243216};
```
Reference: https://ahrs.readthedocs.io/en/latest/filters/ekf.html
Note: Values above are examples. Calibrate your own hardware for best results.

## Contributing
- Issues and PRs are welcome. Keep changes focused and documented.
- Include clear steps to reproduce, and tests when possible.
- Follow the project structure and existing style.

## License
MIT License. See `LICENSE`.

## Troubleshooting
- No IMU data? Confirm `SERIAL_PORT_REC` and run: `python -m serial.tools.miniterm <COM> 115200`
- No motion? Verify your firmware decodes command packets and servos are powered.
- Python can’t connect? Close Arduino Serial Monitor (only one program can open a COM port).
- Unstable gait? Lower PID gains in `pidTuner.py`; verify `config.py` geometry values.
