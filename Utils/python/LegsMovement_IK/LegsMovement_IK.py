import os
import sys
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.widgets import Button
from matplotlib.widgets import CheckButtons
from matplotlib.widgets import Slider

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)

# Import the config module
from config import BAUD_RATE_SEND
from config import SERIAL_PORT_SEND

try:
    ser = serial.Serial(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=1)
    time.sleep(2)
except Exception as e:
    ser = None
    print(f"⚠️ Could not connect to serial port: {e}")

from config import HIP_KNEE_LENGTH
from config import JOINT_ANGLES_INIT
from config import JOINT_NAMES
from config import JOINT_RANGES
from config import KNEE_FOOT_LENGTH
from config import NUM_JOINTS
from config import NUM_LEGS


# Compute offset for (0, 0) at midpoint angles
def forward_kinematics(knee_joint_angle, hip_leg_joint_angle, apply_offset=False):
    theta1 = np.radians(hip_leg_joint_angle)  # Hip angle from X-axis
    theta2 = np.radians(knee_joint_angle)  # Inner knee angle

    # Position of knee
    x1 = HIP_KNEE_LENGTH * np.cos(theta1)
    y1 = HIP_KNEE_LENGTH * np.sin(theta1)

    # Angle of the lower leg relative to horizontal
    theta_lower = theta1 + theta2 - np.pi

    # Position of foot
    x2 = x1 + KNEE_FOOT_LENGTH * np.cos(theta_lower)
    y2 = y1 + KNEE_FOOT_LENGTH * np.sin(theta_lower)

    if apply_offset:
        x2 -= offset_x
        y2 -= offset_y

    return x1, y1, x2, y2


# Calculate offset based on midpoint angles
# offset_x, offset_y = forward_kinematics(
#     JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1], apply_offset=False
# )

offset_x = 0
offset_y = 0

# ==== Set up Plot ====
fig, axes = plt.subplots(2, 2, figsize=(14, 12))
plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.35, wspace=0.2, hspace=0.2)

# Storage for plot elements and sliders
lines = [[None for _ in range(2)] for _ in range(2)]
target_dots = [[None for _ in range(2)] for _ in range(2)]
sliders = [[None for _ in range(4)] for _ in range(NUM_LEGS)]  # Knee, Hip, X, Y per leg
buttons = [None for _ in range(NUM_LEGS)]  # Reset buttons per leg
dragging = [False for _ in range(NUM_LEGS)]
ignore_slider_callback = [False for _ in range(NUM_LEGS)]

# ==== Initialize Subplots ====
for leg in range(NUM_LEGS):
    row = leg // 2
    col = leg % 2
    ax = axes[row, col]
    ax.set_aspect("equal")
    ax.set_xlim(-250 - offset_x, 250 - offset_x)  # Adjust plot limits for offset
    ax.set_ylim(-400 - offset_y, 100 - offset_y)
    ax.set_title(f"Leg {leg+1}")
    (lines[row][col],) = ax.plot([], [], "o-", lw=4, markersize=8, color="blue")
    (target_dots[row][col],) = ax.plot([], [], "rx", markersize=10)


# ==== Draw Robot Leg ====
def update_leg(leg, knee_joint_angle, hip_leg_joint_angle):
    x0, y0 = 0, 0

    x1, y1, x2, y2 = forward_kinematics(knee_joint_angle, hip_leg_joint_angle, apply_offset=True)

    row = leg // 2
    col = leg % 2

    # Apply offset for display
    x2_display = x2 - offset_x
    y2_display = y2 - offset_y

    lines[row][col].set_data([x0, x1, x2], [y0, y1, y2])
    target_dots[row][col].set_data([x2_display], [y2_display])
    fig.canvas.draw_idle()


# ==== Send Angles to Arduino ====
# ==== Send Angles to Arduino ====
def send_all_angles():
    if any(ignore_slider_callback):
        return
    values = []
    for leg in range(NUM_LEGS):
        try:
            theta_knee = sliders[leg][0].val  # Knee
            theta_hip = sliders[leg][1].val  # Hip-Leg
            values.extend([str(int(theta_knee + theta_hip)), str(int(theta_hip)), "0"])  # Hip-Body=0
        except Exception as e:
            print(f"Error processing angles for Leg {leg+1}: {e}")
            values.extend(["0", "0", "0"])

    # Format: <val1,val2,val3,...>
    command = "<" + ",".join(values) + ">"
    print(command)
    if ser and ser.is_open:
        try:
            ser.write(command.encode())
            now = datetime.now()
            minute = now.strftime("%M")
            second = now.strftime("%S")
            millisecond = int(now.microsecond / 1000)
            print(f"Sent: {minute}:{second}:{millisecond}:- {command}")
        except Exception as e:
            print(f"Serial error: {e}")
    else:
        print(f"Error: Serial port {SERIAL_PORT_SEND} not available")


sync_states = [False for _ in range(NUM_LEGS)]  # Track which legs are synced


def update_synced_legs(theta_knee_deg, theta_hip_deg, x, y):
    global ignore_slider_callback
    synced_legs = [i for i in range(NUM_LEGS) if sync_states[i]]
    for i in synced_legs:
        ignore_slider_callback[i] = True
        sliders[i][0].set_val(theta_knee_deg)
        sliders[i][1].set_val(theta_hip_deg)
        sliders[i][2].set_val(x)
        sliders[i][3].set_val(y)
        ignore_slider_callback[i] = False
        update_leg(i, theta_knee_deg, theta_hip_deg)


# ==== Inverse Kinematics ====
def inverse_kinematics(x, y):
    # Convert display coordinates back to original coordinates
    x_orig = x + offset_x
    y_orig = y + offset_y

    # Distance from origin to target
    L = np.hypot(x_orig, y_orig)

    # Check reachability
    if L > HIP_KNEE_LENGTH + KNEE_FOOT_LENGTH or L < abs(HIP_KNEE_LENGTH - KNEE_FOOT_LENGTH):
        return None, None

    # Law of cosines for knee angle (inner angle)
    cos_knee = (HIP_KNEE_LENGTH**2 + KNEE_FOOT_LENGTH**2 - L**2) / (2 * HIP_KNEE_LENGTH * KNEE_FOOT_LENGTH)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    theta_knee = np.arccos(cos_knee)

    # Law of cosines for angle between thigh and target line
    cos_phi = (HIP_KNEE_LENGTH**2 + L**2 - KNEE_FOOT_LENGTH**2) / (2 * HIP_KNEE_LENGTH * L)
    cos_phi = np.clip(cos_phi, -1.0, 1.0)
    phi = np.arccos(cos_phi)
    # Angle from knee to target point
    angle_to_knee = np.arctan2(y_orig, x_orig) - theta_knee

    # Angle from origin to target point
    angle_to_target = np.arctan2(y_orig, x_orig)

    # Hip angle is angle to target minus φ
    theta_hip = angle_to_target + phi

    # Convert to degrees
    theta_knee_deg = np.degrees(theta_knee)
    theta_hip_deg = np.degrees(theta_hip)

    return theta_knee_deg, theta_hip_deg


# ==== Slider and Mouse Handlers ====
def on_slider_change(leg):
    def handler(val):
        if ignore_slider_callback[leg]:
            return
        t1 = sliders[leg][0].val  # Knee
        t2 = sliders[leg][1].val  # Hip
        x = sliders[leg][2].val  # Target X (display coordinates)
        y = sliders[leg][3].val  # Target Y (display coordinates)
        # Update leg plot
        leg_not_synced = False
        if not sync_states[leg]:
            sync_states[leg] = True  # Enable sync for this leg
            leg_not_synced = True

            update_synced_legs(t1, t2, x, y)

        if leg_not_synced:
            sync_states[leg] = False  # Reset sync state after update

        # update_leg(leg, t1, t2)
        # Compute forward kinematics to get new X, Y (with offset)
        x1, y1, x2, y2 = forward_kinematics(t1, t2, apply_offset=False)
        # Update X, Y sliders
        ignore_slider_callback[leg] = True  # Prevent recursive updates
        sliders[leg][2].set_val(x2)  # Target X
        sliders[leg][3].set_val(y2)  # Target Y
        ignore_slider_callback[leg] = False
        # Send angles to Arduino
        send_all_angles()

    return handler


def on_xy_change(leg):
    def handler(val):
        if ignore_slider_callback[leg]:
            return
        x = sliders[leg][2].val  # Target X (display coordinates)
        y = sliders[leg][3].val  # Target Y (display coordinates)
        apply_ik_and_update(leg, x, y)

    return handler


def on_press(event):
    for leg in range(NUM_LEGS):
        row = leg // 2
        col = leg % 2
        if event.inaxes == axes[row, col]:
            dragging[leg] = True
            break


def on_release(event):
    for leg in range(NUM_LEGS):
        dragging[leg] = False


def on_motion(event):
    for leg in range(NUM_LEGS):
        row = leg // 2
        col = leg % 2
        if dragging[leg] and event.inaxes == axes[row, col]:
            x, y = event.xdata, event.ydata  # Display coordinates
            apply_ik_and_update(leg, x, y)
            break


def on_click(event):
    for leg in range(NUM_LEGS):
        row = leg // 2
        col = leg % 2
        if event.inaxes == axes[row, col]:
            x, y = event.xdata, event.ydata  # Display coordinates
            apply_ik_and_update(leg, x, y)
            break


def apply_ik_and_update(leg, x, y):
    global ignore_slider_callback
    theta_knee_deg, theta_hip_deg = inverse_kinematics(x, y)
    if theta_knee_deg is None:
        print(f"Leg {leg+1}: Target out of reach.")
        return
    theta_knee_deg = np.clip(theta_knee_deg, JOINT_RANGES[0][0], JOINT_RANGES[0][1])
    theta_hip_deg = np.clip(theta_hip_deg, JOINT_RANGES[1][0], JOINT_RANGES[1][1])
    ignore_slider_callback[leg] = True
    sliders[leg][0].set_val(theta_knee_deg)  # Knee
    sliders[leg][1].set_val(theta_hip_deg)  # Hip-Leg
    sliders[leg][2].set_val(x)  # Target X (display)
    sliders[leg][3].set_val(y)  # Target Y (display)
    ignore_slider_callback[leg] = False
    # update_leg(leg, theta_knee_deg, theta_hip_deg)

    leg_not_synced = False
    if not sync_states[leg]:
        sync_states[leg] = True  # Enable sync for this leg
        leg_not_synced = True

    update_synced_legs(theta_knee_deg, theta_hip_deg, x, y)

    if leg_not_synced:
        sync_states[leg] = False  # Reset sync state after update

    send_all_angles()


# ==== Reset Button Handler ====
def reset_angles(leg):
    _, _, x, y = forward_kinematics(JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1], apply_offset=True)

    leg_not_synced = False
    if not sync_states[leg]:
        sync_states[leg] = True  # Enable sync for this leg
        leg_not_synced = True

    update_synced_legs(JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1], x, y)

    if leg_not_synced:
        sync_states[leg] = False  # Reset sync state after update

    send_all_angles()


# Store checkbox states and widgets
checkboxes = [None for _ in range(NUM_LEGS)]

# Add checkboxes above each subplot
checkbox_axes = []

# You need these to store button objects to prevent them from being garbage collected
slider_buttons_up = [[None, None] for _ in range(NUM_LEGS)]  # [ [x_up, y_up], ... ]
slider_buttons_down = [[None, None] for _ in range(NUM_LEGS)]  # [ [x_down, y_down], ... ]


for leg in range(NUM_LEGS):
    row = leg // 2
    col = leg % 2

    base_x = 0.05 + col * 0.55 + 0.05
    base_y = 0.28 - row * 0.15

    # Sliders
    ax_theta1 = plt.axes([base_x, base_y - 0.00, 0.25, 0.02])
    ax_theta2 = plt.axes([base_x, base_y - 0.02, 0.25, 0.02])
    ax_x = plt.axes([base_x, base_y - 0.04, 0.25, 0.02])
    ax_y = plt.axes([base_x, base_y - 0.06, 0.25, 0.02])

    ax_reset = plt.axes([base_x, base_y - 0.115, 0.25, 0.03])  # Button below sliders

    sliders[leg][0] = Slider(
        ax_theta1,
        f"Leg {leg+1} Knee",
        JOINT_RANGES[0][0],
        JOINT_RANGES[0][1],
        valinit=JOINT_ANGLES_INIT[0],
    )
    sliders[leg][1] = Slider(
        ax_theta2,
        f"Leg {leg+1} Hip",
        JOINT_RANGES[1][0],
        JOINT_RANGES[1][1],
        valinit=JOINT_ANGLES_INIT[1],
    )
    sliders[leg][2] = Slider(
        ax_x,
        f"Leg {leg+1} X",
        -200 - offset_x,
        200 - offset_x,
        valinit=forward_kinematics(JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1], apply_offset=False)[2] - offset_x,
    )
    sliders[leg][3] = Slider(
        ax_y,
        f"Leg {leg+1} Y",
        -400 - offset_y,
        100 - offset_y,
        valinit=forward_kinematics(JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1], apply_offset=False)[3] - offset_y,
    )

    # Buttons for X
    ax_x_up = plt.axes([base_x + 0.29, base_y - 0.04, 0.02, 0.02])
    ax_x_down = plt.axes([base_x + 0.32, base_y - 0.04, 0.02, 0.02])
    btn_x_up = Button(ax_x_up, "▲")
    btn_x_down = Button(ax_x_down, "▼")

    # Buttons for Y
    ax_y_up = plt.axes([base_x + 0.29, base_y - 0.06, 0.02, 0.02])
    ax_y_down = plt.axes([base_x + 0.32, base_y - 0.06, 0.02, 0.02])
    btn_y_up = Button(ax_y_up, "▲")
    btn_y_down = Button(ax_y_down, "▼")

    # Store buttons to avoid GC
    slider_buttons_up[leg] = [btn_x_up, btn_y_up]
    slider_buttons_down[leg] = [btn_x_down, btn_y_down]

    # Callbacks for button clicks
    def make_callback(slider, delta):
        return lambda event: slider.set_val(slider.val + delta)

    btn_x_up.on_clicked(make_callback(sliders[leg][2], +1))
    btn_x_down.on_clicked(make_callback(sliders[leg][2], -1))
    btn_y_up.on_clicked(make_callback(sliders[leg][3], +1))
    btn_y_down.on_clicked(make_callback(sliders[leg][3], -1))

    sliders[leg][0].on_changed(on_slider_change(leg))
    sliders[leg][1].on_changed(on_slider_change(leg))
    sliders[leg][2].on_changed(on_xy_change(leg))
    sliders[leg][3].on_changed(on_xy_change(leg))

    # Add reset button
    buttons[leg] = Button(ax_reset, f"Reset Leg {leg+1}")
    buttons[leg].on_clicked(lambda x, l=leg: reset_angles(l))
    checkbox_ax = plt.axes([0.07 + col * 0.4, 0.95 - row * 0.4, 0.08, 0.02])
    checkbox = CheckButtons(checkbox_ax, [f"Sync Leg {leg+1}"], [False])
    checkboxes[leg] = checkbox

    def make_handler(leg_index):
        def handler(label):
            sync_states[leg_index] = not sync_states[leg_index]
            print(f"Leg {leg_index+1} sync set to {sync_states[leg_index]}")

        return handler

    checkbox.on_clicked(make_handler(leg))

# ==== Register Events ====
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("button_press_event", on_click)

# ==== Initial Draw ====
for leg in range(NUM_LEGS):
    update_leg(leg, JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1])

plt.show()

# ==== Cleanup ====
if ser and ser.is_open:
    ser.close()
