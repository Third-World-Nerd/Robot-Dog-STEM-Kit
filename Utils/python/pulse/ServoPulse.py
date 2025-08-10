import os
import sys
import time
import tkinter as tk
from tkinter import ttk

import serial

# Joint Pulse Limits
# define PULSE_MIN   100 // 0 DEGREES
# define PULSE_MAX   530 // 180 DEGREES

# define PULSE_MIN_210   120
# define PULSE_MAX_210   510

# define PULSE_MIN_320   156 110-550
# define PULSE_MAX_320   433


# mg996 (100-530)
# 25kg (120-510)


pulse_min = 156
pulse_max = 433

pulse_for_zero_deg = 156
pulse_for_180_deg = 433

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)

# Import the config module
from config import BAUD_RATE_SEND
from config import SERIAL_PORT_SEND

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
except serial.SerialException:
    print(f"Error: Could not open serial port {SERIAL_PORT_SEND}")
    ser = None


def pulse_to_angle(pulse):
    return round((pulse - pulse_for_zero_deg) * 180 / (pulse_for_180_deg - pulse_for_zero_deg))


def update_angle_label(idx, pulse):
    angle = pulse_to_angle(pulse)
    angle_labels[idx].config(text=f"{angle}°")


def send_raw_pulse(servo_id, pulse):
    pulse = int(float(pulse))
    pulse = max(pulse_min, min(pulse_max, pulse))  # Clamp to valid range

    servo_selected = SERVO.get()
    if servo_selected == 0:
        print("No servo selected!")
        return

    actual_servo_id = servo_id + ((servo_selected - 1) * 3)  # Combine GUI index with selected checkbox
    if ser:
        command = f"{actual_servo_id}:{pulse}\n"
        ser.write(command.encode("utf-8"))
        print(f"Sent: {command.strip()}")

    value_labels[servo_id].config(text=str(pulse))
    update_angle_label(servo_id, pulse)

    if entry_boxes[servo_id].get() != str(pulse):
        entry_boxes[servo_id].delete(0, tk.END)
        entry_boxes[servo_id].insert(0, str(pulse))


def slider_changed(val, idx):
    pulse = int(float(val))
    servo_vars[idx].set(pulse)
    send_raw_pulse(idx, pulse)


def step_pulse(idx, direction):
    current_val = servo_vars[idx].get()
    new_val = max(pulse_min, min(pulse_max, current_val + direction))
    servo_vars[idx].set(new_val)
    sliders[idx].set(new_val)
    send_raw_pulse(idx, new_val)
    entry_updated(idx)


def entry_updated(idx):
    try:
        val = int(entry_boxes[idx].get())
        val = max(pulse_min, min(pulse_max, val))
        servo_vars[idx].set(val)
        sliders[idx].set(val)
        send_raw_pulse(idx, val)
    except ValueError:
        pass


# GUI setup
root = tk.Tk()
root.title("Servo Pulse Calibrator")
root.geometry("1100x600")

initial_value = (pulse_max + pulse_min) / 2
servo_vars = [tk.IntVar(value=initial_value) for _ in range(3)]
value_labels = []
entry_boxes = []
sliders = []
angle_labels = []

# Global variable for selected servo (1–4)
SERVO = tk.IntVar(value=0)


# Function to ensure only one checkbox is selected
def checkbox_selected(idx):
    for i in range(4):
        if i != idx:
            checkbox_vars[i].set(0)
    checkbox_vars[idx].set(1)
    SERVO.set(idx + 1)
    print(f"Selected SERVO: {SERVO.get()}")


# Add checkboxes
checkbox_frame = ttk.LabelFrame(root, text="Select Servo", padding=10)
checkbox_frame.grid(row=0, column=8, rowspan=4, padx=20, pady=20, sticky="n")

checkbox_vars = [tk.IntVar(value=0) for _ in range(4)]
checkboxes = []
for i in range(4):
    cb = tk.Checkbutton(
        checkbox_frame,
        text=f"Servo {i + 1}",
        variable=checkbox_vars[i],
        command=lambda idx=i: checkbox_selected(idx),
        font=("Arial", 14),
    )
    cb.pack(anchor="w", pady=5)
    checkboxes.append(cb)

# Style
style = ttk.Style()
style.configure("TLabel", font=("Arial", 16))
style.configure("TButton", font=("Arial", 14))
style.configure("TEntry", font=("Arial", 14))

# Sliders and controls
for i in range(3):
    ttk.Label(root, text=f"Servo {i}").grid(row=i, column=0, padx=20, pady=25, sticky="w")

    slider = ttk.Scale(
        root,
        from_=pulse_min,
        to=pulse_max,
        orient="horizontal",
        variable=servo_vars[i],
        length=pulse_max - pulse_min,
        command=lambda val, idx=i: slider_changed(val, idx),
    )
    slider.grid(row=i, column=1, padx=10, pady=25)
    sliders.append(slider)

    val_label = ttk.Label(root, text=str(initial_value), width=6)
    val_label.grid(row=i, column=2)
    value_labels.append(val_label)

    angle_label = ttk.Label(root, text=f"{pulse_to_angle(initial_value)}°", width=6)
    angle_label.grid(row=i, column=3)
    angle_labels.append(angle_label)

    entry = ttk.Entry(root, width=8, font=("Arial", 16))
    entry.insert(0, str(initial_value))
    entry.grid(row=i, column=4)
    entry_boxes.append(entry)

    entry_button = ttk.Button(root, text="Set", width=5, command=lambda idx=i: entry_updated(idx))
    entry_button.grid(row=i, column=5, padx=5)

    ttk.Button(root, text="▲", width=3, command=lambda idx=i: step_pulse(idx, +1)).grid(row=i, column=6, padx=5)
    ttk.Button(root, text="▼", width=3, command=lambda idx=i: step_pulse(idx, -1)).grid(row=i, column=7, padx=5)

    send_raw_pulse(i, initial_value)

root.mainloop()

# Close serial on exit
if ser:
    ser.close()


def pulse_to_angle(pulse):
    # Map pulse from range pulse_max to pulse_min to 0-180
    return round((pulse - pulse_for_zero_deg) * 180 / (pulse_for_180_deg - pulse_for_zero_deg))


def update_angle_label(idx, pulse):
    angle = pulse_to_angle(pulse)
    angle_labels[idx].config(text=f"{angle}°")


def send_raw_pulse(servo_id, pulse):
    pulse = int(float(pulse))
    pulse = max(pulse_min, min(pulse_max, pulse))  # limitting between max and min values
    if ser:
        command = f"{servo_id}:{pulse}\n"
        ser.write(command.encode("utf-8"))
        print(f"Sent: {command.strip()}")
    value_labels[servo_id].config(text=str(pulse))
    update_angle_label(servo_id, pulse)
    if entry_boxes[servo_id].get() != str(pulse):
        entry_boxes[servo_id].delete(0, tk.END)
        entry_boxes[servo_id].insert(0, str(pulse))


def slider_changed(val, idx):
    pulse = int(float(val))
    servo_vars[idx].set(pulse)
    send_raw_pulse(idx, pulse)


def step_pulse(idx, direction):
    current_val = servo_vars[idx].get()
    new_val = max(pulse_min, min(pulse_max, current_val + direction))
    servo_vars[idx].set(new_val)
    sliders[idx].set(new_val)
    send_raw_pulse(idx, new_val)
    entry_updated(idx)


def entry_updated(idx):
    try:
        val = int(entry_boxes[idx].get())
        val = max(pulse_min, min(pulse_max, val))
        servo_vars[idx].set(val)
        sliders[idx].set(val)
        send_raw_pulse(idx, val)
    except ValueError:
        pass


# Create GUI
root = tk.Tk()
root.title("Servo Pulse Calibrator")
root.geometry("1100x500")  # Slightly wider for angle labels

initial_value = (pulse_max + pulse_min) / 2
servo_vars = [tk.IntVar(value=initial_value) for _ in range(3)]
value_labels = []
entry_boxes = []
sliders = []
angle_labels = []  # Angle label list

style = ttk.Style()
style.configure("TLabel", font=("Arial", 16))
style.configure("TButton", font=("Arial", 14))
style.configure("TEntry", font=("Arial", 14))

for i in range(3):
    ttk.Label(root, text=f"Servo {i}").grid(row=i, column=0, padx=20, pady=25, sticky="w")

    slider = ttk.Scale(
        root,
        from_=pulse_min,
        to=pulse_max,
        orient="horizontal",
        variable=servo_vars[i],
        length=pulse_max - pulse_min,
        command=lambda val, idx=i: slider_changed(val, idx),
    )
    slider.grid(row=i, column=1, padx=10, pady=25)
    sliders.append(slider)

    val_label = ttk.Label(root, text=str(initial_value), width=6)
    val_label.grid(row=i, column=2)
    value_labels.append(val_label)

    # Angle label
    angle_label = ttk.Label(root, text=f"{pulse_to_angle(initial_value)}°", width=6)
    angle_label.grid(row=i, column=3)
    angle_labels.append(angle_label)

    entry = ttk.Entry(root, width=8, font=("Arial", 16))
    entry.insert(0, str(initial_value))
    entry.grid(row=i, column=4)
    entry_boxes.append(entry)

    entry_button = ttk.Button(root, text="Set", width=5, command=lambda idx=i: entry_updated(idx))
    entry_button.grid(row=i, column=5, padx=5)

    ttk.Button(root, text="▲", width=3, command=lambda idx=i: step_pulse(idx, +1)).grid(row=i, column=6, padx=5)
    ttk.Button(root, text="▼", width=3, command=lambda idx=i: step_pulse(idx, -1)).grid(row=i, column=7, padx=5)

    # Send initial pulse
    send_raw_pulse(i, initial_value)

root.mainloop()

# Close serial on exit
if ser:
    ser.close()
