import os
import sys
import time
import tkinter as tk
from tkinter import ttk

import serial

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)

# Import the config module
from config import BAUD_RATE_SEND
from config import SERIAL_PORT_SEND

Angles = [[90, 0, 0], [90, 30, 0], [90, 0, 0], [90, 0, 0]]

try:
    ser = serial.Serial(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=1)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error: Could not open serial port {SERIAL_PORT_SEND}: {e}")
    ser = None

# === Initialize Tkinter Window ===
root = tk.Tk()
root.title("Leg Calibration GUI")
root.geometry("1200x700")

# Make the grid responsive
root.grid_rowconfigure(0, weight=1)
root.grid_rowconfigure(1, weight=1)
root.grid_columnconfigure(0, weight=1)
root.grid_columnconfigure(1, weight=1)

# === Fonts ===
HEADER_FONT = ("Arial", 16, "bold")
LABEL_FONT = ("Arial", 14)
BUTTON_FONT = ("Arial", 14)
MIDPOINT_FONT = ("Arial", 18, "bold")

from config import JOINT_NAMES
from config import JOINT_RANGES
from config import NUM_JOINTS
from config import NUM_LEGS

# === Storage for UI elements ===
sliders = [[None for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]
offsets = [[None for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]
slider_values = [[None for _ in range(NUM_JOINTS)] for _ in range(NUM_LEGS)]

# Suppress sending during batch updates
suppress_serial = False

# Status label for feedback
status_var = tk.StringVar(value="Ready")
status_label = ttk.Label(root, textvariable=status_var, font=LABEL_FONT)
status_label.grid(row=4, column=0, columnspan=2, pady=10)


# === Send data ===
def send_all_leg_data():
    global suppress_serial
    if suppress_serial:
        return
    values = []
    for leg in range(NUM_LEGS):
        for j in range(NUM_JOINTS):
            try:
                if sliders[leg][j] is None:
                    status_var.set(f"Error: Slider for Leg {leg+1}, Joint {j+1} is not initialized")
                    values.append("0")
                    continue
                angle = sliders[leg][j].get()
                offset_str = offsets[leg][j].get()
                offset = float(offset_str) if offset_str.strip() else 0.0
                total = int(angle + offset)
                min_val, max_val = JOINT_RANGES[j]
                total = min(max(total, min_val), max_val)
                values.append(str(total))
            except ValueError:
                values.append(str(int(sliders[leg][j].get())))
            except Exception as e:
                status_var.set(f"Error in send_all_leg_data: {e}")
                values.append("0")

    # Format as: <val1,val2,...,valN>
    command = "<" + ",".join(values) + ">\n"

    if ser and ser.is_open:
        try:
            ser.write(command.encode("utf-8"))
            status_var.set(f"Sent: {command.strip()}")
        except serial.SerialException as e:
            status_var.set(f"Serial Error: {e}")
    else:
        status_var.set(f"Error: Serial port {SERIAL_PORT_SEND} not available")

    print("Sent:", command.strip())


# === Callback for slider move ===
def on_slider_change(val, leg, j):
    try:
        if sliders[leg][j] is None or slider_values[leg][j] is None:
            status_var.set(f"Error: Slider or label for Leg {leg+1}, Joint {j+1} is not initialized")
            return
        val = int(float(val))
        slider_values[leg][j].config(text=f"{val}°")
        send_all_leg_data()
    except ValueError:
        slider_values[leg][j].config(text="Error")
        status_var.set(f"Error: Invalid slider value for Leg {leg+1}, Joint {j+1}")
    except Exception as e:
        status_var.set(f"Error in on_slider_change: {e}")


# === Increment/Decrement Functions ===
def adjust_slider(leg, j, delta):
    try:
        if sliders[leg][j] is None:
            status_var.set(f"Error: Slider for Leg {leg+1}, Joint {j+1} is not initialized")
            return
        current = sliders[leg][j].get()
        new_val = current + delta
        min_val, max_val = JOINT_RANGES[j]
        new_val = min(max(new_val, min_val), max_val)
        sliders[leg][j].set(new_val)
        on_slider_change(new_val, leg, j)
    except Exception as e:
        status_var.set(f"Error adjusting slider: {e}")


# === Reset all sliders to midpoint ===
def reset_all():
    global suppress_serial
    suppress_serial = True
    try:
        for leg in range(NUM_LEGS):
            for j in range(NUM_JOINTS):
                if sliders[leg][j] is None or slider_values[leg][j] is None:
                    status_var.set(f"Error: Slider or label for Leg {leg+1}, Joint {j+1} is not initialized")
                    continue
                min_val, max_val = JOINT_RANGES[j]
                mid_val = Angles[leg][j]  # Use the initial angles from the Angles list
                sliders[leg][j].set(mid_val)
                slider_values[leg][j].config(text=f"{mid_val}°")
        status_var.set("All sliders reset to midpoint")
    except Exception as e:
        status_var.set(f"Error resetting sliders: {e}")
    finally:
        suppress_serial = False
        send_all_leg_data()


# === Create 2x2 Leg Sections ===
for leg in range(NUM_LEGS):
    frame = ttk.LabelFrame(root, text=f"Leg {leg+1}", padding=10)
    frame.config(labelanchor="n")
    row = leg // 2
    col = leg % 2
    frame.grid(row=row, column=col, padx=20, pady=20, sticky="nsew")
    frame.grid_columnconfigure(2, weight=1)  # Make slider column expandable

    for j in range(NUM_JOINTS):
        # Joint label
        ttk.Label(frame, text=JOINT_NAMES[j], font=LABEL_FONT).grid(row=j, column=0, padx=5, pady=5, sticky="w")

        min_val, max_val = JOINT_RANGES[j]
        mid_val = Angles[leg][j]  # Use the initial angles from the Angles list

        # ▲ button
        up_btn = ttk.Button(
            frame,
            text="▲",
            width=3,
            command=lambda l=leg, jj=j: adjust_slider(l, jj, +1),
        )
        up_btn.grid(row=j, column=1, padx=2)

        # Slider
        slider = ttk.Scale(
            frame,
            from_=min_val,
            to=max_val,
            orient="horizontal",
            length=max_val - min_val,
            command=lambda val, l=leg, jj=j: on_slider_change(val, l, jj),
        )
        slider.set(mid_val)
        slider.grid(row=j, column=2, padx=5, sticky="ew")
        sliders[leg][j] = slider
        print(f"Initialized slider for Leg {leg+1}, Joint {j+1}")

        # ▼ button
        down_btn = ttk.Button(
            frame,
            text="▼",
            width=3,
            command=lambda l=leg, jj=j: adjust_slider(l, jj, -1),
        )
        down_btn.grid(row=j, column=3, padx=2)

        # Current value label
        val_label = ttk.Label(frame, text=f"{mid_val}°", width=5, font=LABEL_FONT)
        val_label.grid(row=j, column=4, padx=5)
        slider_values[leg][j] = val_label
        print(f"Initialized value label for Leg {leg+1}, Joint {j+1}")

        # Offset label and entry
        ttk.Label(frame, text="Offset:", font=LABEL_FONT).grid(row=j, column=5, padx=5, sticky="w")
        offset_var = tk.StringVar(value="0")
        offset_entry = ttk.Entry(frame, textvariable=offset_var, width=6, font=LABEL_FONT)
        offset_entry.grid(row=j, column=6, padx=5)
        offset_entry.bind("<Return>", lambda event: send_all_leg_data())
        offsets[leg][j] = offset_var

        # ⏎ send button
        send_button = ttk.Button(frame, text="⏎", width=3, command=send_all_leg_data)
        send_button.grid(row=j, column=7, padx=5)

# === Global Midpoint Info ===
midpoint_frame = ttk.LabelFrame(root, text="Joint Midpoints (Common for All Legs)", padding=15)
midpoint_frame.grid(row=2, column=0, columnspan=2, padx=20, pady=10, sticky="ew")
midpoint_frame.grid_columnconfigure(0, weight=1)
midpoint_frame.grid_columnconfigure(1, weight=1)
midpoint_frame.grid_columnconfigure(2, weight=1)

for j in range(NUM_JOINTS):
    min_val, max_val = JOINT_RANGES[j]
    mid_val = Angles[0][j]  # Use the initial angles from the Angles list
    ttk.Label(
        midpoint_frame,
        text=f"{JOINT_NAMES[j]} Midpoint: {mid_val}°",
        font=MIDPOINT_FONT,
    ).grid(row=0, column=j, padx=30, pady=10)

# === Reset Button ===
style = ttk.Style()
style.configure("TButton", font=BUTTON_FONT)
reset_btn = ttk.Button(root, text="Reset All to Midpoint", command=reset_all, width=20, style="TButton")
reset_btn.grid(row=3, column=0, columnspan=2, pady=20)


# === Cleanup on window close ===
def on_closing():
    if ser and ser.is_open:
        ser.close()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_closing)

# === Verify Initialization ===
for leg in range(NUM_LEGS):
    for j in range(NUM_JOINTS):
        if sliders[leg][j] is None or slider_values[leg][j] is None:
            status_var.set(f"Initialization Error: Leg {leg+1}, Joint {j+1} not properly set")
            print(f"Initialization Error: Leg {leg+1}, Joint {j+1} - Slider: {sliders[leg][j]}, Label: {slider_values[leg][j]}")

root.mainloop()
