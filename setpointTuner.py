import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np  # (kept in case other parts use it)

# Add paths
libraries_dir = os.path.abspath(os.path.join(__file__, "../libraries/python"))
sys.path.append(libraries_dir)

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


class SetpointTuner:
    def __init__(self, master, pid):
        self.master = master
        self.pid = pid

        self.master.title("PID Setpoint Tuner")
        self.master.geometry("400x200")
        # Note: ttk widgets don't use bg from the master; kept for consistency
        self.master.configure(bg="#f2f2f2")

        # Create styled frame
        container = ttk.Frame(master, padding=15)
        container.pack(fill=tk.BOTH, expand=True)

        # --- Pitch Setpoint ---
        row_pitch = ttk.Frame(container)
        row_pitch.pack(fill=tk.X, pady=10)

        ttk.Label(row_pitch, text="Pitch Setpoint", width=15).pack(side=tk.LEFT, padx=5)

        self.pitch_entry = ttk.Entry(row_pitch, width=6)
        self.pitch_entry.insert(0, "0.00")
        self.pitch_entry.pack(side=tk.RIGHT, padx=5)
        self.pitch_entry.bind("<Return>", self.on_pitch_entry)

        self.pitch_slider = ttk.Scale(row_pitch, from_=-10, to=10, orient="horizontal", command=self.on_pitch_slider)
        self.pitch_slider.set(0.0)
        self.pitch_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

        # --- Roll Setpoint ---
        row_roll = ttk.Frame(container)
        row_roll.pack(fill=tk.X, pady=10)

        ttk.Label(row_roll, text="Roll Setpoint", width=15).pack(side=tk.LEFT, padx=5)

        self.roll_entry = ttk.Entry(row_roll, width=6)
        self.roll_entry.insert(0, "0.00")
        self.roll_entry.pack(side=tk.RIGHT, padx=5)
        self.roll_entry.bind("<Return>", self.on_roll_entry)

        self.roll_slider = ttk.Scale(row_roll, from_=-10, to=10, orient="horizontal", command=self.on_roll_slider)
        self.roll_slider.set(0.0)
        self.roll_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)

    # --- Pitch callbacks ---
    def on_pitch_slider(self, val):
        val = float(val)
        self.pitch_entry.delete(0, tk.END)
        self.pitch_entry.insert(0, f"{val:.2f}")
        self.pid.set_setpoint_pitch(val)

    def on_pitch_entry(self, event):
        try:
            val = float(self.pitch_entry.get())
            self.pitch_slider.set(val)  # tk will clamp to slider range if needed
            self.pid.set_setpoint_pitch(val)
        except ValueError:
            pass

    # --- Roll callbacks ---
    def on_roll_slider(self, val):
        val = float(val)
        self.roll_entry.delete(0, tk.END)
        self.roll_entry.insert(0, f"{val:.2f}")
        self.pid.set_setpoint_roll(val)

    def on_roll_entry(self, event):
        try:
            val = float(self.roll_entry.get())
            self.roll_slider.set(val)  # tk will clamp to slider range if needed
            self.pid.set_setpoint_roll(val)
        except ValueError:
            pass


def main():
    ser_write_command = SerialComm(port=SERIAL_PORT_SEND, baudrate=BAUD_RATE_SEND)
    ser_read_imu = SerialComm(port=SERIAL_PORT_REC, baudrate=BAUD_RATE_REC)

    pid = PIDController(ser_read_imu)
    walk_instance = Walk(ser_write_command)

    leg_deltas = [[0, 0] for _ in range(4)]
    leg_deltas_lock = threading.Lock()

    threading.Thread(target=pid_loop, args=(pid, leg_deltas, leg_deltas_lock), daemon=True).start()

    threading.Thread(target=lambda: walk_instance.reset_dynamic(leg_deltas=leg_deltas, duration=999999), daemon=True).start()

    walk_instance.reset()

    root = tk.Tk()
    app = SetpointTuner(root, pid)
    root.mainloop()


if __name__ == "__main__":
    main()
