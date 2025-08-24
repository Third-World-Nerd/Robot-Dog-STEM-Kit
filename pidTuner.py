import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

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


class PIDTunerUI:
    def __init__(self, master, pid):
        self.master = master
        self.pid = pid

        self.master.title("PID Tuner")
        self.master.geometry("500x400")
        self.master.configure(bg="#f2f2f2")

        # Create styled frame
        container = ttk.Frame(master, padding=15)
        container.pack(fill=tk.BOTH, expand=True)

        # Create sliders + entry fields
        self.gains = {
            "Pitch Kp": (0, 5, pid._kp_pitch, self.update_pitch_gains),
            "Pitch Ki": (0, 3, pid._ki_pitch, self.update_pitch_gains),
            "Pitch Kd": (0, 1, pid._kd_pitch, self.update_pitch_gains),
            "Roll Kp": (0, 5, pid._kp_roll, self.update_roll_gains),
            "Roll Ki": (0, 1, pid._ki_roll, self.update_roll_gains),
            "Roll Kd": (0, 1, pid._kd_roll, self.update_roll_gains),
        }

        self.sliders = {}
        self.entries = {}

        for idx, (label, (min_val, max_val, init_val, callback)) in enumerate(self.gains.items()):
            row = ttk.Frame(container)
            row.pack(fill=tk.X, pady=5)

            ttk.Label(row, text=label, width=10).pack(side=tk.LEFT, padx=5)

            slider = ttk.Scale(row, from_=min_val, to=max_val, orient="horizontal", value=init_val, command=lambda val, cb=callback, l=label: self.on_slider(val, l, cb))
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            entry = ttk.Entry(row, width=6)
            entry.insert(0, f"{init_val:.2f}")
            entry.pack(side=tk.LEFT, padx=5)

            entry.bind("<Return>", lambda event, l=label, cb=callback: self.on_entry(event, l, cb))

            self.sliders[label] = slider
            self.entries[label] = entry

    def on_slider(self, val, label, callback):
        val = float(val)
        self.entries[label].delete(0, tk.END)
        self.entries[label].insert(0, f"{val:.2f}")
        self.update_pid(label, val, callback)

    def on_entry(self, event, label, callback):
        try:
            val = float(self.entries[label].get())
            self.sliders[label].set(val)
            self.update_pid(label, val, callback)
        except ValueError:
            pass  # ignore bad input

    def update_pid(self, label, val, callback):
        if "Pitch" in label:
            kp = float(self.entries["Pitch Kp"].get())
            ki = float(self.entries["Pitch Ki"].get())
            kd = float(self.entries["Pitch Kd"].get())
            callback(kp, ki, kd)
        elif "Roll" in label:
            kp = float(self.entries["Roll Kp"].get())
            ki = float(self.entries["Roll Ki"].get())
            kd = float(self.entries["Roll Kd"].get())
            callback(kp, ki, kd)

    def update_pitch_gains(self, kp, ki, kd):
        print(kp, ki, kd)
        self.pid.set_gains_pitch(kp, ki, kd)

    def update_roll_gains(self, kp, ki, kd):
        self.pid.set_gains_roll(kp, ki, kd)


def main():
    ser_write_command = SerialComm(port=SERIAL_PORT_SEND, baudrate=BAUD_RATE_SEND)
    ser_read_imu = SerialComm(port=SERIAL_PORT_REC, baudrate=BAUD_RATE_REC)

    pid = PIDController(ser_read_imu)
    walk_instance = Walk(ser_write_command)

    leg_deltas = [[0, 0] for _ in range(4)]
    leg_deltas_lock = threading.Lock()

    threading.Thread(target=pid_loop, args=(pid, leg_deltas, leg_deltas_lock), daemon=True).start()

    walk_instance.set_step_lengthX(0.0, 0.0)  # forward
    walk_instance.set_step_lengthY(0.0, 0.0)  # forward
    threading.Thread(target=lambda: walk_instance.walk(leg_deltas=leg_deltas, duration=999999), daemon=True).start()

    walk_instance.reset()

    root = tk.Tk()
    app = PIDTunerUI(root, pid)
    root.mainloop()


if __name__ == "__main__":
    main()
