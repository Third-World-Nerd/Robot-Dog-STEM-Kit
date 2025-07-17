import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import comb

l1, l2 = 5.0, 5.0  # Leg lengths

def bezier_n(t, control_points):
    """Calculate nth degree Bezier point at t with control_points"""
    n = len(control_points) - 1
    point = np.zeros(2)
    for i, P in enumerate(control_points):
        bern = comb(n, i) * (t**i) * ((1 - t)**(n - i))
        point += bern * P
    return point

def inverse_kinematics(x, y, l1, l2):
    D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    if abs(D) > 1:
        return None
    theta2 = np.arccos(D)
    theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))
    return theta1, theta2

def forward_kinematics(theta1, theta2):
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    return (x1, y1), (x2, y2)

# Define 8 control points (degree 7) for full gait (swing + stance)
control_points = np.array([
    [0, 0],      # P0 start/end
    [-2, 0.3],   # P1 adjusted for tangent continuity
    [2, 3],
    [4, 2],
    [5, 0],
    [4.5, -0.5],
    [2, -0.3],
    [0, 0]       # P7 same as P0 to close the loop
])

scaleX = 





n_points = 100
ts_swing = np.linspace(0.0, 0.5, n_points // 2)
ts_stance = np.linspace(0.5, 1.0, n_points // 2)
ts = np.concatenate((ts_swing, ts_stance))
foot_path = np.array([bezier_n(t, control_points) for t in ts])
print(foot_path.shape)
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 4)
line, = ax.plot([], [], 'o-', lw=4)
foot_trace, = ax.plot([], [], 'r--', lw=1)
trace_points = []

def update(i):
    pt = foot_path[i]
    ik = inverse_kinematics(pt[0], pt[1], l1, l2)
    if ik is None:
        return line, foot_trace
    theta1, theta2 = ik
    knee, foot = forward_kinematics(theta1, theta2)
    line.set_data([0, knee[0], foot[0]], [0, knee[1], foot[1]])

    trace_points.append(foot)
    trace_arr = np.array(trace_points)
    foot_trace.set_data(trace_arr[:, 0], trace_arr[:, 1])
    return line, foot_trace

ani = animation.FuncAnimation(fig, update, frames=len(foot_path), interval=30, blit=True)
plt.title("2DOF Robot Leg: Smooth Gait with 7th Degree Bezier Curve")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.grid()
plt.show()