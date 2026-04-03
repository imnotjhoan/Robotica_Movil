#!/usr/bin/env python3
"""
compute_velocity.py  --  Derive v(t) and omega(t) from VICON pose data.

Outputs: vicon_velocity.csv  with columns [t, x, y, yaw, vx, vy, v, omega]
"""

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

# Load extracted pose
df = pd.read_csv("pose_raw.csv")
t   = df["t"].values
x   = df["x"].values
y   = df["y"].values
yaw = np.unwrap(df["yaw"].values)   # unwrap to avoid ±π jumps

# --- Savitzky-Golay differentiation ---
# window_length must be odd; polyorder < window_length
# Tune these if the signal looks noisy or over-smoothed
WINDOW  = 21    # samples (adjust based on VICON rate, e.g. 21 @ 100 Hz = 0.21 s)
POLY    = 3     # polynomial order

# Compute dt (assumed roughly uniform; take median)
dt = np.median(np.diff(t))

vx    = savgol_filter(x,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
vy    = savgol_filter(y,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
omega = savgol_filter(yaw, window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)

# Forward speed in the vehicle body frame
v = np.sqrt(vx**2 + vy**2)
# Sign: positive if moving in the direction of heading
heading_dot = vx * np.cos(yaw) + vy * np.sin(yaw)
v_signed = np.where(heading_dot >= 0, v, -v)

df_out = pd.DataFrame({
    "t": t, "x": x, "y": y, "yaw": yaw,
    "vx": vx, "vy": vy, "v": v_signed, "omega": omega
})
df_out.to_csv("vicon_velocity.csv", index=False)
print("Saved: vicon_velocity.csv")
print(f"  v range : [{v_signed.min():.3f},{v_signed.max():.3f}] m/s")
print(f"  ω range : [{omega.min():.3f},{omega.max():.3f}] rad/s")