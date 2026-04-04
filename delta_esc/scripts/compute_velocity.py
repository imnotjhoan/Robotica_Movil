#!/usr/bin/env python3
"""
compute_velocity.py  --  Derive v(t) and omega(t) from VICON pose data.

Usage:
    python3 compute_velocity.py <path/to/nombre_pose_raw.csv>

"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter

# ── Rutas ─────────────────────────────────────────────────────────────────────
INPUT_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("pose_raw.csv")
OUTPUT_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/velocity")

if not INPUT_PATH.exists():
    print(f"ERROR: no se encontró el archivo '{INPUT_PATH}'")
    sys.exit(1)

# nombre_pose_raw  →  nombre_velocity
out_stem  = INPUT_PATH.stem.replace("pose_raw", "velocity")
out_path  = OUTPUT_DIR / (out_stem + ".csv")

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Carga de datos ─────────────────────────────────────────────────────────────
df  = pd.read_csv(INPUT_PATH)
t   = df["t"].values
x   = df["x"].values
y   = df["y"].values
yaw = np.unwrap(df["yaw"].values)   # unwrap to avoid ±π jumps

# ── Savitzky-Golay differentiation ────────────────────────────────────────────
WINDOW = 21     # samples  (21 @ 100 Hz = 0.21 s  |  41 @ 200 Hz = 0.21 s)
POLY   = 3      # polynomial order  —  debe ser < WINDOW

dt = np.median(np.diff(t))

vx    = savgol_filter(x,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
vy    = savgol_filter(y,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
omega = savgol_filter(yaw, window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)

# ── Velocidad longitudinal con signo ──────────────────────────────────────────
v            = np.sqrt(vx**2 + vy**2)
heading_dot  = vx * np.cos(yaw) + vy * np.sin(yaw)
v_signed     = np.where(heading_dot >= 0, v, -v)

# ── Guardar ───────────────────────────────────────────────────────────────────
df_out = pd.DataFrame({
    "t": t, "x": x, "y": y, "yaw": yaw,
    "vx": vx, "vy": vy, "v": v_signed, "omega": omega
})
df_out.to_csv(out_path, index=False)

print(f"Input : {INPUT_PATH}")
print(f"Saved : {out_path}")
print(f"  v range : [{v_signed.min():.3f}, {v_signed.max():.3f}] m/s")
print(f"  ω range : [{omega.min():.3f}, {omega.max():.3f}] rad/s")