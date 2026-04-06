#!/usr/bin/env python3
"""
compute_velocity.py  --  Derive v(t) and omega(t) from VICON pose data.

Estrategia:
1. Mediana (elimina picos aislados)
2. Hampel (elimina clusters de outliers)
3. Savitzky-Golay (suaviza y deriva)
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter, medfilt

# ── Reemplazar ceros por NaN (datos inválidos) ─────────────────────────
def replace_zeros_with_nan(x, y, yaw, tol=1e-6):
    mask_invalid = (
        (np.abs(x) < tol) &
        (np.abs(y) < tol) &
        (np.abs(yaw) < tol)
    )

    print(f"Datos inválidos detectados (ceros): {np.sum(mask_invalid)}")

    x[mask_invalid] = np.nan
    y[mask_invalid] = np.nan
    yaw[mask_invalid] = np.nan

    return x, y, yaw


def interpolate_nan(signal):
    return pd.Series(signal).interpolate(limit_direction='both').to_numpy()

def remove_jumps(signal, threshold):
    """
    Detecta saltos grandes y los corrige manteniendo continuidad.
    """
    corrected = signal.copy()

    for i in range(1, len(signal)):
        delta = corrected[i] - corrected[i-1]

        if abs(delta) > threshold:
            # reemplazar con valor anterior (o interpolar)
            corrected[i] = corrected[i-1]

    return corrected

# ── Filtro Hampel robusto ─────────────────────────────────────────────────────
def hampel_filter(signal, window_size=5, n_sigmas=3):
    """
    Filtro Hampel robusto para detección de outliers.

    signal: array 1D
    window_size: puntos a cada lado
    n_sigmas: sensibilidad (3 estándar)
    """
    n = len(signal)
    filtered = signal.copy()
    k = 1.4826  # factor de escala para MAD

    outliers = 0

    for i in range(n):
        start = max(i - window_size, 0)
        end   = min(i + window_size + 1, n)

        window = signal[start:end]

        median = np.median(window)
        mad = np.median(np.abs(window - median))

        if mad == 0:
            continue

        sigma = k * mad

        if abs(signal[i] - median) > n_sigmas * sigma:
            filtered[i] = median
            outliers += 1

    print(f"Outliers detectados: {outliers} / {n}")
    return filtered

# ── Rutas ─────────────────────────────────────────────────────────────────────
INPUT_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("pose_raw.csv")
OUTPUT_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/velocity_clean")

if not INPUT_PATH.exists():
    print(f"ERROR: no se encontró el archivo '{INPUT_PATH}'")
    sys.exit(1)

out_stem = INPUT_PATH.stem.replace("pose_raw", "velocity")
out_path = OUTPUT_DIR / (out_stem + ".csv")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Carga de datos ─────────────────────────────────────────────────────────────
df  = pd.read_csv(INPUT_PATH)
t   = df["t"].values
x   = df["x"].values.astype(float)
y   = df["y"].values.astype(float)
yaw = np.unwrap(df["yaw"].values.astype(float))
# ── Eliminar ceros inválidos ─────────────────────────────
x, y, yaw = replace_zeros_with_nan(x, y, yaw)

# ── Interpolar datos faltantes ───────────────────────────
x   = interpolate_nan(x)
y   = interpolate_nan(y)
yaw = interpolate_nan(yaw)


# ── Conversión de unidades ─────────────────────────────────────────────
# Ajusta según tu sistema VICON

ESCALA = 0.001  # mm → m

x = x * ESCALA
y = y * ESCALA


# ── Corrección de saltos grandes ─────────────────────────────
yaw = remove_jumps(yaw, threshold=1.5)   # rad
# x   = remove_jumps(x, threshold=1000.0)     # m
# y   = remove_jumps(y, threshold=1000.0)

# ── Limpieza de Outliers ──────────────────────────────────────────────────────
APLICAR_LIMPIEZA = True

# 🔧 PARÁMETROS (AJUSTA ESTO SEGÚN TUS DATOS)
WINDOW_HAMPEL = 5   # tamaño de ventana (clave)
N_SIGMAS = 3       # sensibilidad

if APLICAR_LIMPIEZA:
    print("Aplicando limpieza de outliers...")

    # 1. Eliminar picos aislados (ruido de 1 muestra)
    x   = medfilt(x, kernel_size=5)
    y   = medfilt(y, kernel_size=5)
    yaw = medfilt(yaw, kernel_size=5)

    # 2. Eliminar clusters de outliers
    x   = hampel_filter(x, window_size=WINDOW_HAMPEL, n_sigmas=N_SIGMAS)
    y   = hampel_filter(y, window_size=WINDOW_HAMPEL, n_sigmas=N_SIGMAS)
    yaw = hampel_filter(yaw, window_size=WINDOW_HAMPEL, n_sigmas=N_SIGMAS)
    # después de derivar



# ── Savitzky-Golay differentiation ────────────────────────────────────────────
WINDOW = 21
POLY   = 3

dt = np.median(np.diff(t))
print(f"Usando dt = {dt:.4f} s para derivación")

vx    = savgol_filter(x,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
vy    = savgol_filter(y,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
omega = savgol_filter(yaw, window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)



# ── Velocidad longitudinal con signo ──────────────────────────────────────────
v           = np.sqrt(vx**2 + vy**2)
heading_dot = vx * np.cos(yaw) + vy * np.sin(yaw)
v_signed    = np.where(heading_dot >= 0, v, -v)

# ── Guardar ───────────────────────────────────────────────────────────────────
df_out = pd.DataFrame({
    "t": t,
    "x": x,
    "y": y,
    "yaw": yaw,
    "vx": vx,
    "vy": vy,
    "v": v_signed,
    "omega": omega
})

df_out.to_csv(out_path, index=False)

print(f"Input : {INPUT_PATH}")
print(f"Saved : {out_path}")
print(f"v range : [{v_signed.min():.3f}, {v_signed.max():.3f}] m/s")
print(f"ω range : [{omega.min():.3f}, {omega.max():.3f}] rad/s")

import matplotlib.pyplot as plt

# Crear figura con 4 subplots (2 filas x 2 columnas)
fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

# Gráfico de velocidad longitudinal v(t)
axs[0, 0].plot(t, v_signed, label="v(t)", color="blue")
axs[0, 0].set_ylabel("Velocidad [m/s]")
axs[0, 0].set_title("Velocidad longitudinal vs tiempo")
axs[0, 0].grid(True)
axs[0, 0].legend()

# Gráfico de velocidad angular ω(t)
axs[0, 1].plot(t, omega, label="ω(t)", color="red")
axs[0, 1].set_ylabel("Velocidad angular [rad/s]")
axs[0, 1].set_title("Velocidad angular vs tiempo")
axs[0, 1].grid(True)
axs[0, 1].legend()

# Gráfico de yaw(t)
axs[1, 0].plot(t, yaw, label="yaw(t)", color="green")
axs[1, 0].set_xlabel("Tiempo [s]")
axs[1, 0].set_ylabel("Yaw [rad]")
axs[1, 0].set_title("Yaw vs tiempo")
axs[1, 0].grid(True)
axs[1, 0].legend()

# Gráfico de posición x(t), y(t)
axs[1, 1].plot(t, x, label="x(t)", color="purple")
axs[1, 1].plot(t, y, label="y(t)", color="orange")
axs[1, 1].set_xlabel("Tiempo [s]")
axs[1, 1].set_ylabel("Posición [m]")
axs[1, 1].set_title("Posición x,y vs tiempo")
axs[1, 1].grid(True)
axs[1, 1].legend()

plt.tight_layout()
plt.show()