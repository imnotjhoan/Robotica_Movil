#!/usr/bin/env python3
"""
validate_model.py -- Validación del modelo de velocidad del vehículo
Modelo: Primer Orden + Deadzone + Dead Time
Adaptado a tus datos (u en %)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.interpolate import interp1d

# ==================== CONFIGURACIÓN ====================
# Rutas de tus archivos (cámbialas si es necesario)
VEL_FILE = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/velocity_clean/sysid_run_25_vel_velocity.csv")   # ejemplo
CMD_FILE = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/cmd_vel/sysid_run_25_vel_input.csv")

# Parámetros del modelo general (usa los que mejor te funcionaron)
K       = 0.205     # m/s por %
tau     = 0.165     # s
u_dead  = 18.0      # %
theta   = 1.60      # s   ← dead time

# ==================== CARGA DE DATOS ====================
print("Cargando datos...")

df_v = pd.read_csv(VEL_FILE)
df_cmd = pd.read_csv(CMD_FILE)

# Columnas
t_vicon = df_v["t"].values.astype(float)
v_vicon = df_v["v"].values.astype(float)

t_cmd = df_cmd["t_bag"].values.astype(float)
u_pct = df_cmd["linear_x_pct"].values.astype(float)

# Normalizar tiempo a empezar en 0
t_vicon -= t_vicon[0]
t_cmd   -= t_cmd[0]

# Interpolar comando sobre el tiempo de VICON (Zero-Order Hold)
u_interp_fn = interp1d(t_cmd, u_pct, kind="zero", fill_value="extrapolate")
u = u_interp_fn(t_vicon)

dt = np.median(np.diff(t_vicon))

print(f"Datos cargados: {len(t_vicon)} muestras")
print(f"dt ≈ {dt:.4f} s")
print(f"Comando u: [{u.min():.1f}, {u.max():.1f}] %")

# ==================== MODELO ====================
def apply_deadzone(u_arr: np.ndarray, u_dead: float) -> np.ndarray:
    u_eff = np.zeros_like(u_arr)
    u_eff[u_arr >  u_dead] = u_arr[u_arr >  u_dead] - u_dead
    u_eff[u_arr < -u_dead] = u_arr[u_arr < -u_dead] + u_dead
    return u_eff


def simulate_model(K, tau, u_dead, theta, u_arr, dt, v0=0.0):
    """Simulación con dead time"""
    u_eff = apply_deadzone(u_arr, u_dead)
    
    # Aplicar dead time
    n_delay = max(0, int(round(theta / dt)))
    u_delayed = np.zeros_like(u_eff)
    if n_delay > 0:
        u_delayed[n_delay:] = u_eff[:-n_delay]
    else:
        u_delayed = u_eff.copy()

    v = np.zeros(len(u_arr))
    v[0] = v0
    
    for i in range(1, len(u_arr)):
        dvdt = (K * u_delayed[i-1] - v[i-1]) / tau
        v[i] = v[i-1] + dvdt * dt
    
    return v


# ==================== SIMULACIÓN Y VALIDACIÓN ====================
print("\nSimulando modelo...")

v0 = v_vicon[0]
v_sim = simulate_model(K, tau, u_dead, theta, u, dt, v0=v0)

rmse = np.sqrt(np.mean((v_sim - v_vicon)**2))
print(f"\n── Resultados de Validación ─────────────────────────────")
print(f"K      = {K:.4f} m/s por %")
print(f"tau    = {tau:.4f} s")
print(f"u_dead = {u_dead:.1f} %")
print(f"theta  = {theta:.3f} s")
print(f"RMSE   = {rmse:.4f} m/s")

# ==================== GRÁFICAS ====================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

ax1.plot(t_vicon, v_vicon, label="Medición VICON", color="steelblue", linewidth=2)
ax1.plot(t_vicon, v_sim, label=f"Modelo Simulado (RMSE = {rmse:.4f} m/s)", 
         color="tomato", linestyle="--", linewidth=2)
ax1.set_ylabel("Velocidad [m/s]")
ax1.set_title("Validación del Modelo de Velocidad")
ax1.legend()
ax1.grid(True, alpha=0.6)

ax2.step(t_vicon, u, label="Comando ESC u [%]", color="darkorange", where="post")
ax2.set_ylabel("Comando u [%]")
ax2.set_xlabel("Tiempo [s]")
ax2.legend()
ax2.grid(True, alpha=0.6)

plt.tight_layout()
plt.savefig("model_validation.png", dpi=180, bbox_inches='tight')
plt.show()

print("\nGráfica guardada como: model_validation.png")