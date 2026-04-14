#!/usr/bin/env python3
"""
fit_model_multi_fopdt.py  --  Modelo de Primer Orden + Dead Time (FOPDT) multi-experimento.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.optimize import minimize, differential_evolution
from scipy.interpolate import interp1d

# ── Directorios ────────────────────────────────────────────────────────────────
VEL_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/velocity_clean")
# CMD_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/cmd_vel")
CMD_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/")
OUT_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/model_fit")
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ── Lista de experimentos ──────────────────────────────────────────────────────
EXPERIMENTS = [
    {
        "label"  : "28 %",
        "vel_csv": VEL_DIR / "sysid_run_28_vel_velocity.csv",
        "cmd_csv": CMD_DIR / "sysid_run_28_vel_joy_raw.csv",
    },

    # Agrega más aquí...
]

# ── Bounds: [K, tau, u_dead, theta] ───────────────────────────────────────────
BOUNDS = [
    (0.005, 0.8),    # K      [m/s por %]
    (0.005,  4.0),    # tau    [s]
    (0.0,  20.0),    # u_dead [%]
    (0.0,  2.5),     # theta  (dead time) [s]
]

WEIGHTS = None   # None = peso igual para todos los experimentos

# ── Carga de datos (sin cambios importantes) ───────────────────────────────────
def load_experiment(exp: dict) -> dict | None:
    vel_path = exp["vel_csv"]
    cmd_path = exp["cmd_csv"]

    if not vel_path.exists() or not cmd_path.exists():
        print(f"  [SKIP] Archivo no encontrado: {exp['label']}")
        return None

    df_v   = pd.read_csv(vel_path)
    df_cmd = pd.read_csv(cmd_path)

    t_col_v = "t_bag" if "t_bag" in df_v.columns else df_v.columns[0]
    v_col   = "v"     if "v"     in df_v.columns else df_v.columns[1]
    t_col_c = "t_bag" if "t_bag" in df_cmd.columns else df_cmd.columns[0]
    u_col   = "linear_x_pct" if "linear_x_pct" in df_cmd.columns else df_cmd.columns[-1]

    t_v = df_v[t_col_v].values.astype(float) - df_v[t_col_v].values[0]
    v   = df_v[v_col].values.astype(float)
    t_c = df_cmd[t_col_c].values.astype(float) - df_cmd[t_col_c].values[0]
    u   = df_cmd[u_col].values.astype(float)

    t_common = t_v[(t_v >= t_c.min()) & (t_v <= t_c.max())]
    if len(t_common) < 10:
        print(f"  [SKIP] Solapamiento insuficiente: {exp['label']}")
        return None

    u_fn = interp1d(t_c, u, kind="zero", fill_value="extrapolate")
    u_align = u_fn(t_common)
    v_align = np.interp(t_common, t_v, v)
    dt = np.median(np.diff(t_common))

    print(f"  [OK]  {exp['label']:8s}  →  {len(t_common)} pts | "
          f"u [{u_align.min():.1f}, {u_align.max():.1f}]% | "
          f"v [{v_align.min():.2f}, {v_align.max():.2f}] m/s")

    return {"label": exp["label"], "t": t_common, "v": v_align, "u": u_align, "dt": dt}


# ── Modelo FOPDT con dead time ─────────────────────────────────────────────────
def apply_deadzone(u_arr: np.ndarray, u_dead: float) -> np.ndarray:
    u_eff = np.zeros_like(u_arr)
    u_eff[u_arr >  u_dead] = u_arr[u_arr >  u_dead] - u_dead
    u_eff[u_arr < -u_dead] = u_arr[u_arr < -u_dead] + u_dead
    return u_eff


def simulate(params: list, u_arr: np.ndarray, dt: float, v0: float = 0.0) -> np.ndarray:
    """Simulación FOPDT: Primer Orden + Dead Time + Deadzone"""
    K, tau, u_dead, theta = params
    
    u_eff = apply_deadzone(u_arr, u_dead)
    
    # Aplicar dead time (desplazamiento)
    n_delay = max(0, int(round(theta / dt)))
    u_delayed = np.zeros_like(u_eff)
    if n_delay == 0:
        u_delayed = u_eff
    else:
        u_delayed[n_delay:] = u_eff[:-n_delay]

    # Integración Euler
    v = np.zeros(len(u_arr))
    v[0] = v0
    for i in range(1, len(u_arr)):
        dvdt = (K * u_delayed[i-1] - v[i-1]) / tau
        v[i] = v[i-1] + dvdt * dt
    return v


# ── Función de costo multi-experimento ────────────────────────────────────────
weights = np.array(WEIGHTS if WEIGHTS is not None else [1.0] * len(EXPERIMENTS))
weights = weights / weights.sum()

def cost_total(params: list) -> float:
    if any(p < 0 for p in params):   # penalización parámetros no físicos
        return 1e9

    total_mse = 0.0
    for ds, w in zip(datasets, weights):
        v0 = ds["v"][0]                    # condición inicial real
        v_sim = simulate(params, ds["u"], ds["dt"], v0=v0)
        mse = np.mean((v_sim - ds["v"]) ** 2)
        total_mse += w * mse
    return total_mse


# ── Carga datos ───────────────────────────────────────────────────────────────
print("\n── Cargando experimentos ────────────────────────────────────────────────")
datasets = []
for exp in EXPERIMENTS:
    data = load_experiment(exp)
    if data is not None:
        datasets.append(data)

if not datasets:
    raise SystemExit("ERROR: No se cargaron experimentos.")

print(f"\nTotal experimentos cargados: {len(datasets)}\n")


# ── Optimización ──────────────────────────────────────────────────────────────
print("── Optimización FOPDT ───────────────────────────────────────────────────")
print("Etapa 1: Búsqueda global (Differential Evolution)...")

de_result = differential_evolution(
    cost_total,
    bounds=BOUNDS,
    seed=42,
    maxiter=800,
    popsize=25,
    tol=1e-8,
    polish=False
)
print(f"Etapa 1 terminada → MSE = {de_result.fun:.6f}")

print("Etapa 2: Refinamiento local (L-BFGS-B)...")
result = minimize(
    cost_total,
    de_result.x,
    method="L-BFGS-B",
    bounds=BOUNDS,
    options={"maxiter": 2000, "ftol": 1e-12}
)

K_fit, tau_fit, u_dead_fit, theta_fit = result.x
rmse_total = np.sqrt(result.fun)

print("\n── Parámetros ajustados (FOPDT) ────────────────────────────────────────")
print(f"K       = {K_fit:.5f}   m/s por %")
print(f"tau     = {tau_fit:.5f}   s")
print(f"u_dead  = {u_dead_fit:.3f}   %")
print(f"theta   = {theta_fit:.4f}   s   ← Dead Time")
print(f"RMSE    = {rmse_total:.4f}   m/s")

# ── Gráficas ──────────────────────────────────────────────────────────────────
n_exp = len(datasets)
fig, axes = plt.subplots(n_exp, 2, figsize=(14, 4.5 * n_exp))

if n_exp == 1:
    axes = np.array([axes])

for i, ds in enumerate(datasets):
    v0 = ds["v"][0]
    v_sim = simulate(result.x, ds["u"], ds["dt"], v0=v0)
    rmse_i = np.sqrt(np.mean((v_sim - ds["v"])**2))

    ax_v, ax_u = axes[i]

    ax_v.plot(ds["t"], ds["v"], color="steelblue", label="Medición (VICON)")
    ax_v.plot(ds["t"], v_sim,  color="tomato", linestyle="--",
              label=f"Modelo (RMSE={rmse_i:.4f} m/s)")
    ax_v.set_title(f"Experimento {ds['label']}")
    ax_v.set_ylabel("Velocidad [m/s]")
    ax_v.legend()
    ax_v.grid(True)

    ax_u.step(ds["t"], ds["u"], color="darkorange", where="post", label="u [%]")
    ax_u.set_ylabel("Entrada [%]")
    ax_u.set_xlabel("Tiempo [s]")
    ax_u.legend()
    ax_u.grid(True)

fig.suptitle(
    f"Modelo FOPDT  –  K={K_fit:.4f} m/s·%⁻¹ | τ={tau_fit:.4f} s | "
    f"u_dead={u_dead_fit:.2f}% | θ={theta_fit:.3f}s | RMSE={rmse_total:.4f} m/s",
    fontsize=12
)
plt.tight_layout()

plot_path = OUT_DIR / "model_fit_fopdt.png"
plt.savefig(plot_path, dpi=180, bbox_inches='tight')
plt.show()

print(f"\nGráfica guardada en: {plot_path}")