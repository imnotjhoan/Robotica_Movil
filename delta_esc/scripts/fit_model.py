#!/usr/bin/env python3
"""
fit_model.py  --  Fit a 1st-order vehicle speed model to VICON + ESC data.

Model:
    u_eff = apply_deadzone(u, u_dead)
    dv/dt = (K * u_eff - v) / tau

Optimizes [K, tau, u_dead] to minimize MSE between simulated v and v_vicon.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.interpolate import interp1d

# ── Load data ──────────────────────────────────────────────────────────────────
df_v   = pd.read_csv("vicon_velocity.csv")
df_cmd = pd.read_csv("esc_command_raw.csv")

t_vicon = df_v["t"].values
v_vicon = df_v["v"].values

t_cmd = df_cmd["t"].values
u_raw = df_cmd["u"].values

# Interpolate ESC command onto VICON time axis
u_interp_fn = interp1d(t_cmd, u_raw, kind="zero", fill_value="extrapolate")
u = u_interp_fn(t_vicon)

dt = np.median(np.diff(t_vicon))


# ── Model simulation ───────────────────────────────────────────────────────────
def apply_deadzone(u_arr, u_dead):
    u_eff = np.zeros_like(u_arr)
    mask_pos = u_arr >  u_dead
    mask_neg = u_arr < -u_dead
    u_eff[mask_pos] = u_arr[mask_pos] - u_dead
    u_eff[mask_neg] = u_arr[mask_neg] + u_dead
    return u_eff


def simulate_model(params, u_arr, dt):
    K, tau, u_dead = params
    u_eff = apply_deadzone(u_arr, u_dead)
    v = np.zeros(len(u_arr))
    for i in range(1, len(u_arr)):
        dvdt = (K * u_eff[i - 1] - v[i - 1]) / tau
        v[i] = v[i - 1] + dvdt * dt
    return v


def cost(params):
    K, tau, u_dead = params
    if K <= 0 or tau <= 0 or u_dead < 0 or u_dead > 0.5:
        return 1e9  # penalty for unphysical params
    v_sim = simulate_model(params, u, dt)
    return np.mean((v_sim - v_vicon) ** 2)


# ── Optimization ───────────────────────────────────────────────────────────────
# Initial guess: K=1.0 m/s, tau=0.3 s, u_dead=0.05
x0 = [1.0, 0.3, 0.05]
bounds = [(0.1, 5.0), (0.01, 2.0), (0.0, 0.4)]

result = minimize(cost, x0, method="L-BFGS-B", bounds=bounds,
                  options={"maxiter": 1000, "ftol": 1e-10})

K_fit, tau_fit, u_dead_fit = result.x
print(f"\n── Fitted Parameters ────────────────────────────────")
print(f"  K      ={K_fit:.4f}  m/s per unit command")
print(f"  tau    ={tau_fit:.4f}  s  (time constant)")
print(f"  u_dead ={u_dead_fit:.4f}  (normalized dead zone)")
print(f"  MSE    ={result.fun:.6f}  (m/s)²")
print(f"  RMSE   ={np.sqrt(result.fun):.4f}  m/s")

# ── Plot ───────────────────────────────────────────────────────────────────────
v_fit = simulate_model(result.x, u, dt)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

ax1.plot(t_vicon, v_vicon, label="VICON (ground truth)", color="steelblue")
ax1.plot(t_vicon, v_fit,   label=f"Model fit  (K={K_fit:.3f}, τ={tau_fit:.3f}, dead={u_dead_fit:.3f})",
         color="tomato", linestyle="--")
ax1.set_ylabel("Speed [m/s]")
ax1.legend()
ax1.grid(True)

ax2.step(t_vicon, u, label="ESC command u", color="orange", where="post")
ax2.set_ylabel("ESC command [-1, +1]")
ax2.set_xlabel("Time [s]")
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.savefig("model_fit.png", dpi=150)
plt.show()
print("Saved: model_fit.png")