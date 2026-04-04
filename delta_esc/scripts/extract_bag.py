#!/usr/bin/env python3
"""
extract_bag.py  --  Read a rosbag, export raw and synchronized data to CSV.
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# --- Configuración ---
# --- Configuración ---
BAG_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)    
OUTPUT_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data")
OUTPUT_DIR_SYNC = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/synchronized")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT_DIR_SYNC.mkdir(parents=True, exist_ok=True)


# Nombre base derivado del bag
bag_name = BAG_PATH.name

# Parámetro de sincronización (50 Hz)
DT_SYNC = 0.02 

def read_topic(reader, typestore, topic_name, fields_fn):
    rows = []
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        return pd.DataFrame()
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        t = timestamp * 1e-9  
        rows.append((t,) + fields_fn(msg))
    return pd.DataFrame(rows)

def extract_pose(msg):
    # Manejo robusto de Pose vs PoseStamped
    try:
        p, o = msg.position, msg.orientation
    except AttributeError:
        p, o = msg.pose.position, msg.pose.orientation
    
    yaw = np.arctan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
    return (p.x, p.y, p.z, yaw)

def extract_joy(msg):
    throttle = float(msg.axes[1]) if len(msg.axes) > 1 else 0.0
    steering  = float(msg.axes[3]) if len(msg.axes) > 3 else 0.0
    return (throttle, steering)

# --- Proceso ---
with Reader(BAG_PATH) as reader:
    print(f"Reading bag: {BAG_PATH}")
    df_pose = read_topic(reader, TYPESTORE, "/robot1/pose", extract_pose)
    df_joy  = read_topic(reader, TYPESTORE, "/joy", extract_joy)

if df_pose.empty:
    print("ERROR: No se encontraron datos de Pose.")
    sys.exit(1)

# Nombres y Limpieza
df_pose.columns = ["t", "x", "y", "z", "yaw"]
if not df_joy.empty:
    df_joy.columns = ["t", "joy_throttle", "joy_steering"]

# Alineación de tiempo (t=0)
t0 = df_pose["t"].min()
df_pose["t"] -= t0
if not df_joy.empty:
    df_joy["t"] -= t0
# 1. Guardar archivos RAW
df_pose.to_csv(OUTPUT_DIR / f"{bag_name}_pose_raw.csv", index=False)
if not df_joy.empty:
    df_joy.to_csv(OUTPUT_DIR / f"{bag_name}_joy_raw.csv", index=False)


# 2. SINCRONIZACIÓN (Interpolación)
print("Sincronizando datos a 50Hz...")

# Crear vector de tiempo uniforme basado en la intersección de ambos sensores
t_min = max(df_pose["t"].min(), df_joy["t"].min() if not df_joy.empty else 0)
t_max = min(df_pose["t"].max(), df_joy["t"].max() if not df_joy.empty else 1e9)
t_sync = np.arange(t_min, t_max, DT_SYNC)

df_sync = pd.DataFrame({"t": t_sync})

# Interpolar Posición (Lineal: el auto se mueve suavemente entre puntos)
for col in ["x", "y", "z", "yaw"]:
    df_sync[col] = np.interp(t_sync, df_pose["t"], df_pose[col])

# Interpolar Joy (Zero-Order Hold: el comando se mantiene hasta el siguiente cambio)
if not df_joy.empty:
    for col in ["joy_throttle", "joy_steering"]:
        # Usamos np.interp con los datos de joy pero con un pequeño truco
        # para que se comporte como escalones (no líneas diagonales)
        from scipy.interpolate import interp1d
        
        # 'kind=step-after' o 'nearest' asegura que el comando no se "invente" valores intermedios
        f_joy = interp1d(df_joy["t"], df_joy[col], kind='nearest', fill_value="extrapolate")
        df_sync[col] = f_joy(t_sync)

# Guardar el resultado final

df_sync.to_csv(OUTPUT_DIR_SYNC / f"{bag_name}_data_synchronized.csv", index=False)

print("-" * 30)
print(f"Completado exitosamente.")
print(f"Archivos guardados en: {OUTPUT_DIR}")
print(f"Muestras sincronizadas: {len(df_sync)}")