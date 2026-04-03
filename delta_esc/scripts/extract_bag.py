#!/usr/bin/env python3
"""
extract_bag.py  --  Read a rosbag and export synchronized data to CSV (Pose & Joy).

Usage:
    python3 extract_bag.py <path_to_bag_folder>
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Configuración de ruta y tipos de ROS 2
BAG_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


def read_topic(reader, typestore, topic_name, fields_fn):
    """Extrae (timestamp_s, *fields) de un tópico usando una función extractora."""
    rows = []
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        print(f"  WARNING: topic '{topic_name}' not found in bag")
        return pd.DataFrame()
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        t = timestamp * 1e-9  # nanosegundos → segundos
        rows.append((t,) + fields_fn(msg))
    return pd.DataFrame(rows)


def extract_pose(msg):
    """Extrae posición y convierte cuaternión a Yaw."""
    p = msg.position
    o = msg.orientation
    # Asunción 2D: Cálculo de Yaw
    yaw = np.arctan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    )
    return (p.x, p.y, p.z, yaw)


def extract_joy(msg):
    """Extrae datos del control (throttle y steering)."""
    # axes[1] = stick izquierdo vertical, axes[3] = stick derecho horizontal
    throttle = float(msg.axes[1]) if len(msg.axes) > 1 else 0.0
    steering  = float(msg.axes[3]) if len(msg.axes) > 3 else 0.0
    return (throttle, steering)


# --- Proceso de lectura ---
with Reader(BAG_PATH) as reader:
    print(f"Reading bag: {BAG_PATH}")

    # Solo extraemos Pose (Vicon) y Joy
    df_pose = read_topic(reader, TYPESTORE, "/robot1/pose", extract_pose)
    df_joy  = read_topic(reader, TYPESTORE, "/joy", extract_joy)

# Verificación de datos extraídos
if df_pose.empty:
    print("ERROR: No se encontraron datos de Pose. Verifica el nombre del tópico.")
    sys.exit(1)

# Asignación de nombres de columnas
df_pose.columns = ["t", "x", "y", "z", "yaw"]
if not df_joy.empty:
    df_joy.columns  = ["t", "joy_throttle", "joy_steering"]

# Ordenar por tiempo
for df in [df_pose, df_joy]:
    if not df.empty:
        df.sort_values("t", inplace=True)
        df.reset_index(drop=True, inplace=True)

# Alinear tiempo para que comience en 0 (basado en el primer dato de Pose)
t0 = df_pose["t"].iloc[0]
df_pose["t"] -= t0
if not df_joy.empty:
    df_joy["t"] -= t0

# Guardar archivos CSV
# Guardar archivos CSV en la carpeta data
output_dir = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data")
# Guardar archivos CSV en la carpeta data
output_dir = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data")

df_pose.to_csv(output_dir / "pose_raw.csv", index=False)
if not df_joy.empty:
    df_joy.to_csv(output_dir / "joy_raw.csv", index=False)

# Resumen en consola
print("-" * 30)
print(f"  VICON: {len(df_pose)} muestras, dt={df_pose['t'].diff().median()*1000:.1f} ms")
if not df_joy.empty:
    print(f"  Joy:   {len(df_joy)} muestras, dt={df_joy['t'].diff().median()*1000:.1f} ms")
    print("Saved: pose_raw.csv, joy_raw.csv")
else:
    print("  Joy:   No se encontraron datos.")
    print("Saved: pose_raw.csv")