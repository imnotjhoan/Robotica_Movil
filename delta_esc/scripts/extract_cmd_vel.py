#!/usr/bin/env python3
"""
extract_cmd_vel_full.py  --  Lee un rosbag y exporta cmd_vel a CSV con
                              columnas de porcentaje para linear_x y angular_z.

Uso:
    python3 extract_cmd_vel_full.py <ruta_al_bag>

Ejemplo:
    python3 extract_cmd_vel_full.py ~/bags/sysid_run_28_vel

Columnas generadas:
  - linear_x_pct : porcentaje real de velocidad lineal, extraído del nombre
                   del bag (e.g. "sysid_run_28_vel" → ±28 %)
                   Fórmula: (linear_x / 3) * pct_archivo

  - angular_z_pct: porcentaje de velocidad angular sobre escala fija ±100 %
                   donde +3 = +100 % y -3 = -100 %
                   Fórmula: (angular_z / 3) * 100
"""

import re
import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# ---------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------
BAG_PATH  = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

CMD_VEL_TOPIC = "/cmd_vel"

OUTPUT_DIR = Path("/home/jhoan/mrad_ws_2601_delta2/src/delta_esc/scripts/data/cmd_vel")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

bag_name = BAG_PATH.name

# Valor máximo de linear_x y angular_z enviado desde el PC (siempre ±3)
LINEAR_X_MAX  = 3.0
ANGULAR_Z_MAX = 3.0

# ---------------------------------------------------------------------------
# Extraer porcentaje de velocidad lineal del nombre del bag
# Soporta: sysid_run_28_vel, circulo_ocho_22_vel, cualquier_nombre_NN_vel
# ---------------------------------------------------------------------------

def extract_pct_from_name(name: str) -> float:
    """
    Busca el patrón '<número>_vel' en el nombre del bag.
    Devuelve el número como float (e.g. 28.0), o NaN si no se encuentra.
    """
    match = re.search(r"(\d+)_vel", name)
    if match:
        pct = float(match.group(1))
        print(f"  [INFO] Porcentaje de velocidad lineal detectado: {pct:.0f} %")
        return pct
    else:
        print(f"  [WARN] No se encontró patrón '<N>_vel' en '{name}'. "
              f"'linear_x_pct' quedará como NaN.")
        return float("nan")

vel_pct_file = extract_pct_from_name(bag_name)

# ---------------------------------------------------------------------------
# Helper de lectura
# ---------------------------------------------------------------------------

def read_topic(reader, typestore, topic_name, fields_fn):
    """Itera sobre los mensajes de un topic y devuelve un DataFrame."""
    rows = []
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        print(f"  [WARN] Topic '{topic_name}' no encontrado en el bag.")
        return pd.DataFrame()

    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        t = timestamp * 1e-9          # ns → s
        rows.append((t,) + fields_fn(msg))

    print(f"  [OK] '{topic_name}': {len(rows)} mensajes leídos.")
    return pd.DataFrame(rows)


def extract_cmd_vel(msg):
    """
    Extrae campos de geometry_msgs/TwistStamped o Twist plano.
    No incluye frame_id ni t_msg.
    """
    try:
        twist = msg.twist
    except AttributeError:
        twist = msg

    lx = float(twist.linear.x)
    ly = float(twist.linear.y)
    lz = float(twist.linear.z)
    ax = float(twist.angular.x)
    ay = float(twist.angular.y)
    az = float(twist.angular.z)

    return (lx, ly, lz, ax, ay, az)


# ---------------------------------------------------------------------------
# Lectura del bag
# ---------------------------------------------------------------------------

with Reader(BAG_PATH) as reader:
    print(f"\nBag: {BAG_PATH}")
    print(f"Topics disponibles: {[c.topic for c in reader.connections]}\n")

    df_cmd = read_topic(reader, TYPESTORE, CMD_VEL_TOPIC, extract_cmd_vel)

if df_cmd.empty:
    print(f"\nERROR: No se encontraron datos en '{CMD_VEL_TOPIC}'. "
          f"Verifica el nombre del topic.")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Columnas y limpieza
# ---------------------------------------------------------------------------

df_cmd.columns = [
    "t_bag",
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
]

# t = 0 en el primer mensaje
t0 = df_cmd["t_bag"].min()
df_cmd["t_bag"] -= t0

# ---------------------------------------------------------------------------
# Columna linear_x_pct
#
#   linear_x  |  linear_x_pct (ejemplo archivo _28_vel)
#   ──────────┼──────────────────────────────────────────
#      +3      |  +28.0 %
#       0      |    0.0 %
#      -3      |  -28.0 %
#
# Fórmula: linear_x_pct = (linear_x / LINEAR_X_MAX) * vel_pct_file
# ---------------------------------------------------------------------------

df_cmd["linear_x_pct"] = (df_cmd["linear_x"] / LINEAR_X_MAX) * vel_pct_file

# ---------------------------------------------------------------------------
# Columna angular_z_pct
#
#   angular_z  |  angular_z_pct
#   ───────────┼────────────────
#      +3       |  +100.0 %
#       0       |     0.0 %
#      -3       |  -100.0 %
#
# Fórmula: angular_z_pct = (angular_z / ANGULAR_Z_MAX) * 100
# ---------------------------------------------------------------------------

df_cmd["angular_z_pct"] = (df_cmd["angular_z"] / ANGULAR_Z_MAX) * 100.0

# ---------------------------------------------------------------------------
# Guardar CSV
# ---------------------------------------------------------------------------

raw_path = OUTPUT_DIR / f"{bag_name}_input.csv"
df_cmd.to_csv(raw_path, index=False)

# ---------------------------------------------------------------------------
# Resumen
# ---------------------------------------------------------------------------

print(f"\n[RAW]  Guardado en : {raw_path}")
print(f"       Muestras    : {len(df_cmd)}")
print(f"       Columnas    : {list(df_cmd.columns)}")

print("\n── linear_x_pct ──────────────────────────────────────")
if not np.isnan(vel_pct_file):
    print(f"  Porcentaje del archivo : {vel_pct_file:.0f} %")
    print(f"  Valores únicos         : {sorted(df_cmd['linear_x_pct'].unique().tolist())}")
else:
    print("  Porcentaje no detectado en el nombre del archivo.")

print("\n── angular_z_pct ─────────────────────────────────────")
print(f"  Escala fija ±100 %")
print(f"  Valores únicos : {sorted(df_cmd['angular_z_pct'].unique().tolist())}")

print("\n" + "-" * 55)
print("Completado exitosamente.")
print(f"  RAW → {raw_path}")
print("-" * 55)
print("\nVista previa (primeras 8 filas):")
print(df_cmd.head(8).to_string(index=False))