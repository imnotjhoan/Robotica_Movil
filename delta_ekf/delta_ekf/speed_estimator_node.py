#!/usr/bin/env python3
"""
vehicle_sim_node.py  (v2 – corregido sobreestimación de distancia)
====================================================================
Correcciones respecto a v1:

  1. Selección del modelo basada en u_delayed (entrada retardada), no en
     current_pct, para que los parámetros K/τ/θ sean coherentes con la
     entrada que realmente mueve el sistema.

  2. Dinámica ASIMÉTRICA:
       • Aceleración   → FOPDT normal (ZOH exacto).
       • Desaceleración→ decaimiento más rápido controlado por
         'braking_factor' (τ_eff = τ / braking_factor).
         Si current_pct ≈ 0, el objetivo se fuerza a 0 directamente,
         ignorando el buffer de tiempo muerto.

  3. La velocidad se acota a v_ss = K * u_eff para que el tiempo muerto
     nunca acumule error por encima del régimen permanente del modelo.

Modelos FOPDT identificados:
  G(s) = K · e^(−θ·s) / (τ·s + 1)

  %   K [m/s/%]   τ [s]    u_dead [%]  θ [s]
  20  0.19466     0.16348  19.002      2.1170
  22  0.19466     0.16348  19.002      2.1170
  25  0.21662     0.17514  17.677      1.5617
  28  0.08083     0.50192   0.000      1.2233
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped

# ---------------------------------------------------------------------------
# Modelos FOPDT por punto de operación
# ---------------------------------------------------------------------------
MODELS: dict[int, dict] = {
    20: {"K": 0.19466, "tau": 0.16348, "u_dead": 19.002, "theta": 2.1170},
    22: {"K": 0.19466, "tau": 0.16348, "u_dead": 19.002, "theta": 2.1170},
    25: {"K": 0.21662, "tau": 0.17514, "u_dead": 17.677, "theta": 1.5617},
    28: {"K": 0.08083, "tau": 0.50192, "u_dead": 0.000,  "theta": 1.2233},
}

OPERATING_POINTS = sorted(MODELS.keys())   # [20, 22, 25, 28]
MAX_BUFFER_SECONDS = 5.0                   # ventana máxima del buffer


def select_model(percentage: float) -> dict:
    """Modelo cuyo punto de operación es el más cercano al porcentaje dado."""
    nearest = min(OPERATING_POINTS, key=lambda op: abs(op - percentage))
    return MODELS[nearest]


def now_sec(node: Node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


# ---------------------------------------------------------------------------
# Nodo principal
# ---------------------------------------------------------------------------
class VehicleSimNode(Node):

    def __init__(self):
        super().__init__("vehicle_sim_node")

        # ── Parámetros ──────────────────────────────────────────────────────
        self.declare_parameter("update_rate_hz",  20.0)
        self.declare_parameter("cmd_vel_topic",   "cmd_vel")
        self.declare_parameter("pose_topic",      "/robot1/pose")
        # Factor por el que se divide τ al frenar. Ajustar según el vehículo real.
        # 1.0 = simétrico (sin corrección), 4.0 = frena 4x más rápido que acelera.
        self.declare_parameter("braking_factor",  4.0)
        # Umbral de porcentaje bajo el cual se considera "acelerador suelto"
        self.declare_parameter("zero_cmd_threshold_pct", 1.0)

        update_hz           = self.get_parameter("update_rate_hz").value
        cmd_topic           = self.get_parameter("cmd_vel_topic").value
        pose_topic          = self.get_parameter("pose_topic").value
        self.braking_factor = self.get_parameter("braking_factor").value
        self.zero_threshold = self.get_parameter("zero_cmd_threshold_pct").value

        self.dt = 1.0 / update_hz

        # ── Subscriber / Publisher ──────────────────────────────────────────
        self.sub_cmd = self.create_subscription(
            TwistStamped, cmd_topic, self._cmd_vel_cb, 10
        )
        self.pub_pose = self.create_publisher(Pose, pose_topic, 10)

        # ── Estado interno ──────────────────────────────────────────────────
        self.velocity: float = 0.0
        self.position_x: float = 0.0

        # Buffer cronológico: (timestamp_s, porcentaje)
        self.input_buffer: deque[tuple[float, float]] = deque()
        self.current_pct: float = 0.0   # comando más reciente (sin retardo)

        # ── Timer ───────────────────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._update)

        self.get_logger().info(
            f"[vehicle_sim_node] v2 iniciado | dt={self.dt:.3f} s | "
            f"cmd='{cmd_topic}' | pose='{pose_topic}' | "
            f"braking_factor={self.braking_factor}"
        )

    # -----------------------------------------------------------------------
    # Callback suscriptor
    # -----------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        pct = (msg.twist.linear.x / 9.8) * 100.0
        self.current_pct = pct

        t = now_sec(self)
        self.input_buffer.append((t, pct))

        # Limpiar entradas demasiado antiguas
        cutoff = t - MAX_BUFFER_SECONDS
        while self.input_buffer and self.input_buffer[0][0] < cutoff:
            self.input_buffer.popleft()

    # -----------------------------------------------------------------------
    # Buffer de tiempo muerto
    # -----------------------------------------------------------------------
    def _get_delayed_input(self, theta: float) -> float:
        """
        Entrada que llegó hace exactamente 'theta' segundos (tiempo muerto).
        Retorna 0 si no hay entradas suficientemente antiguas en el buffer.
        """
        t_target = now_sec(self) - theta
        delayed = 0.0
        for t, u in self.input_buffer:
            if t <= t_target:
                delayed = u    # el más reciente con t <= t_target
            else:
                break          # buffer en orden cronológico
        return delayed

    # -----------------------------------------------------------------------
    # Loop de simulación
    # -----------------------------------------------------------------------
    def _update(self) -> None:

        # ── PASO 1: Pre-selección con θ máximo para obtener u_delayed_ref ──
        # Usamos el θ mayor de todos los modelos para tener una estimación
        # conservadora de la entrada retardada con la que elegir el modelo.
        max_theta = max(m["theta"] for m in MODELS.values())
        u_delayed_ref = self._get_delayed_input(max_theta)

        # ── PASO 2: Seleccionar modelo basado en u_delayed_ref ──────────────
        # Coherencia: los parámetros K/τ/θ corresponden al mismo punto de
        # operación que generó los datos retardados.
        model  = select_model(u_delayed_ref)
        K      = model["K"]
        tau    = model["tau"]
        u_dead = model["u_dead"]
        theta  = model["theta"]

        # ── PASO 3: Entrada retardada con el θ correcto del modelo elegido ──
        u_delayed = self._get_delayed_input(theta)

        # ── PASO 4: Zona muerta ─────────────────────────────────────────────
        u_eff = max(0.0, u_delayed - u_dead)

        # ── PASO 5: Velocidad de régimen permanente ─────────────────────────
        v_ss = K * u_eff

        # ── PASO 6: Detectar modo aceleración vs frenado ────────────────────
        #
        # Se considera frenado cuando:
        #   a) El comando ACTUAL es prácticamente 0 (acelerador suelto), o
        #   b) La velocidad actual ya supera el objetivo v_ss (desaceleración).
        #
        # En caso (a) el objetivo se fuerza a 0 directamente, sin esperar a
        # que el buffer de tiempo muerto drene — esto es lo que evita la
        # mayor parte de la sobreestimación de distancia.
        driver_released = self.current_pct < self.zero_threshold

        if driver_released:
            v_target = 0.0
            tau_eff  = tau / self.braking_factor
            is_braking = True
        elif v_ss < self.velocity:
            v_target = v_ss
            tau_eff  = tau / self.braking_factor
            is_braking = True
        else:
            v_target = v_ss
            tau_eff  = tau
            is_braking = False

        # ── PASO 7: Discretización exacta (ZOH) ────────────────────────────
        alpha = math.exp(-self.dt / tau_eff)
        self.velocity = alpha * self.velocity + (1.0 - alpha) * v_target

        # Garantizar que la velocidad no sea negativa
        self.velocity = max(0.0, self.velocity)

        # ── PASO 8: Acotar al régimen permanente durante aceleración ────────
        # Impide que el buffer de tiempo muerto acumule velocidad por encima
        # de lo que el modelo dice que puede alcanzar con u_eff actual.
        if not is_braking and v_ss > 0.0:
            self.velocity = min(self.velocity, v_ss)

        # ── PASO 9: Integrar posición ────────────────────────────────────────
        self.position_x += self.velocity * self.dt

        # ── PASO 10: Publicar ────────────────────────────────────────────────
        self._publish_pose()

        self.get_logger().info(
            f"pct={self.current_pct:.2f}%  "
            f"u_del={u_delayed:.2f}%  "
            f"u_eff={u_eff:.2f}%  "
            f"v={self.velocity:.4f} m/s  "
            f"x={self.position_x:.4f} m  "
            f"{'[FRENO]' if is_braking else '[ACEL]'}"
        )

    # -----------------------------------------------------------------------
    # Publicar geometry_msgs/Pose
    # -----------------------------------------------------------------------
    def _publish_pose(self) -> None:
        msg = Pose()
        msg.position.x = self.position_x
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        self.pub_pose.publish(msg)


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()