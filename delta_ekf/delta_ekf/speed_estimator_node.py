#!/usr/bin/env python3
"""
vehicle_sim_node.py
====================
Nodo ROS 2 que:
  1. Se suscribe a /cmd_vel (geometry_msgs/msg/TwistStamped).
  2. Convierte linear.x → porcentaje: pct = (v / 9.8) * 100.
  3. Selecciona el modelo FOPDT (primer orden + tiempo muerto) más cercano
     al punto de operación (20 / 22 / 25 / 28 %).
  4. Simula la dinámica del vehículo en tiempo discreto.
  5. Integra la velocidad estimada → posición x.
  6. Publica geometry_msgs/msg/Pose en /robot1/pose a 20 Hz.

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

OPERATING_POINTS = sorted(MODELS.keys())          # [20, 22, 25, 28]
MAX_BUFFER_SECONDS = 5.0                          # ventana máxima del buffer


def select_model(percentage: float) -> dict:
    """Devuelve el modelo cuyo punto de operación es el más cercano al porcentaje dado."""
    nearest = min(OPERATING_POINTS, key=lambda op: abs(op - percentage))
    return MODELS[nearest]


def now_sec(node: Node) -> float:
    """Tiempo actual del nodo en segundos (float)."""
    return node.get_clock().now().nanoseconds * 1e-9


# ---------------------------------------------------------------------------
# Nodo principal
# ---------------------------------------------------------------------------
class VehicleSimNode(Node):
    def __init__(self):
        super().__init__("vehicle_sim_node")

        # ── Parámetros declarados (ajustables desde CLI o launch) ──────────
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_topic",  "cmd_vel")
        self.declare_parameter("pose_topic",     "/robot1/pose")

        update_hz   = self.get_parameter("update_rate_hz").value
        cmd_topic   = self.get_parameter("cmd_vel_topic").value
        pose_topic  = self.get_parameter("pose_topic").value

        self.dt = 1.0 / update_hz         # paso de simulación [s]

        # ── Subscriber ─────────────────────────────────────────────────────
        self.sub_cmd = self.create_subscription(
            TwistStamped,
            cmd_topic,
            self._cmd_vel_cb,
            10,
        )

        # ── Publisher ──────────────────────────────────────────────────────
        self.pub_pose = self.create_publisher(Pose, pose_topic, 10)

        # ── Estado del simulador ───────────────────────────────────────────
        self.velocity: float = 0.0        # velocidad estimada [m/s]
        self.position_x: float = 0.0     # posición acumulada [m]

        # Buffer de entradas con marca de tiempo para implementar el tiempo muerto
        # Cada elemento: (timestamp_s: float, percentage: float)
        self.input_buffer: deque[tuple[float, float]] = deque()

        # Última entrada recibida (sin retardo)
        self.current_pct: float = 0.0

        # ── Timer de simulación ────────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._update)

        self.get_logger().info(
            f"[vehicle_sim_node] iniciado  |  "
            f"dt={self.dt:.3f} s  |  "
            f"cmd='{cmd_topic}'  |  pose='{pose_topic}'"
        )

    # -----------------------------------------------------------------------
    # Callback: recepción de cmd_vel
    # -----------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        linear_x = msg.twist.linear.x          # velocidad de referencia [m/s]
        pct = (linear_x / 9.8) * 100.0        # porcentaje de la referencia

        self.current_pct = pct

        # Guardar en buffer con timestamp actual
        t = now_sec(self)
        self.input_buffer.append((t, pct))

        # Descartar entradas más antiguas de MAX_BUFFER_SECONDS
        cutoff = t - MAX_BUFFER_SECONDS
        while self.input_buffer and self.input_buffer[0][0] < cutoff:
            self.input_buffer.popleft()

    # -----------------------------------------------------------------------
    # Helpers del buffer de tiempo muerto
    # -----------------------------------------------------------------------
    def _get_delayed_input(self, theta: float) -> float:
        """
        Devuelve la entrada que ocurrió hace 'theta' segundos (tiempo muerto).
        Si no hay datos suficientemente antiguos, devuelve 0.
        """
        t_target = now_sec(self) - theta

        delayed = 0.0
        for t, u in self.input_buffer:
            if t <= t_target:
                delayed = u          # tomamos el más reciente ≤ t_target
            else:
                break                # el buffer está en orden cronológico
        return delayed

    # -----------------------------------------------------------------------
    # Loop de simulación a tasa fija
    # -----------------------------------------------------------------------
    def _update(self) -> None:
        # 1. Seleccionar modelo según el porcentaje actual
        model = select_model(self.current_pct)
        K      = model["K"]
        tau    = model["tau"]
        u_dead = model["u_dead"]
        theta  = model["theta"]

        # 2. Obtener entrada con tiempo muerto
        u_delayed = self._get_delayed_input(theta)

        # 3. Restar zona muerta (dead-band); la velocidad no puede ser negativa
        u_eff = max(0.0, u_delayed - u_dead)

        # 4. Discretización exacta de primer orden (ZOH):
        #    y[k] = e^(-dt/τ) · y[k-1] + K·(1 - e^(-dt/τ)) · u_eff
        alpha = math.exp(-self.dt / tau)
        v_target = K * u_eff
        self.velocity = alpha * self.velocity + (1.0 - alpha) * v_target
        
        self.get_logger().info(
            f"vel={self.velocity:.3f} m/s |  "
            f"position='{self.position_x:.3f} m'  |  "
            f"porcentajel={self.current_pct:.2f}%  |  "
        )

        # 5. Integrar velocidad → posición (Euler hacia adelante)
        self.position_x += self.velocity * self.dt

        # 6. Publicar Pose
        self._publish_pose()

        self.get_logger().debug(
            f"pct={self.current_pct:.2f}%  "
            f"u_eff={u_eff:.2f}%  "
            f"v={self.velocity:.4f} m/s  "
            f"x={self.position_x:.4f} m"
        )

    # -----------------------------------------------------------------------
    # Publicar geometry_msgs/Pose
    # -----------------------------------------------------------------------
    def _publish_pose(self) -> None:
        msg = Pose()
        msg.position.x = self.position_x
        msg.position.y = 0.0
        msg.position.z = 0.0
        # Orientación neutra (sin rotación)
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