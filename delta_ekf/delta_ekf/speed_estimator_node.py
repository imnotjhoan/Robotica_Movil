#!/usr/bin/env python3
"""
vehicle_sim_node.py
====================
Nodo ROS 2 que:
  - Simula dinámica FOPDT bidireccional (forward + reverse)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped

# ---------------------------------------------------------------------------
# Modelos FOPDT (forward)
# ---------------------------------------------------------------------------
MODELS: dict[int, dict] = {
    20: {"K": 0.19466, "tau": 0.16348, "u_dead": 19.002, "theta": 2.1170},
    22: {"K": 0.13066, "tau": 0.16348, "u_dead": 19.002, "theta": 1.5170},
    25: {"K": 0.20662, "tau": 0.17514, "u_dead": 17.677, "theta": 1.5617},
    28: {"K": 0.06253, "tau": 0.50192, "u_dead": 0.000,  "theta": 1.2233},
}

OPERATING_POINTS = sorted(MODELS.keys())
MAX_BUFFER_SECONDS = 4.0


def select_model(percentage: float) -> dict:
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

        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_topic",  "cmd_vel")
        self.declare_parameter("pose_topic",     "/robot1/pose")

        update_hz   = self.get_parameter("update_rate_hz").value
        cmd_topic   = self.get_parameter("cmd_vel_topic").value
        pose_topic  = self.get_parameter("pose_topic").value

        self.dt = 1.0 / update_hz

        # Subs / pubs
        self.sub_cmd = self.create_subscription(
            TwistStamped,
            cmd_topic,
            self._cmd_vel_cb,
            10,
        )

        self.pub_pose = self.create_publisher(Pose, pose_topic, 10)

        # Estado
        self.velocity: float = 0.0
        self.position_x: float = 0.0

        self.input_buffer: deque[tuple[float, float]] = deque()
        self.current_pct: float = 0.0

        self.timer = self.create_timer(self.dt, self._update)

        self.get_logger().info("vehicle_sim_node iniciado")

    # -----------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: TwistStamped) -> None:
        linear_x = msg.twist.linear.x
        pct = (linear_x / 9.8) * 100.0

        self.current_pct = pct

        t = now_sec(self)
        self.input_buffer.append((t, pct))

        cutoff = t - MAX_BUFFER_SECONDS
        while self.input_buffer and self.input_buffer[0][0] < cutoff:
            self.input_buffer.popleft()

    # -----------------------------------------------------------------------
    def _get_delayed_input(self, theta: float) -> float:
        t_target = now_sec(self) - theta

        delayed = 0.0
        for t, u in self.input_buffer:
            if t <= t_target:
                delayed = u
            else:
                break
        return delayed

    # -----------------------------------------------------------------------
    def _update(self) -> None:
        # 1. Modelo usando magnitud
        model = select_model(abs(self.current_pct))
        K      = model["K"]
        tau    = model["tau"]
        u_dead = model["u_dead"]
        theta  = model["theta"]

        # 2. Input con retardo
        u_delayed = self._get_delayed_input(theta)

        # 3. Manejo correcto de signo + zona muerta
        sign = 1.0 if u_delayed >= 0.0 else -1.0
        u_mag = abs(u_delayed)

        u_eff_mag = max(0.0, u_mag - u_dead)
        u_eff = sign * u_eff_mag

        # 4. Dinámica FOPDT discreta
        alpha = math.exp(-self.dt / tau)
        v_target = K * u_eff
        self.velocity = alpha * self.velocity + (1.0 - alpha) * v_target

        # 5. Integración
        self.position_x += self.velocity * self.dt

        # 6. Debug útil
        self.get_logger().info(
            f"u_del={u_delayed:.2f}% | "
            f"u_eff={u_eff:.2f}% | "
            f"vel={self.velocity:.3f} m/s | "
            f"x={self.position_x:.3f} m | "
            f"pct={self.current_pct:.2f}%"
        )

        # 7. Publicar
        self._publish_pose()

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