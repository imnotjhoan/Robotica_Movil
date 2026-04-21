#!/usr/bin/env python3
"""
madgwick_ahrs_node.py
─────────────────────────────────────────────────────────────────────────────
Nodo ROS 2 que implementa el filtro Madgwick AHRS (9-DOF) para estimar la
orientación de un vehículo en tiempo real a partir de:
  - /imu/data_raw  →  sensor_msgs/Imu   (giroscopio + acelerómetro)
  - /imu/mag       →  sensor_msgs/MagneticField  (magnetómetro)

Publica la orientación como:
  - /madgwick/pose         →  geometry_msgs/Pose   (para RViz2 / tf_node)
  - /madgwick/imu_filtered →  sensor_msgs/Imu      (orientación + covarianzas)

Uso rápido:
  ros2 run <tu_paquete> madgwick_ahrs_node
  ros2 run <tu_paquete> madgwick_ahrs_node \
        --ros-args -p beta:=0.05 -p sample_period:=0.05

Parámetros declarados:
  beta            (float, default 0.1)    – Ganancia del gradiente descendente.
                                            Aumentar → más corrección accel/mag,
                                            menos peso al giroscopio.
  sample_period   (float, default 0.05)   – Periodo nominal [s]. Solo se usa
                                            como respaldo; el nodo calcula Δt
                                            real desde los stamps del mensaje.
  imu_topic       (str)                   – Topic de datos crudos de la IMU.
  mag_topic       (str)                   – Topic del magnetómetro.
  pose_topic      (str)                   – Topic de salida Pose.
  imu_out_topic   (str)                   – Topic de salida Imu filtrado.
  mag_timeout_s   (float, default 0.5)    – Tiempo máximo sin datos de mag
                                            antes de ignorar su contribución.

Notas sobre unidades:
  - Acelerómetro: m/s²  → el filtro solo necesita la DIRECCIÓN (normaliza).
  - Magnetómetro: T o µT (REP-145) → solo la DIRECCIÓN importa (normaliza).
  - Giroscopio:   rad/s.

ROS REP-103 (marcos de referencia):
  - frame_id de la Pose publicada es el mismo que el de la IMU ('imu_link').
  - RViz2 visualiza la orientación si el tf_node republica la Pose como TF.
─────────────────────────────────────────────────────────────────────────────
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Pose


# ═══════════════════════════════════════════════════════════════════════════ #
#  Implementación del filtro Madgwick AHRS  (9-DOF)
# ═══════════════════════════════════════════════════════════════════════════ #

class MadgwickAHRS:
    """
    Filtro de orientación complementario basado en gradiente descendente.

    Referencia:
        S. Madgwick, "An efficient orientation filter for inertial and
        inertial/magnetic sensor arrays", Technical report, 2010.

    Convención interna de cuaternión: [w, x, y, z]  (escalar primero).
    """

    def __init__(self, sample_period: float = 1.0 / 20.0, beta: float = 0.1):
        """
        Parameters
        ----------
        sample_period : float
            Δt nominal entre actualizaciones [s]. Se usa como respaldo cuando
            el Δt calculado desde los timestamps no es válido.
        beta : float
            Ganancia del paso de gradiente descendente.
            • Alto  → converge rápido, más sensible a ruido de accel/mag.
            • Bajo  → suave, pero el yaw puede derivar si la mag es débil.
            Rango típico: 0.033 – 0.1 para IMUs de bajo coste.
        """
        self.sample_period = sample_period
        self.beta = beta
        # Cuaternión de orientación: [w, x, y, z], identidad al inicio
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # ------------------------------------------------------------------ #
    def update(self,
               gyroscope: np.ndarray,
               accelerometer: np.ndarray,
               magnetometer: np.ndarray,
               dt: float = None) -> None:
        """
        Ejecuta un paso completo del filtro Madgwick AHRS (9-DOF).

        Parameters
        ----------
        gyroscope     : (3,) array  – [gx, gy, gz]  rad/s
        accelerometer : (3,) array  – [ax, ay, az]  m/s²
        magnetometer  : (3,) array  – [mx, my, mz]  T / µT (se normaliza)
        dt            : float | None – Δt real [s]; None → usa sample_period
        """
        T = dt if (dt is not None and dt > 0) else self.sample_period
        q = self.quaternion.copy()
        gx, gy, gz = gyroscope
        ax, ay, az = accelerometer
        mx, my, mz = magnetometer

        # ── Normalizar acelerómetro ─────────────────────────────────────
        # Madgwick usa sólo la DIRECCIÓN de la gravedad. Si la norma es ≈0
        # (caída libre), no hay referencia; sólo se integra el giroscopio.
        norm_a = np.linalg.norm([ax, ay, az])
        if norm_a < 1e-10:
            self._integrate_gyro_only(gyroscope, T)
            return
        ax /= norm_a;  ay /= norm_a;  az /= norm_a

        # ── Normalizar magnetómetro ─────────────────────────────────────
        # Ídem: sólo la dirección del campo magnético importa.
        norm_m = np.linalg.norm([mx, my, mz])
        if norm_m < 1e-10:
            # Sin campo magnético: degradar a 6-DOF (roll+pitch, yaw deriva)
            self._update_6dof(gyroscope, np.array([ax, ay, az]), T)
            return
        mx /= norm_m;  my /= norm_m;  mz /= norm_m

        # ── Términos auxiliares (notación del paper de Madgwick) ────────
        q1, q2, q3, q4 = q        # w, x, y, z
        _2q1 = 2.0*q1;  _2q2 = 2.0*q2
        _2q3 = 2.0*q3;  _2q4 = 2.0*q4
        q1q1 = q1*q1;  q1q2 = q1*q2
        q1q3 = q1*q3;  q1q4 = q1*q4
        q2q2 = q2*q2;  q2q3 = q2*q3
        q2q4 = q2*q4;  q3q3 = q3*q3
        q3q4 = q3*q4;  q4q4 = q4*q4

        # ── Campo de referencia en marco Tierra (b) ─────────────────────
        # Se rota el vector mag medido al marco Tierra usando q actual y
        # se proyecta sólo en X (Norte) y Z (Abajo) → aísla el yaw del
        # heading magnético sin verse afectado por inclinación.
        #   h = q ⊗ [0, mx, my, mz] ⊗ q*
        hx = mx*(q1q1 + q2q2 - q3q3 - q4q4) \
           + 2.0*my*(q2q3 - q1q4) \
           + 2.0*mz*(q2q4 + q1q3)
        hy = 2.0*mx*(q2q3 + q1q4) \
           + my*(q1q1 - q2q2 + q3q3 - q4q4) \
           + 2.0*mz*(q3q4 - q1q2)
        hz = 2.0*mx*(q2q4 - q1q3) \
           + 2.0*my*(q3q4 + q1q2) \
           + mz*(q1q1 - q2q2 - q3q3 + q4q4)

        # bx = componente Norte (plano horizontal), bz = componente vertical
        bx = np.sqrt(hx*hx + hy*hy)
        bz = hz
        _2bx = 2.0*bx;  _2bz = 2.0*bz
        _4bx = 2.0*_2bx;  _4bz = 2.0*_2bz

        # ── Gradiente ∇f = Jᵀ · [f_accel ; f_mag] ─────────────────────
        # f_accel: error entre la gravedad estimada y la medida
        # f_mag  : error entre el campo magnético estimado y el medido
        # s1..s4 son las 4 componentes del gradiente (∂/∂q1 .. ∂/∂q4)
        s1 = (-_2q3*(2.0*q2q4 - 2.0*q1q3 - ax)
              + _2q2*(2.0*q1q2 + 2.0*q3q4 - ay)
              - _2bz*q3*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
              + (-_2bx*q4 + _2bz*q2)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
              + _2bx*q3*(_2bx*(q1q3 + q2q4) + _2bz*(0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4*(2.0*q2q4 - 2.0*q1q3 - ax)
              + _2q1*(2.0*q1q2 + 2.0*q3q4 - ay)
              - 4.0*q2*(1.0 - 2.0*q2q2 - 2.0*q3q3 - az)
              + _2bz*q4*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
              + (_2bx*q3 + _2bz*q1)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
              + (_2bx*q4 - _4bz*q2)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1*(2.0*q2q4 - 2.0*q1q3 - ax)
              + _2q4*(2.0*q1q2 + 2.0*q3q4 - ay)
              - 4.0*q3*(1.0 - 2.0*q2q2 - 2.0*q3q3 - az)
              + (-_4bx*q3 - _2bz*q1)*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
              + (_2bx*q2 + _2bz*q4)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
              + (_2bx*q1 - _4bz*q3)*(_2bx*(q1q3 + q2q4) + _2bz*(0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2*(2.0*q2q4 - 2.0*q1q3 - ax)
              + _2q3*(2.0*q1q2 + 2.0*q3q4 - ay)
              + (-_4bx*q4 + _2bz*q2)*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2q4 - q1q3) - mx)
              + (-_2bx*q1 + _2bz*q3)*(_2bx*(q2q3 - q1q4) + _2bz*(q1q2 + q3q4) - my)
              + _2bx*q2*(_2bx*(q1q3 + q2q4) + _2bz*(0.5 - q2q2 - q3q3) - mz))

        # Normalizar gradiente → paso de tamaño fijo independiente de la magnitud
        norm_s = np.linalg.norm([s1, s2, s3, s4])
        if norm_s > 1e-10:
            s1 /= norm_s;  s2 /= norm_s;  s3 /= norm_s;  s4 /= norm_s

        # ── Tasa de cambio del cuaternión ───────────────────────────────
        # q̇ = (1/2)·q⊗ω  −  β·∇f
        #   Término 1: cinemática del cuerpo rígido (integración del giroscopio)
        #   Término 2: corrección por acelerómetro + magnetómetro (Madgwick)
        q_dot = 0.5 * np.array([
            -q2*gx - q3*gy - q4*gz,    # dw/dt
             q1*gx + q3*gz - q4*gy,    # dx/dt
             q1*gy - q2*gz + q4*gx,    # dy/dt
             q1*gz + q2*gy - q3*gx     # dz/dt
        ]) - self.beta * np.array([s1, s2, s3, s4])

        # ── Integración Euler (1er orden) ───────────────────────────────
        # q_new = q_old + q̇·Δt
        # 1er orden es suficiente con Δt ≤ 0.02 s (error ∝ Δt²).
        q = q + q_dot * T

        # ── Renormalizar ────────────────────────────────────────────────
        # La integración numérica introduce pequeños errores en ‖q‖.
        # Renormalizar tras cada paso mantiene q en SO(3).
        self.quaternion = q / np.linalg.norm(q)

    # ------------------------------------------------------------------ #
    def _integrate_gyro_only(self, gyroscope: np.ndarray, dt: float) -> None:
        """Integra sólo el giroscopio (sin corrección). Deriva en yaw."""
        q = self.quaternion
        gx, gy, gz = gyroscope
        q_dot = 0.5 * np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
             q[0]*gx + q[2]*gz - q[3]*gy,
             q[0]*gy - q[1]*gz + q[3]*gx,
             q[0]*gz + q[1]*gy - q[2]*gx
        ])
        q = q + q_dot * dt
        self.quaternion = q / np.linalg.norm(q)

    # ------------------------------------------------------------------ #
    def _update_6dof(self,
                     gyroscope: np.ndarray,
                     acc_normalized: np.ndarray,
                     dt: float) -> None:
        """
        Actualización 6-DOF (sin magnetómetro).
        Estima roll y pitch con referencia gravitacional; yaw deriva.
        """
        q = self.quaternion.copy()
        gx, gy, gz = gyroscope
        ax, ay, az = acc_normalized      # ya normalizado por el llamador

        q2q2 = q[1]*q[1];  q3q3 = q[2]*q[2]

        s1 = 4.0*q[0]*q3q3 + 2.0*q[2]*ax + 4.0*q[0]*q2q2 - 2.0*q[1]*ay
        s2 = (4.0*q[1]*q[3]*q[3] - 2.0*q[3]*ax
              + 4.0*q[0]*q[0]*q[1] - 2.0*q[0]*ay
              - 4.0*q[1] + 8.0*q[1]*q2q2 + 8.0*q[1]*q3q3 + 4.0*q[1]*az)
        s3 = (4.0*q[0]*q[0]*q[2] + 2.0*q[0]*ax
              + 4.0*q[2]*q[3]*q[3] - 2.0*q[3]*ay
              - 4.0*q[2] + 8.0*q[2]*q2q2 + 8.0*q[2]*q3q3 + 4.0*q[2]*az)
        s4 = 4.0*q2q2*q[3] - 2.0*q[1]*ax + 4.0*q3q3*q[3] - 2.0*q[2]*ay

        norm_s = np.linalg.norm([s1, s2, s3, s4])
        if norm_s > 1e-10:
            s1 /= norm_s;  s2 /= norm_s;  s3 /= norm_s;  s4 /= norm_s

        q_dot = 0.5 * np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
             q[0]*gx + q[2]*gz - q[3]*gy,
             q[0]*gy - q[1]*gz + q[3]*gx,
             q[0]*gz + q[1]*gy - q[2]*gx
        ]) - self.beta * np.array([s1, s2, s3, s4])

        q = q + q_dot * dt
        self.quaternion = q / np.linalg.norm(q)

    # ------------------------------------------------------------------ #
    def get_euler_angles(self) -> tuple:
        """
        Convierte el cuaternión a ángulos de Euler (roll, pitch, yaw) [rad].
        Convención ZYX compatible con ROS REP-103.
        """
        q = self.quaternion        # [w, x, y, z]
        # Roll  (eje X)
        sinr_cosp = 2.0*(q[0]*q[1] + q[2]*q[3])
        cosr_cosp = 1.0 - 2.0*(q[1]**2 + q[2]**2)
        roll  = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch (eje Y)
        sinp  = 2.0*(q[0]*q[2] - q[3]*q[1])
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        # Yaw   (eje Z – heading)
        siny_cosp = 2.0*(q[0]*q[3] + q[1]*q[2])
        cosy_cosp = 1.0 - 2.0*(q[2]**2 + q[3]**2)
        yaw   = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def get_quaternion_ros(self) -> tuple:
        """
        Devuelve el cuaternión en orden ROS (x, y, z, w).
        El filtro usa [w, x, y, z] internamente.
        """
        w, x, y, z = self.quaternion
        return x, y, z, w


# ═══════════════════════════════════════════════════════════════════════════ #
#  Nodo ROS 2
# ═══════════════════════════════════════════════════════════════════════════ #

class MadgwickNode(Node):
    """
    Nodo ROS 2 que envuelve el filtro Madgwick AHRS.

    Suscripciones
    ─────────────
      /imu/data_raw   sensor_msgs/Imu          – giroscopio + acelerómetro
      /imu/mag        sensor_msgs/MagneticField – magnetómetro

    Publicaciones
    ─────────────
      /madgwick/pose         geometry_msgs/Pose  → RViz2 / tf_node
      /madgwick/imu_filtered sensor_msgs/Imu     → orientación + covarianzas
    """

    def __init__(self):
        super().__init__('madgwick_ahrs_node')

        # ── Parámetros ──────────────────────────────────────────────────
        self.declare_parameter('beta',           0.1)
        self.declare_parameter('sample_period',  0.05)     # 20 Hz por defecto
        self.declare_parameter('imu_topic',      '/imu/data_raw')
        self.declare_parameter('mag_topic',      '/imu/mag')
        self.declare_parameter('pose_topic',     '/madgwick/pose')
        self.declare_parameter('imu_out_topic',  '/madgwick/imu_filtered')
        # Máximo tiempo sin dato de magnetómetro antes de usar modo 6-DOF
        self.declare_parameter('mag_timeout_s',  0.5)

        beta          = self.get_parameter('beta').value
        sample_period = self.get_parameter('sample_period').value
        imu_topic     = self.get_parameter('imu_topic').value
        mag_topic     = self.get_parameter('mag_topic').value
        pose_topic    = self.get_parameter('pose_topic').value
        imu_out_topic = self.get_parameter('imu_out_topic').value
        self._mag_timeout = self.get_parameter('mag_timeout_s').value

        # ── Filtro ──────────────────────────────────────────────────────
        self._filter = MadgwickAHRS(sample_period=sample_period, beta=beta)

        # ── Estado interno ──────────────────────────────────────────────
        self._last_mag        = None   # np.ndarray | None – último vector mag
        self._last_mag_stamp  = None   # float | None – timestamp en segundos
        self._last_imu_stamp  = None   # float | None – para calcular Δt real
        self._initialized     = False  # skip la primera llegada (sin Δt)

        # ── Publicadores ────────────────────────────────────────────────
        self._pose_pub = self.create_publisher(Pose, pose_topic,     10)
        self._imu_pub  = self.create_publisher(Imu,  imu_out_topic,  10)

        # ── Suscriptores ────────────────────────────────────────────────
        # El magnetómetro llega a menor frecuencia que la IMU.
        # Se guarda la última lectura y se usa en cada ciclo de la IMU.
        self._mag_sub = self.create_subscription(
            MagneticField, mag_topic, self._mag_callback, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, 10)

        self.get_logger().info(
            f"[Madgwick AHRS] Iniciado\n"
            f"  β={beta}  |  T_nom={sample_period:.4f}s\n"
            f"  IMU   ← '{imu_topic}'\n"
            f"  MAG   ← '{mag_topic}'\n"
            f"  Pose  → '{pose_topic}'\n"
            f"  Imu   → '{imu_out_topic}'"
        )

    # ------------------------------------------------------------------ #
    #  Callback del magnetómetro
    # ------------------------------------------------------------------ #
    def _mag_callback(self, msg: MagneticField) -> None:
        """
        Almacena la lectura más reciente del magnetómetro.

        El mag suele publicarse a ≤20 Hz y la IMU a ≥50 Hz.
        El filtro toma prestada la última muestra de mag disponible,
        introduciendo un retardo de ≤1 periodo de mag (< 50 ms a 20 Hz),
        lo cual es completamente aceptable para estimación de heading.
        """
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        norm_m = np.linalg.norm([mx, my, mz])
        if norm_m > 1e-10:
            self._last_mag       = np.array([mx, my, mz], dtype=np.float64)
            self._last_mag_stamp = self._stamp_to_sec(msg.header.stamp)

    # ------------------------------------------------------------------ #
    #  Callback de la IMU  (gatillo principal)
    # ------------------------------------------------------------------ #
    def _imu_callback(self, msg: Imu) -> None:
        """
        Invoca el filtro y publica la orientación estimada.

        Flujo por mensaje:
          1. Extraer giroscopio y acelerómetro.
          2. Calcular Δt real desde el stamp anterior.
          3. Decidir modo: 9-DOF (con mag) o 6-DOF / gyro-only.
          4. Actualizar el filtro.
          5. Publicar geometry_msgs/Pose y sensor_msgs/Imu filtrado.
        """
        # ── 1. Extraer mediciones ──────────────────────────────────────
        gyro  = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float64)

        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float64)

        current_stamp = self._stamp_to_sec(msg.header.stamp)

        # ── 2. Calcular Δt real ────────────────────────────────────────
        if not self._initialized:
            # Primera llegada: no hay stamp previo → saltar, guardar stamp
            self._last_imu_stamp = current_stamp
            self._initialized    = True
            return

        dt = current_stamp - self._last_imu_stamp
        self._last_imu_stamp = current_stamp

        if dt <= 0.0 or dt > 1.0:
            # Stamp inválido (reinicio del reloj, lag grande): usar nominal
            self.get_logger().warn(
                f"Δt fuera de rango ({dt:.4f} s) → usando periodo nominal.",
                throttle_duration_sec=5.0)
            dt = self._filter.sample_period

        # ── 3. Determinar disponibilidad del magnetómetro ─────────────
        use_mag = False
        mag     = np.zeros(3, dtype=np.float64)

        if self._last_mag is not None and self._last_mag_stamp is not None:
            mag_age = current_stamp - self._last_mag_stamp
            if mag_age <= self._mag_timeout:
                use_mag = True
                mag     = self._last_mag.copy()
            else:
                self.get_logger().warn(
                    f"Mag obsoleto ({mag_age:.2f} s > {self._mag_timeout} s)."
                    " Modo 6-DOF activado (yaw derivará).",
                    throttle_duration_sec=5.0)

        # ── 4. Actualizar filtro ───────────────────────────────────────
        if use_mag:
            # 9-DOF: giroscopio + acelerómetro + magnetómetro
            self._filter.update(gyro, accel, mag, dt=dt)
        else:
            # Degradar a 6-DOF o sólo giroscopio
            accel_norm = np.linalg.norm(accel)
            if accel_norm > 1e-10:
                self._filter._update_6dof(gyro, accel / accel_norm, dt)
            else:
                self._filter._integrate_gyro_only(gyro, dt)

        # ── 5. Publicar resultados ─────────────────────────────────────
        qx, qy, qz, qw      = self._filter.get_quaternion_ros()
        roll, pitch, yaw    = self._filter.get_euler_angles()

        # ─ 5a. geometry_msgs/Pose  (usado por tf_node / RViz2) ─────────
        # La posición queda en el origen (0,0,0). El tf_node puede agregar
        # odometría de posición sobre esta orientación si lo requiere.
        pose_msg = Pose()
        pose_msg.position.x    = 0.0
        pose_msg.position.y    = 0.0
        pose_msg.position.z    = 0.0
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        self._pose_pub.publish(pose_msg)

        # ─ 5b. sensor_msgs/Imu filtrado (para otros nodos / diagnóstico) ─
        imu_out = Imu()
        imu_out.header          = msg.header          # mismo frame_id y stamp
        imu_out.orientation.x   = qx
        imu_out.orientation.y   = qy
        imu_out.orientation.z   = qz
        imu_out.orientation.w   = qw
        # Covarianza de orientación: diagonal pequeña (confianza del filtro)
        imu_out.orientation_covariance = [
            1e-4, 0.0,  0.0,
            0.0,  1e-4, 0.0,
            0.0,  0.0,  1e-4
        ]
        # Repasar mediciones crudas sin modificar
        imu_out.angular_velocity              = msg.angular_velocity
        imu_out.angular_velocity_covariance   = msg.angular_velocity_covariance
        imu_out.linear_acceleration           = msg.linear_acceleration
        imu_out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._imu_pub.publish(imu_out)

        # Log de depuración periódico (nivel DEBUG → no satura consola)
        self.get_logger().debug(
            f"Roll={np.degrees(roll):+7.2f}°  "
            f"Pitch={np.degrees(pitch):+7.2f}°  "
            f"Yaw={np.degrees(yaw):+7.2f}°  "
            f"Δt={dt*1000:.1f}ms  9-DOF={use_mag}"
        )

    # ------------------------------------------------------------------ #
    #  Utilidades
    # ------------------------------------------------------------------ #
    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        """Convierte un builtin_interfaces/Time a segundos float."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ═══════════════════════════════════════════════════════════════════════════ #
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════ #

def main(args=None):
    rclpy.init(args=args)
    node = MadgwickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()