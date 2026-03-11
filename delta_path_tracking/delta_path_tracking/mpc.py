#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple, Dict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

# ImportaciÃ³n crucial para el MPC
from scipy.optimize import minimize
import numpy as np
from typing import Tuple, Dict

def euler_from_quaternion(q) -> float:
    """
    FunciÃ³n auxiliar para convertir un cuaterniÃ³n de ROS2 a Ã¡ngulos de Euler.
    Retorna Ãºnicamente el Ã¡ngulo Yaw (rotaciÃ³n sobre el eje Z).
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MpcControllerNode(Node):
    """
    Controlador MPC CinemÃ¡tico para robot de accionamiento diferencial.
    Utiliza CasADi + IPOPT para la optimizaciÃ³n en tiempo real.
    """

    def __init__(self):
        super().__init__("mpc_controller_node")
        # TÃ³picos y Frames (Reciclado de Pure Pursuit)
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("debug_topic","~/debug/mpc_prediction")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("goal_tolerance", 0.1)
        self.declare_parameter("tf_timeout_sec", 0.2)

        # ParÃ¡metros del Horizonte de PredicciÃ³n (Nuevos para MPC)
        self.declare_parameter("mpc_N", 10)         # Horizonte de predicciÃ³n (pasos)
        self.declare_parameter("mpc_dt", 0.1)       # Tiempo de muestreo (segundos)

        # Restricciones CinemÃ¡ticas (LÃ­mites de los actuadores)
        self.declare_parameter("v_max", 1.0)        # m/s
        self.declare_parameter("v_min", -0.2)       # m/s (permite ir en reversa un poco)
        self.declare_parameter("omega_max", 1.5)    # rad/s

        # Matrices de Pesos (Q: Estado, R: Control R_d:Cambio de la ley de control) - Â¡Afinables al vuelo!
        self.declare_parameter("weight_x", 1.0)
        self.declare_parameter("weight_y", 1.0)
        self.declare_parameter("weight_theta", 0.5)
        self.declare_parameter("weight_v", 0.1)
        self.declare_parameter("weight_omega", 0.1)
        self.declare_parameter("weight_accel", 0.5)   # Penaliza cambios bruscos en v
        self.declare_parameter("weight_alpha", 0.5) # Penaliza cambios bruscos en omega

        # Habilitar debug 
        self.declare_parameter("debug", True) # Si es true, se publicarÃ¡ la predicciÃ³n del MPC en RViz para visualizaciÃ³n.
        

        # Lectura de parÃ¡metros... (Omitida por brevedad, asume que se leen aquÃ­)
        self._load_parameters()

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)

        if self.debug_value:
            self.pred_pub = self.create_publisher(Path, self.debug_topic, 10)
            self.get_logger().info("Modo Debug Activado: Se publicara la predicciÃ³n que hace el MPC en el topic.")
        
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Estado interno
        self.path: List[PoseStamped] = []
        self.path_frame: Optional[str] = None
        self.has_path = False

        # AquÃ­ pre-compilamos el problema de optimizaciÃ³n en memoria
        self.solver, self.mpc_args = self._setup_mpc_problem()

        # Timer de control
        dt_timer = 1.0 / self.rate_hz
        # Variables de estado para MPC y Warm Start
        self.last_cmd = np.array([0.0, 0.0])
        self.U_prev = np.zeros(2 * self.N) # Memoria del Warm Start
        self.last_closest_index = 0

        self.timer = self.create_timer(dt_timer, self.on_timer)
        self.get_logger().info(f"Velocidad mÃ¡xima: {self.v_max}, Goal tolerance {self.goal_tol}")
        self.get_logger().info("Nodo MPC inicializado y esperando trayectoria.")


    def _load_parameters(self):
        """Lee todos los parÃ¡metros declarados y los asigna a variables de clase."""
        self.path_topic = self.get_parameter("path_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.N = self.get_parameter("mpc_N").value
        self.dt = self.get_parameter("mpc_dt").value
        self.debug_value = self.get_parameter("debug").value
        # ... (leer el resto de pesos y lÃ­mites) ...

    # ==========================================================
    # SECCIÃ“N MATEMÃTICA (El Cerebro)
    # ==========================================================
    def _setup_mpc_problem(self) -> Tuple[None, Dict]:
        """
        Prepara las matrices de pesos y los lÃ­mites para SciPy.
        Al usar Single-Shooting, el optimizador solo buscarÃ¡ un vector 1D
        con la secuencia de comandos: U = [v0, w0, v1, w1, ..., vN-1, wN-1]
        """
        self.get_logger().info("Configurando MPC usando SciPy (Single-Shooting)...")
        
        # 1. Construir las matrices de pesos Q y R como arreglos de numpy
        self.Q = np.diag([
            self.get_parameter("weight_x").value,
            self.get_parameter("weight_y").value,
            self.get_parameter("weight_theta").value
        ])
        
        self.R = np.diag([
            self.get_parameter("weight_v").value,
            self.get_parameter("weight_omega").value
        ])
        self.R_d = np.diag([
            self.get_parameter("weight_accel").value, # peso para suavizar aceleraciÃ³n lineal
            self.get_parameter("weight_alpha").value  # peso para suavizar aceleraciÃ³n angular
        ])
        
        # 2. Definir los lÃ­mites (bounds) de los actuadores para todo el horizonte
        # self.N veces el par (limites_v, limites_w)
        v_min = self.get_parameter("v_min").value
        self.v_max = self.get_parameter("v_max").value
        w_max = self.get_parameter("omega_max").value
        
        self.bounds = []
        for _ in range(self.get_parameter("mpc_N").value):
            self.bounds.append((v_min, self.v_max))       # LÃ­mites para v_k
            self.bounds.append((-w_max, w_max))      # LÃ­mites para omega_k
            
        return None, {} # SciPy no compila un objeto solver externo
    
    def _kinematic_model(self, state: np.ndarray, v: float, omega: float) -> np.ndarray:
        """
        Ecuaciones del modelo cinemÃ¡tico discreto (MÃ©todo de Euler).
        state = [x, y, theta]
        """
        x, y, theta = state[0], state[1], state[2]
        
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + omega * self.dt
        
        return np.array([x_next, y_next, theta_next])
    
    def _cost_function(self, U: np.ndarray, current_state: np.ndarray, ref_traj: np.ndarray, current_cmd: np.ndarray) -> float:
        """
        FunciÃ³n objetivo J que SciPy intentarÃ¡ minimizar.
        Simula el robot hacia el futuro usando los comandos U y penaliza el error.
        """
        cost = 0.0
        state = np.copy(current_state)

        prev_u = np.copy(current_cmd)        
        # U viene como un arreglo 1D plano: [v0, w0, v1, w1, ...]
        for k in range(self.N):
            v = U[2*k]
            omega = U[2*k + 1]
            u_vec = np.array([v, omega])
            
            
            # 1. Simular un paso hacia adelante (La fÃ­sica)
            state = self._kinematic_model(state, v, omega)
            
            # 2. Calcular el error respecto a la trayectoria de referencia
            error = state - ref_traj[k]
            
            # Normalizar el error de orientaciÃ³n (theta) para que estÃ© entre -pi y pi.
            # Esto evita que el robot dÃ© vueltas locas si el error salta a 2*pi.
            error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
            
            # 3. Sumar el costo de este paso: e^T * Q * e  +  u^T * R * u
            delta_u = u_vec - prev_u # Calcular el cambio brusco de velocidad (Delta U)
            
            state_cost = error.T @ self.Q @ error
            control_cost = u_vec.T @ self.R @ u_vec
            delta_control_cost = delta_u.T @ self.R_d @ delta_u
            
            cost += (state_cost + control_cost + delta_control_cost)

            prev_u = u_vec
            
        return cost
    
    def _solve_mpc(self, current_state: np.ndarray, ref_traj: np.ndarray) -> Tuple[float, float]:
        """
        Ejecuta el optimizador SLSQP de SciPy.
        """
        # Adivinanza inicial: Asumir que el robot se queda quieto (todo ceros)
        U0 = np.zeros(2 * self.N)
        U0[:-2] = self.U_prev[2:]  # Desplazar controles hacia la izquierda
        U0[-2:] = self.U_prev[-2:] # Repetir el Ãºltimo comando para llenar el vacÃ­o
        
        # Llamar al motor de optimizaciÃ³n
        res = minimize(
            self._cost_function, 
            U0, 
            args=(current_state, ref_traj, self.last_cmd), 
            method='SLSQP', 
            bounds=self.bounds,
            options={'ftol': 1e-3, 'maxiter': 50}
        )
        
        if res.success:
            self.U_prev = res.x
            # Extraer solo el PRIMER par de comandos (Receding Horizon)
            v_opt = res.x[0]
            omega_opt = res.x[1]

            # Actualizar el Ãºltimo comando enviado
            self.last_cmd = np.array([v_opt, omega_opt])
            if self.debug_value:
                self._publish_prediction(res.x, current_state)

            return float(v_opt), float(omega_opt)
        else:
            self.get_logger().warn(f"Optimizador fallÃ³: {res.message}. Deteniendo robot.")
            self.U_prev = np.zeros(2 * self.N) # Limpiar memoria corrupta
            self.last_cmd = np.array([0.0, 0.0])
            return 0.0, 0.0

    def _get_local_reference_trajectory(self, tf) -> np.ndarray:
        """
        Extrae un segmento de N puntos de la trayectoria global,
        transformados al marco base_link, para alimentar al solver.
        """
        ref_traj = np.zeros((self.N, 3))
        
        # Seguridad: Si no hay ruta, retornamos ceros (el robot intentarÃ¡ quedarse quieto)
        if not self.path:
            return ref_traj

        # ---------------------------------------------------------
        # PASO 1: Encontrar el Ã­ndice del punto mÃ¡s cercano al robot
        # ---------------------------------------------------------
        min_dist = float('inf')
        closest_idx = 0
        
        # Reciclamos tu lÃ³gica de Pure Pursuit: empezar desde el Ãºltimo Ã­ndice conocido
        # para no tener que buscar desde el inicio de la ruta en cada iteraciÃ³n.
        start_idx = getattr(self, 'last_closest_index', 0)
        n_poses = len(self.path)
        
        # Limitamos la bÃºsqueda hacia adelante para eficiencia (ej. buscar en los prÃ³ximos 50 puntos)
        search_range = min(start_idx + 50, n_poses)
        
        for i in range(start_idx, search_range):
            # Transformamos el punto global al chasis del robot
            pose_b = do_transform_pose_stamped(self.path[i], tf)
            x = pose_b.pose.position.x
            y = pose_b.pose.position.y
            
            # Distancia euclidiana al origen del base_link (0,0)
            dist = math.hypot(x, y)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Guardamos el Ã­ndice para la prÃ³xima iteraciÃ³n (Timer)
        self.last_closest_index = closest_idx

        # ---------------------------------------------------------
        # PASO 2: Extraer N puntos hacia el futuro
        # ---------------------------------------------------------
        # En una implementaciÃ³n avanzada, calcularÃ­as la distancia geomÃ©trica entre puntos.
        # AquÃ­ usaremos un salto de Ã­ndices (index_step) aproximado.
        # Si tu path tiene puntos cada 5cm, y esperas que el robot avance 10cm por cada dt (0.1s),
        # deberÃ­as saltar de a 2 Ã­ndices. Lo dejaremos parametrizado.
        
        index_step = 1 # Esto deberÃ­a ser idealmente un parÃ¡metro o calculado dinÃ¡micamente
        
        for k in range(self.N):
            # Calculamos el Ã­ndice objetivo saltando hacia adelante
            target_idx = closest_idx + (k * index_step)
            
            # Si el target_idx supera el final de la ruta, nos quedamos en el Ãºltimo punto (padding)
            # Esto es vital para que el robot se detenga suavemente al final y no se salga de rango.
            target_idx = min(target_idx, n_poses - 1)
            
            # Transformar el punto objetivo al base_link
            pose_b = do_transform_pose_stamped(self.path[target_idx], tf)
            
            x = pose_b.pose.position.x
            y = pose_b.pose.position.y
            
            # Extraer el cuaterniÃ³n y convertirlo a Yaw (theta)
            q = pose_b.pose.orientation
            yaw = euler_from_quaternion(q)
            
            # Llenar la matriz de referencia
            ref_traj[k] = [x, y, yaw]

        return ref_traj

    # ==========================================================
    # SECCIÃ“N ROS2 (El Middleware y Loop de Control)
    # ==========================================================
    def on_path(self, msg: Path) -> None:
        """Callback de la trayectoria (IdÃ©ntico a Pure Pursuit)."""
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id
        self.has_path = len(self.path) > 0

        self.last_closest_index = 0

    def on_timer(self) -> None:
        """Loop de control principal a N Hz."""
        if not self.has_path:
            self.publish_stop()
            return

        # 1. Obtener la Transformada Actual
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.path_frame,
                rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup fallÃ³: {ex}")
            self.publish_stop()
            return

        # 2. Verificar si llegamos a la meta (IdÃ©ntico a Pure Pursuit)
        # ... (LÃ³gica de goal_dist <= goal_tol) ...

        # 3. Preparar los datos para el MPC
        # El estado actual del robot respecto a su propio chasis siempre es cero
        current_state = np.array([0.0, 0.0, 0.0]) 
        
        # Obtener la referencia futura local (Horizonte N)
        ref_traj = self._get_local_reference_trajectory(tf)

        # 4. Resolver el problema de optimizaciÃ³n
        v_cmd, omega_cmd = self._solve_mpc(current_state, ref_traj)

        # 5. Publicar el comando
        self._publish_cmd(v_cmd, omega_cmd)

    def _publish_cmd(self, v: float, omega: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def _publish_prediction(self, U_opt: np.ndarray, current_state: np.ndarray) -> None:
        """
        Simula el modelo cinemÃ¡tico hacia adelante usando los comandos Ã³ptimos
        y publica el resultado para visualizaciÃ³n en RViz.
        """
        path_msg = Path()
        
        # FIX TF2: Al instanciar un Time de rclpy vacÃ­o, se genera una estampa de tiempo 0.
        # Esto evita que RViz encole el mensaje esperando una transformada futura.
        path_msg.header.stamp = rclpy.time.Time().to_msg()
        
        # La predicciÃ³n siempre nace y se dibuja desde el chasis del robot
        path_msg.header.frame_id = self.base_frame 
        
        state = np.copy(current_state)
        
        for k in range(self.N):
            v = U_opt[2*k]
            omega = U_opt[2*k + 1]
            
            # Avanzar un paso en el futuro
            state = self._kinematic_model(state, v, omega)
            
            # Crear el punto espacial
            pose = PoseStamped()
            pose.header = path_msg.header # Hereda el tiempo 0 y el frame_id
            pose.pose.position.x = float(state[0])
            pose.pose.position.y = float(state[1])
            
            # Un cuaterniÃ³n neutro bÃ¡sico para que RViz no arroje advertencias
            pose.pose.orientation.w = 1.0 
            
            path_msg.poses.append(pose)
            
        self.pred_pub.publish(path_msg)

    def publish_stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

def main():
    rclpy.init()
    node = MpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()