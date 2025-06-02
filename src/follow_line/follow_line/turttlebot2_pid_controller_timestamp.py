import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import BumperEvent 

import numpy as np
import math
import time
import csv
from datetime import datetime
import matplotlib.pyplot as plt

from pid_controller import PIDController

class TrajectoryGenerator:
    def __init__(self, N=100):
        self.div = round(N / 4)  # Resolución base
        self.pointX = [0, 1.75, 1.75, 0, 0]  # 1.75
        self.pointY = [0, 0, 1.75, 1.75, 0]
        self.pointYaw = np.radians([0.1, 90, 179, -90, -0.1])

        self.px = []
        self.py = []
        self.pyaw = []
        self.generate_trajectory()
        
        # for plotting pid linear and angular
        self.log_time = []
        self.log_error_dist = []
        self.log_linear_speed = []
        self.log_p_linear = []
        self.log_i_linear = []
        self.log_d_linear = []

        self.log_error_yaw = []
        self.log_angular_speed = []
        self.log_p_angular = []
        self.log_i_angular = []
        self.log_d_angular = []

    def generate_trajectory(self):
        for p in range(len(self.pointX) - 1):
            # Calcular la distancia entre los puntos
            dx = self.pointX[p + 1] - self.pointX[p]
            dy = self.pointY[p + 1] - self.pointY[p]
            distance = np.sqrt(dx**2 + dy**2)
            
            # Ajustar la resolución según la distancia
            local_div = max(int(self.div * distance / 0.1), 10)  # 0.1
            
            # Interpolación de puntos
            self.px.extend(np.linspace(self.pointX[p], self.pointX[p + 1], local_div))
            self.py.extend(np.linspace(self.pointY[p], self.pointY[p + 1], local_div))
            
            # Interpolación circular para yaw
            yaw_start = self.pointYaw[p]
            yaw_end = self.pointYaw[p + 1]
            if yaw_end - yaw_start > np.pi:
                yaw_end -= 2 * np.pi
            elif yaw_end - yaw_start < -np.pi:
                yaw_end += 2 * np.pi
            self.pyaw.extend(np.linspace(yaw_start, yaw_end, local_div))

    def get_trajectory(self):
        return self.px, self.py, self.pyaw

class TurtlebotTrajectoryFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_pid_trajectory_follower')
        self.publisher_ = self.create_publisher(Twist, '/commands/velocity', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.bumper_sub = self.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)

        # Inicialización del PID
        self.linear_pid = PIDController(0.8, 0.01, 0.05)
        self.angular_pid = PIDController(3.0, 0.005, 1.0)  # 3.0, 0.001, 1.0

        # for plotting pid linear and angular
        self.log_time = []
        self.log_error_distance = []
        self.log_linear_speed = []
        self.log_p_linear = []
        self.log_i_linear = []
        self.log_d_linear = []
        
        self.log_error_distance = []
        self.log_angular_speed = []
        self.log_p_angular = []
        self.log_i_angular = []
        self.log_d_angular = []
        
        self.command = Twist()

        # Generación de trayectoria
        self.trajectory = TrajectoryGenerator(N=100)  # N=100
        self.waypoints_x, self.waypoints_y, self.waypoints_yaw = self.trajectory.get_trajectory()
        self.current_index = 0
        self.pose = None
        self.last_time = time.time()

        # Inicialización de Path
        self.path = Path()
        self.path.header.frame_id = "odom"  # Frame de referencia para RViz

        # Timer para el control PID
        self.timer = self.create_timer(0.1, self.control_loop)

        self.execution_stopped = False

    def odom_callback(self, msg):
        current_pose = PoseStamped()
        current_pose.header = msg.header
        current_pose.pose = msg.pose.pose

        # Añadir la pose actual al path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(current_pose)
        self.path_pub.publish(self.path)

        # Guardar la pose actual para otros cálculos
        self.pose = current_pose.pose

        # self.get_logger().info(f"Pose added: x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}")

    def control_loop(self):
        if self.execution_stopped:

            return
        
        if self.pose is None or self.current_index >= len(self.waypoints_x):
            return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Obtener el punto objetivo actual
        target_x = self.waypoints_x[self.current_index]
        target_y = self.waypoints_y[self.current_index]

        # self.get_logger().info(f"Target: x={target_x:.2f}, y={target_y:.2f}")

        # Posición actual del robot
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.pose.orientation)

        # Calcular errores
        error_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        error_yaw = angle_to_target - current_yaw

        # Normalizar el error angular a [-pi, pi]
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))

        # Calcular velocidades usando PID
        linear_speed = self.linear_pid.compute(error_distance, dt)
        angular_speed = self.angular_pid.compute(error_yaw, dt)

        # Limitar velocidades
        linear_speed = max(min(linear_speed, 0.4), -0.4)
        angular_speed = max(min(angular_speed, 2.0), -2.0)
        
        # for plotting offline pid data
        # Guardar logs para PID lineal
        self.log_time.append(current_time)
        self.log_error_dist.append(error_distance)
        self.log_linear_speed.append(linear_speed)
        self.log_p_linear.append(self.linear_pid.p_term)
        self.log_i_linear.append(self.linear_pid.i_term)
        self.log_d_linear.append(self.linear_pid.d_term)

        # Guardar logs para PID angular
        self.log_error_yaw.append(error_yaw)
        self.log_angular_speed.append(angular_speed)
        self.log_p_angular.append(self.angular_pid.p_term)
        self.log_i_angular.append(self.angular_pid.i_term)
        self.log_d_angular.append(self.angular_pid.d_term)

        # Publicar comandos de velocidad
        # cmd = Twist()
        # cmd.linear.x = linear_speed
        self.command.linear.x = linear_speed
        # cmd.angular.z = angular_speed
        self.command.angular.z = angular_speed
        self.publisher_.publish(self.command)
        
        self.log_time.append(current_time)
        self.log_error_dist.append(error_distance)
        self.log_linear_speed.append(linear_speed)
        self.log_p_linear.append(self.linear_pid.p_term)
        self.log_i_linear.append(self.linear_pid.i_term)
        self.log_d_linear.append(self.linear_pid.d_term)

        # Avanzar al siguiente punto si el robot está cerca
        if error_distance < 0.05:
            self.current_index += 1
            if self.current_index >= len(self.waypoints_x):
                self.get_logger().info("Trayectoria completada.")
                self.publisher_.publish(Twist())  # Detener el robot
                self.save_path_to_csv("robot_path_pid_controller")
                self.save_pid_logs_to_csv()
                # Para graficar PID lineal:
                self.plot_pid_response(
                    self.log_time,
                    self.log_linear_speed,
                    self.log_error_dist,
                    self.log_p_linear,
                    self.log_i_linear,
                    self.log_d_linear,
                    label="Lineal"
                )

                # Para graficar PID angular:
                self.plot_pid_response(
                    self.log_time,
                    self.log_angular_speed,
                    self.log_error_yaw,
                    self.log_p_angular,
                    self.log_i_angular,
                    self.log_d_angular,
                    label="Angular"
)

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_path_to_csv(self, filename):
        """Guarda la trayectoria en un archivo CSV."""
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{filename}_{timestamp_str}.csv"
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'yaw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for pose in self.path.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                yaw = self.get_yaw_from_quaternion(pose.pose.orientation)
                
                stamp = pose.header.stamp
                timestamp_sec = stamp.sec + stamp.nanosec * 1e-9
                                
                writer.writerow({
                    'timestamp': timestamp_sec,
                    'x': x,
                    'y': y,
                    'yaw': yaw
                })

        self.get_logger().info(f"Trayectoria guardada en {filename}")

    def save_pid_logs_to_csv(self):
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pid_log_{timestamp_str}.csv"

        with open(filename, 'w', newline='') as csvfile:
            fieldnames = [
                'time',
                'error_dist', 'linear_speed', 'p_linear', 'i_linear', 'd_linear',
                'error_yaw', 'angular_speed', 'p_angular', 'i_angular', 'd_angular'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for i in range(len(self.log_time)):
                writer.writerow({
                    'time': self.log_time[i],
                    'error_dist': self.log_error_dist[i],
                    'linear_speed': self.log_linear_speed[i],
                    'p_linear': self.log_p_linear[i],
                    'i_linear': self.log_i_linear[i],
                    'd_linear': self.log_d_linear[i],
                    'error_yaw': self.log_error_yaw[i],
                    'angular_speed': self.log_angular_speed[i],
                    'p_angular': self.log_p_angular[i],
                    'i_angular': self.log_i_angular[i],
                    'd_angular': self.log_d_angular[i],
                })

        self.get_logger().info(f"Logs de PID guardados en {filename}")

    def stop_robot(self):
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self.publisher_.publish(self.command)

    def bumper_callback(self, msg):
        bumper_name = {0: "Izquierdo", 1: "Central", 2: "Derecho"}.get(msg.bumper, "Desconocido")
        state = "PRESIONADO" if msg.state == 1 else "LIBRE"
        self.get_logger().info(f"Bumper: {bumper_name} | Estado: {state}")
        if msg.state == 1 and not self.execution_stopped:
            self.execution_stopped = True
            self.stop_robot()
            self.save_path_to_csv("robot_path_stopped.csv")
            self.save_path_to_csv("robot_path_stopped")
            self.save_pid_logs_to_csv()
            self.get_logger().info(f"¡Bumper {bumper_name} activado! Deteniendo el robot.")
            self.destroy_node()

    def plot_pid_response(self, log_time, log_output, log_error, log_p, log_i, log_d, label=""):
        """
        Grafica la respuesta de un controlador PID.

        Parámetros:
            log_time: Lista o array de tiempo.
            log_output: Salida del controlador (velocidad).
            log_error: Error entre referencia y medición.
            log_p, log_i, log_d: Componentes P, I, D.
            label: Etiqueta para el tipo de controlador (ej. 'Lineal' o 'Angular').
        """
        plt.figure(figsize=(12, 8))

        # Salida del controlador
        plt.subplot(3, 1, 1)
        plt.plot(log_time, log_output, label=f'Salida {label}', color='blue')
        plt.xlabel('Tiempo (s)')
        plt.ylabel(f'Salida {label}')
        plt.title(f'Salida del Controlador PID ({label})')
        plt.grid()
        plt.legend()

        # Error
        plt.subplot(3, 1, 2)
        plt.plot(log_time, log_error, label=f'Error {label}', color='red')
        plt.xlabel('Tiempo (s)')
        plt.ylabel(f'Error {label}')
        plt.title(f'Error de Seguimiento ({label})')
        plt.grid()
        plt.legend()

        # Términos PID
        plt.subplot(3, 1, 3)
        plt.plot(log_time, log_p, label='P', color='green')
        plt.plot(log_time, log_i, label='I', color='orange')
        plt.plot(log_time, log_d, label='D', color='purple')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('Términos PID')
        plt.title(f'Términos del PID ({label})')
        plt.grid()
        plt.legend()

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()