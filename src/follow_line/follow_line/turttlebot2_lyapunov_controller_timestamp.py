#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import BumperEvent 
import math
import time
import csv
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

# Importar el controlador Lyapunov
from lyapunov_controller import LyapunovController

class TrajectoryGenerator:
    def __init__(self, N=100):
        self.div = round(N / 4)  # Resolución base
        self.pointX = [0, 1.75, 1.75, 0, 0]  # trayectoria cuadrada
        self.pointY = [0, 0, 1.75, 1.75, 0]
        self.pointYaw = np.radians([0.1, 90, 179, -90, -0.1])

        self.px = []
        self.py = []
        self.pyaw = []
        self.generate_trajectory()

    def generate_trajectory(self):
        for p in range(len(self.pointX) - 1):
            dx = self.pointX[p + 1] - self.pointX[p]
            dy = self.pointY[p + 1] - self.pointY[p]
            distance = np.sqrt(dx**2 + dy**2)
            local_div = max(int(self.div * distance / 0.1), 10)
            self.px.extend(np.linspace(self.pointX[p], self.pointX[p + 1], local_div))
            self.py.extend(np.linspace(self.pointY[p], self.pointY[p + 1], local_div))
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
        super().__init__('turtlebot_lyapunov_trajectory_follower')
        self.publisher_ = self.create_publisher(Twist, '/commands/velocity', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.bumper_sub = self.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)

        # Inicializar el controlador basado en Lyapunov
        self.lyapunov_controller = LyapunovController(k_rho=0.8, k_alpha=1.5, k_beta=-0.6)

        self.command = Twist()

        # Generación de la trayectoria
        self.trajectory = TrajectoryGenerator(N=100)
        self.waypoints_x, self.waypoints_y, self.waypoints_yaw = self.trajectory.get_trajectory()
        self.current_index = 0
        self.pose = None
        self.last_time = time.time()

        # Inicializar la trayectoria para RViz
        self.path = Path()
        self.path.header.frame_id = "odom"

        # Timer para el lazo de control
        self.timer = self.create_timer(0.1, self.control_loop)  # 0.1

        self.execution_stopped = False
        
        # Para registrar métricas del controlador Lyapunov
        self.log_time = []
        self.log_rho = []
        self.log_alpha = []
        self.log_beta = []
        self.log_v = []
        self.log_w = []

    def odom_callback(self, msg):
        current_pose = PoseStamped()
        current_pose.header = msg.header
        current_pose.pose = msg.pose.pose

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(current_pose)
        self.path_pub.publish(self.path)

        self.pose = current_pose.pose

    def control_loop(self):
        if self.execution_stopped:
            return
        
        if self.pose is None or self.current_index >= len(self.waypoints_x):
            return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Obtener el waypoint objetivo actual
        target_x = self.waypoints_x[self.current_index]
        target_y = self.waypoints_y[self.current_index]
        target_yaw = self.waypoints_yaw[self.current_index]  # opcional

        # Estado actual del robot
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.pose.orientation)

        # Calcular los comandos de control usando el controlador Lyapunov
        v, w, rho, alpha, beta = self.lyapunov_controller.compute_control(
            current_x, current_y, current_yaw, target_x, target_y, target_yaw
        )
        
        self.log_time.append(current_time)
        self.log_rho.append(rho)
        self.log_alpha.append(alpha)
        self.log_beta.append(beta)
        self.log_v.append(v)
        self.log_w.append(w)

        # Limitar velocidades si es necesario
        v = max(min(v, 0.4), -0.4)   # original 0.4
        w = max(min(w, 2.0), -2.0)
        # self.get_logger().info(v)
        print(v)


        self.command.linear.x = v
        self.command.angular.z = w
        self.publisher_.publish(self.command)

        # Avanzar al siguiente waypoint si se está suficientemente cerca
        if rho < 0.05:
            self.current_index += 1
            if self.current_index >= len(self.waypoints_x):
                self.get_logger().info("Trayectoria completada.")
                self.publisher_.publish(Twist())  # Detener el robot
                self.save_path_to_csv("robot_path_lyapunov_controller.csv")
                self.save_lyapunov_logs_to_csv()
                self.plot_lyapunov_metrics()

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_path_to_csv(self, filename):
        """Guarda la trayectoria seguida en un archivo CSV."""
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'yaw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for pose in self.path.poses:
                writer.writerow({
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'yaw': self.get_yaw_from_quaternion(pose.pose.orientation)
                })
        self.get_logger().info(f"Trayectoria guardada en {filename}")

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
            self.get_logger().info(f"¡Bumper {bumper_name} activado! Deteniendo el robot.")
            self.destroy_node()
            
    def save_lyapunov_logs_to_csv(self):
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"lyapunov_log_{timestamp_str}.csv"
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['time', 'rho', 'alpha', 'beta', 'v', 'w']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for i in range(len(self.log_time)):
                writer.writerow({
                    'time': self.log_time[i],
                    'rho': self.log_rho[i],
                    'alpha': self.log_alpha[i],
                    'beta': self.log_beta[i],
                    'v': self.log_v[i],
                    'w': self.log_w[i],
                })
        self.get_logger().info(f"Logs de Lyapunov guardados en {filename}")
        
    def plot_lyapunov_metrics(self):
        
        t0 = self.log_time[0]
        tiempo = [t - t0 for t in self.log_time]

        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(tiempo, self.log_rho, label='rho')
        plt.plot(tiempo, self.log_alpha, label='alpha')
        plt.plot(tiempo, self.log_beta, label='beta')
        plt.title("Errores de control")
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 2)
        plt.plot(tiempo, self.log_v, label='v (lineal)')
        plt.ylabel("Velocidad lineal (m/s)")
        plt.grid()

        plt.subplot(3, 1, 3)
        plt.plot(tiempo, self.log_w, label='w (angular)', color='purple')
        plt.xlabel("Tiempo (s)")
        plt.ylabel("Velocidad angular (rad/s)")
        plt.grid()

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
