import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import BumperEvent 
from geometry_msgs.msg import Quaternion

import numpy as np
import math
import time
import csv

from pid_controller import PIDController


import numpy as np

class TrajectoryGenerator:
    def __init__(self, N_line=50, N_curve=50, L_U=1.0, P0=(1, 2)):
        """
        Genera una trayectoria que inicia en el punto P0 dado, continúa con una línea recta,
        y luego realiza un cuarto de circunferencia como la figura original.

        Parámetros:
            N_line  : Número de puntos en la trayectoria recta desde P0 a P1.
            N_curve : Número de puntos en la trayectoria curva desde P1 hasta P2.
            L_U     : Longitud unidad para la trayectoria.
            P0      : Coordenadas del punto inicial.
        """
        self.N_line = N_line
        self.N_curve = N_curve
        self.L_U = L_U
        self.P0 = P0

        self.px = []
        self.py = []
        self.pyaw = []

        self.generate_trajectory()

    def generate_trajectory(self):
        # Trayectoria recta desde P0 hasta P1 (horizontal hacia la derecha, longitud 5*L_U)
        P1 = (self.P0[0] + 5*self.L_U, self.P0[1])
        x_line = np.linspace(self.P0[0], P1[0], self.N_line)
        y_line = np.linspace(self.P0[1], P1[1], self.N_line)
        yaw_line = np.zeros_like(x_line)  # orientación horizontal a la derecha

        # Trayectoria curva (cuarto de circunferencia) desde P1 hasta P2
        center_x = P1[0]
        center_y = P1[1] - 5*self.L_U
        angles = np.linspace(np.pi/2, 0, self.N_curve)
        x_curve = center_x + 5*self.L_U*np.cos(angles)
        y_curve = center_y + 5*self.L_U*np.sin(angles)
        yaw_curve = angles - np.pi/2  # orientación tangencial

        # Concatenar trayectorias
        self.px = np.concatenate([x_line, x_curve])
        self.py = np.concatenate([y_line, y_curve])
        self.pyaw = np.concatenate([yaw_line, yaw_curve])

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
        self.linear_pid = PIDController(1.2, 0.08, 0.05)   # valores iniciales (0.8, 0.01, 0.05)
        self.angular_pid = PIDController(3.0, 0.005, 1.0)  # 3.0, 0.001, 1.0

        self.command = Twist()

        # Generación de trayectoria
        self.trajectory = TrajectoryGenerator(N_line=50, N_curve=50, L_U=0.35, P0=(0.3, 0.3))  
        self.waypoints_x, self.waypoints_y, self.waypoints_yaw = self.trajectory.get_trajectory()
        self.current_index = 0
        self.pose = None
        self.last_time = time.time()

        self.desired_path_pub = self.create_publisher(Path, '/desired_path', 10)
        self.publish_desired_path()

        # Inicialización de Path
        self.path = Path()
        self.path.header.frame_id = "odom"  # Frame de referencia para RViz

        # Timer para el control PID
        self.timer = self.create_timer(0.1, self.control_loop)

        self.execution_stopped = False

        self.linear_speed = 0
        self.angular_speed = 0

    def publish_desired_path(self):
        traj_x, traj_y, traj_yaw = self.trajectory.get_trajectory()

        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()     

        for x, y, yaw in zip(traj_x, traj_y, traj_yaw):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Utiliza aquí la función correcta yaw → quaternion
            pose.pose.orientation = self.quaternion_from_yaw(yaw)

            path_msg.poses.append(pose)

        self.desired_path_pub.publish(path_msg) 
        self.get_logger().info("Trayectoria deseada publicada en RViz.")

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

        self.linear_speed.append(linear_speed)
        self.angular_speed.append(angular_speed)

        # Limitar velocidades
        linear_speed = max(min(linear_speed, 0.4), -0.4)
        angular_speed = max(min(angular_speed, 2.0), -2.0)

        # Publicar comandos de velocidad
        # cmd = Twist()
        # cmd.linear.x = linear_speed
        self.command.linear.x = linear_speed
        # cmd.angular.z = angular_speed
        self.command.angular.z = angular_speed
        self.publisher_.publish(self.command)

        # Avanzar al siguiente punto si el robot está cerca
        if error_distance < 0.05:
            self.current_index += 1
            if self.current_index >= len(self.waypoints_x):
                self.get_logger().info("Trayectoria completada.")
                self.publisher_.publish(Twist())  # Detener el robot
                self.save_path_to_csv("robot_path_pid_controller.csv")

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def quaternion_from_yaw(self, yaw):
        """Convierte un ángulo yaw a quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


    def save_path_to_csv(self, filename):
        """Guarda la trayectoria en un archivo CSV."""
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'yaw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for pose in self.path.poses:
                writer.writerow({
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'yaw': self.get_yaw_from_quaternion(self.pose.orientation),
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


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
