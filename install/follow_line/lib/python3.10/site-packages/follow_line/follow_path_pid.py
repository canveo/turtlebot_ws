import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler
import math


class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Path,
            '/plan',  # Topic donde Nav2 publica la trayectoria
            self.path_callback,
            10
        )
        self.path = []  # Lista de puntos de la trayectoria (x, y)

    def path_callback(self, msg):
        self.get_logger().info('Recibida nueva trayectoria')
        # Extraer los puntos de la trayectoria
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f'Trayectoria recibida: {self.path}')


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_pose = (0, 0, 0)  # (x, y, theta) del robot
        self.target_index = 0
        self.kp_linear = 1.0  # Ganancia proporcional para la velocidad lineal
        self.kp_angular = 1.5  # Ganancia proporcional para la velocidad angular
        self.path = []  # Trayectoria obtenida de PathSubscriber

    def update_current_pose(self, odom_msg):
        """Actualiza la posición actual del robot (para usar con mensajes de odometría)."""
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = quat2euler(quaternion)

        self.current_pose = (position.x, position.y, yaw)

    def follow_path(self):
        """Control PID para seguir la trayectoria."""
        if not self.path or self.target_index >= len(self.path):
            self.get_logger().info('No hay puntos restantes en la trayectoria.')
            return

        # Punto objetivo actual en la trayectoria
        target = self.path[self.target_index]
        error_x = target[0] - self.current_pose[0]
        error_y = target[1] - self.current_pose[1]

        # Cálculo de errores
        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        angular_error = angle_to_target - self.current_pose[2]

        # Normalizar el error angular entre -pi y pi
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi

        # Si el robot está cerca del objetivo, pasa al siguiente punto
        if distance_error < 0.1:  # Tolerancia de proximidad
            self.target_index += 1
            self.get_logger().info(f'Pasando al siguiente punto. Índice: {self.target_index}')
            return

        # Control PID
        linear_speed = self.kp_linear * distance_error
        angular_speed = self.kp_angular * angular_error

        # Limitar las velocidades para evitar movimientos bruscos
        linear_speed = min(linear_speed, 0.2)  # Velocidad máxima lineal
        angular_speed = max(min(angular_speed, 1.0), -1.0)  # Velocidad máxima angular

        # Publicar comandos de velocidad
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Crear instancias de los nodos
    path_subscriber = PathSubscriber()
    pid_controller = PIDController()

    # Ciclo principal
    try:
        while rclpy.ok():
            rclpy.spin_once(path_subscriber)  # Procesar la trayectoria
            pid_controller.path = path_subscriber.path  # Asignar la trayectoria al controlador
            pid_controller.follow_path()  # Ejecutar el control PID
    except KeyboardInterrupt:
        print("Nodo terminado por el usuario.")
    finally:
        rclpy.shutdown()
