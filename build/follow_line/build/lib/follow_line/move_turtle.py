import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class SquareControl(Node):
    def __init__(self):
        super().__init__('square_control')
        self.pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        
        # Inicializació de trayectoria
        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.command = Twist()

        # Variables de control para el movimiento
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        self.stage = 0  # Indica el estado del robot: avanzar o girar
        self.moving = False

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Obtener la orientación del robot en forma de cuaternión
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not self.moving:
            # Establecer las posiciones iniciales cuando comenzamos a movernos
            self.initial_x = current_x
            self.initial_y = current_y
            self.initial_yaw = current_yaw
            self.moving = True

        # Comprobar si estamos avanzando o girando
        if self.stage % 2 == 0:  # Avanzar
            distance = math.sqrt((current_x - self.initial_x) ** 2 + (current_y - self.initial_y) ** 2)
            if distance < 0.50:
                # Avanzar hacia adelante
                self.command.linear.x = 0.2
                self.command.angular.z = 0.0
            else:
                # Cambiar al estado de giro
                self.stage += 1
                self.moving = False
        else:  # Girar
            angle_difference = abs(current_yaw - self.initial_yaw)
            if angle_difference < math.pi / 2:
                # Girar 90 grados
                self.command.linear.x = 0.0
                self.command.angular.z = 0.3
            else:
                # Cambiar al estado de avanzar
                self.stage += 1
                self.moving = False

        # Si se han completado 4 etapas de avanzar y 4 de girar, se detiene el movimiento
        if self.stage >= 8:
            self.command.linear.x = 0.0
            self.command.angular.z = 0.0
            self.get_logger().info('Cuadrado completado.')
            self.pub.publish(self.command)
            return

        # Publicar el comando
        self.pub.publish(self.command)

        # Agregar la psocisión actual a la trayectoria y publicarla
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = SquareControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
