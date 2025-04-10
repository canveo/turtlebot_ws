import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from transforms3d.euler import euler2quat
from rclpy.qos import QoSPresetProfiles

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.path_pub = self.create_publisher(Path, '/trajectory', QoSPresetProfiles.SENSOR_DATA.value)
        
        # Publicar cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_square)

    def publish_square(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # Definir puntos del cuadrado
        points = [
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.0),
            (0.5, 0.5, math.pi / 2),
            (0.0, 0.5, math.pi),
            (0.0, 0.0, -math.pi / 2),
        ]

        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info(f'Trayectoria publicada con {len(path.poses)} puntos.')

    def yaw_to_quaternion(self, yaw):
        """Convierte un ángulo yaw a un cuaternión usando transforms3d."""
        q = euler2quat(0, 0, yaw)
        return PoseStamped().pose.orientation.__class__(x=q[1], y=q[2], z=q[3], w=q[0])


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
