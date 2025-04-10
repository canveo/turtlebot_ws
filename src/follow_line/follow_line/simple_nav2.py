import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class NavigationGoal(Node):
    def __init__(self):
        super().__init__('navigation_goal')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Establecer coordenadas del objetivo
        goal.pose.position.x = 2.0
        goal.pose.position.y = 2.0
        goal.pose.orientation.w = 1.0

        self.publisher.publish(goal)
        self.get_logger().info('¡Objetivo enviado!')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
