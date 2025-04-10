import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import math
import csv

class PIDController:
    """Controlador PID básico."""
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=float('-inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, delta_time):
        """Calcula la salida del PID."""
        if delta_time <= 0.0:
            return 0.0

        
        p = self.kp * error

        
        self.integral += error * delta_time
        i = self.ki * self.integral

        
        d = self.kd * (error - self.prev_error) / delta_time
        self.prev_error = error

        
        output = p + i + d
        return max(self.min_output, min(self.max_output, output))


class SquareControlWithBumpers(Node):
    def __init__(self):
        super().__init__('square_control_with_bumpers')
        self.pub = self.create_publisher(Twist, '/commands/velocity', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_bumper = self.create_subscription(Bool, '/events/bumper', self.bumper_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)

        
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.saved_path = []  

        self.command = Twist()

        
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.target_yaw = 0.0
        self.stage = 0
        self.moving = False
        self.last_time = self.get_clock().now()

        
        self.linear_pid = PIDController(kp=0.01, ki=0.01, kd=0.1, max_output=0.5, min_output=0.0)
        self.angular_pid = PIDController(kp=2.8, ki=0.1, kd=0.6, max_output=1.0, min_output=-1.0)

        
        self.distance_threshold = 0.04
        self.angle_threshold = 0.1

        
        self.bumper_activated = False
        self.execution_stopped = False  

    def bumper_callback(self, msg):
        """Callback para manejar el estado del bumper."""
        if msg.data and not self.execution_stopped:
            self.bumper_activated = True
            self.execution_stopped = True  
            self.stop_robot()
            self.save_path_to_csv("robot_path.csv")
            self.get_logger().info("Bumper activado. Deteniendo el robot y guardando la trayectoria.")
            self.destroy_node()  

    def odom_callback(self, msg):
        """Callback para manejar el movimiento del robot."""
        if self.execution_stopped:
            
            return

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.moving:
            
            self.initial_x = current_x
            self.initial_y = current_y
            self.target_yaw = self.calculate_target_yaw()
            self.moving = True

        
        if self.stage % 2 == 0:  
            distance = math.sqrt((current_x - self.initial_x) ** 2 + (current_y - self.initial_y) ** 2)
            distance_error = 1.0 - distance
            angle_error = math.atan2(math.sin(self.target_yaw - current_yaw), math.cos(self.target_yaw - current_yaw))

            if abs(distance_error) > self.distance_threshold:
                
                linear_speed = self.linear_pid.compute(distance_error, delta_time)
                angular_speed = self.angular_pid.compute(angle_error, delta_time)
                self.command.linear.x = linear_speed
                self.command.angular.z = angular_speed
            else:
                
                self.stage += 1
                self.moving = False
        else:  
            angle_error = math.atan2(math.sin(self.target_yaw - current_yaw), math.cos(self.target_yaw - current_yaw))

            if abs(angle_error) > self.angle_threshold:
                
                angular_speed = self.angular_pid.compute(angle_error, delta_time)
                self.command.linear.x = 0.0
                self.command.angular.z = angular_speed
            else:
                
                self.stage += 1
                self.moving = False

        
        if self.stage >= 8:
            self.command.linear.x = 0.0
            self.command.angular.z = 0.0
            self.get_logger().info('Cuadrado completado.')
            self.pub.publish(self.command)
            self.save_path_to_csv("robot_path.csv")
            return

        
        self.pub.publish(self.command)

        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)
        self.saved_path.append((current_x, current_y, current_yaw))  
        self.path_pub.publish(self.path)

    def calculate_target_yaw(self):
        """Calcula el ángulo objetivo para el lado actual del cuadrado."""
        base_yaw = 0.0  
        yaw_increment = math.pi / 2  
        return (base_yaw + (self.stage // 2) * yaw_increment) % (2 * math.pi)

    def stop_robot(self):
        """Detiene el robot."""
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self.pub.publish(self.command)

    def save_path_to_csv(self, filename):
        """Guarda la trayectoria en un archivo CSV."""
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'yaw']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()
            for x, y, yaw in self.saved_path:
                writer.writerow({'x': x, 'y': y, 'yaw': yaw})

        self.get_logger().info(f"Trayectoria guardada en {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = SquareControlWithBumpers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.execution_stopped:
            node.save_path_to_csv("robot_path.csv")
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
