import os
import cv2
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv

from cv_bridge import CvBridge, CvBridgeError
from example_interfaces.srv import SetBool


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')        

        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0


        self.path_output = '/home/canveo/carla_ws/turtlebot_data'
        file_path = os.path.join(self.path_output, 'odom_data.csv')
        self.csv_file = open(file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z'])

    def odom_callback(self, msg):
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        z_pos = msg.pose.pose.position.z
        timestamp = msg.header.stamp.nanosec
        
        self.get_logger().info('I received odom: [%f,%f,%f]' % (x_pos, y_pos, z_pos))
        self.csv_writer.writerow([timestamp, x_pos, y_pos, z_pos])

    def close(self):
        self.csv_file.close()
        

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
