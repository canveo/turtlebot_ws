#!/usr/bin/env python3

"""
Copyright (c) 2023, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

...

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GoToGoalPublisher(Node):
    def __init__(self):
        super().__init__('go_to_goal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'commands/velocity', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.goal_reached = False
        
        # Initial goal parameters for square trajectory
        self.targets = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.current_target_index = 0
        self.target_x, self.target_y = self.targets[self.current_target_index]
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        self.velocity = Twist()
        self.turning = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(f"Current Odom -> x: {self.current_x}, y: {self.current_y}, yaw: {self.yaw}")

    def publish_velocity(self):
        if not self.goal_reached:
            distance = math.sqrt((self.target_x - self.current_x) ** 2 + (self.target_y - self.current_y) ** 2)
            if distance > 0.1 and not self.turning:
                self.velocity.linear.x = 0.2  # Move forward
                self.velocity.angular.z = 0.0  # No rotation
            elif self.turning:
                target_yaw = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
                yaw_diff = target_yaw - self.yaw
                yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))  # Normalize to [-pi, pi]
                if abs(yaw_diff) > 0.1:
                    self.velocity.linear.x = 0.0
                    self.velocity.angular.z = 0.8 if yaw_diff > 0 else -0.8  # Rotate to face the target
                else:
                    self.turning = False
                    self.velocity.linear.x = 0.0
                    self.velocity.angular.z = 0.0
                    self.get_logger().info(f"Finished turning to target ({self.target_x}, {self.target_y})")
            else:
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
                self.get_logger().info(f"Reached target ({self.target_x}, {self.target_y})")
                self.current_target_index = (self.current_target_index + 1) % len(self.targets)
                self.target_x, self.target_y = self.targets[self.current_target_index]
                self.turning = True
                self.get_logger().info(f"New target set to ({self.target_x}, {self.target_y})")
            
            self.publisher_.publish(self.velocity)
        else:
            # Stop the robot if the goal is reached
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.publisher_.publish(self.velocity)

    def shutdown(self):
        self.get_logger().info("Shutting down GoToGoalPublisher node")
        self.timer.cancel()  # Stop the timer to prevent further callbacks
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        if rclpy.ok():
            self.publisher_.publish(self.velocity)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()