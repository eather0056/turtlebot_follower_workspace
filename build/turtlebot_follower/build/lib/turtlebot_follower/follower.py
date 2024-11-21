#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.subscription = self.create_subscription(
            PoseStamped,
            'aruco_pose',
            self.pose_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Adjustable parameters
        self.target_distance = 0.5  # Desired following distance in meters
        self.linear_speed = 0.2
        self.angular_speed = 0.5

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Calculate distance and angle to target
        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(y, x)

        cmd = Twist()
        if distance > self.target_distance:
            # Move towards the target
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_speed * angle_to_target
        else:
            # Stop if within the target distance
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
