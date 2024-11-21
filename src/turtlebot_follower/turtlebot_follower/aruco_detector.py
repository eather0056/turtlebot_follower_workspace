#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(PoseStamped, 'aruco_pose', 10)
        self.bridge = CvBridge()

        # Updated to use the new OpenCV API for predefined dictionaries
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for corner, marker_id in zip(corners, ids):
                # Pose estimation
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corner, 0.05, self.camera_matrix, self.dist_coeffs)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'camera_frame'

                # Extract pose values
                pose.pose.position.x = tvec[0][0][0]
                pose.pose.position.y = tvec[0][0][1]
                pose.pose.position.z = tvec[0][0][2]
                pose.pose.orientation.x = rvec[0][0][0]
                pose.pose.orientation.y = rvec[0][0][1]
                pose.pose.orientation.z = rvec[0][0][2]
                pose.pose.orientation.w = 1.0  # Simplified quaternion

                # Publish pose
                self.publisher_.publish(pose)

                # Draw markers and axis
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        # Display frame for debugging
        cv2.imshow('Aruco Detector', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
