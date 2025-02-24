#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
from hand_interfaces.msg import FingerData
from mediapipehelper import extract_keypoints

class HandToRobot(Node):
    def __init__(self):
        super().__init__('hand_to_robot')
        self.sub = self.create_subscription(Image, '/hand_keypoints', self.callback, 10)
        self.pub_left = self.create_publisher(TwistStamped, '/ur_left_arm/velocity_controller/command', 10)
        self.pub_right = self.create_publisher(TwistStamped, '/ur_right_arm/velocity_controller/command', 10)
        self.pub_gripper_left = self.create_publisher(Float32, '/ur_left_gripper/command', 10)
        self.pub_gripper_right = self.create_publisher(Float32, '/ur_right_gripper/command', 10)

    def callback(self, msg):
        keypoints = extract_keypoints(msg)  # 解析手部关键点
        twist_left = TwistStamped()
        twist_right = TwistStamped()
        
        # 计算左右手腕的速度控制
        twist_left.twist.linear.x = keypoints["left_wrist"].x * 0.5
        twist_left.twist.linear.y = keypoints["left_wrist"].y * 0.5
        twist_right.twist.linear.x = keypoints["right_wrist"].x * 0.5
        twist_right.twist.linear.y = keypoints["right_wrist"].y * 0.5
        
        self.pub_left.publish(twist_left)
        self.pub_right.publish(twist_right)

        # 计算夹爪状态（拇指和食指距离）
        left_gripper_cmd = 1.0 if keypoints["left_thumb_tip"].x - keypoints["left_index_tip"].x > 0.05 else 0.0
        right_gripper_cmd = 1.0 if keypoints["right_thumb_tip"].x - keypoints["right_index_tip"].x > 0.05 else 0.0
        self.pub_gripper_left.publish(left_gripper_cmd)
        self.pub_gripper_right.publish(right_gripper_cmd)

def main():
    rclpy.init()
    node = HandToRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
