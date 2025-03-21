#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32
import time

class DetectCtrl(Node):
    def __init__(self):
        super().__init__('detect_controller')
        self.subscription_yolo = self.create_subscription(Int32, 'yolo_class_id', self.yolo_callback, 1)
        self.subscription_dist = self.create_subscription(Float32, 'target_dist', self.dist_callback, 1)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.linear_speed_limit = 1.0
        self.angular_speed_limit = 5.0
        self.stopped = False
        self.target_dist = 0

    def timer_callback(self):
        if not self.stopped:
            # 正常向前移动
            self.get_logger().info("未检测到目标，前进")
            self.publish_twist(linear={'x': 0.1, 'y': 0.0, 'z': 0.0}, angular={'x': 0.0, 'y': 0.0, 'z': 0.0})

    def yolo_callback(self, msg):
        if msg.data == 11 and 400 <= self.target_dist <= 500:
            # 当 yolov5_results=11 且 target_dist 在 400 到 500 之间时，停止小车
            self.stopped = True
            self.publish_twist(linear={'x': 0.0, 'y': 0.0, 'z': 0.0}, angular={'x': 0.0, 'y': 0.0, 'z': 0.0})
            self.get_logger().info("检测到目标，停止两秒")
            time.sleep(2)
            self.publish_twist(linear={'x': 0.1, 'y': 0.0, 'z': 0.0}, angular={'x': 0.0, 'y': 0.0, 'z': 0.0})
            self.get_logger().info("检测到目标，强制前进五秒")
            time.sleep(5)
        else:
            self.stopped = False

    def dist_callback(self, msg):
        self.target_dist = msg.data

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = min(linear['x'], self.linear_speed_limit)
        msg.angular.z = min(angular['z'], self.angular_speed_limit)
        self.publisher_cmd_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    detect_controller = DetectCtrl()
    rclpy.spin(detect_controller)
    detect_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
