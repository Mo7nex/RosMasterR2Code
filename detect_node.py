#!/usr/bin/env python
# encoding: utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
import time

class DetectNode(Node):
    def __init__(self):
        super().__init__('detect_node')
        self.subscription_yolo = self.create_subscription(Int32, 'yolo_class_id', self.yolo_callback, 1)
        self.subscription_dist = self.create_subscription(Float32, 'target_dist', self.dist_callback, 1)
        self.publisher_pause = self.create_publisher(Bool, 'pause_signal', 1)
        self.target_dist = 0.0  # 初始化 target_dist

    def yolo_callback(self, msg):
        pause_signal = Bool()
        self.get_logger().info(str(msg.data))
        self.get_logger().info(str(self.target_dist))
        if msg.data == 11 and self.target_dist <= 500:
            pause_signal.data = True
            self.publisher_pause.publish(pause_signal)
            time.sleep(3)
            pause_signal.data = False
            self.publisher_pause.publish(pause_signal)
            self.target_dist = 750.
            time.sleep(10)

    def dist_callback(self, msg):
        self.target_dist = msg.data

def main(args=None):
    rclpy.init(args=args)
    detect_node = DetectNode()
    rclpy.spin(detect_node)
    detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
