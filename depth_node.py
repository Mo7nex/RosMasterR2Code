# 图像格式为640×480

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class TargetDistPub(Node):

    def __init__(self):
        super().__init__('target_dist_pub')
        self.sub_stop_location = self.create_subscription(Float32MultiArray, 'stop_location', self.stop_location_callback, 1)
        self.sub_depth_image = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_image_callback, 1)
        self.pub_target_dist = self.create_publisher(Float32, 'target_dist', 1)
        self.bridge = CvBridge()
        self.has_stop_location = False

    def stop_location_callback(self, msg):
        # Extract x1, y1, x2, y2 from Float32MultiArray message
        x1, y1, x2, y2 = msg.data
        # Store coordinates for later use
        self.x1, self.y1, self.x2, self.y2 = int(x1), int(y1), int(x2), int(y2)
        if self.x1 > 0:
            self.has_stop_location = True
        else:
            self.has_stop_location = False

    def depth_image_callback(self, msg):
        if self.has_stop_location:
            # Convert ROS Image message to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Ensure the coordinates are within the image boundaries
            x1 = max(0, min(self.x1, depth_image.shape[1] - 1))
            y1 = max(0, min(self.y1, depth_image.shape[0] - 1))
            x2 = max(0, min(self.x2, depth_image.shape[1] - 1))
            y2 = max(0, min(self.y2, depth_image.shape[0] - 1))

            # Calculate the center coordinates
            center_x = (x1 + x2) // 2 + 10
            center_y = (y1 + y2) // 2

            # Calculate the boundaries for the 10x10 region around the center
            start_x = max(0, center_x - 5)
            end_x = min(depth_image.shape[1] - 1, center_x + 5)
            start_y = max(0, center_y - 5)
            end_y = min(depth_image.shape[0] - 1, center_y + 5)

            # Extract the 10x10 region
            region = depth_image[start_y:end_y + 1, start_x:end_x + 1]

            # Calculate the average depth value
            average_depth = np.mean(region)
            #self.get_logger().info(str(average_depth))

            # Publish the average depth value
            msg = Float32()
            msg.data = average_depth
            self.pub_target_dist.publish(msg)
#        else:
#            # Publish target_dist as 0 if stop_location hasn't been received
#            msg = Float32()
#            msg.data = 0.0
#            self.pub_target_dist.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    target_dist_pub = TargetDistPub()
    rclpy.spin(target_dist_pub)
    target_dist_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
