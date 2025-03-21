import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class FogNode(Node):
    def __init__(self):
        super().__init__('fog_node')
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.handle_image, 1)
        self.pub_image = self.create_publisher(Image, '/fogged_image', 1)

        self.get_logger().info("加雾节点初始化成功！")

    def handle_image(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            original_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return
        
        # 添加雾
        fogged_frame = self.add_fog(original_frame)

        # 转换回 ROS 图像消息并发布
        fogged_msg = self.bridge.cv2_to_imgmsg(fogged_frame, encoding="bgr8")
        self.pub_image.publish(fogged_msg)

        # 显示加雾后的图像
        cv2.imshow('加雾后的图像', fogged_frame)
        cv2.waitKey(1)  # 等待以刷新窗口

    def add_fog(self, frame, fog_intensity=0.6):
        """
        向视频帧添加雾。

        :param frame: 输入帧。
        :param fog_intensity: 雾的强度，范围在0到1之间，默认是0.6。
        """
        brightness = 240  # 雾的亮度
        fog_overlay = np.zeros_like(frame)
        fog_overlay[:] = brightness  # 创建雾覆盖层
        result = cv2.addWeighted(frame, 1 - fog_intensity, fog_overlay, fog_intensity, 0)  # 混合
        return result

def main():
    rclpy.init()
    fog_node = FogNode()
    rclpy.spin(fog_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
