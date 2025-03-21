import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DefogNode(Node):
    def __init__(self):
        super().__init__('defog_node')
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.handle_image, 1)
        self.pub_image = self.create_publisher(Image, '/defogged_image', 1)

        self.get_logger().info("去雾节点初始化成功！")

    def handle_image(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 对图像进行去雾处理
        defogged_frame = self.deHaze(frame / 255.0) * 255
        defogged_frame = np.clip(defogged_frame, 0, 255).astype(np.uint8)

        # 转换回 ROS 图像消息并发布
        defogged_msg = self.bridge.cv2_to_imgmsg(defogged_frame, encoding="bgr8")
        self.pub_image.publish(defogged_msg)

        # 显示去雾后的图像
        cv2.imshow('去雾后的图像', defogged_frame)
        cv2.waitKey(1)  # 等待以刷新窗口

    def zmMinFilterGray(self, src, r=7):
        return cv2.erode(src, np.ones((2 * r + 1, 2 * r + 1)))

    def guidedfilter(self, I, p, r, eps):
        height, width = I.shape
        m_I = cv2.boxFilter(I, -1, (r, r))
        m_p = cv2.boxFilter(p, -1, (r, r))
        m_Ip = cv2.boxFilter(I * p, -1, (r, r))
        cov_Ip = m_Ip - m_I * m_p

        m_II = cv2.boxFilter(I * I, -1, (r, r))
        var_I = m_II - m_I * m_I

        a = cov_Ip / (var_I + eps)
        b = m_p - a * m_I

        m_a = cv2.boxFilter(a, -1, (r, r))
        m_b = cv2.boxFilter(b, -1, (r, r))
        return m_a * I + m_b

    def Defog(self, m, r, eps, w, maxV1):
        V1 = np.min(m, 2)
        Dark_Channel = self.zmMinFilterGray(V1, 7)
        V1 = self.guidedfilter(V1, Dark_Channel, r, eps)
        
        bins = 2000
        ht = np.histogram(V1, bins)
        d = np.cumsum(ht[0]) / float(V1.size)
        for lmax in range(bins - 1, 0, -1):
            if d[lmax] <= 0.999:
                break
        A = np.mean(m, 2)[V1 >= ht[1][lmax]].max()
        V1 = np.minimum(V1 * w, maxV1)
        return V1, A

    def deHaze(self, m, r=50, eps=0.001, w=0.95, maxV1=0.80, bGamma=True):
        Y = np.zeros(m.shape)
        Mask_img, A = self.Defog(m, r, eps, w, maxV1)

        for k in range(3):
            Y[:, :, k] = (m[:, :, k] - Mask_img) / (1 - Mask_img / A)
        Y = np.clip(Y, 0, 1)
        if bGamma:
            Y = Y ** (np.log(0.5) / np.log(Y.mean()))
        return Y

def main():
    rclpy.init()
    defog_node = DefogNode()
    rclpy.spin(defog_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
