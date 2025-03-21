import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

# 从 ultralytics 中导入 YOLO
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.handle_image, 1)
        self.pub_yolo_class_id = self.create_publisher(Int32, 'yolo_class_id', 1)
        self.pub_stop_location = self.create_publisher(Float32MultiArray, 'stop_location', 1)

        # 加载一个预训练的 COCO YOLOv8n 模型
        self.model = YOLO('root/ros2_ws/src/car_pkg/car_pkg/yolov8n.pt')

        # 显示模型信息（可选）
        self.model.info()
        self.get_logger().info("YOLO模型加载成功！")

    def handle_image(self, msg):
        # 将 ROS Image 消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 使用 YOLOv8n 模型进行推理
        results = self.model(frame)
        #self.get_logger().info(str(results))
        for r in results:
            class_id = r.boxes.cls  # Boxes object for bbox outputs
            target = r.boxes.xyxy
            self.get_logger().info(str(class_id))
            
            if 11. in class_id:
                index = (class_id == 11).nonzero().item()  # 获取索引位置
                location = target[index]  # 获取对应位置的信息
                self.get_logger().info(str(target))
                self.get_logger().info(str(location))
                location_msg = Float32MultiArray()
                location_msg.data = location.tolist()
                self.pub_stop_location.publish(location_msg)
                
                id_msg = Int32()
                id_msg.data = 11
                self.pub_yolo_class_id.publish(id_msg)
            else:
                zero_msg = Float32MultiArray()
                zero_msg.data = [0., 0., 0., 0.]
                self.pub_stop_location.publish(zero_msg)
                id_msg = Int32()
                id_msg.data = 0
                self.pub_yolo_class_id.publish(id_msg)
            
def main():
    rclpy.init()
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
