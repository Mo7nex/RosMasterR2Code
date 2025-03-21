import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import onnxruntime as ort
import numpy as np
import cv2

class ONNXNode(Node):
    def __init__(self):
        super().__init__('onnx_node')
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.handle_image, 1)
        self.pub_yolo_class_id = self.create_publisher(Int32, 'yolo_class_id', 1)
        self.pub_stop_location = self.create_publisher(Float32MultiArray, 'stop_location', 1)

        # 加载一个onnx模型
        self.model = ort.InferenceSession('root/ros2_ws/src/car_pkg/car_pkg/yolo.onnx')

        self.get_logger().info("模型加载成功！")

    def handle_image(self, msg):
        # 将 ROS Image 消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        original_frame = frame.copy()  # 保留原始图像
        # self.get_logger().info(f'Image dimensions: {original_frame.shape[1]} x {original_frame.shape[0]}')
        
        # 图像预处理
        input_tensor = self.preprocess(frame)

        # 使用模型进行推理
        inputs = {self.model.get_inputs()[0].name: input_tensor}
        outputs = self.model.run(None, inputs)

        # 处理模型输出
        self.postprocess(original_frame, outputs[0])

    def preprocess(self, image):
        # 将图像预处理为模型需要的输入格式
        input_shape = (640, 640)
        image_resized = cv2.resize(image, input_shape)
        image_transposed = np.transpose(image_resized, (2, 0, 1))
        input_tensor = image_transposed.astype(np.float32) / 255.0
        input_tensor = np.expand_dims(input_tensor, axis=0)
        return input_tensor

    def postprocess(self, image, outputs):
        # 输出包含 (1, 25200, 85) 形状的预测框，其中85 = 4 (bbox) + 1 (objectness score) + 80 (class scores)
        boxes, scores, class_ids = [], [], []
        for detection in outputs[0]:
            box = detection[:4]
            objectness = detection[4]
            class_scores = detection[5:]
            class_id = np.argmax(class_scores)
            score = class_scores[class_id] * objectness
            
            if score > 0.5:  # 阈值
                # self.get_logger().info(str(class_id))
                boxes.append(box)
                scores.append(score)
                class_ids.append(class_id)

        # 映射检测框坐标回原始图像尺寸
        for box, score, class_id in zip(boxes, scores, class_ids):
            if class_id ==11.0:
                cx, cy, w, h = box
                cx = cx * (640 / 640)
                cy = cy * (480 / 640)
                w = w * (640 / 640)
                h = h * (480 / 640)

                x1 = int(cx - w / 2)
                y1 = int(cy - h / 2)
                x2 = int(cx + w / 2)
                y2 = int(cy + h / 2)

                location = [x1, y1, x2, y2]  # 获取对应位置的信息
                location = [float(val) for val in location]
                # self.get_logger().info(str(location))
                location_msg = Float32MultiArray()
                location_msg.data = location
                self.pub_stop_location.publish(location_msg)

                id_msg = Int32()
                id_msg.data = int(class_id)
                self.pub_yolo_class_id.publish(id_msg)

                # 确保坐标在图像范围内
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(640, x2), min(480, y2)

                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, f'{class_id} {score:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
#            else:
#                zero_msg = Float32MultiArray()
#                zero_msg.data = [0., 0., 0., 0.]
#                self.pub_stop_location.publish(zero_msg)
#                id_msg = Int32()
#                id_msg.data = 0
#                self.pub_yolo_class_id.publish(id_msg)


        # 显示结果
        # self.get_logger().info(f'Image dimensions: {image.shape[1]} x {image.shape[0]}')
        cv2.imshow('YOLO Detection', image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    onnx_node = ONNXNode()
    rclpy.spin(onnx_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
