import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Test(Node):

    def __init__(self):
        super().__init__('test')
        self.publisher_ = self.create_publisher(Bool, 'pause_signal', 1)
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.count = 1

    def timer_callback(self):
        if self.count % 2 ==1:
            self.count += 1
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            self.count += 1
            msg = Bool()
            msg.data = False
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
