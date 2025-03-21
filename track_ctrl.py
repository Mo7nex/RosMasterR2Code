import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time
import threading

class TrackController(Node):
    def __init__(self):
        super().__init__('path_ctrl')
        
        # 创建路径字典
        self.paths = {
            'A_B': [('straight', 18), ('right', 5.6)],
            'B_C': [('straight', 3), ('right', 5.3)],
            'C_D': [('straight', 15.5), ('right', 5.2)],
            'D_A': [('straight', 3), ('right', 5.5)],
        }

        """ 
        self.paths = {
            'A_B': [('straight', 3), ('right', 20), ('straight', 3)],
            'B_C': [('straight', 3)],
            'D_A': [('straight', 3)],
            'D_C': [('straight', 3), ('left', 20), ('straight', 3)],
            'B_E': [('right', 7), ('straight', 3)],
            'E_D': [('straight', 3), ('left', 7.5)],
            'C_D': [('straight', 3), ('right', 20), ('straight', 3)],
            'C_E': [('left', 7.5), ('straight', 3)],
            'E_A': [('straight', 3), ('right', 7)]
        }
         """

        # 定义路线顺序
        self.route = ['A', 'B', 'C', 'D', 'A']
        # self.route = ['A', 'B', 'C', 'D', 'A', 'B', 'E', 'D', 'C', 'E', 'A']
        self.route_index = 0
        
        # 创建一个发布器
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建一个计时器，每0.1秒钟检查一次状态
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.command_index = 0

        # 创建暂停信号的订阅
        self.subscription = self.create_subscription(Bool, 'pause_signal', self.pause_callback, 1)
        
        self.paused = False
        self.pause_time = None
        self.remaining_duration = 0.0
        self.current_command = None
        self.command_thread = None

    def timer_callback(self):
        if self.paused:
            return
        
        if self.command_thread is not None and self.command_thread.is_alive():
            # 当前有命令在执行，等待命令完成
            return

        if self.route_index >= len(self.route) - 1:
            msg = Twist()
            msg.linear.x = msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y = msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Reached final destination.')
            self.timer.cancel()  # 停止计时器 
            return

        current_node = self.route[self.route_index]
        target_node = self.route[self.route_index + 1]
        path_key = f'{current_node}_{target_node}'

        if path_key not in self.paths:
            self.get_logger().warn(f'No path from {current_node} to {target_node}.')
            self.route_index += 1
            return

        path = self.paths[path_key]
        if self.command_index >= len(path):
            self.get_logger().info(f'Reached target node {target_node}.')
            self.route_index += 1
            self.command_index = 0
            return
        
        command = path[self.command_index]
        self.command_thread = threading.Thread(target=self.publish_move_command, args=(command,))
        self.command_thread.start()
        self.command_index += 1

    def publish_move_command(self, command):
        self.current_command = command
        msg = Twist()
        direction, duration = command
        
        if direction == 'straight':
            msg.linear.x = 0.1  # 设置直行速度
            msg.angular.z = 0.04
        elif direction == 'left':
            msg.linear.x = 0.1
            msg.linear.y = 0.5  # 设置左转速度
        elif direction == 'right':
            msg.linear.x = 0.1
            msg.linear.y = -0.475  # 设置右转速度

        """ if direction == 'straight':
            msg.linear.x = 0.1  # 设置直行速度
            msg.angular.z = 0.0
        elif direction == 'left':
            msg.linear.x = 0.1
            msg.angular.z = 0.2  # 设置左转速度
        elif direction == 'right':
            msg.linear.x = 0.1
            msg.angular.z = -0.19  # 设置右转速度 """
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving {direction} for {duration} seconds.')

        start_time = time.time()
        while time.time() - start_time < duration:
            if self.paused:
                self.remaining_duration = duration - (time.time() - start_time) + 1
                return
            time.sleep(0.1)
        
        self.remaining_duration = 0.0
        self.current_command = None

    def pause_callback(self, msg):
        if msg.data:
            self.pause()
        else:
            self.resume()

    def pause(self):
        self.paused = True
        self.pause_time = time.time()
        self.get_logger().info("小车暂停！")
        self.publish_stop_command()

    def resume(self):
        self.paused = False
        self.get_logger().info("恢复小车运动！")
        if self.remaining_duration > 0:
            command = self.current_command
            if command:
                self.command_thread = threading.Thread(target=self.publish_move_command, args=((command[0], self.remaining_duration),))
                self.command_thread.start()
        else:
            self.timer_callback()

    def publish_stop_command(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
