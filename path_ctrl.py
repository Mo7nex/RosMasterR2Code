import heapq
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.task import Future

def dijkstra(adj_matrix, start, end):
    n = len(adj_matrix)
    dist = [float('inf')] * n
    prev = [None] * n
    dist[start] = 0
    pq = [(0, start)]

    while pq:
        current_dist, u = heapq.heappop(pq)

        if u == end:
            break

        if current_dist > dist[u]:
            continue

        for v in range(n):
            if adj_matrix[u][v] != 0:  # there is an edge
                alt = dist[u] + adj_matrix[u][v]
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(pq, (alt, v))

    # Reconstruct the shortest path
    path = []
    u = end
    while prev[u] is not None:
        path.insert(0, u)
        u = prev[u]
    if path:
        path.insert(0, u)

    return path

def index_to_grid(index):
    return (index // 3, index % 3)

def generate_instructions(path_coords):
    instructions = []
    current_pos = path_coords[0]
    current_direction = 'n'  # 初始方向为北

    for next_pos in path_coords[1:]:
        # 计算当前位置和下一位置的坐标差
        delta_x = next_pos[0] - current_pos[0]
        delta_y = next_pos[1] - current_pos[1]

        # 确定小车的移动方向
        if delta_x == 1:  # 向北移动
            if current_direction == 'n':
                instructions.append('forward')
            elif current_direction == 's':
                instructions.extend(['left', 'left', 'forward'])
            elif current_direction == 'w':
                instructions.extend(['right', 'forward'])
            elif current_direction == 'e':
                instructions.extend(['left', 'forward'])
            current_direction = 'n'
        elif delta_x == -1:  # 向南移动
            if current_direction == 'n':
                instructions.extend(['left', 'left', 'forward'])
            elif current_direction == 's':
                instructions.append('forward')
            elif current_direction == 'w':
                instructions.extend(['left', 'forward'])
            elif current_direction == 'e':
                instructions.extend(['right', 'forward'])
            current_direction = 's'
        elif delta_y == 1:  # 向东移动
            if current_direction == 'n':
                instructions.extend(['right', 'forward'])
            elif current_direction == 's':
                instructions.extend(['left', 'forward'])
            elif current_direction == 'w':
                instructions.extend(['left', 'left', 'forward'])
            elif current_direction == 'e':
                instructions.append('forward')
            current_direction = 'e'
        elif delta_y == -1:  # 向西移动
            if current_direction == 'n':
                instructions.extend(['left', 'forward'])
            elif current_direction == 's':
                instructions.extend(['right', 'forward'])
            elif current_direction == 'w':
                instructions.append('forward')
            elif current_direction == 'e':
                instructions.extend(['left', 'left', 'forward'])
            current_direction = 'w'

        current_pos = next_pos  # 更新当前位置

    return instructions

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(Bool, 'pause_signal', self.pause_callback, 1)
        self.instructions = []
        self.shortest_path_coords = []
        self.instruction_index = 0
        self.stopped = False
        self.stop_sent = False
        self.timer = None
        self.paused = False
        self.remaining_duration = 0.0
        self.current_time = None  # 记录当前时间
        self.pause_time = None  # 记录暂停时间
        self.finished_future = Future() 

    def set_instructions(self, instructions, shortest_path_coords):
        self.instructions = instructions
        self.shortest_path_coords = shortest_path_coords

    def execute_instructions(self):
        self.time_callback()

    def time_callback(self):
        if self.paused:
            return

        if self.instruction_index < len(self.instructions):
            instruction = self.instructions[self.instruction_index]
            twist = Twist()

            if self.remaining_duration <= 0:
                self.remaining_duration = self.get_duration_for_instruction(instruction)

            if instruction == 'forward':
                twist.linear.x = 0.1
                twist.angular.z = 0.03
            elif instruction == 'left':
                twist.linear.x = 0.1
                twist.linear.y = 0.5
            elif instruction == 'right':
                twist.linear.x = 0.1
                twist.linear.y = -0.5
            elif instruction == 'backward':
                twist.linear.x = -0.1
                twist.angular.z = 0.0

            self.get_logger().info(f"执行指令 {instruction}：剩余 {self.remaining_duration} 秒")
            self.current_time = time.time()  # 记录执行指令时的当前时间
            self.publisher_.publish(twist)
            self.publisher_.publish(twist)
            self.publisher_.publish(twist)
            self.get_logger().info(f"起始时间：{self.current_time}")

            if self.timer:
                self.timer.cancel()
            self.timer = self.create_timer(self.remaining_duration, self.next_instruction)
        else:
            self.get_logger().info("停止小车！")
            self.stop()

    def next_instruction(self):
        self.remaining_duration = 0.0
        self.instruction_index += 1
        self.time_callback()

    def get_duration_for_instruction(self, instruction):
        
        index = 0
        i = 0
        while (i <= self.instruction_index):
            if self.instructions[i] == 'forward':
                index = index + 1
            i = i + 1
        
        # 左转之后直行向左偏转
        # 定义一个字典，存储坐标对及其对应的 defult_time
        time_lookup = {
			((0, 0), (0, 1)): 9.5, # b 
			((0, 0), (1, 0)): 8, # start 
			((0, 1), (0, 0)): 9.5, # b 
			((0, 1), (0, 2)): 9.5, # a 
			((0, 1), (1, 1)): 8, # start 
			((0, 2), (0, 1)): 9.5, # a 
			((0, 2), (1, 2)): 8, # start 
			((1, 0), (0, 0)): 8, # start 
			((1, 0), (1, 1)): 9.5, # b 
			((1, 0), (2, 0)): 17.5, # c  
			((1, 1), (0, 1)): 8, # start 
			((1, 1), (1, 0)): 9.5, # b 
			((1, 1), (1, 2)): 9.5, # a 
			((1, 1), (2, 1)): 17.5, # c  
			((1, 2), (0, 2)): 8, # start 
			((1, 2), (1, 1)): 9.5, # a 
			((1, 2), (2, 2)): 17.5, # c  
			((2, 0), (1, 0)): 16, # c  
			((2, 0), (2, 1)): 9.5, # b 
			((2, 1), (1, 1)): 17.5, # c  
			((2, 1), (2, 0)): 9.5, # b 
			((2, 1), (2, 2)): 9.5, # a 
			((2, 2), (1, 2)): 17.5, # c  
			((2, 2), (2, 1)): 9.5, # a 
		}

		# 使用字典查找 defult_time
        if index == 0:
            current_coords = self.shortest_path_coords[index]
            next_coords = self.shortest_path_coords[index + 1]
            if (current_coords, next_coords) in time_lookup:
                defult_time = time_lookup[(current_coords, pre_coords)]
        else:
            if index < len(self.shortest_path_coords):
                current_coords = self.shortest_path_coords[index]
                pre_coords = self.shortest_path_coords[index - 1]
            if (pre_coords, current_coords) in time_lookup:
                defult_time = time_lookup[(current_coords, pre_coords)]
        self.get_logger().info(str(defult_time))

        if instruction == 'forward':
            if self.instruction_index == 0:
                if len(self.instructions) > 1:
                    instruction_next = self.instructions[self.instruction_index + 1]
                    if instruction_next in ['left', 'right']:
                        return (defult_time - 1.75)
            else:
                instruction_pre = self.instructions[self.instruction_index - 1]
                instruction_next = self.instructions[self.instruction_index + 1] if self.instruction_index + 1 < len(self.instructions) else None

                if (instruction_pre in ['left', 'right']) and (instruction_next in ['left', 'right']):
                    return (defult_time - 3.5)
                else:
                    if (instruction_pre in ['left', 'right']) or (instruction_next in ['left', 'right']):
                        return (defult_time - 1.75)
                
            return (defult_time)
        elif instruction == 'left':
            return (5.2)
        elif instruction == 'right':
            return (5.3)
        elif instruction == 'backward':
            return 3.0
        return defult_time

    def stop(self):
        if not self.stop_sent:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.publisher_.publish(twist)
            self.publisher_.publish(twist)
            self.stop_sent = True
            self.finished_future.set_result(True)

    def pause(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.publisher_.publish(twist)
        self.publisher_.publish(twist)
        self.paused = True
        self.pause_time = time.time()  # 记录暂停时的当前时间
        if self.timer:
            self.get_logger().info(f"暂停时间：{self.pause_time}")
            self.remaining_duration = self.remaining_duration - (self.pause_time - self.current_time) + 1
            self.timer.cancel()
        self.get_logger().info("小车暂停！")

    def resume(self):
        self.paused = False
        self.get_logger().info("恢复小车运动！")
        self.time_callback()

    def pause_callback(self, msg):
        if msg.data:
            self.pause()
        else:
            self.resume()


def main(args=None):
    # 计算最短路径
    adj_matrix = [
        [0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0],
        [1, 0, 0, 0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1, 0, 1, 0, 1],
        [0, 0, 0, 0, 0, 1, 0, 1, 0],
    ]

    start = int(input("Enter start node (0-8): "))
    end = int(input("Enter end node (0-8): "))
    shortest_path = dijkstra(adj_matrix, start, end)
    shortest_path_coords = [index_to_grid(node) for node in shortest_path]
    print(f"Shortest path from node {index_to_grid(start)} to node {index_to_grid(end)} is: {shortest_path_coords}")

    instructions = generate_instructions(shortest_path_coords)
    print("Instruction sequence: ", instructions)

    rclpy.init(args=args)
    path_controller = PathController()
    path_controller.set_instructions(instructions, shortest_path_coords)
    path_controller.execute_instructions()

    # 等待指令执行完成
    rclpy.spin_until_future_complete(path_controller, path_controller.finished_future)

    print("All instructions completed. Shutting down.")
    path_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
