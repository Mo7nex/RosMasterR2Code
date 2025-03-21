#!/usr/bin/env python
# encoding: utf-8
#import public lib
from geometry_msgs.msg import Twist
import sys, select, termios, tty

#import ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
W A S D

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'd': (1, -1),
    'a': (1, 1),
    's': (-1, 0),
    'W': (1, 0),
    'D': (1, -1),
    'A': (1, 1),
    'S': (-1, 0)
}

speedBindings = {
    'u': (1.1, 1.1),
    'm': (.9, .9),
    'i': (1.1, 1),
    ',': (.9, 1),
    'o': (1, 1.1),
    '.': (1, .9),
    'U': (1.1, 1.1),
    'M': (.9, .9),
    'I': (1.1, 1),
    'O': (1, 1.1),
}

class Keyboard(Node):
	def __init__(self, name):
		super().__init__(name)
		self.pub = self.create_publisher(Twist,'cmd_vel',1)
		self.declare_parameter("linear_speed_limit",1.0)
		self.declare_parameter("angular_speed_limit",5.0)
		self.linenar_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value
		self.settings = termios.tcgetattr(sys.stdin)
		
	#从键盘读取键位
	def getKey(self): 
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist: key = sys.stdin.read(1)
		else: key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key
		
	#打印返回值
	def vels(self, speed, turn):
		return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main():
	rclpy.init()   #初始化ROS2客户端库rclpy
	keyboard = Keyboard("keyboard_ctrl_demo")
	xspeed_switch = True   #初始化变量xspeed_switch，用于控制速度控制是在X轴或Y轴
	(speed, turn) = (0.2, 1.0)   #初始化机器人的线速度speed和角速度turn
	(x, th) = (0, 0)   #初始化机器人的X轴和Y轴的速度控制变量x和th，都设置为0
	status = 0   #初始化一个变量status，用于控制消息提示的显示状态
	stop = False   #初始化一个变量stop，用于控制机器人是否停止
	count = 0   #初始化一个计数器count
	twist = Twist()   #创建一个Twist消息实例
	
	#编写具体操作细节
	try:
		print(msg)   #打印指令说明
		print(keyboard.vels(speed, turn))   #打印出当前的线速度和角速度
		while (1):
			key = keyboard.getKey()
			if key == "t" or key == "T":   #如果用户按下't'或'T'键，切换速度控制的轴
				xspeed_switch = not xspeed_switch
			elif key == "k" or key == "K":   #切换键盘控制的停止状态
				print ("stop keyboard control: {}".format(not stop))
				stop = not stop
			if key in moveBindings.keys():   #如果按下的键在moveBindings字典的键中，执行相应的移动操作
				x = moveBindings[key][0]
				th =moveBindings[key][1]
				count = 0
			elif key in speedBindings.keys():   #如果按下的键在speedBindings字典的键中，调整速度
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				count = 0
				if speed > keyboard.linenar_speed_limit:
					speed = keyboard.linenar_speed_limit
					print("Linear speed limit reached!")
				if turn > keyboard.angular_speed_limit:
					turn = keyboard.angular_speed_limit
					print("Angular speed limit reached!")
				print (keyboard.vels(speed, turn))
				if (status == 14):   #更新status变量的值，用于控制消息提示的显示
					print(msg)
				status = (status + 1) % 15
			elif key == " ":   #空格键，停止机器人
				(x, th) = (0, 0)
			else:
				count = count + 1
				if count > 4:
					(x, th) = (0, 0)
				if (key == '\x03'):   #如果用户按下CTRL-C，退出循环
					break
			if xspeed_switch:    #设置线速度的分量
				twist.linear.x = speed * x
			else:
				twist.linear.y = speed * x
			twist.angular.z = turn *th
			if not stop:
				keyboard.pub.publish(twist)
			if stop:    #如果停止控制，发布一个空的Twist消息来停止机器人
				keyboard.pub.publish(Twist())
	except Exception as e:   #捕获并打印任何异常信息
		print(e)
	finally:   #开始finally块，用于在异常发生时或正常结束时执行的代码
		keyboard.pub.publish(Twist())   #发布一个空的Twist消息来停止机器人
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keyboard.settings)   #恢复标准输入的原始设置
	keyboard.destroy_node()   #销毁节点
	rclpy.shutdown()   #关闭rclpy客户端库
