from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist, Point, PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import *

import numpy as np
import math
import time


# ----------------------- ROS2 Car Node -----------------------


class ControllerNode(Node):

	def __init__(self):
		super().__init__('Controller')
		self.publisher_command = self.create_publisher(Twist, 'cmd_roues', 10)  # queuesize=10
		self.publisher_nearball = self.create_publisher(Bool, 'near_ball', 10)
		self.publisher_relache = self.create_publisher(Bool, 'relache', 10)
		self.publisher_catch = self.create_publisher(Bool, 'catch', 10)

		self.subscriber_inpince = self.create_subscription(Bool, 'in_pince', self.clbk_inpince, 0)
		self.subscriber_outpince = self.create_subscription(Bool, 'out_pince', self.clbk_outpince, 0)
		# self.subscriber_balls = self.create_subscription(PoseArray, 'posBall', self.clbk_balls, 0)
		self.subscriber_rob = self.create_subscription(Pose, 'posRob', self.clbk_rob, 0)
		self.subscriber_pose = self.create_subscription(Odometry, 'odom_roues', self.clbk_odom, 0)
		self.subscriber_target = self.create_subscription(Pose, 'next_pos', self.clbk_target, 0)
		self.timer = self.create_timer(1/20., self.timer_callback)  # 20 Hz

		self.get_logger().info('Initialisation complete')

		self.initial_position = Point(x=0., y=5.)
		self.old_position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.yaw = 0.
		self.state = 0  # finite state machine
		self.desired_position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.going_to_ball = True
		self.started = False
		self.taking_ball = False

		self.balls = []
		self.regions = None

		# parameters
		self.yaw_precision = math.pi / 90 # +/- 2 degree allowed
		self.dist_precision = 1.  # longueur des pinces 0.6

# ----------------------- Callback for ROS Topics -----------------------

	def clbk_odom(self, msg):
		self.position.x, self.position.y = msg.pose.pose.position.x, msg.pose.pose.position.y
		quatx, quaty, quatz, quatw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
		
		self.yaw = self.euler_from_quaternion(quatx, quaty, quatz, quatw)[2]

	def timer_callback(self):
		if not self.started: return

		self.publisher_catch.publish(Bool(data=False))

		if self.state == 0: self.move_to()
		elif self.state == 1: self.done()

	def clbk_inpince(self, msg):
		print("In pince :", msg.data)
		if msg.data == True:
			self.going_to_ball = False
			self.publisher_catch.publish(Bool(data=True))
			self.change_state(state=0) 
			

	def clbk_outpince(self, msg):
		print("Out pince :", msg.data)
		if msg.data == True:
			self.going_to_ball = True
			self.publisher_catch.publish(Bool(data=True))
			self.change_state(state=0)
			
	"""def clbk_balls(self, msg):  # type pose array
		for i in range(len(msg.poses)):
			self.balls.append( [msg.poses[i].position.x, msg.poses[i].position.y] )

		if self.going_to_ball:
			self.desired_position = Point(x=self.balls[0][0], y=self.balls[0][1])

		if not self.started:
			self.desired_position = Point(x=self.balls[0][0], y=self.balls[0][1])
			self.started = True"""

	def clbk_target(self, msg):
		self.desired_position = Point(x=msg.position.x, y=msg.position.y)
		print(self.desired_position)
		self.started = True

	def clbk_rob(self, msg):  # pose
		self.position.x, self.position.y = msg.position.x, msg.position.y

	# ----------------------- Additional functions -----------------------
	
	def move_to(self):

		p = np.array([[self.position.x], [self.position.y]])		
		self.phat = np.array([[self.desired_position.x], [self.desired_position.y]])		
		w = - 1. * (p - self.phat)
		# w = w + (p - qhat) / norm(p - qhat) ** 3
		
		tetabar = np.arctan2(w[1, 0], w[0, 0])
		u = self.sawtooth(tetabar - self.yaw)
		
		dist_to_waypoint = np.linalg.norm(p-self.phat)
		if dist_to_waypoint <= 1.: 
			self.change_state(state=1)

		msg_cmd = Twist()
		msg_cmd.linear.x = 0.5
		msg_cmd.angular.z = u
		self.publisher_command.publish(msg_cmd)  # angular velocity command (yaw)

	def done(self):
		if not self.taking_ball:
			twist_msg = Twist()
			self.publisher_command.publish(twist_msg)
			time.sleep(1)

		if self.started:
			if self.going_to_ball:
				self.taking_ball = True
				self.publisher_nearball.publish(Bool(data=True))
				self.publisher_relache.publish(Bool(data=False))
			else:
				self.publisher_nearball.publish(Bool(data=False))
				self.publisher_relache.publish(Bool(data=True))

	def change_state(self, state): 
		self.state = state
	
	def sawtooth(self, x): return (x + np.pi) % (2 * np.pi) - np.pi   # or equivalently   2*arctan(tan(x/2))

	def euler_from_quaternion(self, x, y, z, w):
		"""
		Converts quaternion (w in last place) to euler roll, pitch, yaw
		quaternion = [x, y, z, w]
		Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
		"""

		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = np.arctan2(sinr_cosp, cosr_cosp)

		sinp = 2 * (w * y - z * x)
		pitch = np.arcsin(sinp)

		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = np.arctan2(siny_cosp, cosy_cosp)

		return roll, pitch, yaw


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = ControllerNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
