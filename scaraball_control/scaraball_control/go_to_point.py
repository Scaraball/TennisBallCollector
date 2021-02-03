from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist, Point, PoseArray
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
		super().__init__('ReadingLaser')
		self.publisher_command = self.create_publisher(Twist, 'cmd_vel', 10)  # queuesize=10
		self.publisher_nearball = self.create_publisher(Bool, 'near_ball', 10)
		self.publisher_relache = self.create_publisher(Bool, 'relache', 10)
		
		self.subscriber_laserscan = self.create_subscription(LaserScan, 'scan', self.clbk_laser, qos_profile_sensor_data)
		self.subscriber_inpince = self.create_subscription(Bool, 'in_pince', self.clbk_inpince, 0)
		self.subscriber_outpince = self.create_subscription(Bool, 'out_pince', self.clbk_outpince, 0)
		self.subscriber_balls = self.create_subscription(PoseArray, 'posBall', self.clbk_balls, 0)
		self.subscriber_pose = self.create_subscription(Odometry, 'odom', self.clbk_odom, 0)
		self.timer = self.create_timer(1/20., self.timer_callback)  # 20 Hz

		self.get_logger().info('Initialisation complete')

		self.initial_position = Point(x=1., y=5.)
		self.old_position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.yaw = 0.
		self.state = 0  # finite state machine
		self.left_storage_area, self.right_storage_area = Point(x=8., y=15.), Point(x=-8., y=-15.)
		self.desired_position = Point(x=self.initial_position.x, y=self.initial_position.y)
		self.desired_position = Point(x=0., y=-6.)
		
		self.balls = []
		self.ball_index = 0
		self.regions = None

		# parameters
		self.yaw_precision = math.pi / 90 # +/- 2 degree allowed
		self.dist_precision = 1.5  # longueur des pinces 0.6

# ----------------------- Callback for ROS Topics -----------------------

	def clbk_laser(self, msg):

		# 720 samples / 5 sectors = 5 sectors of 144 samples
		self.regions = {
			'right': min(min(msg.ranges[0:144]), 10),  
			'fright': min(min(msg.ranges[144:288]), 10),
			'front': min(min(msg.ranges[288:432]), 10),
			'fleft': min(min(msg.ranges[432:576]), 10),
			'left': min(min(msg.ranges[576:720]), 10),
		}

		self.publisher_nearball.publish(Bool(data=False))
		self.publisher_relache.publish(Bool(data=False))

		# self.get_logger().info(str(self.regions) + '\n')

	def clbk_odom(self, msg):
		self.old_position.x, self.old_position.y = self.position.x, self.position.y

		self.position.x, self.position.y = msg.pose.pose.position.x, msg.pose.pose.position.y
		quatx, quaty, quatz, quatw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
		
		self.yaw = self.euler_from_quaternion(quatx, quaty, quatz, quatw)[2]

	def timer_callback(self):
		if self.regions == None: return

		if self.state == 0: self.fix_yaw(self.desired_position)
		elif self.state == 1: self.go_straight_ahead(self.desired_position)
		elif self.state == 2: self.turn_left()
		elif self.state == 3: self.follow_wall()
		elif self.state == 4: self.turn_right()
		elif self.state == 5: self.done()

	def clbk_inpince(self, msg):
		if msg.data == True: 
			self.desired_position = self.right_storage_area if y < 0 else self.left_storage_area

	def clbk_outpince(self, msg):
		if msg.data == True:
			self.desired_position = self.balls[self.ball_index]
			self.ball_index = self.ball_index + 1
			if self.ball_index == 10: self.change_state(state=5)
		return

	def clbk_balls(self, msg):  # type pose
		for i in range(len(msg.poses)):
			self.balls[i] = [msg.poses[i].x, msg.poses[i].y]
		return

	# ----------------------- Additional functions -----------------------

	def fix_yaw(self, des_pos):
		desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
		err_yaw = desired_yaw - self.yaw

		twist_msg = Twist()
		if math.fabs(err_yaw) > self.yaw_precision:
			twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
		self.publisher_command.publish(twist_msg)

		# state change conditions
		if math.fabs(err_yaw) <= self.yaw_precision:
			msg = Twist()
			self.publisher_command.publish(msg)
			time.sleep(1)
			self.change_state(state=1)

	def go_straight_ahead(self, des_pos):
		desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
		err_yaw = desired_yaw - self.yaw
		err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))

		if self.regions["front"] < 1.5:
			msg = Twist()
			self.publisher_command.publish(msg)
			time.sleep(1)
			self.change_state(state=2)
			return

		if err_pos > self.dist_precision:
			twist_msg = Twist()
			twist_msg.linear.x = 0.6
			twist_msg.angular.z = err_yaw * 0.3
			self.publisher_command.publish(twist_msg)
		else:
			self.change_state(state=5)

	def turn_left(self):
		desired_yaw = 0.
		err_yaw = desired_yaw - self.yaw

		twist_msg = Twist()
		if math.fabs(err_yaw) > self.yaw_precision:
			twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
		self.publisher_command.publish(twist_msg)

		# state change conditions
		if math.fabs(err_yaw) <= self.yaw_precision:
			msg = Twist()
			self.publisher_command.publish(msg)
			time.sleep(1)
			self.change_state(state=3)

	def follow_wall(self):
		desired_yaw = 0.
		err_yaw = desired_yaw - self.yaw

		msg = Twist()
		msg.linear.x = 0.5
		msg.angular.z = 0.3 * err_yaw
		self.publisher_command.publish(msg)

		if self.regions["front"] < 1.5:
			msg = Twist()
			self.publisher_command.publish(msg)
			time.sleep(1)
			self.change_state(state=4)
			return

	def turn_right(self):
		desired_yaw = -np.pi/2 if self.position.y > 0 else np.pi/2
		err_yaw = desired_yaw - self.yaw

		twist_msg = Twist()
		if math.fabs(err_yaw) > self.yaw_precision:
			twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
		self.publisher_command.publish(twist_msg)

		# state change conditions
		if math.fabs(err_yaw) <= self.yaw_precision:
			msg = Twist()
			self.publisher_command.publish(msg)
			time.sleep(1)

			twist_msg = Twist()
			twist_msg.linear.x = 0.5
			self.publisher_command.publish(twist_msg)
			time.sleep(4)

			self.change_state(state=0)
	
	def done(self):
		twist_msg = Twist()
		self.publisher_command.publish(twist_msg)
		time.sleep(1)
		self.publisher_nearball.publish(Bool(data=True))

	def change_state(self, state): 
		self.state = state

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
