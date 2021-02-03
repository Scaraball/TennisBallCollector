from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float64, Int32
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


# ----------------------- ROS2 Car Node -----------------------


class ReadingLaserNode(Node):

	def __init__(self):
		super().__init__('ReadingLaser')
		self.subscriber_laserscan = self.create_subscription(LaserScan, 'scan', self.clbk_laser, qos_profile_sensor_data)
		self.publisher_command = self.create_publisher(Twist, 'cmd_vel', 10)  # queuesize=10
		self.get_logger().info('Initialisation complete')

# ----------------------- Callback for ROS Topics -----------------------

	def clbk_laser(self, msg):

		# 360 / 8 = 45

		regions = {
			'0': min(min(msg.ranges[0:45]), 10),
			'1': min(min(msg.ranges[45:90]), 10),
			'2': min(min(msg.ranges[90:135]), 10),
			'3': min(min(msg.ranges[135:180]), 10),
			'4': min(min(msg.ranges[180:225]), 10),
			'5': min(min(msg.ranges[225:270]), 10),
			'6': min(min(msg.ranges[270:315]), 10),
			'7': min(min(msg.ranges[315:360]), 10),
		}

		#self.get_logger().info(str(regions) + '\n')
		# self.take_action(regions)

	def take_action(regions):
		'''
		This function implements the obstacle avoidance logic. 
		Based on the distances sensed in the five region 
		(left, center-left, center, center-right, right). 
		We consider possible combinations for obstacles, 
		once we identify the obstacle configuration 
		we steer the robot away from obstacle.
		'''
		msg = Twist()
		linear_x = 0
		angular_z = 0

		state_description = ''

		if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
			state_description = 'case 1 - nothing'
			linear_x = 0.6
			angular_z = 0
		elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
			state_description = 'case 2 - front'
			linear_x = 0
			angular_z = 0.3
		elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
			state_description = 'case 3 - fright'
			linear_x = 0
			angular_z = 0.3
		elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
			state_description = 'case 4 - fleft'
			linear_x = 0
			angular_z = -0.3
		elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
			state_description = 'case 5 - front and fright'
			linear_x = 0
			angular_z = 0.3
		elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
			state_description = 'case 6 - front and fleft'
			linear_x = 0
			angular_z = -0.3
		elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
			state_description = 'case 7 - front and fleft and fright'
			linear_x = 0
			angular_z = 0.3
		elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
			state_description = 'case 8 - fleft and fright'
			linear_x = 0.3
			angular_z = 0
		else:
			state_description = 'unknown case'
			rospy.loginfo(regions)

		rospy.loginfo(state_description)
		msg.linear.x = linear_x
		msg.angular.z = angular_z
		self.publisher_command.publish(msg)

# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = ReadingLaserNode()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
