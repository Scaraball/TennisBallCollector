from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
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
		self.get_logger().info('Initialisation complete')

# ----------------------- Callback for ROS Topics -----------------------

	def clbk_laser(self, msg):

		# 360 / 10 = 36

		regions = [
		min(min(msg.ranges[0:36]), 10),
		min(min(msg.ranges[36:72]), 10),
		min(min(msg.ranges[72:108]), 10),
		min(min(msg.ranges[108:144]), 10),
		min(min(msg.ranges[144:180]), 10),
		min(min(msg.ranges[180:216]), 10),
		min(min(msg.ranges[216:252]), 10),
		min(min(msg.ranges[252:288]), 10),
		min(min(msg.ranges[288:324]), 10),
		min(min(msg.ranges[324:360]), 10),
		]

		self.get_logger().info(str(regions))


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
