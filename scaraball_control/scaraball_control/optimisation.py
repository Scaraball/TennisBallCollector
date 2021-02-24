from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist,PoseArray,Pose
from std_msgs.msg import Float64, Int32,Bool
import numpy as np
import rclpy
import time
from rclpy.node import Node


class Optimisation(Node):
	
	def __init__(self):
		super().__init__('Optimisation')
		self.subscriber_postion_ball = self.create_subscription(PoseArray,"/posBall",self.callback_calc,1)
		self.subscriber_position_pince = self.create_subscription(Bool,'/catch',self.callback_catch, 10)
		self.publisher_next_ball_pos = self.create_publisher(Pose,'/next_pos',10)

		self.get_logger().info('Initialisation complete')

		self.stockage_area_gauche = np.array([[7.0, 13.7]]).T
		self.stockage_area_droite = -self.stockage_area_gauche
		self.state = 1  # at starting point, the state will change to 0 which is "going to ball"
		self.go_stock = False
		self.two_balls = False
		self.last_data = False

	def callback_calc(self,msg):

		# on commence par initialiser la position de la nouvelle balle à aller chercher et la distance du détour
		self.new_ball = np.array([[None], [None]])
		self.distance_detour = float('inf')

		# on récupère la position de la balle vers laquelle on va
		self.ball_1_position = np.array([[msg.poses[0].position.x, msg.poses[0].position.y]]).T

		if self.ball_1_position[1,0] < 0:
			self.stockage = self.stockage_area_droite
			y = -1
		
		else :
			self.stockage = self.stockage_area_gauche
			y = 1

	def callback_catch(self,msg):
		if msg.data==True:
			self.last_data = msg.data
			self.state += 1

			if self.state % 2 == 0: self.go_stock = False
			else: self.go_stock = True

			if self.go_stock:
				position = Pose()
				position.position.x = self.stockage[0,0]
				position.position.y = self.stockage[1,0]
				self.publisher_next_ball_pos.publish(position)
			else:
				position = Pose()
				position.position.x =  self.ball_1_position[0, 0]
				position.position.y =  self.ball_1_position[1, 0]
				self.publisher_next_ball_pos.publish(position)


# --------------------------------------------------------------------


def main(args=None):
	rclpy.init(args=args)

	node = Optimisation()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
