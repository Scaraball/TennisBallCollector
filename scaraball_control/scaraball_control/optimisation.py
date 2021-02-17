from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist,PoseArray,Pose
from std_msgs.msg import Float64, Int32,Bool
import numpy as np
import rclpy
import time
from rclpy.node import Node


class Opitmisation(Node):
	
	def __init__(self):
		super().__init__('Optimisation')
		self.subscriber_postion_ball = self.create_subscription(PoseArray,"/posBall",self.callback_calc,1)
		self.subscriber_position_pince = self.create_subscription(Bool,'/catch',self.callback_catch,10)
		self.publisher_next_ball_pos = self.create_publisher(Pose,'/next_pos',10)

		self.get_logger().info('Initialisation complete')


		self.stockage_area_gauche = np.array([[7.0,13.7]]).T
		self.stockage_area_droite = -self.stockage_area_gauche
		self.state = 0
		self.go_stock = False
		self.two_balls = False
		self.last_data = False

	def callback_calc(self,msg):

		# on commence par initialiser la position de la nouvelle balle à aller chercher et la distance du détour
		self.new_ball = None  
		self.distance_detour = float('inf')

		# on récupère la position de la balle vers laquelle on va
		self.ball_1_position = np.array([[msg.poses[0].position.x,msg.poses[0].position.y]]).T 

		# Selction de la partie du terrain dans laquelle le robot se trouve
		if self.ball_1_position[1,0] < 0:
			self.stockage = self.stockage_area_droite
			y = -1
		else :
			self.stockage = self.stockage_area_gauche
			y = 1

		# Distance entre la balle courante et la distance de stockage
		self.distance_to_stock = np.linalg.norm(self.ball_1_position-self.stockage)


		# on parcourt la liste des balles et on regarde celle qui pourrait être collectée sans grand détour
		# parcours de la dernière apparue à la deuxième, pour sélectionner en priorité la balle apparue en premier
		for ball_pos in msg.poses[:0:-1]: 

			# position de la balle sélectionnée
			self.ball_2_position = np.array([[ball_pos.position.x,ball_pos.position.y]]).T

			# calcul de la distance entre la balle sélectionnée et la zone de stockage
			ball2_to_stock = np.linalg.norm(self.ball_2_position-self.stockage)
			# calcul de la distance entre la balle courante et la balle sélectionnée
			ball1_to_ball2 = np.linalg.norm(self.ball_1_position-self.ball_2_position)

			# si le détour est inférieur à 20% de la distance et que les balles sont du même côté du terrain
			if (ball2_to_stock +ball1_to_ball2)  < 1.2 * self.distance_to_stock and (self.ball_2_position[1,0]*y)>0:
				
				# si il existe déjà une balle qui peut être ramassée, on regarde si l'autre balle vaut le coup
				# plus la balle est arrivée tôt plus elle sera priorisée
				if  (ball2_to_stock +ball1_to_ball2) < 1.05*self.distance_detour:
					self.distance_detour = (ball2_to_stock +ball1_to_ball2)
					self.new_ball = self.ball_2_position

		if self.new_ball.all() != None and self.two_balls:
			position = Pose()
			position.position.x =  self.new_ball[0,0]
			position.position.y =  self.new_ball[1,0]
			self.publisher_next_ball_pos.publish(position)
		elif self.go_stock:
			position = Pose()
			position.position.x =  self.stockage[0,0]
			position.position.y =  self.stockage[1,0]
			self.publisher_next_ball_pos.publish(position)
		else:
			position = Pose()
			position.position.x =  msg.poses[0].position.x
			position.position.y =  msg.poses[0].position.y
			self.publisher_next_ball_pos.publish(position)


	def callback_catch(self,msg):
		self.get_logger().info(('state = '+str(self.state)))
		if msg.data != self.last_data:
			if msg.data == True: 
				self.state += 1
				self.last_data = True
			if msg.data == False: 
				self.last_data = False
		if self.state == 1:
			self.two_balls = True
			self.go_stock = False
		if self.state == 2:
			self.two_balls = False
			self.go_stock = True
		if self.state > 2:
			self.state = 0
			self.two_balls =False
			self.go_stock = False



def main(args=None):
	rclpy.init(args=args)

	node = Opitmisation()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()




