from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float64, Int32,Bool
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool


# ----------------------- ROS2 Pinces Node -----------------------


class ControlPinces(Node):
	
	def __init__(self):
		super().__init__('ControlPinces')
		self.subscriber_postion_ok = self.create_subscription(Bool,'near_ball',self.callback_initialize,1)
		self.subscriber_relache_ok = self.create_subscription(Bool,'relache',self.callback_relache,1)
		self.publisher_pinces_commande = self.create_publisher(Twist, 'cmd_pince', 10)
		self.publisher_roues_commande = self.create_publisher(Twist, 'cmd_roues', 10)
		self.publisher_in_pince = self.create_publisher(Bool,'in_pince',10)
		self.publisher_out_pince = self.create_publisher(Bool,'out_pince',10)
		self.get_logger().info('Initialisation complete')
		self.take_ball = True
		self.relache_ball = True

		self.cli = self.create_client(SetBool, '/switch')
		self.req = SetBool.Request()
		self.req.data = False
		self.cli.call_async(self.req)

	def callback_initialize(self,msg):
		if msg.data == True and self.take_ball:
			self.ouvre(5)
			self.stop_ouverture_fermeture()
			# self.avance()
			# self.stop()

			self.req.data = True
			self.cli.call_async(self.req)
			time.sleep(3)

			self.ferme(5)
			self.stop_ouverture_fermeture()
			self.recule(1)
			self.stop()

			self.req.data = False
			self.cli.call_async(self.req)
			time.sleep(3)
			
			self.take_ball = False
			self.publisher_in_pince.publish(Bool(data=True))
		if msg.data == False:
			self.take_ball = True

	def callback_relache(self, msg):
		if msg.data == True and self.relache_ball:
			self.ouvre(3)
			self.stop_ouverture_fermeture()
			self.recule(2)
			self.stop()
			self.ferme(1)
			self.stop_ouverture_fermeture()
			self.relache_ball = False
			self.publisher_out_pince.publish(Bool(data=True))
		if msg.data == False:
			self.relache_ball = True
			
	def ouvre(self,t):
		print('ouvre')
		control = Twist()
		control.angular.z = 0.2 
		self.publisher_pinces_commande.publish(control)
		time.sleep(t)

	def stop_ouverture_fermeture(self):
		print('stop')
		control = Twist()
		control.angular.z = 0.0
		self.publisher_pinces_commande.publish(control)
		time.sleep(0.2)

	def avance(self):
		print('avance')
		self.stop()

		control = Twist()
		control.linear.x = 0.5
		self.publisher_roues_commande.publish(control)
		time.sleep(2)

	def stop(self):
		print('stop')
		control = Twist()
		control.linear.x = 0.0
		control.angular.z = 0.0
		self.publisher_roues_commande.publish(control)
		time.sleep(0.5)

	def ferme(self,t):
		print('ferme')
		control = Twist()
		control.angular.z = -0.4 
		self.publisher_pinces_commande.publish(control)
		time.sleep(t)

	def recule(self, t):
		print('recule')
		self.stop()

		
		control = Twist()
		control.linear.x = -0.5
		control.angular.z = 0.0
		self.publisher_roues_commande.publish(control)
		time.sleep(t)


# ----------------------- Main program -----------------------


def main(args=None):
	rclpy.init(args=args)

	node = ControlPinces()
	rclpy.spin(node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
