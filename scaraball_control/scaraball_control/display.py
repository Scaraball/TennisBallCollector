import numpy as np
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float64MultiArray, Int32
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

np.seterr(divide='ignore', invalid='ignore')  
# avoid warning when a value of the meshgrid X1, X2 is equal to an obstacle position (when dividing by norm draw_field function)


# ----------------------- ROS2 Display Node -----------------------


class DisplayNode():

	def __init__(self):
	
		# Initialize and clear ax
		self.xmin, self.xmax, self.ymin, self.ymax = -7.52, 7.52, -14.5, 14.5
		self.fig = plt.figure(0)
		self.ax = self.fig.add_subplot(111, aspect='equal')	
		self.ax.xmin, self.ax.xmax, self.ax.ymin, self.ax.ymax = self.xmin, self.xmax, self.ymin, self.ymax
		plt.cla()
		self.ax.set_xlim(self.ax.xmin, self.ax.xmax)
		self.ax.set_ylim(self.ax.ymin, self.ax.ymax)

		self.yrange = np.linspace(-14.8, 14.8, 100)
		self.wall1 = [np.array([[7.8], [y]]) for y in self.yrange]
		self.wall3 = [np.array([[-7.8], [y]]) for y in self.yrange]

		self.xrange = np.linspace(-7.8, 7.8, 100)
		self.wall2 = [np.array([[x], [-14.8]]) for x in self.xrange]
		self.wall4 = [np.array([[x], [14.8]]) for x in self.xrange]

		self.netrange = np.linspace(-5.5, 5.5, 100)
		self.net = [np.array([[x], [0.]]) for x in self.netrange]
		self.circle1, self.circle2 = [], []

		self.miniyrange, self.minixrange = np.linspace(14.5, 14.5-0.80, 10), np.linspace(7.52, 7.52-0.8 ,10)
		self.miniwall_1 = [np.array([[-5.73], [-y]]) for y in self.miniyrange]
		self.miniwall_2 = [np.array([[5.73], [y]]) for y in self.miniyrange]
		self.miniwall_3 = [np.array([[-x], [-12.33]]) for x in self.minixrange]
		self.miniwall_4 = [np.array([[x], [12.33]]) for x in self.minixrange]

		Mx = np.arange(self.xmin, self.xmax, 0.5)
		My = np.arange(self.ymin, self.ymax, 0.5)
		X1, X2 = np.meshgrid(Mx, My)

		self.phat = np.array([[7.0], [13.7]])
		VX = - 2 * (X1 - self.phat[0])
		VY = - 2 * (X2 - self.phat[1])

		for qhat in self.net: 
			VX = VX + (X1 - qhat[0]) / np.sqrt( (X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 3
			VY = VY + (X2 - qhat[1]) / np.sqrt( (X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 3

		for qhat in self.miniwall_1: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 10
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 10

		for qhat in self.miniwall_2: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 6
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 6

		for qhat in self.miniwall_3: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 10
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 10

		for qhat in self.miniwall_4: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 6
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 6

		for qhat in self.wall1: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11

		for qhat in self.wall2: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11

		for qhat in self.wall3: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11

		for qhat in self.wall4: 
			VX = VX + (X1 - qhat[0]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11
			VY = VY + (X2 - qhat[1]) / np.sqrt((X1 - qhat[0]) ** 2 + (X2 - qhat[1]) ** 2) ** 11



		# for qhat in self.wall1: w = w + (p - qhat) / np.linalg.norm(p - qhat) ** 8
		# for qhat in self.wall3: w = w + (p - qhat) / np.linalg.norm(p - qhat) ** 8
		# for qhat in self.wall2: w = w + (p - qhat) / np.linalg.norm(p - qhat) ** 8
		# for qhat in self.wall4: w = w + (p - qhat) / np.linalg.norm(p - qhat) ** 8

		R = np.sqrt(VX**2 + VY**2)  # normalisation
		
		self.ax.quiver(Mx, My, VX/R, VY/R)

		plt.show()

# -----------------------------------


if __name__ == '__main__':
	DisplayNode()
