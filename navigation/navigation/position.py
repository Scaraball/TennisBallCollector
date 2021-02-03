import sys
import time
import cv2
import numpy as np
import random
import math
import os
import signal
import glob
import matplotlib.pyplot as plt
import cv_bridge
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan,Image
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose,PoseArray

#




class position_node(Node):

    def __init__(self):
        super().__init__("position")
        print("init")
        self.cmd_pos_publisher = self.create_publisher(PoseArray,"/pos",10)
        self.cmd_posRob_publisher = self.create_publisher(Pose, "/posRob", 10)
        self.lidar_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.img_callback, qos_profile_sensor_data)
        time.sleep(2)



    def img_callback(self,msg):

        pose_array = PoseArray()


        br = cv_bridge.CvBridge()
        zenith_cam = br.imgmsg_to_cv2(msg)
        cvImg = cv2.cvtColor(zenith_cam, cv2.COLOR_RGB2BGR)
        cvImg_bis = cvImg.copy()
        lower_y = np.array([10, 150, 100])
        upper_y = np.array([40, 255, 255])

        hsv = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh = cv2.inRange(hsv, lower_y, upper_y)
        contours, hierarchy = cv2.findContours(tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("coordonnées")
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            pose = Pose()
            if area < 1000 and area > 10:
                if (peri != 0):
                    # if abs((4 * np.pi * area / (peri) ** 2) - 1) < 0.4:
                        # print("tout est en ordre, circulez")
                    M = cv2.moments(c)

                    if (M["m00"] != 0):
                        # print("Je suis M")

                        cX = int(M["m10"] / M["m00"]) - 33
                        cY = int(M["m01"] / M["m00"]) - 39

                        print(cX,cY)
                        pose.position.x = float(cX)
                        pose.position.y = float(cY)

                        cv2.circle(cvImg_bis, (cX+33, cY+39), 5, (0, 255, 0), -1)
            pose_array.poses.append(pose)

        lower_g = np.array([36, 0, 0])
        upper_g = np.array([86, 255, 255])

        hsv = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh = cv2.inRange(hsv, lower_g, upper_g)
        contours, hierarchy = cv2.findContours(tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("robot position")
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            pose = Pose()
            if area > 100:
                if (peri != 0):
                    # if abs((4 * np.pi * area / (peri) ** 2) - 1) < 0.4:
                    # print("tout est en ordre, circulez")
                    M = cv2.moments(c)

                    if (M["m00"] != 0):
                        # print("Je suis M")

                        cX = int(M["m10"] / M["m00"]) - 33
                        cY = int(M["m01"] / M["m00"]) - 39

                        print(cX, cY)
                        pose.position.x = float(cX)
                        pose.position.y = float(cY)

                        cv2.circle(cvImg_bis, (cX + 33, cY + 39), 5, (255, 255, 0), -1)




        cv2.imshow('real',cvImg)
        cv2.imshow('tresh', tresh)
        cv2.imshow('balls',cvImg_bis)
        cv2.waitKey(1)
        self.cmd_pos_publisher.publish(pose_array)
        self.cmd_posRob_publisher.publish(pose)



def main():

    rclpy.init()
    position = position_node()
    rclpy.spin(position)




if __name__ =="__main__":
    main()