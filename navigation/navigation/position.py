import time
import cv2
import numpy as np
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
        self.cmd_posBall_publisher = self.create_publisher(PoseArray,"/posBall",10)
        self.cmd_posRob_publisher = self.create_publisher(Pose, "/posRob", 10)
        self.lidar_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.img_callback, qos_profile_sensor_data)
        time.sleep(2)



    def img_callback(self,msg):

        pose_array = PoseArray()
        pose_rob = Pose()

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

                        # cX = int(M["m10"] / M["m00"]) - 33
                        # cY = int(M["m01"] / M["m00"]) - 39

                        cY = 640 - int(M["m10"] / M["m00"])
                        cX = 362 - int(M["m01"] / M["m00"])

                        # print(cX/60,cY/40)
                        kx = 5.1 / 219
                        ky = 11.383945 / 480
                        pose.position.x = float(cX)*kx
                        pose.position.y = float(cY)*ky
                        print(cX*kx, cY*ky)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        cv2.circle(cvImg_bis, (cX, cY), 5, (0, 255, 0), -1)
            pose_array.poses.append(pose)

        lower_g = np.array([41, 200, 100])
        upper_g = np.array([86, 255, 255])

        hsv2 = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh2 = cv2.inRange(hsv2, lower_g, upper_g)
        contours, hierarchy = cv2.findContours(tresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("robot position")
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)

            if area < 1000 and area > 10:
                if (peri != 0):
                    # if abs((4 * np.pi * area / (peri) ** 2) - 1) < 0.4:
                    # print("tout est en ordre, circulez")
                    M = cv2.moments(c)

                    if (M["m00"] != 0):
                        # print("Je suis M")

                        cY = 640 - int(M["m10"] / M["m00"])
                        cX = 362 - int(M["m01"] / M["m00"])


                        kx = 5.1/219
                        ky = 11.383945/480
                        pose_rob.position.x = float(cX)*kx
                        pose_rob.position.y = float(cY)*ky

                        print(cX*kx, cY*ky)

                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        cv2.circle(cvImg_bis, (cX, cY), 5, (255,0, 0), -1)




        cv2.imshow('real',cvImg)
        # cv2.imshow('tresh', tresh)
        cv2.imshow('balls',cvImg_bis)
        # cv2.imshow('tresh2',tresh2)
        cv2.waitKey(1)
        self.cmd_posBall_publisher.publish(pose_array)
        self.cmd_posRob_publisher.publish(pose_rob)



def main():

    rclpy.init()
    position = position_node()
    rclpy.spin(position)




if __name__ =="__main__":
    main()