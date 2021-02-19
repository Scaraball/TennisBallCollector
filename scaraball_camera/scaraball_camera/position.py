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

import math
from scaraball_camera.munkres import linear_assignment # local import

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#

def pixel2gazebo(pos):
    x = pos[0]
    y = pos[1]

    kx = 5.1 / 219
    ky = 11.383945 / 480

    X = (360 - y) * kx
    Y = (640 - x) * ky

    return X, Y

# = = = = = = = = = = = = = = = = = = = =

class Ball(object):
    staticID = 0
    def __init__(self, pos, nb):
        self.pos = [pos]
        self.lastSeen = 0
        self.number = nb
        Ball.staticID += 1
        self.color = tuple(255 * (0.2 + 4 * np.random.random((3)) / 5))

    def move(self, pos):
        self.pos.append(pos)

    def dist(self, pt, offset=0):
        return math.sqrt((pt[0] - self.pos[-(1 + offset)][0]) ** 2 + (pt[1] - self.pos[-(1 + offset)][1]) ** 2)

    def draw(self, img):
        cv2.circle(img, (self.pos[-1][0] , self.pos[-1][1] ), 5,self.color, -1)
        cv2.putText(img, 'Ball '+str(self.number), (self.pos[-1][0] + 10, self.pos[-1][1]), cv2.FONT_HERSHEY_SIMPLEX ,0.7, (255, 0, 0) , 1, cv2.LINE_AA)

# = = = = = = = = = = = = = = = = = = = =

class Human(object):
    def __init__(self, truc):
        self.pos = [0,0]
        self.entre_jambes = 20
        self.name = truc
        self.list_pos = [[0,0]]
        self.real_pos = [0,0]


    def update_pos(self,x,y,w,h):

        if self.name == "right":

            if (y + h / 2) > 320:  # on est en bas a droite
                pos_x = x
                pos_y = y + self.entre_jambes / 2
            else:  # on est en haut a droite
                pos_x = x
                pos_y = y + h - self.entre_jambes / 2
            self.pos[0] = int(pos_x)
            self.pos[1] = int(pos_y)



        if self.name == "left":

            if (y+h/2) > 320: # on est en bas a gauche
                pos_x = x + w
                pos_y = y + self.entre_jambes/2

            else: # on est en haut a gauche
                pos_x = x + w
                pos_y = y + h - self.entre_jambes/2

            # if abs(self.pos[0]-pos_x) < 20 and  abs(self.pos[1]-pos_y): # valeurs cohérentes
            self.pos[0] = int(pos_x)
            self.pos[1] = int(pos_y)
        if abs(self.pos[0] - self.list_pos[-1][0]) < 20 or len(self.list_pos) == 1:
            self.historique(3)


    def historique(self, n):
        [x,y] = self.pos
        self.list_pos.append([x,y])
        if len(self.list_pos) >= n:
            self.list_pos.pop(0)

        liste_x = [x[0] for x in self.list_pos]
        liste_y = [y[1] for y in self.list_pos]

        posx = int(np.mean(liste_x))
        posy = int(np.mean(liste_y))
        self.real_pos = [posx,posy]


    def draw(self, cvImg):

        cv2.circle(cvImg, (self.real_pos[0],self.real_pos[1]), 5, (0, 0, 255), -1)
        if self.name == "right":
            cv2.putText(cvImg, "Mme Lepen", (self.real_pos[0] + 10, self.real_pos[1]), cv2.FONT_HERSHEY_SIMPLEX ,0.7, (200, 0, 0) , 1, cv2.LINE_AA)
        else:
            cv2.putText(cvImg, "M. Melenchon", (self.real_pos[0] + 10, self.real_pos[1]), cv2.FONT_HERSHEY_SIMPLEX ,0.7, (0, 0, 255) , 1, cv2.LINE_AA)







class position_node(Node):

    def __init__(self):
        super().__init__("position")
        self.get_logger().info('init')
        self.cmd_posBall_publisher = self.create_publisher(PoseArray,"/posBall",10)
        self.cmd_posRob_publisher = self.create_publisher(Pose, "/posRob", 10)
        self.cmd_posHuman_publisher = self.create_publisher(PoseArray, "/posHuman", 10)

        self.lidar_subscriber = self.create_subscription(Image,"/zenith_camera/image_raw",self.img_callback, qos_profile_sensor_data)
        self.list_balls = []
        self.nb = 0
        self.list_human = []
        self.nb_human = 0
        self.robot = [0,0]
        time.sleep(0.5)

        self.Lepen = Human("right")
        self.Melenchon = Human("left")

    def ball_detection(self,cvImg): # Function for the balls detection

        lower_y = np.array([10, 150, 100])
        upper_y = np.array([40, 255, 255])
        hsv = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh = cv2.inRange(hsv, lower_y, upper_y)
        contours, hierarchy = cv2.findContours(tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        keypoints = []
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)

            if area < 1000 and area > 10:
                if (peri != 0):
                    M = cv2.moments(c)

                    if (M["m00"] != 0):


                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        keypoints.append([cX,cY])


        if len(self.list_balls) == 0:  # no bees yet, no matching to do, just add them
            for kp in keypoints:
                self.list_balls.append(Ball(kp,self.nb))
                self.nb += 1
        else:
            freeBees = [True for i in range(len(self.list_balls))]
            freeKP = [True for i in range(len(keypoints))]

            # build cost matrix
            cost = np.zeros((len(keypoints), len(self.list_balls)))
            for i, kp in enumerate(keypoints):
                for j, b in enumerate(self.list_balls):
                    cost[i, j] = b.dist(kp)

            # proper assignment
            assignment = linear_assignment(cost)
            for ass in assignment:
                if cost[ass[0], ass[1]] < 300: # maximum "jump" a bee can make between two consecutive detections in pixels
                    self.list_balls[ass[1]].move(keypoints[ass[0]])
                    freeBees[ass[1]] = False
                    freeKP[ass[0]] = False

            for i in range(len(freeKP)):  # new keypoints
                if freeKP[i]:
                    self.list_balls.append(Ball(keypoints[i],self.nb))
                    self.nb +=1

    def human_detection(self,cvImg): # Function for the balls detection

        # lower_b = np.array([0, 0, 25])
        # upper_b = np.array([30, 50, 100])
        lower_b = np.array([70, 0, 35])
        upper_b = np.array([250, 180, 160])
        hsv = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh = cv2.inRange(hsv, lower_b, upper_b)
        contours, hierarchy = cv2.findContours(tresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


        for c in contours:

            rect = cv2.boundingRect(c)
            if rect[2] < 100 or rect[3] < 100:
                x, y, w, h = rect
                if h > 15 or w > 15:

                    if x > 640: # concerne le joueur a droite
                        self.Lepen.update_pos(x,y,w,h)

                    else:
                        self.Melenchon.update_pos(x,y,w,h)








    def robot_detection(self,cvImg): # Function for the robot position detection

        lower_g = np.array([41, 200, 100])
        upper_g = np.array([86, 255, 255])

        hsv2 = cv2.cvtColor(cvImg, cv2.COLOR_BGR2HSV)
        tresh2 = cv2.inRange(hsv2, lower_g, upper_g)
        contours, hierarchy = cv2.findContours(tresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)

            if area < 1000 and area > 10: # size criteria
                if (peri != 0):
                    M = cv2.moments(c)

                    if (M["m00"] != 0):

                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])

                        self.robot = [cX,cY]




    def img_callback(self,msg):

        br = cv_bridge.CvBridge()
        zenith_cam = br.imgmsg_to_cv2(msg)
        cvImg = cv2.cvtColor(zenith_cam, cv2.COLOR_RGB2BGR)
        cvImg_bis = cvImg.copy()

        # - - - - - -

        self.ball_detection(cvImg)
        self.robot_detection(cvImg)
        self.human_detection(cvImg)

        # - - - - - -

        for b in self.list_balls:
            b.draw(cvImg_bis)

        cv2.circle(cvImg_bis, (self.robot[0] , self.robot[1] ), 6, (0,0,255), -1)
        cv2.putText(cvImg_bis, 'Robot' , (self.robot[0] + 10, self.robot[1] ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0,255), 1, cv2.LINE_AA)

        cv2.imshow('balls', cvImg_bis)

        # - - - - - -

        self.Lepen.draw(cvImg_bis)
        self.Melenchon.draw(cvImg_bis)

        cv2.imshow('balls', cvImg_bis)
        cv2.waitKey(1)

        # - - - - - -

        pose_array = PoseArray()

        for b in self.list_balls:

            X,Y = pixel2gazebo(b.pos[-1])
            pose = Pose()

            pose.position.x = float(X)
            pose.position.y = float(Y)
            pose.position.z = float(b.number)
            # infologger = 'position balle '+ str(int(pose.position.z)) +  ' est : ' + str(pose.position.x) + ', ' + str(pose.position.y)
            # self.get_logger().info(infologger)
            pose_array.poses.append(pose)

        pose_human = PoseArray()
        poseL = Pose()	
        xl,yl = pixel2gazebo(self.Lepen.real_pos)

        poseL.position.x = float(xl)
        poseL.position.y = float(yl)

        poseM = Pose()

        xm, ym = pixel2gazebo(self.Melenchon.real_pos)

        poseM.position.x = float(xm)
        poseM.position.y = float(ym)

        pose_human.poses.append(poseL)
        pose_human.poses.append(poseM)


        pose_rob = Pose()

        X_rob, Y_rob = pixel2gazebo(self.robot)

        pose_rob.position.x = float(X_rob)
        pose_rob.position.y = float(Y_rob)

        self.cmd_posBall_publisher.publish(pose_array)
        self.cmd_posRob_publisher.publish(pose_rob)
        self.cmd_posHuman_publisher.publish(pose_human)



def main():

    rclpy.init()
    position = position_node()
    rclpy.spin(position)



if __name__ =="__main__":
    main()
