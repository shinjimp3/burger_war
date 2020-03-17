#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import abstractCcr
from geometry_msgs.msg import PoseWithCovarianceStamped #msgfor pose

class RandomBot(abstractCcr.AbstractCcr):
    def __init__(self, bot_name):
        super(RandomBot, self).__init__(use_lidar=True ,use_camera=True, use_bumper=False,
        use_opt=False, use_usonic=False,
        camera_preview=False)
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('enemy', String, queue_size=30)
        #rospy.Subscriber("/red_bot/image_raw", Image, self.show)
        self.bridge = CvBridge()
        self.myPosX = 0
        self.myPosY = 0
        self.myRotZ = 0
        
        self.keep_time = 30
        self.enemy = -1 # not detected or lost
        self.last_enemy = -1
        self.miss_time = 0 

    def hough(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img = img[...,::-1]
        red,green,blue=cv2.split(img)
        img[blue>50]=0
        img[green>50]=0

        red,green,blue=cv2.split(img)
        red[red>100]=255
        red[red<=100]=0        
        if red[red!=0].sum() > 255*5:
            enemy = np.mean(np.dstack(np.where(red!=0))[0,:,1]) / (img.shape[1])
            self.last_enemy = enemy        
        elif self.miss_time < self.keep_time and self.last_enemy != 0:
            enemy = self.last_enemy // 0.5

            self.miss_time += 1
        else:            
            enemy = -1
            self.enemy = -1
            self.last_enemy = 0
            self.miss_time = 0
        self.vel_pub.publish(str(enemy))

    def print_msg(self, msg):
        print(msg, "?")

    def strategy(self):
        r = rospy.Rate(10) #

        #rospy.init_node('DetectRed', anonymous=True)
        
        
        
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            enemy = rospy.Subscriber("/image_raw", Image, self.hough)
            rospy.spin()

if __name__ == '__main__':
    rospy.init_node('DetectRed')
    bot = RandomBot("red")
    bot.strategy()
