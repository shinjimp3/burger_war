#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from abc import ABCMeta, abstractmethod
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped #msgfor pose
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import actionlib_msgs

from cv_bridge import CvBridge, CvBridgeError
import cv2

#define interfaces
class AbstractCcr(object):
    __metaclass__ = ABCMeta
    def __init__(self, 
         use_lidar=False ,use_camera=True, use_bumper=False,
         use_opt=False, use_usonic=False,
         camera_preview=False):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # position and orientation subscriber
        self.amcl_pose = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.AmclPoseCallback)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # lidar scan subscriber
        if use_lidar:
            self.scan = LaserScan()
            self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # optical sensor scan subscriber
        if use_opt:
            self.opt = [LaserScan(), LaserScan()]
            self.opt_left_sub = rospy.Subscriber('opt_left', LaserScan, self.optLeftCallback)
            self.opt_right_sub = rospy.Subscriber('opt_right', LaserScan, self.optRightCallback)

        # ultrasonic sensor scan subscriber
        if use_usonic:
            self.usonic = [LaserScan(), LaserScan()]
            self.usonic_left_sub = rospy.Subscriber('us_left', LaserScan, self.usonicLeftCallback)
            self.usonic_right_sub = rospy.Subscriber('us_right', LaserScan, self.usonicRightCallback)

        # camera subscribver
        # please uncoment out if you use camera
        #        if use_camera:

       
        # for convert image topic to opencv obj
        self.img = None
        self.camera_preview = camera_preview
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.imageCallback)

        self.enemy = -1
        self.enemy_pos = rospy.Subscriber('red_bot/enemy_pos', String, self.detectCallback)

    def detectCallback(self, data):
        self.enemy = int(data)
        
        
    def AmclPoseCallback(self, data): #update estimated position

       self.myPosX = data.pose.pose.position.x
       self.myPosY = data.pose.pose.position.y

       q = data.pose.pose.orientation
       e = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
       self.myRotZ = e[2] #z axis
       print('[' + str(self.myPosX) + ',' + str(self.myPosY) + ',' + str(self.myRotZ) + ']') 

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data

    # optical  scan topic call back sample
    # update lidar scan state
    def optLeftCallback(self, data):
        self.opt[0] = data

    def optRightCallback(self, data):
        self.opt[1] = data

    # usonic  scan topic call back sample
    # update lidar scan state
    def usonicLeftCallback(self, data):
        self.usonic[0] = data

    def usonicRightCallback(self, data):
        self.usonic[1] = data

    # bumper topic call back sample
    # update bumper state
    def bumperCallback(self, data):
        self.left_bumper = data.left.state
        self.right_bumper = data.right.state

    # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data): 
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.camera_preview:
          cv2.imshow("Image window", self.img)
          cv2.waitKey(1)


    @abstractmethod
    def strategy(self):
        pass

