#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist
import std_msgs.msg # for JointState
import numpy as np
import socket
import tf2_ros
import yaml
import time 
import obstacle_detector
from scipy.spatial.transform import Rotation as scp
from scipy.spatial.transform import Slerp

class obstacleDist():
    def __init__(self):
        self.obstacles = rospy.Subscriber('/obstacles',obstacle_detector,
                                                 self.calcObsPosition,queue_size=1)
    def calcObsPosition(self):
        print('test')
