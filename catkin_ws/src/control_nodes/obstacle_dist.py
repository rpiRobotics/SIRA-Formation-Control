#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist
import std_msgs.msg # for JointState
import numpy as np
import socket
import tf2_ros
import yaml
import time 
from obstacle_detector.msg import Obstacles
from scipy.spatial.transform import Rotation as scp
from scipy.spatial.transform import Slerp

class obstacleDist():
    def __init__(self):
        self.follower_frame = 'ridgeBframe'
        rospy.init_node('obstacle_distance_calculator',anonymous=True)
        self.obstacles = rospy.Subscriber('/obstacles',Obstacles,
                                                 self.calcObsPosition,queue_size=1)

    def calcObsPosition(self,obstacles):
        self.obstacle_dist = []
        for circle in obstacles.circles:
            mag = np.sqrt(circle.center.x**2 + circle.center.y**2)
            obs_dist = mag - circle.radius
            if obs_dist > .25:
                self.obstacle_dist.append(np.abs(obs_dist))
        for segment in obstacles.segments:
            p1,p2 = (segment.first_point.x,segment.first_point.y),(segment.last_point.x,segment.last_point.y)
            a,b,c = self.calcLine(p1,p2)
            obs_dist = c / np.sqrt(a**2 + b**2)
            if obs_dist > .1:
                self.obstacle_dist.append(np.abs(obs_dist))
        self.obstacle_dist = np.asarray(self.obstacle_dist)


    def calcLine(self,p1,p2):
        if p1[0] == p2[0]:
            theta = np.pi/2
            c = p1[0]
        else:
            delta_x = p2[0] - p1[0]
            delta_y = p2[1] - p1[1]
            theta = np.arctan2(-delta_x,delta_y)
            a = np.cos(theta)
            b = np.sin(theta)
            c = -p1[0]*a - p1[1]*b
            if c <= 0:
                return a,b,c
            else:
                return -a,-b,-c

if __name__=='__main__':
    obstacleDist = obstacleDist()
    rospy.spin()
