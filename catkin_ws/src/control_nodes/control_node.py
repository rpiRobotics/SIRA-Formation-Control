#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist
import std_msgs.msg # for JointState
import numpy as np
import socket
import tf2_ros
import yaml
import time
from scipy.spatial.transform import Rotation as scp
from scipy.spatial.transform import Slerp


class followerPosControl():
    def __init__(self):
        rospy.init_node('follower_control_node',anonymous=True)
        #set initialization variables
        self.cur_pos = np.zeros([3,1])
        self.leader_pos = np.zeros([3,1])
        self.ld = 5
        self.psid = np.pi

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #Subscribed messages
        self.name = socket.gethostname()
        if self.name == 'sirab-T15':
            self.name = 'sira_b'
            self.leader = 'sira_r'
            self.leader_frame = 'ridgeRframe_from_marker_left'
            self.follower_frame = 'ridgeBframe'
        else:
            self.name = 'sira_r'
            self.leader = 'sira_b'
            self.leader_frame = 'ridgeBframe_from_marker_right'
            self.follower_frame = 'ridgeRframe'

        #self.leader_vel = rospy.Subscriber('/' + self.leader + '/ridgeback/vel_feedback',std_msgs.msg.String,
                                                 #self.taskmodecallback,
                                                 #queue_size=1)
        #Published messages
        #self.sira_vel = rospy.Publisher('/'+self.name+'/sira_follower_vel', geometry_msgs.msg.Twist,queue_size=1)
        frames_dict = yaml.safe_load(self.tfBuffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())

        print(self.calcRelPosition())

    def calcRelPosition(self):
        time.sleep(10)
        self.T_human2ee = self.tfBuffer.lookup_transform(self.leader_frame , self.follower_frame,rospy.Time())
        return self.T_human2ee

    def leaderPosUpdateCallback(self,new_leader_pos):
        self.leader_pos[0] = new_leader_pos.pose.position.x
        self.leader_pos[1] = new_leader_pos.pose.position.y
        self.leader_pos[2] = new_leader_pos.pose.position.z

    def calcAngle(self,x,y,ang_in):
        angle = np.atan2(x,y)
        if abs(angle-ang_in) > 5:
            if angle > ang_in:
                self.theta = angle-2*np.pi
            else:
                self.theta = angle+2*np.pi

    def calcVel(self):
        error = np.array([[self.k1*(self.ld-self.l)],[self.k2*(self.thetad-self.theta)]])
        uj = error - self.leader_vel
        self.sira_vel = np.dot(np.array([[np.cos(self.theta), 0],[np.sin(self.theta),0],[0, 1]]), uj)





        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    