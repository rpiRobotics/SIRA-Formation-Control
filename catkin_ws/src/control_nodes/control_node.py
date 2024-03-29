#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist
import std_msgs.msg # for JointState
import numpy as np
import socket
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

        #Subscribed messages
        self.name = socket.gethostname()
        if self.name == 'sirab-T15':
            self.name = 'sira_b'
            self.leader = 'sira_r'
        else:
            self.name = 'sira_r'
            self.leader = 'sira_b'

        self.leader_position = rospy.Subscriber('/base/'+self.leader+'_rel_pos',
                                                geometry_msgs.msg.PoseStamped,
                                                self.leader_pos_update_callback,
                                                queue_size=1)
        self.leader_vel = rospy.Subscriber()
        #Published messages
        self.sira_vel = rospy.Publisher('/'+self.name+'/sira_follower_vel', geometry_msgs.msg.Twist,queue_size=1)


    def leader_pos_update_callback(self,new_leader_pos):
        self.leader_pos[0] = new_leader_pos.pose.position.x
        self.leader_pos[1] = new_leader_pos.pose.position.y
        self.leader_pos[2] = new_leader_pos.pose.position.z

    def calc_angle(self,x,y,ang_in):
        angle = np.atan2(x,y)
        if abs(angle-ang_in) > 5:
            if angle > ang_in:
                self.theta = angle-2*np.pi
            else:
                self.theta = angle+2*np.pi
    def calc_rel_pos(self):
        x = self.leader_pos[0] - self.cur_pos[0]
        y = self.leader_pos[1] - self.cur_pos[1]
        self.l = np.sqrt(x**2 + y**2)
        self.theta = self.calc_angle(x,y,self.theta)
    def calc_vel(self):
        error = np.array([[self.k1*(self.ld-self.l)],[self.k2*(self.thetad-self.theta)]])
        uj = error - self.leader_vel
        self.sira_vel = np.dot(np.array([[np.cos(self.theta), 0],[np.sin(self.theta),0],[0, 1]]), uj)





        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    