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
        #set initialization variables
        self.cur_pos(np.zeros([7,1]))
        self.leader_pos = np.zeros([7,1])
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
        self.follower_position = rospy.Subscriber('/base/'+self.name+'_rel_pos',
                                            geometry_msgs.msg.PoseStamped,
                                            self.pos_update_callback,
                                            queue_size=1)

        self.leader_position = rospy.Subscriber('/base/'+self.leader+'_rel_pos',
                                                geometry_msgs.msg.PoseStamped,
                                                self.leader_pos_update_callback,
                                                queue_size=1)
        def pos_update_callback(self,new_pos):
            self.cur_pos[0] = new_pos.pose.position.x
            self.cur_pos[1] = new_pos.pose.position.y
            self.cur_pos[2] = new_pos.pose.position.z
            self.cur_pos[3] = new_pos.pose.orientation.x
            self.cur_pos[4] = new_pos.pose.orientation.y
            self.cur_pos[5] = new_pos.pose.orientation.z
            self.cur_pos[6] = new_pos.pose.orientation.w
        def leader_pos_update_callback(self,new_leader_pos):
            self.leader_pos[0] = new_leader_pos.pose.position.x
            self.leader_pos[1] = new_leader_pos.pose.position.y
            self.leader_pos[2] = new_leader_pos.pose.position.z
            self.leader_pos[3] = new_leader_pos.pose.orientation.x
            self.leader_pos[4] = new_leader_pos.pose.orientation.y
            self.leader_pos[5] = new_leader_pos.pose.orientation.z
            self.leader_pos[6] = new_leader_pos.pose.orientation.w
        def calc_angle(self,x,y,ang_in):
            self.angle = np.atan2(x,y)
            if abs(self.angle-ang_in) > 5:
                if self.angle > ang_in:
                    self.angle = self.angle-2*np.pi
                else:
                    self.angle = self.angle+2*np.pi
        def calc_rel_pos(self):
            x = self.leader_pos[0] - self.cur_pos[0]
            y = self.leader_pos[1] - self.cur_pos[1]
            self.l = np.sqrt(x**2 + y**2)
            self.theta = calc_angle(x,y,self.theta)
        def calc_vel(self):
            error = np.array([[self.k1*(self.ld-self.l)],[self.k2*(self.thetad-self.theta)]])
            uj = error - self.leader_vel
            self.zdot = np.array([[np.cos(self.theta), 0],[np.sin(self.theta),0],[0, 1]]) @ uj




        #Auxiliary Functions
print(socket.gethostname())