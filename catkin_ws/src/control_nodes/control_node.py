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
        self.xd = 0
        self.yd = 1.5
        self.psid = 0
        self.k1 = 1
        self.k2 = 2
        self.theta = np.pi/2

        # to publish at proper frequency
        self.last_publish_vel = geometry_msgs.msg.Twist()
        self.time_out = 0.008

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        #Subscribed messages
        self.name = socket.gethostname()
        if self.name == 'sirab-T15':
            self.follower = 'sirab'
            self.leader = 'sirar'
            self.leader_frame = 'ridgeRframe_from_marker_left'
            self.follower_frame = 'ridgeBframe'
        else:
            self.follower = 'sirar'
            self.leader = 'sirab'
            self.leader_frame = 'ridgeBframe_from_marker_right'
            self.follower_frame = 'ridgeRframe'
        time.sleep(2)
        self.leader_vel = rospy.Subscriber('/'+self.follower+'/ridgeback/vel_feedback',geometry_msgs.msg.Twist,
                                                 self.calcVel,queue_size=1)
        #Published messages
        self.follower_vel = rospy.Publisher('/'+self.follower+'/ridgeback/ridgeback_velocity_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.cur_pos = np.zeros([3,1])
        self.leader_pos = np.zeros([3,1])

        # setup callback to repeat command at desired frequency
        rospy.Timer(rospy.Duration(self.time_out), self.cmd_vel_timeout_callback)

    def calcRelPosition(self):
        self.pos = self.tfBuffer.lookup_transform(self.leader_frame,self.follower_frame,rospy.Time())
        self.x = self.pos.transform.translation.x
        self.y = self.pos.transform.translation.y
        self.psi = self.pos.transform.rotation.z
        #self.theta = self.calcAngle(self.x,self.y,self.theta)
        self.theta = np.arctan2(self.x,self.y)

    def calcAngle(self,x,y,ang_in):
        angle = np.arctan2(x,y)
        if abs(angle-ang_in) > 5:
            if angle > ang_in:
                self.theta = angle-2*np.pi
            else:
                self.theta = angle+2*np.pi

    def calcVel(self,leader_velocity):
        self.calcRelPosition()
        self.leader_vel = np.zeros((3,1))
        #self.leader_vel[0] = np.sqrt(leader_velocity.linear.x**2 + leader_velocity.linear.y**2) #uncomment when testing with sirar
        #self.leader_vel[1] = leader_velocity.angular.z
        #print('relative position: {}, {}'.format(self.l,self.theta))

        error = np.array([self.k1*(self.xd-self.x),self.k1*(self.yd-self.y),self.k2*(self.psid-self.psi)]).reshape(3,1)
        uj = error - self.leader_vel
        R = np.array([[np.cos(self.theta),-np.sin(self.theta), 0],[np.sin(self.theta),np.cos(self.theta),0],[0,0,1]])
        self.sira_vel = np.dot(R, uj)
        self.sira_vel[2] = -1 * self.sira_vel[2]
        self.publishVel()
    
    def publishVel(self):
        publish_vel = geometry_msgs.msg.Twist()
        publish_vel.linear.x = self.sira_vel[0][0] / 10
        publish_vel.linear.y = self.sira_vel[1][0] / 10
        publish_vel.linear.z = 0
        publish_vel.angular.x = 0
        publish_vel.angular.y = 0
        publish_vel.angular.z = self.sira_vel[2][0] / 10
        # self.follower_vel.publish(publish_vel)
        print(publish_vel)

        # store last state
        self.last_publish_vel = publish_vel

    # publish desired velocity on proper frequency
    def cmd_vel_timeout_callback(self):
        print("cmd vel timeout!")
        self.follower_vel.publish(self.last_publish_vel)

        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    