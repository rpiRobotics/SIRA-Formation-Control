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
        self.yd = 1
        self.psid = 0
        self.k1 = 1
        self.k2 = 2
        self.theta = np.pi/2
        self.avoidance_length = 1.5
        self.repel_strength = 100


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
        self.obs_frame = 'ridgeRframe_from_marker_left' #obstacle frame
        time.sleep(2)
        self.leader_vel = rospy.Subscriber('/'+self.follower+'/ridgeback/vel_feedback',geometry_msgs.msg.Twist,
                                                 self.calcVel,queue_size=1)
        #Published messages
        self.follower_vel = rospy.Publisher('/'+self.follower+'/ridgeback/ridgeback_velocity_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.cur_pos = np.zeros([3,1])
        self.leader_pos = np.zeros([3,1])

    def calcRelPosition(self):
        pos = self.tfBuffer.lookup_transform(self.leader_frame,self.follower_frame,rospy.Time())
        self.x = pos.transform.translation.x
        self.y = pos.transform.translation.y
        self.psi = pos.transform.rotation.z
        #self.theta = self.calcAngle(self.x,self.y,self.theta)
        self.theta = np.arctan2(self.x,self.y)

    def calcObsPosition(self):
        obstacle = self.tfBuffer.lookup_transform(self.obs_frame,self.follower_frame,rospy.Time())
        obsx = obstacle.transform.translation.x
        obsy = obstacle.transform.translation.y
        self.obs_angle = np.arctan2(obsx,obsy)
        self.obs_dist = np.sqrt(obsx**2+obsy**2)
        print('distance: {}'.format(self.obs_dist))
        print('angle: {}'.format(self.obs_angle))

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
        self.obstacleAvoidance()
        self.publishVel()

    def obstacleAvoidance(self):
        self.calcObsPosition()
        if self.obs_dist < self.avoidance_length:
            repel_force = 0.5 * self.repel_strength * (1/self.obs_dist - 1/self.avoidance_length)**2
            print(repel_force)
            if self.sira_vel[0] < 0:
                self.sira_vel[0] = self.sira_vel[0]+repel_force*np.sin(self.obs_angle)
            else:
                self.sira_vel[0] = self.sira_vel[0]-repel_force*np.sin(self.obs_angle)
            if self.sira_vel[1] < 0:
                self.sira_vel[1] = self.sira_vel[1]+repel_force*np.cos(self.obs_angle)
            else:
                self.sira_vel[1] = self.sira_vel[1]-repel_force*np.cos(self.obs_angle)
            
        
    
    def publishVel(self):
        publish_vel = geometry_msgs.msg.Twist()
        publish_vel.linear.x = self.sira_vel[0][0] / 5
        publish_vel.linear.y = self.sira_vel[1][0] / 5
        publish_vel.linear.z = 0
        publish_vel.angular.x = 0
        publish_vel.angular.y = 0
        publish_vel.angular.z = self.sira_vel[2][0] / 5
        #self.follower_vel.publish(publish_vel)
        print(publish_vel)

        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    