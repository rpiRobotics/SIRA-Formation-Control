#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist
import std_msgs.msg # for JointState
import numpy as np
import socket
import tf2_ros
import yaml
import time
import message_filters
import obstacle_detector.msg
from scipy.spatial.transform import Rotation as scp
from scipy.spatial.transform import Slerp
from identity import sira_name, sira_follower, sira_leader, sira_leader_frame, sira_follower_frame


class followerPosControl():
    def __init__(self):
        print('initializing...')
        print('my name ' + sira_name + '   follower ' + sira_follower + '   leader ' + sira_leader + '   leader frame ' + sira_leader_frame + '   follower frame ' + sira_follower_frame)
        rospy.init_node('follower_control_node',anonymous=True)
        #set initialization variables
        self.xd = 0
        self.yd = 1.5
        self.psid = 0
        self.k1 = 1
        self.k2 = 2
        self.theta = np.pi/2
        self.avoidance_length = .8
        self.repel_strength = .75


        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.latest_leader_vel = np.zeros((3,1))
        self.leader_vel = np.zeros((3,1))

        #Subscribed messages
        time.sleep(2)
        rospy.Subscriber('/'+sira_leader+'/ridgeback/vel_feedback_rebroadcast',geometry_msgs.msg.Twist,
                                                 self.save_leader_vel,queue_size=1)
        self.circles = rospy.Subscriber('/'+sira_follower+'/closest_obstacle',obstacle_detector.msg.CircleObstacle,
                                                 self.calcObsPosition,queue_size=1)
        self.walls = rospy.Subscriber('/'+sira_follower+'/closest_wall',obstacle_detector.msg.SegmentObstacle,
                                                 self.calcWallPosition,queue_size=1)

        rospy.Timer(rospy.Duration(secs=1.0/100.0), self.calcVel)
        
        #Published messages
        self.follower_vel = rospy.Publisher('/'+sira_follower+'/ridgeback/ridgeback_velocity_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.cur_pos = np.zeros([3,1])
        self.leader_pos = np.zeros([3,1])

    def calcRelPosition(self):
        pos = self.tfBuffer.lookup_transform(sira_leader_frame,sira_follower_frame,rospy.Time())
        self.x = pos.transform.translation.x
        self.y = pos.transform.translation.y
        self.psi = pos.transform.rotation.z
        #self.theta = self.calcAngle(self.x,self.y,self.theta)
        self.theta = np.arctan2(self.x,self.y)

    def calcObsPosition(self,obstacles):
        mag = np.sqrt(obstacles.center.x**2 + obstacles.center.y**2)
        self.circ_dist = np.abs(mag - obstacles.radius)
        self.circ_ang = np.arctan2(obstacles.center.y,obstacles.center.x)


    def calcWallPosition(self,walls):
        p1,p2 = (walls.first_point.x,walls.first_point.y),(walls.last_point.x,walls.last_point.y)
        x_close = min(p1[0],p2[0])
        x_far = max(p1[0],p2[0])
        y_close = min(p1[1],p2[1])
        y_far = max(p1[1],p2[1])
        a,b,c = self.calcLine(p1,p2)
        self.wall_dist = np.abs(c / np.sqrt(a**2 + b**2))
        self.wall_angle = np.arccos(a) + np.pi/2

        if x_close < self.wall_dist*np.cos(self.wall_angle) < x_far and y_close < self.wall_dist*np.sin(self.wall_angle) < y_far:
            pass
        else:
            p1_dist = np.sqrt(p1[0]**2 + p1[1]**2)
            p2_dist = np.sqrt(p2[0]**2 + p2[1]**2)
            self.wall_dist = min(p1_dist,p2_dist)
            if p1_dist > p2_dist:
                self.wall_angle = np.arctan2(p2[1],p2[0])
            else:
                self.wall_angle = np.arctan2(p1[1],p2[0])

        
        
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

    def calcAngle(self,x,y,ang_in):
        angle = np.arctan2(x,y)
        if abs(angle-ang_in) > 5:
            if angle > ang_in:
                self.theta = angle-2*np.pi
            else:
                self.theta = angle+2*np.pi

    def save_leader_vel(self, leader_velocity):
        self.latest_leader_vel = leader_velocity

    def calcVel(self, event=None):
        self.calcRelPosition()
        self.leader_vel = np.zeros((3,1))
        #self.leader_vel[0] = np.sqrt(self.latest_leader_vel.linear.x**2 + self.latest_leader_vel.linear.y**2) #uncomment when testing with sirar
        #self.leader_vel[1] = self.latest_leader_vel.angular.z
        #print('relative position: {}, {}'.format(self.l,self.theta))

        error = np.array([self.k1*(self.xd-self.x),self.k1*(self.yd-self.y),self.k2*(self.psid-self.psi)]).reshape(3,1)
        uj = error - self.leader_vel
        R = np.array([[np.cos(self.theta),-np.sin(self.theta), 0],[np.sin(self.theta),np.cos(self.theta),0],[0,0,1]])
        self.sira_vel = np.dot(R, uj)
        self.sira_vel[2] = -1 * self.sira_vel[2]
        #self.obstacleAvoidance()

        if np.abs(self.sira_vel[0]) > 0.2:
            if self.sira_vel[0] > 0:
                self.sira_vel[0] = 0.2
            else:
                self.sira_vel[0] = -0.2
        if np.abs(self.sira_vel[1]) > 0.2:
            if self.sira_vel[1] > 0:
                self.sira_vel[1] = 0.2
            else:
                self.sira_vel[1] = -0.2

        self.publishVel()

    def obstacleAvoidance(self):
        if self.circ_dist < self.wall_dist:
            self.obs_dist = self.circ_dist
            self.obs_angle = self.circ_ang
        else:
            self.obs_dist = self.wall_dist
            self.obs_angle = self.wall_angle

        #print('obstacle distance{0}'.format(self.obs_dist))
        #print('obstacle angle {0}'.format(self.obs_angle))

        print(self.obs_dist)
        if self.obs_dist < self.avoidance_length:
            repel_force = self.repel_strength * (1/self.obs_dist - 1/self.avoidance_length) * (-1/self.obs_dist)
            #print(repel_force*np.sin(self.obs_angle))
            self.sira_vel[0] = self.sira_vel[0]+repel_force*np.cos(self.obs_angle)
            self.sira_vel[1] = self.sira_vel[1]+repel_force*np.sin(self.obs_angle)
        
        
            
        
    
    def publishVel(self):
        publish_vel = geometry_msgs.msg.Twist()
        publish_vel.linear.x = self.sira_vel[0][0] / 5
        publish_vel.linear.y = self.sira_vel[1][0] / 5
        publish_vel.linear.z = 0
        publish_vel.angular.x = 0
        publish_vel.angular.y = 0
        publish_vel.angular.z = self.sira_vel[2][0] / 5
        # self.follower_vel.publish(publish_vel)
        print(publish_vel)

        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    
