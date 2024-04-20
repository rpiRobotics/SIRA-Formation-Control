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
        self.avoidance_length = 1.5
        self.repel_strength = 1


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
        self.circles = rospy.Subscriber('/'+self.follower+'/closest_obstacle',obstacle_detector.msg.CircleObstacle,
                                                 self.calcObsPosition,queue_size=1)
        self.walls = rospy.Subscriber('/'+self.follower+'/closest_wall',obstacle_detector.msg.SegmentObstacle,
                                                 self.calcWallPosition,queue_size=1)
        
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

    def calcObsPosition(self,obstacles):
        mag = np.sqrt(obstacles.center.x**2 + obstacles.center.y**2)
        self.circ_dist = np.abs(mag - obstacles.radius)
        self.circ_ang = np.arctan2(obstacles.center.x,obstacles.center.y)


    def calcWallPosition(self,walls):
        p1,p2 = (walls.first_point.x,walls.first_point.y),(walls.last_point.x,walls.last_point.y)
        a,b,c = self.calcLine(p1,p2)
        self.wall_dist = np.abs(c / np.sqrt(a**2 + b**2))
        self.wall_ang = np.arccos(a)
        
        
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
        if self.circ_dist < self.wall_dist:
            self.obs_dist = self.circ_dist
            self.obs_angle = self.circ_ang
        else:
            self.obs_dist = self.wall_dist
            self.obs_angle = self.wall_ang
        if self.obs_dist < self.avoidance_length:
            repel_force = self.repel_strength * (1/self.obs_dist - 1/self.avoidance_length) * (-1/self.obs_dist)
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
    