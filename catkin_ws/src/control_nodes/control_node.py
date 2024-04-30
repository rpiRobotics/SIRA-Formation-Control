#!/usr/bin/env python

import rospy
import geometry_msgs.msg # for Twist and Wrench
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

        ### position related variables ###
        
        # desired values
        self.xd = 0
        self.yd = 1.5
        self.psid = 0
        
        # scale factors
        self.k_position_xy = 1
        self.k_position_angle = 2
        self.theta = np.pi/2

        ### obstacle related variables ###
        self.avoidance_length = .8
        self.repel_strength = .75
        self.circ_dist = self.avoidance_length
        self.circ_angle = 0
        self.wall_dist = self.avoidance_length
        self.wall_angle = 0


        ### force related variables ###
        self.target_force = 20

        ### compliant controller ###
        
        # set scale factors
        self.k_position = 1
        self.k_obstacle = 0.75
        self.k_force = 1
        
        # initialize compliant controller terms to 0
        self.position_term = np.zeros((3, 1))
        self.obstacle_term = np.zeros((3, 1))
        self.force_term = np.zeros((3, 1))

        # create tf butter to lookup transformations
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.leader_vel = np.zeros((3,1))

        #Subscribed messages
        time.sleep(2)
        rospy.Subscriber('/'+sira_leader+'/ridgeback/vel_feedback_rebroadcast',geometry_msgs.msg.Twist,
                         self.saveLeaderVel,queue_size=1)
        rospy.Subscriber('/'+sira_follower+'/closest_obstacle',obstacle_detector.msg.CircleObstacle,
                         self.calcObsPosition,queue_size=1)
        rospy.Subscriber('/'+sira_follower+'/closest_wall',obstacle_detector.msg.SegmentObstacle,
                         self.calcWallPosition,queue_size=1)
        rospy.Subscriber('/'+sira_follower+'/netft/transformed_world_forces', geometry_msgs.msg.WrenchStamped,
                         self.saveForce, queue_size=1)

        rospy.Timer(rospy.Duration(secs=1.0/100.0), self.calcVel)
        rospy.Timer(rospy.Duration(secs=1.0/100.0), self.calcPositionComponent)
        
        #Published messages
        self.follower_vel = rospy.Publisher('/'+sira_follower+'/ridgeback/ridgeback_velocity_controller/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        self.cur_pos = np.zeros([3,1])
        self.leader_pos = np.zeros([3,1])

    def calcRelPosition(self):
        pos = self.tfBuffer.lookup_transform(sira_leader_frame,sira_follower_frame,rospy.Time())
        self.x = pos.transform.translation.x
        self.y = pos.transform.translation.y
        self.psi = pos.transform.rotation.z
        #self.theta = followerPosControl.calcAngle(self.x,self.y,self.theta)
        self.theta = np.arctan2(self.x,self.y)

    def calcObsPosition(self,obstacles):
        mag = np.sqrt(obstacles.center.x**2 + obstacles.center.y**2)
        self.circ_dist = np.abs(mag - obstacles.radius)
        self.circ_ang = np.arctan2(obstacles.center.y,obstacles.center.x)

        # update obstacle component with new data
        self.calcObstacleComponent()

    def calcWallPosition(self,walls):
        p1,p2 = (walls.first_point.x,walls.first_point.y),(walls.last_point.x,walls.last_point.y)
        x_close = min(p1[0],p2[0])
        x_far = max(p1[0],p2[0])
        y_close = min(p1[1],p2[1])
        y_far = max(p1[1],p2[1])
        a,b,c = followerPosControl.calcLine(p1,p2)
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
        # update obstacle component with new data
        self.calcObstacleComponent()

    def calcLine(p1,p2):
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

    def calcAngle(x,y,ang_in):
        angle = np.arctan2(x,y)
        if abs(angle-ang_in) > 5:
            if angle > ang_in:
                self.theta = angle-2*np.pi
            else:
                self.theta = angle+2*np.pi

    def saveLeaderVel(self, leader_velocity):
        self.leader_vel = np.zeros((3,1))
        #self.leader_vel[0] = np.sqrt(leader_velocity.linear.x**2 + leader_velocity.linear.y**2) #uncomment when testing with sirar
        #self.leader_vel[1] = leader_velocity.angular.z

    def calcPositionComponent(self, event=None):
        '''
        Read the latest relative position and update the position term in the compliant controller
        '''
        # get latest relative position to leader
        self.calcRelPosition()

        # calculate position-based control
        # this includes leader velocity
        error = np.array([self.k_position_xy*(self.xd-self.x),self.k_position_xy*(self.yd-self.y),self.k_position_angle*(self.psid-self.psi)]).reshape(3,1)
        # uj = error + self.leader_vel
        uj = error # 
        
        # rotate the forces from leader to follower frame
        R = np.array([[np.cos(self.theta),-np.sin(self.theta), 0],
                      [np.sin(self.theta),np.cos(self.theta),0],
                      [0,0,1]])
        
        self.position_term = np.dot(R, uj)


    def calcObstacleComponent(self, event=None):
        '''
        Read the latest obstacles and update the obstacle term in the compliant controller
        '''
        # get latest relative position to obstacles
        self.obstacleAvoidance()

        # avoid obstacles
        if self.obs_dist < self.avoidance_length:
            repel_force_magnitude = self.repel_strength * (1/self.obs_dist - 1/self.avoidance_length) * (-1/self.obs_dist)
            #print(repel_force*np.sin(self.obs_angle))
            self.obstacle_term[0] = repel_force_magnitude * np.cos(self.obs_angle)
            self.obstacle_term[1] = repel_force_magnitude * *np.sin(self.obs_angle)
        else:
            self.obstacle_term = np.zeros((3, 1))

    def calcForceComponent(self, wrenchStamped):
        '''
        Read the current force and update the force term in the compliant controller
        '''
        current_force = wrenchStamped.wrench.force
        force_magnitude = (current_force.x ** 2 + current_force.y ** 2) ** 0.5

        # check if the force exceeds the target force
        # if it does, we need to consider this force in our controller
        self.force_term = np.zeros((3, 1))
        if force_magnitude > self.target_force:
            self.force_term[0] = (force_magnitude - self.target_force) * current_force.x / force_magnitude
            self.force_term[1] = (force_magnitude - self.target_force) * current_force.y / force_magnitude
        

    def calcVel(self, event=None):
        '''
        This function calculates target velocity based on relative
        position to the leader and to the closest obstacle
        '''

        self.sira_vel = self.k_position * self.position_term + self.k_obstacle * self.obstacle_term + self.k_force * self.force_term)

        # saturate target velocity
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

        # print(self.obs_dist)
    
    def publishVel(self):
        publish_vel = geometry_msgs.msg.Twist()
        publish_vel.linear.x = self.sira_vel[0][0] / 5
        publish_vel.linear.y = self.sira_vel[1][0] / 5
        publish_vel.linear.z = 0
        publish_vel.angular.x = 0
        publish_vel.angular.y = 0
        publish_vel.angular.z = self.sira_vel[2][0] / 5
        # self.follower_vel.publish(publish_vel)
        # print(publish_vel)

        #Auxiliary Functions
if __name__=='__main__':
    followerPosControl = followerPosControl()
    rospy.spin()
    
