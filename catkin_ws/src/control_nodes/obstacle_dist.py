#!/usr/bin/env python

import rospy
import numpy as np
import socket
import tf2_ros
import obstacle_detector.msg

class obstacleDist():
    def __init__(self):
        rospy.init_node('obstacle_distance_calculator',anonymous=True)
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

        
        self.obstacles = rospy.Subscriber('/obstacles',obstacle_detector.msg.Obstacles,
                                                 self.calcObsPosition,queue_size=1)
        self.closest_obstacle = rospy.Publisher('/'+self.follower+'/closest_obstacle',obstacle_detector.msg.CircleObstacle,queue_size=10)
        self.closest_wall = rospy.Publisher('/'+self.follower+'/closest_wall',obstacle_detector.msg.SegmentObstacle,queue_size=10)
        

    def calcObsPosition(self,obstacles):
        self.obstacle_dist = []
        self.wall_dist = []
        for circle in obstacles.circles:
            mag = np.sqrt(circle.center.x**2 + circle.center.y**2)
            obs_dist = mag - circle.radius
            if obs_dist > .25:
                self.obstacle_dist.append({'mag':np.abs(obs_dist),'obs':circle,'type':'circle'})
        for segment in obstacles.segments:
            p1,p2 = (segment.first_point.x,segment.first_point.y),(segment.last_point.x,segment.last_point.y)
            x_close = min(p1[0],p2[0])
            x_far = max(p1[0],p2[0])
            y_close = min(p1[1],p2[1])
            y_far = max(p1[1],p2[1])
            a,b,c = self.calcLine(p1,p2)
            wall_dist = c / np.sqrt(a**2 + b**2)
            wall_ang = np.arccos(a) + np.pi/2

            if x_close < wall_dist*np.cos(wall_ang) < x_far and y_close < wall_dist*np.sin(wall_ang) < y_far:
                pass
            else:
                p1_dist = np.sqrt(p1[0]**2 + p1[1]**2)
                p2_dist = np.sqrt(p2[0]**2 + p2[1]**2)
                wall_dist = min(p1_dist,p2_dist)
                if p1_dist > p2_dist:
                    wall_ang = np.arctan2(p2[1],p2[0])
                else:
                    wall_ang = np.arctan2(p1[1],p2[0])
                if abs(wall_dist) > .25:
                    self.wall_dist.append({'mag':np.abs(wall_dist),'obs':segment,'type':'segment'})

        self.sorted_obstacles = sorted(self.obstacle_dist,key=lambda x: x['mag'])
        self.sorted_walls = sorted(self.wall_dist,key=lambda x: x['mag'])

        #print(self.sorted_walls[0]['mag'])
        self.closest_obstacle.publish(self.sorted_obstacles[0]['obs'])
        self.closest_wall.publish(self.sorted_walls[0]['obs'])



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
