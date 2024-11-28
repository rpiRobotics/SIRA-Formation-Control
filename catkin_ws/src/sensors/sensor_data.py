#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
import numpy as np
from scipy.optimize import least_squares

# antenna delay measured in meters
delays = {
    '29EF': .0584,
    '0418': .0515,
    '2A32': .0490,
    '48D3': .0129,
    '43E6': .0549,
    '0D40': .0929,
    '48C4': .0645,
    'C326': .0865,
}

# anchor to index within readings matrix
anchor_indices = {
    '43E6': 0,
    '2A32': 1,
    '48D3': 2,
    'C326': 3,
}
# tag to index within readings matrix
tag_indices = {
    '0D40': 0,
    '0418': 1,
    '29Ef': 2,
    '48C4': 3,
}

#

# for anchor tag setup:
# t1    a1
# t2    a2
# takes in 4 readings calculates distance and angle between robots
def calc_position(readings):
    r11, r12, r21, r22 = readings
    # need to return distance between centerpoint of arm base
    # and angle betwee  n two robots
    
    # define positive angular direction as turning towards each other
    pos = r11 < r22
    # distance between anchor and tag = 25cm
    d_ta = 0.25
    theta = 0   

    # b
    # |  
    # |29cm
    # |
    # ----- t
    #  20cm
    # 
    # r = 35cm

    if pos:
        theta_a1t1a2 = np.arccos((r11**2 + r12**2 - d_ta**2) / (2*r11*r12))
        theta_a2t1t2 = np.arccos((r12**2 + d_ta**2 - r22**2) / (2*r12*d_ta))
        theta_t1a1a2 = np.arccos((r11**2 + d_ta**2 - r12**2) / (2*r11*d_ta))

        theta = theta_a1t1a2 + theta_a2t1t2 + theta_t1a1a2 - np.pi
    else:
        theta_a1t2a2 = np.arccos((r22**2 + r21**2 - d_ta**2) / (2*r22*r21))
        theta_a1t2t1 = np.arccos((r21**2 + d_ta**2 - r11**2) / (2*r21*d_ta))
        theta_t2a2a1 = np.arccos((r22**2 + d_ta**2 - r21**2) / (2*r22*d_ta))

        theta = theta_a1t2a2 + theta_a1t2t1 + theta_t2a2a1 - np.pi
        theta*=-1

    # TODO(fan.du): find adjustment to get distance between arm base
    # current calculation assumes a1 and t1 are both front 
    # treats distances as vectors to find final distance
    theta_base = np.arctan(29/20)+np.pi/2
    
    theta_t1a1a2 = np.arccos((r11**2 + d_ta**2 - r12**2) / (2*r11*d_ta))
    theta_a1t1t2 = np.arccos((r11**2 + d_ta**2 - r21**2) / (2*r11*d_ta))
    theta_gap = np.pi - theta_base - theta_a1t1t2
    theta_arm = theta_gap + np.pi*3/2-theta_t1a1a2
    y = -0.29 + np.sin(np.pi*3/2-theta_t1a1a2)*r11 + np.sin(theta_arm)*0.35
    x = -0.20 + np.cos(np.pi*3/2-theta_t1a1a2)*r11 + np.cos(theta_arm)*0.35

    dist = np.sqrt(y**2+x**2)
    
    # TODO(fan.du): find angle adjustments when using different uwb tag anchor combinations


    return dist, theta

def parse_reading(uwb_string):
    # "DIST,4,AN0,2F2F,3.05,2.68,0.00,2.21,AN1,2C9D,-0.04,2.91,0.00,2.39,AN2,2ED0,3.02,0.00,0.00,2.19,AN3,2BA2,0.00,0.00,0.00,2.56,POS,1.59,1.65,1.27,44"
    # 'DIST', '3', 'AN0', '2A32', '0.00', '0.00', '0.00', '1.69', 'AN1', '38D3', '0.74' ...
    split_reading = uwb_string.strip().split(',')

    # TODO(fan.du): This is a temporary fix as 4 anchors messes up indexing with extra fields
    anchors = split_reading[3:-5:6]
    dists = split_reading[7:-1:6]
    assert(len(anchors)==len(dists))

    tag = split_reading[-1][2:]
    for i in range(len(dists)):
        dists[i] = float(dists[i])
        dists[i] += delays[tag]
        dists[i] += delays[anchors[i]]
    return tag, anchors, dists

class Sensor:
    def __init__(self):
        rospy.init_node('sensor_test', anonymous=False)
        self.uwb_topic_name =  rospy.get_param('~uwb_topic_name')
        rospy.Subscriber(self.uwb_topic_name, String, self.uwb_callback, queue_size=1)
        self.dists_mat = np.zeros([4,4])
        self.dist_buffer_ = np.zeros([10])
        # should be 16 readings
        # list in form of [[r11, r12, r13, r14],[r21, r22, r23, r24],...]
        # tag first, anchor second
        self.readings = np.full([4,4], np.nan)

	
    def uwb_callback(self, data):
        # data = data.data.strip().split(',')
        tag, anchors, dists = parse_reading(data.data)
        for i in range(len(anchors)):
            self.readings[tag_indices[tag],anchor_indices[anchors[i]]] = dists[i]
        tag_mins = np.min(self.readings,axis=1)
        tag_ind = np.argpartition(tag_mins,2)[:2]
        calc_readings = [0,0,0,0]
        for i in tag_ind:
            an_ind = np.sort(np.argpartition(self.readings[i,:],2)[:2])
            calc_readings[2*i] = self.readings[i,an_ind[0]]
            calc_readings[2*i+1] = self.readings[i,an_ind[1]]

        dist, angle = calc_position(calc_readings)
        self.dist_buffer_[:-1] = self.dist_buffer_[1:]
        self.dist_buffer_[-1] = dist
        print(np.mean(self.dist_buffer_))
        print(angle/np.pi*180)
        
if __name__ == '__main__':
    sensor = Sensor()
    rospy.spin()