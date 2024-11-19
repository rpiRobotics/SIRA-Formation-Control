#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
import numpy as np

# antenna delay measured in meters
delays = {
    '29EF': .0560,
    '0418': .1117,
    '2A32': .1159,
    '48D3': .0787,
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

def parse_reading(uwb_string: String):
    # "DIST,4,AN0,2F2F,3.05,2.68,0.00,2.21,AN1,2C9D,-0.04,2.91,0.00,2.39,AN2,2ED0,3.02,0.00,0.00,2.19,AN3,2BA2,0.00,0.00,0.00,2.56,POS,1.59,1.65,1.27,44"
    return uwb_string.strip().split(',')[7::6]

class Sensor:
    def __init__(self):
        rospy.init_node('sensor_test', anonymous=False)
        self.uwb_topic_name =  rospy.get_param('~uwb_topic_name')
        rospy.Subscriber(self.uwb_topic_name, String, self.uwb_callback, queue_size=1)
        self.dists_mat = np.zeros([4,4])
        self.dist_buffer_ = np.zeros([10])
	
    def uwb_callback(self, data):
        data = data.data.strip().split(',')
        print(data)
        delay_t = delays['2A32']
        delay_a = delays['29EF']
        # 2A32 is tag
        dist = float(data[-1]) + delay_t + delay_a
        self.dist_buffer_[:-1] = self.dist_buffer_[1:]
        self.dist_buffer_[-1] = dist
        print(np.mean(self.dist_buffer_))
        # process data here
        # find and store valid reading pairs
        # add delay values
        # call calc_position to get relative distance and angle between robots

if __name__ == '__main__':
    sensor = Sensor()
    rospy.spin()