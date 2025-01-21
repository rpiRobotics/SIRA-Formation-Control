# this is the list of transforms
# rosrun tf2_ros static_transform_publisher -.29 -.2  h 0 0 0 sirab_base tag1
# rosrun tf2_ros static_transform_publisher  .29 -.2 -h 0 0 0 anchor1 sirar_base
# where h is height from ground to uwb
# this node calculates the necessary transforms from tag1 to anchor1

#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
import numpy as np
from scipy.optimize import least_squares
import tf2_ros
import geometry_msgs.msg
import tf_conversions

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
    '29EF': 2,
    '48C4': 3,
}


class DistEstimator:
    def __init__(self):
        self.readings = np.zeros(4)
                
    def set_readings(self,readings):
        self.readings = np.array(readings)
        
    def least_squares_loss(self, estimates):
        d_ta = 0.25
        
        theta = estimates[2]
        x = estimates[0]
        y = estimates[1]
        # given estimate for front sensor at (x,y) and angle theta, calculate loss
        # (x,y) is defined with origin from tag1
        back_x = x+d_ta*np.sin(theta)
        back_y = y-d_ta*np.cos(theta)
        readings = [np.sqrt(x**2 + y**2),\
                    np.sqrt(back_x**2+back_y**2),\
                    np.sqrt(x**2 + (y+d_ta)**2),\
                    np.sqrt(back_x**2 + (back_y+d_ta)**2)]
        return self.readings-readings
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

def publish_transform(coord, rotation):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'tag1'
    t.child_frame_id = 'anchor1'
    t.transform.translation.x = coord[0]
    t.transform.translation.y = coord[1]
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, rotation)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

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

class UwbTransform:
    def __init__(self):
        rospy.init_node('uwb_transform', anonymous=False)
        self.uwb_topic_name =  rospy.get_param('~uwb_topic_name')
        rospy.Subscriber(self.uwb_topic_name, String, self.uwb_callback, queue_size=1)
        self.dists_mat = np.zeros([4,4])
        self.dist_buffer_ = np.zeros([10])
        # should be 16 readings
        # list in form of [[r11, r12, r13, r14],[r21, r22, r23, r24],...]
        # tag first, anchor second
        self.readings = np.full([4,4], np.nan)
        self.estimator = DistEstimator()

    def estimate_position(self,readings):
        x0 = np.array([0,0,0])
        self.estimator.set_readings(readings)
        result = least_squares(self.estimator.least_squares_loss,x0)
        angle = result.x[-1]/np.pi*180
        coord =  np.array([result.x[1],result.x[0]*-1])
        #TODO(fan.du): use tf2 for position transforms

        return coord, angle

	
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

        # dist, angle = calc_position(calc_readings)
        # self.dist_buffer_[:-1] = self.dist_buffer_[1:]
        # self.dist_buffer_[-1] = dist

        # TODO(fan.du): implement lookup table of tag anchor combinations to transformation
        # decompose overall transformation to estimate (lsq) + orientation (ta lookup, should just be a rotation)

        coord, angle = self.estimate_position(calc_readings)
        publish_transform(coord,angle)
        print(np.mean(self.dist_buffer_))
        print(angle/np.pi*180)
        
if __name__ == '__main__':
    uwb = UwbTransform()
    rospy.spin()