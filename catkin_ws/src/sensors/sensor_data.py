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

class Sensor:
    def __init__(self):
        rospy.init_node('sensor_test', anonymous=False)
        self.uwb_topic_name = 'uwb_reading'
        rospy.Subscriber(self.uwb_topic_name, String, self.uwb_callback, queue_size=1)
        self.dists_mat = np.zeros([4,4])
        self.dist_buffer_ = np.zeros([10])
	
    def uwb_callback(self, data):
        data = data.data.strip().split(',')
        delay_t = delays['2A32']
        delay_a = delays['29EF']
        # 2A32 is tag
        dist = float(data[-1]) + delay_t + delay_a
        self.dist_buffer_[:-1] = self.dist_buffer_[1:]
        self.dist_buffer_[-1] = dist
        print(np.mean(self.dist_buffer_))
        # process data here
        # data should include publishing module
        # use to find antenna delays
        # if not including publishing module, use indices to figure out which anchor
		
if __name__ == '__main__':
    sensor = Sensor()
    rospy.spin()