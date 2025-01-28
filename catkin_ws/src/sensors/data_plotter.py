#!/usr/bin/env python

# this node compiles uwb_data published transforms and plots them

import rospy
import time
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from seaborn import jointplot
import matplotlib.pyplot as plt

class UwbPlotter:
    def __init__(self):
        rospy.init_node('uwb_plot', anonymous=False)
        self.transform_topic_name =  rospy.get_param('~transform_topic_name')
        self.num_readings = rospy.get_param('~uwb_delay_readings')
        rospy.Subscriber(self.transform_topic_name, geometry_msgs.msg.TransformStamped, self.uwb_callback, queue_size=1)
        # [x,y,dist,theta]
        self.data = np.full([self.num_readings,4],np.nan)
        self.ind = 0

    def uwb_callback(self,est):
        if self.ind<self.num_readings:
            tf = est.transform
            self.data[self.ind,0] = tf.translation.x
            self.data[self.ind,1] = tf.translation.y
            self.data[self.ind,2] = np.linalg.norm(self.data[self.ind,0:2])
            self.data[self.ind,3] = tf_conversions.transformations.euler_from_quaternion(tf.rotation)

    def check_full(self):
        return self.ind>=self.num_readings
    
    def plot(self):
        jointplot(x=self.data[:,0],y=self.data[:,1])
        print('stds')
        print(np.std(self.data,axis=0))
        print('means')
        print(np.mean(self.data,axis=0))
        plt.hist(self.data[:,-1])
        plt.show()
