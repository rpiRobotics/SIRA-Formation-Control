# This node is for estimating uwb delays using onboard data

#!/usr/bin/env python

from uwb_data import *
import sys
import matplotlib.pyplot as plt

# set delays to 0 to use raw readings in parse
delays = {
    '29EF': 0,
    '0418': 0,
    '2A32': 0,
    '48D3': 0,
    '43E6': 0,
    '0D40': 0,
    '48C4': 0,
    'C326': 0,
}
class DelayEstimator:
    def __init__(self):
        rospy.init_node('delay_test', anonymous=False)
        self.uwb_topic_name =  rospy.get_param('~uwb_topic_name')
        rospy.Subscriber(self.uwb_topic_name, String, self.uwb_callback, queue_size=1)

        self.num_readings = rospy.get_param('~uwb_delay_readings')
        # should be 16 readings
        # list in form of [[r11, r12, r13, r14],[r21, r22, r23, r24],...]
        # tag first, anchor second
        self.readings = np.full([self.num_readings,4,4], np.nan)
        self.fill_ind = 0

        # temporary hack since not using one tag
        self.readings[:,-1,:] = np.zeros([self.num_readings,4])

    
    # def lsq_loss(self,delays):
        
        
    def uwb_callback(self, data):
        # data = data.data.strip().split(',')
        # 4 tags so need 4 different indices to track
        read_indices = np.array([0,0,0,0])
        if np.isnan(np.sum(self.readings)):
            tag, anchors, dists = parse_reading(data.data)
            if read_indices[tag_indices[tag]] < self.num_readings:
                for i in range(len(anchors)):
                    self.readings[read_indices[tag_indices[tag]],tag_indices[tag],anchor_indices[anchors[i]]] = dists[i]
                read_indices[tag_indices[tag]] += 1
        
        # if self.fill_ind < self.num_readings:
        
        #     tag, anchors, dists = parse_reading(data.data)
        #     for i in range(len(anchors)):
        #         self.readings[self.fill_ind,tag_indices[tag],anchor_indices[anchors[i]]] = dists[i]
        #     self.fill_ind += 1
        else:
            assert(not np.isnan(np.sum(self.readings)))

            avg = np.mean(self.readings,axis=0)
            # estimate delays here
            # for tag and anchors t1, t2, a1, a2
            # we need readings t1a1, t1a2, t2a1, t2a2
            t1 = '0D40'
            t2 = '0418'
            a1 = '43E6'
            a2 = '2A32'
            
            r11 = avg[tag_indices[t1],anchor_indices[a1]]
            r12 = avg[tag_indices[t1],anchor_indices[a2]]
            r21 = avg[tag_indices[t2],anchor_indices[a1]]
            r22 = avg[tag_indices[t2],anchor_indices[a2]]
            
            gt11 = 1.245
            gt12 = 1.25
            gt21 = 1.31
            gt22 = 1.26
            
            # need to solve system r_vec = gt_vec + [1 0 1 0;1 0 0 1;0 1 1 0;0 1 0 1] * delay_vec
            # = r_vec - gt_vec (b) = [1 0 1 0;1 0 0 1;0 1 1 0;0 1 0 1] (a) * delay_vec (x)
            a = np.array([[1,0,1,0],[1,0,0,1],[0,1,1,0],[0,1,0,1]])
            b = np.array([r11,r12,r21,r22])
            b = b - np.array([gt11,gt12,gt21,gt22])
            delays = np.linalg.solve(a,b)
            print(delays)
            new_est = np.array([r11,r12,r21,r22]) + a * delays
            print(new_est)
            fig, axs = plt.subplots(4,4)
            for i in range(4):
                for j in range(4):
                    count, bins = np.histogram(self.readings[:,i,j])
                    axs[i,j].hist(count,bins)
            plt.show()
            sys.exit()

if __name__ == '__main__':
    delays = DelayEstimator()
    rospy.spin()