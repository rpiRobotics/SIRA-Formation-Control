#!/usr/bin/env python

# This node is for estimating uwb delays using onboard data
# and data visualization

# TODO(fan.du): check deletion is working correctly
#   might be due to numpy version not treating binary as mask, check if argwhere fixes

from uwb_data import *
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

        # 4 tags so need 4 different indices to track
        self.read_indices = np.array([0,0,0,0])

        # temporary hack since using 3 tags only
        # TODO(fan.du): remove this section once using 4 tags
        self.readings[:,-1,:] = np.zeros([self.num_readings,4])
        self.read_indices[-1] = self.num_readings

    def check_full(self):
        return np.sum(self.read_indices) >= 4*self.num_readings

    def uwb_callback(self, data):
            
        # if np.isnan(np.sum(self.readings)):
            # NOTE: this is a temporary workaround for the following issue:
            # sometimes tags only get 3 readings instead of 4 and thus a nan is leftover
            # a proper fix would involve tracking the anchor indices but currently we delete
            # the nan rows
        if np.sum(self.read_indices) < 4*self.num_readings:
            tag, anchors, dists = parse_reading(data.data)
            if self.read_indices[tag_indices[tag]] < self.num_readings:
                for i in range(len(anchors)):
                    self.readings[self.read_indices[tag_indices[tag]],tag_indices[tag],anchor_indices[anchors[i]]] = dists[i]
                self.read_indices[tag_indices[tag]] += 1

    def calc_delays(self):
        # NOTE: see uwb_callback note
        if np.isnan(np.sum(self.readings)):
            print('nans present, eliminating nan rows')
            self.readings = np.delete(self.readings, np.argwhere(np.any(np.isnan(self.readings), axis=(1,2))),axis=0)
        print(self.readings.shape)
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
        # NOTE: matrix is singular, need to use least squares to take into account geometric constraints
        a = np.array([[1,0,1,0],[1,0,0,1],[0,1,1,0],[0,1,0,1]])
        b = np.array([r11,r12,r21,r22])
        b = b - np.array([gt11,gt12,gt21,gt22])
        # delays = np.linalg.solve(a,b)
        # print(delays)
        # new_est = np.array([r11,r12,r21,r22]) + a * delays
        # print(new_est)
        fig, axs = plt.subplots(4,4)
        for i in range(4):
            for j in range(4):
                axs[i,j].hist(self.readings[:,i,j])
        print('std')
        print(np.std(self.readings, axis=0))
        print('means')
        print(np.mean(self.readings, axis=0))
        plt.show()
            

if __name__ == '__main__':
    delays = DelayEstimator()
    while not delays.check_full():
        pass
    delays.calc_delays()