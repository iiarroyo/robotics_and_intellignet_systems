#!/usr/bin/env python  
import sys, os
import rospy  
import numpy as np
from sensor_msgs.msg import LaserScan


class TransformLidar():
    def __init__(self):  
        self.new_lidar_pub= rospy.Publisher("base_scan", LaserScan, queue_size=1) 
        rospy.Subscriber("scan", LaserScan, self.lidar_cb ) 
        rate = rospy.Rate(10)
        self.lidar = LaserScan()
        self.received_lidar = False
        while not rospy.is_shutdown():
            if self.received_lidar:
                new_lidar = self.transform_lidar(self.lidar)
                self.new_lidar_pub.publish(new_lidar)
            self.received_lidar = False
            rate.sleep()

    def transform_lidar(self, original_lidar):
        """
        original_lidar -> LaserScan()
        returns new_lidar with transformed data
        """
        # new_lidar = LaserScan()
        # new_ranges = [np.inf for _ in len(original_lidar.ranges)]
        new_ranges = np.roll(original_lidar.ranges, len(original_lidar.ranges)/2)
        new_lidar = original_lidar
        new_lidar.ranges = new_ranges
        new_lidar.header.frame_id = "base_link"
        return new_lidar

    def lidar_cb(self, msg): 
        self.lidar = msg
        self.received_lidar = True

    

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    # first thing, init a node! 
    rospy.init_node('transform_lidar_data')  
    TransformLidar()  