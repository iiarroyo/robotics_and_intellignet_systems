#!/usr/bin/env python

import tf
import tf2_ros 

import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import my_constants as constants
from tf.transformations import quaternion_from_euler

import itertools
count = itertools.count()


class DeadReckoning():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(constants.deltat)
        rospy.Subscriber(
            "wl", Float32, self.wl_listener)
        rospy.Subscriber(
            "wr", Float32, self.wr_listener)
        self.pose_pub = rospy.Publisher(
            "odom", Odometry, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        self.xdot = 0
        self.ydot = 0
        self.thetadot = 0
        self.theta = 0
        self.w = 0
        self.wl = 0
        self.wr = 0
        self.robot_odom = Odometry()
        while not rospy.is_shutdown():
            self.calc_vels()
            self.calc_pose()
            rate.sleep()

    def calc_vels(self):
        """
        Calculate v and w(omega)
        """
        v = constants.r*(self.wr + self.wl)/2
        w = constants.r*(self.wr - self.wl)/constants.L

        self.xdot = v*np.sin(self.theta)
        self.ydot = v*np.sin(self.theta)
        self.thetadot = w

    def calc_pose(self):
        """
        
        """ 
        self.robot_odom.pose.position.x += self.xdot
        self.robot_odom.pose.position.y += self.ydot
        self.robot_odom.pose.position.z = constants.r

        self.theta += self.thetadot
        self.robot_odom.orientation = Quaternion(
            quaternion_from_euler(0, 0, self.theta))
        
    def publish_odom(self):
        self.robot_odom.header.stamp = rospy.Time.now()
        self.robot_odom.header.frame_id = "odom"
        self.robot_odom.child_frame_id = "base_link"
        self.pose_pub.publish(self.robot_odom)

    def wl_listener(self, msg):
        self.wl = msg

    def wr_listener(self, msg):
        self.wr = msg

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('localization')
    DeadReckoning()
