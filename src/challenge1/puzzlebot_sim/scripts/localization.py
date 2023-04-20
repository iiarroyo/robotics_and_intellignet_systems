#!/usr/bin/env python

import rospy
import tf2_ros 
import numpy as np
from std_msgs.msg import String
import my_constants as constants
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from deadreckoning import DeadReckoning

import itertools
count = itertools.count()

np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 
class Localization():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(constants.deltat)
        rospy.Subscriber(
            "wl", Float32, self.wl_listener)
        rospy.Subscriber(
            "wr", Float32, self.wr_listener)
        self.odom_pub = rospy.Publisher(
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
        # self.E = np.
        while rospy.get_time() == 0: 
            rospy.loginfo("No simulated time has been received yet")
        while not rospy.is_shutdown():
            self.calc_vels()
            self.calc_pose()
            self.publish_odom()
            rate.sleep()

    def calc_vels(self):
        """
        Calculate v and w(omega)
        """
        wl = self.wl # prevent changes in interruption
        wr = self.wr
        v = constants.r*(wr + wl)/2
        w = constants.r*(wr - wl)/constants.L

        self.xdot = v*np.sin(self.theta)
        self.ydot = v*np.sin(self.theta)
        self.thetadot = w
        self.robot_odom.twist.twist.linear.x = v
        self.robot_odom.twist.twist.angular.z = w

    def calc_pose(self):
        """
        
        """ 
        self.robot_odom.pose.pose.position.x += self.xdot * constants.deltat**-1
        self.robot_odom.pose.pose.position.y += self.ydot * constants.deltat**-1
        self.robot_odom.pose.pose.position.z = constants.r

        self.theta += self.thetadot * constants.deltat**-1
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        self.robot_odom.pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, self.theta))
        
        
        
        
    def publish_odom(self):
        self.robot_odom.header.stamp = rospy.Time.now()
        self.robot_odom.header.frame_id = "odom"
        self.robot_odom.child_frame_id = "base_link"
        self.robot_odom.header.seq = next(count)
        self.odom_pub.publish(self.robot_odom)

    def wl_listener(self, msg):
        self.wl = msg.data

    def wr_listener(self, msg):
        self.wr = msg.data

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('localization')
    Localization()
