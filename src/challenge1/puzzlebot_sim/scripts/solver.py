#!/usr/bin/env python

import tf
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import my_constants as my_constants
from tf.transformations import quaternion_from_euler

import itertools
count = itertools.count()


class EqSolver():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.pose_pub = rospy.Publisher(
            "pose", PoseStamped, queue_size=10)
        rospy.Subscriber(
            "xdot", Float32, self.xdot_listener)
        rospy.Subscriber(
            "ydot", Float32, self.ydot_listener)
        rospy.Subscriber(
            "thetadot", Float32, self.thetadot_listener)
        rate = rospy.Rate(my_constants.deltat)
        self.br = tf.TransformBroadcaster()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xdot = 0
        self.ydot = 0
        self.thetadot = 0

        while not rospy.is_shutdown():
            self.get_pose(self.xdot, self.ydot, self.thetadot)
            rate.sleep()

    def get_pose(self, xdot, ydot, thetadot):
        self.x += my_constants.deltat**-1 * xdot
        self.y += my_constants.deltat**-1 * ydot
        self.theta += my_constants.deltat**-1 * thetadot

        msg = PoseStamped()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation = Quaternion(*quaternion_from_euler(0,0,self.theta))
        msg.header.frame_id = "drobot"
        msg.header.seq = next(count)
        msg.header.stamp = rospy.get_rostime()


        self.pose_pub.publish(msg)


    def cmd_listener(self, cmd_msg):
        self.v = cmd_msg.linear.x
        self.w = cmd_msg.angular.z

    def theta_listener(self, theta_msg):
        self.theta = theta_msg.data
    def xdot_listener(self, msg):
        self.xdot = msg.data
    def ydot_listener(self, msg):
        self.ydot = msg.data
    def thetadot_listener(self, msg):
        self.thetadot = msg.data

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    EqSolver()