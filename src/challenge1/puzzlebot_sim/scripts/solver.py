#!/usr/bin/env python

import tf
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import my_constants as constants
from tf.transformations import quaternion_from_euler

import itertools
count = itertools.count()


class EqSolver():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.pose_pub = rospy.Publisher(
            "pose", PoseStamped, queue_size=10)
        self.thetal_pub = rospy.Publisher(
            "thetal", Float32, queue_size=10)
        self.thetar_pub = rospy.Publisher(
            "thetar", Float32, queue_size=10)
        rospy.Subscriber(
            "xdot", Float32, self.xdot_listener)
        rospy.Subscriber(
            "ydot", Float32, self.ydot_listener)
        rospy.Subscriber(
            "thetadot", Float32, self.thetadot_listener)
        rospy.Subscriber(
            "wl", Float32, self.wl_listener)
        rospy.Subscriber(
            "wr", Float32, self.wr_listener)
        rate = rospy.Rate(constants.deltat)
        self.br = tf.TransformBroadcaster()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.thetal = 0 # left wheel angle
        self.thetar = 0 # right wheel angle
        self.xdot = 0
        self.ydot = 0
        self.thetadot = 0
        self.wl = 0
        self.wr = 0

        while not rospy.is_shutdown():
            rate.sleep()

    def get_poses(self, xdot, ydot, thetadot, wl, wr):
        self.x += constants.deltat**-1 * xdot
        self.y += constants.deltat**-1 * ydot
        self.theta += constants.deltat**-1 * thetadot
        self.thetal += constants.deltat**-1 * wl
        self.thetar += constants.deltat**-1 * wr


        msg = PoseStamped()
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, self.theta))
        msg.header.frame_id = "chassis"
        msg.header.seq = next(count)
        msg.header.stamp = rospy.Time.now()
        self.pose_pub.publish(msg)
        self.thetal_pub.publish(self.thetal)
        self.thetar_pub.publish(self.thetar)

    def cmd_listener(self, cmd_msg):
        self.v = cmd_msg.linear.x
        self.w = cmd_msg.angular.z

    def theta_listener(self, theta_msg):
        self.theta = theta_msg.data

    def xdot_listener(self, msg):
        self.xdot = msg.data
        self.get_poses(self.xdot,
                      self.ydot,
                      self.thetadot,
                      self.wl,
                      self.wr)

    def ydot_listener(self, msg):
        self.ydot = msg.data

    def thetadot_listener(self, msg):
        self.thetadot = msg.data
    
    def wl_listener(self, msg):
        self.wr = msg.data
        
    def wr_listener(self, msg):
        self.wl = msg.data

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    EqSolver()
