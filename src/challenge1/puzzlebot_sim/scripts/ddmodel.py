#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import my_constants as my_constants
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

"""
Variables
v: velocity in x
w(omega): angular velocity in z

theta: current heading
x, y: position in x and y, respectively
"""
class DifferentialDriveModel():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.xdot_pub = rospy.Publisher(
            "xdot", Float32, queue_size=10)
        self.ydot_pub = rospy.Publisher(
            "ydot", Float32, queue_size=10)
        self.thetadot_pub = rospy.Publisher(
            "thetadot", Float32, queue_size=10)
        rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_listener)
        rospy.Subscriber(
            "theta", PoseStamped, self.theta_listener)
        rate = rospy.Rate(50)
        self.v = 0
        self.w = 0
        self.theta = 0

        while not rospy.is_shutdown():
            self.calc(self.v, self.w)
            rate.sleep()

    def calc(self, v, w):
        self.xdot = self.v * np.cos(self.theta)
        self.ydot = self.v * np.sin(self.theta)
        self.thetadot = self.w

        self.xdot_pub.publish(self.xdot)
        self.ydot_pub.publish(self.ydot)
        self.thetadot_pub.publish(self.thetadot)

    def cmd_listener(self, cmd_msg):
        self.v = cmd_msg.linear.x
        self.w = cmd_msg.angular.z
    def theta_listener(self, msg):
        _, _, self.theta = euler_from_quaternion(msg.pose.orientation)

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    DifferentialDriveModel()