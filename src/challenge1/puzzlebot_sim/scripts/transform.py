#!/usr/bin/env python

import tf
import rospy
import numpy as np
import my_constants as constants
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

import itertools
count = itertools.count()


class EqSolver():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.wl_pub = rospy.Publisher(
            "wl", Float32, queue_size=10)
        self.wr_pub = rospy.Publisher(
            "wr", Float32, queue_size=10)
        rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_listener)
        rate = rospy.Rate(constants.deltat)

        while not rospy.is_shutdown():
            rate.sleep()

    def calc_ws(self, v, w):
        wl = (v/constants.r) - (w*constants.L)/(2*constants.r)
        wr = (v/constants.r) + (w*constants.L)/(2*constants.r)
        self.wl_pub.publish(wl)
        self.wr_pub.publish(wr)

    def cmd_listener(self, cmd_msg):
        v = cmd_msg.linear.x
        w = cmd_msg.angular.z
        self.calc_ws(v, w)

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    EqSolver()
