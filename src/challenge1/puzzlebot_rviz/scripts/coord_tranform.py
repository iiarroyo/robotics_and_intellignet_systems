#!/usr/bin/env python

import tf
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class CoordinateTransform():
    def __init__(self):
        rospy.Subscriber(
            "pose", PoseStamped, self.pose_listener)
        rate = rospy.Rate(50)
        self.pose = PoseStamped()
        """
        1 pose
            - x,y,x
            - quaternion
        2 header
         - stamp(tiempo de ros)
         - frame_id: "base_link"
        """
        while not rospy.is_shutdown():
            self.calc_tf(self.pose)
            rate.sleep()

    def calc_tf(self, pose):
        """
        TODO:
        - recieve PoseStamped msg
        - broadcast to tf
        """
        pass

    def pose_listener(self, msg):
        self.pose = msg

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    CoordinateTransform()