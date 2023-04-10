#!/usr/bin/env python

import tf
import rospy
from geometry_msgs.msg import PoseStamped


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

    def calc_tf(self, msg):
        """
        TODO:
        - recieve PoseStamped msg
        - broadcast to tf
        """
        p = msg.pose.position
        o = msg.pose.orientation
        br = tf.TransformBroadcaster()
        br.sendTransform(translation=(p.x, p.y, p.z),
                         rotation=(o.x, o.y, o.z, o.w),
                         time=msg.header.stamp,
                         child="chassis",
                         parent="/world")

    def pose_listener(self, msg):
        self.pose = msg

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    CoordinateTransform()