#!/usr/bin/env python

import rospy
import tf2_ros 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import my_constants as constants

class CoordinateTransform():
    def __init__(self):
        rospy.Subscriber(
            "pose", PoseStamped, self.robot_pose_listener)
        rospy.Subscriber(
            "thetal", Float32, self.left_wheel_angle_listener)
        rospy.Subscriber(
            "thetar", Float32, self.right_wheel_angle_listener)
        rate = rospy.Rate(constants.node_freq)
        self.robot_pose = PoseStamped()
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.br = tf2_ros.TransformBroadcaster()

        """
        1 pose
            - x,y,x
            - quaternion
        2 header
         - stamp(tiempo de ros)
         - frame_id: "base_link"
        """
        while not rospy.is_shutdown():
            self.calc_base_link_tf(self.robot_pose)
            self.calc_chassis_tf(self.robot_pose)
            self.calc_left_wheel_tf(self.left_wheel_angle)
            self.calc_right_wheel_tf(self.right_wheel_angle)
            rate.sleep()

    def calc_base_link_tf(self, msg=PoseStamped()): 
        # This receives the robot's pose and broadcast a transformation. 
         
        t = TransformStamped() 
 
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "odom" 
        t.child_frame_id = "base_link" 
        #Copy data from the received pose to the tf  
        t.transform.translation.x = msg.pose.position.x 
        t.transform.translation.y = msg.pose.position.y 
        t.transform.translation.z = msg.pose.position.z 
         
        t.transform.rotation.x = msg.pose.orientation.x 
        t.transform.rotation.y = msg.pose.orientation.y 
        t.transform.rotation.z = msg.pose.orientation.z 
        t.transform.rotation.w = msg.pose.orientation.w 
        # Send the transformation 
        self.br.sendTransform(t)

    def calc_chassis_tf(self, msg):
        """
        TODO:
        - recieve PoseStamped msg
        - broadcast to tf
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "base_link" 
        t.child_frame_id = "chassis" 
        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.0 
        t.transform.translation.y = 0.0 
        t.transform.translation.z = 0.0 
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0 
        t.transform.rotation.w = 1
        self.br.sendTransform(t)
        
    def calc_left_wheel_tf(self, angle):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "base_link" 
        t.child_frame_id = "left_wheel" 
        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.0 
        t.transform.translation.y = 0.09
        t.transform.translation.z = 0.0 
        quat = quaternion_from_euler(0.0, -angle, 0.0) 
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 
        self.br.sendTransform(t)
        
    def calc_right_wheel_tf(self, angle):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now() 
        t.header.frame_id = "base_link" 
        t.child_frame_id = "right_wheel" 
        #Copy data from the received pose to the tf  
        t.transform.translation.x = 0.0 
        t.transform.translation.y = -0.09
        t.transform.translation.z = 0.0
        rospy.loginfo(angle)
        quat = quaternion_from_euler(0.0, -angle, 0.0) 
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 
        self.br.sendTransform(t)

    def robot_pose_listener(self, msg):
        self.robot_pose = msg
    def left_wheel_angle_listener(self, msg):
        self.left_wheel_angle = msg.data
    def right_wheel_angle_listener(self, msg):
        self.right_wheel_angle = msg.data

    def cleanup(self):
        print("Stopping {}".format(rospy.get_name()))
        # exit()


if __name__ == "__main__":
    rospy.init_node('ddmodel')
    CoordinateTransform()