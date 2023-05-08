#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan  
import numpy as np 
import my_constants as constants
from tf.transformations import quaternion_from_euler

class Robot(): 
    '''
    This class implements the differential drive model of the robot 
    '''
    
    def __init__(self):
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)
        self.pose_pub = rospy.Publisher('puzzlebot_pose', Pose, queue_size=10)  
        self.x     = 0.0 # X position of the robot [m] 
        self.y     = 0.0 # Y position of the robot [m] 
        self.theta = 0.0 # Angle of the robot [rad]
        self.wl = 0.0
        self.wr = 0.0
        rate = rospy.Rate(constants.node_freq) #freq Hz  
        while not rospy.is_shutdown():
            self.update_state(self.wr,
                              self.wl,
                              constants.deltat)
            self.publish_pose()
            rate.sleep()

    def update_state(self, wr, wl, delta_t): 
        '''
        This function returns the robot's state 
        This functions receives the wheel speeds wr and wl in [rad/sec]  
        and returns the robot's state 
        '''
        v=constants.r*(wr+wl)/2 
        w=constants.r*(wr-wl)/constants.L

        self.theta=self.theta + w*delta_t 
        self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta)
        self.x+=vx*delta_t
        self.y+=vy*delta_t

        self.pose_pub()

    def publish_pose(self):
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, self.theta))
        self.pose_pub(msg)

    def wl_cb(self, wl):  
        '''
        This function receives a the left wheel speed [rad/s] 
        '''
        self.wl = wl.data 

    def wr_cb(self, wr):  
        '''
        This function receives a the right wheel speed.  
        '''
        self.wr = wr.data


if __name__ == "__main__":  
    rospy.init_node("robot", anonymous=True)  
    Robot()