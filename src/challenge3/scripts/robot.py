#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan  
import numpy as np 
import my_constants as constants

class Robot(): 
    '''
    This class implements the differential drive model of the robot 
    '''
    
    def __init__(self):
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        self.x     = 0.0 # X position of the robot [m] 
        self.y     = 0.0 # Y position of the robot [m] 
        self.theta = 0.0 # Angle of the robot [rad] 
        while not rospy.is_shutdown():
            self.update_state()

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
        self.x=self.x+vx*delta_t
        self.y=self.y+vy*delta_t

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