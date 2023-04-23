#!/usr/bin/env python  
from tf.transformations import quaternion_from_euler 
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
import my_constants as constants
import numpy as np 
import rospy  

np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 

class LocalizationClass():  
    def __init__(self):  
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
        rospy.Subscriber("wl", Float32, self.wl_cb ) 
        rospy.Subscriber("wr", Float32, self.wr_cb ) 
 
        #Robot constants  
        constants.r = 0.05 #[m] radius of the wheels 
        constants.L = 0.18 #[m] distance between wheels 
         
        #Robot state 
        odom = Odometry() 
        self.wl = 0.0 #Robot left wheel angular speed 
        self.wr = 0.0 #Robot right wheel angular speed
        x, y, theta = 0.0, 0.0, 0.0 # Position and orientation
        v, w        = 0.0, 0.0 #[m/s] Robot linear and angular speed 
        dt          = 0.0 #time interval  

        while rospy.get_time() == 0: 
            print("No simulated time has been received yet") 
        print("Got time") 
        last_time = rospy.get_time() 
        self.received_wl = 0 
        self.received_wr = 0 
        rate = rospy.Rate(constants.node_freq) # The rate of the while loop 
        while not rospy.is_shutdown(): 
            if self.received_wl and self.received_wr: 
                #Define the sampling time  
                current_time = rospy.get_time() 
                dt = current_time-last_time 
                #Get a copy of wr an wl to get unexpected changes 
                wr = self.wr 
                wl = self.wl 
                #Compute the robot linear and angular speeds.  
                v = constants.r*(wr+wl)/2.0         # Robot's linear speed 
                w = constants.r*(wr-wl)/constants.L # Robot's Angular Speed 
 
                # Calculate Covariance Matrix Sigma 
                Sigma_pose = np.zeros([3,3]) #Creates the Covariance matrix (3x3) for x, y and theta 
                Sigma_pose[0,0] = 1.0  #cov(xx) 
                Sigma_pose[0,1] = 0.0  #cov(xy) 
                Sigma_pose[0,2] = 0.0  #cov(x, theta) 
                Sigma_pose[1,0] = 0.0  #cov(y,x) 
                Sigma_pose[1,1] = 5.0  #cov(y,y) 
                Sigma_pose[1,2] = 0.0  #cov(y,theta) 
                Sigma_pose[2,0] = 0.0  #cov(theta,x) 
                Sigma_pose[2,1] = 0.0  #cov(theta,y) 
                Sigma_pose[2,2] = 1.0  #cov(thetat,theta) 
 
                #Pose estimation (x,y,theta) 
                x=x+v*np.cos(theta)*dt 
                y=y+v*np.sin(theta)*dt 
                theta = theta + w*dt 
                #Crop theta from -pi to pi 
                theta =np.arctan2(np.sin(theta), np.cos(theta)) #Make theta from -pi to pi 
 
                #last_time =  current_time 
                last_time = current_time 
 
                odom = self.fill_odom(x, y,theta, Sigma_pose, v, w) 
                self.odom_pub.publish(odom) 
                rate.sleep() 
 
    def wl_cb(self, msg): 
        self.wl = msg.data 
        self.received_wl = 1 
 
    def wr_cb(self, msg): 
        self.wr = msg.data 
        self.received_wr = 1 
 
    def fill_odom(self,x, y, theta, Sigma_pose, v, w): 
        # (x,y) -> robot position 
        # theta -> robot orientation 
        # Sigma_pose -> 3x3 pose covariance matrix 
        odom=Odometry() 
        odom.header.stamp =rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 
        odom.pose.pose.position.x = x 
        odom.pose.pose.position.y = y 
        odom.pose.pose.position.z = 0.0 
        quat=quaternion_from_euler(0.0, 0.0, theta) 
        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 
        odom.pose.covariance = [0.0]*36 
        # Fill the covariance matrix 
        odom.pose.covariance[0] = Sigma_pose[0,0] 
        odom.pose.covariance[1] = Sigma_pose[0,1] 
        odom.pose.covariance[5] = Sigma_pose[0,2] 
        odom.pose.covariance[6] = Sigma_pose[1,0] 
        odom.pose.covariance[7] = Sigma_pose[1,1] 
        odom.pose.covariance[11] = Sigma_pose[1,2] 
        odom.pose.covariance[30] = Sigma_pose[2,0] 
        odom.pose.covariance[31] = Sigma_pose[2,1] 
        odom.pose.covariance[35] = Sigma_pose[2,2] 
 
        odom.twist.twist.linear.x = v 
        odom.twist.twist.angular.z = w 
 
        return odom 
 
 
#---------------------- MAIN PROGRAM ----------------------#  
if __name__ == "__main__":  
    rospy.init_node('localization')  
    LocalizationClass()  