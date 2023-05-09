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
        self.x     = 0.0 # X position of the robot [m] 
        self.y     = 0.0 # Y position of the robot [m] 
        self.theta = 0.0 # Angle of the robot [rad]   

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
 
class GoToGoal():  
    '''
    This class will make the puzzlebot move to a given goal 
    '''  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  

        #--------------- Variables ---------------# 
        self.x_target   = 0.0  # X position of the goal 
        self.y_target   = 0.0  # Y position of the goal 
        self.d_goal     = 0.0  # Distance to goal at start [m]
        self.d_goal_tao = 0.0  # Distance to goal at tao [m]
        self.wr         = 0.0  # Right wheel speed [rad/s] 
        self.wl         = 0.0  # Left wheel speed [rad/s]         
        v_msg=Twist()          # Robot's desired speed  
        self.robot=Robot()     # Robot object instance
        self.lidar_received = False # Indicate if the laser scan has been received 
        self.current_state = 'Stop' # Robot's current state 

        #--------------- Publishers ---------------#  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        
        #--------------- Subscribers ---------------#  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
 
        #--------------- Init node ---------------#  
        rate = rospy.Rate(constants.node_freq) #freq Hz  
        rospy.loginfo("Node initialized") 

        #--------------- Main loop ---------------#
        while not rospy.is_shutdown():  
            self.robot.update_state(self.wr, self.wl, constants.deltat) #update the robot's state 
            if self.lidar_received: 
                 
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                thetaAO = self.get_theta_ao(closest_angle) 
                thetaGTG =self.get_theta_gtg(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                self.d_goal = np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2) 

                rospy.loginfo("Distance to goal: {}".format(self.d_goal))

                if self.at_goal():  
                    rospy.loginfo("Goal reached") 
                    self.current_state = 'Stop' 
                    #rospy.loginfo("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 
                elif self.current_state == 'GoToGoal':                  
                    if closest_range <= constants.fw_distance: 
                        # Implement the following walls behavior 
                        rospy.loginfo("Change to following walls") 
                        self.d_goal_tao = self.d_goal
                        self.current_state = "Clockwise"                    
                    else: 
                        # rospy.loginfo("Moving to the Goal") 
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg 
 
                elif self.current_state == 'Clockwise': 
                    rospy.loginfo("Distance to goal at tao: {}".format(self.d_goal_tao))
                    if self.d_goal < self.d_goal_tao - constants.threshold and abs(thetaAO - thetaGTG) < np.pi/2:
                        self.current_state = 'GoToGoal' 
                        rospy.loginfo("Change to go to goal") 
                    else: 
                        thetaFWC=self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise 
                        vFWC, wFWC = self.compute_fw_control(thetaFWC) 
                        v_msg.linear.x = vFWC 
                        v_msg.angular.z = wFWC                         
                 
                elif self.current_state == 'Stop': 
                    rospy.loginfo("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 
                         
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()  
     
    def at_goal(self): 
        '''
        This function returns true if the robot is close enough to the goal 
        '''
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2)<constants.target_position_tolerance 
 
    def get_closest_object(self, lidar_msg): 
        '''
        This function returns the closest object to the robot 
        This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 
        returns  closest_range [m], closest_angle [rad]
        '''
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 
 
     
    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        """
        This function returns the angle to the goal 
        """
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        return e_theta 
 
    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        """
        This function returns the linear and angular speed to reach a given goal 
        This functions receives the goal's position (x_target, y_target) [m] 
        and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        """
        # Compute errors
        ed=np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
 
        # Compute the robot's angular speed 
        kw = constants.kwmax*(1-np.exp(-constants.aw*e_theta**2))/abs(e_theta) 
        w  = kw*e_theta 
        
        # First turn to the goal 
        if abs(e_theta) > np.pi/8: 
            v=0  
        else: 
            kv=constants.kvmax*(1-np.exp(-constants.av*ed**2))/abs(ed) 
            v=kv*ed 
        return v,w 
 
    def get_theta_ao(self, theta_closest): 
        """
        This function returns the angle for the Avoid obstacle behavior  
        theta_closest is the angle to the closest object [rad] 
        This functions returns the angle for the Avoid obstacle behavior [rad] 
        """
        thetaAO=theta_closest-np.pi 
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
        return thetaAO 
  
    def get_theta_fw(self, thetaAO, clockwise): 
        """
        This function computes the linear and angular speeds for the robot 
        It receives thetaAO [rad] and clockwise [bool] 
        """
        if(clockwise):
            theta_fw = np.pi/2 + thetaAO
        else:
            theta_fw = - np.pi/2 + thetaAO
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))     
        return theta_fw  
     
    def compute_fw_control(self, thetaFW): 
        """
        This function computes the linear and angular speeds for the robot 
        It receives thetaFW [rad]    
        Compute linear and angular speeds 
        """
        v = constants.v_fw
        w = constants.kw_fw*thetaFW 
        return v,w
    
    def get_angle(self, idx, angle_min, angle_increment):  
        ## This function returns the angle for a given element of the object in the lidar's frame  
        angle= angle_min + idx * angle_increment  
        # Limit the angle to [-pi,pi]  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle  
     
    def polar_to_cartesian(self,r,theta):  
        ## This function converts polar coordinates to cartesian coordinates  
        x = r*np.cos(theta)  
        y = r*np.sin(theta)  
        return (x,y)  
     
    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = True  
 
    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 
         
    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  
     
    def goal_cb(self, goal):  
        ## This function receives a the goal from rviz.  
        rospy.loginfo("Goal received I'm moving to x= "+str(goal.pose.position.x)+" y= "+str(goal.pose.position.y)) 
        self.current_state = "GoToGoal" 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 
         
    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 
 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("bug_0", anonymous=True)  
    GoToGoal()  