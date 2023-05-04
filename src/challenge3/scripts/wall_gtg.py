#!/usr/bin/env python  
from this import d
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 
 
class Robot(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius [m] 
        self.L = 0.19 #wheel separation [m] 
        ############ Variables ############### 
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 
     
    def update_state(self, wr, wl, delta_t): 
        #This function returns the robot's state 
        #This functions receives the wheel speeds wr and wl in [rad/sec]  
        # and returns the robot's state 
        v=self.r*(wr+wl)/2 
        w=self.r*(wr-wl)/self.L 
         
        self.theta=self.theta + w*delta_t 
        #Crop theta_r from -pi to pi 
        self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
 
        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta) 
 
        self.x=self.x+vx*delta_t  
        self.y=self.y+vy*delta_t 
         
 
#This class will make the puzzlebot move to a given goal 
class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
         
        self.robot=Robot() #create an object of the Robot class 
        ############ Variables ############### 
        self.x_target=0.0 #x position of the goal 
        self.y_target=0.0 #y position of the goal 
        self.goal_received=0 #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.15 #target position tolerance [m] 
        blended_distance = 0.8 # distance to activate the blended controller [m] 
        ao_distance = 0.3 # distance to activate the obstacle avoidance [m] 
        stop_distance = 0.20 # distance to stop the robot [m] 
        eps=0.15 #Fat guard epsilon 
        blended_alpha = 0.7 # blending factor for the blended controller (thet higher the more aggressive obstacle avoidance) 
        v_msg=Twist() #Robot's desired speed  
        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 
        self.current_state = 'Stop' #Robot's current state 
        follow_wall = False
        self.e_theta = 0
        self.thetaAO = 0
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
 
        #********** INIT NODE **********###  
        freq = 10 
        rate = rospy.Rate(freq) #freq Hz  
        Dt =1.0/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 
        ################ MAIN LOOP ################  
         
        while not rospy.is_shutdown():  
            self.robot.update_state(self.wr, self.wl, Dt) #update the robot's state IMPORTANT!! CALL IT every loop  
            print("\n--------\n")
            if self.lidar_received: 

                self.lidar_received = False
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                print("Closest range:", closest_range)
                
                if self.goal_received == 1:
                    self.current_state = "GoToGoal"
                    self.goal_received = 0

                ed = np.sqrt(pow(self.x_target - self.robot.x, 2) + pow(self.y_target - self.robot.y, 2))  #At goal
                tetha_fwc = self.thetaAO - np.pi/2
                theta_fwcc = self.thetaAO + np.pi/2
                

                if self.current_state == "GoToGoal":
                    if(ed<self.target_position_tolerance):    #At goal
                        self.current_state = 'Stop' 
                    elif(closest_range<stop_distance and cw):     #CW
                        self.current_state = "CW Follow"
                    elif(closest_range<stop_distance and ccw):    #CCW
                        self.current_state = "CCW Follow"
                    else:
                        print("Current State:","Going to goal")              #Go to goal
                        v_gtg, w_gtg, ed = self.compute_gtg_control(self.x_target,self.y_target,self.robot.x,self.robot.y,self.robot.theta) 
                        v_msg.linear.x = v_gtg
                        v_msg.angular.z = w_gtg                             

                elif self.current_state == "CW Follow":
                    pass

                elif self.current_state == "CCW Follow":
                    pass

                elif self.current_state == 'AvoidObstacle':  
                    if(ed<self.target_position_tolerance):     #At goal
                        self.current_state = 'Stop' 
                    elif(closest_range>ao_distance):           #Near to obstavle                    
                        self.current_state = 'Blended'
                    else: 
                        print("Current State:","Avoiding obstacle")         #Avoinding
                        v_ao, w_ao = self.compute_ao_control(self.lidar_msg) 
                        v_msg.linear.x = v_ao 
                        v_msg.angular.z = w_ao 
                         
                elif self.current_state == 'Stop': 
                    print("Current State:","Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 
                 
            self.pub_cmd_vel.publish(v_msg)  

            print("Robot pose  x: ",self.robot.x, "y:",self.robot.y)
            print("Target pose x: ",self.x_target,"y:",self.y_target)
            #print("Current State",self.current_state)
            rate.sleep()  
     
    def at_goal(self): 
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2)<self.target_position_tolerance 
 
    def get_closest_object(self, lidar_msg): 
        #This function returns the closest object to the robot 
        #This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 
        #returns  closest_range [m], closest_angle [rad], 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        # limit the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 
     
    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        kv = 0.3
        kw = 1.2
        # errors
        self.e_theta = np.arctan2(y_target - y_robot , x_target-x_robot) - theta_robot
        e_d = np.sqrt(pow(y_target - y_robot, 2) + pow(x_target - x_robot, 2))

        ######### YOU CAN ADD YOUR OWN GO-TO-GOAL CODE HERE ######## 
        ################ YOUR CODE HERE ########################### 
        v= min(kv*e_d,0.3)     # Modify this line to change the robot's speed 
        w= kw*e_theta # Modify this line to change the robot's angular speed 
        ################ END OF YOUR CODE ########################### 
        return v, w, e_d
 
         
    def compute_ao_control(self,lidar_msg):  
        ## This function computes the linear and angular speeds for the robot  
        # given the distance and the angle theta as error references  
        kvmax = 0.1 #linear speed maximum gain  
        a = 1.0   #Constant to adjust the exponential's growth rate  
        kw = 0.2 # Angular speed gain  
        v_desired=0.3  
        d_max = 1.0 #Maximum distance to consider obstacles 
        d_min=0.1 #If there are obstacles closer than this distance, the robot will stop 
        angles_array=[]  
        x_array=[]  
        y_array=[]  
        #Trimm lidar array to consider just the interesting parts 
        array_size = len(lidar_msg.ranges) 
        min_array=int(array_size*0.25) 
        max_array=array_size-min_array 
        if min(lidar_msg.ranges)< d_min: #If there are obstacles closer than d_min, the robot will stop 
            v=0 
            w=0 
        else: 
            if  min(lidar_msg.ranges[min_array:max_array])> d_max: #If there are no obstacles (to the front) 
                v = v_desired #Move to the front 
                w = 0.0  
            else:  
                for i in range(min_array,max_array): 
                    angle = self.get_angle(i, lidar_msg.angle_min, lidar_msg.angle_increment)  
                    r = lidar_msg.ranges[i]  
                    if not (np.isinf(r)): #Ignore infinite values 
                        thetaAO=angle-np.pi #flip the vector to point away from the obstacle.  
                        #limit the angle to [-pi,pi] 
                        self.thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
                        r_AO = lidar_msg.range_max - r #make vectors closer to the obstacle bigger.  
                        (x,y)=self.polar_to_cartesian(r_AO,thetaAO)  
                        x_array.append(x)  
                        y_array.append(y)  
                        angles_array.append(thetaAO)  
                xT=sum(x_array)  
                yT=sum(y_array)  
                thetaT=np.arctan2(yT,xT)  
                dT=np.sqrt(xT**2+yT**2)  
                if not dT == 0:  
                    kv=kvmax*(1-np.exp(-a*dT**2))/(dT) #Constant to change the speed  
                    v = kv * dT #linear speed  
                w = kw * thetaT #angular speed  
        return v, w 
 
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
        print("Goal received I'm moving") 
        #self.current_state = "GoToGoal" 
        # reset the robot's state 
        self.robot.x=0 
        self.robot.y=0  
        self.robot.theta=0 
        # assign the goal position 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 
        self.goal_received=1 
        print("Goal x: ", self.x_target) 
        print("Goal y: ", self.y_target) 
         
    def cleanup(self):  
        #This function is called just before finishing the node  
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 
 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("go_to_goal_with_obstacles", anonymous=True)  
    GoToGoal()