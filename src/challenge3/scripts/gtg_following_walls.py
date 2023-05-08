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

def run_once(f):
    def wrapper(*args, **kwargs):
        if not wrapper.has_run:
            wrapper.has_run = True
            return f(*args, **kwargs)
    wrapper.has_run = False
    return wrapper

class GoToGoal():
    '''
    This class will make the puzzlebot move to a given goal 
    '''  
    def __init__(self):          
        rospy.on_shutdown(self.cleanup)   

        #--------------- Variables ---------------# 
        self.x_target = 0.0           # X goal position   [m]
        self.y_target = 0.0           # Y goal position   [m]
        self.d_goal   = 0.0           # distance to goal [m]
        self.d_goal_tao = 0.0
        self.wr       = 0.0           # Right wheel speed [rad/s] 
        self.wl       = 0.0           # Left wheel speed  [rad/s] 
        self.theta_ao = 0.0           # Avoid obstacle angle [rad]
        self.thet_gtg = 0.0           # Go to goal angle  [rad]
        self.lidar_received = False   # Indicate if the laser scan has been received 
        self.current_state  = 'GoToGoal' # Robot's current state 
        self.robot = Robot()          # Create a robot object
        v_msg = Twist()               # Robot's desired speed  

        
        #--------------- Publishers ---------------#  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        #--------------- Subscribers ---------------#  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 

        #--------------- Init node ---------------#  
        rate = rospy.Rate(constants.node_freq) #freq Hz  
        print("Node initialized") 

        #--------------- Main loop ---------------#
        save_distance_once = run_once(self.save_distance)
        save_distance_once.has_run = False
        while not rospy.is_shutdown():  
            #Update odometry
            self.robot.update_state(self.wr, self.wl, constants.deltat)  
            if not self.lidar_received:
                continue
            closest_range, _= self.get_closest_object(self.lidar_msg) 

            if self.current_state == 'GoToGoal':
                save_distance_once.has_run = False
                if closest_range < constants.FW_distance:
                    self.current_state = 'Follow_Wall'
                elif self.at_goal():
                    self.current_state = 'Stop'
                    rospy.loginfo("I'm in the goal")
                else:
                    print("GoToGoal")
                    v_msg.linear.x, v_msg.angular.z = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta)

            elif self.current_state == 'Follow_Wall':
                rospy.loginfo("distance2goal: {} distance@tao: {}".format(self.d_goal, self.d_goal_tao))
                save_distance_once(self.d_goal)
                #TODO: keep track of pose while following wall
                #      compute control should return theta and distance, not velocities
                if self.at_goal():
                    self.current_state = 'Stop'
                    rospy.loginfo("I'm in the goal")
                #BUG0
                #TODO: make subtraction a constant
                elif self.d_goal < self.d_goal_tao - 0.2 and abs(self.thet_gtg - self.theta_ao) < np.pi/2:
                    rospy.loginfo("CLEAR SHOT")
                    self.current_state = 'GoToGoal'
                #BUG2
                elif False and self.d_goal < self.d_goal_tao - 0.2 and abs(self.thet_gtg - self.theta_ao) < np.pi/2:
                    rospy.loginfo("CLEAR SHOT")
                    self.current_state = 'GoToGoal'

                elif closest_range > constants.FW_distance:

                    # pass
                    # self.current_state = 'GoToGoal'
                    print("wall is far away")
                else:
                    rospy.loginfo("Following Wall")
                    v_gtg, w_gtg, _ = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta)
                    v_ao, w_ao = self.compute_ao_control(self.lidar_msg)
                    pi = np.pi
                    #TODO: make this not hard coded
                    kw = 1.5
                    theta_fw = pi/2 + self.theta_ao
                    theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
                    th_fwc = -np.pi/2 + self.theta_ao
                    if (abs(th_fwc-self.thet_gtg) <= np.pi/2):
                        pi = pi*-1
                    # print("Theta FW: ",theta_fw)
                    #TODO: make this NOT hard coded
                    v_msg.linear.x = 0.11
                    v_msg.angular.z = kw*theta_fw

            elif self.current_state == 'Stop':
                save_distance_once.has_run = False
                print("Stop") 
                v_msg.linear.x = 0 
                v_msg.angular.z = 0 

            self.pub_cmd_vel.publish(v_msg)
            
            rate.sleep()  

    def save_distance(self, current_distance):
        rospy.loginfo("saved distance {}".format(current_distance))
        self.d_goal_tao = current_distance

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

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        """
        This function returns the linear and angular speed to reach a given goal 
        This functions receives the goal's position (x_target, y_target) [m] 
        and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        """
        # Compute errors
        ed           = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 
        theta_target = np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta      = theta_target-theta_robot 
        e_theta      = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
 
        #Compute the robot's angular speed 
        kw = constants.kwmax*(1-np.exp(-constants.a2*e_theta**2))/abs(e_theta) 
        w = kw*e_theta 
        
        #we first turn to the goal 
        if abs(e_theta) > np.pi/8: 
            v=0 #linear speed  
        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv=constants.kvmax*(1-np.exp(-constants.av*ed**2))/abs(ed) 
            v=kv*ed 
        return v,w 
             
    def compute_ao_control(self,lidar_msg):
        ## This function computes the linear and angular speeds for the robot  
        # given the distance and the angle theta as error references  
        constants.kvmax = 0.3 #linear speed maximum gain  
        a = 1.0 #Constant to adjust the exponential's growth rate  
        kw = 0.35  # Angular speed gain  
        v_desired=0.3  
        d_max = 1.0 #Maximum distance to consider obstacles 
        d_min=0.12 #If there are obstacles closer than this distance, the robot will stop 
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
            # if  min(lidar_msg.ranges[min_array:max_array])> d_max: #If there are no obstacles (to the front) 
            #     v = v_desired #Move to the front 
            #     w = 0.0  
            # else:  
                for i in range(min_array,max_array): 
                    angle = self.get_angle(i, lidar_msg.angle_min, lidar_msg.angle_increment)  
                    r = lidar_msg.ranges[i]  
                    if not (np.isinf(r)): #Ignore infinite values 
                        thetaAO=angle-np.pi #flip the vector to point away from the obstacle.  
                        #limit the angle to [-pi,pi] 
                        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
                        r_AO = lidar_msg.range_max - r #make vectors closer to the obstacle bigger.  
                        (x,y)=self.polar_to_cartesian(r_AO,thetaAO)  
                        x_array.append(x)  
                        y_array.append(y)  
                        angles_array.append(thetaAO)  

                xT=sum(x_array)  
                yT=sum(y_array)  

                thetaT=np.arctan2(yT,xT)
                min_range = min(lidar_msg.ranges)
                idx = idx = lidar_msg.ranges.index(min_range)
                self.theta_ao = lidar_msg.angle_min + idx * lidar_msg.angle_increment
                # rospy.loginfo("theta_ao: {}".format(self.theta_ao))
                dT=np.sqrt(xT**2+yT**2)  
                #BUG: at some point, this is not satisfied
                if not dT == 0:  
                    kv=constants.kvmax*(1-np.exp(-a*dT**2))/(dT) #Constant to change the speed  
                    v = kv * dT #linear speed  

                w = kw * thetaT #angular speed  
        return v, w 

    def get_angle(self, idx, angle_min, angle_increment):  
        '''
        This function returns the angle for a given element of the object in the lidar's frame  
        '''
        angle= angle_min + idx * angle_increment  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle  

    def polar_to_cartesian(self,r,theta):  
        '''
        This function converts polar coordinates to cartesian coordinates  
        '''
        x = r*np.cos(theta)  
        y = r*np.sin(theta)  
        return (x,y)  

    def laser_cb(self, msg):   
        '''
        This function receives a message of type LaserScan   
        '''
        self.lidar_msg = msg  
        self.lidar_received = True  

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

    def goal_cb(self, goal):  
        '''
        This function receives a the goal from rviz. 
        '''
        print("Goal received I'm moving") 
        self.current_state = "GoToGoal" 
        # reset the robot's state 
        self.robot.x     = 0 
        self.robot.y     = 0  
        self.robot.theta = 0 
        # assign the goal position 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 
        print("Goal x: ", self.x_target) 
        print("Goal y: ", self.y_target) 

    def cleanup(self):  
        '''
        This function is called just before finishing the node.   
        '''
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

#--------------- Main program ---------------#  
if __name__ == "__main__":  
    rospy.init_node("go_to_goal_with_obstacles", anonymous=True)  
    GoToGoal()