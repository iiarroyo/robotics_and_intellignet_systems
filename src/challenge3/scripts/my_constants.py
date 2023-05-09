#!/usr/bin/env python

# put constants, like sampling time, here
node_freq = 10.0
deltat = 1.0/float(node_freq)
L = 0.19
r = 0.05
kv = 0.3
kw = 1.2
kvmax = 0.25 #linear speed maximum gain  
kwmax = 0.8    #angular angular speed maximum gain 
av = 2.0 #Constant to adjust the exponential's growth rate   
aw = 2.0 #Constant to adjust the exponential's growth rate 

target_position_tolerance=0.10 #target position tolerance [m] 
threshold = 0.20

v_fw = 0.1
kw_fw = 1.75

fw_distance = 0.42 # distance to activate the blended controller [m] 
ao_distance = 0.20 # distance to activate the obstacle avoidance [m] 
stop_distance = 0.15 # distance to stop the robot [m] 
eps=0.15 #Fat guard epsilon 
blended_alpha = 0.7 # blending factor for the blended controller (thet higher the more aggressive obstacle avoidance) 
