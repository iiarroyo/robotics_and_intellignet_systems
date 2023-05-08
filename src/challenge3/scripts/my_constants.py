#!/usr/bin/env python

# put constants, like sampling time, here
node_freq = 50.0
deltat = 1.0/node_freq
L = 0.18
r = 0.065


target_position_tolerance=0.10 #target position tolerance [m] 
FW_distance = 0.4 # distance to activate the blended controller [m] 
ao_distance = 0.20 # distance to activate the obstacle avoidance [m] 
stop_distance = 0.15 # distance to stop the robot [m] 
eps=0.15 #Fat guard epsilon 
blended_alpha = 0.7 # blending factor for the blended controller (thet higher the more aggressive obstacle avoidance) 
