#!/usr/bin/env python

import rospy
import geometry_msgs
import std_msgs
from visual_servo_drone.msg import RcChannels
from visual_servo_drone.msg import ControlInput
import tf2_ros
import tf2_msgs.msg
import tf.transformations as tft
import math
from threading import Thread
import time

max_yaw = 0.8
max_xy = 0.5
max_z = 0.25

age_ms = 100 # 10 hz requirement to produce safe outputs
max_age = rospy.Duration(nsecs=age_ms*1000000)

u_des_latest = ControlInput()

first_rc_recv = False

def handle_rc_chan(data):
	# chan 1: yaw
	# chan 2: throttle
	# chan 3: pitch
	# chan 4: roll
    
    global u_des_latest
    global first_rc_recv
	
	# create message
    u_des = ControlInput()
	
    u_des.x_b_dot = 0
    u_des.y_b_dot = 0
    u_des.z_b_dot = 0
    u_des.theta_b_dot = 0
	
	# get age of incoming message
    msg_age = rospy.Time.now() - data.header.stamp
	
	# verify the age and validty of the message
    if(msg_age < max_age and (data.chan1 > 900)): 
		# map to -1 to 1 (roughly)
        yaw_raw = (data.chan1 - 1500) / 500
        z_raw = (data.chan2 - 1500) / 500
        x_raw = (data.chan3 - 1500) / 500
        y_raw = (data.chan4 - 1500) / 500
		
		# dead zone
        if(abs(yaw_raw) < 0.05):
            yaw_raw = 0
		
        if(abs(x_raw) < 0.05):
            x_raw = 0
			
        if(abs(y_raw) < 0.05):
            y_raw = 0
			
        if(abs(z_raw) < 0.2):
            z_raw = 0
		
		#print(f"{x_raw} {y_raw} {z_raw} {yaw_raw}");
		
		# map to limits and saturate
        yaw = sat(yaw_raw * max_yaw, max_yaw)
        x = sat(x_raw * max_xy, max_xy)
        y = sat(y_raw * max_xy, max_xy)
        z = sat((-z_raw) * max_z, max_z)

		# populate into messgae
        u_des.x_b_dot = x
        u_des.y_b_dot = y
        u_des.z_b_dot = z
        u_des.theta_b_dot = yaw
        
        u_des_latest = u_des
        
        first_rc_recv = True
		
	
def sat(val, lim):
	if(val > lim):
		return lim
	elif(val < (-lim)):
		return -lim
	else:
		return val

if(__name__ == "__main__"):	
	# setup node
    rospy.init_node('velocity_planner', anonymous=True)
	
	# setup publisher and subscriber
    sub_rc_chan = rospy.Subscriber('mav/rc_channels_raw', RcChannels, handle_rc_chan)
    pub_u_des = rospy.Publisher('control/des', ControlInput, queue_size=1)
    
    rate = rospy.Rate(20.0)
    
    run_loop = True
	
	# loop at a 50hz target speed
    while(not rospy.is_shutdown() and run_loop):
        if(first_rc_recv):
            pub_u_des.publish(u_des_latest)
        rate.sleep()
        
