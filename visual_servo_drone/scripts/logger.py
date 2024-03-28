import numpy as np
import socket
import queue
import time
import os

from visual_servo_drone.msg import ControlInput
from visual_servo_drone.msg import PositionStates
from visual_servo_drone.msg import SafetyStates
from visual_servo_drone.msg import RcChannels

from std_msgs.msg import UInt32

import rospy
import geometry_msgs
import std_msgs

log_dir = "/home/angus/logs/"

def log(var, string):
    f = open(log_dir + "log" + str(counter) + "/log" + str(counter) + "_" + var + ".csv", "a")
    f.write(string + "\n")
    f.close()
    
def log_control_des(msg):
    log("control_des", f"{rospy.Time.now()},{msg.x_b_dot},{msg.y_b_dot},{msg.z_b_dot},{msg.theta_b_dot}")

def log_control_out(msg):
    log("control_out", f"{rospy.Time.now()},{msg.x_b_dot},{msg.y_b_dot},{msg.z_b_dot},{msg.theta_b_dot}")
    
def log_rc_chan(msg):
    log("rc_chan", f"{rospy.Time.now()},{msg.chan1},{msg.chan2},{msg.chan3},{msg.chan4},{msg.chan5},{msg.chan6},{msg.chan7},{msg.chan8}")
    
def log_pos(msg):
    log("pos_state", f"{rospy.Time.now()},{msg.x},{msg.y},{msg.z},{msg.theta}")
    
def log_saf(msg):
    log("saf_state", f"{rospy.Time.now()},{msg.phi_h},{msg.phi_v},{msg.d}")
    
def log_frame(msg):
    log("aruco_frame", f"{rospy.Time.now()},{msg.data}")
    
# MAIN FUNCTION

if(__name__ == '__main__'):
    
    rospy.init_node('logger', anonymous=True)
    
    f = open(log_dir + ".counter", "r")
    try:
        counter = int(f.read()) + 1
    except:
        counter = 0
    f.close()
    
    f = open(log_dir + ".counter", "w")
    f.write(str(counter))
    f.close()
    
    os.mkdir(log_dir + "log" + str(counter))
    
    print("logging at: " + str(counter))
    
    rospy.Subscriber('control/des', ControlInput, log_control_des)
    rospy.Subscriber('control/out', ControlInput, log_control_out)
    
    rospy.Subscriber('mav/rc_channels_raw', RcChannels, log_rc_chan)
    
    rospy.Subscriber('uav/position', PositionStates, log_pos)
    rospy.Subscriber('aruco/saftey_states', SafetyStates, log_saf)
    
    rospy.Subscriber('aruco/frame', UInt32, log_frame)
    
    pub_log = rospy.Publisher('log/num', UInt32, queue_size=1)
    
    log_num_msg = UInt32()
    log_num_msg.data = counter
    
    rate = rospy.Rate(1)

    while(not rospy.is_shutdown()):
        
        pub_log.publish(log_num_msg)
                
        rate.sleep()
