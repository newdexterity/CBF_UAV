#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Float32, Bool
from visual_servo_drone.msg import ControlInput
from visual_servo_drone.msg import RcChannels
from visual_servo_drone.msg import PositionStates

import os

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil

import numpy as np
import math
import time
import sys
from threading import Thread

class MavlinkManager():

    def __init__(self):
        # Flag to run loop
        self.run_loop = True
        
        self.connect_status_pub = rospy.Publisher("/mav/heartbeat", Bool, queue_size=1)
        
        self.connected = False
        
        while(not self.connected):
            # UART on RPI
            #self.uav = mavutil.mavlink_connection("/dev/ttyS0", 961200)
            self.uav = mavutil.mavlink_connection("/dev/ttyAMA0", 921600)

            # Prodcue a companion computer (18) heartbeat to the flight controller
            time.sleep(0.2)
            self.uav.mav.heartbeat_send(18, 0, 0, 0, 0)

            rospy.loginfo("waiting for fc")
            
            # Wait to hear one back
            msg = self.uav.recv_match(blocking=True, type='HEARTBEAT', timeout=5)
            
            if(msg != None):
                self.connected = True
            else:
                self.connected = False
                self.connect_status_pub.publish(Bool(False))
        
        # request quaterntion
        #self.req_msg(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20000)
        #self.req_msg(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20000)
        self.req_msg(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 20000)

        rospy.loginfo("done")
        
        # Setup the ros communications
        
        # setup publisher
        self.rc_chan_pub = rospy.Publisher("/mav/rc_channels_raw", RcChannels, queue_size=1)

        rospy.Subscriber("/control/out", ControlInput, self.offboard_velocity)
        rospy.Subscriber("/uav/position", PositionStates, self.vision_position)

        self.mav_yaw = 0

        # dial value
        self.dial_val = 0

        # start time
        self.start_time = time.time()

        # report position estimate rates
        self.count = 0
        self.last_calc = time.time()

        self.aruco_aquired = False
    
        # Counter to send heartbeats async
        self.time_last_heartbeat = time.time()

         # Seconds between heartbeats
        self.heartbeat_rate = 1

    def req_msg(self, msg_id, interval, timeout=0.5):
        self.uav.mav.command_long_send(
            self.uav.target_system,  # Target system ID
            self.uav.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            int(0),  # Confirmation
            float(msg_id),     # param1: Message ID to be streamed
            interval, # param2: Interval in microseconds
            float(2),          # param3 (unused)
            float(0),          # param4 (unused)
            float(0),          # param5 (unused)
            float(0),          # param6 (unused)
            float(0)
            )
            
        #response = self.uav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
        #print(response)

    def vision_position(self, message):
        if(True):
            self.uav.mav.vision_position_estimate_send(int((time.time() - self.start_time) * 1000000),
                                                    float(message.x),
                                                    float(message.y),
                                                    float(message.z),
                                                    0, 0, message.theta) # roll pitch yaw


    def offboard_velocity(self, message):
        
            offboard_msg = self.uav.mav.set_position_target_local_ned_encode(0,
                                                               0, 0, # target system, target component
                                                               8, # 8 for body fixed NED OFFSET frame (1 for absolute frame)
                                                               0b010111000111, # x y z vel, and yaw rate only mask
                                                               0, # x target
                                                               0, # y target
                                                               0, # z target
                                                               message.x_b_dot, # x vel
                                                               message.y_b_dot, # y vel
                                                               message.z_b_dot, # z vel
                                                               0, 0, 0, # accelerations
                                                               0, # yaw target
                                                               message.theta_b_dot) # yaw rate

                        

            self.uav.mav.send(offboard_msg)

            #rospy.loginfo(offboard_msg)

    # Ends loop and joins loop thread
    def end_loop(self):
        self.run_loop = False

        self.loop_thread.join()

    # Starts the main loop on a new thread
    def start_loop(self):
        self.loop_thread = Thread(target=self.main_loop)
        self.run_loop = True
        self.loop_thread.start()

    # main loop
    def main_loop(self):

        # loop
        while(self.run_loop):

            # Send heartbeat at set rate
            if(time.time() - self.time_last_heartbeat > self.heartbeat_rate):
                #print("Producing Heartbeat")
                # 18 = companion computer, 
                self.uav.mav.heartbeat_send(18, 0, 0, 0, 0)

                fps = (self.count / (time.time() - self.last_calc))

                self.count = 0
                self.last_calc = time.time()

                #gd.debug(f"{mavz} {(shared_array[gd.SharedMem.Z.value] / 1000)} {target_z} {str(mav_z - ((shared_array[gd.SharedMem.Z.value] / 1000) - (target_z)))}", debug_level=gd.DebugLevels.SYST)
                
                self.time_last_heartbeat = time.time()

            # check if there is a message
            msg = self.uav.recv_match(blocking=False)

            # check is a good messgae
            if(msg != None and msg.get_msgId() >= 0):

                # get the message type as a string
                msg_type = msg.get_type()
                
                if(msg_type == "HEARTBEAT"):
                    self.connect_status_pub.publish(Bool(True))
                    pass

                if(msg_type == "SYSTEM_TIME"):
                    pass
                    
                if(msg_type == "COMMAND_ACK"):
                    pass

                if(msg_type == "RC_CHANNELS"):
                    chan_msg = RcChannels()
                    
                    chan_msg.header.stamp = rospy.Time.now()
                    
                    chan_msg.chan1 = msg.chan1_raw
                    chan_msg.chan2 = msg.chan2_raw
                    chan_msg.chan3 = msg.chan3_raw
                    chan_msg.chan4 = msg.chan4_raw
                    chan_msg.chan5 = msg.chan5_raw
                    chan_msg.chan6 = msg.chan6_raw
                    chan_msg.chan7 = msg.chan7_raw
                    chan_msg.chan8 = msg.chan8_raw
                    chan_msg.chan9 = msg.chan9_raw
                    chan_msg.chan10 = msg.chan10_raw
                    chan_msg.chan11 = msg.chan11_raw
                    chan_msg.chan12 = msg.chan12_raw
                    chan_msg.chan13 = msg.chan13_raw
                    chan_msg.chan14 = msg.chan14_raw
                    chan_msg.chan15 = msg.chan15_raw
                    chan_msg.chan16 = msg.chan16_raw

                    self.rc_chan_pub.publish(chan_msg)
                    pass

                if(msg_type == "ATTITUDE"):
                    self.mav_yaw = msg.yaw
                    pass 

                if(msg_type == "LOCAL_POSITION_NED"):
                    pass

                if(msg_type == "ATTITUDE_QUATERNION"):
                    quat = Quaternion()
                    quat.w = msg.q1
                    quat.x = msg.q2
                    quat.y = msg.q3
                    quat.z = msg.q4

                    #self.quat_publisher.publish(quat)
                    pass

                


if __name__ == '__main__':
    
    rospy.init_node("mavlink_interface", anonymous=True)

    # Create the mavlink communication object
    manager = MavlinkManager()
    
    if(manager.connected):
    
        # Start the mavlink looop
        manager.start_loop()

        # setup subscriber for velocity target
        #rospy.Subscriber("/nmpc/velocity_setpoint", Twist, manager.offboard_velocity)

        # setup subscriber for aruco localisation
        #rospy.Subscriber("/nmpc/aruco_input", AUV_States, manager.vision_position)

        # Loop forever and service the node
        rospy.spin()

        # end the mavlink loop after the node dies
        manager.end_loop()
