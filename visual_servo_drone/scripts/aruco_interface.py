#!/usr/bin/env python

from threading import Thread
import time

import rospy
import geometry_msgs
import std_msgs
from visual_servo_drone.msg import Aruco
from visual_servo_drone.msg import SafetyStates
from std_msgs.msg import UInt32

import paho.mqtt.client as mqtt
import paho.mqtt.enums as mqttenum
import pickle

import tf.transformations as tf2

class ArucoInterface():
    
    def __init__(self):
        # ros publisher
        self.pub_aruco = rospy.Publisher('aruco/marker', Aruco, queue_size=1)
        self.pub_ss = rospy.Publisher('aruco/saftey_states', SafetyStates, queue_size=1)
        self.pub_frame = rospy.Publisher('aruco/frame', UInt32, queue_size=1)
        
        # create this node
        rospy.init_node('aruco_interface', anonymous=True)
        
    # MQTT connect function
    def on_connect(self, client, userdata, flags, reason_code):
        print(f"connected to mqtt broker: {userdata}, {flags}, {reason_code}")
        client.subscribe("aruco/marker")
        client.subscribe("aruco/safety")
        client.subscribe("aruco/frame")

    def on_message(self, client, userdata, mqtt_msg):
        if(mqtt_msg.topic == "aruco/marker"):
            # decode message
            msg = pickle.loads(mqtt_msg.payload)
            
            curr_aruco = Aruco()

            quat = tf2.quaternion_from_matrix(msg['rot_mat'])

            curr_aruco.marker.rotation = geometry_msgs.msg.Quaternion(*quat)
            curr_aruco.marker.translation = geometry_msgs.msg.Point(*msg['tvec'])

            curr_aruco.id = int(msg['id'])
                                        
            self.pub_aruco.publish(curr_aruco)
            
        elif(mqtt_msg.topic == "aruco/safety"):
            # decode message
            msg = pickle.loads(mqtt_msg.payload)
                    
            curr_ss = SafetyStates()
            
            curr_ss.header.stamp = rospy.Time.now()
            curr_ss.phi_h = msg['phi_h']
            curr_ss.phi_v = msg['phi_v']
            curr_ss.d = msg['d']
                                        
            self.pub_ss.publish(curr_ss)
        elif(mqtt_msg.topic == "aruco/frame"):
            
            self.pub_frame.publish(UInt32(int(mqtt_msg.payload)))

def log_num(data):
    mqttc.publish("log/num", int(data.data))

if(__name__ == "__main__"):
    
    # setup interface class
    aruco_if = ArucoInterface()
    
    # Start the mqtt thread
    mqttc = mqtt.Client(mqttenum.CallbackAPIVersion.VERSION1, client_id="ros_sub", clean_session=False, userdata=None)
    mqttc.on_connect = aruco_if.on_connect
    mqttc.on_message = aruco_if.on_message
    mqttc.connect("localhost", 1883, 60)

    mqtt_thread = Thread(target=mqttc.loop_forever)
    mqtt_thread.start()
    
    rospy.Subscriber('log/num', UInt32, log_num)
    
    # spin the main thread at 1hz
    rate = rospy.Rate(1) # 1hz
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        mqttc.disconnect()
        print("ending connection thread")
