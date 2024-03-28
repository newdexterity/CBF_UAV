#!/usr/bin/env python

import rospy
import geometry_msgs
import std_msgs
from visual_servo_drone.msg import Aruco
from visual_servo_drone.msg import PositionStates
import tf2_ros
import tf2_msgs.msg
import tf.transformations as tft
import math
from threading import Thread
import time

class StaticFrame():
	def __init__(self, parent, child, x, y, z, z_r, y_r, x_r, delay):
		
		self.br = tf2_ros.StaticTransformBroadcaster()
		self.t = geometry_msgs.msg.TransformStamped()
		
		self.delay = delay
		self.loop = True
		
		self.t.header.stamp = rospy.Time.now()
		self.t.header.frame_id = parent
		self.t.child_frame_id = child
		
		self.t.transform.translation.x = float(x)
		self.t.transform.translation.y = float(y)
		self.t.transform.translation.z = float(z)

		quat = tft.quaternion_from_euler(float((math.pi/180)*z_r),float((math.pi/180)*y_r),float((math.pi/180)*x_r))
		self.t.transform.rotation.x = quat[0]
		self.t.transform.rotation.y = quat[1]
		self.t.transform.rotation.z = quat[2]
		self.t.transform.rotation.w = quat[3]
		
		self.thread = Thread(target=self.loop_static_frame)
		self.thread.start()
	
	def stop(self):
		self.loop = False
	
	def loop_static_frame(self):
		while self.loop:
			self.br.sendTransform(self.t)
			
			time.sleep(self.delay)

# for tracking what markers to average to produce postion estimate
marker_available = {}

age_ms = 100 # 10 hz requirement to produce safe outputs
max_age = rospy.Duration(nsecs=age_ms*1000000)

def handle_marker(data):
	# make the transform broadcaster and an empty stamped transform
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()
	
	# Assemble the message
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "uav_camera"
	t.child_frame_id = "marker_" + str(data.id)
	t.transform = data.marker
	
	br.sendTransform(t)
	
	# store in local markers age db
	marker_available[str(data.id)] = {"id": str(data.id), "stamp": rospy.Time.now()}
	
if(__name__ == "__main__"):	
	rospy.init_node('position_estimator', anonymous=True)
	
	sf = []
	
	# Setup the relations betwen points (should read from a config file)
	sf.append(StaticFrame("uav_camera", "uav_body", 0, 0, 0, -90, -90, 0, 0.1))
	
	sf.append(StaticFrame("marker_66", "marker_66_orr", 0, 0, 0, -90, 90, 180, 0.1))
	sf.append(StaticFrame("marker_66_orr", "world_66", -2, 0, 1, 0, 0, 0, 0.1))
		
	sf.append(StaticFrame("marker_68", "marker_68_orr", 0, 0, 0, -90, 90, 180, 0.1))
	sf.append(StaticFrame("marker_68_orr", "world_68", -2, 0, 2, 0, 0, 0, 0.1))
	
	
	sf.append(StaticFrame("marker_65", "marker_65_orr", 0, 0, 0, -90, 90, 180, 0.1))
	sf.append(StaticFrame("marker_65_orr", "world_65", -2, 0, 2, 0, 0, 0, 0.1))
	
	sf.append(StaticFrame("marker_67", "marker_67_orr", 0, 0, 0, -90, 180, 180, 0.1))
	sf.append(StaticFrame("marker_67_orr", "world_67", -2, 0, 2, 0, 0, 0, 0.1))

	
	sub_aruco = rospy.Subscriber('aruco/marker', Aruco, handle_marker)
	
	pub_pos = rospy.Publisher('uav/position', PositionStates, queue_size=1)	
	
	# estimate positon
	rate = rospy.Rate(50.0)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	run_loop = True
	
	# loop at a 50hz target speed
	while(not rospy.is_shutdown() and run_loop):
		try:
			# to compute average
			markers_used = 0
			
			x = 0
			y = 0
			z = 0
			theta = 0

			# go through each marker ever seen
			for marker in marker_available.items():
			# see if this marker has been seen recently
				marker_age = rospy.Time.now() - marker[1]["stamp"]
				if(marker_age < max_age):
					markers_used = markers_used + 1
					# look up the transform from world origin to uav centroid
					trans = tfBuffer.lookup_transform("world_" + str(marker[1]["id"]), "uav_body", rospy.Time())
					
					(roll, pitch, yaw) = tft.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
					
					theta = theta + yaw
					x = x + trans.transform.translation.x
					y = y + trans.transform.translation.y
					z = z + trans.transform.translation.z
				
			#print(yaw)
			
			if(markers_used > 0):
				# populate the position states message
				curr_pos = PositionStates()
				
				curr_pos.header.stamp = rospy.Time.now()
				curr_pos.theta = theta / markers_used
				curr_pos.x = x / markers_used
				curr_pos.y = y / markers_used
				curr_pos.z = z / markers_used
				
				pub_pos.publish(curr_pos)
			pass
		except KeyboardInterrupt:
			run_loop = False
		except Exception as e:
			pass
			#print(e)
		
		rate.sleep()


	for frame in sf:
		frame.stop()
