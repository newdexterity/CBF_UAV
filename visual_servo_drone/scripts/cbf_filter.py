import numpy as np
import cvxpy as cp
import socket
import queue
import time
from threading import Thread

from visual_servo_drone.msg import ControlInput
from visual_servo_drone.msg import PositionStates
from visual_servo_drone.msg import SafetyStates
import tf2_ros
import tf2_msgs.msg
import tf.transformations as tft
import rospy
import geometry_msgs
import std_msgs

# CONTROL BARRIER FUNCTION

# Number of control inputs
m = 4

# System gains
fov_h = 0.4 # from testing
fov_v = 0.3 # from testing
k = pow(0.65, 2) 

class ControlBarrier():
    def __init__(self):
        
        rospy.init_node('cbf_filter', anonymous=True)
        
        self.last_pos_state = PositionStates()
        self.last_saf_state = SafetyStates()
        
        self.sub_control = rospy.Subscriber('control/des', ControlInput,  self.handle_des_control)
        self.pub_control = rospy.Publisher('control/out', ControlInput, queue_size=1)
        
        self.sub_pos = rospy.Subscriber('uav/position', PositionStates, self.handle_pos_state)
        self.sub_saf = rospy.Subscriber('aruco/saftey_states', SafetyStates, self.handle_saf_state)
        
        # setep time limits for merging states
        # setep max time delta object
        delta_ms = 250
        self.max_delta = rospy.Duration(nsecs=delta_ms*1000000)
        
        age_ms = 150 # 10 hz requirement to produce safe outputs
        self.max_age = rospy.Duration(nsecs=age_ms*1000000)
    
    def handle_des_control(self, data):
        
        # get the states and setup the u_out message
        state = self.get_states_vector()
        u_out = ControlInput()

        # form the u_des message into a vector
        u_des = np.array([data.x_b_dot, data.y_b_dot, data.z_b_dot, data.theta_b_dot])
        
        # if the states are not allowed to be used (time constraint breaks) then we send zeros / hover.
        if(state != None):
            alpha = 0.75
            # set alpha based on if h is positive
            if(self.h(state) <= 0.0):
                alpha = 3.0
			
            u_out_vec = self.cbf(state, u_des, alpha)
            
            u_out.x_b_dot = u_out_vec[0]
            u_out.y_b_dot = u_out_vec[1]
            u_out.z_b_dot = u_out_vec[2]
            u_out.theta_b_dot = u_out_vec[3]
        else:
            u_out.x_b_dot = 0
            u_out.y_b_dot = 0
            u_out.z_b_dot = 0
            u_out.theta_b_dot = 0
            
        #print(u_out)
            
        self.pub_control.publish(u_out)
    
    def handle_pos_state(self, data):
        # keep the latest message into a buffer
        self.last_pos_state = data
        
    def handle_saf_state(self, data):
        # keep the latest message into a buffer
        self.last_saf_state = data
        
    def get_states_vector(self):
        
        # check that the time stamps are within the reasonable range
        time_delta = abs(self.last_pos_state.header.stamp - self.last_saf_state.header.stamp)
        pos_age = rospy.Time.now() - self.last_pos_state.header.stamp
        saf_age = rospy.Time.now() - self.last_saf_state.header.stamp
        
        # if time contraints are met return state vector
        if(time_delta < self.max_delta and pos_age < self.max_age and saf_age < self.max_age):
            return [self.last_pos_state.x, self.last_pos_state.y, self.last_pos_state.z, self.last_pos_state.theta, self.last_saf_state.phi_h, self.last_saf_state.phi_v, self.last_saf_state.d]
        else:
            return None

    def cbf(self, state, u_des, alpha=0.75):

        # setup control output as a 4x1 variable
        u = cp.Variable(m)

        # form the objective function to minimise
        objective = cp.sum_squares(u - u_des)

        # form the constraint function
        constraint = [self.Lfh(state) + self.Lgh(state) @ u >= -alpha * self.h(state)]

        # Solve the problem
        prob = cp.Problem(cp.Minimize(objective), constraint)
        prob.solve()

        return u.value

    def Lfh(self, state):
        return 0

    def Lgh(self, state):
        val1 = 0
        val2 = (2 * state[4]) / (state[6] * pow(fov_h, 2))
        val3 = ((-2) * state[5]) / (state[6] * pow(fov_v, 2))
        val4 = (2 * state[4]) / (pow(fov_h, 2))
        
        return np.array([val1, val2, val3, val4])

    def h(self, state):
        return k - pow((state[4] / fov_h), 2) - pow((state[5] / fov_v), 2)

# MAIN FUNCTION

if(__name__ == '__main__'):
    
    cbf_class = ControlBarrier()
    
    rospy.spin()
