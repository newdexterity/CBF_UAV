#!/usr/bin/python3

import cv2
import numpy as np

from picamera2 import Picamera2, Preview
import libcamera
import math

import time
import pickle

from threading import Thread
import paho.mqtt.client as mqtt

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as Rot

# System gains (must match cbf)
fov_h = 0.4*2 # from testing
fov_v = 0.3*2 # from testing
k = pow(0.65, 2) # middle 65%

log_num = 0

fullscreen = False

# dir to save still
still_dir = "/home/angus/ros_drone/aruco_server/stills/"

def h(phi_h, phi_v):
    return k - pow((phi_h / (fov_h / 2)), 2) - pow((phi_v / (fov_v / 2)), 2)
        

# MQTT connect function
def on_connect(client, userdata, flags, reason_code):
	print(f"connected to mqtt broker: {userdata}, {flags}, {reason_code}")
	client.subscribe("log/num")
    
def on_message(self, client, mqtt_msg):
    if(mqtt_msg.topic == "log/num"):
        global log_num
        log_num = int(mqtt_msg.payload)

# class to run opencv preview in its own thread

class SlowWindow(object):
    def __init__(self, name):
        self.name = name
        
        self.run = True
        
        self.frame = np.zeros((480, 640, 3))

    def stop(self):
        self.run = False
        
    def setFrame(self, frame):
        self.frame = frame
        
    def main(self):
        while(self.run):
            if(fullscreen):
                cv2.namedWindow(self.name, cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty(self.name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow(self.name, self.frame)
            
            # react to keys on window
            k = cv2.waitKey(5)
            if(k > 0):
                print(k)
                if(k == 113): # lower case q
                    self.stop()
                    cv2.destroyAllWindows()
        
class FilteredMarker():
    # each accepted marker 
    def __init__(self, mark_id):
        self.mark_id = mark_id
        
        self.last_corners = np.zeros([1, 4, 2])
        self.last_corners_time = 0
        
    def id(self):
        return self.mark_id
        
    def __str__(self):
        return f"Filter Object for Aruco: {self.mark_id}"
    
    def measure(self, corners):
        
        # see the sum of the differnece between this corners incoming and the one we have
        # only update if the new one is substantially differenet. Trying to make a "dead zone"
        # and also I don't care about losing the mm precision.
        
        tl_delta = abs(self.last_corners[0][0][0] - corners[0][0][0]) + abs(self.last_corners[0][0][1] - corners[0][0][1]) 
        tr_delta = abs(self.last_corners[0][1][0] - corners[0][1][0]) + abs(self.last_corners[0][1][1] - corners[0][1][1]) 
        br_delta = abs(self.last_corners[0][2][0] - corners[0][2][0]) + abs(self.last_corners[0][2][1] - corners[0][2][1]) 
        bl_delta = abs(self.last_corners[0][3][0] - corners[0][3][0]) + abs(self.last_corners[0][3][1] - corners[0][3][1]) 
        
        #print(f"{tl_delta}, {tr_delta}, {br_delta}, {bl_delta}")
        
        sum_delta = tl_delta + tr_delta + br_delta + bl_delta
        
        #print(sum_delta)
        
        if(sum_delta > 6):
            self.last_corners = corners
        
        self.last_corners_time = time.time()
        
    def last_seen(self):
        if(self.last_corners_time > 0):
            return time.time() - self.last_corners_time
        else:
            return None
    
    # implement filter here 
    def get(self):
        # only persist up to half a second
        if(self.last_seen() is not None and self.last_seen() < 0.1):
            return self.last_corners
        else:
            return None

    ## PROGRAM STARTS HERE ##
if(__name__ == '__main__'):
    # Create the pi camera object
    picam2 = Picamera2()

    # Used for raw preview window
    #picam2.start_preview(Preview.QTGL)

    # Setup the configuration, and rotate the image 180 deg
    #preview_config = picam2.create_preview_configuration(main={"size": (1456, 1088), "format": "RGB888"})
    preview_config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})

    # flip images
    #preview_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
    
    # list of accepted markers
    allowed_markers = [65, 66, 67, 68, 78, 4, 3]
    
    # make dict of filters
    markers_filtered = {}
    
    for m_id in allowed_markers:
        markers_filtered[m_id] = FilteredMarker(m_id)
        
    marker_pairs = [[67, 68], [65, 66]]
        
    #print(str(markers_filtered))

    # apply configs
    picam2.configure(preview_config)

    # Setup aruco library
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    parameters =  cv2.aruco.DetectorParameters_create()

    # Setup properties for annotating markers
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1.0
    colour = (255,0,0)
    thickness = 2

    # intrinsic values of the camera 
    
    # RUNNING AT 640 X 480
    mtx = np.array([789.44435169, 0, 310.70260044, 0, 788.68401899, 251.17006037, 0, 0, 1]).reshape((3,3))
    dist = np.array([-5.10062729e-01,  4.29630227e-01, -3.75500265e-04, -1.57077276e-03, -4.87915683e-01])
    
    # RUNNING AT 1456 X 1088
    #mtx = np.array([1782.7613651041636, 0.0, 711.7926582787674, 0.0, 1782.5157056773135, 573.4375063225476,0.0, 0.0, 1.0]).reshape((3,3))
    #dist = np.array([-0.500253929315102, 0.32637082045964705, -0.0004371323063115204, -0.0018514733623917906, -0.1498391951594985])
    
    # Property of marker
    marker_length = 0.1 #m

    # Setup a slow window thread to display the output
    window = SlowWindow("Camera")
    window_thread = Thread(target=window.main)
    window_thread.start()
    
    
    # Start the mqtt thread
    mqttc = mqtt.Client(client_id="pub", clean_session=False, userdata=None)
    mqttc.on_connect = on_connect
    mqttc.on_message = on_message
    mqttc.connect("localhost", 1883, 60)

    mqtt_thread = Thread(target=mqttc.loop_forever)
    mqtt_thread.start()

    # Start camera stream
    picam2.start()
    
    # for saving to file
    last_save = time.time()
    freq_save = 5
    
    counter = 0
    
    # Setup  Filter:
    
    # for calculating fps
    last_time = time.time()
    freq_fps_report = 5 # how often to print fps to console

    # Used to divide how often frames are written to display
    frames_since_update = 0
    
    frame_count = 0

    # Run the loop until sigint
    try:
        while window.run:
            # for printout
            is_marker_found = False
            
            
            # Read latest frame
            im = picam2.capture_array()
            
            frame_count = frame_count + 1
            frames_since_update = frames_since_update + 1
            
            
            # Detect markers
            (corners, ids, rejected) = cv2.aruco.detectMarkers(im, dictionary, parameters=parameters)
                        
            # if any markers are found
            if(len(corners) > 0):
                # loop through all markers
                for i in range(0, len(ids)):
                    # target marker is in the list of allowed markers
                    if(ids[i] in allowed_markers):
                        markers_filtered[int(ids[i])].measure(corners[i])
                        
                        # draw bounding polygon based on corners
                        colour = (0, 255, 0)
                        thickness = 2
                        im = cv2.line(im, (int(corners[i][0][0][0]), int(corners[i][0][0][1])), (int(corners[i][0][1][0]), int(corners[i][0][1][1])), colour, thickness)
                        im = cv2.line(im, (int(corners[i][0][1][0]), int(corners[i][0][1][1])), (int(corners[i][0][2][0]), int(corners[i][0][2][1])), colour, thickness)
                        im = cv2.line(im, (int(corners[i][0][2][0]), int(corners[i][0][2][1])), (int(corners[i][0][3][0]), int(corners[i][0][3][1])), colour, thickness)
                        im = cv2.line(im, (int(corners[i][0][3][0]), int(corners[i][0][3][1])), (int(corners[i][0][0][0]), int(corners[i][0][0][1])), colour, thickness)
            
            # Process the marker selection
            phi_h_best = 0
            phi_v_best = 0
            d_best = 0
                
            safe_score = -1000
            safe_id = 0
                        
            # run the grouped marker detection
            
            mark_size = 0.1
            mark_offset = 0.035
            mark_rot = math.radians(15)
            
            for pair in marker_pairs:
                corners_left = markers_filtered[pair[0]].get()
                corners_right = markers_filtered[pair[1]].get()
                                
                if(corners_left is not None and corners_right is not None):
                    
                    corners = np.array([corners_left[0][0], corners_left[0][1],
                                        corners_right[0][0], corners_right[0][1], 
                                        corners_right[0][2], corners_right[0][3],
                                        corners_left[0][2], corners_left[0][3]])
                                                                       
                    world_points = np.array([[ -(math.cos(mark_rot) * (mark_size + mark_offset)),  mark_size/2, -(math.sin(mark_rot) * (mark_size + mark_offset))],
                                             [ -(math.cos(mark_rot) * (mark_offset)),              mark_size/2, -(math.sin(mark_rot) * (mark_offset))],
                                             [  (math.cos(mark_rot) * (mark_offset)),              mark_size/2, -(math.sin(mark_rot) * (mark_offset))],
                                             [  (math.cos(mark_rot) * (mark_size + mark_offset)),  mark_size/2, -(math.sin(mark_rot) * (mark_size + mark_offset))],
                                             [  (math.cos(mark_rot) * (mark_size + mark_offset)), -mark_size/2, -(math.sin(mark_rot) * (mark_size + mark_offset))],
                                             [  (math.cos(mark_rot) * (mark_offset)),             -mark_size/2, -(math.sin(mark_rot) * (mark_offset))],
                                             [ -(math.cos(mark_rot) * (mark_offset)),             -mark_size/2, -(math.sin(mark_rot) * (mark_offset))],
                                             [ -(math.cos(mark_rot) * (mark_size + mark_offset)), -mark_size/2, -(math.sin(mark_rot) * (mark_size + mark_offset))]])
                                             
                    print(np.shape(corners))
                    print(np.shape(world_points))
                    print(" ")
                    
                    					
                    rectval, rvec, tvec = cv2.solvePnP(world_points, corners, mtx, dist)
                    
                    #rvec = np.array([[-2.77691744], [ 1.15311676], [-0.85460873]])
                    #tvec = np.array([[ 0.07873795], [-0.06060775], [ 0.08275515]])
                    
                    
                    if(tvec[2] > 0.2):
                        cv2.drawFrameAxes(im, mtx, dist, rvec, tvec, marker_length)
                                            
                        corners_x = corners[0].T[0]
                        corners_y = corners[0].T[1]
                        
                                
                        center_x = np.min(corners_x) + ((np.max(corners_x) - np.min(corners_x)) / 2.0)
                        center_y = np.min(corners_y) + ((np.max(corners_y) - np.min(corners_y)) / 2.0)
                        
                                
                        norm_x = (center_x - (im.shape[1] / 2)) / (im.shape[1] / 2)
                        norm_y = (center_y - (im.shape[0] / 2)) / (im.shape[0] / 2)

                    else:
                        pass
                    
                                                
                            # get saftey states
                    try:
                        phi_h = math.asin(tvec[0]/tvec[2])
                        phi_v = -math.asin(tvec[1]/tvec[2])
                        d = float(tvec[2])
                        
                        #print(f"{phi_h} {phi_v} {d}")
                        
                        # calc comapre to safest
                        saftey = h(phi_h, phi_v)
                        if(saftey > safe_score):
                            phi_h_best = phi_h
                            phi_v_best = phi_v
                            d_best = d
                            safe_score = saftey
                            safe_id = pair[0]
                            
                        is_marker_found = True
                        
                            
                        #print(pair[0])
                                                        
                        # make rotation matrix
                        rot_mat = np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
                        rot_mat[:3, :3], _ = cv2.Rodrigues(rvec)
                        
                                                
                        packet = {'id': pair[0], 'rot_mat': rot_mat, 'tvec': tvec}
                        packet_serialised = pickle.dumps(packet)
                        mqttc.publish("aruco/marker", packet_serialised)
                    except Exception as e:
                        pass
                        #print(e)
                                    
            # publish score if a marker is found
            if(is_marker_found):
                packet = {'phi_h': phi_h_best, 'phi_v': phi_v_best, 'd': d_best}
                                
                packet_serialised = pickle.dumps(packet)
                mqttc.publish("aruco/safety", packet_serialised)
                
                            
                # annotate the frame before sending to window
            
            mqttc.publish("aruco/frame", str(frame_count))
            
            # store a frame every now and then
            if((time.time() - last_save) > freq_save):
                if(log_num > 0):
                    cv2.imwrite(still_dir + "exp" + str(log_num) + "img" + str(frame_count) + ".jpg", im)
                    last_save = time.time()

            
            black = np.zeros((520, 640, 3), np.uint8)
            black[0:480,0:640, 0:3] = cv2.resize(im, (640,480))
        
            out_im = cv2.putText(black, "exp: " + str(log_num), (10, 510), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            out_im = cv2.putText(out_im, "frame: " + str(frame_count), (320, 510), cv2.FONT_HERSHEY_SIMPLEX,1, (0, 0, 255), 2, cv2.LINE_AA)
            
            
            window.setFrame(out_im)
            #window.setFrame(im)
                
            if((time.time() - last_time) > freq_fps_report):
                # print fps
                fps = frames_since_update / (time.time() - last_time)
                last_time = time.time()
                frames_since_update = 0
            
                print(f"FPS: {fps:.0f} ")
 
    # If the program ends join the window loop
    finally:
        print("Ending window thread...")
        window.stop()
        mqttc.disconnect()
        
        window_thread.join()

