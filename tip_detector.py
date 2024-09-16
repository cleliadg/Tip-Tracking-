#!/usr/bin/env python3
# this class detects the tips -

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
from std_msgs.msg import String
from geometry_msgs.msg import Point
from collections import deque
import collections 
import ast
import threading  
import time 
import statistics 
import threading

class TipDetector:
    def __init__(self):
        self.bridge = CvBridge()
        home_directory = os.path.expanduser("~")
        directory = os.path.join(home_directory)
        #filename = 'model_config_new.json'
        filename = 'model_config_clelia.json'
        file_path = os.path.join(directory, filename)
        with open(file_path) as json_file:
            json_config = json_file.read()

        self.model = tf.keras.models.model_from_json(json_config)
        #self.model.load_weights(os.path.join(directory, 'weights_new.h5'))
        self.model.load_weights(os.path.join(directory, 'weights_clelia.h5'))
        self.leftmost_tip = None 
        self.rightmost_tip = None 
        self.contour_left = None 
        self.contour_right = None 



    def process_image(self, cv_image):
        resized = cv2.resize(cv_image, (400, 400))
        resized = np.expand_dims(resized, axis=0)
        mask = self.model.predict(resized, verbose= False)
        mask_resized = cv2.resize(mask[0], (400, 400))
        mask_binary = (mask_resized < 0.5).astype(np.uint8)
        mask = (mask_resized > 0.5).astype(np.uint8)
        contours, _ = cv2.findContours(mask_binary, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        return contours, mask

    def find_tips(self, contours, cv_image):
        edge_threshold = 30
        leftmost_tip = None
        rightmost_tip = None
        contour_left = None 
        contour_right = None 


        for contour in contours:
            hull = cv2.convexHull(contour)
            M = cv2.moments(hull)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = None, None
                continue 

            distances = np.sqrt((hull[:,:,0] - cX)**2 + (hull[:,:,1] - cY)**2)
            max_distance = 0
            tip = None

            for i in range(len(distances)):
                point = tuple(hull[i][0])
                if point[0] < edge_threshold or point[0] > cv_image.shape[1] - edge_threshold or \
                    point[1] < edge_threshold or point[1] > cv_image.shape[0] - edge_threshold:
                    continue

                if distances[i] > max_distance:
                    max_distance = distances[i]
                    tip = point

            if tip is not None:
                if cX > cv_image.shape[0] / 2:
                    if rightmost_tip is None or tip[0] < rightmost_tip[0]:
                        rightmost_tip = tip
                       
                else:
                    if leftmost_tip is None or tip[0] > leftmost_tip[0]:
                        leftmost_tip = tip
                      

        return leftmost_tip, rightmost_tip
    #, contour_left, contour_right
    


    def process_tips(self,frame): 
            contours, mask = self.process_image(frame)
            self.leftmost_tip, self.rightmost_tip = self.find_tips(contours, frame)
            return self.leftmost_tip, self.rightmost_tip, mask

