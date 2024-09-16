#!/usr/bin/env python3

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

class ArmTracker:
    def __init__(self, detector):
        self.num_arms = 2
        self.needs_reset = [True] * self.num_arms
        self.tips = [None] * self.num_arms
        self.frames = [None] * self.num_arms
        self.width = 50
        self.centers = [None] * self.num_arms
        self.bboxes = [None] * self.num_arms
        self.trackers = [cv2.TrackerCSRT_create() for _ in range(self.num_arms)]
        self.tip_pub = rospy.Publisher("/tip", String, queue_size=10)
        self.lock = threading.Lock()
        self.lock_trackers = [threading.Lock() for _ in range(self.num_arms)]

    def track_arm(self, index, frame):
        ret = False

        if self.bboxes[index] is not None:
            with self.lock_trackers[index]:
                ret, bbox = self.trackers[index].update(frame)

            if ret:
                p1 = (int(self.bboxes[index][0]), int(self.bboxes[index][1]))
                p2 = (int(self.bboxes[index][0] + self.width), int(self.bboxes[index][1] + self.width))
                center = (int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2))

                with self.lock:
                    self.centers[index] = center
                    self.bboxes[index] = bbox
                    self.frames[index] = frame
            else:
                self.needs_reset[index] = True

        return ret

    def set_needs_reset(self, index, needs_reset):
        self.needs_reset[index] = needs_reset

    def reset_arm(self, index, frame, tip):
        if tip is None:
            rospy.logwarn(f"Tip for arm {index} was None")
            return

        bbox = (int(tip[0] - self.width / 2), int(tip[1] - self.width / 2), int(self.width), int(self.width))

        with self.lock_trackers[index]:
            self.trackers[index] = cv2.TrackerCSRT_create()
            self.trackers[index].init(frame, bbox)

        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        center = (int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2))

        with self.lock:
            self.centers[index] = center
            self.bboxes[index] = bbox
            self.frames[index] = frame
        self.needs_reset[index] = False

    def get_values(self, index):
        with self.lock:
            return self.centers[index], self.bboxes[index], self.frames[index]
