#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
# print("\n\n\n\n\n\n tf version ", tf.__version__)
from std_msgs.msg import String
from geometry_msgs.msg import Point,PointStamped
from collections import deque
import collections 
import ast
import threading  
import time 
import statistics 
import threading
from loftr import LoFTRMatcher
from arm_tracker import ArmTracker
from tip_detector import TipDetector
from std_msgs.msg import Float32, Bool

class MainTracker:
    def __init__(self):
        self.mode = None 
        self.mode = 'distance_c'
        rospy.init_node('main_tracker_node')
        self.detector = TipDetector()
        self.tracker = ArmTracker(self.detector)
        self.loftr = LoFTRMatcher()
        self.frame = None
        self.otherframe = None
        self.tracker1 = cv2.TrackerCSRT_create()
        self.tracker2 = cv2.TrackerCSRT_create()
        self.bridge = CvBridge()
        self.arm_choice = "right"
        self.lock = threading.Lock()
        self.tip = None
        self.width = 50
        self.image_pub1 = rospy.Publisher("/image_tracked", Image, queue_size=10)
        self.image_pub2 = rospy.Publisher("/other_image_tracked", Image, queue_size=10)
        self.center_pub1 = rospy.Publisher("/lobsterscope/current_tip", Point, queue_size=10)
        self.mask0_pub = rospy.Publisher("/mask0", Image, queue_size=10)
        self.mask1_pub = rospy.Publisher("/mask1", Image, queue_size=10)
        self.deque_depth = collections.deque(maxlen=5)
        self.deque_depth2 = collections.deque(maxlen=5)
        self.center_pub2 = rospy.Publisher("/lobsterscope/tip_in_other_frame", Point, queue_size=10)

        #NEW STUFF 
        self.image_empty_pub = rospy.Publisher("/image_empty", Image, queue_size=10)
        self.other_image_empty_pub = rospy.Publisher("/other_image_empty", Image, queue_size=10)

        #modality of depth
        self.depth_from_the_background = None 
        self.control_sub = rospy.Subscriber("depth_mode", String, self.depth_cb)
        self.depth_from_the_background_sub= rospy.Subscriber("/depth_from_background", Float32, self.depth_fromb_cb)
        self.arm_sub = rospy.Subscriber("arm_choice", String, self.arm_cb)
        self.image_sub = rospy.Subscriber("/my_stereo/left/image_rect_color", Image, self.image_callback)
        self.otherimage_sub = rospy.Subscriber("/my_stereo/right/image_rect_color", Image, self.other_image_callback)

        #NEW STUFF 
        self.get_fixed_tip_activated_sub  = rospy.Subscriber("/fixed_tip", Bool, self.fixed_tip_callback)
        self.got_tip = True
        self.fixed_tip = False 
        self.tracker_tip = None 
        self.tracker_pub = rospy.Publisher("/visual_servoing/target", PointStamped,queue_size=1)
        self.timer_fixed_tip = None

       
        self.depth_from_the_background = None 
        self.detector_timer = rospy.Timer(rospy.Duration(0.20), self.call_detector)
        #self.tracker_timer = rospy.Timer(rospy.Duration(0.1), self.call_tracker)
        self.tracker_timer = rospy.Timer(rospy.Duration(0.05), self.call_tracker)

        #to filter out the tracking 
        self.deque_center1 = collections.deque(maxlen=5)
        self.deque_center2 = collections.deque(maxlen=5)


    ##NEW STUFF 
    def fixed_tip_callback(self, msg):
        if (msg.data):
            self.fixed_tip = True
            self.got_tip = False 
            self.tracker_tip = None 
            print("we are looking for the tip")
            self.timer_fixed_tip = None


        else:
            self.fixed_tip = False
            self.got_tip = False
            self.tracker_tip = None 
            print("we are stopping for the tip")
            self.timer_fixed_tip = None
            




    def depth_fromb_cb(self,msg):
        self.depth_from_the_background = msg.data

    

    def arm_cb(self,msg):
        rospy.loginfo("new message received")
        if msg.data == "left":
            rospy.loginfo("Starting left control.")
            self.arm_choice = "left"
            ###
            #for filter
            self.deque_center1.clear()
            self.deque_center2.clear()
            #in general
            frame = self.frame.copy()
            otherframe = self.otherframe.copy()
            tip1, right, mask0 = self.detector.process_tips(frame)
            tip2, right, mask1  = self.detector.process_tips(otherframe)
            self.tracker.reset_arm(0, frame, tip1)
            self.tracker.reset_arm(1, otherframe, tip2)



        
        elif msg.data == "right":
            rospy.loginfo("Stopping right control.")
            self.arm_choice = "right"
            ##reset the tip
            #for filter
            self.deque_center1.clear()
            self.deque_center2.clear()
            #in general 
            frame = self.frame.copy()
            otherframe = self.otherframe.copy()
            left, tip1, mask0 = self.detector.process_tips(frame)
            left, tip2, mask1  = self.detector.process_tips(otherframe)
            self.tracker.reset_arm(0, frame, tip1)
            self.tracker.reset_arm(1, otherframe, tip2)

        


    def depth_cb(self, msg):
        if msg.data == "depth_from_camera":
            rospy.loginfo("depth from camera selected")
            self.mode = 'distance_c'
        elif msg.data == "depth_from_background":
            rospy.loginfo("depth from background selected")
            self.mode = 'distance_b'



    def image_callback(self, data):
        #they got inverted so invert it here 
        if self.arm_choice == 'right':
            try:
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

            except CvBridgeError as e:
                print(e)

        elif self.arm_choice == 'left':
            try:
                self.otherframe = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

    def other_image_callback(self, data):
        if self.arm_choice == 'left':
            try:
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

        elif self.arm_choice == 'right':
            try:
                self.otherframe = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

    #NEW STUFF
    def fixed_tip_timer_callback(self,event):
        if self.tracker_tip is not None:
            tracker_tip = PointStamped()
            tracker_tip.point.x = self.tracker_tip[0]
            tracker_tip.point.y = self.tracker_tip[1]
            self.publish(tracker_tip)

    def publish(self,tracker_tip):
        self.tracker_pub.publish(tracker_tip)




    def compute_depth(self, center1 ,center2 , frame,otherframe):

        d = None
        if center1 is not None and center2 is not None:

            M = 2.01707617    
            if (center1[0] - center2[0]==0):
                    return None 
  
            d = abs(M / (center1[0] - center2[0]))*100
            if d is not None:
                self.deque_depth.append(d) 
                if len(self.deque_depth) == 5:  # If the deque is full

                    median_value_left = statistics.median(self.deque_depth)  
                    d = median_value_left

            if self.mode == 'distance_c':
                    return d 

            elif self.mode == "distance_b":
                # print("we are here")
                depth_from_the_background = self.depth_from_the_background
                # print("depth deptgh",depth_from_the_background )

                if depth_from_the_background is not  None: 
                    print("test1")
                    # self.deque_depth2.append(depth_from_the_background) 
                    final_d = depth_from_the_background-d-0.3

                    # if len(self.deque_depth2) == 5:  # If the deque is full
                    #     median_value_left = statistics.median(self.deque_depth2)  
                    #     final_d = median_value_left - d - 0.2
 
                    return (final_d)
                else: 
                    return None 
                    print("test 2")



        else: 
            return None 
            print("testb 3")



    

    def call_detector(self, event):
        if self.frame is not None and self.otherframe is not None:
            frame = self.frame.copy()
            otherframe = self.otherframe.copy()

            # Detect tips on both frames
            #maybe not neeeded 
            mask0 = None 
            mask1 = None 
            if self.arm_choice == 'right':
                left, tip1, mask0 = self.detector.process_tips(frame)
                left, tip2, mask1  = self.detector.process_tips(otherframe)
                



            if self.arm_choice == 'left':
                tip1, right, mask0 = self.detector.process_tips(frame)
                tip2, right, mask1  = self.detector.process_tips(otherframe)
 
            if mask0 is not None and mask1 is not None:
                mask0_msg = self.bridge.cv2_to_imgmsg(mask0, encoding="mono8")
                mask1_msg = self.bridge.cv2_to_imgmsg(mask1, encoding="mono8")
                self.mask0_pub.publish(mask0_msg)
                self.mask1_pub.publish(mask1_msg)

            # Check if tips were detected
            if not tip1 or tip1 is None:
                self.tracker.set_needs_reset(True, 1)  # Notify tracker to reset for left arm
                rospy.logwarn("Tip on left frame outside of the FOV")

            if not tip2 or tip2 is None:
                self.tracker.set_needs_reset(True, 2)  # Notify tracker to reset for right arm
                rospy.logwarn("Tip on right frame outside of the FOV")

            # Reset tracker if needed
            if self.tracker.needs_reset[0]:
                self.tracker.reset_arm(0, frame, tip1)

            if self.tracker.needs_reset[1]:
                self.tracker.reset_arm(1, otherframe, tip2)

            # Get current values from tracker
            (center1, bbox1, tracked_frame1), (center2, bbox2, tracked_frame2) = self.tracker.get_values(0),self.tracker.get_values(1)
             #NEW PART --CHECK

            # self.depth_from_the_background = self.loftr.depth_from_the_background(frame, otherframe,mask0,mask1)


            # Check if tips are within bounding boxes, if not reset
            if bbox1 is not None and tip1 is not None:

                if abs(center1[0]- tip1[0]) >100 or  abs(center1[1]- tip1[1]) >100:
                    center1 = tip1
                    bbox1 = (int(tip1[0] - self.width / 2), int(tip1[1] - self.width / 2), int(self.width), int(self.width))



                if (tip1[1] < bbox1[1] or tip1[0] < bbox1[0] or
                    tip1[1] > bbox1[1] + bbox1[3] or tip1[0] > bbox1[0] + bbox1[2]):
                    rospy.logwarn("Left arm tracker lost")
                    self.tracker.reset_arm(0, frame, tip1)

            if bbox2 is not None and tip2 is not None:

                if abs(center2[0]- tip2[0]) >100 or  abs(center2[1]- tip2[1]) >100:
                    center2 = tip2
                    bbox2 = (int(tip2[0] - self.width / 2), int(tip2[1] - self.width / 2), int(self.width), int(self.width))

                if (tip2[1] < bbox2[1] or tip2[0] < bbox2[0] or
                    tip2[1] > bbox2[1] + bbox2[3] or tip2[0] > bbox2[0] + bbox2[2]):
                    rospy.logwarn("Right arm tracker lost")
                    self.tracker.reset_arm(1, otherframe, tip2)

            old_tip1 = tip1
            oldtip2 = tip2
            with self.lock:
                self.tip = tip1  

    def call_tracker(self, event):
        if self.frame is not None and self.otherframe is not None:
            frame = self.frame.copy()
            otherframe = self.otherframe.copy()

            # Track arms in both frames
            success1 = self.tracker.track_arm(0, frame)
            success2 = self.tracker.track_arm(1, otherframe)

            # Publish tracked image and center points
            try:
                (center1, bbox1, tracked_frame1), (center2, bbox2, tracked_frame2) = self.tracker.get_values(0),self.tracker.get_values(1)
               
                if center1 is not None:
                    #filter centers -- comment if not needed 
                    self.deque_center1.append(center1) 
                    if len(self.deque_center1) == 5:  # If the deque is full
                        median_center1 = statistics.median(self.deque_center1)  
                        center1 = median_center1

                if center2 is not None:
                    self.deque_center2.append(center2) 
                    if len(self.deque_center2) == 5:  # If the deque is full
                        median_center2 = statistics.median(self.deque_center2)  
                        center2 = median_center2





                if center1 is not None and center2 is not None: 
                    d= self.compute_depth(center1, center2,frame,otherframe)
                else:
                    d = None 
                #print(d,"d")
                 
                #NEW STUFF 
                if self.got_tip is False and self.fixed_tip is True and center1 is not None: 
                    self.tracker_tip = center1 
                    self.got_tip = True

                if self.tracker_tip is not None and self.fixed_tip is True and self.got_tip is True:

                    self.timer_fixed_tip = rospy.Timer(rospy.Duration(1.0/30.0),self.fixed_tip_timer_callback)

                    

                #print("self.depth_from_the_background",self.depth_from_the_background)
                #NEW STUFF 
                

                if center1 is not None and d is not None:
                    center_point1 = Point()
                    center_point1.x = center1[0]
                    center_point1.y = center1[1]
                    center_point1.z = d
                    self.center_pub1.publish(center_point1)

                if center2 is not None and d is not None:
                    center_point2 = Point()
                    center_point2.x = center2[0]
                    center_point2.y = center2[1]
                    center_point2.z = d
                    self.center_pub2.publish(center_point2)

                if tracked_frame1 is not None:
                    #NEW STUFF 
                    self.image_empty_pub.publish(self.bridge.cv2_to_imgmsg(tracked_frame1, "bgr8"))

                    cv2.circle(tracked_frame1, (center1[0], center1[1]), 2, (255, 255, 0), -1)
                    box_top_left = (center1[0] - 25, center1[1] - 25)  # 25 pixels offset for a 50x50 box
                    cv2.rectangle(tracked_frame1, box_top_left, (center1[0] + 25, center1[1] + 25), (255, 255, 0), 1)
                    self.image_pub1.publish(self.bridge.cv2_to_imgmsg(tracked_frame1, "bgr8"))

                if tracked_frame2 is not None:

                    #NEW STUFF 
                    self.other_image_empty_pub.publish(self.bridge.cv2_to_imgmsg(tracked_frame2, "bgr8"))

                    cv2.circle(tracked_frame2, (center2[0], center2[1]), 2, (255, 255, 0), -1)
                    box_top_left = (center2[0] - 25, center2[1] - 25)
                    cv2.rectangle(tracked_frame2, box_top_left, (center2[0] + 25, center2[1] + 25), (255, 255, 0), 1)
                    self.image_pub2.publish(self.bridge.cv2_to_imgmsg(tracked_frame2, "bgr8"))

            except CvBridgeError as e:
                print(e)

if __name__ == "__main__":
    main_tracker = MainTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()