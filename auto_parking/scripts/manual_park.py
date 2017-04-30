#!/usr/bin/env python

""" This script recognizes parking spots using opencv in ROS. """

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy
import math
from matplotlib import pyplot as plt

LEFT_ENDPOINT_X = 237
RIGHT_ENDPOINT_X = 403
class ParkingSpotRecognizer(object):
    """ This robot will recognize a parking spot. """


    def __init__(self):
        """ Initialize the parking spot recognizer """
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.process_camera)
        self.cv_image = None                        # the latest image from the camera
        self.dst =  np.zeros((480, 640, 3), np.uint8)
        self.arc_image = np.zeros((480, 640, 3), np.uint8)
        self.transformed = np.zeros((480, 640, 3), np.uint8)
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window')
        self.hsv_lb = np.array([0, 70, 60]) # hsv lower bound 
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber('/cmd_vel', Twist, self.process_twist)
        self.hsv_ub = np.array([30, 255, 140]) # hsv upper bound
        self.K = None
        self.dst = None
        self.is_spot_occupied = None
        self.occupied_checks = []
        self.start_time = rospy.Time.now()
        self.vel = None
        self.omega = None
        self.color = (0,0,255)
        # self.isImg = False
        
        
                
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.img_copy = deepcopy(self.cv_image)
        #self.calculate_arc()
        self.arc_image = np.zeros((480, 640, 3), np.uint8)
        self.draw_arc()

        if self.vel is not None and self.omega is not None:
            pts1 = np.float32([[0,0], [0, 480], [640, 480], [640, 0]  ])
            pts2 = np.float32([[200,240], [0, 480], [640, 480], [400, 240]])
            M = cv2.getPerspectiveTransform(pts1, pts2)
            self.transformed = cv2.warpPerspective(self.arc_image, M, (self.cv_image.shape[0], self.cv_image.shape[1]))
            rows, cols, channels = self.transformed.shape
            self.transformed = self.transformed[0:480, 0: cols]

        #overlay the arc on the image
        rows, cols, channels = self.transformed.shape
        roi = self.cv_image[0:rows, 0:cols]

        #change arc_image to grayscale
        arc2gray = cv2.cvtColor(self.transformed, cv2.COLOR_BGR2GRAY)
        ret, mask = cv2.threshold(arc2gray, 10, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        #black out area of arc in ROI
        img1_bg = cv2.bitwise_and(roi, roi, mask=mask_inv)
        img2_fg = cv2.bitwise_and(self.transformed, self.transformed, mask=mask)

        #put arc on ROI and modify the main image
        dst = cv2.add(img1_bg, img2_fg)
        self.cv_image[0:rows, 0:cols] = dst
        
        if self.omega is not None and self.omega == 0.0:
            self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            self.binary_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
            self.spot_delineators = self.find_delineators()
            self.color = (0,0,255) if not self.check_aligned() else (0,255,0)

                    
    def process_twist(self, twist):
        self.vel = twist.linear.x
        self.omega = twist.angular.z


    def calculate_arc(self):
        """ Compute the endpoint of of our current projected arc"""
        if self.vel and self.omega and self.omega != 0:
            self.radius = self.vel/self.omega
            #calculate where the real life endpoint of our arc is
            endpoint = np.array([[self.radius], [abs(self.radius)], [1]])
            
            #convert that endpoint to a pixel in our image
            pixel_x = np.matmul(np.asarray(self.K).reshape(3,3), endpoint)[0][0]
            pixel_y = 0.13 * self.fy / abs(self.radius) + self.cy
            
            pixel = (pixel_x, pixel_y)
 
        
            endpoint1 = (pixel[0] - 20, pixel[1])
            endpoint2 = (pixel[0] + 20, pixel[1])
        
            self.endpoints = (endpoint1, endpoint2)
        
    
    def draw_arc(self):       
        if self.vel is not None and self.omega is not None:
            if self.omega != 0.0 and self.vel > 0.0:
                start_angle = 0 if self.omega > 0 else 180
                end_angle = start_angle + 80 if self.omega < 0 else start_angle - 80
                self.radius = (self.vel/self.omega)*500
                
                #left wheel projection
                cv2.ellipse(img=self.arc_image, center=(int(320 - self.radius - 150),480), axes=(int(abs(self.radius)), int(abs(self.radius))),
                            angle=0, startAngle=start_angle, endAngle= end_angle, color=(0, 0, 255), thickness=8)
                
                #right wheel projection
                cv2.ellipse(img=self.arc_image, center=(int(320 - self.radius + 150),480), axes=(int(abs(self.radius)), int(abs(self.radius))),
                            angle=0, startAngle=start_angle, endAngle= end_angle, color=(0, 0, 255), thickness=8)

            else:
                if self.vel > 0.0:
                    #should draw line
                    cv2.line(self.arc_image,(170,480),(170,200),self.color,6)
                    cv2.line(self.arc_image,(470,480),(470,200),self.color,6)
    
    def check_aligned(self):
        if self.spot_delineators is not None:
            return self.spot_delineators[0][0] <= LEFT_ENDPOINT_X and self.spot_delineators[1][0] >= RIGHT_ENDPOINT_X
        else: 
            return False

    def process_camera(self, cameramsg):
        """ Callback method to store camera parameters from a 
            camera info message"""
        self.K = cameramsg.K
        self.fy = cameramsg.K[4]
        self.cx = cameramsg.K[2]
        self.cy = cameramsg.K[5]

               
    def hough_lines(self):
       """ This function uses the Hough Line Transform function to identify and visualize lines in our binary image."""
       
       lines = cv2.HoughLinesP(self.binary_image, rho=5, theta=np.deg2rad(10), threshold=100, minLineLength=25, maxLineGap=0)
       lines_filtered = []
       if not lines is None:
            for x1,y1,x2,y2 in lines[0]:
                if y1 >100 and y2 > 100 and abs(y1 - y2) > 10:
                    # if the line is actually on the ground (not noise)
                    # and is more than 10 pixels vertically, include it
                    # cv2.line(self.cv_image,(x1,y1),(x2,y2),(0,0,255),2)
                    lines_filtered.append((x1,y1,x2,y2))
       return lines_filtered

    def find_delineators(self):
        """This function uses the pinhole camera model to determine the endpoints of a parking spot in 3d space."""
        lines = self.hough_lines()
        if len(lines) < 3:
            return
        # sorting by left to right in the image
        lines.sort(key = lambda x: x[0])
        spot_delineator1 = lines[0]
        spot_delineator2 = -1

        # first line should correspond to the left-line (the left dilineator) of the leftmost spot
        endpoint1 = lines[0]
        endpoint2 = None
        leftmostx = lines[0][0]
        x_range = 120
        index = 1

        # find line with lowest y1 value corresponds to the leftmost possible x1
        # value for this dilineator so we know the "bottom" point of this dilineator
        while index < len(lines) and lines[index][0] - leftmostx < x_range :
            if spot_delineator1[1] < lines[index][1]:
                spot_delineator1 = lines[index]
            if endpoint1[3] > lines[index][3]:
                endpoint1 = lines[index]
            index += 1

        # assume that the next line with x1 more than x_range pixels away
        # from the left dilineator is part of the right dilineator of the same spot
        if index < len(lines):
            spot_delineator2 = lines[index]
            endpoint2 = lines[index]
            leftmostx = spot_delineator2[0]
            
            while index < len(lines) and lines[index][0] - leftmostx < x_range:
                if spot_delineator2[1] < lines[index][1]:
                    spot_delineator2 = lines[index]
                if endpoint2[3] > lines[index][3]:
                    endpoint2 = lines[index]
                index += 1

        if spot_delineator2 != -1:
            self.endpoint1 = endpoint1
            self.endpoint2 = endpoint2
            # cv2.line(self.cv_image,(spot_delineator1[0],spot_delineator1[1]),
            #     (spot_delineator1[2],spot_delineator1[3]),(0,255,0),2)
            # cv2.line(self.cv_image,(spot_delineator2[0],spot_delineator2[1]),
            #     (spot_delineator2[2],spot_delineator2[3]),(0,255,0),2)
            return [spot_delineator1, spot_delineator2]
        else:
            # all lines belong to the same cluster; a spot was not found
            return None

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window', self.cv_image)
                
                  # if not self.crop_img is None:
                #     cv2.imshow('crop_img', self.crop_img)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('parking_spot_recognizer')
    node = ParkingSpotRecognizer()
    node.run()
