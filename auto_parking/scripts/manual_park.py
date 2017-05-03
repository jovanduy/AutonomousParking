#!/usr/bin/env python

""" This script generates parking guidelines for the Neato using opencv in ROS. """

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

LEFT_ENDPOINT_X = 237
RIGHT_ENDPOINT_X = 403
IMG_HEIGHT = 480
IMG_WIDTH = 640
IMG_SHAPE = (IMG_HEIGHT, IMG_WIDTH, 3)
HALF_WIDTH = IMG_WIDTH/2

class ParkingGuidelines(object):
    """ This class displays parking guidelines corresponding with the Neato's projected arc. """

    def __init__(self):
        """ Initialize the parking spot recognizer """
        
        #Subscribe to topics of interest
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        rospy.Subscriber('/cmd_vel', Twist, self.process_twist)

        # Initialize video images
        cv2.namedWindow('video_window')        
        self.cv_image = None # the latest image from the camera
        self.dst =  np.zeros(IMG_SHAPE, np.uint8)
        self.arc_image = np.zeros(IMG_SHAPE, np.uint8)
        self.transformed = np.zeros(IMG_SHAPE, np.uint8)
        
        # Declare instance variables
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV
        self.hsv_lb = np.array([0, 70, 60]) # hsv lower bound to filter for parking lines
        self.hsv_ub = np.array([30, 255, 140]) # hsv upper bound
        self.vel = None
        self.omega = None
        self.color = (0,0,255)  
                        
            
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.arc_image = np.zeros((480, 640, 3), np.uint8)
        self.draw_arc()

        # Transform the image of our arc from a top down image into the plane of our CV 
        self.transform_img()
        
        # overlay the projected path onto cv_image
        self.overlay_img()
        
        if self.omega is not None and self.omega == 0.0:
            self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            self.binary_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
            self.spot_delineators = self.find_delineators() 
            if self.color != (0, 255, 0): # This logic makes it such that once the lines turn green, they stay green
                self.color = (0,0,255) if not self.check_aligned() else (0,255,0)
          
    def transform_img(self):
        """ Transform the top-down image of the arc so that it lays flat in a plane on our cv_image """
        if self.vel is not None and self.omega is not None:
            pts1 = np.float32([[0,0], [0, IMG_HEIGHT], [IMG_WIDTH, IMG_HEIGHT], [IMG_WIDTH, 0]])
            pts2 = np.float32([[200,240], [0, IMG_HEIGHT], [IMG_WIDTH, IMG_HEIGHT], [400, 240]])
            M = cv2.getPerspectiveTransform(pts1, pts2)
            self.transformed = cv2.warpPerspective(self.arc_image, M, (self.cv_image.shape[0], self.cv_image.shape[1]))
            rows, cols, channels = self.transformed.shape
            self.transformed = self.transformed[0:IMG_HEIGHT, 0: cols]
            
            
    def overlay_img(self):
        """Overlay the transparent, transformed image of the arc onto our CV image"""
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
        

    def process_twist(self, twist):
        """ Process cmd_vel messages from ROS and stash them as instance variables."""
        self.vel = twist.linear.x
        self.omega = twist.angular.z
    
    
    def draw_arc(self): 
        """Draw the arc that corresponds with the Neato's current projected path, based on current vel and omega."""
        # If Neato is moving:
        if self.vel is not None and self.omega is not None:
            # If we are moving forward and turning:
            if self.omega != 0.0 and self.vel > 0.0:
                start_angle = 0 if self.omega > 0 else 180
                end_angle = start_angle + 80 if self.omega < 0 else start_angle - 80
                self.radius = (self.vel/self.omega)*500
                
                radius_len = int(abs(self.radius))
                
                #left wheel projection
                cv2.ellipse(img=self.arc_image, center=(int(HALF_WIDTH - self.radius - 150),IMG_HEIGHT), axes=(radius_len, radius_len),
                            angle=0, startAngle=start_angle, endAngle= end_angle, color=(0, 0, 255), thickness=8)
                
                #right wheel projection
                cv2.ellipse(img=self.arc_image, center=(int(HALF_WIDTH - self.radius + 150),IMG_HEIGHT), axes=(radius_len, radius_len),
                            angle=0, startAngle=start_angle, endAngle= end_angle, color=(0, 0, 255), thickness=8)

            # Otherwise we must be moving straight forwards
            else:
                if self.vel > 0.0:
                    #should draw line
                    cv2.line(self.arc_image,(170,IMG_HEIGHT),(170,200),self.color,6)
                    cv2.line(self.arc_image,(470,IMG_HEIGHT),(470,200),self.color,6)
    
    
    def check_aligned(self):
        '''Check if the neato is aligned with the spot. If so, return true.'''
        #if the neato is goting straight forward, check if the projected path of the two wheels are between the spot delineators.
        if self.spot_delineators is not None:
            return self.spot_delineators[0][0] <= LEFT_ENDPOINT_X and self.spot_delineators[1][0] >= RIGHT_ENDPOINT_X
        else: 
            return False

               
    def hough_lines(self):
       """ This function uses the Hough Line Transform function to identify and visualize lines in our binary image."""
       
       lines = cv2.HoughLinesP(self.binary_image, rho=5, theta=np.deg2rad(10), threshold=100, minLineLength=25, maxLineGap=0)
       lines_filtered = []
       if not lines is None:
            for x1,y1,x2,y2 in lines[0]:
                if y1 >100 and y2 > 100 and abs(y1 - y2) > 10:
                    # if the line is actually on the ground (not noise)
                    # and is more than 10 pixels vertically, include it
                    lines_filtered.append((x1,y1,x2,y2))
       return lines_filtered


    def find_delineators(self):
        """This function scans through our hough lines to determine the pixel coordinate endpoints of a parking spot."""
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
                cv2.waitKey(5)
            r.sleep()

            
if __name__ == '__main__':
    rospy.init_node('parking_guidelines')
    node = ParkingGuidelines()
    node.run()