#!/usr/bin/env python

""" This is a script that autonomously parks the Neato either in parallel or perpendicular mode."""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
import math

"""Global Variables"""
LENGTH_OF_SPOT = 0.5 # The parking spots are half a meter long.
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
SPEED = 0.2
FORWARD = Twist(linear=Vector3(SPEED,0.0,0.0), angular=Vector3(0.0,0.0,0.0))


class ParkingNode(object):
    """Class for parking neato in either mode """
    def __init__(self):
        """Constructor for the class
        initialize topic subscription and 
        instance variables
        """
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # user chooses which parking mode to be performed
        self.is_parallel = rospy.get_param('~parallel', False)

        # Instance Variables
        self.timestamp1 = None
        self.timestamp2 = None
        self.dist2Neato = None
        self.dist2Wall = None
        self.widthOfSpot = None
        self.twist = None
        self.radius = None
        # Adjusment to be made before moving along the arc
        self.adjustment = 0 
        self.isAligned = False
        
        
    def process_scan(self, m):
        """ This function is the callback for our laser subscriber. """
        currDist = m.ranges[270]
        # check if the neato has passed the spot. If not, move forward
        if self.timestamp2 is None:
            self.twist = FORWARD
            # if thers is no distance to neato stored, store the lidar reading
            #  as distance to neato
            if self.dist2Neato is None:
                self.dist2Neato = currDist
                print "dist2Neato: ", self.dist2Neato
            # else check whether the current lidar reading is within the range of wall distance
            elif currDist > (self.dist2Neato + LENGTH_OF_SPOT - .05):
                # if so, update radius and set the first timestamp
                self.dist2Wall = currDist
                self.radius = (self.dist2Wall/2.0)
                if self.timestamp1 is None:
                    self.timestamp1 = rospy.Time.now()
                    print "TIME1: ", self.timestamp1
            # if the first timestamp has been set, check if the current lidar reading is
            # close enough to dis2neato, we assume the neato has passed the spot
            if abs(currDist - self.dist2Neato) <= 0.2 and self.timestamp1 is not None:
                self.timestamp2 = rospy.Time.now()
                print "TIME2: ",  self.timestamp2
                self.twist = STOP

        # If we have both timestamps, we can determine the width of the spot, and begin parking. 
        elif self.timestamp1 is not None and self.timestamp2 is not None:
            self.adjustment = 0.2 if self.is_parallel else -0.1
            self.widthOfSpot = SPEED * (self.timestamp2.secs - self.timestamp1.secs)
            if self.dist2Neato >= 0.3 and not self.isAligned: #determine later what the threshold should be
                self.align_with_origin()
                self.park()
                rospy.signal_shutdown("Done parking.")
            elif self.dist2Neato < 0.3:
                print "Neato was too close to park!"
               
       
    def stop(self):
        """This method publishes a twist to make the Neato stop."""
        self.publisher.publish(STOP)
        
    
    def align_with_origin(self):
        """After stopping next to the second parked Neato, 
        this function will align us properly so that we can successfully drive our circle."""

        # distance to move before drive the arc
        dist = self.radius - self.widthOfSpot/2.0 + self.adjustment
        now = rospy.Time.now()
        travelTime = dist/SPEED #dist/speed = time
        while rospy.Time.now() - now <= rospy.Duration(travelTime):
            self.twist = FORWARD
        self.twist = STOP
        self.isAligned = True

        
    def drive_arc(self, omega, travelTime, sign):
        '''given the omega, travel time and direction, drive in the corresponding arc'''
        # The third parameter (sign) represents whether the forward velocity of the twist will be positive or negative
        now = rospy.Time.now()
        while rospy.Time.now() - now <= travelTime:
            self.twist = Twist(linear=Vector3(sign*SPEED,0,0), angular=Vector3(0,0,omega))

                
    def park(self):
        '''Given the parking mode, plan the travel route'''
        if self.is_parallel:
            # first arc
            omega = SPEED / (self.radius + 0.25)
            travelTime = rospy.Duration(math.pi/2.5/omega)
            self.drive_arc(omega, travelTime, -1)
            # second arc

            omega = SPEED / self.radius
            travelTime = rospy.Duration(math.pi/3.0/omega - 0.2)
            self.drive_arc(-omega, travelTime, -1)
            
            # drive forward to realign
            omega = -0.4
            travelTime = rospy.Duration(1)
            self.drive_arc(omega, travelTime, 1)
            
        else:
            # drive into spot
            omega = SPEED / (self.radius + 0.15)
            travelTime = rospy.Duration(math.pi/2.0/omega)
            self.drive_arc(omega, travelTime, -1)
            
            # drive back to fully enter spot
            self.drive_arc(0, rospy.Duration(1), -1)
            
        self.twist = STOP

        
    def run(self):
        """ This function is the main run loop."""
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if self.twist:
                self.publisher.publish(self.twist)
            self.r.sleep()

            
if __name__ == '__main__':            
    rospy.init_node('parking')
    parking_node = ParkingNode()
    parking_node.run()
