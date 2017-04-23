#!/usr/bin/env python

""" This is a script that autonomously parks the Neato."""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import LaserScan
import math

"""Global Variables"""
LENGTH_OF_SPOT = 0.5 # The parking spots are half a meter long.
STOP = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
SPEED = 0.2
FORWARD_SLOW = Twist(linear=Vector3(SPEED,0.0,0.0), angular=Vector3(0.0,0.0,0.0))


class ParkingNode(object):
    def __init__(self):
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.is_parallel = rospy.get_param('parallel', False)
        self.timestamp1 = None
        self.timestamp2 = None
        self.dist2Neato = None
        self.dist2Wall = None
        self.widthOfSpot = None
        self.twist = None
        self.radius = None
        self.adjustment = 0
        self.isAligned = False
        
        
    def process_scan(self, m):
        """ This function is the callback for our laser subscriber. """
        self.ranges = m.ranges
        if self.timestamp2 is None:
            self.twist = FORWARD_SLOW
            if self.dist2Neato is None:
                self.dist2Neato = self.ranges[270]
                self.adjustment = 0.2 if self.is_parallel else -0.1
                print self.dist2Neato
            elif self.ranges[270] > (self.dist2Neato + LENGTH_OF_SPOT - .05):
                self.dist2Wall = self.ranges[270]
                self.radius = (self.dist2Wall/2.0)
                if self.timestamp1 is None:
                    self.timestamp1 = rospy.Time.now()
                    print "TIME1: ", self.timestamp1
            if abs(self.ranges[270] - self.dist2Neato) <= 0.2 and self.timestamp1 is not None:
                self.timestamp2 = rospy.Time.now()
                print "TIME2: ",  self.timestamp2
                self.twist = STOP

        elif self.timestamp1 is not None and self.timestamp2 is not None:
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
        """After stopping next to the second parked Neato, this function will align us properly so that we can successfully drive our circle."""
        dist = self.radius - self.widthOfSpot/2.0 + self.adjustment
        now = rospy.Time.now()
        travelTime = dist/SPEED #dist/speed = time
        while rospy.Time.now() - now <= rospy.Duration(travelTime):
            self.twist = FORWARD_SLOW
        self.twist = STOP
        self.isAligned = True

    def drive_arc(self, omega, travelTime, forward=False):
        if forward:
            now = rospy.Time.now()
            while rospy.Time.now() - now <= travelTime:
                self.twist = Twist(linear=Vector3(SPEED,0,0), angular=Vector3(0,0,omega))
        else:
            now = rospy.Time.now()
            while rospy.Time.now() - now <= travelTime:
                self.twist = Twist(linear=Vector3(-SPEED,0,0), angular=Vector3(0,0,omega))

    def park(self):
        if self.is_parallel:
            # first arc
            omega = SPEED / (self.radius + 0.25)
            travelTime = rospy.Duration(math.pi/2.5/omega)
            drive_arc(omega, travelTime)
            # second arc
            omega = -SPEED/self.radius
            travelTime = rospy.Duration(math.pi/3.0/omega - 0.2)
            drive_arc(omega, travelTime)
            # drive forward to realign
            omega = -0.4
            travelTime = rospy.Duration(1)
            drive_arc(omega, travelTime, True)
        else:
            omega = SPEED / (self.radius + 0.15)
            travelTime = rospy.Duration(math.pi/2.0/omega)
            # drive into spot
            drive_arc(omega, travelTime)
            # drive back to fully enter spot
            drive_arc(0, rospy.Duration(1))
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
