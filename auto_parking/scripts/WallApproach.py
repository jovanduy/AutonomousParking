#!/usr/bin/env python
""" This is a ROS node that approaches a wall using proportional control """

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class WallApproach(object):
    def __init__(self):
        rospy.init_node('wall_approach')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def stop(self):
        # stop the neato
        self.pub.publish(Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0)))

    def process_scan(self, scan):
        print scan.ranges[90] 

    def run(self):
        r = rospy.Rate(5)
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            self.pub.publish(Twist(linear=Vector3(-0.1,0,0), angular=Vector3(0,0,-0.2)))
            r.sleep()

if __name__ == '__main__':
    node = WallApproach()
    node.run()