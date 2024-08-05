#! /usr/bin/python3

import math
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class rangefinder:

    def __init__(self):
        self.sub = rospy.Subscriber(
            '/scan_raw',
            LaserScan,
            self.callback,
            queue_size=1,
        )
        self.marker_pub = rospy.Publisher(
            'Marker_laser',
            Marker,
            queue_size=1,
        )

    def idx_from_angle(self, angle, msg):
        return int((math.radians(angle) - msg.angle_min)/msg.angle_increment)

    def callback(self, msg):
        idx_90 = self.idx_from_angle(90, msg)
        idx_m90 = self.idx_from_angle(-90, msg)
        idx_0 = self.idx_from_angle(0, msg)

        print('--------------------------------------------------------------')
        print(msg.angle_min, msg.angle_max, msg.angle_increment)
        print(msg.ranges[idx_90])
        print(msg.ranges[idx_m90])
        print(msg.ranges[idx_0])
        print('--------------------------------------------------------------')
