#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

TOPIC = '/mobile_base_controller/cmd_vel'


class BaseMover:

    def __init__(self) -> None:
        self.base_pub = rospy.Publisher(
            TOPIC,
            Twist,
            queue_size=1,
        )

    def adjust_pose(
            self,
            x=0.0,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
    ) -> None:
        print('Twisting to zero pose')
        new_pose = Twist()
        new_pose.linear.x = x
        new_pose.linear.y = y
        new_pose.linear.z = z
        new_pose.angular.x = roll
        new_pose.angular.y = pitch
        new_pose.angular.z = yaw
        self.base_pub.publish(new_pose)
