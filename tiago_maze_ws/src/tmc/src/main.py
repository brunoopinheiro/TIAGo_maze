#! /usr/bin/python3

import rospy
from math import inf, radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from typing import Tuple


LASER_SUB_TOPIC = '/scan_raw'
BASE_CONT_TOPIC = '/mobile_base_controller/cmd_vel'


class myRobot():

    @property
    def laser_values(self) -> Tuple[float, float, float]:
        return (self.v270, self.v0, self.v90)

    def __init__(self):
        # Subscriber odometria
        # Subscriber laser
        self.v90 = inf
        self.v270 = inf
        self.v0 = inf
        self.sub = rospy.Subscriber(
            LASER_SUB_TOPIC,
            LaserScan,
            self.callback_laser,
            queue_size=1,
        )
        # Client Service camera
        # Publisher base
        self.base_pub = rospy.Publisher(
            BASE_CONT_TOPIC,
            Twist,
            queue_size=1,
        )
        # Publisher cabeca
        # self._adjust_pose()

    def callback_odometria(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria

    def __idx_from_angle(self, angle, msg):
        return int((radians(angle) - msg.angle_min)/msg.angle_increment)

    def callback_laser(self, msg):
        idx_90 = self.__idx_from_angle(90, msg)
        idx_m90 = self.__idx_from_angle(-90, msg)
        idx_0 = self.__idx_from_angle(0, msg)
        self.v90 = msg.ranges[idx_90]
        self.v270 = msg.ranges[idx_m90]
        self.v0 = msg.ranges[idx_0]
        print(self.v0)

    def move_base(
            self,
            x=0.0,
            y=0.0,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
    ) -> None:
        new_pose = Twist()
        new_pose.linear.x = x
        new_pose.linear.y = y
        new_pose.linear.z = z
        new_pose.angular.x = roll
        new_pose.angular.y = pitch
        new_pose.angular.z = yaw
        self.base_pub.publish(new_pose)

    def move_straight(self):
        threshold = 0.7
        while (self.v0 - threshold) > 0:
            move = self.v0 - threshold
            print('Move: ', move)
            if move > 0.3:
                move = 0.3
            if move < 0.01:
                move = 0.01
            self.move_base(x=move)

    def turn(self, sens):
        print('turn')
        # error = ...
        # while(abs(error) < value):

    def decision(self):
        print('decision')
        #

    def _adjust_pose(self):
        while inf in self.laser_values:
            self.__base.adjust_pose(yaw=-0.6)

        diff = self.laser_values[0] - self.laser_values[2]
        while diff >= 0.01:
            if diff > 0.1 or diff < 0.1:
                diff = 0.1
            self.__base.adjust_pose(yaw=diff)
            diff = self.laser_values[0] - self.laser_values[2]
        print(self.laser_values[0] - self.laser_values[2])


if __name__ == '__main__':

    rospy.init_node('tiago_controller_rria')

    tiago = myRobot()

    state = 0
    # while not rospy.is_shutdown():
    print(tiago.laser_values)
    tiago.move_straight()
    # rospy.spin()
     # if state == 0:
        # decision
        # compute next state
     # else if state == 1
        # image porcessing
        # compute next state
     # else if state == 3
        # move straight
        # compute next state
     # else if state == 4
        # turn
        # compute next state
