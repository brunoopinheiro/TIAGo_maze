#! /usr/bin/python3

import rospy
from math import inf
from base_mover import BaseMover
from range_finder import RangeFinder
from typing import Tuple


class myRobot():

    @property
    def laser_values(self) -> Tuple[float, float, float]:
        return (self.__laser.v270, self.__laser.v0, self.__laser.v90)

    def __init__(self):
        print('init')
        # Subscriber odometria
        # Subscriber laser
        self.__laser = RangeFinder()
        # Client Service camera
        # Publisher base
        self.__base = BaseMover()
        # Publisher cabeca
        # self._adjust_pose()

    def callback_odometria(self, msg):
        print('callback odometria')
        # Armazenar os dados de odometria

    def callback_laser(self, msg):
        print('callback laser')
        # Armazenar os dados do laser

    def moveStaright(self):
        print('move straight')
        # error = ...
        # while(abs(error) < value):

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
    #     print(tiago.laser_values)
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
