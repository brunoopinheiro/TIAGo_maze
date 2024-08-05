#! /usr/bin/python3

import rospy


class myRobot():

    def __init__(self):
        print('init')
        # Subscriber odometria
        # Subscriber laser
        # Client Service camera
        # Publisher base
        # Publisher cabeca

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


if __name__ == '__main__':

    rospy.init_node('nodeName')

    tiago = myRobot()

    state = 0
    # while(...):
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
