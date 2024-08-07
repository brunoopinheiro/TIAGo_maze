#! /usr/bin/python3

import rospy
from math import inf, radians
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from control_msgs.msg import PointHeadActionGoal
from typing import Tuple
from tf.transformations import euler_from_quaternion, quaternion_from_euler



LASER_SUB_TOPIC = '/scan_raw'
BASE_CONT_TOPIC = '/mobile_base_controller/cmd_vel'
BASE_ORIENT_TOPIC = '/mobile_base_controller/odom'
HEAD_CONTROLLER_TOPIC = '/head_controller/point_head_action/goal'


class myRobot():

    @property
    def laser_values(self) -> Tuple[float, float, float]:
        return (self.v270, self.v0, self.v90)

    def __init__(self):
        # Subscriber odometria
        self.sub_odom = rospy.Subscriber(
            BASE_ORIENT_TOPIC,
            Odometry,
            self.callback_odometry,
            queue_size=1,
        )
        self.yaw = 0.0
        # Subscriber laser
        self.v90 = inf
        self.v270 = inf
        self.v0 = inf
        self.sub_laser = rospy.Subscriber(
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
        self.head_pub = rospy.Publisher(
            HEAD_CONTROLLER_TOPIC,
            PointHeadActionGoal,
            queue_size=1,
        )
        # self._adjust_pose()

    def callback_odometry(self, msg):
        quat = msg.pose.pose.orientation
        quat_list = [quat.x,quat.y,quat.z,quat.w]
        (roll,pitch,yaw) = euler_from_quaternion(quat_list)
        self.yaw = yaw        
        # Armazenar os dados de odometria

    def __idx_from_angle(self, angle, msg):
        """Receives an angle and the message, and calculates
        the index from the message that corresponds to that
        angle.

        Args:
            angle (float): the angle in degrees
            msg (LaserScan): The LaserScan message

        Returns:
            int: the index for LaserScan.ranges
        """
        return int((radians(angle) - msg.angle_min)/msg.angle_increment)

    def callback_laser(self, msg):
        """Callback used to update the robot's scanner
        sensors. Updates the values stored in the
        `v90`, `v270` and `v0` properties.

        Args:
            msg (LaserScan): The message read from
            the `/scan_raw` topic.
        """
        idx_90 = self.__idx_from_angle(90, msg)
        idx_m90 = self.__idx_from_angle(-90, msg)
        idx_0 = self.__idx_from_angle(0, msg)
        self.v90 = msg.ranges[idx_90]
        self.v270 = msg.ranges[idx_m90]
        self.v0 = msg.ranges[idx_0]

    def move_base(
            self,
            x=0.0,
            yaw=0.0,
    ) -> None:
        """Moves the TIAGo base in the `x` or `yaw` refferences
        the given velocity (m/s).

        Args:
            x (float, optional): The desired velocity.
                Defaults to 0.0.
            yaw (float, optional): The desired velocity.
                Defaults to 0.0.
        """
        new_pose = Twist()
        new_pose.linear.x = x
        new_pose.angular.z = yaw
        self.base_pub.publish(new_pose)

    def move_straight(self):
        """Moves the TIAGo robot in a straight line until
        it reaches 0.7 meters from a wall, detected by its
        scan (oriented by its 0 degrees laser).
        """
        threshold = 0.7
        while (self.v0 - threshold) > 0:
            move = self.v0 - threshold
            if move > 0.3:
                move = 0.3
            if move < 0.01:
                move = 0.01
            self.move_base(x=move)

    def move_head(self):
        r = rospy.Rate(10)
        phag = PointHeadActionGoal()
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)
        phag.goal. = 10.0
        self.head_pub.publish(phag)
        r.sleep()

    def turn(self, sens):
        print('turn')
        # error = ...
        # while(abs(error) < value):

    def decision(self):
        print('decision')
        #


if __name__ == '__main__':

    rospy.init_node('tiago_controller_rria')

    tiago = myRobot()

    state = 0
    # while not rospy.is_shutdown():
    print(tiago.laser_values)
    # # tiago.move_straight()
    tiago.move_head()
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
