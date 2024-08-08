#! /usr/bin/python3

import rospy
from math import inf, radians
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from typing import Tuple
from tf.transformations import euler_from_quaternion
from control_msgs.msg import PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib import SimpleActionClient


LASER_SUB_TOPIC = '/scan_raw'
BASE_CONT_TOPIC = '/mobile_base_controller/cmd_vel'
BASE_ORIENT_TOPIC = '/mobile_base_controller/odom'
HEAD_CONTROLLER_TOPIC = '/head_controller/command'
# Action Controller
HEAD_ACTION_TOPIC = '/head_controller/point_head_action'


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
            JointTrajectory,
            queue_size=1,
        )
        self.head_ac = SimpleActionClient(
            HEAD_ACTION_TOPIC,
            PointHeadAction,
        )
        # self._adjust_pose()

    def callback_odometry(self, msg):
        quat = msg.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, yaw) = euler_from_quaternion(quat_list)
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

    def move_straight(self, wall_limit=0.7):
        """Moves the TIAGo robot in a straight line until
        it reaches 0.7 meters from a wall, detected by its
        scan (oriented by its 0 degrees laser).

        Args:
            wall_limit (float, optional): Distance from the wall.
            Defaults to 0.7
        """
        threshold = wall_limit
        while (self.v0 - threshold) > 0:
            move = self.v0 - threshold
            if move > 0.3:
                move = 0.3
            if move < 0.01:
                move = 0.01
            self.move_base(x=move)

    def move_head_action(self, x=1.0, y=0.0, z=1.1, block=False):
        """Moves the TIAGo head to a given x, y, z point.
        The y value makes the TIAGo look to the left or right.
        A negative value means left, a positive value means right.
        Args:
            x (float, optional): Foward. Defaults to 1.0.
            y (float, optional): Left/Right. Defaults to 0.0.
            z (float, optional): Height. Defaults to 1.1.
            block (bool, optional): Blocks the code.
        """
        rospy.sleep(0.5)
        goal = PointHeadGoal()
        goal.pointing_frame = 'xtion_optical_frame'
        goal.pointing_axis.z = 1.0
        goal.max_velocity = 1.5
        goal.min_duration = rospy.Duration(0.5)
        goal.target.header.frame_id = 'base_link'
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        if block is False:
            self.head_ac.send_goal(goal)
        else:
            self.head_ac.send_goal_and_wait(goal)
        rospy.sleep(0.5)

    def move_head(self, joint_1=0.0, joint_2=0.0):
        """Moves the TIAGo head horizontally and/or vertically.

        The Max absolute value for joint_1 is 1.5.
        Negative values rotate the head to the right,
        Positive values rotate the head to the left.

        And the Max absolute value for joint_2 is 1.0.
        Positive values rotate the head up,
        Negative values rotate the head down.

        Args:
            joint_1 (float, optional): Moves the head Horizontally.
            Defaults to 0.0.
            joint_2 (float, optional): Moves the head Vertically.
            Defaults to 0.0.
        """
        rospy.sleep(0.5)
        cmd = JointTrajectory()
        cmd.joint_names.extend([
            'head_1_joint',
            'head_2_joint',
        ])
        point = JointTrajectoryPoint()
        point.positions = [joint_1, joint_2]
        point.time_from_start = rospy.Duration(1)
        cmd.points.append(point)
        rate = rospy.Rate(1)
        self.head_pub.publish(cmd)
        rate.sleep()

    def turn(self, left=False):
        # yaw -> 3 a -3
        modifier = 1.5
        move = 0.1
        if left is False:
            modifier = -1.5
            move = -0.1
        initial_value = self.yaw
        desired_value = initial_value + modifier
        if desired_value > 3:
            diff = 3 - self.yaw
            modifier = 1.5 - diff
            desired_value = -3 + modifier
        if desired_value < -3:
            diff = -3 + self.yaw
            modifier = -1.5 + diff
            desired_value = 3 - modifier
        while round(self.yaw, 2) != desired_value:
            print(f'Yaw: {self.yaw} :: {desired_value}')
            self.move_base(yaw=move)

    def decision(self):
        print('decision')
        #


if __name__ == '__main__':

    rospy.init_node('tiago_controller_rria')

    tiago = myRobot()

    state = 0
    # while not rospy.is_shutdown():
    # # tiago.move_straight()
    tiago.turn(left=False)
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
