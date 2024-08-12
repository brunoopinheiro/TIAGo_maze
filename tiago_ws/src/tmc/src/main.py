#! /usr/bin/python3

import rospy
from math import inf, radians
from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from typing import Tuple
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LASER_SUB_TOPIC = '/scan_raw'
BASE_CONT_TOPIC = '/mobile_base_controller/cmd_vel'
BASE_ORIENT_TOPIC = '/mobile_base_controller/odom'
HEAD_CONTROLLER_TOPIC = '/head_controller/command'
ARM_CONTROLLER_TOPIC = '/arm_controller/command'


class TIAGoState(Enum):

    IDLE = 0
    CAMERA_DECISION = 1
    MOVE_STRAIGHT = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    FINISH = 5


class myRobot():

    @property
    def laser_values(self) -> Tuple[float, float, float]:
        return (self.v270, self.v0, self.v90)

    def __init__(self):
        self.state = TIAGoState.IDLE
        self.inside_the_maze = False
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
        self.arm_pub = rospy.Publisher(
            ARM_CONTROLLER_TOPIC,
            JointTrajectory,
            queue_size=1,
        )
        # Publisher cabeca
        self.head_pub = rospy.Publisher(
            HEAD_CONTROLLER_TOPIC,
            JointTrajectory,
            queue_size=1,
        )

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

    def __detect_collision(self):
        pass

    def move_straight(self, wall_limit=0.6):
        """Moves the TIAGo robot in a straight line until
        it reaches 0.7 meters from a wall, detected by its
        scan (oriented by its 0 degrees laser).

        Args:
            wall_limit (float, optional): Distance from the wall.
            Defaults to 0.7
        """
        if not self.inside_the_maze:
            self.inside_the_maze = True
        threshold = wall_limit
        while (self.v0 - threshold) > 0:
            move = self.v0 - threshold
            if move > 0.4:
                move = 0.4
            if move < 0.01:
                move = 0.01
            self.move_base(x=move)

    def move_arm(
        self,
        joint1=0.0,
        joint2=0.0,
        joint3=0.0,
        joint4=0.0,
        joint5=0.0,
        joint6=0.0,
        joint7=0.0,
    ):
        """Moves the TIAGo arm to the given joint configuration"""
        cmd = JointTrajectory()
        cmd.joint_names.extend([
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint',
        ])
        point = JointTrajectoryPoint()
        point.positions = [
            joint1,
            joint2,
            joint3,
            joint4,
            joint5,
            joint6,
            joint7,
        ]
        point.time_from_start = rospy.Duration(1)
        cmd.points.append(point)
        rate = rospy.Rate(1)
        self.arm_pub.publish(cmd)
        rate.sleep()

    def move_arm_trajectory(self, list_poses):
        """Moves the TIAGo arm through the
        list of joint positions.

        Args:
            list_poses (List[float]): List of 7 floats,
            representing the joint positions.
        """
        cmd = JointTrajectory()
        cmd.joint_names.extend([
            'arm_1_joint',
            'arm_2_joint',
            'arm_3_joint',
            'arm_4_joint',
            'arm_5_joint',
            'arm_6_joint',
            'arm_7_joint',
        ])
        rate = rospy.Rate(1)
        point = JointTrajectoryPoint()
        point.positions = [0] * 7
        cmd.points.append(point)
        i = 0
        while i < len(list_poses):
            pose = list_poses[i]
            cmd.points[0].positions = pose
            point.time_from_start = rospy.Duration(i + 1)
            self.arm_pub.publish(cmd)
            rate.sleep()
            rospy.sleep(0.5)
            i += 1

    def wave_arm(self):
        """Makes the TIAGo wave its arm above its head.
        """
        jointvalues = [
            [-0.8, 0.8, 0.0, 0.0, 1.5, 0.2, 0.0],
            [-0.8, 0.8, 0.0, -0.8, 1.5, 1.0, 0.0]
        ]
        i = 0
        for i in range(6):
            if i % 2 == 0:
                self.move_arm(*jointvalues[0])
            else:
                self.move_arm(*jointvalues[1])
            rospy.sleep(0.5)

    def arm_initial_pose(self):
        """Puts the TIAGo arm back to the initial position.
        """
        joint_values = [0.199, -1.338, -0.199, 1.937, -1.570, 1.369, 1.375]
        self.move_arm(*joint_values)

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

    def __find_signs(self):
        self.move_head(joint_1=-1.5)
        rospy.loginfo('Capture Image')
        self.move_head(joint_1=1.5)
        self.move_head(joint_1=0.0)

    def camera_detection(self):
        self.__find_signs()

    def __turn_left(self):
        """Turns the TIAGo robot 90º to the left.
        The function takes the inner refference of
        the `yaw` value, obtained through the
        odometry subscriber and sends a Twist
        message to the Base Publisher to move the
        Yaw at a fixed velocity."""
        init_v = self.yaw
        target_v = init_v + 1.5
        if target_v > 3:
            diff = 3 - self.yaw
            modf = 1.5 - diff
            target_v = -3 + modf
        while abs(target_v - self.yaw) > 0.35:
            self.move_base(yaw=0.35)
        while abs(target_v - self.yaw) > 0.1:
            self.move_base(yaw=0.1)
        while abs(target_v - self.yaw) > 0.005:
            self.move_base(yaw=0.005)

    def __turn_right(self):
        """Turns the TIAGo robot 90º to the right.
        The function takes the inner refference of
        the `yaw` value, obtained through the
        odometry subscriber and sends a Twist
        message to the Base Publisher to move the
        Yaw at a fixed velocity."""
        init_v = self.yaw
        target_v = init_v - 1.5
        if target_v < -3:
            diff = -3 + self.yaw
            modf = -1.5 + diff
            target_v = 3 - modf
        while abs(target_v - self.yaw) > 0.35:
            self.move_base(yaw=-0.35)
        while abs(target_v - self.yaw) > 0.1:
            self.move_base(yaw=-0.1)
        while abs(target_v - self.yaw) > 0.005:
            self.move_base(yaw=-0.005)

    def turn(self, right=False):
        """Turns the TIAGo robot 90º to the left or right.

        Args:
            right (bool, optional): Defaults to False.
        """
        if right is True:
            self.__turn_right()
        else:
            self.__turn_left()

    def decision(self):
        """TIAGo decision state.
        Updates its internal (simplified) State Machine
        to indicate the next action it should do at his
        loop's next iteration.
        """
        frontwall = self.v0
        leftwall = self.v90
        rightwall = self.v270
        rospy.loginfo(f'State: {self.state}')
        rospy.loginfo(f'Maze: {self.inside_the_maze}')
        rospy.loginfo(f'Walls: [{leftwall}, {frontwall}, {rightwall}]')
        if not self.inside_the_maze:
            rospy.loginfo('Decision: Move Straight')
            self.state = TIAGoState.MOVE_STRAIGHT
            return
        elif (leftwall == inf
              and rightwall == inf
              and self.inside_the_maze is True):
            rospy.loginfo('Decision: Finish')
            self.state = TIAGoState.FINISH
            return
        elif (frontwall is not inf
              and (leftwall is not inf
                   or rightwall is not inf)):
            if frontwall > 1.0:
                rospy.loginfo('Decision: Move Straight')
                self.state = TIAGoState.MOVE_STRAIGHT
                return
            elif leftwall < 1.0:
                rospy.loginfo('Decision: Turn Right')
                self.state = TIAGoState.TURN_RIGHT
                return
            elif rightwall < 1.0:
                rospy.loginfo('Decision: Turn Left')
                self.state = TIAGoState.TURN_LEFT
                return
            else:
                rospy.loginfo('Decision: Camera Decision')
                self.state = TIAGoState.CAMERA_DECISION
                return


if __name__ == '__main__':

    rospy.init_node('tiago_controller_rria')
    tiago = myRobot()
    rospy.sleep(0.5)

    finish = False
    while not rospy.is_shutdown() and not finish:
    # for _ in range(5):
        if tiago.state == TIAGoState.IDLE:
            tiago.decision()
        elif tiago.state == TIAGoState.CAMERA_DECISION:
            # image porcessing
            # compute next state
            tiago.camera_detection()
            tiago.turn()
            tiago.move_straight()
            tiago.decision()
        elif tiago.state == TIAGoState.MOVE_STRAIGHT:
            tiago.move_straight()
            tiago.decision()
        elif tiago.state == TIAGoState.TURN_RIGHT:
            tiago.turn(right=True)
            tiago.decision()
        elif tiago.state == TIAGoState.TURN_LEFT:
            tiago.turn()  # defaults left
            tiago.decision()
        elif tiago.state == TIAGoState.FINISH:
            tiago.wave_arm()
            tiago.arm_initial_pose()
            tiago.turn(right=True)
            tiago.turn(right=False)
            finish = True
            tiago.decision()
