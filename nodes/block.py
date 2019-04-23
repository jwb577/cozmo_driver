#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import sys
import tty
import termios
import atexit
from select import select
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CozmoTeleop(object):
    settings = None

    def __init__(self):
        # setup
        CozmoTeleop.settings = termios.tcgetattr(sys.stdin)
        atexit.register(self.reset_terminal)

        # vars
        self.head_angle = STD_HEAD_ANGLE
        self.lift_height = STD_LIFT_HEIGHT

        # params
        self.lin_vel = rospy.get_param('~lin_vel', 0.2)
        self.ang_vel = rospy.get_param('~ang_vel', 1.5757)

        # pubs
        self._head_pub = rospy.Publisher('head_angle', Float64, queue_size=1)
        self._lift_pub = rospy.Publisher('lift_height', Float64, queue_size=1)
        self._cmd_vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)


    def run(self):
        r = rospy.Rate(20)

        while not rospy.is_shutdown():

            # reset twist (commanded velocities)
            cmd_vel = Twist()

            # reset states
            head_changed = False
            lift_changed = False

            # get key
            key1 = self.get_key()
	    key2 = self.get_key()
            # get number from key
            ord_key1 = ord(key1)
	    ord_key2 = ord(key2)

            # ctrl+c
            if ord_key1 == 3:
                print('Shutdown')
                break

            # esc
            if ord_key1 == 27:
                print('Escaping')
                key_1 = self.get_key()
                key_2 = self.get_key()
                ord_key2 = ord(key_2)
                if ord_key2 == 65:
                    ord_key = 119
                elif ord_key2 == 66:
                    ord_key = 115
                elif ord_key2 == 67:
                    ord_key = 97
                elif ord_key2 == 68:
                    ord_key = 100

            # robot
            # w - forward
            elif ord_key1 == 119 or ord_key2 == 119:
                cmd_vel.linear.x = self.lin_vel
            # s - backward
            elif ord_key1 == 115 or ord_key2 == 115:
                cmd_vel.linear.x = -self.lin_vel
            # a - turn left
            elif ord_key1 == 97 or ord_key2 == 97:
                cmd_vel.angular.z = self.ang_vel
            # d - turn right
            elif ord_key1 == 100 or ord_key2 == 100:
                cmd_vel.angular.z = -self.ang_vel

            # head movement
            # r - up
            elif ord_key1 == 114:
                self.head_angle += 2.0
                if self.head_angle > MAX_HEAD_ANGLE:
                    self.head_angle = MAX_HEAD_ANGLE
                head_changed = True
            # f - center
            elif ord_key1 == 102:
                self.head_angle = STD_HEAD_ANGLE
                head_changed = True
            # v - down
            elif ord_key1 == 118:
                self.head_angle -= 2.0
                if self.head_angle < MIN_HEAD_ANGLE:
                    self.head_angle = MIN_HEAD_ANGLE
                head_changed = True

            # lift movement
            # t - up
            elif ord_key1 == 116:
                self.lift_height += 2.0
                if self.lift_height > MAX_LIFT_HEIGHT:
                    self.lift_height = MAX_LIFT_HEIGHT
                lift_changed = True
            # g - center
            elif ord_key1 == 103:
                self.lift_height = STD_LIFT_HEIGHT
                lift_changed = True
            # b - down
            elif ord_key1 == 98:
                self.lift_height -= 2.0
                if self.lift_height < MIN_LIFT_HEIGHT:
                    self.lift_height = MIN_LIFT_HEIGHT
                lift_changed = True

            # debug
            else:
                print(ord(ord_key1), ord(ord_key2))

            # publish commands (head angle and lift height on change only)
            self._cmd_vel_pub.publish(cmd_vel)
            if head_changed:
                self._head_pub.publish(data=self.head_angle)
            if lift_changed:
                self._lift_pub.publish(data=(self.lift_height-MIN_LIFT_HEIGHT)/SUM_LIFT_HEIGHT)

            r.sleep()

# start ROS node
rospy.init_node('teleop_key')
# initialize keyboard teleoperation
cozmo_teleop = CozmoTeleop()
# loop
cozmo_teleop.run()
