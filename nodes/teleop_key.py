#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import sys
import tty
import termios
import atexit
import rospy
import math
from select import select
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose


class CozmoTeleop(object):
    settings = None

    def __init__(self): 
        # vars
        self.block_x = 0
        self.block_y = 0
        self.block_theta = 0
        self.turtle_x = 0
        self.turtle_y = 0
        self.turtle_theta = 0
        # params
        self.lin_vel = rospy.get_param('~lin_vel', 0.2)
        self.ang_vel = rospy.get_param('~ang_vel', 1.5757)

        # pubs
        self._cmd_vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
        
        def turtlePose(pose):
            self.turtle_x = pose.x
            self.turtle_y = pose.y
            self.turtle_theta = pose.theta

        def blockPose(pose):
            self.block_x = pose.x
            self.block_y = pose.y
            self.block_theta = pose.theta

        #subs
        rospy.Subscriber('block/pose', Pose, blockPose)
        rospy.Subscriber('turtle1/pose', Pose, turtlePose)

    def closestDock(self, block_x, block_y, block_theta, turtle_x, turtle_y):
        assert(block_theta >= 0)
        flat_angles = [(block_theta + (i * math.pi)/2) % (2*math.pi) for i in range(0, 4)]
        docking_points_x = [block_x + 2*math.cos(angle) for angle in flat_angles]
        docking_points_y = [block_y + 2*math.sin(angle) for angle in flat_angles]
        closest_dock = min(map(
            self.euclidean_distance, 
            docking_points_x, 
            docking_points_y, 
            [turtle_x for _ in range(0, 4)], 
            [turtle_y for _ in range(0, 4)]), 
            key=lambda x:x[1])
        return closest_dock[0]

    def toBlockAngle(self, block_x, block_y, turtle_x, turtle_y):
        return math.atan2(block_y - turtle_y, block_x - turtle_x)
 
    def turnFaceBlock(self, cmd_vel, to_block_angle, turtle_angle):
        turn_angle = to_block_angle - turtle_angle
        if(turn_angle > 0):
            cmd_vel.angular.z = self.ang_vel
        else:
            cmd_vel.angular.z = -self.ang_vel
    
    def moveForward(self, cmd_vel):
        cmd_vel.linear.x = self.lin_vel

    def equalAngles(self, a, b):
        if(abs(a-b)<.05):
            return True
        else:
            return False
    
    def euclidean_distance(self, x, y, turtle_x, turtle_y):
        return [(x, y), math.sqrt(pow((x - turtle_x), 2) + pow((y - turtle_y), 2))]

    def run(self):

        def at(point, x, y):
            if self.euclidean_distance(point[0], point[1], x, y)[1] < .05:
                return True
            else:
                return False
 
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            # cmd_vel.linear.x = self.lin_vel
            
            # s - backward
            # cmd_vel.linear.x = -self.lin_vel
            
            # a - turn left
            #cmd_vel.angular.z = self.ang_vel
            #if(self.is_block):
            turtle_angle = self.turtle_theta
            dock_point = self.closestDock(self.block_x, self.block_y, turtle_angle, self.turtle_x, self.turtle_y)
            to_block_angle = self.toBlockAngle(dock_point[0], dock_point[1], self.turtle_x, self.turtle_y)
            if to_block_angle < 0:
                to_block_angle = 2*math.pi + to_block_angle
            #if turtle_angle < 0:
            #    turtle_angle = 2*math.pi + turtle_angle
            print(str(to_block_angle) + " : " + str(abs(turtle_angle)))
            if not self.equalAngles(to_block_angle, abs(turtle_angle)) and not at(dock_point, self.turtle_x, self.turtle_y):
                self.turnFaceBlock(cmd_vel, to_block_angle, turtle_angle)  
            elif not at(dock_point, self.turtle_x, self.turtle_y):
                self.moveForward(cmd_vel)
            else:
                to_block_angle = self.toBlockAngle(self.block_x, self.block_y, self.turtle_x, self.turtle_y)
                self.turnFaceBlock(cmd_vel, to_block_angle, turtle_angle)
                
            #self.closestDockingPosition(self.block_x, self.block_y, self.turtle_x, self.turtle_y)
            # d - turn right
            # cmd_vel.angular.z = -self.ang_vel

            # head movement
            # r - up
            # self.head_angle += 2.0
            # if self.head_angle > MAX_HEAD_ANGLE:
            #     self.head_angle = MAX_HEAD_ANGLE
            #     head_changed = True
            # v - down
            # self.head_angle -= 2.0
            # if self.head_angle < MIN_HEAD_ANGLE:
            #     self.head_angle = MIN_HEAD_ANGLE
            #     head_changed = True
            self._cmd_vel_pub.publish(cmd_vel)
            r.sleep()
            # lift movement

            # t - up
            # self.lift_height += 2.0
            # if self.lift_height > MAX_LIFT_HEIGHT:
            #     self.lift_height = MAX_LIFT_HEIGHT
            #     lift_changed = True
            # b - down
            # self.lift_height -= 2.0
            # if self.lift_height < MIN_LIFT_HEIGHT:
            #     self.lift_height = MIN_LIFT_HEIGHT
            #     lift_changed = True

            # publish commands (head angle and lift height on change only)
            #self._cmd_vel_pub.publish(cmd_vel)
            #if head_changed:
            #    self._head_pub.publish(data=self.head_angle)
            #if lift_changed:
            #    self._lift_pub.publish(data=(self.lift_height-MIN_LIFT_HEIGHT)/SUM_LIFT_HEIGHT)

            #r.sleep()

# start ROS node
rospy.init_node('teleop_key', disable_signals=True)
# initialize keyboard teleoperation
cozmo_teleop = CozmoTeleop()
# loop
cozmo_teleop.run()
