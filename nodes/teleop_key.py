#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import sys
import tty
import termios
import atexit
import rospy
import math
import matplotlib.pyplot as plt
from select import select
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose


class CozmoTeleop(object):
    settings = None

    def __init__(self, debug=False): 
        # vars
        self.debug = debug
        self.block_x = 0
        self.block_y = 0
        self.block_theta = 0
        self.turtle_x = 0
        self.turtle_y = 0
        self.turtle_theta = 0
        # params
        self.lin_vel = rospy.get_param('~lin_vel', 0.4)
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

    def closestDock(self):
        assert(self.block_theta >= 0)
        flat_angles = [(self.block_theta + (i * math.pi)/2) % (2*math.pi) for i in range(0, 4)]
        docking_points_x = [self.block_x + 1*math.cos(angle) for angle in flat_angles]
        docking_points_y = [self.block_y + 1*math.sin(angle) for angle in flat_angles]
        docks = map(
            self.euclidean_distance, 
            docking_points_x, 
            docking_points_y
        )
        val, closest_dock = min((val, closest_dock) for (closest_dock, val) in enumerate(docks))
        docking_points = list(zip(docking_points_x, docking_points_y))
        
        if self.debug: 
            print(set(zip(docking_points_x, docking_points_y)))
            docking_points_x.append(block_x)
            docking_points_y.append(block_y)
            plt.plot(docking_points_x, docking_points_y, 'ro')
            plt.show()
        return docking_points[closest_dock]

    def toPointAngle(self, point_x, point_y):
        if point_x > 0 and point_y > 0:
            offset = 0
        elif point_x < 0 and point_y > 0:
            offset = math.pi/2
        elif point_x < 0 and point_y < 0:
            offset = math.pi
        else:
            offset = math.pi + math.pi/2
            
        return offset + math.atan2(point_y - self.turtle_y, point_x - self.turtle_x)
 
    def turnFacePoint(self, cmd_vel, to_point_angle):
        if not self.equalAngles(to_point_angle, self.turtle_theta):
            if abs(self.turtle_theta) - to_point_angle > 0:
                cmd_vel.angular.z = -self.ang_vel
            else:
                cmd_vel.angular.z = self.ang_vel
        else:
            cmd_vel.angular.z = 0
    
    def moveForward(self, cmd_vel):
        cmd_vel.linear.x = self.lin_vel

    def equalAngles(self, a, b):
        if(abs(a-b)<.05):
            return True
        else:
            return False
    
    def euclidean_distance(self, x, y):
        return math.sqrt(pow((x - self.turtle_x), 2) + pow((y - self.turtle_y), 2))

    def run(self):

        def at(point):
            if self.euclidean_distance(point[0], point[1]) < .05:
                return True
            else:
                return False
 
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            
            dock_point = self.closestDock()
            to_dock_angle = self.toPointAngle(dock_point[0], dock_point[1])
            if to_dock_angle < 0:
                to_dock_angle = 2*math.pi + to_dock_angle
            
            if not at(dock_point):
                self.turnFacePoint(cmd_vel, to_dock_angle)
                self.moveForward(cmd_vel)
            else:
                to_block_angle = self.toPointAngle(self.block_x, self.block_y)
                cmd_vel.linear.x = 0
                cmd_vel.linear.y = 0
                self.turnFacePoint(cmd_vel, to_block_angle)
                
            self._cmd_vel_pub.publish(cmd_vel)
            r.sleep()

# start ROS node
rospy.init_node('teleop_key')
# initialize keyboard teleoperation
cozmo_teleop = CozmoTeleop()
cozmo_teleop.run()
