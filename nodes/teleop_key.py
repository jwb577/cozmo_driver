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
        
        if self.debug: 
            print(set(zip(docking_points_x, docking_points_y)))
            docking_points_x.append(block_x)
            docking_points_y.append(block_y)
            plt.plot(docking_points_x, docking_points_y, 'ro')
            plt.show()
        return closest_dock[0]

    def toPointAngle(self, point_x, point_y, turtle_x, turtle_y):
        return math.atan2(point_y - turtle_y, point_x - turtle_x)
 
    def turnFacePoint(self, cmd_vel, to_point_angle, turtle_angle):
        turn_angle = to_point_angle - turtle_angle
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
            
            turtle_angle = self.turtle_theta
            dock_point = self.closestDock(self.block_x, self.block_y, turtle_angle, self.turtle_x, self.turtle_y)
            to_dock_angle = self.toPointAngle(dock_point[0], dock_point[1], self.turtle_x, self.turtle_y)
            if to_dock_angle < 0:
                to_dock_angle = 2*math.pi + to_dock_angle
            
            if not self.equalAngles(to_dock_angle, abs(turtle_angle)) and not at(dock_point, self.turtle_x, self.turtle_y):
                self.turnFacePoint(cmd_vel, to_dock_angle, turtle_angle)  
            elif not at(dock_point, self.turtle_x, self.turtle_y):
                self.moveForward(cmd_vel)
            else:
                to_block_angle = self.toPointAngle(self.block_x, self.block_y, self.turtle_x, self.turtle_y)
                self.turnFacePoint(cmd_vel, to_block_angle, turtle_angle)
                
            self._cmd_vel_pub.publish(cmd_vel)
            r.sleep()

# start ROS node
rospy.init_node('teleop_key', disable_signals=True)
# initialize keyboard teleoperation
cozmo_teleop = CozmoTeleop(debug=True)
if cozmo_teleop.debug:
    #debug
    print(cozmo_teleop.closestDock(4,4,0,5,1))
else:
    # loop
    cozmo_teleop.run()
