#!/usr/bin/env python

import rospy
from math import *
import numpy as np
import tf
import lab4 as controller
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from lab4 import Lab2Solution
from lab4 import RobotStatus

# for move_base
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


class Bug2():
    """docstring for Bug2"""
    def __init__(self):

        # ros related variables
        rospy.init_node('bug2')
        # odometry messages
        self.odom = Odometry()
        self.starting_odom = Odometry()
        # bumper event message
        self.bumper_msg = BumperEvent()
        # twist message
        self.twist_msg = Twist()

        # robot's position and direction
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.straight_distance = 0.0

        # things you might want to consider using
        self.bumper_pressed = -1
        self.goal_distance = -1

        # reading the laser data
        self.scan = LaserScan()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)

        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback, queue_size=1)
        # subscribe to odometry messages
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        # publisher for twist messages
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

        # loop rate
        rate = rospy.Rate(50)
        start_time = rospy.Time.now()
        
        # wait until specific amount of time, until your messeges start to be received by your node
        print "Waiting, the node is initializing..."
        
        # set the goal

        # you might want to define a temporary goal for when the bot face towards a wall

        
		drive_straight(.3 , .6)
		
        self.face_to_goal()
        while (not rospy.is_shutdown() & (distance_to_goal > min_distance_to_your_goal) ):
            # implement bug2 algorithm

            # you might want to print your current position and your tmp and actual goal here

        rospy.loginfo("Stopping Turtlebot")
        self.cmd_vel.publish(Twist())
    
    def radian2degree(self, radian):
        return radian*57.2958

    def degree2radian(self, degree):
        return degree/57.2958

# Following are some functions that you may want to implement for # this lab. 

	# Import controller from Lab4
	def drive_straight(self, speed, distance, time=.1):
		return controller.Lab2Solution.drive_straight(self, speed, distance, time=.1)
		
	def rotate(self, angle):
		return controller.Lab2Solution.rotate(self, angle)

    def face_to_goal(self):
        # turn the robot until it faces towards the goal

    def turn_right(self):
        # you want to turn to your right when you see an obstacle, or maybe left. Up to you, doesn't really matter.

    def set_linear_goal(self, x_goal, y_goal):
        # If you are using codes from Lab 4: when the robot is far away from a goal, set a relevently large value for your speed. Not that large though. 
        # when the robot gets close to a goal, set a small value for your speed. If you design a P controller, you don't need to do this manually. 

    def go_to_goal(self, x_goal, y_goal, end_time):
        # make it move towards the goal
        # don't forget to use sleep function to sync your while loop with the frequency of other nodes

        # maybe it's not a bad idea to publish an empty twist message to reset everything at the end of this function
        self.cmd_vel.publish(Twist())

        if (distance_to_goal < min_distance_to_your_goal):
            # we made it to the goal
            print "Yay, we made it"
            return True
        else:
            # an error happened
            print "go to goal has failed"
            return False

    def distance(self, x_goal, y_goal):
        return sqrt(((self.x - x_goal)**2)+((self.y - y_goal)**2))

    def scan_callback(self, scan_msg):
        # get an idea from this link about how to read the laser scanner
        # https://gist.github.com/atotto/c47bc69a48ed38e86947b5506b8e0e61



    def bumper_callback(self, bumper_msg):


    def odom_callback(self, odom_msg):
        # try to save x, y, and theta in your instance variables, for your own convience.

    def bug_angle():
        return atan2(goal.pose.pose.position.y - poser.pose.pose.position.y, goal.pose.pose.position.x- poser.pose.pose.position.x)


if __name__ == '__main__':
    try:
        Bug2()
    except rospy.ROSInterruptException:
        pass

    