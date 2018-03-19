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

STATUS = ["STOPPED", "STRAIGHT", "ROTATING", "ARC","FACING_GOAL","WALL_FOLLOWING","GOALSEEKING"]

MAX_LIN_VEL = .7
MAX_ROT_VEL = 3.14

BASE_DIAMETER = .23
WHEEL_RADIUS = .035
WHEEL_WIDTH = .021
TICKS_PER_REVOLUTION = 52
PULSES_PER_REVOLUTION = 13
TICK_TO_METER = .000085292090497737556558
METER_TO_TICK = 11724.41658029856624751591
MILLIMITER_TO_TICK = 11.72441658029856624751591
TICK_TO_RADIAN = .002436916871363930187454

# THRESHOLDS CHANGE THOSE AS YOU SEEM FIT
# x, y distance form target to consider correct
min_distance_to_your_goal = .1
# same for rotation angle
ROT_THRES = 5

def normalize_angle(angle):
    """REDUCE ANGLES TO -180 180"""
    angle %= 360
    angle = (angle + 360) % 360
    if angle > 180:
	angle -= 360
    return angle


class RobotStatus:
    STOPPED, STRAIGHT, ROTATING, ARC = range(4)

    def __init__(self):
        pass
    
class TaskStatus:
    NOTASK, REACH_OBSTACLE, FACING_GOAL, WALL_FOLLOWING, GOAL_SEEKING = range(6)
    
    def __init__(self):
	pass    


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
	self.tmp_goal_distance = -1
	self.tmp_goal_rotation = -1
	self.goal_x = 0.0
	self.goal_y = 0.0
	#distance_to_goal = 0.0	
	self.status = RobotStatus.STOPPED
	self.taskStatus = TaskStatus.NOTASK
	self.reached_goal = False
	self.hit_point_x = 0.0
	self.hit_pint_y = 0.0

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
        print ("Waiting, the node is initializing...")
        
        # set the goal
	self.set_linear_goal(5.0,0.0)
        # you might want to define a temporary goal for when the bot face towards a wall
	
		
        self.face_to_goal()
        while (not rospy.is_shutdown() & (self.goal_distance > min_distance_to_your_goal) ):
            # implement bug2 algorithm

            # you might want to print your current position and your tmp and actual goal here
	    
	    
	    #Start Robot, move robot toward goal
	    if (self.status == RobotStatus.STOPPED) & (self.taskStatus == TaskStatus.NOTASK):
		# Go to goal
		self.taskStatus = TaskStatus.GOAL_SEEKING
		self.drive_straight(.3,self.goal_distance)
		
	    #Wall Following
	    if(Reach_obstacle) & (self.taskStatus != TaskStatus.WALL_FOLLOWING):	
		self.taskStatus = TaskStatus.WALL_FOLLOWING

		#Following Wall, Change back to FACING_GOAL after done.
		self.taskStatus = TaskStatus.FACING_GOAL
		
	    #Roate Robot to face the goal, then head to goal;
	    if(self.taskStatus == TaskStatus.FACING_GOAL):
		self.face_to_goal()
		self.go_to_goal()
	    
	    
	    
	    self.cmd_vel.publish(self.twist_msg)
	    rate.sleep()
	    
	
	rospy.loginfo("Stopping Turtlebot")
	self.cmd_vel.publish(Twist())
    
    
    ########################
    def radian2degree(self, radian):
        return radian*57.2958

    def degree2radian(self, degree):
        return degree/57.2958

# Following are some functions that you may want to implement for # this lab. 
    
	# Import code from Lab4
    def euclidean_distance(self):
	"""Calculate euclidean distance between two points"""
	#print (self.starting_odom.pose.pose.position.x)
	#print (self.starting_odom.pose.pose.position.y)
	#print (self.odom.pose.pose.position.x)
	#print (self.odom.pose.pose.position.y)
	dis = np.sqrt(math.pow((self.starting_odom.pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
	                  math.pow((self.starting_odom.pose.pose.position.y - self.odom.pose.pose.position.y), 2))
	# print dis
	return dis
    
    def rotation_distance(self):
	"""Calculate the difference in yaw between two quaternions"""
	(r1, p1, y1) = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x,
	                                                         self.odom.pose.pose.orientation.y,
	                                                         self.odom.pose.pose.orientation.z,
	                                                         self.odom.pose.pose.orientation.w])
	(r2, p2, y2) = tf.transformations.euler_from_quaternion([self.starting_odom.pose.pose.orientation.x,
	                                                         self.starting_odom.pose.pose.orientation.y,
	                                                         self.starting_odom.pose.pose.orientation.z,
	                                                         self.starting_odom.pose.pose.orientation.w])
	# r, p, y now contain the roll, pitch and yaw from the two quaternions
	# you can derive the angle from that in a number of ways
	euler = abs(y2 - y1)
	yaw_degrees = euler * 180.0 / math.pi
	# yaw_degrees += 360.0 if (yaw_degrees < 0) else yaw_degrees
	# print yaw_degrees
	return yaw_degrees    
    
    def drive_straight(self, speed, distance):
	# Loop Rate
	rate = rospy.Rate(50)
	
	self.starting_odom = self.odom()
	self.status = RobotStatus.STRAIGHT
	twist = Twist()
	if speed >= 0:
	    twist.linear.x = MAX_LIN_VEL if speed > MAX_LIN_VEL else speed
	else:
	    # linvel = -MAX_LIN_VEL if speed < -MAX_LIN_VEL else speed
	    rospy.logwarn("You are moving backwards, cancelling")
	ed = self.euclidean_distance();
	dis_to_go = abs(self.euclidean_distance() - distance)
	while (dis_to_go > min_distance_to_your_goal) & (self.taskStatus != TaskStatus.REACH_OBSTACLE):
	    self.cmd_vel.publish(twist)
	    dis_to_go = abs(self.euclidean_distance() - distance)
	    rate.sleep()
	# Stop robot
	self.cmd_vel.publish(Twist())
	pass
    
    def rotate(self, angle):
	twist = Twist()
	twist.angular.z = 1 if angle > 0 else -1
	self.status = RobotStatus.ROTATING
	self.starting_odom = self.odom()
	
	angle = normalize_angle(angle)
	print("Rotating %.4f degrees" % angle)
    
	goal_rotation = abs(angle)
	rd = abs(self.rotation_distance())
	angle_to_go = abs(rd - goal_rotation)
	while (angle_to_go > ROT_THRES) & (self.status != RobotStatus.STOPPED):
	    self.cmd_vel.publish(twist)
	    angle_to_go = abs(abs(self.rotation_distance()) - goal_rotation)
	    
	#Stop Robot
	self.cmd_vel.publish(Twist())
    
    def stop_robot(self):
    
	rospy.logerr("Stoping Robot")
	self.twist_msg = Twist()
	self.status = RobotStatus.STOPPED
	
	    
    def check_process(self):
	if self.taskStatus == TaskStatus.GOAL_SEEKING:
	    self.goal_distance = self.distance(self.goal_x,self.goal_y)
	    #rospy.logwarn("%.4f meters to go " % (self.goal_distance))
	    if self.goal_distance <= min_distance_to_your_goal:
		self.stop_robot()
		print ("Yay, we made it")
	#elif self.taskStatus == TaskStatus.
    
	
    def face_to_goal(self):
        # turn the robot until it faces towards the goal
	return 0

    def turn_right(self):
        # you want to turn to your right when you see an obstacle, or maybe left. Up to you, doesn't really matter.
	return 0

    def set_linear_goal(self, x_goal, y_goal):
        # If you are using codes from Lab 4: when the robot is far away from a goal, set a relevently large value for your speed. Not that large though. 
        # when the robot gets close to a goal, set a small value for your speed. If you design a P controller, you don't need to do this manually. 
	self.goal_x = x_goal
	self.goal_y = y_goal
	self.goal_distance = self.distance(x_goal,y_goal)
	
	return 0

    def go_to_goal(self, x_goal, y_goal, end_time):
        # make it move towards the goal
        # don't forget to use sleep function to sync your while loop with the frequency of other nodes
	
	drive_straight(0.3)
	"""
        # maybe it's not a bad idea to publish an empty twist message to reset everything at the end of this function
        self.cmd_vel.publish(Twist())

        if (distance_to_goal < min_distance_to_your_goal):
            # we made it to the goal
            print ("Yay, we made it")
            return True
        else:
            # an error happened
            print ("go to goal has failed")
            return False
	"""

    def distance(self, x_goal, y_goal):
        return sqrt(((self.x - x_goal)**2)+((self.y - y_goal)**2))

    def scan_callback(self, scan_msg):
        # get an idea from this link about how to read the laser scanner
        # https://gist.github.com/atotto/c47bc69a48ed38e86947b5506b8e0e61
	return 0



    def bumper_callback(self, bumper_msg):
	return 0


    def odom_callback(self, odom_msg):
        # try to save x, y, and theta in your instance variables, for your own convience.
	self.x = odom_msg.pose.pose.position.x
	self.y = odom_msg.pose.pose.position.y
	self.theta = odom_msg.pose.pose.position.z
	self.odom = odom_msg
	return 0

    def bug_angle():
        return atan2(goal.pose.pose.position.y - poser.pose.pose.position.y, goal.pose.pose.position.x- poser.pose.pose.position.x)


if __name__ == '__main__':
    try:
        Bug2()
    except rospy.ROSInterruptException:
        pass

