#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 29 15:04:02 2017

@author: Sam Reed
"""
import rospy
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point, Pose2D
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

class Position:
    """ Helper class that describes the positon of the turtlebot:
            x,y and theta 
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
class RobotState():
    spin = 0
    move_base = 2
    
def normalize_angle(angle):
    """REDUCE ANGLES TO -pi pi"""
    angle %= np.pi*2
    if angle > np.pi:
        angle -= np.pi*2
    return angle


class Frontier_Based_Exploration():
        
    def costmap_callback(self, map_msg):
        """ Callback to handle Map messages. """
        pass

                
    def status_callback(self, status_msg):
        """ """
        if  len(status_msg.status_list) >0:
            for i in range(len(status_msg.status_list)):
                ID = int(status_msg.status_list[i].goal_id.id.split('-')[1])
                # Store the stautus of all WPTs
                if len(self.move_base_status) >= ID+1:
                    self.move_base_status[ID] = status_msg.status_list[i].status
                else:
                    self.move_base_status.append(status_msg.status_list[i].status)
                    
    def odom_callback(self, odom_msg):
        """ callback to handle odometry messages"""        
        # convert those into x/y/theta positions
        pass

    def xy2grid(self, x,y):   
        """ Converts the local X,Y coordinates to the grid coordinate system."""
        gridX = int(round((x-self.origin.x)/self.grid_size))
        gridY = int(round((y-self.origin.y)/self.grid_size))
        return gridX,gridY
    
    def grid2xy(self, gridX, gridY):
        """ Converts the grid coordinates to the local X,Y. """
         
        x = gridX*self.grid_size+self.origin.x
         
        y = gridY*self.grid_size+self.origin.y
        return x,y
     
     
    def xy2mapIndex(self, x,y):
        """ Converts the x,y grid coordinates into a row-major index for the 
        map. """
        if x>self.ogrid_sizeX or y>self.ogrid_sizeY:
            print('MAP IS TOO SMALL!!!')
            return self.ogrid_sizeX * self.ogrid_sizeY -1
        else:
            return int(y*self.ogrid_sizeY + x)
        
    def mapIndex2xy(self, index):
        """ Converts the row-major index for the map into x,y coordinates."""
        x = np.mod(index, self.ogrid_sizeY)
        y = (index-x)/self.ogrid_sizeY
        return x,y
    
    
    # Distance on grid? or real world?
    def distanceFomula(self, x1,y1, x2,y2):
        return sqrt(((x1 - x2)**2)+((y1 - y2)**2))
        

         
    def calcCost(self, centroidX, centroidY, frontierLength):
        """ Calculate the cost of the frontier's centroid using a combo of
            the distance to the centroid and the length of the centroid.
        """

        
    def calcCost_dist(self, X, Y):
        """ Calculate the cost of the frontier's centroid using just the
            distance to the centroid. 
        """
        
    def onFrontier(self, index):
        """ This function takes in an index of a cell in the map and determines
            if it is part of the frontier. The frontier is defined as regions
            on the border of empty space and unknown space.
        """
        connected = False
        top = False
        bottom = False

        # Skip the known cell
        if self.current_map.data[index] < 0 or self.current_map.data[index] >= 90:
            return connected
        
        x,y = self.mapIndex2xy(index)
        
        # Check the cell above to see if its connected to a known 
        #   cell
        # x, y + 1
        if y + 1 < self.ogrid_sizeY:
            ac = self.xy2mapIndex(x,y+1)
            if self.current_map.data[ac] == -1:
                return True
            elif self.current_map.data[ac] > 90:
                return connected            


        # Check the cell below to see if its connected to a known 
        #   cell
        # x, y -1
        if y -1 >= 0:
            bc = self.xy2mapIndex(x,y-1)
            if self.current_map.data[bc] == -1:
                return True
            elif self.current_map.data[bc] > 90:
                return connected            
        


        # Check the cell to the left to see if its connected to a  
        #   known cell
        # x - 1, y
        if x -1 >= 0:
            lc = self.xy2mapIndex(x -1 ,y)
            if self.current_map.data[lc] == -1:
                return True
            elif self.current_map.data[lc] > 90:
                return connected            
            
        # Check top left
        # x - 1, y + 1
        if x - 1 >= 0 and y + 1 < self.ogrid_sizeY:
            tl = self.xy2mapIndex(x - 1, y + 1)
            if self.current_map.data[tl] == -1:
                return True
            elif self.current_map.data[tl] > 90:
                return connected            
        

        # Check bottom left
        # x - 1, y - 1
        if x - 1 >= 0 and y - 1 >= 0:
            bl = self.xy2mapIndex(x - 1, y - 1)
            if self.current_map.data[bl] == -1:
                return True
            elif self.current_map.data[bl] > 90:
                return connected            

        # Check the cell to the right to see if its connected to a 
        #   known cell
        # x + 1, y
        if x + 1 < self.ogrid_sizeX:
            rc = self.xy2mapIndex(x + 1, y)
            if self.current_map.data[rc] == -1:
                return True
            elif self.current_map.data[rc] > 90:
                return connected


        # Check top right
        # x + 1, y + 1
        if x + 1 < self.ogrid_sizeX and y + 1< self.ogrid_sizeY:
            tr = self.xy2mapIndex(x + 1, y + 1)
            if self.current_map.data[tr] == -1:
                return True
            elif self.current_map.data[tr] > 90:
                return connected            
        
        # Check bottom right
        # x + 1, y - 1
        if x + 1 < self.ogrid_sizeX and y - 1 >= 0:
            br = self.xy2mapIndex(x + 1, y - 1)
            if self.current_map.data[br] == -1:
                return True
            elif self.current_map.data[br] > 90:
                return connected            
        
        
        return connected
    
    
    def Frontier(self):
        """ This funtion finds the frontier on the map and returns a 1D vector 
            containing the indices of the cells on the frontier."""
        frontier = []
        for i in range(len(self.current_map.data)):
            if self.onFrontier(i):
                frontier.append(i)
        
        return frontier
        
        
    def blogDetection(self, frontier):
        """ This function runs the connected components algorithm on the 
            inputed frontier, which is a 1D vector containing the indices of 
            the cells on the frontier.
        """
        labels = np.zeros_like(frontier, dtype=np.int16)
        full_labels = np.ones_like(self.current_map.data, dtype=np.int16)*-1
        equiv = []
        cur_label = 0
        cntr = -1
        # Do first pass and label frontiers
        for i in range(len(frontier)):
            #print(frontier[i])
            near_frontier = self.getNearFrontier(frontier[i],frontier)
            near_labels = []
            for e in range(len(near_frontier)):
                ei = frontier.index(near_frontier[e])
                near_labels.append(labels[ei])
            nonzero_labels = [e for e in near_labels if e !=0]
            #print(str(near_labels) +" => " + str(nonzero_labels))
            
            if len(nonzero_labels) == 0:
                cur_label = cur_label + 1
                temp_label = cur_label
            elif len(nonzero_labels) == 1:
                temp_label = nonzero_labels[0]
            else:
                u_l = list(set(nonzero_labels))
                temp_label = max(nonzero_labels)
                if len(u_l) > 1:
                    if nonzero_labels not in equiv:
                        equiv.append(nonzero_labels)
            
            labels[i] = temp_label
            #print("Label " + str(frontier[i]) + " with " + str(temp_label))
                
            
        return labels, equiv

    
    def getFrontier(self):
        """ This function defines and labels the frontier. If the froniter is 
            smaller than a meter it is removed.""" 
        unlabeledFrontier = self.Frontier()
        labels, equiv = self.blogDetection(unlabeledFrontier)        
        print(equiv)
        # Initialize the frontier list
        frontier = []
        frontier_dict = {}
        for n in range(1,max(labels)+1):
            frontier_dict[n] = []
            
        # Second pass to remove equivilencies and store the frontiers
        for i in range(len(labels)):
            frontier_dict.get(labels[i]).append(unlabeledFrontier[i])
        
        print(frontier_dict)
            # Next store the index of the map into the correct row of the
            #   frontier 
        for i in range(len(equiv)):
            rel = equiv[i] # list of equal lables
            j = 0
            while j + 1 < len(rel):
                if type(frontier_dict.get(rel[j])) == list:
                    cur_list = frontier_dict.get(rel[j])
                    equal_lab = rel[j + 1]
                    self.mergeList(frontier_dict,cur_list,equal_lab)
                    frontier_dict[rel[j]] = equal_lab
                j = j + 1

        #print(frontier_dict)
        values = list(frontier_dict.values())
        frontier = [e for e in values if type(e) ==list]
        # Remove all frontiers smaller than 100 cm   
        
        return frontier
    
    def mergeList(self,mydict,cur_list,next):
        if type(mydict.get(next)) == list:
            mydict.get(next).extend(cur_list)
        else:
            self.mergeList(mydict, cur_list, mydict.get(next))    


    def getNearFrontier(self,index,frontier):
        near_frontier = []
        x,y = self.mapIndex2xy(index)
        
        if x -1 >= 0:
            if self.xy2mapIndex(x -1, y) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y))
        
        if x -1 >= 0 and y -1 >= 0:
            if self.xy2mapIndex(x -1, y -1 ) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y - 1))
                
        if x -1 >= 0 and y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x -1, y +1 ) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y + 1))
        
        if y -1 >= 0:
            if self.xy2mapIndex(x, y -1) in frontier:
                near_frontier.append(self.xy2mapIndex(x, y -1 ))
        
        if y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x, y + 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x, y + 1))
        
        if x + 1 < self.ogrid_sizeX:
            if self.xy2mapIndex(x  + 1, y) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y))
        
        if x +1 < self.ogrid_sizeX and y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x + 1, y + 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y+1))
                
        if x + 1 < self.ogrid_sizeX and y -1 >= 0:
            if self.xy2mapIndex(x + 1, y - 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y - 1))
        return near_frontier

        
    def calc_centroid(self, points):
        """ This function takes in a set of points and finds its centroid.
            
            Input:
                points - 2D array where the each row contains the X,Y location 
                    of the frontier cell
        """
        
    
    def pickBestCentroid(self, frontiers):
        """ Takes in all frontiers (as a 3D array) and choses the best frontier"""
        self.centroidX = []
        self.centroidY = []
        self.centroidIndex = []
        self.cost = []
        
        centroid_index = 0
        ...
                
        return self.bestCentroid()
            
    def updateBestCentoid(self):
        """ """

        
    def bestCentroid(self):
        """ This function takes the precalculated x/y and cost values of the 
            centroid and picks the returns the index to the cell that has the minimum cost"""
            
        
        
    def makeMarker(self, centroidX, centroidY, ID, action = Marker.ADD,  new=True):
        """ Creates a marker on RVIZ for the centroid of the frontier"""
        

            
    def makelineMarker(self, XY, ID, action = Marker.ADD, new=True):
        """ Creates a line marker on RVIZ for the entire frontier"""

            
    def removeAllMarkers(self):
        """This function removes all markers from rviz"""

        
    def pose_callback(self, pose_msg):
        """ callback to handle odometry messages"""        
        self.position.time = rospy.get_rostime().to_sec()
        self.position.x = pose_msg.x
        self.position.y = pose_msg.y
        self.position.theta = pose_msg.theta
            
        # print "x: " + str(self.position.x) + " y: " + str(self.position.y) + " theta: " + str(self.position.theta)

    
    def newPose(self, x, y, rot_z=0.1,rot_w=0.1):
        """Takes in the new waypoint and sets the new goal position"""        
        new = PoseStamped()
        new.header.seq=1
        new.header.stamp.secs = rospy.get_rostime().to_sec()
        
        new.header.frame_id= 'map'
        
        # Positions in the map
        ...
        
        # Publish the new position
        self.cmd_pose.publish(new)
    
    def angle_traveled(self):
        """ Calculates the angle traveled between the current and previous 
            angle.
        """
        diff = self.cur_odom.theta-self.prev_odom.theta
        # If there is a big jump, then it must have crossed the -180/180 
        #  boundary.

        
    def spin360(self):
        self.done_spinning = False
        """ Spins the robot 360 degrees w.r.t the odometer"""
        
        self.done_spinning = True
        self.start_spin = False

    def move2frontier(self,X, Y):
        """ Navigate to the centroid of the chosen frontier"""
                
    def run(self):
        """ Runs the frontier based exploration. """
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec()-start) < 1:
            self.r.sleep()
        
        self.start_spin = True
        print('Spinning')
        self.spin360()
        
        while not rospy.is_shutdown(): # removed timing info
            try:
                # self.get_current_position()
                # If the robot is currently spinning, check if it completed a 
                #   rotation and if it has, find the frontiers.
               pass
                        
            except KeyboardInterrupt:
                print('Exiting')
            
        # At the end remove all rviz markers
        self.removeAllMarkers()
        for i in range(5):
            self.r.sleep()
        print 'exiting'
        
    def __init__(self):
        """ Initialize """
        rospy.init_node('Frontier_exploration')
        
        self.listener = tf.TransformListener()        
        
        # Get the parameters for the grid
        self.ogrid_sizeX = rospy.get_param('x_size', 500)
        self.ogrid_sizeY = rospy.get_param('y_size', 500)
        self.grid_size = rospy.get_param('grid_size', 0.05) # in meters/cell (5cm)
        
        # Sensor Meta data
        self.min_range = rospy.get_param('max_range',0.4)
        self.max_range = rospy.get_param('max_range',6.0)
        
        # reliability
        self.p_measurement_given_occupied = rospy.get_param('p_z|occ',0.9)
        self.p_measurement_given_notOccupied = rospy.get_param('p_z|notOcc',0.3)
        
        # Initialize some varables to store the objects to be published/subscribed
        self.position = Position()
        self.robotState = RobotState()
        self.frontierCentroid = Position()
        self.markerArray = MarkerArray()
        self.lineMarker = MarkerArray()
        self.cur_odom = Position()
        self.prev_odom = Position()
        
        self.current_map = OccupancyGrid()
        self.meta = MapMetaData()
        self.new_pose = PoseStamped()
        
        # Rotation Booleans
        self.start_spin = True
        self.done_spinning = True
        self.new_cmd = False
        
        self.unreachable_frontiers = []
        self.reached_centroids=[]
        self.WPT_ID = 0
        self.angle_trav = 0
        self.move_base_status = [0]
        
        self.origin = Position()
        
        self.r = rospy.Rate(50)
        
        # publishers for OccupancyGrid and move base messages
        self.marker_pub = rospy.Publisher('/centroid_marker', MarkerArray, queue_size=100)
        self.lineMarker_pub = rospy.Publisher('/frontier_marker', MarkerArray, queue_size=100)
        self.cmd_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=100)
        
        # subscribers for odometry and position (/move_base/feedback) messages
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.sub_map = rospy.Subscriber('/frontier_map', OccupancyGrid, self.costmap_callback)
        self.sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        self.base_pos_sub = rospy.Subscriber("/pose2D", Pose2D, self.pose_callback)
                

        self.run()
        
        return

if  __name__=="__main__":
    l = Frontier_Based_Exploration()

