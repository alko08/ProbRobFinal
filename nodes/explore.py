#!/usr/bin/env python3
"""
 The explore node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from random import randrange
import time

class Explore:

    def __init__(self):
        """ Initialize environment """
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Simple Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.logdebug("move_base is ready") 

        self.x = 0
        self.y = 0
        self.completion = 0

        # Initialize subscribers:
        self.map = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.map_width = None
        self.map_height = None
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.count = 0
        rospy.sleep(2)

    def map_callback(self, data):
        """ Callback function for map subscriber.
        Subscribes to /map to get the OccupancyGrid of the map.
        """
        self.map = data.data
        self.resolution = data.info.resolution
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.map_width = data.info.width
        self.map_height = data.info.height

        valid = False

        while not valid:
            map_size = randrange(len(self.map))

            if self.map[map_size] == -1:
                edges = self.check_neighbors(data, map_size)
                if edges:
                    valid = True

        col = map_size % self.map_width
        row = map_size // self.map_width

        self.x = col * self.resolution + self.origin_x
        self.y = row * self.resolution + self.origin_y

        if self.completion % 2 == 0:
            self.completion += 1
            self.set_goal()

    def set_goal(self):
        """ Set goal position for move_base. """
        rospy.logdebug("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.logdebug(f"goal: {goal.target_pose.pose.position.x, goal.target_pose.pose.position.y}")
        self.move_base.send_goal(goal, self.goal_status)

    def goal_status(self, status, result):
        """ Check the status of a goal - goal reached, aborted, or rejected. """
        self.completion += 1

        # Goal reached
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded")

        # Goal aborted
        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted")

        # Goal rejected
        elif status == GoalStatus.REJECTED:
            rospy.loginfo("Goal rejected")

    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map. """
        unknowns = 0
        obstacles = 0

        for dx in range(-3, 4):
            for dy in range(-3, 4):
                neighbor_idx = map_size + dx * self.map_width + dy
                try:
                    if data.data[neighbor_idx] == -1:
                        unknowns += 1
                    elif data.data[neighbor_idx] > 65:  # Adjusted threshold for obstacles
                        obstacles += 1
                except IndexError:
                    pass

        if unknowns > 0 and obstacles < 2:
            return True
        return False

def main():
    """ The main() function """
    rospy.init_node('explore', log_level=rospy.DEBUG)
    Explore()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass