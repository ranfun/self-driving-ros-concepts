#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
from rrt import find_path_RRT
import cv2
import numpy as np

# Define global variables
current_map = None
map_resolution = None
map_origin = None


def get_index_from_coordinates(x_coord, y_coord):
    # Calculate the x and y indices in the occupancy map using the provided equations
    x_index = int(round((x_coord - map_origin.position.x) / map_resolution))
    y_index = int(round((y_coord - map_origin.position.y) / map_resolution))
    return x_index, y_index


def start_goal_callback(msg):
    global current_map, map_resolution, map_origin

    # Extract the start and goal positions from the message
    x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data

    # Convert the real-world start and goal positions to indices in the occupancy map
    x_start_index, y_start_index = get_index_from_coordinates(x_start_real, y_start_real)
    x_goal_index, y_goal_index = get_index_from_coordinates(x_goal_real, y_goal_real)

    # Find the path using the RRT algorithm
    start, goal = ([x_start_index, y_start_index], [x_goal_index, y_goal_index])
    path,graph= find_path_RRT(start, goal, current_map)

    # Publish the resulting trajectory on the /trajectory topic
    trajectory = Float64MultiArray()
    for point in path:
        # Convert the indices back to real-world coordinates
        x_coord = point[0] * map_resolution + map_origin.position.x 
        y_coord = point[1] * map_resolution + map_origin.position.y
        trajectory.data.extend([x_coord, y_coord])
    trajectory_pub.publish(trajectory)


def map_callback(msg):
    global current_map, map_resolution, map_origin

    # Convert the OccupancyGrid message to a 2D numpy array
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))
    current_map = data

    # Save the map resolution and origin for use in get_index_from_coordinates()
    map_resolution = msg.info.resolution
    map_origin = msg.info.origin


if __name__ == '__main__':
    rospy.init_node('rrt_node')

    # Subscribe to the /map and /start_goal topics
    map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
    start_goal_sub = rospy.Subscriber('/start_goal', Float64MultiArray, start_goal_callback)

    # Initialize the /trajectory publisher
    trajectory_pub = rospy.Publisher('/trajectory', Float64MultiArray, queue_size=10)

    rospy.spin()