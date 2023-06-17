#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def get_user_input():
    # Get the start and goal points from the user
    start_x = float(input("Enter the x-coordinate of the start point: "))
    start_y = float(input("Enter the y-coordinate of the start point: "))
    goal_x = float(input("Enter the x-coordinate of the goal point: "))
    goal_y = float(input("Enter the y-coordinate of the goal point: "))

    # Return the start and goal points as a tuple
    return (start_x, start_y), (goal_x, goal_y)


if __name__ == '__main__':
    rospy.init_node('motion_planner_node')

    # Create a publisher for the start/goal points
    start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)

    # Create a subscriber for the trajectory
    def trajectory_callback(msg):
        # Print the received trajectory
        print("Received trajectory:", msg.data)
    trajectory_sub = rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)

    # Ask the user for start and goal points
    start, goal = get_user_input()

    # Publish the start and goal points
    start_goal_msg = Float64MultiArray()
    start_goal_msg.data = list(start) + list(goal)
    start_goal_pub.publish(start_goal_msg)

    rospy.spin()