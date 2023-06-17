#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

class MotionPlanner:
    def __init__(self):
        rospy.init_node('Motion_Planner', anonymous=True)
        self.reference_pose_publisher = rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)

    def get_goal_pose(self):
        goal_x = float(input("Set your x goal: "))
        goal_y = float(input("Set your y goal: "))
        return [goal_x, goal_y]

    def publish_goal_pose(self, goal_pose):
        reference_pose_msg = Float64MultiArray()
        reference_pose_msg.data = goal_pose
        self.reference_pose_publisher.publish(reference_pose_msg)

if __name__ == '__main__':
    try:
        motion_planner = MotionPlanner()
        goal_pose = motion_planner.get_goal_pose()
        motion_planner.publish_goal_pose(goal_pose)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass