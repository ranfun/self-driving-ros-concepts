#!/usr/bin/env python

import csv
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped,Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion

# Initialize the ROS node
rospy.init_node('data_collection_node', anonymous=True)

# Create CSV files for storing data
# model_states_file = open('model_states.csv', 'w')
# model_states_writer = csv.writer(model_states_file)
# model_states_writer.writerow(['position_x', 'position_y', 'orientation_z'])

# cmd_vel_file = open('cmd_vel.csv', 'w')
# cmd_vel_writer = csv.writer(cmd_vel_file)
# cmd_vel_writer.writerow(['linear_x', 'angular_z'])

# move_base_file = open('move_base.csv', 'w')
# move_base_writer = csv.writer(move_base_file)
# move_base_writer.writerow(['goal_x', 'goal_y'])

data_file=open('data.csv','w')
data_writer=csv.writer(data_file)
data_writer.writerow(['goal_x', 'goal_y','position_x', 'position_y', 'orientation_z','linear_x', 'angular_z'])

goal_x=0
goal_y=0
pose_x=0
pose_y=0
pose_theta=0
linear_x=0
angular_z=0

# Callback function for /gazebo/model_states topic
def model_states_callback(data):
    # Filter and extract data from /gazebo/model_states topic
    global pose_x
    global pose_y
    global pose_theta
    pose_x = round(data.pose[2].position.x, 4)
    pose_y = round(data.pose[2].position.y, 4)
    rotations_rads = euler_from_quaternion([data.pose[2].orientation.x, data.pose[2].orientation.y, data.pose[2].orientation.z, data.pose[2].orientation.w])
    pose_theta = rotations_rads[2]

    # Write model_states data to CSV
    # model_states_writer.writerow([pose_x, pose_y, pose_theta])

# Callback function for /cmd_vel topic
def cmd_vel_callback(data):
    # Extract and write data from /cmd_vel topic
    global linear_x
    global angular_z
    linear_x = data.linear.x
    angular_z = data.angular.z

    # Write cmd_vel data to CSV
    # cmd_vel_writer.writerow([linear_x, angular_z])

# Callback function for /move_base/goal topic
def move_base_goal_callback(data):
    # Extract and write data from /move_base/goal topic
    global goal_x
    global goal_y
    goal_x = data.goal.target_pose.pose.position.x
    goal_y = data.goal.target_pose.pose.position.y

    # Write move_base data to CSV
    # move_base_writer.writerow([goal_x, goal_y])

# Function to write excel sheet
def xl_writer():
    data_writer.writerow([goal_x, goal_y,pose_x, pose_y, pose_theta,linear_x,angular_z])

# Subscribe to the /gazebo/model_states topic
model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

# Subscribe to the /cmd_vel topic
cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

# Subscribe to the /move_base/goal topic
move_base_goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, move_base_goal_callback)

# Set the desired publishing rate
publish_rate = rospy.Rate(10)  # Publish at 10 Hz

# Main loop
while not rospy.is_shutdown():
    xl_writer()
    # Sleep to control the publishing rate
    publish_rate.sleep() 

# Close CSV files
model_states_file.close()
cmd_vel_file.close()
move_base_file.close()