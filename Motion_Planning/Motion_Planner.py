#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point
from math import radians, sqrt, pow, pi, atan2

# global traj_plus_ten

def update_pose(data):
        """Callback function which is called when a new message of type ModelStates
        is
        received by the subscriber."""
        global pose_x
        global pose_y
        pose_x = round(data.pose[1].position.x, 4)
        pose_y = round(data.pose[1].position.y, 4)
        # print(pose_x)
        # pose_theta = data.pose[1].orientation.z

# Publish to reference pose
pid_pub=rospy.Publisher('/reference_pose', Float64MultiArray, queue_size=10)
pose_subscriber = rospy.Subscriber('/gazebo/model_states',ModelStates, update_pose)

def get_user_input():
    # Get the goal point from the user
    goal_x = float(input("Enter the x-coordinate of the goal point: "))
    goal_y = float(input("Enter the y-coordinate of the goal point: "))

    # Return the goal point as a tuple
    return (-10,-10), (goal_x, goal_y)


def set_model_state():
    # Initialize ROS node
    # rospy.init_node('set_model_state_node')

    # Wait for the service to become available
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Set the model state
    model_state_msg = ModelState()
    model_state_msg.model_name = 'turtlebot3_burger'
    model_state_msg.pose.position = Point(0, 0, 0)  # set position to (-10,-10,0)
    model_state_msg.pose.orientation.w = 1  # set orientation to upright
    set_model_state_srv(model_state_msg)

def compute_angle(current_point_idx):
    x1, y1 = traj_plus_ten[current_point_idx:current_point_idx+2]
    x2, y2 = traj_plus_ten[current_point_idx+2:current_point_idx+4]
    if abs(x2-x1)<0.05:
        return 1.57
    elif abs(y2-y1)<0.05:
        return 0
    else:
        return atan2(y2 - y1, x2 - x1)
    # Have a look at different quadrant functions. 

# def get_distance(point1, point2):
#     return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def euclidean_distance(goal_pose):
    """Euclidean distance between current pose and the goal."""
    return sqrt(pow((goal_pose[0] - pose_x), 2) + pow((goal_pose[1] - pose_y), 2))

def send_reference_pose(traj):
    # Initialize the current point index
    global current_point_idx
    global traj_plus_ten
    traj_plus_ten =[]
    current_point_idx = 2


    # Continuously send reference pose until final 2 points in trajectory are reached
    while current_point_idx < len(traj):
        # Extract the current path and compute the current angle
        for x in traj:
            traj_plus_ten.append(x + 10)
        current_path = traj_plus_ten[current_point_idx:current_point_idx+2]
        current_angle = compute_angle(current_point_idx)
        
        # Publish the current coordinates and angle to the PID controller
        pid_msg = Float64MultiArray()
        pid_msg.data = [current_path[0], current_path[1], current_angle]
        pid_pub.publish(pid_msg)
        
        # Wait until the robot reaches the current point in the trajectory
        while True:
            if euclidean_distance(current_path) < 0.05:
                # Increment the current point index and break out of the loop
                print("Reached ({},{})".format(traj[current_point_idx],traj[current_point_idx+1]))
                # print("Reached checkpoint!")
                current_point_idx += 2
                break
            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('motion_planner_node')
    set_model_state()

    # Create a publisher for the start/goal points
    start_goal_pub = rospy.Publisher('/start_goal', Float64MultiArray, queue_size=10)

    # Create a subscriber for the trajectory
    def trajectory_callback(msg):
        # Print the received trajectory
        print("Received trajectory:", msg.data)
        global traj
        traj=msg.data    
        # Call the function to send the reference pose to the PID controller
        send_reference_pose(traj)

    trajectory_sub = rospy.Subscriber('/trajectory', Float64MultiArray, trajectory_callback)

    # Ask the user for start and goal points
    start, goal = get_user_input()

    # Publish the start and goal points
    start_goal_msg = Float64MultiArray()
    start_goal_msg.data = list(start) + list(goal)
    start_goal_pub.publish(start_goal_msg)

    rospy.spin()