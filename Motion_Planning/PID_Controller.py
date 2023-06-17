#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
import tf
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import os
import time

# global reference_pose

class PID:
    def __init__(self):
        # Creates a node with name 'PID_Control_US' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('PID_Control_US', anonymous=False)
        # Subscribe to the reference pose topic
        self.reference_pose_sub = rospy.Subscriber('/reference_pose', Float64MultiArray, self.reference_pose_callback)
        # Publisher which will publish to the topic 'cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel',Twist, queue_size=5)
        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type ModelStates is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states',ModelStates, self.update_pose)
        ############### AGENT'S POSE in the gazebo environment
        #######################
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0
        ############### Time discretization for the PID
        #############################
        self.time_discr = 0.001
        ############### PID parameters for linear velocity
        ############################
        self.error_prior_linear = 0
        self.integral_prior_linear = 0
        self.Kp_gain_linear = 5
        self.Ki_gain_linear = 0.1
        self.Kd_gain_linear = 0
        ############### PID parameters for angular velocity
        ############################
        self.error_prior_angular = 0
        self.integral_prior_angular = 0
        self.Kp_gain_angular = 1
        self.Ki_gain_angular = 0.1
        self.Kd_gain_angular = 0
        self.rate = rospy.Rate(10)
    
    def update_pose(self, data):
        """Callback function which is called when a new message of type ModelStates
        is
        received by the subscriber."""
        self.pose_x = round(data.pose[1].position.x, 4)
        self.pose_y = round(data.pose[1].position.y, 4)
        # self.pose_theta = data.pose[1].orientation.z
        rotations_rads = euler_from_quaternion([data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w])
        self.pose_theta = rotations_rads[2]
    
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) + pow((goal_pose[1] - self.pose_y), 2))
    
    def steering_angle(self, goal_pose):
        return atan2(goal_pose[1] - self.pose_y, goal_pose[0] - self.pose_x)
    
    def proportional_linear_vel(self, goal_pose):
        return min(self.Kp_gain_linear * self.euclidean_distance(goal_pose), 0.1)

    def derivative_linear_vel(self, goal_pose):
        # update the error prior
        error = self.euclidean_distance(goal_pose)
        derivative_linear_vel = (error - self.error_prior_linear)/self.time_discr
        self.error_prior_linear = error
        return self.Kd_gain_linear * derivative_linear_vel
        
    def integral_linear_vel(self, goal_pose):
        # update the integral prior
        integral_linear_vel=self.integral_prior_linear+self.euclidean_distance(goal_pose)*self.time_discr
        self.integral_prior_linear += integral_linear_vel
        return self.Ki_gain_linear * integral_linear_vel
        
    def proportional_angular_vel(self, goal_pose):
        error_angle = self.steering_angle(goal_pose) - self.pose_theta
        return min(self.Kp_gain_angular * error_angle, 0.1)
        
    def derivative_angular_vel(self, goal_pose):
        # update the error prior
        error_angle = self.steering_angle(goal_pose) - self.pose_theta
        derivative_angular_vel = (error_angle - self.error_prior_angular)/self.time_discr
        self.error_prior_angular = error_angle
        return self.Kd_gain_linear * derivative_angular_vel
        
    def integral_angular_vel(self, goal_pose):
        # update the integral prior
        integral_angular_vel=self.integral_prior_angular+self.steering_angle(goal_pose)*self.time_discr
        self.integral_prior_angular += integral_angular_vel
        return self.Ki_gain_linear * integral_angular_vel
        
    def PID_controller_linear(self, goal_pose):
        pid_control_linear = self.proportional_linear_vel(goal_pose) + self.derivative_linear_vel(goal_pose) + self.integral_linear_vel(goal_pose)
        # set error priors for linear to 0
        # self.error_prior_linear = 0
        self.integral_prior_linear = 0
        return pid_control_linear
        
    def PID_controller_angular(self, goal_pose):
        pid_control_angular = self.proportional_angular_vel(goal_pose) + self.derivative_angular_vel(goal_pose) + self.integral_angular_vel(goal_pose)
        # update error priors for angular to 0
        # self.error_prior_angular = 0
        self.integral_prior_angular = 0
        return pid_control_angular
    
    def reference_pose_callback(self,msg):
        # print('Im here')
        self.reference_pose = msg
        goal_pose_x, goal_pose_y, goal_pose_theta = self.reference_pose.data
        # print('Im here2')
        goal_pose = [goal_pose_x, goal_pose_y, goal_pose_theta]
        distance_tolerance = 0.05
        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # PID controller.
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.PID_controller_linear(goal_pose)
            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.PID_controller_angular(goal_pose)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()

            if self.euclidean_distance(goal_pose) < distance_tolerance:
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)
                # set error priors for linear and angular to 0
                self.error_prior_linear = 0
                self.integral_prior_linear = 0
                self.error_prior_angular = 0
                self.integral_prior_angular = 0
                break

    def move2goal(self):
        # print('Im here3')
        while not rospy.is_shutdown():
            # Wait until the reference pose message is received
            # if self.reference_pose is None:
            #     continue
            # print('Im here 4')
            # Get the goal pose from the reference pose message
            goal_pose_x, goal_pose_y, goal_pose_theta = self.reference_pose.data
            # print('Im here2')
            goal_pose = [goal_pose_x, goal_pose_y, goal_pose_theta]
            distance_tolerance = 0.05
            vel_msg = Twist()
            while self.euclidean_distance(goal_pose) >= distance_tolerance:
                # PID controller.
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.PID_controller_linear(goal_pose)
                # Angular velocity in the z-axis.
                vel_msg.angular.z = self.PID_controller_angular(goal_pose)
                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
                # Publish at the desired rate.
                self.rate.sleep()
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            # set error priors for linear and angular to 0
            self.error_prior_linear = 0
            self.integral_prior_linear = 0
            self.error_prior_angular = 0
            self.integral_prior_angular = 0



if __name__ == '__main__':
    try:
        x = PID()
        x.move2goal()
    except:
        rospy.ROSInterruptException
    
    rospy.spin()