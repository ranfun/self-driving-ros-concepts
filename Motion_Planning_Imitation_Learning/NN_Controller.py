#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
from math import radians, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import h5py  # You may need to install it first, just open a terminal and do "pip install h5py"
import numpy as np

class YourNetwork:
    def __init__(self):
        # Define the dimensions of the network
        self.input_size = 5
        self.hidden_size1 = 128
        self.hidden_size2 = 128
        self.hidden_size3 = 64
        self.hidden_size4 = 64
        self.output_size = 2

        # Initialize weights
        self.W1 = np.zeros((self.input_size, self.hidden_size1))
        self.b1 = np.zeros(self.hidden_size1)
        self.W2 = np.zeros((self.hidden_size1, self.hidden_size2))
        self.b2 = np.zeros(self.hidden_size2)
        self.W3 = np.zeros((self.hidden_size2, self.hidden_size3))
        self.b3 = np.zeros(self.hidden_size3)
        self.W4 = np.zeros((self.hidden_size3, self.hidden_size4))
        self.b4 = np.zeros(self.hidden_size4)
        self.W5 = np.zeros((self.hidden_size4, self.output_size))
        self.b5 = np.zeros(self.output_size)

    def forward(self, x):
        # Perform forward propagation
        h1 = np.dot(x, self.W1) + self.b1
        h1_relu = np.maximum(0, h1)
        h2 = np.dot(h1_relu, self.W2) + self.b2
        h2_relu = np.maximum(0, h2)
        h3 = np.dot(h2_relu, self.W3) + self.b3
        h3_relu = np.maximum(0, h3)
        h4 = np.dot(h3_relu, self.W4) + self.b4
        h4_relu = np.maximum(0, h4)
        y_pred = np.dot(h4_relu, self.W5) + self.b5
        return y_pred

    def set_weights(self, weights1, biases1, weights2, biases2, weights3, biases3, weights4, biases4, weights5, biases5):
        # Set the loaded weights to the network
        self.W1 = weights1
        self.b1 = biases1
        self.W2 = weights2
        self.b2 = biases2
        self.W3 = weights3
        self.b3 = biases3
        self.W4 = weights4
        self.b4 = biases4
        self.W5 = weights5
        self.b5 = biases5


class NN:

    def __init__(self):
        # Creates a node with name 'NN_Control_US' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('NN_Control_US', anonymous=False)

        # Publisher which will publish to the topic 'cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type ModelStates is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.reference_pose_subscriber = rospy.Subscriber('/reference_pose', Float64MultiArray, self.move2goal)

        # Load the H5 weights
        with h5py.File('model.h5', 'r') as f:
            # Access the weights dataset for the specific layer
            # Layer 1:
            weights1 = f['model_weights']['dense']['dense']['kernel:0'][:]
            biases1 = f['model_weights']['dense']['dense']['bias:0'][:]

            # Layer 2:
            weights2 = f['model_weights']['dense_1']['dense_1']['kernel:0'][:]
            biases2 = f['model_weights']['dense_1']['dense_1']['bias:0'][:]

            # Layer 3:
            weights3 = f['model_weights']['dense_2']['dense_2']['kernel:0'][:]
            biases3 = f['model_weights']['dense_2']['dense_2']['bias:0'][:]

            # Layer 4:
            weights4 = f['model_weights']['dense_3']['dense_3']['kernel:0'][:]
            biases4 = f['model_weights']['dense_3']['dense_3']['bias:0'][:]

            # Layer 5:
            weights5 = f['model_weights']['dense_4']['dense_4']['kernel:0'][:]
            biases5 = f['model_weights']['dense_4']['dense_4']['bias:0'][:]

        # Create an instance of the network
        self.network = YourNetwork()

        # Set the loaded weights to the network
        self.network.set_weights(weights1, biases1, weights2, biases2, weights3, biases3,
                                 weights4, biases4, weights5, biases5)

        ############### AGENT'S POSE in the gazebo environment #######################
        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

        ############### Time discretization for the NN  #############################
        self.time_discr = 0.001

        self.rate = rospy.Rate(10)

        rospy.spin()

    def update_pose(self, data):
        """Callback function which is called when a new message of type ModelStates is
        received by the subscriber."""

        self.pose_x = round(data.pose[1].position.x, 4)
        self.pose_y = round(data.pose[1].position.y, 4)
        rotations_rads = euler_from_quaternion(
            [data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z,
             data.pose[1].orientation.w])
        self.pose_theta = rotations_rads[2]

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - self.pose_x), 2) +
                    pow((goal_pose[1] - self.pose_y), 2))

    def move2goal(self, goal):
        goal_pose_x = goal.data[0]
        goal_pose_y = goal.data[1]

        goal_pose = [goal_pose_x, goal_pose_y]

        distance_tolerance = 0.2
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # NN controller.
            output_array = self.network.forward(
                np.array([[goal_pose_x, goal_pose_y, self.pose_x, self.pose_y, self.pose_theta]]))
            vel_msg.linear.x = output_array[0][0]
            vel_msg.angular.z = output_array[0][1]

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        NN()
    except rospy.ROSInterruptException:
        pass