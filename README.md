# Autonomous Vehicle Concepts with ROS
This repository contains a collection of code examples and implementations showcasing various concepts used in autonomous vehicles and self-driving cars, utilizing the Robot Operating System (ROS). The repository focuses on fundamental components such as localization, mapping, path planning, and motion control.

## Programs and Functionalities
## turtlesim_teleop
turtlesim_teleop.py: This program allows the user to control the movement of a turtle using customizable keys. Users can define their preferred keys for forward, backward, left, and right movements.

## swim
swim.py: In this program, the turtle moves in a figure-eight (8) shape, demonstrating a predefined swimming motion.
swim_to_goal.py: The turtle in this program swims towards a goal point specified by the user.

## PID_Controller
As the name suggests, this node implements a Proportional-Integral-Derivative (PID) controller. Users can configure the controller's parameters (kp, ki, kd) to their liking, allowing the gazebo bot to move towards a specific inputted point.

## Path_Planning

This program implements the Rapidly-Exploring Random Tree (RRT) algorithm. Users can provide a map and specify a start and goal point. The algorithm generates a trajectory in the form of a list of points representing the planned path.

## Motion_Planning

This program combines the functionality of path planning and the PID controller. Users are prompted to provide initial and final points, and the gazebo bot is guided to the destination based on the generated trajectory and PID control.

## Motion_Planning_Imitation_Learning
This section focuses on motion planning using neural network-based controllers.

data_collection.py: This script is used to collect a dataset with a 1:1 correspondence, which is necessary for training a neural network.
check_layers.py: This script returns the names of the layers in the neural network. Users need to adjust the layers in the NN_Controller.py script accordingly.
motion_planner.py: Users can input a point to which they want the bot to move using this program.
NN_Controller.py: This script implements a neural network-based motion controller. It utilizes the training data to guide the gazebo bot towards the specified point. 

I have attached my model for reference as well. Data is collected by using the 2D Navigation tool on RViz and gazebo, and running the data_collection script parallely.

# Getting Started
Clone the repository to your local machine using the following command:
```
git clone https://github.com/your-username/self-driving-ros-concepts.git
```
Make sure you have the required dependencies installed, such as ROS and any additional packages used by the programs.
Follow the instructions in each program's description to run and interact with the respective functionalities.
Feel free to modify the README description as needed, add installation instructions, or provide any additional information that might be useful for users of your repository.
