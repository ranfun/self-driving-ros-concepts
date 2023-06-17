#!/usr/bin/env python

import rospy
import random
import math
from geometry_msgs.msg import Twist

def swim_node():
    rospy.init_node('swim_node', anonymous=True)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel_msg = Twist()

    # set the time for the turtle to swim in a figure-8 motion
    swim_time = 22.0
    start_time = rospy.Time.now().to_sec()

    # initialize the circle parameters
    center_x = random.uniform(-5, 5)
    center_y = random.uniform(-5, 5)
    radius = random.uniform(1, 5)

    while not rospy.is_shutdown():
        # calculate the time elapsed
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        # calculate the velocities for the circle motion
        x = center_x + radius * math.cos(elapsed_time/5.0)
        y = center_y + radius * math.sin(elapsed_time/5.0)
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.6 * (radius/abs(radius)) # change direction when reaching starting point

        # update Twist message with new velocities
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        vel_pub.publish(vel_msg)

        if elapsed_time >= swim_time/2:
            break

        rate.sleep()

    # update circle parameters for the second circle
    center_x = x
    center_y = y
    radius = random.uniform(1, 3)

    while not rospy.is_shutdown():
    
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time

        x = center_x + radius * math.cos(elapsed_time/5.0 + math.pi)
        y = center_y + radius * math.sin(elapsed_time/5.0 + math.pi)
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = -0.6 * (radius/abs(radius)) 

        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        vel_pub.publish(vel_msg)

        if elapsed_time >= swim_time:
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        swim_node()
    except rospy.ROSInterruptException:
        pass