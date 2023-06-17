#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import click

def my_teleop():
	rospy.init_node("my_teleop_node",anonymous=False)
	pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
	rate=rospy.Rate(50)
	move_cmd=Twist()

	while not rospy.is_shutdown():
		c=click.getchar()
		if c=='w':
			move_cmd.angular.z=0.0
			move_cmd.linear.x=1.0
		elif c=='s':
			move_cmd.angular.z=0.0
			move_cmd.linear.x=-1.0
		elif c=='a':
			move_cmd.linear.x=0.0
			move_cmd.angular.z=1.0
		elif c=='d':
			move_cmd.linear.x=0.0
			move_cmd.angular.z=-1.0
		else:
			move_cmd.linear.x=0.0
			move_cmd.angular.z=0.0
		pub.publish(move_cmd)
		rate.sleep()

if __name__=="__main__":
	try:
		my_teleop()
	except rospy.ROSInterruptException:
		pass