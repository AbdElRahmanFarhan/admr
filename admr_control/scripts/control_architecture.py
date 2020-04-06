#! /usr/bin/env python
import rospy
import numpy as np
from math import pi
from state_feedback_controller import *
from std_msgs.msg import Float32MultiArray

x_path=np.array([])
y_path=np.array([])

def callback_x(x_path_msg):
	global x_path
	x_path=x_path_msg.data

def callback_y(y_path_msg):
	global y_path
	y_path=y_path_msg.data

def main():
	try:
		global x_path
		global y_path
		rospy.init_node("control_architecture")
		rospy.Subscriber("/desired_path_x", Float32MultiArray, callback_x)
		rospy.Subscriber("/desired_path_y", Float32MultiArray, callback_y)
		rospy.sleep(1)
		sz = len(x_path)
		# control parameters
		linear_const = 0.04
		angular_const = 0.7
		distance_tolerance = 0.06
		max_linear_vel = 0.1
		min_linear_vel = 0.05
		max_angular_vel = 1.5
		# initilize controller
		diff_robot = state_feedback_controller( "diff_wheeled_robot",x_path,y_path,linear_const,angular_const,distance_tolerance,max_linear_vel, min_linear_vel, max_angular_vel)
		# go to next point
		for i in range(sz):
			if (i>0):
				diff_robot.gotoGoal(x_path[i],y_path[i])
				
	except rospy.ROSInterruptException:
		print "Error Communication: Exception!"
if __name__ == '__main__':
	main()		
