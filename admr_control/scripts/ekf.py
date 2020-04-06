#! /usr/bin/env python

import numpy as np
import math
import rospy
from std_msgs.msg 			import Float32MultiArray
from geometry_msgs.msg  	import Twist

ds = float()
d_th = float()
filtered_data =  Float32MultiArray()
noisy_data =  Float32MultiArray()
ks = 0.0001
k_th = 0.001
x_prior = np.zeros((3,1))
p_prior = np.eye(3)
Q = np.eye(3)*0.00001

def control_action_callback(twist):
	global ds, d_th
	dt = 0.1
	lin_vel = twist.linear.x
	ds = lin_vel*dt
	ang_vel = twist.angular.z
	d_th = ang_vel*dt
	
def ekf_callback(odom_raw):
	global ds, d_th, Q, ks, k_th, x_prior, p_prior, ekf, filtered_data, noisy_data
	odom_raw_states = odom_raw.data
	odom_noisy_states = np.random.multivariate_normal(odom_raw_states, Q)
	g = calc_g(ds, d_th, odom_noisy_states, Q, ks, k_th)
	jacob = cal_jacob(ds, d_th, odom_noisy_states, Q, ks, k_th)
	R = calc_R(ds, d_th, odom_noisy_states, Q, ks, k_th)
	z = odom_noisy_states.reshape((3,1))
	noisy_data.data = z
	odom_noisy_pub.publish(noisy_data)
	x_updated, p_updated = ekf(g, jacob, R, z, Q)
	filtered_data.data = x_updated.reshape((3,1))
	ekf_pub.publish(filtered_data)
	x_prior = x_updated
	p_prior = p_updated
	
	
def calc_g(ds, d_th, odom_noisy_states, cov, ks, k_th):
	g = np.zeros((3, 1))
	g[0] =	math.cos(odom_noisy_states[2]+d_th)*ds
	g[1] =	math.sin(odom_noisy_states[2]+d_th)*ds
	g[2] = 	d_th
	return g
	
def calc_R(ds, d_th, odom_noisy_states, cov, ks, k_th):
	process_cov = np.zeros((2,2))
 	process_cov[0,0] = ks*abs(ds)
 	process_cov[1,1] = k_th*abs(d_th)
 	Jt = np.zeros((3, 2))
 	Jt[0,0] = math.cos(odom_noisy_states[2]+d_th)
 	Jt[0,1] = -1*ds*math.sin(odom_noisy_states[2]+d_th)
 	Jt[1,0] = math.sin(odom_noisy_states[2]+d_th)
 	Jt[1,1] = ds*math.cos(odom_noisy_states[2]+d_th)
 	Jt[2,0] = 0
 	Jt[2,1] = 1
 	R = np.dot(Jt, np.dot(process_cov, Jt.reshape((2,3))))
 	return R
	
def cal_jacob(ds, d_th, odom_noisy_states, cov, ks, k_th):
	jacob = np.eye(3)
	jacob[0,2] = -1*math.sin(odom_noisy_states[2]+d_th)*ds
	jacob[1,2] = math.cos(odom_noisy_states[2]+d_th)*ds
	return jacob
	
	
def ekf(g,jacob,R,z,Q):
	global x_prior, p_prior
	x_hat = x_prior+g
 	p_hat = np.dot(np.dot(jacob, p_prior), jacob.T)+R
 	kt = np.dot(p_hat , np.linalg.inv(p_hat+Q))
 	x_updated = x_hat+np.dot(kt,(z-x_hat))
 	p_updated = np.dot(p_hat, (np.eye(3)-kt))
 	return x_updated, p_updated

rospy.init_node("ekf")
rospy.Subscriber('/odom_raw_sync', Float32MultiArray, ekf_callback)
rospy.Subscriber('/cmd_vel', Twist, control_action_callback)
ekf_pub = rospy.Publisher('/filtered_odom', Float32MultiArray, queue_size=5)
odom_noisy_pub = rospy.Publisher('/noisy_odom', Float32MultiArray, queue_size=5)
rospy.spin()
				
