#! /usr/bin/env python
import math
import time
import rospy
import rosbag
import numpy as np
from gazebo_msgs.msg    	import ModelState
from gazebo_msgs.srv    	import GetModelState
from gazebo_msgs.srv    	import ApplyJointEffort
from gazebo_msgs.srv   	 	import GetJointProperties
from geometry_msgs.msg  	import Twist
from std_msgs.msg 			import Float32MultiArray
from nav_msgs.msg 			import Odometry
from sensor_msgs.msg 		import Imu


x_odom = float()
y_odom = float()
psi_odom = float()
x_odom_filtered = float()
y_odom_filtered = float()
psi_odom_filtered = float()
acc_x_imu_raw = float()
acc_y_imu_raw = float()
x_imu_raw = float()
y_imu_raw = float()
z_imu_raw = float()
w_imu_raw = float()
acc_x_imu_noisy = float()
acc_y_imu_noisy = float()
x_imu_noisy = float()
y_imu_noisy = float()
z_imu_noisy = float()
w_imu_noisy = float()

def odom_callback(odom_states):
	global x_odom , y_odom , psi_odom
	x_odom = odom_states.pose.pose.position.x
	y_odom = odom_states.pose.pose.position.y
	psi_odom = 2*math.asin(odom_states.pose.pose.orientation.z)
	
def odom_callback_filtered(odom_states):
	global x_odom_filtered , y_odom_filtered , psi_odom_filtered
	msg = odom_states.data
	x_odom_filtered = msg[0]
	y_odom_filtered = msg[1]
	psi_odom_filtered = msg[2]
	
def imu_raw_sync_callback(data):
	global acc_x_imu_raw , acc_y_imu_raw , x_imu_raw , y_imu_raw , z_imu_raw , w_imu_raw
	acc_x_imu_raw = data.linear_acceleration.x
	acc_y_imu_raw = data.linear_acceleration.y
	x_imu_raw = data.orientation.x
	y_imu_raw = data.orientation.y
	z_imu_raw = data.orientation.z
	w_imu_raw = data.orientation.w
		
def imu_noisy_sync_callback(data):
	global acc_x_imu_noisy , acc_y_imu_noisy , x_imu_noisy , y_imu_noisy , z_imu_noisy , w_imu_noisy
	acc_x_imu_noisy = data.linear_acceleration.x
	acc_y_imu_noisy = data.linear_acceleration.y
	x_imu_noisy = data.orientation.x
	y_imu_noisy = data.orientation.y
	z_imu_noisy = data.orientation.z
	w_imu_noisy = data.orientation.w

class state_feedback_controller:
	global x_odom , y_odom , psi_odom
	global x_odom_filtered , y_odom_filtered , psi_odom_filtered
	global acc_x_imu_noisy , acc_y_imu_noisy , x_imu_noisy , y_imu_noisy , z_imu_noisy , w_imu_noisy
	global acc_x_imu_raw , acc_y_imu_raw , x_imu_raw , y_imu_raw , z_imu_raw , w_imu_raw
	
	def __init__(self, name, path_x, path_y ,linear_const, angular_const, distance_tolerance, max_linear_vel, min_linear_vel, max_angular_vel):
		self._name = name
		# controller initilizations
		self._path_x = path_x
		self._path_y = path_y
		self._x = path_x[0]
		self._y = path_y[0]
		self._psi = self.calcSteeringAngle (self._path_x[1] , self._path_y[1])
		self._total_distance = self.calc_total_distance()
		self._traveled_distance = 0
		#self._distance_error = self._total_distance
		self._linearConst = linear_const
		self._angularConst = angular_const
		self._tolerance = distance_tolerance
		self._maxLinearVel = max_linear_vel
		self._minLinearVel = min_linear_vel
		self._maxAngularVel = max_angular_vel
		self._rate = rospy.Rate(10)
		# subscriber to sensor un-synced readings
		rospy.Subscriber("/odom", Odometry, odom_callback)
		rospy.Subscriber("/filtered_odom", Float32MultiArray, odom_callback_filtered)
		rospy.Subscriber("/imu", Imu, imu_raw_sync_callback)
		rospy.Subscriber("/imu_noisy", Imu, imu_noisy_sync_callback)
		rospy.sleep(1)
		# publishing synced readings
		self._pub_imu_raw = rospy.Publisher('/imu_raw_sync', Float32MultiArray, queue_size=5)
		self._pub_imu_noisy = rospy.Publisher('/imu_noisy_sync', Float32MultiArray, queue_size=5)
		self._pub_real = rospy.Publisher('/real_states', Float32MultiArray, queue_size=5)
		self._pub_odom_raw = rospy.Publisher('/odom_raw_sync', Float32MultiArray, queue_size=5)
		# publish control inputs
		self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		rospy.sleep(1)
		# gazebo ros communication
		self._modelStateSetter = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size = 10, latch = True)
		self._jointPropertiesGetter = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		rospy.wait_for_service('/gazebo/get_joint_properties')
		self._jointEffortApplier = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
		rospy.wait_for_service('/gazebo/apply_joint_effort')
		self._modelStateGetter = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		rospy.wait_for_service('/gazebo/get_model_state')
		# setting initial state
		self._msg = ModelState()
		self._msg.model_name = self._name
		self._msg.pose.position.x = self._x
		self._msg.pose.position.y = self._y
		self._msg.pose.position.z = 0.0
		self._msg.pose.orientation.x = 0.0
		self._msg.pose.orientation.y = 0.0
		self._msg.pose.orientation.z = math.sin(self._psi/2)
		self._msg.pose.orientation.w = math.cos(self._psi/2)
		self._msg.twist.linear.x = 0.0
		self._msg.twist.linear.y = 0.0
		self._msg.twist.linear.z = 0.0
		self._msg.twist.angular.x = 0.0
		self._msg.twist.angular.y = 0.0
		self._msg.twist.angular.z = 0
		self._modelStateSetter.publish(self._msg)
		self.setVelocity( 0, 0, 0)
		rospy.sleep(1)
		self.log_data()
		# bag to log data
		
		
	# sending commands to gazebo
	def getJointProperties(self, joint_name):
		joints = self._jointPropertiesGetter(joint_name)
		return joints

	def applyJointEffor(self, joint_name, effort, duration):
		start_time = rospy.get_rostime()
		self._jointEffortApplier(joint_name, effort, start_time, duration)

	def setVelocity(self, x_dot, y_dot, yawr):
		twist=Twist()
		twist.linear.x = x_dot
		twist.linear.y = 0
		twist.linear.z=0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = yawr
		self._pub.publish(twist)
		
	# controller calculations ###########################
	
	def calc_distance(self , x , y):
		distance = (((x - self._x)**2) + ((y - self._y)**2))**0.5
		return distance
		
	def calc_total_distance(self):
		sz = len(self._path_x)-1
		total_distance = 0
		for i in range(sz):
			distance = (((self._path_x[i]- self._path_x[i+1])**2) + ((self._path_y[i]- self._path_y[i])**2))**0.5
			total_distance = total_distance+ distance
		return total_distance

	def calcLinearVelocity(self, distance_error ,linearConst):
		linear_velocity = linearConst * distance_error
		return linear_velocity

	def calcSteeringAngle(self, goal_x, goal_y):
		steering_angle = math.atan2(goal_y - self._y, goal_x - self._x)
		return steering_angle
		
	def calcAngularVelocity(self, goal_x, goal_y, angularConst):
		angular_velocity = angularConst * (self.calcSteeringAngle(goal_x, goal_y) - self._psi)
		return angular_velocity
	
	# getting feedback ###################################
	
	def updateModelState(self):
		state = self._modelStateGetter(self._name, "world")
		self._x = state.pose.position.x
		self._y = state.pose.position.y
		self._psi = 2*math.asin(state.pose.orientation.z)
		
	def update_odom_feedback_raw(self):
		self._x = x_odom
		self._y = y_odom
		self._psi = psi_odom
		
	def update_odom_feedback_filtered(self):
		self._x = x_odom_filtered
		self._y = y_odom_filtered
		self._psi = psi_odom_filtered
	
	# punlishing robot states to be logged ##############
	
	def publish_real_states(self):
		get_state = self._modelStateGetter(self._name, "world")
		x = get_state.pose.position.x
		y = get_state.pose.position.y
		psi = 2*math.asin(get_state.pose.orientation.z)
		pub_states= Float32MultiArray()
		pub_states.data= np.array([x, y, psi])
		self._pub_real.publish(pub_states)
		
	def publish_odom_raw_sync(self):
		x = x_odom
		y = y_odom
		psi = psi_odom
		odom_raw_sync = Float32MultiArray()
		odom_raw_sync.data = np.array([x, y, psi])
		self._pub_odom_raw.publish(odom_raw_sync)
		
	def publish_imu_raw_sync(self):
		acc_x = acc_x_imu_raw
		acc_y = acc_y_imu_raw
		x = x_imu_raw 
		y = y_imu_raw 
		z = z_imu_raw
		w = w_imu_raw
		imu_raw_sync = Float32MultiArray()
		imu_raw_sync.data = np.array([acc_x, acc_y, w , x , y ,z])
		self._pub_imu_raw.publish(imu_raw_sync)
		
	def publish_imu_noisy_sync(self):
		acc_x = acc_x_imu_noisy
		acc_y = acc_y_imu_noisy
		x = x_imu_noisy
		y = y_imu_noisy
		z = z_imu_noisy
		w = w_imu_noisy
		imu_noisy_sync = Float32MultiArray()
		imu_noisy_sync.data = np.array([acc_x, acc_y, w , x , y ,z])
		self._pub_imu_noisy.publish(imu_noisy_sync)
		
	def log_data(self):
		self.publish_odom_raw_sync()
		self.publish_imu_raw_sync()
		self.publish_imu_noisy_sync()
		self.publish_real_states()
	
	# control loop ######################################
	
	def gotoGoal(self, goal_x, goal_y):
		distance_tolerance = self._tolerance
		linearConst = self._linearConst
		angularConst = self._angularConst
		dist2_point = self.calc_distance( goal_x, goal_y)
		while (dist2_point >= distance_tolerance):
			# calculate error
			distance_error = self._total_distance - self._traveled_distance
			#print ("remaining distance = " + str(distance_error))
			print ("distance to next point = " + str(dist2_point))
			# calculate and send control action
			yaw_velocity = self.calcAngularVelocity(goal_x, goal_y, angularConst)
			x_velocity = self.calcLinearVelocity(distance_error, linearConst)
			x_velocity  = min( x_velocity, self._maxLinearVel )
			x_velocity  = max( x_velocity, self._minLinearVel )
			if(yaw_velocity >= 0):
  				yaw_velocity = min(yaw_velocity, self._maxAngularVel)
  			if(yaw_velocity < 0):
  				yaw_velocity = max(yaw_velocity, -self._maxAngularVel)
			self.setVelocity(x_velocity, 0, yaw_velocity)
			# store current state
			x_old = self._x
			y_old = self._y
			# get feedback and update states
			#self.updateModelState()
			# data logging used also to send odom_raw_data to be filtered
			self.log_data()
			self.update_odom_feedback_filtered()
			# update traveled distance
			self._traveled_distance = self._traveled_distance + self.calc_distance(x_old, y_old)
			dist2_point = self.calc_distance( goal_x, goal_y)

			self._rate.sleep()
		print "Reached point!"
		if( goal_x == self._path_x[-1] and goal_y == self._path_y[-1] ):
			print "Reached goal!"
			# stop
			x_velocity = 0.0
			yaw_velocity = 0.0
			self.setVelocity(x_velocity, 0, yaw_velocity)
			# data logging
			self.log_data()
