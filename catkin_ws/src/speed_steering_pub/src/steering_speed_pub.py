#!/usr/bin/env python
'''
Import Python Packages, ROS messages
'''
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
import time
from nav_msgs.msg import Odometry
import math
import numpy as np


#create an object of type Pose Array message, this will hold the list of points
#that are published from tb_rviz_interaction
goal_list = PoseArray()

#Initialize variable for goal index and goal flag , this is used to hold index
#published to tb_rviz_interaction
goal_index = 0
goal_flag = True
#Current position and orientation of the Car (x,y,theta)
current_pos= [0,0,0]
next_goal_pos = [float('inf'),float('inf'),0]
#This callback is called everytime this node receives list of nodes from path_pt_publisher
#This publishes the first pose of the array to the Robot to navigate.
#There is slight difference in the message we receive from tb_rviz_interaction
#and the message we should send to Robot. This conversion from Pose Array
#to Pose Stamped is done here.
def new_goal_list_callback(data):
	#A global message is used in thought that it will be helpful later if we do
	#some processing
	global next_goal_pos
	#Take the immediate point into consideration
	next_goal_pos[0] = data.poses[0].position.x
	next_goal_pos[1] = data.poses[0].position.y

	#TODO Remove
	#print('next goal:' , next_goal_pos)

def odom_callback(data):
	global current_pos
	current_pos[0] =data.pose.pose.position.x
	current_pos[1] =data.pose.pose.position.y
	quat = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
	(roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quat)
	current_pos[2] =math.degrees(yaw) #yaw # check current orientation


# Intializes everything
def start():
	# Create Global Publishers
	global pub1,pub2
	#Initialize current node with some name
	rospy.init_node('steering_speed_publisher')
	#Assigin publisher that publishes the index of the goal just accomplished
	pub1 = rospy.Publisher('/goal_completed', Int16, queue_size=1)
	#Speed
	speed = rospy.Publisher('/manual_control/speed', Int16, queue_size=1)
	#Steering
	steering = rospy.Publisher('/manual_control/steering', Int16, queue_size=1)
	#subscribe to list of goals from rviz interaction package
	rospy.Subscriber("/list_of_goals", PoseArray, new_goal_list_callback)
	#subscribe to Odometry
	image_sub = rospy.Subscriber("/odom",Odometry, odom_callback)
	#This keeps the function active till node are shurdown.
	#rospy.spin()
	rate = rospy.Rate(5) # 5Hz 
	while not rospy.is_shutdown():
		#Calculate the slope of the line joining current position and destination
		drive_angle = np.rad2deg(np.arctan2(next_goal_pos[1] - current_pos[1], next_goal_pos[0] - current_pos[0]))
		current_angle = current_pos[2]
		turn = current_angle - drive_angle
		skp = 3
		steering_angle = 90+skp*turn
		#Boundary conditions
		if steering_angle < 40:
			steering_angle = 40
		elif steering_angle > 140:
			steering_angle = 140
		#If two points are in 15cm range consider them as met, stop vehicle and publish goal completed to the planner
		dist_to_goal = abs(np.linalg.norm(np.array([next_goal_pos[0],next_goal_pos[1]])-np.array([current_pos[0],current_pos[1]]), 2, 0))
		print('dist_to_goal ',dist_to_goal)
		if( dist_to_goal > 0.15 ):
			speed.publish(-750)
		else:
			speed.publish(0)
			#publish the index of goal completed - currently first value of list
			pub1.publish(0)

		steering.publish(steering_angle)
		rate.sleep()

#Main function
if __name__ == '__main__':
	start()
