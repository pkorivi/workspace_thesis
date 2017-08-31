#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int16
import time

goal_list = PoseArray()

goal_index = 0
goal_flag = True

goal_to_publish = PoseStamped()

def new_goal_list_callback(data):
	global goal_flag
	global goal_to_publish
	print (data)
	goal_to_publish.header.stamp = rospy.get_rostime()
	goal_to_publish.header.frame_id = 'map'
	goal_to_publish.pose.position = data.poses[0].position
	goal_to_publish.pose.orientation.w = 1
	pub2.publish(goal_to_publish)
	
	#print 'callback_executed'


def goal_status_callback(data):
	#status = 4 -> failed
	#status = 3 -> completed
	global goal_flag
	print ('Goal status: ', data.status_list[0].status, ' Flag: ', goal_flag ) 
	
	if (data.status_list[0].status == 1):
		goal_flag = True
	if(((data.status_list[0].status == 4 ) or (data.status_list[0].status == 3)) and (goal_flag == True)):
		#current accomplished goal
		global goal_index
		pub1.publish(goal_index)
		goal_flag = False
		
	
# Intializes everything
def start():
	# Publish
	global pub1,pub2
	rospy.init_node('Turtlebot_Goal_List')
	#Should publish the index of the goal just accomplished
	pub1 = rospy.Publisher('/goal_completed', Int16, queue_size=1)
	#Should publish the goal to the robo from the Array of goals.
	pub2 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
	# subscribed to list goals from rviz interaction and goal status from mobile base
	rospy.Subscriber("/list_of_goals", PoseArray, new_goal_list_callback)
	rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_callback)

	#while not rospy.is_shutdown():

	rospy.spin()

if __name__ == '__main__':
	start()
