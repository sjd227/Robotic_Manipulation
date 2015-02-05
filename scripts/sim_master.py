#!/usr/bin/env python
 
from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy

msg = State(); #global State variable, defining symbolic world

def handle_move_robot(req):
	print "Moving Robot for action: %s, target: %s"%(req.Action, req.Target)
	return AddTwoIntsResponse(req.a + req.b)
	   
def sim_master_server():

	print "HI"
	#Initialize node
	rospy.init_node('sim_master_server')
	s = rospy.Service('move_robot', MoveRobot, handle_move_robot)
	print "Ready to move robot"

	#get params
	n = rospy.get_param("/num_blocks")
	config = rospy.get_param("/configuration")


	#determine initial state of world
	block_is_over = ()
	str1= "test"
	if (config == "scattered"):
		for i in range(0,n):
			block_is_over += (0,)
	elif (config == "stacked_descending"):
		for i in range(2,n+1):
			block_is_over += (i,)
		block_is_over+= (0,)
	elif (config == "stacked_ascending"):
		for i in range(0,n):		
			block_is_over+= (i,)
	else:
		print "Invalid Configuration!"
		return

	#initialize the gripper as being open
	block_in_gripper = 0;	

	#create message
	global msg
	msg = State(block_in_gripper,block_is_over);

	#initialize publishing    	
	pub = rospy.Publisher('state', State, queue_size=10)
    	rate = rospy.Rate(1) # 10hz

	#publish state of the world    
	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
        	rate.sleep()


 
if __name__ == "__main__":
	try:
		print "main"
		sim_master_server()
	except rospy.ROSInterruptException:
        	pass
