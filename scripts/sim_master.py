#!/usr/bin/env python
 
from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
import rospy

uint8 
 
def handle_move_robot(req):
	print "Moving Robot for action: %s, target: %s"%(req.Action, req.Target)
	return AddTwoIntsResponse(req.a + req.b)
	   
def sim_master_server():
	#Initialize node
	rospy.init_node('sim_master_server')
	s = rospy.Service('move_robot', MoveRobot, handle_move_robot)
	print "Ready to move robot"

	#initialize publishing    	
	pub = rospy.Publisher('state', State, queue_size=10)
    	rate = rospy.Rate(1) # 10hz

	#get params
	n = rospy.get_param("/num_blocks")
	config = rospy.get_param("/config")

	#determine initial state of world
	block_is_over = ()
	if (config is "scattereed"):
		for i in range(0,n):
			block_is_over += (0,)
	else if (config is "stacked_ascending"):
		for i in range(2,n-1):
			block_is_over += (i,)
		block_is_over(n-1) = 0
	else if (config is "stacked_descending"):
		for i in range(0,n):		
			block_is_over+= (i,)
	

	#publish state of the world    
	while not rospy.is_shutdown():
	
 
if __name__ == "__main__":
	try:
		sim_master_server()
	except rospy.ROSInterruptException:
        	pass
