#!/usr/bin/env python
 
from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy

msg = State() #global State variable, defining symbolic world

def handle_move_robot(req):
	isvalid = 1
	print "Moving Robot for action: %s, target: %s"%(req.Action, req.Target)
	
	if (req.Action == 0):
		#open gripper
		if (msg.gripper_state == 1):
			#gripper is closed, so open it
			msg.gripper_state = 0
			if (msg.block_in_gripper > 0):
				#there was a block in the gripper, so note where it landed
				i = msg.block_in_gripper
				msg.block_under[i-1] = msg.block_under_gripper

				#set no block in gripper
				msg.block_in_gripper = 0
				
		else:
			#gripper already open, so it's an invalid action
			isvalid = 0
	elif (req.Action == 1):
		#close gripper
		if (msg.gripper_state == 0):
			#gripper is currently open, so close it
			msg.gripper_state = 1
		else:
			#gripper is already closed, so it's an invalid action
			isvalid = 0
	
	elif (req.Target <= msg.num_blocks):

		if (req.Action == 2 and req.Target > 0):
			#move to block
			if ((msg.gripper_state == 0) and (req.Target == 0 or req.Target not in msg.block_under)):
				#gripper is open, and target isn't under any block, so 
				#ok to move to block.
				#record new block in gripper and one underneath
				i = req.Target
				msg.block_in_gripper = i
				msg.block_under_gripper = msg.block_under[i-1] 
			else:
				#gripper is closed, so can't move to block
				isvalid = 0
		elif (req.Action == 3):
			#move over block
		
			if (req.Target ==0 or req.Target not in msg.block_under):
				#no blocks over target, so move over it
				msg.block_under_gripper = req.Target
			
				if (msg.gripper_state == 0):
					#gripper was open, so block previously in gripper region
					#didn't travel along with the gripper
					msg.block_in_gripper = 0
	
			else:
				#a block is already over target, so this action is invalid
				isvalid = 0
		else:
			#unknown action	
			isvalid = 0
	else:
		#unknown target
		isvalid = 0
	return MoveRobotResponse(isvalid)
	   
def sim_master_server():

	print "HI"
	#Initialize node
	rospy.init_node('sim_master_server')
	s = rospy.Service('move_robot', MoveRobot, handle_move_robot)
	print "Ready to move robot"

	#get params
	n = rospy.get_param("/num_blocks")
	config = rospy.get_param("/configuration")

	#initialize the gripper as being open, with no block, over table
	gripper_state = 0
	block_in_gripper = 0
	block_under_gripper = 0

	#determine initial state of world
	block_under = []
	str1= "test"
	if (config == "scattered"):
		for i in range(0,n):
			block_under.append(0)
			#block_under_i += (0,)
	elif (config == "stacked_descending"):
		for i in range(2,n+1):
			block_under.append(i)
			#block_under_i += (i,)
		block_under.append(0)
		#block_under_i+= (0,)
	elif (config == "stacked_ascending"):
		for i in range(0,n):	
			block_under.append(i)	
			#block_under_i+= (i,)
	else:
		print "Invalid Configuration!"
		return

	#create message
	global msg
	msg = State(gripper_state,block_in_gripper,block_under_gripper,n,block_under);

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
