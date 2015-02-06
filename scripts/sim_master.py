#!/usr/bin/env python
 
from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy

msg = State() #global State variable, defining symbolic world of simulation

def handle_move_robot(req):
	# Handles the service "MoveRobot.srv", for which the request has an Action and Target field 
	# and response has isValid field:
	#	Action:
	#		0 --> open gripper
	#		1 --> close gripper
	#		2 --> move to block (i.e., gripper around block)
	#		3 --> move over block (i.e., gripper above block)
	#	Target:
	#		n --> nth block, where block 0 represents the table
	#	---
	#	isValid: boolean indicating whether action can be performed
	
	#Initialize action as valid by default
	isValid = 1 

	#Determine action, and act appropriately
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
				
	elif (req.Action == 1):
		#close gripper
		if (msg.gripper_state == 0):
			#gripper is currently open, so close it
			msg.gripper_state = 1
	
	elif (req.Target <= msg.num_blocks):

		if (req.Action == 2):
			#move to block
			if ((msg.gripper_state == 0) and (req.Target > 0) and (req.Target not in msg.block_under)):
				#gripper is open, and target isn't under any block, so 
				#ok to move to block.
				#record new block in gripper and one underneath
				i = req.Target
				msg.block_in_gripper = i
				msg.block_under_gripper = msg.block_under[i-1] 
			else:
				#gripper is closed, so can't move to block
				isValid = 0
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
				#some block is already over target, so this action is invalid
				isValid = 0
		else:
			#unknown action	
			isValid = 0
	else:
		#unknown target
		isValid = 0
	
	#Send out response
	return MoveRobotResponse(isValid)
	   
def sim_master_server():
	# Master server for this simple simulation. Handles service "move_robot" for 
	# commanding the robot to perform an action on a specific block, and 
	# publishes to the topic "state" the current state of the world it sees, "State", 
	# with features described as follows:
	#
	# gripper_state:	0 (open) or 1 (closed)
	# block_in_gripper: 	0 (no block) or n (nth block) within gripper. Gripper could be open or closed around block
	# block_under_gripper:	0 (no block) or n (nth block) under gripper
	# num_blocks:		n (n blocks in simulation)
	# block_under:		1xn array, where ith element corresponds to block underneath ith block. Table is block 0. 

	#Initialize node
	rospy.init_node('sim_master_server')

	#Create Service "move_robot"
	s = rospy.Service('move_robot', MoveRobot, handle_move_robot)

	#Get global parameters
	n = rospy.get_param("/num_blocks")
	config = rospy.get_param("/configuration")

	#Initialize the gripper as being open, with no block, over table
	gripper_state = 0
	block_in_gripper = 0
	block_under_gripper = 0

	#Determine initial state of world
	block_under = []

	if (config == "scattered"):
		#scattered config. --> all blocks are on table
		for i in range(0,n):
			block_under.append(0)

	elif (config == "stacked_descending"):
		#stacked, descending config. --> blocks descend in number from ground up
		for i in range(2,n+1):
			block_under.append(i)
		block_under.append(0)

	elif (config == "stacked_ascending"):
		#stacked, ascending config. --> blocks ascend in number from ground up
		for i in range(0,n):	
			block_under.append(i)	

	else:
		rospy.loginfo("Invalid initial configuration.")
		return

	#Assemble message
	global msg
	msg = State(gripper_state,block_in_gripper,block_under_gripper,n,block_under);

	#Initialize topic "state" to publish to    	
	pub = rospy.Publisher('state', State, queue_size=10)
    	rate = rospy.Rate(1) # 1hz publish rate

	#Publish state of the world    
	while not rospy.is_shutdown():
		#rospy.loginfo(msg)
		pub.publish(msg)
        	rate.sleep()


 
if __name__ == "__main__":
	try:
		#Run simulation master server
		sim_master_server()
	except rospy.ROSInterruptException:
        	pass
