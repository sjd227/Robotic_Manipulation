#!/usr/bin/env python

from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy
import sys

msg = State() #global State variable, defining symbolic world

#Define enumerators for move_robot actions
openGripper 	= 0
closeGripper 	= 1
moveToBlock 	= 2
moveOverBlock 	= 3

def set_config(config):
	# Responsible for manipulation blocks into configurations, including scattering blocks, 
	# putting blocks in ascending order after scattered, and putting blocks 
	# in descending order after scattered.

	# Set rate for sampling simulation state
    	rate = rospy.Rate(1) # 1hz
	
	# Set initial constants
	isValid = 1	# indicates if manipulations are valid
	complete = 0	# indicates if overall manipulation complete
	block = 1	# initial block for manipulation

	# Continue manipulation loop while actions have been valid and sequencce is not complete
	while (isValid and not complete):

		if (config == 'scatter'):

			# Scatter blocks. Find current block on top of stack by looping through all until find one	
			block = 1		
			while (block <= msg.num_blocks) and ((block in msg.block_under) or (msg.block_under[block-1] == 0)): 
				block+=1

			if (block > msg.num_blocks):
				# All blocks checked; no more to move
				complete=1

 				# Do last check to make sure
				block = 1
				while (block <= msg.num_blocks and msg.block_under[block-1] == 0):
					block = block+1
				if (block <= msg.num_blocks):
					# One block isn't over table (block 0). Must have been an uncaught manipulation error
					isValid = 0
			
			else:
				# Block found on top of stack, so move it
				rospy.loginfo("Moving block %d",block)
				try:
					# Proxy service
        				move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
					# Perform the following manipulation sequence:
					# 	1) Move robot to block
					#       2) Close gripper
					#       3) Move robot over table (block 0)
					#       4) Open gripper to drop block on table
					#
					# PRECONDITION: gripper already open. 
					# Assumption: don't need to check state between each manipulation
					# If one of the actions are invalid, quit imediately
					isValid = move_robot(moveToBlock, block)
					if (not isValid):
						return 0
					isValid = move_robot(closeGripper,0)
					if (not isValid):
						return 0
					isValid = move_robot(moveOverBlock,0)
					if (not isValid):
						return 0
					isValid = move_robot(openGripper,0)
					if (not isValid):
						return 0
					# Sleep before next iteration					
					rate.sleep()
				

				except rospy.ServiceException, e:
					# Error with service. End manipulation process
        				rospy.loginfo("Service call failed: %s"%e)
					isValid = 0

		elif (config == "scattered_to_ascending"):
			# Move blocks to ascending order, assuming blocks were previously scattered

			rospy.loginfo("Moving block %d",block)
			try:
				# Proxy service
        			move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
				# Perform the following manipulation sequence:
				# 	1) Move robot to block
				#       2) Close gripper
				#       3) Move robot over previous (block i-1)
				#       4) Open gripper to drop block onto other
				#
				# PRECONDITION: gripper already open. 
				# If one of the actions are invalid, quit imediately
				isValid = move_robot(moveToBlock, block)
				if (not isValid):
					return 0
				isValid = move_robot(closeGripper,0)
				if (not isValid):
					return 0
				isValid = move_robot(moveOverBlock,block-1)
				if (not isValid):
					return 0
				isValid = move_robot(openGripper,0)
				if (not isValid):
					return 0

				# Sleep before next iteration or completion check					
				rate.sleep()

				if (block == msg.num_blocks):
					# All blocks manipulated
					complete = 1
	
					# Check if manipulation was successful by comparing it to expected arangement
					expected = ()
					for i in range(0,msg.num_blocks):
						expected+=(i,)
					isValid = expected == msg.block_under
					if (not isValid):
						rospy.loginfo("Manipulation complete, but not in ascending order!") 
				else:
					# Manipulate next block
					block+=1

			except rospy.ServiceException, e:
				# Error with service. End manipulation process
        			rospy.loginfo("Service call failed: %s"%e)
				isValid = 0

		elif (config == "scattered_to_descending"):
			# Move blocks to descending order, assuming blocks were previously scattered
			blockToManipulate = msg.num_blocks-block+1	#manipulate in reverse order
			rospy.loginfo("Moving block %d",blockToManipulate)

			try:
				# Proxy service
        			move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
				#Perform the following manipulation sequence:
				# 	1) Move robot to block
				#       2) Close gripper
				#       3) Move robot over proceeding block (block i+1)
				#       4) Open gripper to drop block onto other
				#
				# PRECONDITION: gripper already open. 
				# If one of the actions are invalid, quit imediately
				isValid = move_robot(moveToBlock, blockToManipulate)
				if (not isValid):
					return 0
				isValid = move_robot(closeGripper,0)
				if (not isValid):
					return 0
				# Determine proceeding block
				if blockToManipulate == msg.num_blocks:
					overBlock = 0
				else:
					overBlock = blockToManipulate+1
				isValid = move_robot(moveOverBlock,overBlock)
				if (not isValid):
					return 0
				isValid = move_robot(openGripper,0)
				if (not isValid):
					return 0

				# Sleep before next iteration or completion check					
				rate.sleep()
				if (block == msg.num_blocks):
					# Did all blocks!
					complete = 1

					# Check if manipulation was successful by comparing it to expected arangement
					expected = ()
					for i in range(2,msg.num_blocks+1):
						expected+=(i,)
					expected+=(0,)
					isValid = expected == msg.block_under
					if (not isValid):
						rospy.loginfo("Manipulation complete, but not in ascending order!") 
				else:
					# Manipulate next block
					block+=1

			except rospy.ServiceException, e:
				# Error with service. End manipulation process        			
				rospy.loginfo("Service call failed: %s"%e)
				isValid = 0

	return isValid

def callback_state(data):
	global msg
	msg = data

def callback_command(data):
	# Callback for "command" topic subscriber. Takes in data (String type), and responds to the command. 
	# The three possible commands are: 
	# 
	# "scattered":		all blocks on table
	# "ascend_stacked":	blocks aranged in increasing order from ground up
	# "descend_stacked":	blocks aranged in decreasing order from ground up

	# Get command
	command = data.data
	rospy.loginfo(command)
	
	# Respond to command
	if (command == 'scattered'):
		# Scatter blocks
		isvalid = set_config('scatter') 
	elif (command == 'ascend_stacked'):
		# Scatter blocks, then put in ascending order
		isvalid = set_config('scatter')
		isvalid = isvalid and set_config('scattered_to_ascending')
	elif (command == 'descend_stacked'):
		# Scatter blocks, then put in descending order
		isvalid = set_config('scatter')
		isvalid = isvalid and set_config('scattered_to_descending')	
	else:
		# Unknown command
		isvalid = 0;
		rospy.loginfo("Unkown command sent to controller.")

	# Check if manipulation occured successfully
	if (isvalid):
		rospy.loginfo("Commands execution successful")
	else:
		rospy.loginfo("Command execution incomplete")

def controller():
	# Controller for simple block manipulation simulator. Subscribes to topics "state", which 
	# indicates state of the simulator's symbolic world, and "command", which listens for configuration commands
	# published for the controller

	# Wait until the move_robot service is established
	rospy.wait_for_service('move_robot')

	# Initialize node
	rospy.init_node('controller', anonymous=True)

	# Subscribe to "state" and "command" topics
	rospy.Subscriber("state", State, callback_state)
	rospy.Subscriber("command", String, callback_command)

	# Keep python from exiting until this node is stopped
	rospy.spin()

if __name__ == "__main__":
	# Run controller
	controller() 
