#!/usr/bin/env python

from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy
import sys

msg = Bi_State() #global State variable, defining symbolic world

#Define enumerators for move_robot actions
openGripper 	= 0
closeGripper 	= 1
moveToBlock 	= 2
moveOverBlock 	= 3

def openBothGrippers():
	try:
		# Proxy service
        	Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)

		isValid = Bi_move_robot(openGripper,0, 0)
		if (not isValid):
			return 0
		isValid = Bi_move_robot(openGripper,0, 1)
		if (not isValid):
			return 0
		return 1
	except rospy.ServiceException, e:
			# Error with service. End manipulation process
			rospy.loginfo("Service call failed: %s"%e)
			return 0

def find_block_to_scatter():
	# Scatter blocks. Find current block on top of stack by looping through all until find one	
	block = 1		
	while (block <= msg.num_blocks) and ((block in msg.block_under) or (msg.block_under[block-1] == 0)): 
		block+=1
	if (block > msg.num_blocks):
		# All blocks checked; no more to move
		return 0
	else:
		return block

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
	Robot = 0;

	# Continue manipulation loop while actions have been valid and sequencce is not complete
	while (isValid and not complete):

		if (config == 'scatter'):

			# Get block for first robot to move
			block= find_block_to_scatter()
			if (block>0):
				# Block found on top of stack, so move it
				rospy.loginfo("Moving block %d with Robot %d",block,Robot)

				try:				
					# Perform the following manipulation sequence:
					# 	1) Open robot A gripper to drop block on table (or not in beginning)
					#       2) Move robot A to top block in stack
					#       3) Close robot A gripper to grab block on stack
					#       4) Move robot A to empty spot on the table
					#	- switch to robot B
					# If one of the actions are invalid, quit imediately
					
					# Proxy service
        				Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)

					isvalid = Bi_move_robot(openGripper,0,Robot)
					if (not isValid):
						return 0
					isvalid = Bi_move_robot(moveToBlock,block,Robot)
					if (not isValid):
						return 0
					isvalid = Bi_move_robot(closeGripper,0,Robot)
					if (not isValid):
						return 0
					isvalid = Bi_move_robot(moveOverBlock,0,Robot)
					if (not isValid):
						return 0
					Robot = 1-Robot
				except rospy.ServiceException, e:
					# Error with service. End manipulation process
        				rospy.loginfo("Service call failed: %s"%e)
					isValid = 0
				# Sleep before comparing so we receive most recent state				
				rate.sleep()
			else:
				# All blocks checked; no more to move
				complete=1
				try:
					# Proxy service
        				Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)
					
					# Drop last block 
					openBothGrippers()

				except rospy.ServiceException, e:
					# Error with service. End manipulation process
        				rospy.loginfo("Service call failed: %s"%e)
					isValid = 0

				# Do last check to make sure
				rate.sleep() #sleep so we get most rescent state
				block = 1
				while (block <= msg.num_blocks and msg.block_under[block-1] == 0):
					block = block+1
				if (block <= msg.num_blocks):
					# One block isn't over table (block 0). Must have been an uncaught manipulation error
					isValid = 0

		elif (config == "scattered_to_ascending"):
			# Move blocks to ascending order, assuming blocks were previously scattered

			rospy.loginfo("Moving block %d with Robot %d",block,Robot)
			try:
				# Proxy service
        			Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)

				if (block == 1):
					#starter robot
					isValid = Bi_move_robot(moveToBlock, block, Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(closeGripper,0, Robot)
					if (not isValid):
						return 0
					Robot = 1-Robot
				elif (block <= msg.num_blocks):

					# Perform the following manipulation sequence:
					# 	1) Move robot A (gripper open) to block on table
					#       2) Close robot A gripper on block
					#       3) Move robot B (gripper closed) over stacked block
					#       4) Open robot B gripper to drop block on stack
					#
					# If one of the actions are invalid, quit imediately

					isValid = Bi_move_robot(moveToBlock, block, Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(closeGripper,0, Robot)
					if (not isValid):
						return 0
					Robot = 1-Robot
					isValid = Bi_move_robot(moveOverBlock,block-2,Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(openGripper,0, Robot)
					if (not isValid):
						return 0

				else:
					#no more blocks left, so have last robot drop off block and open other gripper
					Robot = 1-Robot
					isValid = Bi_move_robot(moveOverBlock,block-2,Robot)
					if (not isValid):
						return 0
					isValid = openBothGrippers()

			except rospy.ServiceException, e:
				# Error with service. End manipulation process
				rospy.loginfo("Service call failed: %s"%e)
				isValid = 0
			
			# Sleep before comparing so we receive most recent state				
			rate.sleep()

			if (block > msg.num_blocks):
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



		elif (config == "scattered_to_descending"):

			# Move blocks to descending order, assuming blocks were previously scattered

			blockToManipulate = msg.num_blocks-block+1	#manipulate in reverse order\			
			rospy.loginfo("Moving block %d with Robot %d",block,Robot)
			try:
				# Proxy service
        			Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)
		
				if (block == 1):
					#starter robot
					isValid = Bi_move_robot(moveToBlock, blockToManipulate, Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(closeGripper,0, Robot)
					if (not isValid):
						return 0
					Robot = 1-Robot
				elif (block <= msg.num_blocks):

					# Perform the following manipulation sequence:
					# 	1) Move robot A (gripper open) to block on table
					#       2) Close robot A gripper on block
					#       3) Move robot B (gripper closed) over stacked block
					#       4) Open robot B gripper to drop block on stack
					#
					# If one of the actions are invalid, quit imediately

					rospy.loginfo(blockToManipulate)
					isValid = Bi_move_robot(moveToBlock, blockToManipulate, Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(closeGripper,0, Robot)
					if (not isValid):
						return 0
					Robot = 1-Robot
					# Determine proceeding block
					if (blockToManipulate+1) == msg.num_blocks:
						overBlock = 0
					else:
						overBlock = blockToManipulate+2
					isValid = Bi_move_robot(moveOverBlock,overBlock,Robot)
					if (not isValid):
						return 0
					isValid = Bi_move_robot(openGripper,0, Robot)
					if (not isValid):
						return 0

				else:
					#no more blocks left, so have last robot drop off block
					Robot = 1-Robot
					isValid = Bi_move_robot(moveOverBlock,blockToManipulate+2,Robot)
					if (not isValid):
						return 0
					openBothGrippers()

			except rospy.ServiceException, e:
				# Error with service. End manipulation process
        			rospy.loginfo("Service call failed: %s"%e)
				isValid = 0

			# Sleep before comparing so we receive most recent state				
			rate.sleep()

			if (block > msg.num_blocks):
				# Did all blocks!
				complete = 1

				# Check if manipulation was successful by comparing it to expected arangement
				expected = ()
				for i in range(2,msg.num_blocks+1):
					expected+=(i,)
				expected+=(0,)
				isValid = expected == msg.block_under
				if (not isValid):
					rospy.loginfo("Manipulation complete, but not in descending order!") 
			else:
				# Manipulate next block
				block+=1

		elif (config == 'scattered_to_even_odd'):
			# Separate blocks into two stacks, using bimanual robot

			# Determine blocks for moving
			robotEvenBlock = block+1;			
			robotOddBlock = block;
			robotEvenOverBlock = max(robotEvenBlock-2,0)
			robotOddOverBlock = max(robotOddBlock-2,0)

			if (robotOddBlock <= msg.num_blocks):

				try:
					# Proxy service
					Bi_move_robot = rospy.ServiceProxy("Bi_move_robot", Bi_MoveRobot)
				
					# Perform the following manipulation sequence:
					# 	1) Move robot A and B to block on table
					#       2) Close robot A and robot B grippers
					#       3) Move robot A and B over even/odd stacks
					#       4) Open robot A and B grippers to drop block
					#
					# If one of the actions are invalid, quit imediately

					# Move to block for gripping
					if (robotEvenBlock <= msg.num_blocks):
						isValid = Bi_move_robot(moveToBlock, robotEvenBlock, 0)
						if (not isValid):
							return 0
					isValid = Bi_move_robot(moveToBlock, robotOddBlock, 1)
					if (not isValid):
						return 0
					# Close gripper
					if (robotEvenBlock <= msg.num_blocks):
						isValid = Bi_move_robot(closeGripper,0, 0)
						if (not isValid):
							return 0
					isValid = Bi_move_robot(closeGripper,0, 1)
					if (not isValid):
						return 0
			
					# Move block ontop of appropriate stack
					if (robotEvenBlock <= msg.num_blocks):				
						isValid = Bi_move_robot(moveOverBlock,robotEvenOverBlock,0)
						if (not isValid):
							return 0
					isValid = Bi_move_robot(moveOverBlock,robotOddOverBlock,1)
					if (not isValid):
						return 0

					# Open gripper to release block
					if (robotEvenBlock <= msg.num_blocks):				
						isValid = Bi_move_robot(openGripper,0, 0)
						if (not isValid):
							return 0
					isValid = Bi_move_robot(openGripper,0, 1)
					if (not isValid):
						return 0
				except rospy.ServiceException, e:
					# Error with service. End manipulation process
					rospy.loginfo("Service call failed: %s"%e)
					isValid = 0
			
				#increase block

				block = robotEvenBlock+1
				
				# Sleep before next iteration				
				rate.sleep()

			else:
				#All blocks accounted for
				complete = 1
				openBothGrippers() #open both grippers to prevent future problems
			
				#check if manipulation was successful
				expected = (0,0,)
				for i in range(1,msg.num_blocks-1):
					expected+=(i,)
				isValid = expected == msg.block_under
				if (not isValid):
					rospy.loginfo("Manipulation complete, but not in descending order!")

		else:
			#Unknown config
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
	elif (command == 'odd_even'):
		# Organize blocks into odd and even stacks
		isvalid = set_config('scatter')
		isvalid = isvalid and set_config('scattered_to_even_odd')	
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
	rospy.wait_for_service('Bi_move_robot')

	# Initialize node
	rospy.init_node('controller', anonymous=True)

	# Subscribe to "state" and "command" topics
	rospy.Subscriber("Bi_state", Bi_State, callback_state)
	rospy.Subscriber("command", String, callback_command)

	# Keep python from exiting until this node is stopped
	rospy.spin()

if __name__ == "__main__":
	# Run controller
	controller() 
