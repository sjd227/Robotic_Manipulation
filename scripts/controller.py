#!/usr/bin/env python

from sjd227_hw1.srv import *
from sjd227_hw1.msg import *
from std_msgs.msg import String
import rospy
import sys

msg = State() #global State variable, defining symbolic world

openGripper = 0
closeGripper =1
moveToBlock = 2
moveOverBlock = 3

def configure(configAction):
	rospy.loginfo('Configuring action: %s',configAction)
    	rate = rospy.Rate(1) # 1hz
	
	#scatter blocks. Loop through all blocks to see if its under any
	isvalid = 1
	complete = 0
	block = 1
	while (isvalid and not complete):

		if (configAction == 'scatter'):	
			block = 1		
			while (block <= msg.num_blocks) and ((block in msg.block_under) or (msg.block_under[block-1] == 0)): 
				block+=1
			if (block > msg.num_blocks):
				#all blocks checked. no more to move!
				complete=1
 				#Do last check to make sure:
				block = 1
				while (block <= msg.num_blocks and msg.block_under[block-1] == 0):
					block = block+1
				if (block <= msg.num_blocks):
					#there must have been an error :( 
					isvalid = 0
			else:
				#block isn't under another block or on table, so move it!
				rospy.loginfo("moving block %d",block)
				try:
        				move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
					#PRECONDITION: gripper already open
					isvalid = move_robot(moveToBlock, block)
					if (isvalid):
						isvalid = move_robot(closeGripper,0)
						if (isvalid):
							isvalid = isvalid and move_robot(moveOverBlock,0)
							if (isvalid):
								isvalid = isvalid and move_robot(openGripper,0)

				except rospy.ServiceException, e:
        				rospy.loginfo("Service call failed: %s"%e)
					isvalid = 0
		elif (configAction == "scattered_to_ascending"):
			rospy.loginfo("moving block %d",block)
			try:
        			move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
				#PRECONDITION: gripper already open
				isvalid = move_robot(moveToBlock, block)
				if (isvalid):
					isvalid = move_robot(closeGripper,0)
					if (isvalid):
						isvalid = isvalid and move_robot(moveOverBlock,block-1)
						if (isvalid):
							isvalid = isvalid and move_robot(openGripper,0)
				
				rate.sleep()
				if (block == msg.num_blocks):
					#did all blocks!
					complete = 1
					#check if valid
					trueArr = ()
					for i in range(0,msg.num_blocks):
						trueArr+=(i,)
					rospy.loginfo('%s',trueArr)
					rospy.loginfo('%s',msg.block_under)
					isvalid = trueArr == msg.block_under
					if (not isvalid):
						rospy.loginfo("complete, but not in ascending order!") 
				else:
					block+=1
			except rospy.ServiceException, e:
        			rospy.loginfo("Service call failed: %s"%e)
				isvalid = 0

		elif (configAction == "scattered_to_descending"):
			block2 = msg.num_blocks-block+1
			rospy.loginfo("moving block %d",block2)
			try:
        			move_robot = rospy.ServiceProxy("move_robot", MoveRobot)
					
				#PRECONDITION: gripper already open
				isvalid = move_robot(moveToBlock, block2)
				if (isvalid):
					isvalid = move_robot(closeGripper,0)
					if (isvalid):
						if block2 == msg.num_blocks:
							overBlock = 0
						else:
							overBlock = block2+1
						isvalid = isvalid and move_robot(moveOverBlock,overBlock)
						if (isvalid):
							isvalid = isvalid and move_robot(openGripper,0)
				
				rate.sleep()
				if (block == msg.num_blocks):
					#did all blocks!
					complete = 1
					#check if valid
					trueArr = ()
					for i in range(2,msg.num_blocks+1):
						trueArr+=(i,)
					trueArr+=(0,)
					rospy.loginfo('%s',trueArr)
					rospy.loginfo('%s',msg.block_under)
					isvalid = trueArr == msg.block_under
					if (not isvalid):
						rospy.loginfo("complete, but not in descending order!") 
				else:
					block+=1
			except rospy.ServiceException, e:
        			rospy.loginfo("Service call failed: %s"%e)
				isvalid = 0
		if (isvalid):
			#movement successful. Wait to get new robot state
			if (not complete):
				rate.sleep()
		else:
			#error somewhere
			rospy.loginfo("Failed to move on block %d",block)
	if (complete):
		rospy.loginfo("Configuration set successful!")

	return isvalid

def callback_state(data):
	global msg
	msg = data

def callback_command(data):

	command = data.data
	rospy.loginfo(command)
	
	if (command == 'scattered'):
		isvalid = configure('scatter')
	elif (command == 'ascend_stacked'):
		isvalid = configure('scatter')
		isvalid = isvalid and configure('scattered_to_ascending')
	elif (command == 'descend_stacked'):
		isvalid = configure('scatter')
		isvalid = isvalid and configure('scattered_to_descending')

	

	
	

def controller():
	
	rospy.wait_for_service('move_robot')

	rospy.init_node('controller', anonymous=True)

	rospy.Subscriber("command", String, callback_command)
	rospy.Subscriber("state", State, callback_state)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == "__main__":
	controller()
