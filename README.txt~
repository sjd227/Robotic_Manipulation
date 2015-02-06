Steven Dourmashkin
CS 5272
README for HW1, 2/6/14

-----

Explination of state message:

gripper_state:	0 (open) or 1 (closed)
block_in_gripper: 	0 (no block) or n (nth block) within gripper. Gripper could be open or closed around block
block_under_gripper:	0 (no block) or n (nth block) under gripper
num_blocks:		n (n blocks in simulation)
block_under:		1xn array, where ith element corresponds to block underneath ith block. Table is block 0. 

----

After adding the package “sjd227_hw1” to the ROS working directory, the launch file for the single-armed robot  can be ran by:

roslaunch sjd227_hw1 hw1.launch

This launch file starts nodes sim_master and controller, using the default global params of \congiruation = “stacked_ascending”, and \num_blocks = 5. Occasionally, if the package can't be found in a new Terminal window, use the following command:

source ~/ros_ws/devel/setup.bash

To view the state given by the sim_master node, enter the following into a new Terminal window:

rosservice call \move_robot [arg1] [arg2]

where [arg1] is an integer 0, 1, 2, or 3, representing the Action: opening the gripper, closing the gripper, moving to a block, and moving above a block, respectively, and [arg2] represents the Target: an integer 1, 2, …, n reprsenting one of the n blocks to either move to or above. For instance:

rosservice call \move_robot 3 4

would move the gripper to the fifth block if the configuration is “stacked_ascending”, assuming that the gripper was previously opened. The reesponse of this service, isValid, indicates whether or not the motion is possible given the current state of the system, and, if so, whether or not the action was performed. To view changes to the state of the entire system, one can echo the /state topic as follows:

rostopic echo state

---

To run the bimanual version, the following command is used: 

roslaunch sjd227_hw1 Bi_hw1.launch

The \Bi_move_robot service for the binomial version includes a third argument indicating which arm to run the action on (0 or 1); for example, 

rosservice call \Bi_move_robot 2 5 0

could move the “0” arm to the 5th block. To view the state of the bimanual system, the same rostopic command is used:


rostopic echo Bi_state

When viewing the state, element “0” of, for instance, gripper_state, represents the gripper_state of arm “0”. 
 
