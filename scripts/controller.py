#!/usr/bin/env python

import sys
import rospy
from sjd227_hw1.srv import *

def callback(data):
	rospy.loginfo('Command = %s',data.data)

def controller():
    rospy.wait_for_service('move_robot')

    rospy.init_node('controller', anonymous=True)

    rospy.Subscriber("command", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    controller()
