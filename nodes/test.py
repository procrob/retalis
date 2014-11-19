#!/usr/bin/env python

import sys
import rospy
from retalis.srv import *

def add(X):
    rospy.wait_for_service('subscribe_output')
    try:
        add_two_ints = rospy.ServiceProxy('subscribe_output', SubscribeOutput)
        resp1 = add_two_ints('geometry_msgs__0__PoseStamped(std_msgs__0__Header(_,_,X),_)','X="4x4_1"',"_","marker","m2")
	print resp1.result	        
	return 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    add(1)

