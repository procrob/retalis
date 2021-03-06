#!/usr/bin/env python

##################################################################################################
#    This is part of the Retalis Language for Information Processing and Management in Robotics
#    Copyright (C) 2014 __Pouyan Ziafati__ pziafati@gmail.com 
#
#    Retalis is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Retalis is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.                   
#
#    You should have received a copy of the GNU General Public License
#    along with Retalis.  If not, see <http://www.gnu.org/licenses/>.	
#####################################################################################################


import roslib; roslib.load_manifest('retalis')
import xml.etree.ElementTree as ET
from retalis_ros_helper import *
from retalis.srv import *
from std_msgs.msg import String
from threading import *
import rospy
import roslib.message
import rostopic
import rosmsg
import SocketServer
SocketServer.TCPServer.allow_reuse_address = True 
import os
import os.path
import sys
from datetime import datetime
import rospkg

# a dictionary which keeps key-value pairs of the form (topic-name,ros-publisher-to-this-topic)
publish_to = dict()
# a dictionary which keeps key-value pairs of the form (topic-name,message-type-of-this-topic). 
publish_to_msg_type = dict()

subscribed_to = dict()


"""
handler for messages received on topics to which we are subscribed. It converts ROS messages of any type to events.	
"""
def inputFromRosCB(data):
	sendRosMessageToRetalis(data)

"""
handler for messages of type etalis_ros/RegWinSMC received from the topic "smc_query". Each message specifies a subscription window. This handler converts the subscription window query to corresponding etalis query and sent it to etalis.
"""
#def smc_query_callback(data):
#	sendSMCQueryMessageToEtalis(data)


def inputFromRetalis(data):
		#print data
		#ee = data.data.split('\\ \\ \\ \\ \\ \\ ');
		#eee = ee[1].split('-') 
		conv = extractInfoFromSMC(data.data)

		rosMsgClass = roslib.message.get_message_class(publish_to_msg_type[conv[1]])
		rosMsg = rosMsgClass()
		
		convertEventToRos(conv[0],rosMsg,rosMsgClass)
		publish_to[conv[1]].publish(rosMsg)	



def initialize():
        rospy.init_node('retalis_ros_converter')

	#xml processing of the topics of interest to subscribe to and to publish to.
	#basepath = os.path.dirname(__file__)
	#pub_sub_filepath = os.path.abspath(os.path.join(basepath, '..', 'data', 'pub_sub.xml'))
	rospack = rospkg.RosPack()
	pub_sub_filepath = rospack.get_path('retalis')+'/application_source/pub_sub.xml'	
	tree = ET.parse(pub_sub_filepath)
	root = tree.getroot()

	#subscribe to topics of interest
	global subscribed_to
	for to_subscribe in root.iter('subscribe_to'):
		try:			
			message_class = roslib.message.get_message_class(to_subscribe.get('msg_type'))
		except ImportError:
        		raise IOError("cannot load [%s]"%(type_))
		subscribed_to[to_subscribe.get('name')] = rospy.Subscriber(to_subscribe.get('name'), message_class, inputFromRosCB)
		print "[INFO] Sucessfully registered to: ", to_subscribe.get('name')	
 
	#make publisher for topic of interest that we want to publish to in future
	global publish_to
	for to_publish in root.iter('publish_to'):
		try:			
			message_class = roslib.message.get_message_class(to_publish.get('msg_type'))
		except ImportError:
        		raise IOError("cannot load [%s]"%(type_))		
		publish_to[to_publish.get('name')] = rospy.Publisher(to_publish.get('name'),message_class)
		publish_to_msg_type[to_publish.get('name')] = to_publish.get('msg_type')


	

	#rospy.Subscriber("smc_sub", RegWinSMC, smc_query_callback)
	rospy.Subscriber("retalisOutputEvents", String, inputFromRetalis, queue_size = 10)
	s1 = rospy.Service('add_input_subscription', AddInputSubscription, handle_add_input_subscription)
	s2 = rospy.Service('delete_input_subscription', DeleteInputSubscription, handle_delete_input_subscription)       


	
def handle_add_input_subscription(req):
    global subscribed_to
    try:			
		message_class = roslib.message.get_message_class(req.message_type)
    except ImportError:
       		raise IOError("cannot load [%s]"%(req.message_type))
    subscribed_to[req.topic] = rospy.Subscriber(req.topic, message_class, inputFromRosCB)
    print "[INFO] Sucessfully registered to: ", req.topic
    return AddInputSubscriptionResponse('ok')


	
def handle_delete_input_subscription(req):
    global subscribed_to
    subscribed_to[req.topic].unregister()
    return DeleteInputSubscriptionResponse('ok')

	
	

if __name__ == "__main__":
	initialize()
	rospy.spin()
