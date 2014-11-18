##################################################################################################
#    This is part of the Retalis Language for Information Processing and Management in Robotics
#    Copyright (C) 2014 __Pouyan Ziafati__ pziafati@gmail.com 
#    Copyright (C) 2014 __Sergio Sousa__ sergio.sousa@post.lu
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

import socket
import sys
import re

import roslib.message
import rostopic
import rosmsg
import rospy
import genpy
import string
import os,sys,threading
from datetime import datetime
from std_msgs.msg import String

toRetalisPub_ = rospy.Publisher('retalisInputEvents',String, queue_size=5000)

#############################################
#		ROS -> Retalis communication			#
#############################################

def sendRosMessageToRetalis(rawMsg):
	
	event = convertRosToEvent(rawMsg)

	try:
		secs = rawMsg.header.stamp.secs
		nsecs = rawMsg.header.stamp.nsecs
	except:
		now = rospy.get_rostime()
		secs = now.secs
		nsecs = now.nsecs

	event = 'event('+event+',['+datetime.fromtimestamp(secs).strftime('datime(%Y,%m,%d,%H,%M,%S,')+str(nsecs)+'),'+datetime.fromtimestamp(secs).strftime('datime(%Y,%m,%d,%H,%M,%S,')+str(nsecs)+')])'
	
	
	global toRetalisPub_
	toRetalisPub_.publish(event)


"""def sendSMCQueryMessageToEtalis(smcQueryMsg):
	event = smcQueryMsg.winRegQuery #+'\\'+str(smcQueryMsg.header.stamp.secs)+'-'+str(smcQueryMsg.header.stamp.nsecs)
	sendRosMessageToRetalis(event)	
"""

def convertRosToEvent(rawMsg):
	msg = ""
	if hasattr(rawMsg, "_type"):
		msg_type = rawMsg._type
		msg_type = msg_type.replace("/","__0__")
		temp_msg = []	
		
		try:
			msg+=(msg_type + "(")		
			for slot in rawMsg.__slots__:
				x = convertRosToEvent(getattr(rawMsg, slot))				
				msg = msg+x+", "
			msg = msg[:-2]+")"
		except:
			print "[ERROR] Couldn't extract information from ROS message"
			assert(Fail)
	else:
		temp_msg = []		
		if (type(rawMsg) == list) or (type(rawMsg) == tuple):
			if(len(rawMsg) == 0):
				 msg+="[]"			
			else:
				msg+="["
				for i in range(len(rawMsg)) :
					x = convertRosToEvent(rawMsg[i])					
					msg = msg + x + ", "
				msg = msg[:-2]+"]"
		elif (isinstance(rawMsg, genpy.rostime.Time)):
			msg+=("["+str(rawMsg.secs)+", "+str(rawMsg.nsecs)+"]")
		else:		 
			if(type(rawMsg) is str):
				printset = set(string.printable)		
				isprintable = set(rawMsg).issubset(printset)
				if(isprintable):
					msg = "'\"" + rawMsg + "\"'"						
				else:
					msg = 	"'\"FFFFFFFFFF"+ rawMsg.encode('base64','strict').replace("\n", "////") + "\"'"					
			else:
				msg = str(rawMsg)
	return msg



#############################################
#		Retalis -> ROS communication			#
#############################################


	
def extractInfoFromSMC(message):
	outerPar = re.compile("\((.+)\)")
	m = outerPar.search(message)
	stringToAnalyze = m.group(1)
	parts = stringToAnalyze.split(",")
	etalis_msg = ""
	for i in range(len(parts) - 3) :
		etalis_msg=etalis_msg+parts[i]+","
	etalis_msg = etalis_msg+parts[len(parts)-3]
	topic = parts[len(parts)-2]
	msg_id = parts[len(parts)-1]
	return etalis_msg, topic, msg_id


def convertEventToRos(event, rosMsg, msgClass):
	#extracts the content that is in between the outer paranthesis
	outerPar = re.compile("\((.+)\)")
	try:
		m = outerPar.search(event)
		event = m.group(1)
	except:
		True
	parts= event.split(",")
	i = 0
	slot_counter = -1;
	for slot in rosMsg.__slots__:
		slot_counter+=1
		slotAttr = getattr(rosMsg, slot)
		if hasattr(slotAttr, "_type"):
			paranthesis_count= parts[i].count("(") - parts[i].count(")")
			stringPart = str(parts[i])
			while(paranthesis_count>0):
				i += 1
				paranthesis_count = paranthesis_count + parts[i].count("(")
				paranthesis_count = paranthesis_count - parts[i].count(")")
				stringPart += "," + parts[i]
			convertEventToRos(stringPart,slotAttr,type(slotAttr))
			i+=1
		elif (type(slotAttr) == list) or (type(slotAttr) == tuple):
			if (parts[i][1:] == "[]") or (parts[i] == "[]") or (parts[i][:-1] == "[]") :
				True
				i+=1
			else:
				list_type = rosMsg._slot_types[slot_counter]
				list_type = list_type[:list_type.find('[')]
			        if(re.match('bool|int|long|float|string|str',list_type)):
 					array_counter = 0
					parts[i]=parts[i][parts[i].find('[')+1:]
					while(parts[i].find("]")==-1):
						if(re.match('bool',list_type)):
							if (slotAttr[array_counter] is None):
								if(parts[i] in ("True","true","1")):
									slotAttr.append(True)
								else: 
									slotAttr.append(False)
							else:
								if(parts[i] in ("True","true","1")):
									slotAttr[array_counter] = True
								else: 
									slotAttr[array_counter] = False
						elif(re.match('int',list_type)):
							if (slotAttr[array_counter] is None):
								slotAttr.append(int(parts[i]))
							else:
								slotAttr[array_counter] = int(parts[i])
						elif(re.match('long',list_type)):
							if (slotAttr[array_counter] is None):
								slotAttr.append(long(parts[i]))
							else:
								slotAttr[array_counter] = long(parts[i])
						elif(re.match('float',list_type)):
							if (slotAttr[array_counter] is None):
								slotAttr.append(float(parts[i]))
							else:
								slotAttr[array_counter] = float(parts[i])
						elif(re.match('string|str',list_type)):
							if (slotAttr[array_counter] is None):
								if(parts[i].find("\"")>1):
									slotAttr.append(parts[i][parts[i].find("\"")+1:parts[i].rfind("\"")]) #ToDo Add time
								else:
									slotAttr.append(parts[i]) #ToDo Add time
							else:
								if(parts[i].find("\"")>1):
									slotAttr[array_counter] = parts[i][parts[i].find("\"")+1:parts[i].rfind("\"")] #ToDo Add time
								else:
									slotAttr[array_counter] = parts[i] #ToDo Add time
						array_counter+=1
						i+=1
					parts[i]=parts[i][:parts[i].find(']')]
					if(re.match('bool',list_type)):
						if (slotAttr[array_counter] is None):
							if(parts[i] in ("True","true","1")):
								slotAttr.append(True)
							else: 
								slotAttr.append(False)
						else:
							if(parts[i] in ("True","true","1")):
								slotAttr[array_counter] = True
							else: 
								slotAttr[array_counter] = False
					elif(re.match('int',list_type)):
						if (slotAttr[array_counter] is None):
							slotAttr.append(int(parts[i]))
						else:
							slotAttr[array_counter] = int(parts[i])
					elif(re.match('long',list_type)):
						if (slotAttr[array_counter] is None):
							slotAttr.append(long(parts[i]))
						else:
							slotAttr[array_counter] = long(parts[i])
					elif(re.match('float',list_type)):
						if (slotAttr[array_counter] is None):
							slotAttr.append(float(parts[i]))
						else:
							slotAttr[array_counter] = float(parts[i])
					elif(re.match('string|str',list_type)):
						if (slotAttr[array_counter] is None):
							if(parts[i].find("\"")>1):
								slotAttr.append(parts[i][parts[i].find("\"")+1:parts[i].rfind("\"")]) #ToDo Add time
							else:
								slotAttr.append(parts[i]) #ToDo Add time
						else:
							if(parts[i].find("\"")>1):
								slotAttr[array_counter] = parts[i][parts[i].find("\"")+1:parts[i].rfind("\"")] #ToDo Add time
							else:
								slotAttr[array_counter] = parts[i] #ToDo Add time	
					i+=1
				else:
					if( (parts[i].count("[") - parts[i].count("]")) == 0 ):
						partRosMsg = roslib.message.get_message_class(list_type)()
						convertEventToRos(parts[i][parts[i].find('[')+1:parts[i].rfind(']')],partRosMsg,roslib.message.get_message_class(list_type))
						slotAttr.append(partRosMsg)
						i+=1	
					else:
						brak_count= 0
						paranthesis_count = 0						
						stringPart = ""	
						firstTime = True
						contWhile = True
						firstTimeWhile = True
						stringPart = ""
						while(contWhile):		
							if(firstTimeWhile):							
								stringPart += parts[i]
								firstTimeWhile = False
							else:
								stringPart += ", "+parts[i]				
							paranthesis_count +=  parts[i].count("(") - parts[i].count(")")
							brak_count += parts[i].count("[") - parts[i].count("]")
							if(paranthesis_count==0):
								if(firstTime):
									stringPart = stringPart[stringPart.find('[')+1:]
									firstTime = False	
								if(brak_count==0):
									stringPart = stringPart[:stringPart.rfind(']')]
								partRosMsg = roslib.message.get_message_class(list_type)()
								convertEventToRos(stringPart,partRosMsg,roslib.message.get_message_class(list_type))
								slotAttr.append(partRosMsg)
								stringPart=""
							if(brak_count==0): 
								contWhile = False
					                i+=1
						
		elif (isinstance(slotAttr, genpy.rostime.Time)): #ToDo add Duration the same as Time (use or)
			setattr(rosMsg.stamp, "secs", long(parts[i][parts[i].find("[")+1:]))
			setattr(rosMsg.stamp, "nsecs", long(parts[i+1][1:parts[i+1].find("]")]))
			i+=2
		else:
			if( isinstance(slotAttr,bool)):
				if(parts[i] in ("True","true","1")):
					slotAttr.append(True)
				else: 
					slotAttr.append(False)
			elif( isinstance(slotAttr,int)):
				setattr(rosMsg,slot,int(parts[i]))
			elif( isinstance(slotAttr,long)):
				setattr(rosMsg,slot,long(parts[i]))
			elif( isinstance(slotAttr,float)):
				setattr(rosMsg,slot,float(parts[i]))
			elif( isinstance(slotAttr,str)):
				if parts[i].startswith("'\"") or parts[i].startswith("\""):
					tempStr = parts[i][parts[i].find('"')+1:parts[i].rfind('"')]
				else:
					tempStr = parts[i]
				if(tempStr.startswith("FFFFFFFFFF")):
					y=tempStr[tempStr.find("FFFFFFFFFF")+10:].replace("////","\n")
					setattr(rosMsg,slot,y.decode('base64','strict'))
				else:
					setattr(rosMsg,slot,tempStr) #ToDo Add time					
			i+=1

def decodec(c):
    # Expand this into a real mapping if you have more substitutions
    return '\n' if c == '/n' else c[0]
#*******************************
#This two functions where copied from http://code.activestate.com/recipes/510399-byte-to-hex-and-hex-to-byte-string-conversion/
def ByteToHex( byteStr ):
    """
    Convert a byte string to it's hex string representation e.g. for output.
    """
    
    # Uses list comprehension which is a fractionally faster implementation than
    # the alternative, more readable, implementation below
    #   
    #    hex = []
    #    for aChar in byteStr:
    #        hex.append( "%02X " % ord( aChar ) )
    #
    #    return ''.join( hex ).strip()        

    return ''.join( [ "%02X " % ord( x ) for x in byteStr ] ).strip()

def HexToByte( hexStr ):
    """
    Convert a string hex byte values into a byte string. The Hex Byte values may
    or may not be space separated.
    """
    # The list comprehension implementation is fractionally slower in this case    
    #
    #    hexStr = ''.join( hexStr.split(" ") )
    #    return ''.join( ["%c" % chr( int ( hexStr[i:i+2],16 ) ) \
    #                                   for i in range(0, len( hexStr ), 2) ] )
 
    bytes = []

    hexStr = ''.join( hexStr.split(" ") )

    for i in range(0, len(hexStr), 2):
        bytes.append( chr( int (hexStr[i:i+2], 16 ) ) )

    return ''.join( bytes )






