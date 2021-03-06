/****
    This is part of the Retalis Language for Information Processing and Management in Robotics
    Copyright (C) 2014 __Pouyan Ziafati__ pziafati@gmail.com 

    Retalis is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Retalis is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.                   

    You should have received a copy of the GNU General Public License
    along with Retalis.  If not, see <http://www.gnu.org/licenses/>.	
****/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "SWI-cpp.h"


ros::NodeHandle n;
ros::Publisher outputEventsPub_;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
          
PREDICATE(retalis_output_initialize, 1)
{
  outputEventsPub_ = n.advertise<std_msgs::String>("retalisOutputEvents", 5000);
  return TRUE;
}


PREDICATE(output_to_ros, 1)
{
  //ROS_INFO("***** %s",(char *) A1);
  std_msgs::String event;
  event.data = (char *) A1;
  outputEventsPub_.publish(event);
  return TRUE;
}


