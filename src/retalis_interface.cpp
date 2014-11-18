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
#include <sys/stat.h>
#include <boost/thread/condition.hpp>
#include <termios.h>
#include <unistd.h>
#include <ros/package.h>
#include "tinyxml.h"
#include <fstream>
//subscribers_manuals///////////
#include "tf/tfMessage.h"	
#include <string>
//////////////////////////////// 

bool subscribe_to_tf_ = true;

using namespace std;
const string pkgPath = ros::package::getPath("retalis");
const string etalisSource = "consult('"+pkgPath+"/prolog_source/etalis.P"+"')";
const string retalisSource = "consult('"+pkgPath+"/prolog_source/retalis.P"+"')";
 
class Retalis
{
public:
	Retalis(char **argv):
	swipl_(argv[0])
	{
		try
  			{ 
				PlCall("use_foreign_library(foreign(retalis_output_interface))");
				PlCall("retalis_output_initialize(1)");
				PlCall(etalisSource.c_str());
				PlCall(retalisSource.c_str());

				string goalPredicatesFilePath = pkgPath+"/application_source/goalPredicates.txt";
				std::ifstream goalPredicatesFile(goalPredicatesFilePath.c_str());
				for( string line; getline( goalPredicatesFile, line ); )
				{
				    if(!line.empty())
				    {
					PlCall(line.c_str());

				    }
				}
 			} 
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			}

		inputEventsSub_ = nh_.subscribe("retalisInputEvents", 5000, &Retalis::inputEventsCB,this);
	
		//subscribers_manuals///////////
		if(subscribe_to_tf_)
			inputTFSub_ = nh_.subscribe("tf", 500, &Retalis::inputTFCB,this);
		//////////////////////////////// 
		
	}

	~Retalis(void){
	}

	void inputEventsCB(const std_msgs::String::ConstPtr& event)
	{	
		mutex_.lock();
		try
			{
				PlCall(event->data.c_str());
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 			
		mutex_.unlock();
	}

	//subscribers_manuals///////////
	void inputTFCB(const tf::tfMessage::ConstPtr& msg)
	{	
		PlTermv transforms(1);
  		PlTail transformsA(transforms[0]);

		std::vector<geometry_msgs::TransformStamped>::const_iterator iter;
 		for (iter = msg->transforms.begin(); iter != msg->transforms.end(); iter++)
			{
				PlTermv trans(3);  				

				PlTermv  header(3);
				header[0] = PlTerm((long) (*iter).header.seq);	
				PlTermv time(1);
				PlTail timeA(time[0]);
				timeA.append((long)(*iter).header.stamp.sec);
				timeA.append((long)(*iter).header.stamp.nsec);
				timeA.close();
				header[1] =  time[0];
				string frameIDRetalis = "\""+(*iter).header.frame_id+"\"";
				header[2] = PlTerm(frameIDRetalis.c_str());	
				PlCompound headerMsg("std_msgs__0__Header",header);
				PlCompound head("std_msgs__0__Header",header);
				trans[0] = head;

				string childframeIDRetalis = "\""+(*iter).child_frame_id+"\"";
				trans[1] = PlTerm(childframeIDRetalis.c_str());				

				PlTermv vector(3);
				vector[0] = PlTerm( (double)(*iter).transform.translation.x);
				vector[1] = PlTerm( (double)(*iter).transform.translation.y);
				vector[2] = PlTerm( (double)(*iter).transform.translation.z);
				PlCompound vec("geometry_msgs__0__Vector3",vector);
				PlTermv quaternion(4);
				quaternion[0] = PlTerm( (double)(*iter).transform.rotation.x);
				quaternion[1] = PlTerm( (double)(*iter).transform.rotation.y);
				quaternion[2] = PlTerm( (double)(*iter).transform.rotation.z);
				quaternion[3] = PlTerm( (double)(*iter).transform.rotation.w);
				PlCompound quat("geometry_msgs__0__Quaternion",quaternion);
				PlTermv vq(2);
				vq[0] = vec;
				vq[1] = quat;
				PlCompound vecquat("geometry_msgs__0__Transform",vq);
				trans[2] = vecquat;
				
				PlCompound transform("geometry_msgs__0__TransformStamped",trans);
				transformsA.append(transform);
			}
		transformsA.close();
		PlCompound tf("tf__0__tfMessage",transforms);
		//ROS_INFO("%s",(char *)tf);
		mutex_.lock();
		try
			{
				PlCall("event", tf);
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 
		mutex_.unlock(); 
	}
	////////////////////////////////


protected:
	ros::NodeHandle nh_;
        PlEngine swipl_;
	ros::Subscriber inputEventsSub_;
	boost::mutex mutex_;
	//subscribers_manuals///////////
	ros::Subscriber inputTFSub_;
	////////////////////////////////

};




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "retalis");
  Retalis retalis(argv);
  ros::spin();
  return 0;
}

 
