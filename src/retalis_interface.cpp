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
#include "retalis/AddOutputSubscription.h"
#include "retalis/AddMemory.h"
#include "retalis/DeleteOutputSubscription.h"
#include "retalis/DeleteMemory.h"
//subscribers_manuals///////////
#include "tf/tfMessage.h"
#include "ar_pose/ARMarkers.h"
//////////////////////////////// 
#include <string>
#include <sstream>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


bool subscribe_tf_and_ar_pose_ = true;

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
		subscribeToTopics_ = nh_.advertiseService("add_output_subscription", &Retalis::subscribeSrv,this);
		memorizeEvents_    = nh_.advertiseService("add_memory", &Retalis::memorizeSrv,this);
		unSubscribeToTopics_ = nh_.advertiseService("delete_output_subscription", &Retalis::unSubscribeSrv,this);
		unMemorizeEvents_    = nh_.advertiseService("delete_memory", &Retalis::unMemorizeSrv,this);
		

		//subscribers_manuals///////////
		if(subscribe_tf_and_ar_pose_)
		{
			inputTFSub_ = nh_.subscribe("tf", 500, &Retalis::inputTFCB,this);
			inputARPoseSub_ = nh_.subscribe("ar_pose_marker",100,&Retalis::inputMarkersCB,this);
		}
		//timer to test performance // timer_ = nh_.createTimer(ros::Duration(1), &Retalis::timerCB, this);
		//////////////////////////////// 
		
	}

	~Retalis(void){
	}

	bool subscribeSrv(retalis::AddOutputSubscription::Request &req, retalis::AddOutputSubscription::Response &res)
	{	
		string conditions = "["+req.conditions+"]";
		string query = "subscribe("+req.event +", "+ conditions +", "+ req.format +", "+ req.topic+", "+req.id+")";
		mutex_.lock();
		try
			{
				PlCall(query.c_str());
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 			
		mutex_.unlock();
		res.result = "ok";
		return true;
	}
	bool memorizeSrv(retalis::AddMemory::Request &req, retalis::AddMemory::Response &res)
	{	
		string conditions = "["+req.conditions+"]";
		string query = "memorize("+req.event +", "+ conditions +", "+ req.format +", "+ req.id+", "+SSTR(req.size)+")";
		mutex_.lock();
		try
			{
				PlCall(query.c_str());
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 			
		mutex_.unlock();
		res.result = "ok";
		return true;
	}

	bool unSubscribeSrv(retalis::DeleteOutputSubscription::Request &req, retalis::DeleteOutputSubscription::Response &res)
	{	
		
		string query = "delete_subscription("+req.topic+req.id+")";
		mutex_.lock();
		try
			{
				PlCall(query.c_str());
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 			
		mutex_.unlock();
		res.result = "ok";
		return true;
	}
	bool unMemorizeSrv(retalis::DeleteMemory::Request &req, retalis::DeleteMemory::Response &res)
	{	
		
		string query = "delete_memory("+ req.id+")";
		mutex_.lock();
		try
			{
				PlCall(query.c_str());
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 			
		mutex_.unlock();
		res.result = "ok";
		return true;
	}
	void inputEventsCB(const std_msgs::String::ConstPtr& event)
	{	PlFrame fr;
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
		PlFrame fr;
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
				//PlCompound headerMsg("std_msgs__0__Header",header);
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



		void inputMarkersCB(const ar_pose::ARMarkers::ConstPtr& msg)
	{	
		PlFrame fr;
		PlTermv markers(1);
  		PlTail markersA(markers[0]);

		std::vector<ar_pose::ARMarker>::const_iterator iter;
 		for (iter = msg->markers.begin(); iter != msg->markers.end(); iter++)
			{
				PlTermv marker(4);  				

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
				PlCompound head("std_msgs__0__Header",header);
				marker[0] = head;

				marker[1] = PlTerm((long) (*iter).id);				

				PlTermv vector(3);
				vector[0] = PlTerm( (double)(*iter).pose.pose.position.x);
				vector[1] = PlTerm( (double)(*iter).pose.pose.position.y);
				vector[2] = PlTerm( (double)(*iter).pose.pose.position.z);
				PlCompound vec("geometry_msgs__0__Point",vector);
				PlTermv quaternion(4);
				quaternion[0] = PlTerm( (double)(*iter).pose.pose.orientation.x);
				quaternion[1] = PlTerm( (double)(*iter).pose.pose.orientation.y);
				quaternion[2] = PlTerm( (double)(*iter).pose.pose.orientation.z);
				quaternion[3] = PlTerm( (double)(*iter).pose.pose.orientation.w);
				PlCompound quat("geometry_msgs__0__Quaternion",quaternion);
				PlTermv vq(2);
				vq[0] = vec;
				vq[1] = quat;
				PlCompound vecquat("geometry_msgs__0__Pose",vq);
				
				PlTermv cov(1);
				PlTail covA(cov[0]);
 				for (int i =0; i<36; i++)
				{
					covA.append((double)(*iter).pose.covariance[i]);

				}	
				covA.close();
				PlTermv pos(2);
				pos[0] = vecquat;
				pos[1] = cov[0];
				PlCompound posWithCov("geometry_msgs__0__PoseWithCovariance",pos);
				marker[2] = posWithCov;
				

				marker[3] = PlTerm((long) (*iter).confidence);

				PlCompound finalMarker("ar_pose__0__ARMarker",marker);
				markersA.append(finalMarker);
			}
		markersA.close();
			
	
		PlTermv  headerTop(3);
		headerTop[0] = PlTerm((long) msg->header.seq);	
		PlTermv timeTop(1);
		PlTail timeTopA(timeTop[0]);
		timeTopA.append((long)msg->header.stamp.sec);
		timeTopA.append((long)msg->header.stamp.nsec);
		timeTopA.close();
		headerTop[1] =  timeTop[0];
		string frameIDRetalisTop = "\""+msg->header.frame_id+"\"";
		headerTop[2] = PlTerm(frameIDRetalisTop.c_str());	
		PlCompound headTop("std_msgs__0__Header",headerTop);

		PlTermv markerMsg(2);
  		markerMsg[0] = headTop;
		markerMsg[1] = markers[0];
		PlCompound markerMsgFinal("ar_pose__0__ARMarkers",markerMsg);
		
		PlTermv event(3);
		event[0] = markerMsgFinal;
		event[1] = PlTerm( (long)msg->header.stamp.sec);
		event[2] = PlTerm( (long)msg->header.stamp.nsec);
		mutex_.lock();
		try
			{
				PlCall("new_event", event);
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 
		mutex_.unlock(); 
	}
	//timer to test performance
	/*
	void timerCB(const ros::TimerEvent& event)
	{
		
		mutex_.lock();
		try
			{	
				long x = (long) ros::Time::now().toSec();
				double y = ros::Time::now().toSec() - x;
				long z = (long) (y * 1000000000);	
				PlTermv test(2);
				test[0] = PlTerm(x);
				test[1] = PlTerm(z);
				
				PlCall("test_query", test);
			}
		catch ( PlException &ex )
  			{ 
				std::cerr << (char *) ex << std::endl;
  			} 
		mutex_.unlock(); 
	}*/	
	////////////////////////////////


protected:
	ros::NodeHandle nh_;
        PlEngine swipl_;
	ros::Subscriber inputEventsSub_;
	boost::mutex mutex_;
	ros::ServiceServer subscribeToTopics_;
	ros::ServiceServer memorizeEvents_;
	ros::ServiceServer unSubscribeToTopics_;
	ros::ServiceServer unMemorizeEvents_;
	//subscribers_manuals///////////
	ros::Subscriber inputTFSub_;
	ros::Subscriber  inputARPoseSub_;
	// timer to test performance /// ros::Timer timer_;
	////////////////////////////////

};




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "retalis");
  Retalis retalis(argv);
  ros::spin();
  return 0;
}

 
