/********************************************************************
       program: Move_platform
       Author: Bram van de Klundert
       Date: 9-8-2011
       Description: move_platform will provide s
*********************************************************************/

#include "ros/ros.h"
#include "rosbee_control/Platform.h"
#include "rosbee_control/ultrasoon.h"
#include "rosbee_control/Noodstop.h"
#include <geometry_msgs/Twist.h>

#define PLATFORM_PORT "/dev/ttyUSB0"
#define SERIALTIMEOUT 200*1000
#define MULTI 2.0
#define MINHZ 1000000/5
//int EncoderRate = 5; //read at which to read the encoders and ultrasoon, note that the current paralax can only handle 8 messages/sec.

Platform* itsPlatform;
float x,rz;
timeval lastControl;


//service to enable ultrasoon sensors on the platform
bool setUltrasoon(rosbee_control::ultrasoon::Request  &req,
		rosbee_control::ultrasoon::Response &res )
{
	itsPlatform->set_ultrasoon((req.enable==1));
	return true;
}

bool setNoodstop(rosbee_control::Noodstop::Request  &req,
                rosbee_control::Noodstop::Response &res )
{
        if(req.stop==1)
        {
                itsPlatform->move(0,0);
                itsPlatform->setNoodstop(true);
        }
        else itsPlatform->setNoodstop(false);
        return true;
}

//callback to move the platform
void moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	itsPlatform->move(-msg->linear.x,-msg->angular.z);
	rz = -msg->angular.z;
	x = -msg->linear.x;
	gettimeofday(&lastControl,NULL);
}

int main(int argc, char** argv)
{
	timeval now;
	x = rz = 0.0;
	ros::init(argc,argv,"move_platform");
	ros::NodeHandle n;

	ros::Subscriber subx = n.subscribe("cmd_vel",5,moveCallback);
	ros::ServiceServer service = n.advertiseService("Noodstop", setNoodstop);

	gettimeofday(&lastControl,NULL);
//	ros::Rate encRate(EncoderRate);

	//init platform
	itsPlatform = Platform::getInstance(n);
	ROS_DEBUG_NAMED("platform", "conecting");
	while(!itsPlatform->connect(PLATFORM_PORT)) usleep(100);
	ROS_DEBUG_NAMED("platform", "conected");

	//enable the pc control and motion on the platform
//	itsPlatform->pc_control(true);
//	usleep(SERIALTIMEOUT);
//	itsPlatform->Enable_motion(true);
//	usleep(SERIALTIMEOUT);
	while(ros::ok())
	{
		gettimeofday(&now,NULL);
		if((lastControl.tv_usec+MINHZ) < ((now.tv_sec-lastControl.tv_sec)*1000000 + now.tv_usec))
		{
			itsPlatform->move(0,0);
			gettimeofday(&lastControl,NULL);
		}
		usleep(10000);
		ros::spinOnce();
	}
/*	while(ros::ok())
	{
		itsPlatform->read_encoders();
		if(itsPlatform->ultrasoon_enable)
		{
			usleep(SERIALTIMEOUT);
			itsPlatform->read_ultrasoon();
		}
		ros::spinOnce();
		encRate.sleep();
	}*/
}
