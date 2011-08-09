/********************************************************************
       program: Move_platform
       Author: Bram van de Klundert
       Date: 9-8-2011
       Description: move_platform will provide s
*********************************************************************/

#include "ros/ros.h"
#include "rosbee_control/Platform.h"
#include "rosbee_control/ultrasoon.h"
#include <geometry_msgs/Twist.h>

#define PLATFORM_PORT "/dev/ttyUSB0"
#define SERIALTIMEOUT 200*1000

int EncoderRate = 8; //read at which to read the encoders and ultrasoon, note that the current paralax can only handle 8 messages/sec.

Platform* itsPlatform;

//service to enable ultrasoon sensors on the platform
bool setUltrasoon(rosbee_control::ultrasoon::Request  &req,
		rosbee_control::ultrasoon::Response &res )
{
	itsPlatform->set_ultrasoon((req.enable==1));
	return true;
}

//callback to move the platform
void moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	static timeval last;
	timeval now;

	gettimeofday(&now,NULL);
	if(last.tv_usec < now.tv_usec - 200000 || last.tv_sec < now.tv_sec)
	{
		itsPlatform->move((int)(msg->linear.x*63.5),(int)(msg->angular.z*63.5));
		gettimeofday(&last,NULL);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"move_platform");
	ros::NodeHandle n;

	ros::Subscriber subx = n.subscribe("cmd_vel",5,moveCallback);

	ros::Rate encRate(EncoderRate);

	//init platform
	itsPlatform = Platform::getInstance(n);
	itsPlatform->connect(PLATFORM_PORT);

	//enable the pc control and motion on the platform
	itsPlatform->pc_control(true);
	usleep(SERIALTIMEOUT);
	itsPlatform->Enable_motion(true);
	usleep(SERIALTIMEOUT);

	while(ros::ok())
	{
		itsPlatform->read_encoders();
		if(itsPlatform->ultrasoon_enable)
		{
			usleep(SERIALTIMEOUT);
			itsPlatform->read_ultrasoon();
		}
		ros::spinOnce();
		encRate.sleep();
	}
}
