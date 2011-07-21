/********************************************************************
       program: Tester
       Author: Michiel van Osch
       Date: 11-7-2001
       Description: Tester is a test program for testing the PlatformAPI
                    for Keila's Unicycle. With this program I was able
                    to communicate with the platform and read sensor
                    values, and either set PCcontrol or motion enabled.
                    However when I set control and motion after eachother
                    the program gets stuck. Probably because you are
                    not allowed to do this (and need to query in between
                    or because some serial buffer does nog get flushed
                    properly between commands.
                    
                    If I interleave with get_Rwheelpositions() (which
                    now return ultrasoundsensorvalues I beleive
                    the Tester works properly but reading sensor values
                    disables the motion or control enabled.

                    Therefore, the platform did NOT move yet using this
                    program.
*********************************************************************/

#include "rosbee_control/Platform.h"
#include <string>
#include <iostream>
#include <rosbee_control/encoders.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include <sys/time.h>

#define SERIALTIMEOUT 200*1000
#define PULSEDISTANCEM		15.8/1000	

using namespace std;

Platform* itsPlatform;


void move(const geometry_msgs::Twist::ConstPtr& msg)
{
	/*
	int16_t startVal[2];
	int16_t encVal[2];		
	int x = 0;

	itsPlatform->read_encoders(startVal);
	x = msg->linear.x/PULSEDISTANCEM;

	usleep(SERIALTIMEOUT);
	itsPlatform->move(128,0);

	do
	{
		itsPlatform->read_encoders(encVal);
		encVal[0] -= startVal[0];
		encVal[1] -= startVal[1];

		usleep(SERIALTIMEOUT);

	}while(encVal[0] < x);
	
	itsPlatform->move(0,0); */
	static timeval last;
	static float x =0;
	static float y =0;

	timeval now;

	x += msg->linear.x;
	y += msg->linear.y;

	gettimeofday(&now,NULL);

	if(last.tv_usec < now.tv_usec - 200000 || last.tv_sec < now.tv_sec)
	{
		itsPlatform->move((int)x,(int)(y));
		gettimeofday(&last,NULL);
	}	
}


int main(int argc, char** argv)
{
	//init ros stuff
	ros::init(argc,argv,"tester");
	ros::NodeHandle n;	
	ros::Subscriber subx = n.subscribe("cmd_vel",10,move);
 	ros::Publisher pub = n.advertise<rosbee_control::encoders>("enc", 1);
	ros::Rate LoopRate(4);
	rosbee_control::encoders msg;

	//init platform
	itsPlatform = Platform::getInstance();
	itsPlatform->connect("/dev/ttyUSB0");
	itsPlatform->pc_control(true);
	usleep(SERIALTIMEOUT);
	itsPlatform->Enable_motion(true);
	usleep(SERIALTIMEOUT);
	
	int16_t encVal[2];	
	
	while(ros::ok())
	{
		itsPlatform->read_encoders(encVal);
		msg.leftEncoder = encVal[0];
		msg.rightEncoder = encVal[1];
		pub.publish(msg);
		
		ros::spinOnce();
		LoopRate.sleep();
	}
	
}


