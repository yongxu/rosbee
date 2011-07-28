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
#define PULSEDISTANCEM 15.8/1000

using namespace std;

Platform* itsPlatform;


void move(const geometry_msgs::Twist::ConstPtr& msg)
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
	char readchar = 0;
	int movespeed,movedir;
	int enable_in;
	//init ros stuff
	ros::init(argc,argv,"tester");
	ros::NodeHandle n;	
	ros::Subscriber subx = n.subscribe("cmd_vel",10,move);
 	ros::Publisher pub = n.advertise<rosbee_control::encoders>("enc", 1);
	ros::Rate LoopRate(4);
	rosbee_control::encoders msg;

	//init platform
	itsPlatform = Platform::getInstance(n);
	itsPlatform->connect("/dev/ttyUSB0");


	itsPlatform->pc_control(true);
	usleep(SERIALTIMEOUT);
	itsPlatform->Enable_motion(true);
	usleep(SERIALTIMEOUT);
	
	while(ros::ok())
	{
		cout << "select option" << endl;
		cout << "1. move" << endl;
		cout << "2. read encoders" <<endl;
		cout << "3. read ultrasoon" <<endl;
		cout << "4. read status" <<endl;
		cout << "5. set ultrasoon" <<endl;
		cout << "6. clear errors" <<endl;
		cin >> readchar;
		
		switch(readchar)
		{
		case '1': cout << endl << "enter speed (-128 to 127)" << endl;
			cin >> movespeed;
			cout << endl << "enter dir (-128 to 127)" << endl;
			cin >> movedir;
			itsPlatform->move(movespeed,movedir);
			break;
		case '2': itsPlatform->read_encoders();
			cout << "encoders, ";
			for(int i = 0; i< NR_ENCODER; i++)
			{
				cout << i << ": " << itsPlatform->encoders[i] << ", ";
			}
			cout << endl;
			break;
		case '3': itsPlatform->read_ultrasoon();
			cout << "ultrasoon, ";
			for(int i = 0; i< NR_ULTRASOON; i++)
			{
				cout << i << ": " << itsPlatform->ussensor[i] << ", ";
			}
			cout << endl;
			break;
		case '4': itsPlatform->read_status();
			cout << "status, Movemode: " << itsPlatform->Movemode << ", Lastalarm: " << itsPlatform->Lastalarm << ", Xbeetime: " << itsPlatform->Xbeetime << ", Ppcgetcntr: " << itsPlatform->Ppcgetcntr << ", Platenable: " << itsPlatform->Platenable << ", Pcenable: " << itsPlatform->Pcenable << ", Pfstatus: " << itsPlatform->Pfstatus << ", Maincntr: " << itsPlatform->Maincntr << ", Safetycntr: " << itsPlatform->Safetycntr << ", Version: " << endl;
			break;
		case '5': cout << endl << "enable (0 or 1)" << endl;
			cin >> enable_in;
			itsPlatform->set_ultrasoon((enable_in==0)?false:true);
			break;
		case '6': itsPlatform->clear_error();
			break;
		default: cout << "unknown option" << endl;
		}
		ros::spinOnce();
		LoopRate.sleep();
	}
	
}


