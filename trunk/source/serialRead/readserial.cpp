#include <ros/ros.h>
#include <serial_port/lightweightserial.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#define PATH "/dev/ttyUSB0"
#define BAUD 	115200


	void readLine(char * buffer)
	{
		char read;
		stringstream ss;	
		LightweightSerial serial = LightweightSerial(PATH,BAUD);

		while(read != '#')
		{
			read = ' ';
			serial.read(&read);	

			if(read != ' ' && read !='\r')
			{
				ss << read;		
				if(read == '#')
				{
						ROS_DEBUG_NAMED("Serial","READ: %s",ss.str().c_str());
				}
			}
		}
		sprintf(buffer,"%s",ss.str().c_str());
	}


	int main()
	{
		while(1)
		{
			char buffer[40];
			bzero(buffer,40);

			readLine(buffer);
			cout << buffer << endl;
		}
		return 0;

	}
