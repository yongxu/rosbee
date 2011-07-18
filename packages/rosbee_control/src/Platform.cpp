/*
 * Platform.cpp
 *
 *  Created on: Jul 18, 2011
 *      Author: ros
 */

#include <rosbee_control/Platform.h>

Platform::Platform()
{
	connected = false;
}

Platform::~Platform()
{
	if(connected) serialcon.closeDevice();
}

Platform Platform::getInstance()
{
	if(Pinstance == NULL)
	{
		Pinstance = new Platform();
	}
	return Pinstance;
}

bool Platform::connect(char* device)
{
	if(serialcon.openDevice(device,BAUTRATE,1,0,8,1,rlSerial::NONE)<0)
	{
		return connected = false;
	}
	read_status();
	return connected = true;
}


/*** platform control ***/
void Platform::move(int8_t speed,int8_t dir)
{
	//no need to move if the motion on the platform is disabled
	if(motion_enabled) return;
	static int count = 0;

	stringstream ss;

	ss << "PC$" << MOVE_CMD << ',' << count << ',' << speed << ',' << dir << ",#";

	write_to_platform(ss.str().c_str(),ss.str().length());

	count ++;
}

void Platform::Enable_motion(bool enable);

void Platform::pc_control(bool enable);

void Platform::clear_error();

void Platform::set_ultrasoon(bool enable);

int16_t* Platform::read_encoders();

int* Platform::read_ultrasoon();

void Platform::read_status();
/*** end platform control ***/
