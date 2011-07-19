/*
 * Platform.cpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

#include <rosbee_control/Platform.h>

Platform* Platform::Pinstance=NULL;

Platform::Platform()
{
	connected = false;
}

Platform::~Platform()
{
	if(connected) serialcon.closeDevice();
}

Platform* Platform::getInstance()
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
	if(!motion_enabled || !connected) return;
	static int count = 0;

	stringstream ss;

	ss << MOVE_CMD << ',' << count << ',' << speed << ',' << dir;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;

	count ++;
}

void Platform::Enable_motion(bool enable)
{
	if(!connected) return;
	stringstream ss;
	motion_enabled = enable;

	ss << ENABLE_MOTION;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;
}

void Platform::pc_control(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ss << ENABLE_PC;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;
}

void Platform::clear_error()
{
	if(!connected) return;
	stringstream ss;
	ss << CLEAR_ERROR;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;
}

void Platform::set_ultrasoon(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ss << TOGGLE_US;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;
}

void Platform::read_encoders(int16_t* encoders)
{
	if(!connected) return;
	stringstream ss;
	ss << READ_ENCODERS;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;

	char buffer[50];
	bzero(buffer,50);

	if(!read_from_platform(buffer,50)) return;
	//write code to get both encoder values.
}

void Platform::read_ultrasoon(int* ultrasoon)
{
	if(!connected || ultrasoon_enable) return;
	stringstream ss;
	ss << READ_US;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;

	char buffer[50];
	bzero(buffer,50);

	if(!read_from_platform(buffer,50)) return;
	//write code to get all 10 ultrasoon values
}

void Platform::read_status()
{
	if(!connected)return;
	stringstream ss;
	ss << READ_STATUS;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length())) return;

	char buffer[50];
	bzero(buffer,50);

	if(!read_from_platform(buffer,50)) return;
	//set parameters
}
/*** end platform control ***/

bool Platform::write_to_platform(char* message,int size)
{
	//check if we are connected to the platform
	if(!connected)return false;

	//add "pc$" to the start and a "#\0" to the end
	char tmpmsg[size+5];
	strcpy(tmpmsg,"PC$");
	strcpy(tmpmsg+3,message);
	tmpmsg[size-1] = '#';
	tmpmsg[size] = 0;

	//write to the platform
	if(serialcon.writeBlock((unsigned char*)tmpmsg,size+5)<0) return false;
	return true;
}

bool Platform::read_from_platform(char* buffer, int size)
{
	//check if we are connected to the platform
	if(!connected)return false;

	//read from the platform
	/*TODO rewrite with solution to xbees lack of a buffer*/
	if(serialcon.readBlock((unsigned char*)buffer,size,TIMEOUT)<0)return false;
	return true;
}
