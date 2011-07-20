/*
 * Platform.cpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

/*TODO add debug prints*/
/*TODO add comments to improve readability*/
/*TODO rewrite commands to use precompiler defines*/
#include <rosbee_control/Platform.h>

Platform* Platform::Pinstance=NULL;

Platform::Platform()
{
	connected = false;
}

Platform::~Platform()
{
	if(connected) lwserialcon->~LightweightSerial();
}

Platform* Platform::getInstance()
{
	if(Pinstance == NULL)
	{
		Pinstance = new Platform();
	}
	return Pinstance;
}

bool Platform::connect(const char* device)
{

	if(connected && lwserialcon->is_ok())
	{
		ROS_DEBUG_NAMED("serial","connection already open. return");
		return true;
	}
	//delete any existing connection
	else if (!lwserialcon->is_ok()) delete lwserialcon;
	//open a new connection and check if opening was successful.
	ROS_DEBUG_NAMED("serial","opening serial connection.");
	lwserialcon = new LightweightSerial(device,BAUTRATE);
	if(!lwserialcon->is_ok())
	{
		ROS_DEBUG_NAMED("serial","failed to open serial connection.");
		return connected = false;
	}
	//if the connection was opened successfully get the status of the platform
	read_status();
	return connected = true;
}


/*** platform control ***/
void Platform::move(int8_t speed,int8_t dir)
{
	//no need to move if the motion on the platform is disabled
	if(!motion_enabled || !connected) return;
	static int count = 0;

	ROS_DEBUG_NAMED("platform","move(%i,%i)",speed,dir);
	stringstream ss;
	ss << MOVE_CMD << ',' << count << ',' << speed << ',' << dir;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");

	count ++;
}

void Platform::Enable_motion(bool enable)
{
	if(!connected) return;
	stringstream ss;
	motion_enabled = enable;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	ss << ENABLE_MOTION;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");
}

void Platform::pc_control(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	ss << ENABLE_PC;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");
}

void Platform::clear_error()
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","clear_erros()");
	ss << CLEAR_ERROR;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");
}

void Platform::set_ultrasoon(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","set_ultrasoon(%s)",(enable)?"true":"false");
	ss << TOGGLE_US;
	if(enable) ss << 1;
	else ss << 0;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");
}

void Platform::read_encoders(int16_t* encoders_)
{
	if(!connected) return;
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_encoders()");
	ss << READ_ENCODERS;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
	{
		ROS_DEBUG_NAMED("platform","write to platform failed");
		return;
	}

	char buffer[50];
	bzero(buffer,50);

	if(!read_from_platform(buffer,50))
	{
		ROS_DEBUG_NAMED("platform","read from platform failed");
		return;
	}

	char* tmp;
	int i = 0;

	if(READ_ENCODERS != atoi(strtok(buffer+1,",")))
	{
		ROS_DEBUG_NAMED("platform","read message doesnt match send type");
		return;
	}

	tmp = strtok(NULL,",");
	while(tmp != NULL && i < NR_ENCODER)
	{
		encoders[i]=atoi(tmp);
		i++;
		tmp = strtok(NULL,",");
	}
	ROS_DEBUG_NAMED("platform","encoders 0:%i 1:%i",encoders[0],encoders[1]);
	memcpy(encoders,encoders_,NR_ENCODER*sizeof(int16_t));
}

void Platform::read_ultrasoon(int* ultrasoon)
{
	if(!connected || ultrasoon_enable) return;
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_ultrasoon()");
	ss << READ_US;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
	{
		ROS_DEBUG_NAMED("platform","write to platform failed");
		return;
	}

	char buffer[50];
	bzero(buffer,50);

	if(!read_from_platform(buffer,50))
	{
		ROS_DEBUG_NAMED("platform","read from platform failed");
		return;
	}

	char* tmp;
	int i = 0;

	if(READ_US != atoi(strtok(buffer+1,",")))
	{
		ROS_DEBUG_NAMED("platform","read message doesnt match send type");
		return;
	}

	tmp = strtok(NULL,",");
	while(tmp != NULL && i < NR_ULTRASOON)
	{
		ussensor[i]=atoi(tmp);
		i++;
		tmp = strtok(NULL,",");
	}
	ROS_DEBUG_NAMED("platform","encoders 0:%i 1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i 9:%i",
				ussensor[0],ussensor[1],ussensor[2],ussensor[3],ussensor[4],ussensor[5],
				ussensor[6],ussensor[7],ussensor[8],ussensor[9]);
	memcpy(ussensor,ultrasoon,NR_ENCODER*sizeof(int16_t));
}

void Platform::read_status()
{
	if(!connected)return;
	stringstream ss;
	ss << READ_STATUS;

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
	{
		ROS_DEBUG_NAMED("platform","write to platform failed");
		return;
	}

	char buffer[80];
	bzero(buffer,80);

	if(!read_from_platform(buffer,80))
	{
		ROS_DEBUG_NAMED("platform","read from platform failed");
		return;
	}

	if(READ_STATUS!=atoi(strtok(buffer+1,",")))
	{
		ROS_DEBUG_NAMED("platform","read message doesnt match send type");
		return;
	}

	Movemode = atoi(strtok(NULL,","));
	Lastalarm = atoi(strtok(NULL,","));
	Xbeetime = atoi(strtok(NULL,","));
	Ppcgetcntr = atoi(strtok(NULL,","));
	Platenable = atoi(strtok(NULL,","));
	Pcenable = atoi(strtok(NULL,","));
	Pfstatus = atoi(strtok(NULL,","));
	Maincntr = atoi(strtok(NULL,","));
	Safetycntr = atoi(strtok(NULL,","));
	Version = atoi(strtok(NULL,","));


}
/*** end platform control ***/

bool Platform::write_to_platform(char* message,int size)
{
	//check if we are connected to the platform
	if(!connected)
	{
		ROS_DEBUG_NAMED("serial","serial not connected, when trying to write");
		return false;
	}

	//add "pc$" to the start and a "#\0" to the end
	char tmpmsg[size+6];
	strcpy(tmpmsg,"PC$");
	strcpy(tmpmsg+3,message);
	tmpmsg[size-3] = '#';
	tmpmsg[size-2] = '\r';
	tmpmsg[size-1] = 0;

	ROS_DEBUG_NAMED("serial","writing to serial string: \"%s\"",tmpmsg);
	//write to the platform
	return lwserialcon->write_block(tmpmsg,size+5);
}

bool Platform::read_from_platform(char* buffer, int size)
{
	/*TODO figure out what to do with incomplete data*///right now: if incomplete buffer didnt change

	//check if we are connected to the platform
	if(!connected)
	{
		ROS_DEBUG_NAMED("serial","serial not connected, when trying to read");
		return false;
	}

	//read from the platform
	int i = 0;
	char read[size];
	bzero(read,size);

	ROS_DEBUG_NAMED("serial","starting read, searching for \"$\"");
	while(read[0] != '$')lwserialcon->read(read);

	while(read[i] != '#' && i < size-1)
	{
		if(read[i] != ' ' && read[i] != '\r')
		{
			i++;
			read[i] = ' ';
		}
		lwserialcon->read(read+i);
	}
	ROS_DEBUG_NAMED("serial","done reading, received \"%s\"",read);
	strcpy(buffer,read);
	return true;
}
