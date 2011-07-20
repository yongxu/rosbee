/*
 * Platform.cpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

/*TODO add debug prints*/
/*TODO add comments to improve readability*/
/*TODO rewrite commands to precompiler defines*/
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
	if(connected && lwserialcon->is_ok()) return true;
	//delete any existing connection
	else if (!lwserialcon->is_ok()) delete lwserialcon;
	//open a new connection and check if opening was successful.
	lwserialcon = new LightweightSerial(device,BAUTRATE);
	if(!lwserialcon->is_ok()) return connected = false;
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

	char tmpbuff[10];
	bzero(tmpbuff,10);
	char msgnr[4];
	char* comma;
	char* comma2;
	bzero(msgnr,4);
	memcpy(msgnr,buffer+1,3);
	int i = 0;
	/*TODO rewrite command splitting*/
	if(READ_ENCODERS != atoi(msgnr)) return; //maybe change to loop and wait for encoder values


	comma = strchr(buffer,',');
	comma2 = strchr(comma+1,',');

	while(comma2 != 0 && i < 2)
	{
		memcpy(tmpbuff,comma+1,comma2-comma);
		encoders[i] =  atoi(tmpbuff); //get the decimal value of the char array
		bzero(tmpbuff,10);
		comma = comma2;
		comma2 = strchr(comma+1,',');
		i++;
	}



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

	char tmpbuff[10];
	bzero(tmpbuff,10);
	char msgnr[4];
	char* comma;
	char* comma2;
	bzero(msgnr,4);
	memcpy(msgnr,buffer+1,3);
	int i = 0;

	/*TODO rewrite command splitting*/
	if(READ_ENCODERS != atoi(msgnr)) return; //maybe change to loop and wait for encoder values

	comma = strchr(buffer,',');
	comma2 = strchr(comma+1,',');

	while(comma2 != 0 && i < NR_ULTRASOON)
	{
		memcpy(tmpbuff,comma+1,comma2-comma);
		encoders[i] =  atoi(tmpbuff); //get the decimal value of the char array
		bzero(tmpbuff,10);
		comma = comma2;
		comma2 = strchr(comma+1,',');
		i++;
	}
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
	/*TODO set parameters*/
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
	return lwserialcon->write_block(tmpmsg,size+5);
}

bool Platform::read_from_platform(char* buffer, int size)
{
	/*TODO figure out what to do with incomplete data*///if incomplete buffer didnt change

	//check if we are connected to the platform
	if(!connected)return false;

	//read from the platform
	int i = 0;
	char read[size];
	bzero(read,size);

	while(read[0] != '$')if(!lwserialcon->read(read)) return false;

	while(read[i] != '#' && i < size-1)
	{
		if(read[i] != ' ' && read[i] != '\r')
		{
			i++;
			read[i] = ' ';
		}
		if(!lwserialcon->read((read+i))) return false;
	}
	strcpy(buffer,read);
	return true;
}
