/*
 * Platform.cpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

/*TODO add comments to improve readability*/
/*TODO rewrite commands to use precompiler defines*/
#include <rosbee_control/Platform.h>

Platform* Platform::Pinstance=NULL;

Platform::Platform(ros::NodeHandle n)
{
	connected = false;
	pub = n.advertise<rosbee_control::encoders>("enc", 1);
	lwserialcon = NULL;
	bzero(readbuffer,NRMSGS*MSGLENGHT);
}

Platform::~Platform()
{
	if(connected)
	{
		connected = false;
		lwserialcon->~LightweightSerial();
		//pthread_join(*readthread,NULL);
		//pthread_join(*xbeethread,NULL);
		breadthread.join();
		bxbeethread.join();
	}
}

Platform* Platform::getInstance(ros::NodeHandle nh)
{
	if(Pinstance == NULL)
	{
		Pinstance = new Platform(nh);
	}
	return Pinstance;
}

bool Platform::connect(const char* device)
{

	if(connected && lwserialcon != NULL && lwserialcon->is_ok())
	{
		ROS_INFO_NAMED("serial","connection already open. return");
		return true;
	}
	//delete any existing connection
	else if (lwserialcon!=NULL && !lwserialcon->is_ok())delete lwserialcon;
	//open a new connection and check if opening was successful.
	ROS_DEBUG_NAMED("serial","opening serial connection.");
	lwserialcon = new LightweightSerial(device,BAUTRATE);
	if(!lwserialcon->is_ok())
	{
		ROS_DEBUG_NAMED("serial","failed to open serial connection.");
		return connected = false;
	}
	ROS_DEBUG_NAMED("readthread","starting read thread.");
	breadthread = boost::thread(Platform::readloop);
	//pthread_create(readthread,NULL,Platform::readloop,NULL);
	ROS_DEBUG_NAMED("xbeethread","starting xbee thread.");
	bxbeethread = boost::thread(Platform::handlexbee);
	//pthread_create(xbeethread,NULL,Platform::handlexbee,NULL);
	//sleep a short while to make sure the thread had time to start.
	sleep(1);
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
	ss << MOVE_CMD << ',' << (int)count << ',' << (int)speed << ',' << (int)dir << ',';

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
	ss << ENABLE_MOTION<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

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
	ss << ENABLE_PC<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

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
	ss << CLEAR_ERROR << ',';

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
	ss << TOGGLE_US << ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
		ROS_DEBUG_NAMED("platform","write to platform failed");
}

void Platform::read_encoders()
{
	if(!connected)
	{
		ROS_DEBUG_NAMED("platform","read_encoders(),not connected");
		return;
	}
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_encoders()");
	ss << READ_ENCODERS<< ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	if(!write_to_platform(writestring,ss.str().length()))
	{
		ROS_DEBUG_NAMED("platform","write to platform failed");
		return;
	}
}

void Platform::read_ultrasoon()
{
	if(!connected || ultrasoon_enable) return;
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_ultrasoon()");
	ss << READ_US << ',';

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
}

void Platform::read_status()
{
	if(!connected)return;
	stringstream ss;
	ss << READ_STATUS << ',';

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
	stringstream ss;

	//add "pc$" to the start and a "#\0" to the end
	char tmpmsg[size+6];
	ss << "PC$" << message << "#\r\0";
	strcpy(tmpmsg,ss.str().c_str());
	ROS_DEBUG_NAMED("serial","writing to serial string: \"%s\", tmpmsg: \"%s\"",message,tmpmsg);
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
	while(read[0] != '$')
	{
		lwserialcon->read(read);
	}

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

void* Platform::readloop(/*void* ret*/)
{
	int i = 0;
	char read[MSGLENGHT];
	Pinstance->writeindex = 0;
	while(Pinstance->connected)
	{
		bzero(read,MSGLENGHT);
		ROS_DEBUG_NAMED("readthread","waiting for \'$\'");
		while(read[0] != '$' && Pinstance->connected)
		{
			Pinstance->lwserialcon->read(read);
			usleep(10);
		}
		ROS_DEBUG_NAMED("readthread","\'$\' found starting read");
		i = 0;
		while(read[i] != '#' && i < MSGLENGHT-1 && Pinstance->connected)
		{
			if(read[i] != 0 && read[i] != '\r')
			{
				i++;
				read[i] = 0;
			}
			Pinstance->lwserialcon->read(read+i);
			usleep(10);
		}
		ROS_DEBUG_NAMED("readthread","msg read: \"%s\".",read);
		strcpy(Pinstance->readbuffer[Pinstance->writeindex%NRMSGS],read);
		Pinstance->writeindex ++;
	}
	return NULL;
}

void* Platform::handlexbee(/*void* ret*/)
{
	int readindex=0;
	while(Pinstance->connected)
	{
		if(readindex < Pinstance->writeindex)
		{
			ROS_DEBUG_NAMED("xbeethread","handling message: #%i",readindex);
			switch(atoi(strtok(Pinstance->readbuffer[readindex%NRMSGS]+1,",")))
			{
				case READ_ENCODERS: Pinstance->handle_encoder(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				case READ_STATUS: Pinstance->handle_status(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				case READ_US: Pinstance->handle_ultrasoon(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				default:ROS_DEBUG_NAMED("xbeethread","unknown message: %i",atoi(strtok(Pinstance->readbuffer[readindex%NRMSGS]+1,",")));
			}
			readindex++;
		}
		usleep(1000);
	}
	return NULL;
}

void Platform::handle_encoder(char* command)
{
	rosbee_control::encoders msg;
	char* tmp;
	int i = 0;
	ROS_DEBUG_NAMED("xbeethread","handle_encoder(\"%s\")",command);

	if(READ_ENCODERS != atoi(strtok(command+1,",")))
	{
		ROS_DEBUG_NAMED("xbeethread","read message doesnt match send type");
		return;
	}
	tmp = strtok(NULL,",");
	while(tmp != NULL && i < NR_ENCODER)
	{
		encoders[i]=atoi(tmp);
		i++;
		tmp = strtok(NULL,",");
	}
	ROS_DEBUG_NAMED("xbeethread","encoders 0:%i 1:%i",encoders[0],encoders[1]);
	msg.leftEncoder = encoders[0];
	msg.rightEncoder = encoders[1];
	pub.publish(msg);
	ROS_DEBUG_NAMED("xbeethread","encoder values published");
}

void Platform::handle_status(char* command)
{
	ROS_DEBUG_NAMED("xbeethread","handle_status(\"%s\")",command);
	if(READ_STATUS!=atoi(strtok(command+1,",")))
	{
		ROS_DEBUG_NAMED("xbeethread","read message doesnt match send type");
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

void Platform::handle_ultrasoon(char* command)
{
	char* tmp;
	int i = 0;
	ROS_DEBUG_NAMED("xbeethread","handle_ultrasoon(\"%s\")",command);
	if(READ_ENCODERS != atoi(strtok(command+1,",")))
	{
		ROS_DEBUG_NAMED("xbeethread","read message doesnt match send type");
		return;
	}

	tmp = strtok(NULL,",");
	while(tmp != NULL && i < NR_ULTRASOON)
	{
		ussensor[i]=atoi(tmp);
		i++;
		tmp = strtok(NULL,",");
	}
	ROS_DEBUG_NAMED("xbeethread","encoders 0:%i 1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i 9:%i",
					ussensor[0],ussensor[1],ussensor[2],ussensor[3],ussensor[4],ussensor[5],
					ussensor[6],ussensor[7],ussensor[8],ussensor[9]);
}
