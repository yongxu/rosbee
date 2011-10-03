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
	writewindex = 0;
}

Platform::~Platform()
{
	if(connected)
	{
		connected = false;
		lwserialcon->~LightweightSerial();
		breadthread->join();
		bxbeethread->join();
		bwritethread->join();
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
	else connected = true;

	ROS_DEBUG_NAMED("readthread","starting read thread.");
	breadthread = new boost::thread(&Platform::readloop);
	ROS_DEBUG_NAMED("xbeethread","starting xbee thread.");
	bxbeethread = new boost::thread(&Platform::handlexbee);
	ROS_DEBUG_NAMED("writethread","starting write thread.");
	bwritethread = new boost::thread(&Platform::writeloop);
	//sleep a short while to make sure the thread had time to start.
	sleep(1);
	//if the connection was opened successfully get the status of the platform
	read_status();
	return connected;
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
	addwritelist(writestring,ss.str().length());

	count ++;
}

void Platform::Enable_motion(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	motion_enabled = enable;
	ss << ENABLE_MOTION<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());
}

void Platform::pc_control(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	pc_control_enabled = enable;
	ss << ENABLE_PC<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());
}

void Platform::clear_error()
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","clear_erros()");
	ss << CLEAR_ERROR << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());
}

void Platform::set_ultrasoon(bool enable)
{
	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","set_ultrasoon(%s)",(enable)?"true":"false");
	ultrasoon_enable = enable;
	ss << TOGGLE_US << ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());
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
	addwritelist(writestring,ss.str().length());
}

void Platform::read_ultrasoon()
{
	if(!connected || ultrasoon_enable) return;
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_ultrasoon()");
	ss << READ_US << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());

}

void Platform::read_status()
{
	if(!connected)return;
	stringstream ss;
	ss << READ_STATUS << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());
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

	//add "pc$" to the start and a "#\r\0" to the end
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
		if(!lwserialcon->read(read))
			ROS_WARN_NAMED("serial","reading from serial returned false, while waiting for \'$\'");
	}

	while(read[i] != '#' && i < size-1)
	{
		if(read[i] != ' ' && read[i] != '\r')
		{
			i++;
			read[i] = ' ';
		}
		if(!lwserialcon->read(read+i))
			ROS_WARN_NAMED("serial","reading from serial returned false, while reading the message");
	}
	ROS_DEBUG_NAMED("serial","done reading, received \"%s\"",read);
	strcpy(buffer,read);
	return true;
}

void Platform::addwritelist(char* message,int size)
{
	memcpy(writebuffer[writewindex%NRWRITEMSGS],message,size);
	writewindex++;
}

void* Platform::writeloop()
{
	int lastwriten = 0;
	stringstream ss;

	ros::Rate writerate(WRITEFREQ); // create rate object to slow the the writing
	while(Pinstance->connected)
	{
		if(lastwriten < Pinstance->writewindex)
		{
			lastwriten++;
			ss << Pinstance->writebuffer[lastwriten%NRWRITEMSGS];
			if(!Pinstance->write_to_platform(Pinstance->writebuffer[lastwriten%NRWRITEMSGS],ss.str().length()))
				ROS_WARN_NAMED("serial","write to serial returned false.");

			ss.str(string());
		}
		writerate.sleep();
	}
	return NULL;
}

void* Platform::readloop()
{
	int i = 0;
	char read[MSGLENGHT];
	Pinstance->readwindex = 0;
	while(Pinstance->connected)
	{
		bzero(read,MSGLENGHT);
		ROS_DEBUG_NAMED("readthread","waiting for \'$\'");
		while(read[0] != '$' && Pinstance->connected)
		{
			if(!Pinstance->lwserialcon->read(read))
				ROS_WARN_NAMED("serial","reading from serial returned false, while waiting for \'$\'");
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
			if(!Pinstance->lwserialcon->read(read+i))
				ROS_WARN_NAMED("serial","reading from serial returned false, while reading the message");
			usleep(10);
		}
		ROS_DEBUG_NAMED("readthread","msg read: \"%s\".",read);
		strcpy(Pinstance->readbuffer[Pinstance->readwindex%NRMSGS],read);
		Pinstance->readwindex ++;
	}
	return NULL;
}

void* Platform::handlexbee()
{
	int readindex=0;
	char splitmsg[10];
	while(Pinstance->connected)
	{
		if(readindex < Pinstance->readwindex)
		{
			ROS_DEBUG_NAMED("xbeethread","handling message: #%i",readindex);
			bzero(splitmsg,10);
			memcpy(splitmsg,Pinstance->readbuffer[readindex%NRMSGS]+1,3);
			switch(atoi(strtok(splitmsg,",")))
			{
				case READ_ENCODERS: Pinstance->handle_encoder(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				case READ_STATUS: Pinstance->handle_status(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				case READ_US: Pinstance->handle_ultrasoon(Pinstance->readbuffer[readindex%NRMSGS]);
					break;
				default: ROS_DEBUG_NAMED("xbeethread","unknown message: %i",atoi(strtok(Pinstance->readbuffer[readindex%NRMSGS]+1,",")));
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
	msg.leftEncoder = encoders[1];
	msg.rightEncoder = encoders[0];
	pub.publish(msg);
	ROS_DEBUG_NAMED("xbeethread","encoder values published");
}

void Platform::handle_status(char* command)
{
	ROS_DEBUG_NAMED("xbeethread","handle_status(\"%s\")",command);
	char* splitptr;
	if(READ_STATUS!=atoi(strtok(command+1,",")))
	{
		ROS_DEBUG_NAMED("xbeethread","read message doesnt match send type");
		return;
	}

	if((splitptr = strtok(NULL,",")) != NULL) Movemode = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Movemode");
	if((splitptr = strtok(NULL,",")) != NULL) Lastalarm = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Lastalarm");
	if((splitptr = strtok(NULL,",")) != NULL) Xbeetime = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Xbeetime");
	if((splitptr = strtok(NULL,",")) != NULL) Ppcgetcntr = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Ppcgetcntr");
	if((splitptr = strtok(NULL,",")) != NULL) Platenable = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Platenable");
	if((splitptr = strtok(NULL,",")) != NULL) Pcenable = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Pcenable");
	if((splitptr = strtok(NULL,",")) != NULL) Pfstatus = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Pfstatus");
	if((splitptr = strtok(NULL,",")) != NULL) Maincntr = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Maincntr");
	if((splitptr = strtok(NULL,",")) != NULL) Safetycntr = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Safetycntr");
	if((splitptr = strtok(NULL,",")) != NULL) Version = atoi(splitptr);
	else ROS_DEBUG_NAMED("xbeethread","failed to extract: %s","Version");

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
