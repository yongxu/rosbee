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
	Noodstop = false;
	connected = false;
	pub = n.advertise<rosbee_control::encoders>("enc", 1);
	lwserialcon = NULL;
	//bzero(readbuffer,NRMSGS*MSGLENGHT);
	//writewindex = 0;
}

Platform::~Platform()
{
	if(connected)
	{
		connected = false;
		lwserialcon->~LightweightSerial();
		/*breadthread->join();
		bxbeethread->join();
		bwritethread->join();*/
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

	/*ROS_DEBUG_NAMED("readthread","starting read thread.");
	breadthread = new boost::thread(&Platform::readloop);
	ROS_DEBUG_NAMED("xbeethread","starting xbee thread.");
	bxbeethread = new boost::thread(&Platform::handlexbee);
	ROS_DEBUG_NAMED("writethread","starting write thread.");
	bwritethread = new boost::thread(&Platform::writeloop);*/
	//sleep a short while to make sure the thread had time to start.
	//sleep(1);
	//if the connection was opened successfully get the status of the platform
	//read_status();
	return connected;
}

void Platform::setNoodstop(bool stop)
{
        Noodstop = stop;
}


/*** platform control ***/
void Platform::move(float speed,float dir)
{
	//no need to move if the motion on the platform is disabled
	if(!connected || Noodstop) return;

	int l = 0, r = 0;

	convertor(speed,dir,&l,&r);

	ROS_WARN_NAMED("platform","move(%f,%f)",speed,dir);
	stringstream ss;
	ss << "$" << l << ";" << r << "#";
	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	ROS_INFO("writestring: %s",writestring);
	if(!lwserialcon->write_block(writestring,ss.str().length()))
	{
		ROS_INFO("error writing");
		return;
	}

	char readbuffer[16];
	bzero(readbuffer,16);

	read_from_platform(readbuffer,15);
	ROS_INFO("donereading %s",readbuffer);
	handle_encoder(readbuffer);
}

void Platform::Enable_motion(bool enable)
{
//throw string("not implemented in this version");
/*	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	motion_enabled = enable;
	ss << ENABLE_MOTION<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}

void Platform::pc_control(bool enable)
{
//throw string("not implemented in this version");

/*	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","enable_motion(%s)",(enable)?"true":"false");
	pc_control_enabled = enable;
	ss << ENABLE_PC<< ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}

void Platform::clear_error()
{
//throw string("not implemented in this version");

/*	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","clear_erros()");
	ss << CLEAR_ERROR << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}

void Platform::set_ultrasoon(bool enable)
{
//throw string("not implemented in this version");
/*	if(!connected) return;
	stringstream ss;

	ROS_DEBUG_NAMED("platform","set_ultrasoon(%s)",(enable)?"true":"false");
	ultrasoon_enable = enable;
	ss << TOGGLE_US << ",";
	if(enable) ss << 1 << ',';
	else ss << 0 << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}

void Platform::read_encoders()
{
//throw string("not implemented in this version");
/*	if(!connected)
	{
		ROS_DEBUG_NAMED("platform","read_encoders(),not connected");
		return;
	}
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_encoders()");
	ss << READ_ENCODERS<< ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}

void Platform::read_ultrasoon()
{
//throw string("not implemented in this version");
/*	if(!connected || ultrasoon_enable) return;
	stringstream ss;
	ROS_DEBUG_NAMED("platform","read_ultrasoon()");
	ss << READ_US << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/

}

void Platform::read_status()
{
//throw string("not implemented in this version");
/*	if(!connected)return;
	stringstream ss;
	ss << READ_STATUS << ',';

	char writestring[ss.str().length()];
	strcpy(writestring,ss.str().c_str());
	addwritelist(writestring,ss.str().length());*/
}
/*** end platform control ***/


bool Platform::read_from_platform(char* buffer, int size)
{
	/*TODO figure out what to do with incomplete data*///right now: if incomplete buffer didnt change
	//check if we are connected to the platform
	bool timeout = true;
	timeval begin,now;
	gettimeofday(&begin,NULL);
	gettimeofday(&now,NULL);
	if(!connected)
	{
		ROS_DEBUG_NAMED("serial","serial not connected, when trying to read");
		return false;
	}

	//read from the platform
	int i = 1;
	char read[size];
	bzero(read,size);

	ROS_DEBUG_NAMED("serial","starting read, searching for \"$\"");
	while(read[0] != '$'  && (timeout = ((begin.tv_usec+READTIMEOUT) > ((now.tv_sec-begin.tv_sec)*1000000 + now.tv_usec))))
	{
		if(!lwserialcon->read(read))
			ROS_DEBUG_NAMED("serial_warn","reading from serial returned false, while waiting for \'$\'");
		gettimeofday(&now,NULL);
	}
	if(!timeout) 
	{
		ROS_WARN_NAMED("serial","timeout while waiting for $");
		return false;
	}

	while(read[i] != '#' && i < size-1  && (timeout = ((begin.tv_usec+READTIMEOUT) > ((now.tv_sec-begin.tv_sec)*1000000 + now.tv_usec))))
	{

		if(!lwserialcon->read(&read[i]))
		{
			ROS_DEBUG_NAMED("serial_warn","reading from serial returned false, while reading the message");
		}
		else if (read[i] != '#' && read[i] != ' ' && (uint8_t)read[i] != 0)
		{
			ROS_DEBUG_NAMED("serial","read: %d, char: %c",(uint8_t)read[i],read[i]);
			i++;
		}
		gettimeofday(&now,NULL);
	}
	ROS_DEBUG_NAMED("serial","done reading, received \"%s\"",read);
	if(!timeout) 
	{
		ROS_WARN_NAMED("serial","timeout while reading");
		return false;
	}
	strcpy(buffer,read);
	return true;
}
void Platform::handle_encoder(char* command)
{
	rosbee_control::encoders msg;
//	char* tmp;
//	int i = 0;
	ROS_DEBUG_NAMED("xbeethread","handle_encoder(\"%s\")",command);
	sscanf(command,"$%d;%d#",&encoders[1],&encoders[0]);
	ROS_DEBUG_NAMED("xbeethread","encoders 0:%i 1:%i",encoders[0],encoders[1]);
	msg.leftEncoder = encoders[1];
	msg.rightEncoder = encoders[0];
	pub.publish(msg);
	ROS_DEBUG_NAMED("xbeethread","encoder values published");
}

void Platform::convertor(float x, float rz, int* l, int* r)
{
	double rf = 0;
	double lf = 0;

	rf = x +((/*WHEELBASE*/  2.0/2.0)*rz);
	lf = x -((/*WHEELBASE*/ 2.0/2.0)*rz);

	rf = ((rf+2.41)*FACTOR)+MINVALUE;
	lf = ((lf+2.41)*FACTOR)+MINVALUE;
	rf -= CORR;
 
	if(rf < MINVALUE) rf = MINVALUE;
	else if (rf > MAXVALUE) rf = MAXVALUE;

	if(lf < MINVALUE) lf = MINVALUE;
	else if (lf > MAXVALUE) lf = MAXVALUE;


 	*r =(int)rf;
	*l =(int)lf;
}
