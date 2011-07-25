/*
 * platform.hpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

#include <string>
#include <sstream>
#include <rosbee_control/control_commands.h>
#include <rosbee_control/encoders.h>
#include <serial_port/lightweightserial.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <pthread.h>


#define BAUTRATE 115200
#define TIMEOUT -1
#define NR_ULTRASOON 10
#define NR_ENCODER 2
#define MSGLENGHT 50 //max size of one message
#define NRMSGS 10 //size of the buffer in msgs

using namespace std;

class Platform {

public:
	~Platform();

	static Platform* getInstance(ros::NodeHandle nh);
	//only use this if you have called getInstance before, else returns NULL
	inline Platform* getInstance(){ return Pinstance; };

	/*** PLATFORM CONTROL ***/
	//move the platform
	void move(int8_t speed,int8_t dir);
	//enables/disables movement on the robot
	void Enable_motion(bool enable);
	//enables/disables pc controll
	void pc_control(bool enable);
	//clears the current error
	void clear_error();
	//enables/disables ultrasoon sensors
	void set_ultrasoon(bool enable);
	//returns an array with a size of 2 which contains the encoder values
	void read_encoders();
	//returns an array with the ultrasoon distance.
	void read_ultrasoon();
	//reads the current status of the robot
	void read_status();
	/*** END PLATFORM CONTROL ***/

	//connects to the robot on device
	bool connect(const char* device);

private:
	Platform(ros::NodeHandle n);
	static Platform* Pinstance;

	ros::Publisher pub;

	//writes message with size size to the platform
	bool write_to_platform(char* message,int size);
	//reads at most size chars from the platform.
	bool read_from_platform(char* buffer,int size);

	static void* readloop(void* ret);
	static void* handlexbee(void* ret);

	LightweightSerial *lwserialcon;
	pthread_t* readthread;
	pthread_t* xbeethread;

	char readbuffer[NRMSGS][MSGLENGHT];
	int writeindex;

	/*** read functions ***/
	void handle_encoder(char* command);
	void handle_status(char* command);
	void handle_ultrasoon(char* command);

	/*** end read functions ***/

	/*** platform settings ***/
	bool motion_enabled;
	bool pc_control_enabled;
	bool ultrasoon_enable;
	int ussensor[10];
	int16_t encoders[2];
	//query status vars.
	int Movemode,Lastalarm,Xbeetime,Ppcgetcntr,Platenable,Pcenable,Pfstatus,Maincntr,Safetycntr,Version;
	/*** end platform settings ***/
	bool connected;

protected:


};
