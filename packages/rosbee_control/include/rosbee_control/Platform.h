/*
 * platform.hpp
 *
 *  Created on: Jul 18, 2011
 *  Author: Bram van de Klundert
 */

#include <string>
#include <sstream>
#include <rosbee_control/rlserial.h>
#include <rosbee_control/control_commands.h>
#include <serial_port/lightweightserial.h>

#define BAUTRATE B115200
#define TIMEOUT -1
#define NR_ULTRASOON 10

using namespace std;

class Platform {

public:
	~Platform();

	static Platform* getInstance();

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
	void read_encoders(int16_t* encoders);
	//returns an array with the ultrasoon distance.
	void read_ultrasoon(int* ultrasoon);
	//reads the current status of the robot
	void read_status();
	/*** END PLATFORM CONTROL ***/

	//connects to the robot on device
	bool connect(const char* device);

private:
	Platform();
	static Platform* Pinstance;

	//writes message with size size to the platform
	bool write_to_platform(char* message,int size);
	//reads at most size chars from the platform.
	bool read_from_platform(char* buffer,int size);

	LightweightSerial *lwserialcon;

	/*** platform settings ***/
	bool motion_enabled;
	bool pc_control_enabled;
	bool ultrasoon_enable;
	int ussensor[10];
	int16_t encoders[2];
	/*** end platform settings ***/

	bool connected;

protected:


};
