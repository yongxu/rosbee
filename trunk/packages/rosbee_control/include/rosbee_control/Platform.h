/*
 * platform.hpp
 *
 *  Created on: Jul 18, 2011
 *      Author: Bram van de Klundert
 */

#include <string>
#include <rlserial.h>
#include <control_commands.h>

#define BAUTRATE B115200
#define TIMEOUT -1

using namespace std;

class Platform {

public:
	~Platform();

	Platform getInstance();

	/*** platform control ***/
	void move(int8_t speed,int8_t dir);
	void Enable_motion(bool enable);
	void pc_control(bool enable);
	void clear_error();
	void set_ultrasoon(bool enable);
	//returns an array with a size of 2 which contains the encoder values
	int16_t* read_encoders();
	//returns an array with the ultrasoon distance.
	int* read_ultrasoon();
	void read_status();
	/*** end platform control ***/

	bool connect(char* device);

private:
	Platform();
	static Platform Pinstance = NULL;

	bool write_to_platform(char* message,int size);
	bool read_from_platform(char* buffer,int size);

	rlSerial serialcon;

	/*** platform settings ***/
	bool motion_enabled;
	bool pc_control;
	bool ultrasoon;
	/*** end platform settings ***/

	bool connected;

protected:


};
