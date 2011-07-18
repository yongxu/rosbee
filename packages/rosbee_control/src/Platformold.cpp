/*********************************************************************
        Author 		: Michiel van Osch
	Component	: Keila Unicylce Platform Interface
//!	Date		: Mo, 11, Jul 2011  
        Description     : This class implements an API for 
                          communicating with Henk Kiela's Unicycle
                          This is not finished!!
                          This class sends string commands 
                          to the propellor on the platform and receives
                          them back. This API formats them to the right
                          type for the controller ROS node. e.g. 
                          ultrasound sensor information is formatted 
                          to an array of integers.

                          Warning, propellor commands e.g. $908, 
                          may not be correct!
*********************************************************************/

#include "rosbee_control/Platform.h"
#include <string>
#include <iostream>
#include <sstream>

#define DEVICE "/dev/ttyUSB0"



//## class Platform

using namespace std;

// commit sends a control string to the platform
// proper processing of return values is still to be implemented

int Platform::commit(string cmd) {

    int ret;
    unsigned char line[80]; 
    stringstream ss;
    ss << "Pc" << cmd << ",#" << endl;
    cout << ss.str();
    if(!connected) return -1;
    
    //
    // Send command
    //
    
    if (TRACING) cout << "Command=" << ss.str();
    
    const char* command_cstr = ss.str().c_str(); 
    strcpy ((char *)line, command_cstr);
    cout << line;
    ret = getItsConnection()->writeBlock(line,strlen((char *)line));
    
    
    ret = getItsConnection()->select(TIMEOUT);
    if ( ret == 0) return -1; 
    ret = getItsConnection()->readLine(line,sizeof(line)-1,TIMEOUT);
    if(ret == -1) return -1;

    string respons_str((char *)line);
    if (TRACING) cout << "Respons=" << respons_str << endl;
    parse(respons_str);

    
    return -1;
    //#]
}

// query sends a feedback request (e.g. to get sensorvalues)
// to the propellor and receives back the response

int Platform::query(int id) {
    //#[ operation query()
    int ret;
    unsigned char line[80];
    
    if(!connected) return -1;
    
    //
    // Send query
    //
    
    stringstream query_ss;

    query_ss << "Pc$" << id << ",#" 
    		<< endl;
    
    string query_str = query_ss.str();  
    if (TRACING) cout << "Query=" << query_str;
    
    const char* query_cstr = query_str.c_str(); 
    strcpy ((char *)line, query_cstr);
    ret = getItsConnection()->writeBlock(line,strlen((char *)line));
    
    //
    // Process the respons
    //
    ret = getItsConnection()->readLine(line,sizeof(line)-1,TIMEOUT);
    if(ret == -1) return -1;
    
    string respons_str((char *)line);
    if (TRACING) cout << "Respons=" << respons_str << endl;
    
    
    // parse start of message
    if (id == 911){
//          actualLwheelPos := response_str;	
          cout << "wheelpos=" << respons_str << endl;
    }
    else if (id == 912){
//        usensors = 
        cout << "wheelpos=" << respons_str << endl;

    }
    else if (id == 913){
//        actualStatus := response_str;
          cout << "wheelpos=" << respons_str << endl;

    }
    
    return 0;
    //#]
}



void Platform::Move(int cnt, int lin_velocity, int rot_velocity) {
    stringstream ss; 
    ss << "$900," << cnt << "," << lin_velocity << "," << rot_velocity;
    commit(ss.str());
}

void Platform::Enable_motion(bool b) {
   stringstream ss;
   if (b)
     ss << "$901,1";
   else
     ss << "$901,0";
   commit(ss.str());

}

void Platform::Enable_control(bool b){
   stringstream ss;
   if (b)
     ss << "$902,1";
   else
     ss << "$902,0";
   commit(ss.str());
}

void Platform::Enable_us(bool b){
   stringstream ss;
   if (b)
     ss << "$909,1";
   else
     ss << "$909,0";
   commit(ss.str());
}


void Platform::Clear_errors(){
   stringstream ss;
   ss << "$908";
   commit(ss.str());
}


// get_Rwheelposition and Lwheelposition should become one command for
// both wheels
int Platform::get_Rwheelposition(){
  query(911);
  // to be impelented process response
  return 0;
}
  
int Platform::get_Lwheelposition(){
  query(911);
  // to be impelented process response
  return 0;
}

int* Platform::get_usensors(){
  int * pointer;
  int usensors [10];
  pointer = &usensors[0];
  query(912);
  // to be impelented process response
  return pointer;
}

string Platform::get_status(){
  query(912);
  // to be impelented process response
  return "hello";
}

bool Platform::getConnected() const {
    return connected;
}

Platform* Platform::itsInstance(NULL);

//in Linux, the Xbee and USB interfaces become visible as ttyUSB0 and ttyUSB1, depending on which
//you connect first. Set the right Serial Devices to communicate with the propellor

Platform::Platform() : BAUDRATE(B115200), SERIAL_DEVICE(DEVICE), TIMEOUT(-1), TRACING(1), actualLwheelPos(0), actualRwheelPos(0), usensors({-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}), actualStatus(""), connected(false) {
    //#[ operation Platform()
    // Try to open USB Serial connection to low level controller
    cout << "here" << endl;
    if(getItsConnection()->openDevice(SERIAL_DEVICE,BAUDRATE,1,0,8,1,rlSerial::NONE) < 0) {
        cout << "Could not open connection to " << SERIAL_DEVICE << endl;
        connected = false;
    } else {
    	cout << "Connection established" << endl;
    	connected = true;
    }
    
    getItsConnection()->setTrace(TRACING); // for debugging
    //#]
}

Platform::~Platform() {
    //#[ operation ~Platform()
    getItsConnection()->closeDevice();
    //#]
}

Platform* Platform::getInstance() {
    //#[ operation getInstance()
    if (!itsInstance) 
      itsInstance=new Platform();
    return itsInstance;
    
    //#]
}

int Platform::parse(string& msgstr) {
    //#[ operation parse(string&)
    int res;
    size_t pos = msgstr.find("#", 0);
    stringstream token(msgstr.substr(0, pos));
    //cout << "token=" << token.str() << endl;
    token >> res;
    //token.str(string());
    //token.clear();
    msgstr.erase(0, pos+1);
    
    return res;
    //#]
}


rlSerial* Platform::getItsConnection() {
    //#[ operation getItsConnection()
    return (rlSerial*) &itsConnection;
    //#]
}

