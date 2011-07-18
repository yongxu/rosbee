/********************************************************************
	Rhapsody	: 7.5.1 
	Login		: Eric
	Component	: TSRTest 
	Configuration 	: NoOXFConfig
	Model Element	: Platform
//!	Generated Date	: Fri, 18, Jun 2010  
	File Path	: TSRTest/NoOXFConfig/Platform.cpp
*********************************************************************/

//## auto_generated
#include "Platform.h"
//## auto_generated
#include <iostream>
//## auto_generated
#include <string>
//## auto_generated
#include <sstream>
//## auto_generated
#include <rlserial.h>
//## dependency std
using namespace std;

//## package PlatformAPI

//## class Platform

using namespace std;

int Platform::commit() {
    //#[ operation commit()
    
    int ret;
    unsigned char line[80]; 
    
    if(!connected) return -1;
    
    //
    // Send command
    //
    
    stringstream command_ss;
    command_ss << showpos
    		<< "C" 
    		<< omegaLeft << "#"
    		<< omegaRight << "#"
    		<< acceleration << "#"
    		<< endl;
    string command_str = command_ss.str();  
    if (TRACING) cout << "Command=" << command_str;
    
    const char* command_cstr = command_str.c_str(); 
    strcpy ((char *)line, command_cstr);
    ret = getItsConnection()->writeBlock(line,strlen((char *)line));
    
    //
    // Process acknowledgement
    //
    
    ret = getItsConnection()->select(TIMEOUT);
    if ( ret == 0) return -1; 
    ret = getItsConnection()->readLine(line,sizeof(line)-1,TIMEOUT);
    if(ret == -1) return -1;
    
    string acknak_str((char *)line);
    //cout << "AckNak=" << acknak_str << endl;
    stringstream token;
    
    // parse start of message
    if (acknak_str.at(0)== 'A' ) { // Ack?
    	return 0;
    }
    if (acknak_str.at(0)== 'N' ) { // Nak?
    	acknak_str.erase(0,1); // erase first character
    	// parse first parameter
    	return ( parse(acknak_str) );
    }
    cout << "Error in msg" << endl;
    return -1;
    //#]
}

int Platform::query() {
    //#[ operation query()
    int ret;
    unsigned char line[80];
    
    if(!connected) return -1;
    
    //
    // Send query
    //
    
    stringstream query_ss;
    query_ss << "Q" 
    		<< endl;
    
    string query_str = query_ss.str();  
    if (TRACING) cout << "Query=" << query_str;
    
    const char* query_cstr = query_str.c_str(); 
    strcpy ((char *)line, query_cstr);
    ret = getItsConnection()->writeBlock(line,strlen((char *)line));
    
    //
    // Process the respons
    //
    
    ret = getItsConnection()->select(TIMEOUT);
    if ( ret == 0) return -1; 
    ret = getItsConnection()->readLine(line,sizeof(line)-1,TIMEOUT);
    if(ret == -1) return -1;
    
    string respons_str((char *)line);
    if (TRACING) cout << "Respons=" << respons_str << endl;
    
    stringstream token;
    
    // parse start of message
    if (respons_str.at(0)!= 'R') { // good start of message?
    	cout << "Error in msg" << endl;
    	return -1;
    }
    respons_str.erase(0,1); // erase first character
    
    // parse first parameter
    actualOmegaLeft = parse(respons_str);
    
    // parse second parameter
    actualOmegaRight = parse(respons_str);
    
    // parse third parameter
    actualAcceleration = parse(respons_str);
    
    // parse fourth parameter
    actualMotorcurrent = parse(respons_str);
    
    // parse fifth parameter
    actualStatus = parse(respons_str);
    
    return 0;
    //#]
}

void Platform::setOmega(int velocity) {
    //#[ operation setOmega(int)
    omegaRight = velocity;
    omegaLeft = velocity;
    //#]
}

Platform* Platform::itsInstance(NULL);

Platform::Platform() : BAUDRATE(B57600), SERIAL_DEVICE("/dev/ttyUSB0"), TIMEOUT(-1), TRACING(0), acceleration(100), actualAcceleration(-1), actualMotorcurrent(-1), actualOmegaLeft(-1), actualOmegaRight(-1), actualStatus(-1), connected(false), omegaLeft(0), omegaRight(0) {
    //#[ operation Platform()
    // Try to open USB Serial connection to low level controller
    
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

int Platform::getAcceleration() const {
    return acceleration;
}

void Platform::setAcceleration(int p_acceleration) {
    acceleration = p_acceleration;
}

int Platform::getActualAcceleration() const {
    return actualAcceleration;
}

int Platform::getActualMotorcurrent() const {
    return actualMotorcurrent;
}

int Platform::getActualOmegaLeft() const {
    return actualOmegaLeft;
}

int Platform::getActualOmegaRight() const {
    return actualOmegaRight;
}

int Platform::getActualStatus() const {
    return actualStatus;
}

bool Platform::getConnected() const {
    return connected;
}

int Platform::getOmegaLeft() const {
    return omegaLeft;
}

void Platform::setOmegaLeft(int p_omegaLeft) {
    omegaLeft = p_omegaLeft;
}

int Platform::getOmegaRight() const {
    return omegaRight;
}

void Platform::setOmegaRight(int p_omegaRight) {
    omegaRight = p_omegaRight;
}

rlSerial* Platform::getItsConnection() {
    //#[ operation getItsConnection()
    return (rlSerial*) &itsConnection;
    //#]
}

/*********************************************************************
	File Path	: TSRTest/NoOXFConfig/Platform.cpp
*********************************************************************/
