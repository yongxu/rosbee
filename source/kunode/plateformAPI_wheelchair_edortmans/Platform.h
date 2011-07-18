/*********************************************************************
	Rhapsody	: 7.5.1 
	Login		: Eric
	Component	: TSRTest 
	Configuration 	: NoOXFConfig
	Model Element	: Platform
//!	Generated Date	: Fri, 18, Jun 2010  
	File Path	: TSRTest/NoOXFConfig/Platform.h
*********************************************************************/

#ifndef Platform_H
#define Platform_H

//## auto_generated
#include "PlatformAPI.h"
//## auto_generated
#include <string>
//## auto_generated
#include "rlserial.h"
//## package PlatformAPI

//## class Platform

using namespace std;

// This singeleton class represents the physical robot platform in the logical world.
// 
// 
class Platform {
    ////    Constructors and destructors    ////
    
    ////    Operations    ////
    
public :

    // Commit parameter setpoints by sending them to the physical robot platform.
    //## operation commit()
    int commit();
    
    // Query the physical robot platform for actual parameter values.
    // 
    //## operation query()
    int query();
    
    //## operation setOmega(int)
    void setOmega(int velocity);
    
    ////    Additional operations    ////
    
    ////    Attributes    ////

protected :

    // Actual value of angular velocity of left wheel as received by last query.
    int actualOmegaLeft;		//## attribute actualOmegaLeft
    
    // Actual value of angular velocity of right wheel as received by last query.
    int actualOmegaRight;		//## attribute actualOmegaRight
    
    // Angular velocity setpoint for left wheel [-100..100]
    int omegaLeft;		//## attribute omegaLeft
    
    // Angular velocity setpoint for right wheel [-100..100]
    int omegaRight;		//## attribute omegaRight
    
    ////    Relations and components    ////

private :

    //## operation Platform()
    Platform();

public :

    //## operation ~Platform()
    ~Platform();
    
    // Get singleton instance.
    //## operation getInstance()
    static Platform* getInstance();

private :

    //## operation parse(string&)
    int parse(string& msgstr);

public :

    //## auto_generated
    int getAcceleration() const;
    
    //## auto_generated
    void setAcceleration(int p_acceleration);
    
    //## auto_generated
    int getActualAcceleration() const;
    
    //## auto_generated
    int getActualMotorcurrent() const;
    
    //## auto_generated
    int getActualOmegaLeft() const;
    
    //## auto_generated
    int getActualOmegaRight() const;
    
    //## auto_generated
    int getActualStatus() const;
    
    //## auto_generated
    bool getConnected() const;
    
    //## auto_generated
    int getOmegaLeft() const;
    
    //## auto_generated
    void setOmegaLeft(int p_omegaLeft);
    
    //## auto_generated
    int getOmegaRight() const;
    
    //## auto_generated
    void setOmegaRight(int p_omegaRight);

protected :

    const int BAUDRATE;		//## attribute BAUDRATE
    
    const char* SERIAL_DEVICE;		//## attribute SERIAL_DEVICE
    
    const int TIMEOUT;		//## attribute TIMEOUT
    
    const int TRACING;		//## attribute TRACING
    
    // Acceleration setpoint for both wheels [0..100]
    int acceleration;		//## attribute acceleration
    
    // Actual value of acceleration as received by last query.
    int actualAcceleration;		//## attribute actualAcceleration
    
    // Actual value of motor current as received by last query.
    int actualMotorcurrent;		//## attribute actualMotorcurrent
    
    // Status as received by last query.
    int actualStatus;		//## attribute actualStatus
    
    bool connected;		//## attribute connected
    
    static Platform* itsInstance;		//## attribute itsInstance

private :

    //## operation getItsConnection()
    rlSerial* getItsConnection();

protected :

    rlSerial itsConnection;		//## attribute itsConnection
};

#endif
/*********************************************************************
	File Path	: TSRTest/NoOXFConfig/Platform.h
*********************************************************************/
