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
*********************************************************************/

#include <string>
#include <rlserial.h>


using namespace std;

// This singeleton class represents the physical robot platform in the logical world.
// 
// 
class Platform {
    ////    Constructors and destructors    ////
    
    ////    Operations    ////
    
private :

    // Commit parameter setpoints by sending them to the physical robot platform.
    //## operation commit()
    int commit(string cmd);
    
    // Query the physical robot platform for actual parameter values.
    // 
    //## operation query()
    int query(int id);

public :
    
    // platform operations 

    void Move(int count, int lin_velocity, int rot_velocity);
    
    void Enable_motion(bool b);

    void Enable_control(bool b);

    void Enable_us(bool b);

    void Clear_errors();
   
    // to be changed to get both wheel positions in one

    int get_Rwheelposition();
  
    int get_Lwheelposition();

    int* get_usensors();

    string get_status();

    bool getConnected() const;





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

protected :

    const int BAUDRATE;		//## attribute BAUDRATE
    
    const char* SERIAL_DEVICE;		//## attribute SERIAL_DEVICE
    
    const int TIMEOUT;		//## attribute TIMEOUT
    
    const int TRACING;		//## attribute TRACING
    
    int actualLwheelPos;	
    
    int actualRwheelPos;	

    //actual ultrasound sensorvalues
    int  usensors [10];    

    // Status as received by last query.
    string actualStatus;		//## attribute actualStatus


    
    bool connected;		//## attribute connected

    
    static Platform* itsInstance;		//## attribute itsInstance

private :

    //## operation getItsConnection()
    rlSerial* getItsConnection();

protected :

    rlSerial itsConnection;		//## attribute itsConnection

};

