/********************************************************************
       program: Tester
       Author: Michiel van Osch
       Date: 11-7-2001
       Description: Tester is a test program for testing the PlatformAPI
                    for Keila's Unicycle. With this program I was able
                    to communicate with the platform and read sensor
                    values, and either set PCcontrol or motion enabled.
                    However when I set control and motion after eachother
                    the program gets stuck. Probably because you are
                    not allowed to do this (and need to query in between
                    or because some serial buffer does nog get flushed
                    properly between commands.
                    
                    If I interleave with get_Rwheelpositions() (which
                    now return ultrasoundsensorvalues I beleive
                    the Tester works properly but reading sensor values
                    disables the motion or control enabled.

                    Therefore, the platform did NOT move yet using this
                    program.
*********************************************************************/

#include "Platform.h"
#include <string>
#include <iostream>

using namespace std;

Platform* itsPlatform;

int main(  )
{
        int i = 0;
        itsPlatform = Platform::getInstance();
        itsPlatform -> get_Rwheelposition();
        sleep(0.5);
        itsPlatform -> get_Rwheelposition();
        sleep(0.5);
        itsPlatform -> Enable_motion(true);
        sleep(0.5);
 //       itsPlatform -> get_Rwheelposition();
        sleep(0.5);
        itsPlatform -> Enable_control(true);
        sleep(0.5);
        itsPlatform -> get_Rwheelposition();
        sleep(0.5);
        itsPlatform -> Move(0,50,0);
        sleep(2);
        itsPlatform -> get_Rwheelposition();



        

}


