Readme for the Keila Unicycle API
Author: Michiel van Osch
Date: 15-7-2011

Some notes on my experiences.

- In order to program the parallax and see the paralax terminal you only need bst (bst.linux).

- A bug in the paralax terminal is that upon receiving data it does not refresh the terminal so the paralax status becomes unreadable, unless you disconnect.

- With the API in the current state I was able to communicate with the unicycle both with Xbee and USB cable. I was able to read sensorvalues, I was able
to set the motion or the PCcontrol enabled (but not both because it gets stuck) and therefore I was not able to move the wheels of the unicycle yet.

- Reading the sensor values disabled the motion or PcControl enabled.

- The PlatformAPI is based on the wheelchair platform API by Eric Dortmans. However it is simplefied (just a class not a pacakge) and the possible commands
have been/have to be adapted to suit the Kiela Platform

- This API can be used insde a ROS node. As an example I have included the sources of the ROS node made for the wheelchair platform, by Debyjoti Bera. However
this code is a big mess.

- An example joystick node source is also included

Good Luck Boys!
