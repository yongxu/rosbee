/*
** JOYSTICK NODE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <linux/joystick.h>
#include <fcntl.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

// CMD_VEL MESSAGE OBJECTS
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Twist base_cmd;

// Publisher object for topic "/cmd_vel"
ros::Publisher cmd_vel_pub;

// Do not change this value.
bool ARM_CONTROL = false;

/////////////// ADJUST THIS VALUE TO SCALE SPEEDS..////////////////////
/// INCREASE VALUE TO DECREASE OUTPUT SPEED AND VICE VERSA ////////////
int reduction_factor = 1.0;
///////////////////////////////////////////////////////////////////////


/////////// THE MAIN //////////////////
int main(int argc, char *argv[])
{
	
	int joy_fd;  fd_set set;
	int Jx = 0, Jy = 0, Jz = 0;;
	struct timeval tv; 
	struct js_event event;
	int event_count_= 0;

  // INITIALIZE ROS NODE
	ros::init(argc, argv, "stick_controller");
	ros::NodeHandle n;

  // CREATE PUBLISHER OBJECT
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("joy/cmd_vel",10);


  // Set the rate variable for the node.
  ros::Rate r(40);

	ROS_INFO("Publish init done");	

	base_cmd.linear.x = base_cmd.angular.z = 0;
  ros::Time t_prev(ros::Time::now());

  tv.tv_sec = 1;
  tv.tv_usec = 0;

  joy_fd = open("/dev/input/js0", O_RDONLY);

  if (joy_fd != -1)
  {
     close(joy_fd);
     joy_fd = open("/dev/input/js0", O_RDONLY);
  }

	while(n.ok())
	{
					//printf("\nX = %d , Y = %d\n",Jx+Jy,Jy-Jx);

	        ros::Time t(ros::Time::now());
	        ros::Duration d(t - t_prev);

	        base_cmd.linear.x = -Jy/reduction_factor; base_cmd.angular.z = -Jx/reduction_factor;

          // THIS BOOLEAN HAS BEEN SET TO TRUE.
	        if(!ARM_CONTROL)
	        {
   	          cmd_vel_pub.publish(base_cmd);
              //ROS_INFO("CHECK (%d,%d)",-Jy, -Jx);
		          ROS_INFO("published on topic cmd_vel: <%f,%f>",base_cmd.linear.x,base_cmd.angular.z);	// uncomment to read signals sent...
	        }

		FD_ZERO(&set);
		FD_SET(joy_fd, &set);
		int select_out = select(joy_fd+1, &set, NULL, NULL, &tv);

	
		if (FD_ISSET(joy_fd, &set))
		{
		  if (read(joy_fd, &event, sizeof(struct js_event)) == -1 && errno != EAGAIN)
		    //exit(0); // Joystick is probably closed. Definitely occurs.
		  
		  printf("Read data...");
		  
		  event_count_++;

      // DETERMINE EVENT TYPE : AXIS OR BUTTON
      // WE ARE INTERESTED IN AXIS FOR NOW.
		  switch(event.type)
		  {
	
		  case JS_EVENT_BUTTON:
		  case JS_EVENT_BUTTON | JS_EVENT_INIT: printf("\nButton pressed\n");
  			ROS_INFO("Event number: %d, Event Value: %d",event.number, event.value);
		    break;

		  case JS_EVENT_AXIS:
		  case JS_EVENT_AXIS | JS_EVENT_INIT: 
	
			printf("\nstick moved along %d by : %d\n",event.number,event.value);

			if(event.number == 0)
          Jx = (event.value/300); // SCALE AS DESIRED => WILL BE ANGULAR VEL.
			else if(event.number==1) 
          Jy = (event.value/300);  // SCALE AS DESIRED => WILL BE LINEAR VEL.
 			break;
	
		  default:
		    printf("joy_node: Unknown event type.");
	    break;
		  }	
		}

    r.sleep();

    // CAN BE COMMENTED AS THIS NODE IS LONGER LISTENING FOR ANYTHING
		ros::spinOnce();
	}
	close(joy_fd);
	return 0;
}
