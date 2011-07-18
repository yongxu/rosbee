
// roscpp
#include <ros/ros.h>
//rosTF
#include "tf/transform_broadcaster.h"

// Messages that I need
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//## link itsPlatform
#include "Platform.h"

using namespace std;

class Rose_base_controler
{
  private:

	//serial connection to the platform
	Platform* itsPlatform;
	//ROS node handler
	ros::NodeHandle node_;

  /*
  VELOCITY COMMANDS
  */
	  //ROS twist_msgs subscriber
	  ros::Subscriber cmd_vel_sub;
	  //buffer for desired velocities
	  double desiredLinearSpeed;
	  double desiredAngularSpeed;
		//PWM commands for the wheels
		double PWMLeft, PWMRight;

  /*
  ODOMETRY
  */
	  //time stamps used by odometry
	  ros::Time current_time, last_time;

	  //coordinates
	  double x, xref;
    double y, yref;
    double th, thref;
	  double wheelSpan; //wheel span in meters
		double IntegralErrorLeft, IntegralErrorRight;
	  //tf broadcaster
	  tf::TransformBroadcaster odom_broadcaster;
		tf::TransformBroadcaster odom_ref_broadcaster;
	  //odometry_msgs publisher
	  ros::Publisher odom_pub;

   
		public:
    
    
    /*CONSTRUCTOR*/
	  //makes a basic test and subscribes to the velocity commands + advertize odometry
    Rose_base_controler()
    {
				//initializes variables
				desiredLinearSpeed = 0.0;
	  		desiredAngularSpeed = 0.0;
				
				x = 0.0;
				y = 0.0;
				th = 0.0;
				wheelSpan = 0.57;
				
				IntegralErrorLeft = 0.0; IntegralErrorRight = 0.0;

				xref = 0.0; yref = 0.0; thref = 0.0;

				current_time = ros::Time::now();
    		last_time = ros::Time::now();

      	//connect to the parallax chip
				itsPlatform = NULL;
	      itsPlatform = Platform::getInstance();

	      if (doTest() != 0) 
	      {
	            ROS_INFO("Test failed");		
	      }
	      else
	      {
	            ROS_INFO("Test passed");
    	  }

	      //subscribe to velicty messages
	      cmd_vel_sub = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(&Rose_base_controler::cmdvelReceived, this, _1));
	      //advertise odometry messages
	      odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 50);
    }
    
    /*DESTRUCTOR*/
    ~Rose_base_controler()
    {
        //set speeds back to zero
        itsPlatform->setOmegaRight(0);
        itsPlatform->setOmegaLeft(0);
        itsPlatform->setAcceleration(0);
        itsPlatform->commit();
        //set platform back to null	
        cleanUpRelations();
    }

		/*DOTEST*/
		//runs a basic test by sending commands to engines for 1000 cycles
		int doTest() 
		{
			/*
				//#[ operation doTest()
				int result = 0;
				ROS_INFO("Test Starting");
				itsPlatform->setOmegaRight(-100);
				itsPlatform->setOmegaLeft(+100);
				itsPlatform->setAcceleration(50);
				ROS_INFO("Committing");
				result = itsPlatform->commit();
				ROS_INFO("Committed");
				if (result != 0) return -1;
				
				int i = 0;
				while (i<10) {
					ROS_INFO("Sending frame %d of 1000",i);
					result =itsPlatform->query();
					cout << "querry received" << endl;
					if (result != 0) return -1;
					i++;
    }
    */
    return 0;
    //#]
		}

    /*STARTCONTROLLER*/
    //start the controller loop, compute and send velocity commands
	  //give back odometry and tfs
    int startController()
    {
	
	    //check we are connected to the platform
	    if(itsPlatform->getConnected())
	    {
		    ROS_INFO("Parallax seems connected, let's go!");	


				current_time = ros::Time::now();
				last_time = ros::Time::now();

				PWMLeft = 0;
				PWMRight = 0;

				//double KP= 0.03 * 1000.0;
				//double KV= 0.55 * 10.0;

				double KP = 35.0; // 35
				double KV = 7.0;  // 7.0
				double KI = 1.15;  // 1.15
				double KI_left = KI; double KI_right = KI;

				double errorLeft, previous_errorLeft;
				double errorRight, previous_errorRight;
		
				ros::Rate r(40.0);
				

				// Clear out encoders to avoid jump start
				for(int i = 0 ; i < 50; i++)
				{
					int oa = itsPlatform->getActualOmegaLeft();
					int ob =  itsPlatform->getActualOmegaRight();
					itsPlatform->setOmegaRight(0);
    			itsPlatform->setOmegaLeft(0);
    			
					ROS_INFO("Cleared Encoder..");
				}

				// Control Loop
		    while(node_.ok())
		    {
					current_time = ros::Time::now();
					itsPlatform->query();
					
					ros::Duration dt(current_time-last_time);
					int a = itsPlatform->getActualOmegaLeft();
					int b =  itsPlatform->getActualOmegaRight();
					ROS_INFO("\n\nINSIDE MAIN LOOP   (freq = %f)  %d   %d ", 1/dt.toSec(),b ,a );
/*

WARNING GETACTUALOMEGA LEFT AND RIGHT ARE REVERSED

*/
			
					//get actual speed from both wheels in meters/sec
					double ActualSpeedLeft = -((itsPlatform->getActualOmegaRight()/2000.0) * (2*3.14*0.16))/(dt.toSec());
					double ActualSpeedRight = -((itsPlatform->getActualOmegaLeft()/2000.0) * (2*3.14*0.16))/(dt.toSec());

					ROS_INFO("current observed speeds are: Left=%f m/s, Right=%f m/s", ActualSpeedLeft, ActualSpeedRight);


			    //compute individual desired engine speeds from twist commands
			    //TODO seriously (this is a quickly calculated stuff)
					
			    //double DesiredSpeedRight= (desiredLinearSpeed/*1000.0*/) + (wheelSpan*desiredAngularSpeed/*1000.0*/)/2.0;
			    //double DesiredSpeedLeft= (desiredLinearSpeed*2.0/*1000.0*/) - DesiredSpeedRight;

					double DesiredSpeedRight;
					double DesiredSpeedLeft;
					double radOfCurve = 0.0;

					if(desiredAngularSpeed == 0.0)
					{
						radOfCurve = 0.0;
						DesiredSpeedRight = desiredLinearSpeed;					
						DesiredSpeedLeft = desiredLinearSpeed;
					}
					else
					{
						radOfCurve = desiredLinearSpeed / desiredAngularSpeed;
						DesiredSpeedRight = desiredAngularSpeed * (radOfCurve + (0.57/2.0));					
						DesiredSpeedLeft = desiredAngularSpeed * (radOfCurve - (0.57/2.0));
					}
			
					publishOdoNew(-b, -a, DesiredSpeedLeft, DesiredSpeedRight);

					ROS_INFO("current desired speeds are: Left=%f m/s, Right=%f m/s", DesiredSpeedLeft, DesiredSpeedRight);

					//PD controller process
					errorLeft = DesiredSpeedLeft - ActualSpeedLeft;
					errorRight = DesiredSpeedRight - ActualSpeedRight;

					IntegralErrorLeft = IntegralErrorLeft + errorLeft;
					IntegralErrorRight = IntegralErrorRight + errorRight;					

					double integral_high = 7.0; double integral_low = 0.15;

					//if(ActualSpeedLeft < 0.1 && ActualSpeedLeft > -0.1 ) KI_left = 0.0; else KI_left = KI;
					//if(ActualSpeedRight < 0.1 && ActualSpeedRight > -0.1 ) KI_right = 0.0; else KI_right = KI;

					
					// Avoid Integral windup
					if(IntegralErrorLeft > integral_high)	IntegralErrorLeft = integral_high;
					if(IntegralErrorLeft < integral_low && IntegralErrorLeft > 0.0) IntegralErrorLeft = 0.0;

					if(IntegralErrorRight > integral_high)	IntegralErrorRight = integral_high;
					if(IntegralErrorRight < integral_low && IntegralErrorRight > 0.0) IntegralErrorRight = 0.0;

					if(IntegralErrorLeft < (-1*integral_high))	IntegralErrorLeft = -integral_high;
					if(IntegralErrorLeft > (-1*integral_low) && IntegralErrorLeft < 0.0) IntegralErrorLeft = 0.0;

					if(IntegralErrorRight < (-1*integral_high))	IntegralErrorRight = -integral_high;
					if(IntegralErrorRight > (-1*integral_low) && IntegralErrorRight < 0.0) IntegralErrorRight = 0.0;
					

					ROS_INFO("Errors speeds are Left = %f, Right = %f, IntegralErrorLeft: %f, IntegralErrorRight: %f", errorLeft, errorRight, 					
																IntegralErrorLeft,IntegralErrorRight);

					PWMLeft += (KP*errorLeft) + (KV*(errorLeft-previous_errorLeft)) + (KI_left*IntegralErrorLeft);
					PWMRight += (KP*errorRight)  + (KV*(errorRight-previous_errorRight)) + (KI_right*IntegralErrorRight);

					previous_errorLeft = errorLeft;
					previous_errorRight = errorRight;

					ROS_INFO("PD CONTROLLER (KP=%f, KV=%f) SENDS BACK Left= %f Right= %f, %f, %f", KP, KV, PWMLeft, PWMRight, KP*errorLeft,KP*errorRight );
					

					//safety for the engine PWM
					if(PWMLeft > 400.0) PWMLeft = 400.0;
					if(PWMLeft < -400.0) PWMLeft = -400.0;
	
					if(PWMRight > 400.0) PWMRight = 400.0;
					if(PWMRight < -400.0) PWMRight = -400.0;

				/*	PWMLeft = 100;
					PWMRight = 400;
*/
			    //send desire engine speeds
          ROS_INFO("Sending speeds: Left=%d, Right=%d", (int)PWMLeft, (int) PWMRight);
			    itsPlatform->setOmegaRight(-(int) PWMRight);
    			itsPlatform->setOmegaLeft(- (int) PWMLeft);
    			itsPlatform->setAcceleration(50); //TODO => adaptive acceleration???
			    itsPlatform->commit();

			    //get engine speed values
			    //ROS_INFO("Current speeds are: left %d   --  right %d", itsPlatform->getActualOmegaLeft(), itsPlatform->getActualOmegaRight());

			    //process and publish odometry
			    //publishOdo(ActualSpeedLeft, ActualSpeedRight);

			    //receive velocity commands
					last_time = current_time;
			    ros::spinOnce();
					r.sleep();

		    }
	
		    return(0);
	    }
	    else
	    {
		    ROS_INFO("Cannot start the loop as parallax seems not connected");
	    }

	    return -1;
   }


    /*PUBLISHODO*/
    // COMPUTE ODOMETRY, SEND TF AND ODOMETRY BACK
    void publishOdo(double realSpeedLeft, double realSpeedRight)
    {

	    //current_time = ros::Time::now();
      //-------------------------------------------------------------------
      //COMPUTE ODOMETRY
        
        	    // from http://www.asl.ethz.ch/education/master/mobile_robotics/year2007/S_5a_-_Intro__Odometry__Representations.pdf

//      double dt = (current_time - last_time).toSec();
      ros::Duration dt(current_time-last_time);
      //linear distance travelled by the center
      double delta_dist = ((double)(realSpeedRight+realSpeedLeft)*dt.toSec())/(2.0/*1000.0*/); // 2.0*1000.0 because speed comes in mm/sec and ROS is in m/sec
      //angular distance travelled by the center
      double delta_th = ((double)(realSpeedRight-realSpeedLeft)*dt.toSec())/(wheelSpan/*1000.0*/); //  mm/sec => m/sec
		
      //distance travelled by the center on X axis
    	double delta_x = delta_dist * cos(th + delta_th/2.0);
      //distance travelled by the center on Y axis
    	double delta_y = delta_dist * sin(th + delta_th/2.0);
	   
      //update local coordinates
      x += delta_x;
      y += delta_y;
      th += delta_th; 


      //-------------------------------------------------------------------
      //SEND TF AND ODOMETRY FRAMES
	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_link";

	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.0;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster.sendTransform(odom_trans);

	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = current_time;
	    odom.header.frame_id = "odom";

	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.child_frame_id = "base_link";
	    odom.twist.twist.linear.x = delta_dist/dt.toSec();
	    odom.twist.twist.linear.y = 0;
	    odom.twist.twist.angular.z = delta_th/dt.toSec();

	    //publish the message
	    odom_pub.publish(odom);

	//    last_time = current_time;
    }

		
		/*CLEAN UP RELATION*/
		//clean plateform relation
		void cleanUpRelations() {
    if(itsPlatform != NULL)
        {
            itsPlatform = NULL;
        }
		}

    /*CMDVELRECEIVED*/
    // callback function for receiving velocity commands as twist messages
    // update the desired linear and angular speed accordingly
    void cmdvelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      desiredLinearSpeed = cmd_vel->linear.x; // in m/sec
	    desiredAngularSpeed = cmd_vel->angular.z; // in rad/sec
			ROS_INFO("cmd_vel MSG received");
    }

    void publishOdoNew(double L, double R, double DL, double DR)
    {

      ros::Duration dt(current_time-last_time);

      //linear distance travelled by the center
			double LDist = (L/2000.0)*(2*3.14*0.16);
			double RDist = (R/2000.0)*(2*3.14*0.16);
			double CDist = (LDist + RDist)/2.0;
			double DTheta = (RDist - LDist)/0.57;
			th = th + DTheta;
			if(th > 6.2831) th = 0.0;
			if(th < -6.2831) th = 0.0;
			x = x + (CDist * cos(th));
			y = y + (CDist) * sin(th);

      double delta_dist = CDist;
      //angular distance travelled by the center
      double delta_th = DTheta;

			// Reference Trajectory //
			LDist = DL * dt.toSec();
			RDist = DR * dt.toSec();
			CDist = (LDist + RDist) / 2.0;
			DTheta = (RDist - LDist)/0.57;
			thref = thref + DTheta;
			if(thref > 6.2831) thref = 0.0;
			if(thref < -6.2831) thref = 0.0;
			xref = xref + (CDist * cos(th));
			yref = yref + (CDist) * sin(th);
			///////////////////////////

      //-------------------------------------------------------------------
      //SEND TF AND ODOMETRY FRAMES
	    //since all odometry is 6DOF we'll need a quaternion created from yaw
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	    geometry_msgs::Quaternion odom_ref_quat = tf::createQuaternionMsgFromYaw(thref);

	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_trans;
	    odom_trans.header.stamp = current_time;
	    odom_trans.header.frame_id = "odom";
	    odom_trans.child_frame_id = "base_link";
			
	    odom_trans.transform.translation.x = x;
	    odom_trans.transform.translation.y = y;
	    odom_trans.transform.translation.z = 0.15;
	    odom_trans.transform.rotation = odom_quat;

	    //send the transform
	    odom_broadcaster.sendTransform(odom_trans);
			
			//////////////////////////// ref TF ///////////////////////////////////
	    //first, we'll publish the transform over tf
	    geometry_msgs::TransformStamped odom_ref_trans;
	    odom_ref_trans.header.stamp = current_time;
	    odom_ref_trans.header.frame_id = "odom";
	    odom_ref_trans.child_frame_id = "base_ref_link";
			
	    odom_ref_trans.transform.translation.x = xref;
	    odom_ref_trans.transform.translation.y = yref;
	    odom_ref_trans.transform.translation.z = 0.15;
	    odom_ref_trans.transform.rotation = odom_ref_quat;

	    //send the transform --------- Uncomment to see tracking of reference
	    odom_ref_broadcaster.sendTransform(odom_ref_trans);
			/////////////////////////////////////////////////////////////////////////

			ROS_INFO("ODOMETRY: <%f, %f, %f>",x, y, th);


	    //next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = current_time;
	    odom.header.frame_id = "odom";

	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;

	    //set the velocity
	    odom.child_frame_id = "base_link";
	    odom.twist.twist.linear.x = delta_dist/dt.toSec();
	    odom.twist.twist.linear.y = 0;
	    odom.twist.twist.angular.z = delta_th/dt.toSec();

	    //publish the message
	    odom_pub.publish(odom);

			// last_time = current_time;
    }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "rose_base_controller");

  Rose_base_controler controller;

  ros::NodeHandle n;

  // Start up the robot
  if(controller.startController() != 0)
    exit(-1);

  // To quote Morgan, Hooray!
  return(0);
}
