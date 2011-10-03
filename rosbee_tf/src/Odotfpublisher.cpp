#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <rosbee_control/encoders.h>
#include <ros/console.h>


#define DISTANCEBASETOSCANNER  0,0.2,0.2 
#define DISTANCEBASETOLWHEEL  -0.2975,0,0
#define DISTANCEBASETORWHEEL   0.2975,0,0
#define LOOPRATE	5
#define OMTREKWHEELMM				 457.2	
#define FULLCIRCLEPULSE			 36											

 int firstEncL;
 int firstEncR;
 int encR,encL;
 float r,l;

  tf::TransformBroadcaster odom_broadcaster;

	void enc(const rosbee_control::encoders::ConstPtr& msg)
	{
		if(firstEncL == 0 && firstEncR == 0)
			{
				firstEncL = msg->leftEncoder;
				firstEncR = msg->rightEncoder;
				return;
			}
				encR = ((msg->rightEncoder - firstEncR)*360)/FULLCIRCLEPULSE;
				encL = ((msg->leftEncoder - firstEncL)*360)/FULLCIRCLEPULSE;
				ROS_DEBUG_NAMED("TF","ENCL: %i ENCR: %i",encL,encR);
			
				r = ((msg->rightEncoder - firstEncR)/FULLCIRCLEPULSE)*(OMTREKWHEELMM/1000);
				l = ((msg->leftEncoder - firstEncL)/FULLCIRCLEPULSE)*(OMTREKWHEELMM/1000);
	}
	void publishTf()
	{
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		transform.setOrigin( tf::Vector3(DISTANCEBASETOSCANNER));
		transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "openni_camera"));

		transform.setOrigin( tf::Vector3(DISTANCEBASETOLWHEEL));
		transform.setRotation(tf::createQuaternionFromRPY(encL*(M_PI/180),0,0));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "leftWheel"));

		transform.setOrigin( tf::Vector3(DISTANCEBASETORWHEEL));
		transform.setRotation(tf::createQuaternionFromRPY(encR*(M_PI/180),0,0));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "rightWheel"));
		ROS_DEBUG_NAMED("TF","TF Sended");
	}

	void publishOdom()
	{
		geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = r;
   //odom_trans.transform.translation.y = l;
    odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = 0;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

	}
	int main(int argc, char **argv)
	{	

		//ros init
		printf("ffs i aint making a darn nodehandle");fflush(stdout);
		ros::init(argc, argv, "tfOdomBroadcaster");

		//ros::NodeHandle n;
		ROS_INFO("wtf!3");
		ros::Rate loop_rate(LOOPRATE);

firstEncL=firstEncR=encR=encL=0;
	
		//subscribe to encoders		
		//ros::Subscriber subx = n.subscribe("/enc",10,enc);

		while(ros::ok())
		{
			publishTf();
			ros::spinOnce();
			loop_rate.sleep();					
		}
		
		return 0;
	}

