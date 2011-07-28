#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <rosbee_control/encoders.h>
#include <ros/console.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <tf/transform_listener.h>


#define WHEELBASE 			   0.41
#define FULLCIRCLEPULSE		   36.0
#define OMTREKWHEEL			   (2.0* M_PI*(0.153/2.0))
#define DISTANCEBASETOSCANNER  0.155,0,0.135
#define DISTANCEBASETOLWHEEL  	0,-0.205,0
#define DISTANCEBASETORWHEEL   0,0.205,0

double prevEncR,prevEncL = 0;
double encR;
double encL;
geometry_msgs::PoseStamped prevpose;

tf::TransformListener * listener;
ros::Publisher * odom_pub;



void publishOdomMessage(geometry_msgs::PoseStamped pose){

	double vx,vy,vth;
	vx=vy=vth=0;

	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
	odom.pose.pose = pose.pose;

	//set the position
	double delx =   pose.pose.position.x -prevpose.pose.position.x ;
	double dely  =  pose.pose.position.y-  prevpose.pose.position.y;
	double delth =  tf::getYaw(pose.pose.orientation) - tf::getYaw(prevpose.pose.orientation);


	ros::Duration delt;
	delt = ros::Time::now() - prevpose.header.stamp;
	ROS_DEBUG_NAMED("Odometry","delx= %f,dely=%f,delth=%f,delt.nsec=%i,delt.sec=%i",delx,dely,delth,delt.nsec,delt.sec);

	vx = (delx/1000000000.0) * ((float)delt.nsec);
	vy = (dely/1000000000.0) * ((float)delt.nsec);
	vth= (delth/1000000000.0) *((float)delt.nsec);

	//set the velocity
	odom.child_frame_id = "base_link";

	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;


	ROS_DEBUG_NAMED("Odometry","X-velocity= %f Y-velocity= %f TH-velocity= %f",vx,vy,vth);
	//ROS_DEBUG_NAMED("Odometry","odom.pose.pose.orientation x=%f y=%f z=%f w=%f", odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	//publish the message
	odom_pub->publish(odom);
}


void publishTf(const double encL, const double encR, const geometry_msgs::PoseStamped base_pose)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(base_pose.pose.position.x,base_pose.pose.position.y,base_pose.pose.position.z));
	transform.setRotation(tf::Quaternion(base_pose.pose.orientation.x ,base_pose.pose.orientation.y,base_pose.pose.orientation.z,base_pose.pose.orientation.w));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"odom", "base_link"));

	transform.setOrigin( tf::Vector3(DISTANCEBASETOSCANNER));
	transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "openni_camera"));

	transform.setOrigin( tf::Vector3(DISTANCEBASETOLWHEEL));
	transform.setRotation(tf::createQuaternionFromRPY(0,encL,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "leftWheel"));

	transform.setOrigin( tf::Vector3(DISTANCEBASETORWHEEL));
	transform.setRotation(tf::createQuaternionFromRPY(0,encR,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "rightWheel"));

	publishOdomMessage(base_pose);
	prevpose = base_pose;
	ROS_DEBUG_NAMED("TF","TF Sended");
}


geometry_msgs::Pose update_wheel_position(double l, double r) {

	double Lx = -WHEELBASE/2.0;
	double Ly = 0.0;
	double Rx = WHEELBASE/2.0;
	double Ry = 0.0;
	double theta =0;

	geometry_msgs::Pose pose;

	if (l == r) {
		// If both wheels moved about the same distance, then we get an infinite
		// radius of curvature.  This handles that case.

		// find forward by rotating the axle between the wheels 90 degrees
		double axlex = Rx - Lx;
		double axley = Ry - Ly;

		double forwardx, forwardy;
		forwardx = -axley;
		forwardy = axlex;

		// normalize
		double length = sqrt(forwardx*forwardx + forwardy*forwardy);
		forwardx = forwardx / length;
		forwardy = forwardy / length;

		// move each wheel forward by the amount it moved
		Lx = Lx + forwardx * l;
		Ly = Ly + forwardy * l;

		Rx = Rx + forwardx * r;
		Ry = Ry + forwardy * r;
       /*
		Lx=l;
		Rx=r;
		Ly=Ry=0; */

	}
	else
	{
		double rl; // radius of curvature for left wheel
		rl = WHEELBASE * l / (r - l);

		ROS_DEBUG_NAMED("Odometry","Radius of curvature (left wheel): %.2lf", rl);

		// angle we moved around the circle, in radians
		 theta = 2.0 * M_PI * (l / (2.0 * M_PI * rl));
		//theta = l / rl;

		ROS_DEBUG_NAMED("Odometry","Theta: %.2lf radians", theta);

		// Find the point P that we're circling
		double Px, Py;

		Px = Lx + rl*((Lx-Rx)/WHEELBASE);
		Py = Ly + rl*((Ly-Ry)/WHEELBASE);

		ROS_DEBUG_NAMED("Odometry","Center of rotation: (%.2lf, %.2lf)", Px, Py);

		// Translate everything to the origin
		double Lx_translated = Lx - Px;
		double Ly_translated = Ly - Py;

		double Rx_translated = Rx - Px;
		double Ry_translated = Ry - Py;

		ROS_DEBUG_NAMED("Odometry","Translated: (%.2lf,%.2lf) (%.2lf,%.2lf)",
				Lx_translated, Ly_translated,
				Rx_translated, Ry_translated);

		// Rotate by theta
		double cos_theta = cos(theta);
		double sin_theta = sin(theta);

		ROS_DEBUG_NAMED("Odometry","cos(theta)=%.2lf sin(theta)=%.2lf", cos_theta, sin_theta);

		double Lx_rotated = Lx_translated*cos_theta - Ly_translated*sin_theta;
		double Ly_rotated = Lx_translated*sin_theta + Ly_translated*sin_theta;

		double Rx_rotated = Rx_translated*cos_theta - Ry_translated*sin_theta;
		double Ry_rotated = Rx_translated*sin_theta + Ry_translated*sin_theta;

		ROS_DEBUG_NAMED("Odometry","Rotated: (%.2lf,%.2lf) (%.2lf,%.2lf)",
				Lx_rotated, Ly_rotated,
				Rx_rotated, Ry_rotated);

		// Translate back
		Lx = Lx_rotated + Px;
		Ly = Ly_rotated + Py;

		Rx = Rx_rotated + Px;
		Ry = Ry_rotated + Py;
	}

	pose.position.y = ((Lx+Rx)/2.0);
	pose.position.x = ((Ly+Ry)/2.0);
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(-theta); //volgens mij niet helemaal de bedoeling,negatief maken

	ROS_DEBUG_NAMED("Odometry"," new pose generated::pose= x:%f y:%f z:%f orientation= x:%f y:%f z:%f w:%f",pose.position.x,
			pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,
			pose.orientation.z,pose.orientation.w);

	return pose;
}
geometry_msgs::PoseStamped transformPose(double leftwheel,double rightwheel){

	geometry_msgs::PoseStamped base_point;
	geometry_msgs::PoseStamped odom_point;

	odom_point.pose = geometry_msgs::Pose();
	odom_point.pose.orientation =tf::createQuaternionMsgFromYaw(0);


	base_point.header.frame_id = "base_link";
	//	base_point.header.stamp = ros::Time::now();
	base_point.pose = update_wheel_position(leftwheel,rightwheel);

	try{

		listener->transformPose("odom", base_point, odom_point);
		ROS_DEBUG_NAMED("TF","new pos for base  x=%f y=%f",odom_point.pose.position.z, odom_point.pose.position.y);

	}
	catch(tf::TransformException& ex){
		ROS_ERROR_NAMED("TF","Received an exception trying to transform a point: %s", ex.what());
	}
	return odom_point;
}
bool b=false;
void pubOnce()
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0));
	transform.setRotation(tf::createQuaternionFromYaw(90.0 * (M_PI/180.0)));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"odom", "base_link"));

}

void enc(const rosbee_control::encoders::ConstPtr& msg)
{
	if(!b)
	{
		prevEncR = msg->rightEncoder;
		prevEncL =  msg->leftEncoder;
		b = true;
	}


	//get the difference between last and current position
	double delr = msg->rightEncoder - prevEncR;
	double dell = msg->leftEncoder -  prevEncL;
	ROS_DEBUG_NAMED("Odometry","delta r:%f,delta l:%f",delr,dell);

	if(delr == 0)
	{
		delr= 0.0000001;
	}
	if(dell == 0)
	{
		dell= 0.0000001;
	}

	//calulate the distance compared to last mesurement
	double right = (OMTREKWHEEL*delr)/FULLCIRCLEPULSE;
	double left = (OMTREKWHEEL*dell)/FULLCIRCLEPULSE;
	ROS_DEBUG_NAMED("Odometry","New distance: left wheel:%lf right wheel%lf",left,right);

	//calculate the angle from both encoders for tf
	encR += ((360.0*delr)/FULLCIRCLEPULSE) *(M_PI/180.0);
	encL += ((360.0*dell)/FULLCIRCLEPULSE) *(M_PI/180.0);
	ROS_DEBUG_NAMED("Odometry","New Wheel Angle:R= %f L=%f",(encR *(180/M_PI)),(encL*(180/M_PI)));

	//publish TF
	publishTf(encL,encR,transformPose(left,right));


	//save the encoder values for next call
	prevEncR = msg->rightEncoder;
	prevEncL = msg->leftEncoder;

}


int main(int argc, char **argv) {

	//ros init
	ros::init(argc, argv, "tfOdomBroadcaster");
	ros::NodeHandle n;
	//subscribe to encoders
	ros::Subscriber subx = n.subscribe("/enc",10,enc);
	odom_pub = new ros::Publisher();
	*odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	encR=encL=0;
	pubOnce();

	listener = new tf::TransformListener(n);
	ros::spin();

	delete listener;
	delete odom_pub;
	return 0;
}
