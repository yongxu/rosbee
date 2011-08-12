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
#include <sensor_msgs/LaserScan.h>


#define WHEELBASE 			   0.41
#define FULLCIRCLEPULSE		   36.0
#define OMTREKWHEEL			   (2.0* M_PI*(0.153/2.0))
#define DISTANCEBASETOSCANNER  0.155,0,0.155
#define DISTANCEBASETOLASER    0.155,0,0.105
#define DISTANCEBASETORWHEEL   0,-0.205,0
#define DISTANCEBASETOLWHEEL   0,0.205,0

double prevEncR,prevEncL,encR,encL;
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

	//calculate deltas
	double delx =   pose.pose.position.x -prevpose.pose.position.x ;
	double dely  =  pose.pose.position.y-  prevpose.pose.position.y;
	double delth =  tf::getYaw(pose.pose.orientation) - tf::getYaw(prevpose.pose.orientation);
	ros::Duration delt = ros::Time::now() - prevpose.header.stamp;
	ROS_DEBUG_NAMED("Odometry","delx= %f,dely=%f,delth=%f,delt.nsec=%i,delt.sec=%i",delx,dely,delth,delt.nsec,delt.sec);

	//calculate velocity
	vx = (delx/1000000000.0) * ((float)delt.nsec);
	vy = (dely/1000000000.0) * ((float)delt.nsec);
	vth= (delth/1000000000.0) *((float)delt.nsec);

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;


	ROS_DEBUG_NAMED("Odometry","X-velocity= %f Y-velocity= %f TH-velocity= %f",vx,vy,vth);
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

	transform.setOrigin( tf::Vector3(DISTANCEBASETOLASER));
	transform.setRotation(tf::createQuaternionFromRPY(0,0,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "laser"));

	transform.setOrigin( tf::Vector3(DISTANCEBASETOLWHEEL));
	transform.setRotation(tf::createQuaternionFromRPY(0,encL,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "leftWheel"));

	transform.setOrigin( tf::Vector3(DISTANCEBASETORWHEEL));
	transform.setRotation(tf::createQuaternionFromRPY(0,encR,0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"base_link", "rightWheel"));

	ROS_DEBUG_NAMED("TF","TF Sended");
}


geometry_msgs::PoseStamped calculatePlatformPose(double l, double r) {

	double Lx = -WHEELBASE/2.0;
	double Ly = 0.0;
	double Rx = WHEELBASE/2.0;
	double Ry = 0.0;
	double theta =0;

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
	}
	else
	{
		//temporary solution. prevention for dividing 0.
		if(r == 0)
		{
			r= 0.00000001;
		}
		if(l == 0)
		{
			l= 0.00000001;
		}

		double rl; // radius of curvature for left wheel
		rl = WHEELBASE * l / (r - l);

		ROS_DEBUG_NAMED("Odometry","Radius of curvature (left wheel): %.2lf", rl);

		// angle we moved around the circle, in radians
		theta = 2.0 * M_PI * (l / (2.0 * M_PI * rl));

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

	//make posestamped message
	geometry_msgs::PoseStamped posestamp;
	posestamp.header.frame_id ="base_link";
	posestamp.header.stamp = ros::Time(0);
	posestamp.pose.position.y = ((Lx+Rx)/2.0);
	posestamp.pose.position.x = ((Ly+Ry)/2.0);
	posestamp.pose.position.z = 0;
	posestamp.pose.orientation = tf::createQuaternionMsgFromYaw(theta); 

	ROS_DEBUG_NAMED("Odometry"," new Platform pose generated! pose= x:%f y:%f z:%f orientation= x:%f y:%f z:%f w:%f",posestamp.pose.position.x,
			posestamp.pose.position.y,posestamp.pose.position.z,posestamp.pose.orientation.x,posestamp.pose.orientation.y,
			posestamp.pose.orientation.z,posestamp.pose.orientation.w);

	return posestamp;
}

geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped pose){

	geometry_msgs::PoseStamped odom_point;
	odom_point.pose = geometry_msgs::Pose();
	odom_point.pose.orientation =tf::createQuaternionMsgFromYaw(0);

	try{
		listener->transformPose("odom", pose, odom_point);
		ROS_DEBUG_NAMED("TF","new pos for base  x=%f y=%f",odom_point.pose.position.z, odom_point.pose.position.y);

	}
	catch(tf::TransformException& ex){
		ROS_ERROR_NAMED("TF","Received an exception trying to transform a point: %s", ex.what());
	}
	return odom_point;
}

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
	//set the prevEnc values only the first time, because there is no initial value when
	//entering this method for the first time. assuming the values will never be 0.
	if(prevEncL == 0 && prevEncR == 0)
	{
		prevEncR = msg->rightEncoder;
		prevEncL =  msg->leftEncoder;
	}

	//get the difference between last and current position
	double delr = msg->rightEncoder - prevEncR;
	double dell = msg->leftEncoder -  prevEncL;
	ROS_DEBUG_NAMED("Odometry","delta r:%f pulses,delta l:%f pulses",delr,dell);

	//calculate the distance the wheels have traveled, compared to last measurement.
	double right = (OMTREKWHEEL*delr)/FULLCIRCLEPULSE;
	double left = (OMTREKWHEEL*dell)/FULLCIRCLEPULSE;
	ROS_DEBUG_NAMED("Odometry","New distance: left wheel:%lfm right wheel:%lfm",left,right);

	//calculate the wheelangle from both encoders for tf in radian.(can be optimized).
	encR += ((360.0*delr)/FULLCIRCLEPULSE) *(M_PI/180.0);
	encL += ((360.0*dell)/FULLCIRCLEPULSE) *(M_PI/180.0);
	ROS_DEBUG_NAMED("Odometry","New Wheel Angle:R=%f L=%f (degrees)",(encR *(180/M_PI)),(encL*(180/M_PI)));

	//get new pose platform.
	geometry_msgs::PoseStamped pose = calculatePlatformPose(left,right);

	// translate the platform pose to a pose in the odom frame
	pose = transformPose(pose);

	//publish Transforms
	publishTf(encL,encR,pose);

	//publish Odom message
	publishOdomMessage(pose);

	//save prevpose
	prevpose = pose;

	//save the encoder values for next call
	prevEncR = msg->rightEncoder;
	prevEncL = msg->leftEncoder;
}


int main(int argc, char **argv) {

	//ros initialisation
	ros::init(argc, argv, "tfBroadcaster");
	ros::NodeHandle n;

	//init variables
	encR=encL=prevEncL=prevEncR=0;

	//subscribe to encoders. every time an encoder value is published over ROS the enc method is called.
	//we need this values to calculate the position/speed of the platform.
	ros::Subscriber subx = n.subscribe("/enc",10,enc);

	//ros Publisher.will be used later when odomerty information is calculated.
	//you need to initiaze this publisher in the main or else it wont work.
	odom_pub = new ros::Publisher();
	*odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	//transform listener. will be used later
	listener = new tf::TransformListener(n);

	//run node.
	ros::spin();

	//delete pointers
	delete listener;
	delete odom_pub;

	return 0;
}
