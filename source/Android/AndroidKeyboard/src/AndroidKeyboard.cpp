#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Log.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

using namespace std;


#define BUFFERLENGHT 256
#define THRESHOLD 9.0
#define ZERO 0.15


class UDPServer
{
	private:
		int sockfd,portnr;   
  	struct sockaddr_in serv_addr, cli_addr;
		int slen;

		bool openSocket()
		{   
			slen=sizeof(cli_addr);
			sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (sockfd < 0) 
				return false;
			return true;
	}

	bool bindServer()
	{
		bzero((char *) &serv_addr, sizeof(serv_addr));
   		serv_addr.sin_family = AF_INET;
   		serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
   		serv_addr.sin_port = htons(portnr);
    		if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
            	 return false;
			 return true;
	}

    
   public:
	UDPServer(int port)
	{
		portnr = port;
		ROS_INFO("Open Sock %i",port);
		openSocket();
		ROS_INFO("bind server");
		bindServer();
		ROS_INFO("Server UP");
	}

	string readLine()
	{
  		char buffer[BUFFERLENGHT];
		bzero(buffer,BUFFERLENGHT);


//		if(read(sockfd,buffer,BUFFERLENGHT-1) > 0)
			if(recvfrom(sockfd, buffer, sizeof buffer, 0, (struct sockaddr *)&cli_addr, (socklen_t *)&slen))
				return string(buffer);
		return "";
	}

 	bool send(const char* data,int size)
	{
		int sendbytes = 0;
		if((sendbytes = sendto(sockfd,data,size,0,(struct sockaddr *)&cli_addr,sizeof(cli_addr))) >=0)
		{
			ROS_INFO("bytes send: %d",sendbytes);
			return true;
		}
		
		return false;
	}

	void Close()
	{			 
	   //close(newsockfd);
	   close(sockfd);
	}
};

void  StringToValue(string s,double * x, double *y,double *z)
{
		char *pch;
		pch = strtok((char *)s.c_str(),";");
		sscanf(pch,"%lf",x);
		pch = strtok(NULL,";");
		sscanf(pch,"%lf",y);
		pch = strtok(NULL,";");
		sscanf(pch,"%lf",z);

		if(*x > THRESHOLD)
				*x = THRESHOLD;
		else if(*x < -THRESHOLD)
				*x= -THRESHOLD;

		if(*y > THRESHOLD)
				*y = THRESHOLD;
		else if(*y < -THRESHOLD)
				*y= -THRESHOLD;

		if(*z > THRESHOLD)
				*z = THRESHOLD;
		else if(*z < -THRESHOLD)
				*z= -THRESHOLD;

		double factor = 4.0/(THRESHOLD *2.0);
		*x *= factor;
		*y *= factor;
		*z *= factor;

		if(*x >= -ZERO && *x <= ZERO)
		{
			*x=0.0;
		}

		if(*y >= -ZERO && *y <= ZERO)
		{
			*y=0.0;
		}
		if(*z >= -ZERO && *z <= ZERO)
		{
			*z=0.0;
		}
ROS_ERROR("DIT IS EEN VETTE ERROR!");



}

UDPServer * server;

void logCallback(const rosgraph_msgs::Log::ConstPtr& msg)
{
	char * sendstring;

	if(msg->level == 8)
	{
		sendstring =(char *) msg->msg.c_str();
		server->send(sendstring,msg->msg.length());
	}


}
 
int main(int argc, char * argv[])
{
	ros::init(argc, argv, "AndroidKeyboard");
	ros::NodeHandle n;

	 server = new UDPServer(1234);

	ros::Publisher vel_pub;
	ros::Subscriber log_sub;
	geometry_msgs::Twist cmd;

	log_sub = n.subscribe("rosout",10,logCallback);
	cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ros::Rate loop_rate(20);

	while(ros::ok())
	{
		double x,y,z;
		StringToValue(server->readLine(),&x,&y,&z);
		cmd.linear.x = x *-1;
		//cmd.linear.y = z;
		cmd.angular.z = y *-1;

		//publish over ros
		vel_pub.publish(cmd);

		ros::spinOnce();
		loop_rate.sleep();
		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

	}
	
	server->Close();
	delete server;
	return 0;
}
