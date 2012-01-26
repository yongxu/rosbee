#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
using namespace std;


#define BUFFERLENGHT 256


class UDPServer
{
	private:
	int sockfd, newsockfd, portnr;
    socklen_t clilen;

    struct sockaddr_in serv_addr, cli_addr;

	bool openSocket()
	{   
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		   if (sockfd < 0) 
				return false;
			 return true;
	}

	bool bindServer()
	{
		bzero((char *) &serv_addr, sizeof(serv_addr));
    	serv_addr.sin_family = AF_INET;
    	serv_addr.sin_addr.s_addr = INADDR_ANY;
    	serv_addr.sin_port = htons(portnr);
    	if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
             return false;
		 return true;
		}

	bool AcceptClient()
	{
		listen(sockfd,5);
		clilen = sizeof(cli_addr);
		newsockfd = accept(sockfd,(struct sockaddr *) &cli_addr,&clilen);
		if (newsockfd < 0) 
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
		ROS_INFO("Wainting for CLient on port %i ...",port);
		if(AcceptClient())
		{
			ROS_INFO("Client Connected!");
		}
		else
		{
			ROS_INFO("Connection Error!");	
		}		
	}

	string readLine()
	{
  		char buffer[BUFFERLENGHT];
		bzero(buffer,BUFFERLENGHT);
		if(read(newsockfd,buffer,BUFFERLENGHT-1) > 0)
				return string(buffer);
		return "";
	}
 

	void Close()
	{			 
	   close(newsockfd);
	   close(sockfd);
	}
};

void  StringToValue(string s,double * x, double *y,double *z)
{
		ROS_INFO("Incoming:%s",s.c_str());
		//todo
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "AndroidKeyboard");
	ros::NodeHandle n;

	ROS_INFO("blaat");
  	double walk_vel, run_vel, yaw_rate, yaw_rate_run;
	UDPServer server = UDPServer(1234);

	ros::Publisher vel_pub;
	geometry_msgs::Twist cmd;
		
	cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   	//n_private.param("walk_vel", walk_vel, 0.5);
    //n_private.param("run_vel", run_vel, 1.0);
    //n_private.param("yaw_rate", yaw_rate, 1.0);
    //n_private.param("yaw_run_rate", yaw_rate_run, 1.5);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		double x,y,z;
		StringToValue(server.readLine(),&x,&y,&z);
		cmd.linear.x = x;
		cmd.linear.y = y;
		cmd.linear.z = z;

		//publish over ros
		vel_pub.publish(cmd);

		ros::spinOnce();
		loop_rate.sleep();
		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
	}

	server.Close();	
	return 0;
}
