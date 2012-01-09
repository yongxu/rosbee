#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
using namespace std;


#define BUFFERLENGHT 256


class UDPServer
{
	private:
		 int sockfd, newsockfd, port;
     socklen_t clilen;
     char buffer[BUFFERLENGHT];
     struct sockaddr_in serv_addr, cli_addr;

		bool openSocket()
		{   
			sockfd = socket(AF_INET, SOCK_STREAM, 0);
		   if (sockfd < 0) 
				return false;
			 return true;
		}

	  bool bind()
		{
			bzero((char *) &serv_addr, sizeof(serv_addr));
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(port);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
             return false;
		 return true;
		}

	 	bool AcceptClient()
	 	{
				 listen(sockfd,5);
		   clilen = sizeof(cli_addr);
		   newsockfd = accept(sockfd, 
		               (struct sockaddr *) &cli_addr, 
		               &clilen);
		   if (newsockfd < 0) 
					return false;
				return true;	
	 	}
    
   public:
		UDPServer(int port)
		{
			this.port = port;
			openSocket();
			bind();
			AcceptClient();			
		}

		string readLine()
		{
			bzero(buffer,256);
     n = read(newsockfd,buffer,255);
		}
     

		void Close()
		{			 
		   close(newsockfd);
		   close(sockfd);
		}

};


int main(int argc, char * argv[])
{
		ros::init(argc, argv, "tfBroadcaster");
		ros::NodeHandle n;

  	ros::Publisher vel_pub;
	  geometry_msgs::Twist cmd;

		cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
    vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	return 0;
}
