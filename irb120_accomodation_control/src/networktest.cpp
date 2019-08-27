#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>
#include <sys/socket.h>
#include <errno.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "network_test");
	ros::NodeHandle nh;
	double vals;
	char *ip_addr = "192.168.125.1";
	char *msg = "R";
	unsigned char buf[1024];
	int recvlen;
	int fd;
	int enable = 1;
	struct sockaddr_in serv_addr;
	struct sockaddr_in client_addr;
	socklen_t addrlen = sizeof(client_addr);
	struct timeval timeout;
	timeout.tv_sec = 2;
	timeout.tv_usec = 0;
	const unsigned short port_number = 6510;
	const unsigned short port_number_robot = 51265;
	memset((char *)&serv_addr, sizeof(serv_addr), 0);
	memset((char *)&serv_addr, sizeof(client_addr), 0);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(port_number);
	//serv_addr.sin_addr.s_addr = inet_addr(ip_addr);
	client_addr.sin_family = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr(ip_addr);
	//client_addr.sin_port = htons(port_number_robot);
		if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0) {
			ROS_WARN("Cannot create socket, go home");
		}
		if(bind(fd,(struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
	

			ROS_WARN("Cannot bind, go home\n");
			perror("error is : ");
		}
		//if(connect(fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) <0) { ROS_INFO("error");}
		if(setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(timeout)) < 0) ROS_INFO("setsockopt failed");
		if(setsockopt(fd,SOL_SOCKET,SO_REUSEADDR, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");
		if(setsockopt(fd,SOL_SOCKET,SO_REUSEPORT, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");
		if(setsockopt(fd,SOL_SOCKET,IP_FREEBIND, &enable,sizeof(int)) < 0) ROS_INFO("Set reuse failed");




	while(ros::ok()) {
		ROS_INFO("Waiting for message");
		recvlen = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addrlen);

		if(recvlen > 0) {
		ROS_INFO("recvd something!");
			//buf[recvlen] = 0;
			ROS_INFO_STREAM("recv message is of length:" <<recvlen);
		}
		else { 
			ROS_INFO("No response");
		}
		if(sendto(fd, buf, sizeof(buf),0,(struct sockaddr*)&client_addr,addrlen) < 0 ) {
			ROS_WARN("cannot send too");
			perror("error is: ");
		}
		else {
			ROS_INFO("Sent successfully");
		}		
		
	}
		close(fd);
}