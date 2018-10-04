// Created 9/24/18
// Matthew Haberbusch

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h> 

const double K_ = 0.01;
const double DT_ = 0.01;
const double VEL_CMD_ = 0.1; 

double pos_ = 1.1; //start it close to the ground
double vel_ = 0.0; 
double force_ = 0.0;

ros::Publisher arm_position_publisher_; //this makes it global?

void forceCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	force_ = ft_sensor.wrench.force.z; 

	vel_ = VEL_CMD_ - K_ * -force_; //changed force to negative
	pos_ = pos_ + vel_ * DT_;

	std_msgs::Float64 arm_position_msg; 
	arm_position_msg.data = pos_;
	arm_position_publisher_.publish(arm_position_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "compliant_touch"); //name of this node
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Float64>("/irb120/joint2_position_controller/command", 1);
	arm_position_publisher_ = pub; //this makes it global?

	ros::Subscriber force_subscriber = nh.subscribe("ft_sensor_topic", 1, forceCallback);
    
    ros::Rate naptime(100);// in hz
    
    while(ros::ok()) {
		ros::spinOnce();
		naptime.sleep();
	}
	
	//ros::spin();
}
    
