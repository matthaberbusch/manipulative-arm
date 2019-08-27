// Created 9/24/18
// Matthew Haberbusch

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h> 
#include <sensor_msgs/JointState.h> 

const double K_ = 0.45; //Best is 0.01
const double DT_ = 0.01;
const double VEL_CMD_ = 0.1; 

double pos_ = -2.0; //start it close to the ground DEGREES
double vel_ = 0.0; 
double force_ = 0.0;

std_msgs::Float64 K_value;
std_msgs::Float64 V_value;

ros::Publisher arm_position_publisher_; //this makes it global?

void forceCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	force_ = ft_sensor.wrench.force.z; 

	vel_ = VEL_CMD_ - K_ * -force_; //changed force to negative
	//pos_ = pos_ + vel_ * DT_; //for debugging
	pos_ = 0.0001;

	sensor_msgs::JointState arm_position_msg;
	arm_position_msg.header.stamp = ros::Time::now();
	arm_position_msg.position.resize(6);
	arm_position_msg.position[0] = pos_; //for debugging
	//arm_position_msg.position[0] = 0;
	arm_position_msg.position[1] = pos_; // 0 is joing 1; 1 is joint 2 etc.
	arm_position_msg.position[2] = pos_;
	//arm_position_msg.position[2] = 0;
	arm_position_msg.position[3] = pos_;
	//arm_position_msg.position[3] = 0;
	arm_position_msg.position[4] = pos_;
	//arm_position_msg.position[4] = 0;
	arm_position_msg.position[5] = pos_;
	//arm_position_msg.position[5] = 0;
	arm_position_publisher_.publish(arm_position_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "compliant_touch"); //name of this node
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command", 1);
	arm_position_publisher_ = pub; //this makes it global?

	ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("K_value", 1);
	ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("V_value", 1);


	ros::Subscriber force_subscriber = nh.subscribe("robotiq_ft_wrench", 1, forceCallback);

	K_value.data = K_;
    V_value.data = VEL_CMD_;
    
    ros::Rate naptime(100);// in hz

    while(ros::ok()) {
		ros::spinOnce();
		pub2.publish(K_value);
		pub3.publish(V_value);
		naptime.sleep();
	}
	
	//ros::spin();
}
    
