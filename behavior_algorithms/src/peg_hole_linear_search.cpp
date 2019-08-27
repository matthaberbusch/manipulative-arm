//Surag Balajepalli 2/13
//Simple state machine using switch case for linear search in peg-hole applications
//requirements
//State 0: Linear move in X until touch
//State 1: Search for hole from force signatures
//State 2: Linear move in X until touch again
//State 3: Press lightly against surface to keep contact
//TODO: Implement state machine better using OO concepts
//doesnt tell you what to do with your orientation, how to prevent garbage values here?
//Also would like to set different accommodation gains for each state
//Find a way to reuse states


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>


geometry_msgs::Pose virt_attr, curr_pose;
geometry_msgs::Wrench ft_robot_frame;

void cart_state_Cb(const geometry_msgs::Pose& cart_pos) {
	curr_pose = cart_pos;
}

void ft_Cb(const geometry_msgs::Wrench& ft_vals) {
	//these are not values from the sensor. They are f/t values transformed into robot base frame
	ft_robot_frame = ft_vals;
}
int main(int argc, char** argv) {
	ros::init(argc,argv,"peg_hole_linear_search");
	ros::NodeHandle nh;
	ros::Subscriber cart_state_sub = nh.subscribe("cartesian_logger",1, cart_state_Cb);
	ros::Subscriber ft_sub = nh.subscribe("transformed_ft_wrench",1,ft_Cb);
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::Pose>("virt_attr",1);
	ros::Publisher gain_vector_pub = nh.advertise<std_msgs::Float64MultiArray>("Ka_diagonal",1);
	int state = 0;
	std_msgs::Float64MultiArray acc_gain_vec;
	acc_gain_vec.data.resize(6);
	double VIRT_ATTR_DIST_MOVE = 0.05, VIRT_ATTR_DIST_SEARCH = 0.02, VIRT_ATTR_DIST_CONTACT = 0.01, VIRT_ATTR_DIST_SLIDE = 0.01;
	double X_THRESHOLD_CONTACT = 100.0, X_THRESHOLD_SLIDE = 1.0;

	while(ros::ok()) {
		ros::spinOnce();
		switch (state) {
			case 0:
				if(abs(ft_robot_frame.force.x) < X_THRESHOLD_CONTACT) {
					ROS_INFO("In state 1");
					//Until a force in X is felt, keep pulling the virtual attractor away from the current pose 
					virt_attr = curr_pose;
					//virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_MOVE;
					virt_attr.position.x = 0.385;
					//Set accommodation gain here. Since We're moving in X axis until touch, the diagonal would be (1,0,0,0,0,0)
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 0;
					acc_gain_vec.data[2] = 0;
					acc_gain_vec.data[3] = 0;
					acc_gain_vec.data[4] = 0;
					acc_gain_vec.data[5] = 0;
					}
				else {
					state = 3; //Going to state 1, linear search, after feeling a force greater than a threshold
				}
				break;
			case 1:
				if(abs(ft_robot_frame.force.x) > X_THRESHOLD_CONTACT) {
					ROS_INFO("In state 2");
					//search in either Y or Z direction until as long as theres a force felt in X
					virt_attr = curr_pose;
					virt_attr.position.z = curr_pose.position.z + VIRT_ATTR_DIST_SEARCH;
					//virt_attr.position.y = curr_pose.position.y + VIRT_ATTR_DIST_SEARCH;
					virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_CONTACT;
					//set accommodation gain of (1,0,0,0,0,0)
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 0;
					acc_gain_vec.data[2] = 0;
					acc_gain_vec.data[3] = 0;
					acc_gain_vec.data[4] = 0;
					acc_gain_vec.data[5] = 0;
				}
				else {
					//move to fitting over the peg
					state = 2;
				}
				break;
			case 2:
				//slide down the peg while actively correcting the orientation to prevent jamming
				//can the accommodation gains be used to emulate a remote center of compliance?
				//force threshold needs to be higher too?
				ROS_INFO("In state 3");
				if(abs(ft_robot_frame.force.x) < X_THRESHOLD_SLIDE) {
					virt_attr = curr_pose; // also need to emulate remote center of compliance. WORK TODO
					virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_SLIDE; //move in X slowly
					//accommodation gains here need to resemble remote center of compliance
				}
				else {
					//high force felt -> contact with bottom surface, transition to staying at a location with slight press on surface
					state = 3;
				}
				break;
			case 3:
				//softly press against the surface
				virt_attr = curr_pose;
				ROS_INFO("In state 4");
				virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_CONTACT;
				//set accommodation gain to be (1,0,0,0,0,0)
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 0;
					acc_gain_vec.data[2] = 0;
					acc_gain_vec.data[3] = 0;
					acc_gain_vec.data[4] = 0;
					acc_gain_vec.data[5] = 0;
				break;
			default:
				virt_attr = curr_pose;
		}
		virt_attr_pub.publish(virt_attr);
		gain_vector_pub.publish(acc_gain_vec);
	}
}