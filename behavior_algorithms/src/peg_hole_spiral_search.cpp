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
#include <Eigen/QR>
#include <Eigen/Dense>
//#include <iostream.h>

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
	ros::init(argc,argv,"peg_hole_spiral_search");
	ros::NodeHandle nh;
	ros::Subscriber cart_state_sub = nh.subscribe("cartesian_logger",1, cart_state_Cb);
	ros::Subscriber ft_sub = nh.subscribe("transformed_ft_wrench",1,ft_Cb);
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::Pose>("virt_attr",1);
	ros::Publisher gain_vector_pub = nh.advertise<std_msgs::Float64MultiArray>("Ka_diagonal",1);
	int state = 0;
	std_msgs::Float64MultiArray acc_gain_vec;
	acc_gain_vec.data.resize(6);
	double VIRT_ATTR_DIST_MOVE = 0.1, VIRT_ATTR_DIST_SEARCH = 0.02, VIRT_ATTR_DIST_CONTACT = 0.01, VIRT_ATTR_DIST_SLIDE = 0.05;
	double X_THRESHOLD_CONTACT = 1, X_THRESHOLD_SLIDE = -10, WIGGLE_OFFSET = 0.01;
	double has_hit = 0;
	Eigen::VectorXd hit_point(6);
	double theta;
	double r = 0.01;
	double D_THETA = 0.01;
	double D_R = 0.015; // 5 mm
	ros::Rate naptime(50);
	while(ros::ok()) {
		ros::spinOnce();

		switch (state) {
			case 0:
				if(abs(ft_robot_frame.force.x) < X_THRESHOLD_CONTACT) {
					ROS_INFO("State 1");
					//Until a force in X is felt, keep pulling the virtual attractor away from the current pose 
					virt_attr = curr_pose;
					virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_MOVE;
					//Set accommodation gain here. Since We're moving in X axis until touch, the diagonal would be (1,0,0,0,0,0)
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 0;
					acc_gain_vec.data[2] = 0;
					acc_gain_vec.data[3] = 0;
					acc_gain_vec.data[4] = 0;
					acc_gain_vec.data[5] = 0;
					}
				else {
					state = 1; //Going to state 1, spiral search, after feeling a force greater than a threshold
				}
				break;
			case 1:
				ROS_INFO("State 2");

					//spiral search logic goes here
					if(has_hit <= 10) {
						hit_point(0) = curr_pose.position.x;
						hit_point(1) = curr_pose.position.y;
						hit_point(2) = curr_pose.position.z;
						std::cout<<"Hit point"<<std::endl<<hit_point<<std::endl;
			//has_hit = true;
						theta = 0;
						has_hit = has_hit + 1;
						virt_attr = curr_pose;
					}
					else {
						theta = theta + D_THETA;
						if(theta >= 2*M_PI){
							r = r + D_R;
							theta = 0;
						}
					virt_attr = curr_pose;
					virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_CONTACT;//current_ee_pos(0); 
					virt_attr.position.y = hit_point(1) + r*cos(theta);
					virt_attr.position.z = hit_point(2) + r*sin(theta);
				
					}
					
					//set accommodation gain of (1,0,0,0,0,0)
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 1;
					acc_gain_vec.data[2] = 1;
					acc_gain_vec.data[3] = 1;
					acc_gain_vec.data[4] = 1;
					acc_gain_vec.data[5] = 1;
				
				
				if(abs(ft_robot_frame.force.x) < X_THRESHOLD_CONTACT) {
					//move to fitting over the peg
					state = 2;
				}
				break;
			case 2:
				ROS_INFO("State 3");

				//slide down the peg while actively correcting the orientation to prevent jamming
				//can the accommodation gains be used to emulate a remote center of compliance?
				//force threshold needs to be higher too?
				if((ft_robot_frame.force.x) > X_THRESHOLD_SLIDE) {
					virt_attr = curr_pose; // also need to emulate remote center of compliance. WORK TODO
					WIGGLE_OFFSET *= -1;
					virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_SLIDE; //move in X slowly
					virt_attr.position.y = curr_pose.position.y; //+ WIGGLE_OFFSET;
					virt_attr.position.z = curr_pose.position.z;// + WIGGLE_OFFSET;
					//accommodation gains here need to resemble remote center of compliance
					acc_gain_vec.data[0] = 1;
					acc_gain_vec.data[1] = 1;
					acc_gain_vec.data[2] = 1;
					acc_gain_vec.data[3] = 1;
					acc_gain_vec.data[4] = 1;
					acc_gain_vec.data[5] = 1;
				}
				else {
					//high force felt -> contact with bottom surface, transition to staying at a location with slight press on surface
					state = 3;
				}
				break;
			case 3:
				ROS_INFO("State 4");

				//softly press against the surface
				virt_attr = curr_pose;
				virt_attr.position.x = curr_pose.position.x + VIRT_ATTR_DIST_CONTACT;
				
				//set accommodation gain to be (1,0,0,0,0,0)
				acc_gain_vec.data[0] = 1;
				acc_gain_vec.data[1] = 1;
				acc_gain_vec.data[2] = 1;
				acc_gain_vec.data[3] = 1;
				acc_gain_vec.data[4] = 1;
				acc_gain_vec.data[5] = 1;
				break;
			default:
				virt_attr = curr_pose;
		}
		virt_attr_pub.publish(virt_attr);
		gain_vector_pub.publish(acc_gain_vec);
		naptime.sleep();

	}
}