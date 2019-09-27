// 9/3/2019
// Matthew Haberbusch
// Creates a circular buffer from the robot data and publishes the data 4 seconds later.

#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h> 
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <behavior_algorithms/status_service.h> // service message for completion, maybe have the buffer be the client, but that would require skill to be running always


int BUFFER_SIZE = 400; // 100 Hz for 4 seconds
bool jnt_state_update = false;
int write_counter = 0;
int read_counter = 1;
double dt_ = 0.01;

geometry_msgs::PoseStamped ee_pose;
geometry_msgs::PoseStamped virt_attr_pose;
geometry_msgs::Wrench ft_wrench;
sensor_msgs::JointState joint_state;

std::string program_name;
std::string exit_status;
std::string start_status;

// Buffer topics are 
/*
cartesian_logger - geometry_msgs::PoseStamped
tfd_virt_attr - geometry_msgs::PoseStamped
transformed_ft_wrench - geometry_msgs::Wrench
abb120_joint_state - sensor_msgs::JointState
*/

void ee_pose_callback(const geometry_msgs::PoseStamped& ee_pose_from_sub) {
	//geometry_msgs::PoseStamped ee_pose;
	ee_pose = ee_pose_from_sub;
}

void virt_attr_pose_callback(const geometry_msgs::PoseStamped& virt_attr_pose_from_sub) {
	//geometry_msgs::PoseStamped virt_attr_pose;
	virt_attr_pose = virt_attr_pose_from_sub;
}

void ft_wrench_callback(const geometry_msgs::Wrench& ft_wrench_from_sub) {
	//geometry_msgs::Wrench ft_wrench;
	ft_wrench = ft_wrench_from_sub;
	//cout<<ft_wrench_from_sub<<endl;
	//cout<<ft_wrench<<endl;
}

void joint_state_callback(const sensor_msgs::JointState& joint_state_from_sub) {
	jnt_state_update = true;
	//sensor_msgs::JointState joint_state;
	joint_state = joint_state_from_sub;
}

bool exit_status_callback(behavior_algorithms::status_serviceRequest& request, behavior_algorithms::status_serviceResponse& response){
	program_name = request.name.c_str();
	exit_status = request.status.c_str();
	// start_status = "";
	response.success = true;
	return true;
}

bool start_status_callback(behavior_algorithms::status_serviceRequest& request, behavior_algorithms::status_serviceResponse& response){
	program_name = request.name.c_str();
	start_status = request.status.c_str();
	// exit_status = "";
	response.success = true;
	return true;
}

struct RobotDataStruct {
	geometry_msgs::PoseStamped ee_pose;
	geometry_msgs::PoseStamped virt_attr_pose;
	geometry_msgs::Wrench ft_wrench;
	sensor_msgs::JointState joint_state;
	std::string program_name;
	std::string start_status;
	std::string exit_status;
};

int main(int argc, char **argv) {
	// just ROS things
	ros::init(argc, argv, "buffer");
	ros::NodeHandle nh;

	// Subscribers
	ros::Subscriber ee_pose_sub = nh.subscribe("cartesian_logger",1,ee_pose_callback);
	ros::Subscriber virt_attr_pose_sub = nh.subscribe("tfd_virt_attr",1,virt_attr_pose_callback);
	ros::Subscriber ft_wrench_sub = nh.subscribe("transformed_ft_wrench",1,ft_wrench_callback);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,joint_state_callback);

	// Publishers
	ros::Publisher ee_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("delayed_ee_pose",1);
	ros::Publisher virt_attr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("delayed_virt_attr_pose",1);
	ros::Publisher ft_wrench_pub = nh.advertise<geometry_msgs::Wrench>("delayed_ft_wrench",1);
	ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("delayed_joint_state",1);

	// Services
	ros::ServiceServer status_service = nh.advertiseService("status_service", exit_status_callback);
	ros::ServiceServer start_service = nh.advertiseService("start_service", start_status_callback);

	// Create struct
	RobotDataStruct robot_data[BUFFER_SIZE];

	// Spin until we have values
	while(!jnt_state_update) ros::spinOnce();

	// Set the structs values
	robot_data[0].ee_pose = ee_pose;
	robot_data[0].virt_attr_pose = virt_attr_pose;
	robot_data[0].ft_wrench = ft_wrench;
	robot_data[0].joint_state = joint_state;

	ros::Rate naptime(1/dt_);


	while(ros::ok()){
		// Get values
		ros::spinOnce(); // why does it have a spin at the top and bottom?

		// Check if the buffer strings have values, then display them and then clear them
		if(!robot_data[read_counter].program_name.empty()){
			// data is not empty, so read and display then clear

			// check if it is a start or exit status
			if(!robot_data[read_counter].start_status.empty()){
				//  Is a start message
				ROS_WARN("Program %s started with parameter %s", robot_data[read_counter].program_name.c_str(), robot_data[read_counter].start_status.c_str());
				robot_data[read_counter].program_name = "";
				robot_data[read_counter].exit_status = "";
				robot_data[read_counter].start_status = "";

			}
			else{
				//
				ROS_WARN("Program %s ended with reason %s", robot_data[read_counter].program_name.c_str(), robot_data[read_counter].exit_status.c_str());
				robot_data[read_counter].program_name = "";
				robot_data[read_counter].exit_status = "";
				robot_data[read_counter].start_status = "";
			}
			
		}

		// Check if the string variables have values here, then update the buffer value
		if(!program_name.empty()){
			robot_data[write_counter].program_name = program_name;
			robot_data[write_counter].exit_status = exit_status;
			robot_data[write_counter].start_status = start_status;
			program_name = "";
			exit_status = "";
			start_status = "";
		}

		

		robot_data[write_counter].ee_pose = ee_pose;
		robot_data[write_counter].virt_attr_pose = virt_attr_pose;
		robot_data[write_counter].ft_wrench = ft_wrench;
		// cout<<ft_wrench<<endl;
		robot_data[write_counter].joint_state = joint_state;
		// ROS_INFO("Write counter: %d",write_counter);
		// ROS_INFO("Force written: %f",robot_data[write_counter].ft_wrench.force.x);


		ee_pose_pub.publish(robot_data[read_counter].ee_pose);
		virt_attr_pose_pub.publish(robot_data[read_counter].virt_attr_pose);
		ft_wrench_pub.publish(robot_data[read_counter].ft_wrench);
		joint_state_pub.publish(robot_data[read_counter].joint_state);
		// ROS_INFO("Read counter: %d",read_counter);
		// ROS_INFO("Force read: %f",robot_data[read_counter].ft_wrench.force.x);

		// Increase counter
		write_counter = write_counter + 1;
		read_counter = read_counter + 1;
		// If it reaches the length, set to 0 again (could replace this iwht a % BUFFER_SIZE)
		if (write_counter >= BUFFER_SIZE) write_counter = 0;
		if (read_counter >= BUFFER_SIZE) read_counter = 0;

		// Sleep
		naptime.sleep();
		// ros::spinOnce();
	}

}
