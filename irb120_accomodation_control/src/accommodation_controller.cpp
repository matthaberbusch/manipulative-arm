//Updated 10/15/2019
//Surag Balajepalli, Matthew Haberbusch, and Rahul Pokharna
//subscribers to a 'virtual attractor' pose
//Performs accommodation control

// ROS: include libraries
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <irb120_accomodation_control/freeze_service.h>
#include <irb120_accomodation_control/set_task_frame.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h> 
#include <string.h>
#include <stdio.h>
using namespace std;

// Declare freeze mode variables
sensor_msgs::JointState last_desired_joint_state_;
bool freeze_mode = false;
std_msgs::Int8 freeze_mode_status;

// Declare filter variables 
// Uncomment if you decide to use a low pass filter
/*
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;
*/

// Declare global variables for subscribers
Eigen::VectorXd frozen_joint_states_ = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd virtual_attractor_pos(3);
Eigen::Matrix3d virtual_attractor_rotation_matrix;
Eigen::Quaterniond virt_quat;

bool virtual_attractor_established = false; 
bool jnt_state_update = false;

// Declare variables of tool frame vectors for skills to use. 
Eigen::Affine3d tool_with_respect_to_robot_;
geometry_msgs::Vector3 x_vec_message;
geometry_msgs::Vector3 y_vec_message;
geometry_msgs::Vector3 z_vec_message;

// Declare variables of task frame vectors for skills to use. 
geometry_msgs::Vector3 x_vec_task_message_;
geometry_msgs::Vector3 y_vec_task_message_;
geometry_msgs::Vector3 z_vec_task_message_;

// Declare variables for task frame vectors
Eigen::Affine3d task_frame_with_respect_to_robot_;
Eigen::MatrixXd task_frame_rotation_matrix_;
Eigen::Vector3d x_vec_task_;
Eigen::Vector3d y_vec_task_;
Eigen::Vector3d z_vec_task_;
geometry_msgs::PoseStamped  task_frame_pose_stamped;

// Declare global variables for points to task frame calculation
Eigen::Vector3d point_A_;
Eigen::Vector3d point_B_;
Eigen::Vector3d point_C_;

// MATH: Function converts a rotation matrix to a vector of angles (phi_x, phi_y, phi_z)
Eigen::Vector3d decompose_rot_mat(Eigen::Matrix3d rot_mat) {
	Eigen::Vector3d vec_of_angles;
	vec_of_angles(0) = atan2(rot_mat(2,1),rot_mat(2,2));
	vec_of_angles(1) = atan2(-1 * rot_mat(2,0), sqrt(pow(rot_mat(2,1),2) + pow(rot_mat(2,2),2)));
	vec_of_angles(2) = atan2(rot_mat(1,0), rot_mat(0,0)); 
	return vec_of_angles;
}

// MATH: Function takes two rotation matricies and ouputs the angular displacement between them in a vector of angles
Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	// R_reqd = R_rob.inv * R_des
	Eigen::Matrix3d desired_rotation =  dest_rot * source_rot.inverse();
	Eigen::AngleAxisd desired_ang_ax(desired_rotation);
	Eigen::Vector3d delta_phi;
	Eigen::Vector3d axis = desired_ang_ax.axis();
	delta_phi(0) = axis(0)*desired_ang_ax.angle();
	delta_phi(1) = axis(1)*desired_ang_ax.angle();
	delta_phi(2) = axis(2)*desired_ang_ax.angle();
	return delta_phi;
}

// MATH: Function converts a vector of euler angles into a rotation matrix
Eigen::Matrix3d rotation_matrix_from_euler_angles(Eigen::Vector3d euler_angle_vector) {
	Eigen::Matrix3d x_rotation;
	Eigen::Matrix3d y_rotation;
	Eigen::Matrix3d z_rotation;
	Eigen::Matrix3d rotation_matrix_from_euler_angles;

	z_rotation(0,0) = cos(euler_angle_vector(2));
   	z_rotation(0,1) = -sin(euler_angle_vector(2));
   	z_rotation(0,2) = 0;
   	z_rotation(1,0) = sin(euler_angle_vector(2));
   	z_rotation(1,1) = cos(euler_angle_vector(2));
   	z_rotation(1,2) = 0;
   	z_rotation(2,0) = 0;
   	z_rotation(2,1) = 0;
   	z_rotation(2,2) = 1;

   	y_rotation(0,0) = cos(euler_angle_vector(1));
   	y_rotation(0,1) = 0;
   	y_rotation(0,2) = sin(euler_angle_vector(1));
   	y_rotation(1,0) = 0;
   	y_rotation(1,1) = 1;
   	y_rotation(1,2) = 0;
   	y_rotation(2,0) = -sin(euler_angle_vector(1));
   	y_rotation(2,1) = 0;
   	y_rotation(2,2) = cos(euler_angle_vector(1));

   	x_rotation(0,0) = 1;
   	x_rotation(0,1) = 0;
   	x_rotation(0,2) = 0;
   	x_rotation(1,0) = 0;
   	x_rotation(1,1) = cos(euler_angle_vector(0));
   	x_rotation(1,2) = -sin(euler_angle_vector(0));
   	x_rotation(2,0) = 0;
   	x_rotation(2,1) = sin(euler_angle_vector(0));
   	x_rotation(2,2) = cos(euler_angle_vector(0));

   	rotation_matrix_from_euler_angles = z_rotation * y_rotation * x_rotation;
}

// ROS: Callback function receives the force-torque wrench
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	// Subscribed to "robotiq_ft_wrench"
	// Round our force values
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;

	// Optional low pass filter starts here (it's a moving average)
	// This was removed, but kept in case anyone would like to use this, we chose not to
	/* 
	if (filter_counter > 9) { //if last 10 readings are recorded
		//low pass filter
		//Block of size (p,q), starting at (i,j); so block(i,j,p,q)
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6); //move everything down
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); // top row is the new coordinates
		wrench_body_coords_ = wrench_filter.colwise().sum(); // sums up the columns into an array
		wrench_body_coords_ /= 10; // now we have an average
	}
	else {
		filter_counter++;
		wrench_filter.block(1,0,9,6) = wrench_filter.block(0,0,9,6);
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); 
		// and then wrench_body_coords_ is just exactly what the sensor reads
	}
	*/
	// low pass filter ends here

}

// ROS: Callback function receives the joint state from "abb120_joint_state" topic
void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update = true;
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;
}

// ROS: Callback funtion receives the virtual attractor pose from "virtual_attractor" topic
void virtualAttractorCallback(const geometry_msgs::PoseStamped& des_pose) {
	virtual_attractor_established = true;
	virtual_attractor_pos(0) = des_pose.pose.position.x;
	virtual_attractor_pos(1) = des_pose.pose.position.y;
	virtual_attractor_pos(2) = des_pose.pose.position.z;

	virt_quat.w() = des_pose.pose.orientation.w;
	virt_quat.x() = des_pose.pose.orientation.x;
	virt_quat.y() = des_pose.pose.orientation.y;
	virt_quat.z() = des_pose.pose.orientation.z;
			
	//seperately filling out the 3x3 rotation matrix
	virtual_attractor_rotation_matrix = virt_quat.normalized().toRotationMatrix();
}

// ROS: Callback funtion called with freeze_service is called; toggles freezemode and sets frozen joint states
bool freezeServiceCallback(irb120_accomodation_control::freeze_serviceRequest &request, irb120_accomodation_control::freeze_serviceResponse &response) {
	// Toggle freezemode
	freeze_mode = !freeze_mode;
	if (freeze_mode) freeze_mode_status.data = 1;
	else freeze_mode_status.data = 0;

	// Set frozen joint states to last desired joing state
	frozen_joint_states_(0) = last_desired_joint_state_.position[0];
	frozen_joint_states_(1) = last_desired_joint_state_.position[1];
	frozen_joint_states_(2) = last_desired_joint_state_.position[2];
	frozen_joint_states_(3) = last_desired_joint_state_.position[3];
	frozen_joint_states_(4) = last_desired_joint_state_.position[4];
	frozen_joint_states_(5) = last_desired_joint_state_.position[5];

	// Print status info to display
	ROS_WARN("Freeze mode has been toggled");
	if(freeze_mode) {
		ROS_INFO("Currently in latch mode");
		response.status = "Latch mode";
		
	}
	else {
		ROS_INFO("Currently in normal mode");
		response.status = "Communication mode";
	}

	// maybe output the status change into the buffer, whenever the freeze mode is toggled

	return true;
}

// in progress 10/29/19
bool setTaskFrameCallback(irb120_accomodation_control::set_task_frameRequest &request, irb120_accomodation_control::set_task_frameResponse &response) {
	// Find tool's current x, y, and vector for use in skills 
	task_frame_with_respect_to_robot_ = tool_with_respect_to_robot_;

	// Just get the data directly from the tool frame, the service does not send or receive data, just used as a trigger mechanism
	// task_frame_rotation_matrix_ = task_frame_with_respect_to_robot_.linear();
	// x_vec_task_ = task_frame_rotation_matrix_.col(0);
	// y_vec_task_ = task_frame_rotation_matrix_.col(1);
	// z_vec_task_ = task_frame_rotation_matrix_.col(2);

	// New task frame stuff:
		Eigen::Vector3d AB_vector;
		Eigen::Vector3d AC_vector;

		AB_vector(0) = point_B_(0) - point_A_(0);
		AB_vector(1) = point_B_(1) - point_A_(1);
		AB_vector(2) = point_B_(2) - point_A_(2);

		AC_vector(0) = point_C_(0) - point_A_(0);
		AC_vector(1) = point_C_(1) - point_A_(1);
		AC_vector(2) = point_C_(2) - point_A_(2);

		z_vec_task_ = AB_vector.cross(AC_vector).normalized();
		x_vec_task_ = AB_vector.normalized();
		y_vec_task_ = z_vec_task_.cross(x_vec_task_);

		task_frame_rotation_matrix_.col(0) = x_vec_task_;
		task_frame_rotation_matrix_.col(1) = y_vec_task_;
		task_frame_rotation_matrix_.col(2) = z_vec_task_;

	x_vec_task_message_.x = x_vec_task_(0);
	x_vec_task_message_.y = x_vec_task_(1);
	x_vec_task_message_.z = x_vec_task_(2);
	y_vec_task_message_.x = y_vec_task_(0);
	y_vec_task_message_.y = y_vec_task_(1);
	y_vec_task_message_.z = y_vec_task_(2);
	z_vec_task_message_.x = z_vec_task_(0);
	z_vec_task_message_.y = z_vec_task_(1);
	z_vec_task_message_.z = z_vec_task_(2);

	// Define task frame as a pose stamped
	 task_frame_pose_stamped.pose.position.x = point_A_(0);
	 task_frame_pose_stamped.pose.position.y = point_A_(1);
	 task_frame_pose_stamped.pose.position.z = point_A_(2);
	 Eigen::Quaterniond task_frame_orientation_quaternion(task_frame_with_respect_to_robot_.linear());
	 // Eigen::Quaterniond task_frame_orientation_quaternion(task_frame_rotation_matrix_); // TODO switch these lines and find out what is wrong with this one
	 task_frame_pose_stamped.pose.orientation.x = task_frame_orientation_quaternion.x();
	 task_frame_pose_stamped.pose.orientation.y = task_frame_orientation_quaternion.y();
	 task_frame_pose_stamped.pose.orientation.z = task_frame_orientation_quaternion.z();
	 task_frame_pose_stamped.pose.orientation.w = task_frame_orientation_quaternion.w();

	response.status = "task frame set"; 
	return true; 
}

// in progress 11/20/19

// bool setPlanePointCallback(irb120_accomodation_control::set_plane_pointRequest &request, irb120_accomodation_control::set_plane_pointResponse &response) {
// point_A_ = current_end_effector_pose.head(); //TODO make current_end_effector a global

// response.status = "point A set"
// return true;
// }


// Main rogram
int main(int argc, char **argv) {
	// ROS: for communication
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;

	// ROS: Subscribers receive data from these topics
	ros::Subscriber virtual_attractor_sub = nh.subscribe("Virt_attr_pose",1,virtualAttractorCallback); // ROS: Subscribe to the virtual attractor pose
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, forceTorqueSensorCallback); // ROS: Subscribe to the force-torque sensor wrench
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback); // ROS: Subscribe to our robot's joint states

	// ROS: Publishers send data to these topics
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1); // TODO CHANGE FOR ROBOT AGNOSTIC
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); // ROS: Publish the cartesian coordinates of the end effector in the robot coordinate frame
	ros::Publisher robot_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_frame",1); // ROS: Publish a zero pose stamped to visualize the robot frame
	ros::Publisher task_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("task_frame",1); // ROS: Publish the task frame
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1); // ROS: Publish the force-torque wrench in the robot coordinate frame
	ros::Publisher virtual_attractor_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1); // ROS: Publish the virtual attractor pose in the robot coordinate frame
	ros::Publisher bumpless_virtual_attractor_after_tf = nh.advertise<geometry_msgs::PoseStamped>("bumpless_tfd_virt_attr",1); // ROS: Publish the bumpless virtual attractor pose calculated from the force-torque wrench
	ros::Publisher freeze_mode_pub = nh.advertise<std_msgs::Int8>("freeze_mode_topic",1); // ROS: Publish whether freeze mode is on or off
	ros::Publisher x_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_x",1); // ROS: Publish the tool coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_y",1); // ROS: Publish the tool coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_z",1); // ROS: Publish the tool coordinate frame's z vector in the robot coordinate frame
	ros::Publisher x_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_x",1); // ROS: Publish the task coordinate frame's x vector in the robot coordinate frame
	ros::Publisher y_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_y",1); // ROS: Publish the task coordinate frame's y vector in the robot coordinate frame
	ros::Publisher z_vec_task_pub = nh.advertise<geometry_msgs::Vector3>("task_vector_z",1); // ROS: Publish the task coordinate frame's z vector in the robot coordinate frame

	ros::ServiceServer task_frame_service = nh.advertiseService("task_frame_service",setTaskFrameCallback);

	// ROS: Service to toggle freeze mode
	ros::ServiceServer freeze_service = nh.advertiseService("freeze_service",freezeServiceCallback);

	// Set the freeze mode status to off. This does not toggle freeze mode. It's only for display purposes.
	freeze_mode_status.data = 0; // or here make it frozen, and set the mode to be 1 by default
	
	// MATH: Instantiate an object of the custom forward kinematics solver class for our ABB IRB 120 robot
	Irb120_fwd_solver irb120_fwd_solver;

	// Declare matricies and vectors
	Eigen::MatrixXd robot_inertia_matrix(6,6);
	Eigen::VectorXd current_end_effector_pose(6);
	Eigen::VectorXd wrench_with_respect_to_robot(6);
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd desired_joint_velocity = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd desired_cartesian_acceleration(6);
	Eigen::VectorXd desired_twist = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd desired_twist_with_gain = Eigen::VectorXd::Zero(6);

	// Define constants
	double dt_ = 0.01;
	double MAX_JOINT_VELOCITY_NORM = 10;
	
	// Define damping and gains
	double B_virtual_translational = 4000; 
	double B_virtual_rotational = 100; // was 100
	double K_virtual_translational = 1000; 
	double K_virtual_angular = 40;
	
	// Declare joint state, pose stamped, pose, and wrench variables
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virtual_attractor_log, bumpless_virtual_attractor_log;
	geometry_msgs::Pose virtual_attractor;
	geometry_msgs::Wrench transformed_wrench;

	// Declare a zero PoseStamped for robot frame visualization
	geometry_msgs::PoseStamped robot_frame_pose_stamped;
	robot_frame_pose_stamped.header.frame_id = "map";

	// ROS: Format messages for ROS communications
	cartesian_log.header.frame_id = "map";
	virtual_attractor_log.header.frame_id = "map";
	bumpless_virtual_attractor_log.header.frame_id = "map";
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	last_desired_joint_state_.position.resize(6);
	last_desired_joint_state_.velocity.resize(6);
	
	// Define inertia matrix 
	robot_inertia_matrix<<1,0,0,0,0,0,
						  0,1,0,0,0,0,
						  0,0,1,0,0,0,
						  0,0,0,1,0,0,
						  0,0,0,0,1,0,
						  0,0,0,0,0,1;
	robot_inertia_matrix *= 100;

	// MATH: Create inverted inertia matrix
	Eigen::FullPivLU<Eigen::MatrixXd> lu_inertia_mat(robot_inertia_matrix);
	Eigen::MatrixXd inertia_matrix_inverted = lu_inertia_mat.inverse();

	// TODO CHANGE FOR ROBOT AGNOSTIC
	// Declare our sensor's static transform 
	Eigen::Affine3d sensor_with_respect_to_flange;
	Eigen::Matrix3d sensor_rotation;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.08;
	sensor_rotation<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_with_respect_to_flange.linear() = sensor_rotation;
	sensor_with_respect_to_flange.translation() = sensor_origin;

	// Declare our tool's static transform
	Eigen::Affine3d tool_with_repsect_to_sensor;
	Eigen::Matrix3d tool_with_repsect_to_sensor_rotation = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_with_repsect_to_sensor_translation;
	tool_with_repsect_to_sensor_translation<<0,0,0.21; // 0,0,0.1 is with cyllinder, 0.24 -> 0.23 -> 0.22 with pencil, 
	tool_with_repsect_to_sensor.linear() = tool_with_repsect_to_sensor_rotation;
	tool_with_repsect_to_sensor.translation() = tool_with_repsect_to_sensor_translation;
	
	// ROS: Wait until there are some valid values of joint states from the robot controller
	// ros::spinOnce() allows subscribers to look at their topics
	while(!jnt_state_update) ros::spinOnce();

	// Initialize current end effector position and use values recieved from hand controller as offsets to that value

	Eigen::Affine3d flange_with_respect_to_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	Eigen::Affine3d sensor_with_respect_to_robot = flange_with_respect_to_robot * sensor_with_respect_to_flange;
	tool_with_respect_to_robot_ = sensor_with_respect_to_robot * tool_with_repsect_to_sensor;
	//task_frame_with_respect_to_robot_ = tool_with_respect_to_robot_;

	// Define default/starting transform for the task frame, set from values obtained with a working task frame. Can be updated by calling service
	task_frame_with_respect_to_robot_.translation() = Eigen::Vector3d(0.949345922741,0.00167035566543,0.741465747577);
	Eigen::Quaterniond rot(0.49080197581,0.492946916981,0.50987648389,0.50610545221);
	task_frame_with_respect_to_robot_.linear() = rot.toRotationMatrix();

	// Define 
	Eigen::VectorXd initial_end_effector_pose = Eigen::VectorXd::Zero(6); 
	initial_end_effector_pose.head(3) = tool_with_respect_to_robot_.translation();
	initial_end_effector_pose.tail(3) = decompose_rot_mat(tool_with_respect_to_robot_.linear());

	// Find the tool's x, y, and z vectors with respect to the robot; needed for certain skills
	Eigen::MatrixXd tool_R = tool_with_respect_to_robot_.linear();
	Eigen::Vector3d x_vec = tool_R.col(0);
	Eigen::Vector3d y_vec = tool_R.col(1);
	Eigen::Vector3d z_vec = tool_R.col(2);
	x_vec_message.x = x_vec(0);
	x_vec_message.y = x_vec(1);
	x_vec_message.z = x_vec(2);
	y_vec_message.x = y_vec(0);
	y_vec_message.y = y_vec(1);
	y_vec_message.z = y_vec(2);
	z_vec_message.x = z_vec(0);
	z_vec_message.y = z_vec(1);
	z_vec_message.z = z_vec(2);

	// Find the tool's x, y, and z vectors with respect to the robot; needed for certain skills
	task_frame_rotation_matrix_ = task_frame_with_respect_to_robot_.linear();
	x_vec_task_ = task_frame_rotation_matrix_.col(0);
	y_vec_task_ = task_frame_rotation_matrix_.col(1);
	z_vec_task_ = task_frame_rotation_matrix_.col(2);
	x_vec_task_message_.x = x_vec_task_(0);
	x_vec_task_message_.y = x_vec_task_(1);
	x_vec_task_message_.z = x_vec_task_(2);
	y_vec_task_message_.x = y_vec_task_(0);
	y_vec_task_message_.y = y_vec_task_(1);
	y_vec_task_message_.z = y_vec_task_(2);
	z_vec_task_message_.x = z_vec_task_(0);
	z_vec_task_message_.y = z_vec_task_(1);
	z_vec_task_message_.z = z_vec_task_(2);

	// Define task frame as a pose stamped
	task_frame_pose_stamped.pose.position.x =  task_frame_with_respect_to_robot_.translation()(0);
	task_frame_pose_stamped.pose.position.y =  task_frame_with_respect_to_robot_.translation()(1);
	task_frame_pose_stamped.pose.position.z =  task_frame_with_respect_to_robot_.translation()(2);
	Eigen::Quaterniond task_frame_orientation_quaternion( task_frame_with_respect_to_robot_.linear());
	task_frame_pose_stamped.pose.orientation.x = task_frame_orientation_quaternion.x();
	task_frame_pose_stamped.pose.orientation.y = task_frame_orientation_quaternion.y();
	task_frame_pose_stamped.pose.orientation.z = task_frame_orientation_quaternion.z();
	task_frame_pose_stamped.pose.orientation.w = task_frame_orientation_quaternion.w();
	task_frame_pose_stamped.header.frame_id = "map";

	// Declare the bumpless virtual attractor's pose
	Eigen::VectorXd bumpless_virtual_attractor_position(3);
	Eigen::Vector3d bumpless_virtual_attractor_angles;

	// Define the freeze/latch_mode variables
	bool first_loop_after_freeze = true;
	// freeze_mode = true;  // here set freeze mode to be true, always start in freeze mode

	// store the current joint states as the frozen joints before entering the loop
	for(int i = 0; i < 6; i++) frozen_joint_states_(i) = joint_states_(i);

	// Define rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// Main loop
	while(ros::ok()) {
		// ROS: Subscribers get info
		ros::spinOnce();

		// Initialize jacobians 
		// TODO CHANGE FOR ROBOT AGNOSTIC
		// THIS IS NOT ROBOT AGNOSTIC, but if this section is replaced with the appropriate jacobian calculation for a new robot, then the program will run properly. 
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_with_respect_to_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_with_respect_to_robot = flange_with_respect_to_robot * sensor_with_respect_to_flange;
		tool_with_respect_to_robot_ = sensor_with_respect_to_robot * tool_with_repsect_to_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
		Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); // TODO what to do when matrix is non invertible?   
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();

		// Find current end effector pose
		current_end_effector_pose.head(3) = tool_with_respect_to_robot_.translation();
		current_end_effector_pose.tail(3) = decompose_rot_mat(tool_with_respect_to_robot_.linear()); 
		Eigen::Quaterniond flange_quat(tool_with_respect_to_robot_.linear());

		// Find tool's current x, y, and z vector for use in skills 
		tool_R = tool_with_respect_to_robot_.linear();
		Eigen::Vector3d x_vec = tool_R.col(0);
		Eigen::Vector3d y_vec = tool_R.col(1);
		Eigen::Vector3d z_vec = tool_R.col(2);
		x_vec_message.x = x_vec(0);
		x_vec_message.y = x_vec(1);
		x_vec_message.z = x_vec(2);
		y_vec_message.x = y_vec(0);
		y_vec_message.y = y_vec(1);
		y_vec_message.z = y_vec(2);
		z_vec_message.x = z_vec(0);
		z_vec_message.y = z_vec(1);
		z_vec_message.z = z_vec(2);

		// Update wrench with respect to robot
		wrench_with_respect_to_robot.head(3) = sensor_with_respect_to_robot.linear() * (wrench_body_coords_.head(3));
		wrench_with_respect_to_robot.tail(3) = sensor_with_respect_to_robot.linear() * (wrench_body_coords_.tail(3));
		
		// Compute virtual attractor forces
		// If the controller has just started, use the bumpless virtual attractor
		if (!freeze_mode && first_loop_after_freeze) {
			// Find bumpless virtual attractor position: the negative force devided by the translational spring constant, plus the end effector position
			bumpless_virtual_attractor_position = -wrench_with_respect_to_robot.head(3) / K_virtual_translational + current_end_effector_pose.head(3);
			// Update virtual attractor position to bumpless virtual attractor position
			virtual_attractor_pos(0) = bumpless_virtual_attractor_position(0);
			virtual_attractor_pos(1) = bumpless_virtual_attractor_position(1);
			virtual_attractor_pos(2) = bumpless_virtual_attractor_position(2);
			// Find bumpless virtual attractor angles: (the negative torque divided by the anglular spring constant) plus (the angle of the tool with respect to the robot)
			bumpless_virtual_attractor_angles(0) =  -wrench_with_respect_to_robot(3) / K_virtual_angular + decompose_rot_mat(tool_with_respect_to_robot_.linear())(0);
			bumpless_virtual_attractor_angles(1) =  -wrench_with_respect_to_robot(4) / K_virtual_angular + decompose_rot_mat(tool_with_respect_to_robot_.linear())(1);
			bumpless_virtual_attractor_angles(2) =  -wrench_with_respect_to_robot(5) / K_virtual_angular + decompose_rot_mat(tool_with_respect_to_robot_.linear())(2);
			// Update the virtual attractor rotation matrix
			virtual_attractor_rotation_matrix = rotation_matrix_from_euler_angles(bumpless_virtual_attractor_angles);

			// Calculate the virtual forces
			virtual_force.head(3) = K_virtual_translational * (virtual_attractor_pos - current_end_effector_pose.head(3));
			virtual_force.tail(3) = K_virtual_angular * (delta_phi_from_rots(tool_with_respect_to_robot_.linear(), virtual_attractor_rotation_matrix));
			// Output 
			cout<<"Bumpless attractor used"<<endl;
			cout<<"wrench torques"<<endl<<wrench_with_respect_to_robot.tail(3)<<endl;
			cout<<"decomposed angles of the tool frame"<<endl<<decompose_rot_mat(tool_with_respect_to_robot_.linear())<<endl;
			cout<<"bumpless virtual attractor pose: "<<endl;
			cout<<bumpless_virtual_attractor_position<<endl;
		}

		// If the controller has not just started, use the normal virtual attractor
		else {
			// If we have established a virtual attractor
			if(virtual_attractor_established) {
				// Calculate the virtual forces
				virtual_force.head(3) = K_virtual_translational * (virtual_attractor_pos - current_end_effector_pose.head(3));
				virtual_force.tail(3) = K_virtual_angular * (delta_phi_from_rots(tool_with_respect_to_robot_.linear(), virtual_attractor_rotation_matrix));
				// Output
				cout<<"virtual attractor pose"<<endl;
				cout<<virtual_attractor_pos<<endl;
			}
			// If we haven't established a virtual attractor
			else{
				// Set virtual force to 0
				virtual_force<<0,0,0,0,0,0;
				cout<<"No virtual attractor command received"<<endl;	
			}
		}

		cout<<"virtual force: "<<endl<<virtual_force<<endl;

		// CONTROL LAW BEGIN
		// Calculate the desired twists with the two damping gains
		desired_twist_with_gain.head(3) = -B_virtual_translational * desired_twist.head(3);
		desired_twist_with_gain.tail(3) = -B_virtual_rotational * desired_twist.tail(3);
		// Calculate the desired cartesian accelleration
		desired_cartesian_acceleration = inertia_matrix_inverted*(desired_twist_with_gain + wrench_with_respect_to_robot + virtual_force);
		// CONTROL LAW END

		// Calculate the desired twist by integrating the acceleration
		if(!freeze_mode) desired_twist += desired_cartesian_acceleration*dt_;
		// Zero the twist if freeze/latched mode is on
		else {
			desired_twist<<0,0,0,0,0,0;
			// Output
			cout<<"freeze mode on"<<endl;
		}

		// Calculate the desired joint velocities from the desired twist
		// TODO CHANGE FOR ROBOT AGNOSTIC
		// This is where it is not robot agnostic, if you want to change the robot, change the jacobian accordingly
		if(!freeze_mode) desired_joint_velocity = jacobian_inv*desired_twist;
		else desired_joint_velocity<<0,0,0,0,0,0;
									
		// Ensure that desired joint vel is within set safe limits
		if(desired_joint_velocity.norm() > MAX_JOINT_VELOCITY_NORM) desired_joint_velocity = (desired_joint_velocity / desired_joint_velocity.norm()) * MAX_JOINT_VELOCITY_NORM;
		
		// Calculate joint positions from velocities using Euler one step integration
		Eigen::MatrixXd des_jnt_pos = Eigen::VectorXd::Zero(6);
		if(!freeze_mode) des_jnt_pos = joint_states_ + (desired_joint_velocity * dt_);
		else des_jnt_pos = frozen_joint_states_;

		// Put velocity and pososition commands into Jointstate message and round
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; 
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(desired_joint_velocity(i) * 1000) /1000;

		// Publish desired jointstate
		arm_publisher.publish(desired_joint_state);
		last_desired_joint_state_ = desired_joint_state;

		cout<<"Current EE Pose:"<<endl<<current_end_effector_pose<<endl;

		// Publish cartesian coordinates of robot end effector
		cartesian_log.pose.position.x = current_end_effector_pose(0);
		cartesian_log.pose.position.y = current_end_effector_pose(1);
		cartesian_log.pose.position.z = current_end_effector_pose(2);
		cartesian_log.pose.orientation.w = flange_quat.w();
		cartesian_log.pose.orientation.x = flange_quat.x();
		cartesian_log.pose.orientation.y = flange_quat.y();
		cartesian_log.pose.orientation.z = flange_quat.z();
		cartesian_log.header.stamp = ros::Time::now();
		cart_log_pub.publish(cartesian_log);

		// Publish virtual attractor coordinates
		virtual_attractor_log.pose.position.x = virtual_attractor_pos(0);
		virtual_attractor_log.pose.position.y = virtual_attractor_pos(1);
		virtual_attractor_log.pose.position.z = virtual_attractor_pos(2);
		virtual_attractor_log.pose.orientation.w = virt_quat.w();
		virtual_attractor_log.pose.orientation.x = virt_quat.x();
		virtual_attractor_log.pose.orientation.y = virt_quat.y();
		virtual_attractor_log.pose.orientation.z = virt_quat.z();
		virtual_attractor_log.header.stamp = ros::Time::now();
		virtual_attractor_after_tf.publish(virtual_attractor_log);

		// Publish zero posed stamp for robot frame visualization in rviz
		robot_frame_pose_stamped.header.stamp = ros::Time::now();
		robot_frame_pub.publish(robot_frame_pose_stamped);

		// Publish the task frame for visualization in rviz
		task_frame_pose_stamped.header.stamp = ros::Time::now();
		task_frame_pub.publish(task_frame_pose_stamped);

		//publish coordinates of bumpless virtual attractor
		/*
		bumpless_virtual_attractor_log.pose.position.x = bumpless_virtual_attractor_position(0);
		bumpless_virtual_attractor_log.pose.position.y = bumpless_virtual_attractor_position(1);
		bumpless_virtual_attractor_log.pose.position.z = bumpless_virtual_attractor_position(2);
		bumpless_virtual_attractor_log.pose.orientation.w = 0; //TODO CHANGE CHANGE CHANGE CHANGE. THESE ARE PLACEHOLDERS.
		bumpless_virtual_attractor_log.pose.orientation.x = 0;
		bumpless_virtual_attractor_log.pose.orientation.y = 0;
		bumpless_virtual_attractor_log.pose.orientation.z = 0;
		bumpless_virtual_attractor_log.header.stamp = ros::Time::now();
		bumpless_virtual_attractor_after_tf.publish(bumpless_virtual_attractor_log);
		*/

		// Publish force-torque values transformed into robot frame
		transformed_wrench.force.x = wrench_with_respect_to_robot(0);
		transformed_wrench.force.y = wrench_with_respect_to_robot(1);
		transformed_wrench.force.z = wrench_with_respect_to_robot(2);
		transformed_wrench.torque.x = wrench_with_respect_to_robot(3);
		transformed_wrench.torque.y = wrench_with_respect_to_robot(4);
		transformed_wrench.torque.z = wrench_with_respect_to_robot(5);
		ft_pub.publish(transformed_wrench);

		// Publish tool frame vectors
		x_vec_pub.publish(x_vec_message);
		y_vec_pub.publish(y_vec_message);
		z_vec_pub.publish(z_vec_message);

		// Publish task frame vectors
		x_vec_task_pub.publish(x_vec_task_message_);
		y_vec_task_pub.publish(y_vec_task_message_);
		z_vec_task_pub.publish(z_vec_task_message_);

		// Pulish freeze mode status
		freeze_mode_pub.publish(freeze_mode_status);

		// Set freeze/latched mode checkers
		if (freeze_mode) first_loop_after_freeze = true;
		else first_loop_after_freeze = false;

		// Ensure update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}