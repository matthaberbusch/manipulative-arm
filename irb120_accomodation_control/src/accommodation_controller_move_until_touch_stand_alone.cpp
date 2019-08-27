// 6/11/2019
// Surag Balajepalli and Matthew Haberbusch
//
// ROS-specific code is labeled with "ROS:"
// Linear algebra code is labeled with "MATH:"
//
// MOVE UNTIL TOUCH code and CONTROL LAW code is labeled accordingly

// Include libraries
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>

// ROS: Declare global variables for subscribers
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
bool jnt_state_update_ = false;

// ROS: Define callback functions for communication
void ftSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	// MATH: Round the sensor values
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;	
}
void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update_ = true;
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;
}
// MATH: Compute the angular displacement required to reconcile the two rotations
Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	Eigen::Matrix3d desired_rotation =  dest_rot * source_rot.inverse();
	Eigen::AngleAxisd desired_ang_ax(desired_rotation);
	Eigen::Vector3d delta_phi;
	Eigen::Vector3d axis = desired_ang_ax.axis();
	delta_phi(0) = axis(0)*desired_ang_ax.angle();
	delta_phi(1) = axis(1)*desired_ang_ax.angle();
	delta_phi(2) = axis(2)*desired_ang_ax.angle();
	return delta_phi;
}

// Main program
int main(int argc, char **argv) {
	// ROS: for communication between programs
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); 
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	ros::Publisher virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1);

	// ROS: Instantiate an object of the custom FK solver class for the ABB IRB 120 robot
	Irb120_fwd_solver irb120_fwd_solver;
	
	// MATH: Declare matricies and vectors
	Eigen::MatrixXd robot_inertia_matrix(6,6);
	Eigen::VectorXd current_endeffector_position(3);
	Eigen::MatrixXd current_endeffector_rotation;
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd result_twist(6);
	Eigen::VectorXd desired_joint_velocity = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd desired_cartesian_acceleration(6);
	Eigen::VectorXd desired_twist = Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd desired_joint_pos = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd virtual_attractor_position(3);
	Eigen::Matrix3d virtual_attractor_rotation;
	
	// Declare constants
	double dt_ = 0.01;
	double MAX_JNT_VEL_NORM = 10;
	double MAX_TWIST_NORM = 0.1;
	double ATTRACTOR_DISTANCE_FOR_MOVEMENT = 0.05, ATTRACTOR_DISTANCE_FOR_CONTACT = 0.01;
    double CONTACT_FORCE_THRESHOLD = 1, T_X_THRESHOLD_ROTATE = 0.2;
	
	// Declare gains
	double B_VIRTUAL = 4000;
	double K_VIRTUAL = 1000; 
	double K_VIRTUAL_ANGULAR = 1000;
	
	// ROS: for communication message format
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virt_attr_log;
	cartesian_log.header.frame_id = "map";
	virt_attr_log.header.frame_id = "map";
	geometry_msgs::Pose virt_attr;
	geometry_msgs::Wrench transformed_wrench;
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
		
	// Define inertia matrix 
	robot_inertia_matrix<<1,0,0,0,0,0,
						  0,1,0,0,0,0,
						  0,0,1,0,0,0,
						  0,0,0,1,0,0,
						  0,0,0,0,1,0,
						  0,0,0,0,0,1;
	robot_inertia_matrix *= 100;

	// MATH: Create LU decomposition of inertia matrix with complete pivoting, inverse
	Eigen::FullPivLU<Eigen::MatrixXd> lu_inertia_mat(robot_inertia_matrix);
	Eigen::MatrixXd inertia_mat_inv = lu_inertia_mat.inverse();

	// MATH: Create static transform for sensor
	Eigen::Affine3d sensor_wrt_flange;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;

	// MATH: Create static transform for tool
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_wrt_sensor_trans;
	tool_wrt_sensor_trans<<0,0,0.05;
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	
	// ROS: Wait until there are some valid values of joint states from the robot controller
	while(!jnt_state_update_) ros::spinOnce();

	// MATH: Initialize current end effector position 
	Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
	Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		
	// ROS: Define rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// Main loop
	while(ros::ok()) {
		// ROS: for communnication between programs
		ros::spinOnce();

		// MATH: Initialize jacobian
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
		Eigen::MatrixXd jacobian_inv = lu_jac.inverse();    
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		
		// MATH: Calculate current end effector pose
		current_endeffector_position = tool_wrt_robot.translation();
		current_endeffector_rotation = tool_wrt_robot.linear(); 
		Eigen::Quaterniond flange_quat(tool_wrt_robot.linear()); // Quaternion is used for logging
		
		// MATH: Convert forces on the sensor to robot base frame	
		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.head(3));
		wrench_wrt_robot.tail(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.tail(3));
        
		// * * * MOVE UNTIL TOUCH * * * 

		// Set virtual attractor to end effector pose
		virtual_attractor_position = current_endeffector_position;
		virtual_attractor_rotation = current_endeffector_rotation;
		// If no contact is felt...
        if(wrench_wrt_robot(0) < CONTACT_FORCE_THRESHOLD) {
			// Keep pulling end effector with virtual attractor at constant distance
            virtual_attractor_position(0) = current_endeffector_position(0) + ATTRACTOR_DISTANCE_FOR_MOVEMENT;
        }
		// If contact is felt...
        else {
			// Keep virtual attractor slightly below surface
            virtual_attractor_position(0) = current_endeffector_position(0) + ATTRACTOR_DISTANCE_FOR_CONTACT;
        }

        // * * * END MOVE UNTIL TOUCH * * *
		
		// MATH: Calculate virtual force
		virtual_force.head(3) = K_VIRTUAL * (virtual_attractor_position - current_endeffector_position);
		virtual_force.tail(3) = K_VIRTUAL_ANGULAR * (delta_phi_from_rots(current_endeffector_rotation, virtual_attractor_rotation));
		
		// * * * CONTROL LAW * * *

		desired_cartesian_acceleration = inertia_mat_inv * (-B_VIRTUAL * desired_twist + wrench_wrt_robot + virtual_force);
		desired_twist += desired_cartesian_acceleration * dt_;

		// MATH: Convert twist into joint velocity
		desired_joint_velocity = jacobian_inv * desired_twist;

		// * * * END CONTROL LAW * * *

		// Ensure that desired joint vel is within set safe limits
		if(desired_joint_velocity.norm() > MAX_JNT_VEL_NORM) desired_joint_velocity = (desired_joint_velocity / desired_joint_velocity.norm()) * MAX_JNT_VEL_NORM;
			
		// MATH: Calculate position from velocities using euler one step integration
		desired_joint_pos = joint_states_ + (desired_joint_velocity * dt_); 
		
		// ROS: Put velocity and position commands into joint state message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(desired_joint_pos(i) * 1000) /1000; 
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(desired_joint_velocity(i) * 1000) /1000;
		arm_publisher.publish(desired_joint_state);
		
		// ROS: for logging
		Eigen::Quaterniond virt_quat(virtual_attractor_rotation);

		// ROS: Publish cartesian coordinates of robot end effector
		cartesian_log.pose.position.x = current_endeffector_position(0);
		cartesian_log.pose.position.y = current_endeffector_position(1);
		cartesian_log.pose.position.z = current_endeffector_position(2);
		cartesian_log.pose.orientation.w = flange_quat.w();
		cartesian_log.pose.orientation.x = flange_quat.x();
		cartesian_log.pose.orientation.y = flange_quat.y();
		cartesian_log.pose.orientation.z = flange_quat.z();
		cartesian_log.header.stamp = ros::Time::now();
		cart_log_pub.publish(cartesian_log);

		// ROS: Publish coordinates of virtual attractor
		virt_attr_log.pose.position.x = virtual_attractor_position(0);
		virt_attr_log.pose.position.y = virtual_attractor_position(1);
		virt_attr_log.pose.position.z = virtual_attractor_position(2);
		virt_attr_log.pose.orientation.w = virt_quat.w();
		virt_attr_log.pose.orientation.x = virt_quat.x();
		virt_attr_log.pose.orientation.y = virt_quat.y();
		virt_attr_log.pose.orientation.z = virt_quat.z();
		virt_attr_log.header.stamp = ros::Time::now();
		virt_attr_after_tf.publish(virt_attr_log);
		
		// ROS: Publish force torque values transformed into robot frame
		transformed_wrench.force.x = wrench_wrt_robot(0);
		transformed_wrench.force.y = wrench_wrt_robot(1);
		transformed_wrench.force.z = wrench_wrt_robot(2);
		transformed_wrench.torque.x = wrench_wrt_robot(3);
		transformed_wrench.torque.y = wrench_wrt_robot(4);
		transformed_wrench.torque.z = wrench_wrt_robot(5);
		ft_pub.publish(transformed_wrench);
		
		// ROS: Ensure update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}