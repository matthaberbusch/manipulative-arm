//6/2/2018
//Surag Balajepalli
//subscribers to a 'virtual attractor' pose
//Performs accommodation control 


#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <irb120_accomodation_control/freeze_service.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>

// MARKER
#include <ros/ros.h> 
#include <string.h>
#include <stdio.h>  
using namespace std;

sensor_msgs::JointState last_desired_joint_state_;
bool freeze_mode = false;
std_msgs::Int8 freeze_mode_int;

// filter variables 
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;

// global variables for subscribers
Eigen::VectorXd frozen_joint_states_ = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd virt_attr_pos(3);
Eigen::Matrix3d virt_attr_rot;
Eigen::Quaterniond virt_quat;

bool virt_attr_established = false; 
bool jnt_state_update = false;

geometry_msgs::Vector3 x_vec_message;
geometry_msgs::Vector3 y_vec_message;
geometry_msgs::Vector3 z_vec_message;


Eigen::Vector3d decompose_rot_mat(Eigen::Matrix3d rot_mat) {
	//takes rot mat and decomposes it to phi_x, phi_y, phi_z
	Eigen::Vector3d vec_of_angles;
	vec_of_angles(0) = atan2(rot_mat(2,1),rot_mat(2,2));
	vec_of_angles(1) = atan2(-1 * rot_mat(2,0), sqrt(pow(rot_mat(2,1),2) + pow(rot_mat(2,2),2)));
	vec_of_angles(2) = atan2(rot_mat(1,0), rot_mat(0,0)); 
	return vec_of_angles;
}

Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	//Takes source rotation matrix and dest rotation matrix
	//Computes the angular displacement required to reconcile the two rotations
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

// subscriber callbacks
void ftSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	// subscribed to "robotiq_ft_wrench"
	// this rounds our values
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;

	// low pass filter starts here (it's a moving average)
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

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update = true;
	//subscribed to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;

}

void virt_attr_CB(const geometry_msgs::PoseStamped& des_pose) {
	// subscribed to "virt_attr"
	virt_attr_established = true;
	virt_attr_pos(0) = des_pose.pose.position.x;
	virt_attr_pos(1) = des_pose.pose.position.y;
	virt_attr_pos(2) = des_pose.pose.position.z;

	
	virt_quat.w() = des_pose.pose.orientation.w;
	virt_quat.x() = des_pose.pose.orientation.x;
	virt_quat.y() = des_pose.pose.orientation.y;
	virt_quat.z() = des_pose.pose.orientation.z;
			
	//seperately filling out the 3x3 rotation matrix
	virt_attr_rot = virt_quat.normalized().toRotationMatrix();
}

bool freeze_service_Callback(irb120_accomodation_control::freeze_serviceRequest &request, irb120_accomodation_control::freeze_serviceResponse &response) {
	
	freeze_mode = !freeze_mode;
	if (freeze_mode) freeze_mode_int.data = 1;
	else freeze_mode_int.data = 0;

	//frozen_joint_states_ = joint_states_; used to use actual joint state, but now we use commanded joint state
	frozen_joint_states_(0) = last_desired_joint_state_.position[0];
	frozen_joint_states_(1) = last_desired_joint_state_.position[1];
	frozen_joint_states_(2) = last_desired_joint_state_.position[2];
	frozen_joint_states_(3) = last_desired_joint_state_.position[3];
	frozen_joint_states_(4) = last_desired_joint_state_.position[4];
	frozen_joint_states_(5) = last_desired_joint_state_.position[5];

	ROS_INFO("Freeze mode has been toggled");
	if(freeze_mode) {
		ROS_INFO("Currently in latch mode");
		response.status = "Latch mode";
		
	}
	else {
		ROS_INFO("Currently in normal mode");
		response.status = "Communication mode";
	}
	return true;
}

// main program
int main(int argc, char **argv) {
	// just ROS things
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;

	// subscribers
	ros::Subscriber virt_attr_sub = nh.subscribe("Virt_attr_pose",1,virt_attr_CB);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);

	// publishers
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1); // CHANGE FOR ROBOT AGNOSTIC
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); 
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	ros::Publisher virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1);
	ros::Publisher bumpless_virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("bumpless_tfd_virt_attr",1);
	ros::Publisher freeze_mode_pub = nh.advertise<std_msgs::Int8>("freeze_mode_topic",1);
	ros::Publisher x_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_x",1);
	ros::Publisher y_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_y",1);
	ros::Publisher z_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_z",1);
	ros::ServiceServer freeze_service = nh.advertiseService("freeze_service",freeze_service_Callback);

	freeze_mode_int.data = 0;
	
	// Instantiate an object of the custom FK solver class for the ABB IRB 120 robot
	Irb120_fwd_solver irb120_fwd_solver;

	// declare matricies and vectors
	Eigen::MatrixXd robot_inertia_matrix(6,6);
	Eigen::VectorXd current_ee_pos(6);
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd des_jnt_vel = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd des_cart_acc(6);
	Eigen::VectorXd des_twist = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd des_twist_w_gain = Eigen::VectorXd::Zero(6);

	// declare constants
	double dt_ = 0.01;
	double MAX_JNT_VEL_NORM = 10;
	// declare gains
	double B_virt_trans = 4000; //was 4000
	double B_virt_rot = 100; //was 4000
	double K_virt = 1000; 
	double K_virt_ang = 40; //was 1000 (newly 100)
	
	// ROS requirement - messages must be in a certain format
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virt_attr_log, bumpless_virt_attr_log; // NEW rcc
	cartesian_log.header.frame_id = "map";
	virt_attr_log.header.frame_id = "map";
	bumpless_virt_attr_log.header.frame_id = "map";
	geometry_msgs::Pose virt_attr;
	geometry_msgs::Wrench transformed_wrench;
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);

	last_desired_joint_state_.position.resize(6);
	last_desired_joint_state_.velocity.resize(6);
	
	// define inertia matrix 
	robot_inertia_matrix<<1,0,0,0,0,0,
						  0,1,0,0,0,0,
						  0,0,1,0,0,0,
						  0,0,0,1,0,0,
						  0,0,0,0,1,0,
						  0,0,0,0,0,1;
	robot_inertia_matrix *= 100;
	Eigen::FullPivLU<Eigen::MatrixXd> lu_inertia_mat(robot_inertia_matrix);
	Eigen::MatrixXd inertia_mat_inv = lu_inertia_mat.inverse();

	// static transform for sensor
	Eigen::Affine3d sensor_wrt_flange;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.08; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;

	// static transform for tool
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_wrt_sensor_trans;
	tool_wrt_sensor_trans<<0,0,0.1; // 0,0,0.05 is old one
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	
	// wait until there are some valid values of joint states from the robot controller
	// ros::spinOnce() allows subscribers to look at their topics
	while(!jnt_state_update) ros::spinOnce();

	// initialize current end effector position and use values recieved from hand controller as offsets to that value
	// TODO remove hand controller stuff before sending to NASA?
	Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
	Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
	Eigen::VectorXd initial_ee_pos = Eigen::VectorXd::Zero(6); 
	initial_ee_pos.head(3) = tool_wrt_robot.translation();
	initial_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear());

	// NEW find z vector
	Eigen::MatrixXd tool_R = tool_wrt_robot.linear();
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

	//NEW bumpless attractor pose
	Eigen::VectorXd bumpless_virt_attr_pose(3);
	Eigen::Vector3d bumpless_virt_attr_angles;
	bool first_loop_after_freeze = true;
	bool latch = false;

	// rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// main loop
	while(ros::ok()) {
		// subscribers get info
		ros::spinOnce();




		// initialize jacobians 
		// THIS IS NOT ROBOT AGNOSTIC, but if this section is replaced with the appropriate jacobian calculation for a new robot, then the program will run properly. 
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
		Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); // TODO what to do when matrix is non invertible?   
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		



		// find current end effector pose
		current_ee_pos.head(3) = tool_wrt_robot.translation();
		current_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear()); 
		Eigen::Quaterniond flange_quat(tool_wrt_robot.linear());



		// NEW find current z vector for movement relative to tool
		tool_R = tool_wrt_robot.linear();
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

		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.head(3));
		wrench_wrt_robot.tail(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.tail(3));

		// NEW Virtual attractor pose based on force felt
		bumpless_virt_attr_pose = -wrench_wrt_robot.head(3) / K_virt + current_ee_pos.head(3);
		
		// Compute virtual attractor forces
		// If the controller has just started, use the bumpless virtual attractor
		if (!freeze_mode && first_loop_after_freeze) {

			virt_attr_pos(0) = bumpless_virt_attr_pose(0);
			virt_attr_pos(1) = bumpless_virt_attr_pose(1);
			virt_attr_pos(2) = bumpless_virt_attr_pose(2);
			bumpless_virt_attr_angles(0) =  -wrench_wrt_robot(3) / K_virt_ang + decompose_rot_mat(tool_wrt_robot.linear())(0);
			bumpless_virt_attr_angles(1) =  -wrench_wrt_robot(4) / K_virt_ang + decompose_rot_mat(tool_wrt_robot.linear())(1);
			bumpless_virt_attr_angles(2) =  -wrench_wrt_robot(5) / K_virt_ang + decompose_rot_mat(tool_wrt_robot.linear())(2);
			virt_attr_rot = rotation_matrix_from_euler_angles(bumpless_virt_attr_angles);

			virtual_force.head(3) = K_virt * (virt_attr_pos - current_ee_pos.head(3));
			virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));//bumpless_virt_attr_pose.tail(3); // K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));
			cout<<"Bumpless attractor used"<<endl;
			cout<<"wrench torques"<<endl<<wrench_wrt_robot.tail(3)<<endl;
			cout<<"decomposed angles of the tool frame"<<endl<<decompose_rot_mat(tool_wrt_robot.linear())<<endl;
			cout<<"bumpless virtual attractor pose: "<<endl;
			cout<<bumpless_virt_attr_pose<<endl;

		}

		// Otherwise do what the controller usually does
		else {
			// If we have established a virtual attractor
			if(virt_attr_established) {
				virtual_force.head(3) = K_virt * (virt_attr_pos - current_ee_pos.head(3));
				virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));
				cout<<"virtual attractor pose"<<endl;
				cout<<virt_attr_pos<<endl;
				cout<<"bumpless virtual attractor pose: "<<endl;
				cout<<bumpless_virt_attr_pose<<endl;
			}
			else{
				virtual_force<<0,0,0,0,0,0; // this only prevents errors at startup, not if you cut the skill in the middle
				cout<<"No virtual attractor command received"<<endl;	
			}
		}

		cout<<"virtual force: "<<endl<<virtual_force<<endl;

		// Make temporary twist to have the head and tail be affected by the gain, B-virt for the rot and trans
		// -B_virt * des_twist converted to use two gains
		des_twist_w_gain.head(3) = -B_virt_trans * des_twist.head(3);
		des_twist_w_gain.tail(3) = -B_virt_rot * des_twist.tail(3);

		// CONTROL LAW
		des_cart_acc = inertia_mat_inv*(des_twist_w_gain + wrench_wrt_robot + virtual_force); //	used to be des_cart_acc = inertia_mat_inv*(-B_virt * des_twist + wrench_wrt_robot + virtual_force);
		if(!freeze_mode) des_twist += des_cart_acc*dt_;
		else {
			des_twist<<0,0,0,0,0,0;
			cout<<"freeze mode on"<<endl;
		}

		// This is where it is not robot agnostic, if you want to change the robot, change the jacobian accordingly
		if(!freeze_mode) des_jnt_vel = jacobian_inv*des_twist;
		else des_jnt_vel<<0,0,0,0,0,0;
									
		//ensure that desired joint vel is within set safe limits
		if(des_jnt_vel.norm() > MAX_JNT_VEL_NORM) des_jnt_vel = (des_jnt_vel / des_jnt_vel.norm()) * MAX_JNT_VEL_NORM;
		
		//euler one step integration to calculate position from velocities
		Eigen::MatrixXd des_jnt_pos = Eigen::VectorXd::Zero(6);
		if(!freeze_mode) des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		else des_jnt_pos = frozen_joint_states_;

		// put velocity and pososition commands into Jointstate message
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;

		//Publish desired jointstate
		arm_publisher.publish(desired_joint_state);
		last_desired_joint_state_ = desired_joint_state;

		cout<<"Current EE Pose:"<<endl<<current_ee_pos<<endl;

		// publish cartesian coordinates of robot end effector
		cartesian_log.pose.position.x = current_ee_pos(0);
		cartesian_log.pose.position.y = current_ee_pos(1);
		cartesian_log.pose.position.z = current_ee_pos(2);
		cartesian_log.pose.orientation.w = flange_quat.w();
		cartesian_log.pose.orientation.x = flange_quat.x();
		cartesian_log.pose.orientation.y = flange_quat.y();
		cartesian_log.pose.orientation.z = flange_quat.z();
		cartesian_log.header.stamp = ros::Time::now();
		cart_log_pub.publish(cartesian_log);

		// publish coordinates of virtual attractor
		// TODO these are cartesian as well? or in robot frame?
		virt_attr_log.pose.position.x = virt_attr_pos(0);
		virt_attr_log.pose.position.y = virt_attr_pos(1);
		virt_attr_log.pose.position.z = virt_attr_pos(2);
		virt_attr_log.pose.orientation.w = virt_quat.w();
		virt_attr_log.pose.orientation.x = virt_quat.x();
		virt_attr_log.pose.orientation.y = virt_quat.y();
		virt_attr_log.pose.orientation.z = virt_quat.z();
		virt_attr_log.header.stamp = ros::Time::now();
		virt_attr_after_tf.publish(virt_attr_log);

		//publish coordinates of bumpless virtual attractor
		bumpless_virt_attr_log.pose.position.x = bumpless_virt_attr_pose(0);
		bumpless_virt_attr_log.pose.position.y = bumpless_virt_attr_pose(1);
		bumpless_virt_attr_log.pose.position.z = bumpless_virt_attr_pose(2);
		bumpless_virt_attr_log.pose.orientation.w = 0; //TODO CHANGE CHANGE CHANGE CHANGE. THESE ARE PLACEHOLDERS.
		bumpless_virt_attr_log.pose.orientation.x = 0;
		bumpless_virt_attr_log.pose.orientation.y = 0;
		bumpless_virt_attr_log.pose.orientation.z = 0;
		bumpless_virt_attr_log.header.stamp = ros::Time::now();
		bumpless_virt_attr_after_tf.publish(bumpless_virt_attr_log);
			
		//publishing force torque values transformed into robot frame
		transformed_wrench.force.x = wrench_wrt_robot(0);
		transformed_wrench.force.y = wrench_wrt_robot(1);
		transformed_wrench.force.z = wrench_wrt_robot(2);
		transformed_wrench.torque.x = wrench_wrt_robot(3);
		transformed_wrench.torque.y = wrench_wrt_robot(4);
		transformed_wrench.torque.z = wrench_wrt_robot(5);
		ft_pub.publish(transformed_wrench);

		// NEW publish z vector
		x_vec_pub.publish(x_vec_message);
		y_vec_pub.publish(y_vec_message);
		z_vec_pub.publish(z_vec_message);

		//pulish freeze mode status
		freeze_mode_pub.publish(freeze_mode_int);

		if (freeze_mode) first_loop_after_freeze = true;
		else first_loop_after_freeze = false;

		// this ensures update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}