//8/20/19
//Surag Balajepalli and Matthew Haberbusch
//subscribers to a 'virtual attractor' pose
//Performs accommodation control 

/*
In this program we
1) include libraries
2) declare varialbes
3) define useful linear algebra functions
4) define callback functions
5) create nodes that send out info (publishers)
6) create nodes that receive info (subscribers)
7) define jacobians and gains
8) receive messages
9) transform positions and forces
10) carry out control law
11) and publish commands
TODO did I miss anything?
*/

#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
int dbg; // TODO not used

// filter variables 
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;

// global variables for subscribers
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd virt_attr_pos(3);
Eigen::Matrix3d virt_attr_rot;
Eigen::MatrixXd accomodation_gain(6,6);
bool cmd = false; // TODO this is spaghetti. Fix it!
bool jnt_state_update = false;
Eigen::Quaterniond virt_quat;
double SMALL_ANGLE_THRES = 0.5;
geometry_msgs::Vector3 x_vec_message;
geometry_msgs::Vector3 y_vec_message;
geometry_msgs::Vector3 z_vec_message;

// linear algebra functions
Eigen::Matrix3d vectorHat(Eigen::Vector3d vector) {
	// calculates vectorHat
	Eigen::Matrix3d hat_of_vector;
	hat_of_vector(0,0) = 0;
	hat_of_vector(0,1) = - vector(2);
	hat_of_vector(0,2) = vector(1);
	hat_of_vector(1,0) = vector(2);
	hat_of_vector(1,1) = 0;
	hat_of_vector(1,2) = - vector(0);
	hat_of_vector(2,0) = - vector(1);
	hat_of_vector(2,1) = vector(0);
	hat_of_vector(2,2) = 0;
	return hat_of_vector;
	}

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
	// you may remove this if you'd like
	/* removed 6/26/19
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
	cmd = true;
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


// TODO remove this callback?
void acc_gain_Cb(const std_msgs::Float64MultiArray& acc_gain_diag) {
	for(int i = 0; i<6; i++) {
		accomodation_gain(i,i) = acc_gain_diag.data[i];

	}
		accomodation_gain *= 0.0001;
		//cout<<"acc gain"<<accomodation_gain<<endl;
}

// TODO remove this callback? Do we have/need a copy somewhere else?
/*
Eigen::Vector3d delta_phi_from_rots(Eigen::Matrix3d source_rot, Eigen::Matrix3d dest_rot) {
	//Takes the source and destination orientations as rotation matrices
	//Computes angle of rotation required in each of X, Y, and Z axes to reconcile these rotations
	//Important: Useful for small angle approximation only

	//This is to ensure that it is 

	Eigen::Vector3d delta_phi;
	Eigen::Matrix3d delta_rot = source_rot.inverse() * dest_rot;
	Eigen::AngleAxisd ang_ax(delta_rot);
	if (ang_ax.angle() > SMALL_ANGLE_THRES) {
		float d_ang = ang_ax.angle() / 10;
		Eigen::AngleAxisd delta_ang_ax(d_ang, ang_ax.axis());
		delta_rot = delta_ang_ax.toRotationMatrix();
	}
	delta_phi(0) = (delta_rot(2,1) - delta_rot(1,2)) / 2;
	delta_phi(1) = (delta_rot(0,2) - delta_rot(2,0)) / 2;
	delta_phi(2) = (delta_rot(1,0) - delta_rot(0,1)) / 2;
	return delta_phi;
}
*/


// main program
int main(int argc, char **argv) {
	// just ROS things
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;
	// subscribers
	//ros::Subscriber virt_attr_sub = nh.subscribe("virt_attr",1,virt_attr_CB);
	ros::Subscriber virt_attr_sub = nh.subscribe("Virt_attr_pose",1,virt_attr_CB);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	//ros::Subscriber Ka_sub = nh.subscribe("Ka_diagonal",1, acc_gain_Cb);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	// publishers
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); 
	ros::Publisher rcc_pub = nh.advertise<geometry_msgs::PoseStamped>("remote_center_of_compliance",1);  // NEW
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	ros::Publisher virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1);
	ros::Publisher x_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_x",1);
	ros::Publisher y_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_y",1);
	ros::Publisher z_vec_pub = nh.advertise<geometry_msgs::Vector3>("tool_vector_z",1);


	// TODO what is this?
	// Instantiate an object of the custom FK solver class for the ABB IRB 120 robot
	Irb120_fwd_solver irb120_fwd_solver;
	// declare matricies and vectors
	Eigen::MatrixXd robot_inertia_matrix(6,6);
	Eigen::VectorXd current_ee_pos(6);
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd movements_due_to_wrench(6); //NEW 
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd result_twist(6);
	Eigen::VectorXd des_jnt_vel = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd des_cart_acc(6);
	Eigen::VectorXd des_twist = Eigen::VectorXd::Zero(6);
	// declare constants
	double dt_ = 0.01;
	double MAX_JNT_VEL_NORM = 10;
	double MAX_TWIST_NORM = 0.1;
	// declare gains
	double B_virt = 4000;
	double K_virt = 1000; 
	double K_virt_ang = 1000;

	//NEW Bumpless transition
	double timer_counter = 0;
	bool latch = false;
	Eigen::VectorXd bumpless_virt_attr_pose(6);
	//Eigen::Vector3d delta_phis_from_rots(3);
    //Eigen::Matrix3d x_rotation;
    //Eigen::Matrix3d y_rotation;
    //Eigen::Matrix3d z_rotation;
	//double bumpless_transition_gain = 0;


	// NEW
	double alpha_x, alpha_y, a_x, a_y, b_x, b_y;
	double l_rcc = 0.3;

	// NEW RCC stuff
	Eigen::Vector3d remote_center_of_compliance(3);
	remote_center_of_compliance<<0.4605 + 0.24,0.0070,0.7150; //MAKE SURE RCC IS ADJUSTED IN irb120_kinematics.h, now it's 20 cm below
	Eigen::Vector3d r_vector(3);
	Eigen::Vector3d x_force(3);
	Eigen::Vector3d y_force(3);
	Eigen::Vector3d z_force(3);
	Eigen::Vector3d moment_due_to_fx(3);
	Eigen::Vector3d moment_due_to_fy(3);	
	Eigen::Vector3d moment_due_to_fz(3);		
	Eigen::Vector3d moment_due_to_tx(3);
	Eigen::Vector3d moment_due_to_ty(3);
	Eigen::Vector3d moment_due_to_tz(3);
	Eigen::Vector3d moments_around_rcc(3);
	Eigen::Vector3d rcc_omega_desired(3);
	Eigen::VectorXd rcc_twist_desired(6);
	double rcc_moment_gain = -20;
	Eigen::VectorXd x_force_only_wrench_wrt_robot(6);
	
	// ROS requirement - messages must be in a certain format
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virt_attr_log, rcc; // NEW rcc
	cartesian_log.header.frame_id = "map";
	virt_attr_log.header.frame_id = "map";
	rcc.header.frame_id = "map";
	geometry_msgs::Pose virt_attr;
	geometry_msgs::Wrench transformed_wrench;
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);

	// ROS requirement latched joint state
	sensor_msgs::JointState latched_joint_state;
	latched_joint_state.position.resize(6);
	latched_joint_state.velocity.resize(6);	
	
	// // define default values for accommodation gain
	// accomodation_gain<<1,0,0,0,0,0,
	// 					0,1,0,0,0,0,
	// 					0,0,1,0,0,0,
	// 					0,0,0,1,0,0,
	// 					0,0,0,0,1,0,
	// 					0,0,0,0,0,1;
	// accomodation_gain *= 0.0001;

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

	// // begin tool description
	// //TODO Think of another way to make this happen
	// double tool_mass = 0.5;
	// double tool_length = 0.1;
	// Eigen::Vector3d tool_length_vector;
	// tool_length_vector<<0,0,tool_length; //For easier math, length vector described in tool frame
	// Eigen::Vector3d f_g_r; //gravity vector in robot base frame
	// f_g_r<<tool_mass*9.8,0,0; //gravity in tool frame, to be computer for every new joint state
	// Eigen::Vector3d f_g_t; //TODO what is this?
	// Eigen::VectorXd f_comp(6); // compensation wrench 
	// // end tool description
	
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

	// NEW RCC transform
	Eigen::Affine3d rcc_wrt_tool;
	Eigen::Matrix3d rcc_wrt_tool_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d rcc_wrt_tool_trans;
	rcc_wrt_tool_trans<<0,0,0.01; // TODO change
	rcc_wrt_tool.linear() = rcc_wrt_tool_rot;
	rcc_wrt_tool.translation() = rcc_wrt_tool_trans;
	
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

	// NEW Initialize sensor position
	Eigen::Vector3d sensor_position = Eigen::Vector3d::Zero(3); 
	sensor_position = sensor_wrt_robot.translation();

	// NEW Initialize RCC position
	Eigen::Affine3d rcc_wrt_robot = tool_wrt_robot * rcc_wrt_tool;
	Eigen::VectorXd initial_rcc_pos = Eigen::VectorXd::Zero(6); 
	remote_center_of_compliance = rcc_wrt_robot.translation();
	initial_rcc_pos.head(3) = rcc_wrt_robot.translation();
	initial_rcc_pos.tail(3) = decompose_rot_mat(rcc_wrt_robot.linear());

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

	// rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// main loop
	while(ros::ok()) {
		// subscribers get info
		ros::spinOnce();

			// initialize jacobians
			Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
			flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
			sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
			tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
			rcc_wrt_robot = tool_wrt_robot * rcc_wrt_tool; //NEW
			Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
			if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
			Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); // TODO what to do when matrix is non invertible?   
			Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
			
			// find current end effector pose
			current_ee_pos.head(3) = tool_wrt_robot.translation();
			current_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear()); 
			Eigen::Quaterniond flange_quat(tool_wrt_robot.linear());
			
			// NEW find current sensor pose
			sensor_position = sensor_wrt_robot.translation();

			// NEW find current RCC pose
			remote_center_of_compliance = rcc_wrt_robot.translation();
			//older things...
			initial_rcc_pos.head(3) = rcc_wrt_robot.translation();
			initial_rcc_pos.tail(3) = decompose_rot_mat(rcc_wrt_robot.linear());
			Eigen::Quaterniond rcc_quaternion(rcc_wrt_robot.linear());

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
			
			//Update and transform force sensor output
			// TODO delete above comment? was there something here before?
			
			//to find gravity compensation force - make this more reusable
			//rigid body force vector transformations
			// f_g_t = tool_wrt_robot.linear().transpose() * f_g_r;
			// f_comp.head(3) = f_g_t;
			// f_comp.tail(3) = tool_length_vector.cross(f_g_t);
			
			//compensate for these forces
			// TODO describe what the .head and .tail function do
			Eigen::Vector3d force_tool_frame =  wrench_body_coords_.head(3); //- f_comp.head(3); //TODO remove these comments
			Eigen::Vector3d moment_tool_frame =  wrench_body_coords_.tail(3); //- f_comp.tail(3);
					
			wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.head(3)); // - f_comp.head(3);
			wrench_wrt_robot.tail(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.tail(3)); // - f_comp.tail(3);

			// NEW Virtual attractor pose based on force felt
			bumpless_virt_attr_pose.head(3) = -wrench_wrt_robot.head(3) / K_virt + current_ee_pos.head(3);

			/*
			delta_phis_from_rots = wrench_wrt_robot.tail(3) / K_virt_ang;
			z_rotation(0,0) = cos(delta_phis_from_rots(2));
    		z_rotation(0,1) = -sin(delta_phis_from_rots(2));
    		z_rotation(0,2) = 0;
    		z_rotation(1,0) = sin(delta_phis_from_rots(2));
    		z_rotation(1,1) = cos(delta_phis_from_rots(2));
    		z_rotation(1,2) = 0;
    		z_rotation(2,0) = 0;
    		z_rotation(2,1) = 0;
    		z_rotation(2,2) = 1;

    		y_rotation(0,0) = cos(delta_phis_from_rots(1));
    		y_rotation(0,1) = 0;
    		y_rotation(0,2) = sin(delta_phis_from_rots(1));
    		y_rotation(1,0) = 0;
    		y_rotation(1,1) = 1;
    		y_rotation(1,2) = 0;
    		y_rotation(2,0) = -sin(delta_phis_from_rots(1));
    		y_rotation(2,1) = 0;
    		y_rotation(2,2) = cos(delta_phis_from_rots(1));

    		x_rotation(0,0) = 1;
    		x_rotation(0,1) = 0;
    		x_rotation(0,2) = 0;
    		x_rotation(1,0) = 0;
    		x_rotation(1,1) = cos(delta_phis_from_rots(0));
    		x_rotation(1,2) = -sin(delta_phis_from_rots(0));
    		x_rotation(2,0) = 0;
    		x_rotation(2,1) = sin(delta_phis_from_rots(0));
    		x_rotation(2,2) = cos(delta_phis_from_rots(0));
    		*/
    		// TODO check if this is the right way to do this rotation. I might need to make a rotation matrix from angles a different way
			bumpless_virt_attr_pose(3) = 0; // = current_ee_pos.tail(3) * x_rotation * y_rotation * z_rotation;
			bumpless_virt_attr_pose(4) = 0;
			bumpless_virt_attr_pose(5) = 0;
			//virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));


			// NEW RCC math
			r_vector = remote_center_of_compliance - sensor_position; //current_ee_pos.head(3);
			x_force<<wrench_wrt_robot(0),0,0;
			y_force<<0,wrench_wrt_robot(1),0;
			z_force<<0,0,wrench_wrt_robot(2);
			moment_due_to_fx = r_vector.cross(x_force);
			moment_due_to_fy = r_vector.cross(y_force);
			moment_due_to_fz = r_vector.cross(z_force);
			moment_due_to_tx<<wrench_wrt_robot(3),0,0;
			moment_due_to_ty<<0,wrench_wrt_robot(4),0;
			moment_due_to_tz<<0,0,wrench_wrt_robot(5);
			// NEW RCC add all the momements up
			moments_around_rcc =  moment_due_to_fx + moment_due_to_fy + moment_due_to_fz + moment_due_to_tx + moment_due_to_ty + moment_due_to_tz;
			//cout<<"Total moment around RCC: "<<endl<<moments_around_rcc<<endl;
			// NEW RCC Calculate omega
			rcc_omega_desired = rcc_moment_gain * moments_around_rcc;
			rcc_twist_desired(0) = 0;
			rcc_twist_desired(1) = 0;
			rcc_twist_desired(2) = 0;
			rcc_twist_desired.tail(3) = rcc_omega_desired;
			cout<<"rcc_twist_desired: "<<endl<<rcc_twist_desired<<endl;

			//NEW remote center of compliance DOES NOT WORK... YET
			alpha_x = wrench_wrt_robot(3);
			alpha_y = wrench_wrt_robot(4);

			a_x = l_rcc * sin(alpha_x);
			a_y = l_rcc * sin(alpha_y);
			b_x = l_rcc * cos(alpha_x);
			b_y = l_rcc * cos(alpha_y);

			movements_due_to_wrench(0) = wrench_wrt_robot(0) + a_y;
			movements_due_to_wrench(1) = wrench_wrt_robot(1) - a_x;
			movements_due_to_wrench(2) = wrench_wrt_robot(2) - (l_rcc - b_x) - (l_rcc - b_y);
			movements_due_to_wrench(3) = wrench_wrt_robot(3);
			movements_due_to_wrench(4) = wrench_wrt_robot(4);
			movements_due_to_wrench(5) = wrench_wrt_robot(5);
					
			// Compute virtual attractor forces
			// If the controller has just started, use the bumpless virtual attractor
			if (timer_counter < 10 ) {

				virt_attr_pos(0) = bumpless_virt_attr_pose(0);
				virt_attr_pos(1) = bumpless_virt_attr_pose(1);
				virt_attr_pos(2) = bumpless_virt_attr_pose(2);
				virt_attr_rot(0) = 0;
				virt_attr_rot(1) = 0;
				virt_attr_rot(2) = 0;

				virtual_force.head(3) = K_virt * (virt_attr_pos - current_ee_pos.head(3));
				virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));
			cout<<"Bumpless attractor used"<<endl;

				//if(!cmd) virtual_force<<0,0,0,0,0,0; I don't think we need this
			}
			// Otherwise do what the controller usually does
			else {
				virtual_force.head(3) = K_virt * (virt_attr_pos - current_ee_pos.head(3));
				virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(tool_wrt_robot.linear(), virt_attr_rot));
				if(!cmd) {
					virtual_force<<0,0,0,0,0,0; // this only prevents errors at startup, not if you cut the skill in the middle
					cout<<"No virtual attractor command received"<<endl;
				}
			}


			// TODO remove this 
			//Representing rotations with 3 vals loses info
			//Instead, 
			//Find delta phi_x, phi_y, and phi_z  from source and destination rotation
			//Multiply with stiffness 
			
			//control law 1. Simple accommodation again
			/*
			Eigen::VectorXd result_twist =   accomodation_gain * (virtual_force + wrench_wrt_robot);
			if(result_twist.norm() > MAX_TWIST_NORM) result_twist = (result_twist / result_twist.norm()) * MAX_TWIST_NORM;
			des_jnt_vel = jacobian_inv * result_twist;
			//clip vel command  and remove nan that might have made their way through jacobian inverse
			//nan in jnt vel means Jacobian is losing rank - Fix 1: Stop moving - Make vels 0;
			for(int i = 0; i < 6; i++) { if(isnan(des_jnt_vel(i))) des_jnt_vel<<0,0,0,0,0,0; }
				//if at singularity - just dont move. Redundant test
				//FullPivLu decomposition always provides an inverse - I think?
			*/
			//virtual_force<<10,0,0,0,0,0;
			cout<<"Virtual force"<<endl<<virtual_force<<endl;
			//wrench_wrt_robot<<0,0,0,0,0,0;	
			//control law 2: NASA compliance controller
			/*
			Eigen::VectorXd feed_forward_vel = Eigen::VectorXd::Zero(6);
			ROS_INFO("here");
			feed_forward_vel(0) = 0.0001;
			Eigen::VectorXd feed_forward_joint_vel;
			feed_forward_joint_vel = jacobian_inv * feed_forward_vel; 
			*/

			//NEW only consider the x force in compliant motion for RCC
			x_force_only_wrench_wrt_robot<<wrench_wrt_robot(0),0,0,0,0,0;

			//NEW bumpless transition gain
			//if(bumpless_transition_gain < 1){
			//	bumpless_transition_gain = bumpless_transition_gain + dt_ / 2; // So 2 seconds until gain is 1
			//}

			// CONTROL LAW
			des_cart_acc = inertia_mat_inv*(-B_virt * des_twist + wrench_wrt_robot + virtual_force); //	used to be des_cart_acc = inertia_mat_inv*(-B_virt * des_twist + wrench_wrt_robot + virtual_force);
			//des_cart_acc = inertia_mat_inv*(-B_virt * des_twist + x_force_only_wrench_wrt_robot + virtual_force + rcc_twist_desired); //NEW FOR REMOTE CENTER OF COMPLIANCE
			des_twist += des_cart_acc*dt_;
			des_jnt_vel = jacobian_inv*des_twist;


			//old old law used to be des_jnt_vel = des_jnt_vel + (robot_inertia_matrix.inverse()*(-B_virt*des_jnt_vel + jacobian_inv*(virtual_force + wrench_wrt_robot)))*dt_;
							
			//ensure that desired joint vel is within set safe limits
			if(des_jnt_vel.norm() > MAX_JNT_VEL_NORM) des_jnt_vel = (des_jnt_vel / des_jnt_vel.norm()) * MAX_JNT_VEL_NORM;
			cout<<"des_jnt_vel"<<endl<<des_jnt_vel<<endl;
			
			//euler one step integration to calculate position from velocities
			Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
			
			// put velocity and pososition commands into Jointstate message
			for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
			for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;

			//Publish desired jointstate
			arm_publisher.publish(desired_joint_state);

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

			// NEW publish set vitr attr coordinates 
			rcc.pose.position.x = current_ee_pos(0);
			rcc.pose.position.y = current_ee_pos(1);
			rcc.pose.position.z = current_ee_pos(2);
			rcc.pose.orientation.w = rcc_quaternion.w();
			rcc.pose.orientation.x = rcc_quaternion.x();
			rcc.pose.orientation.y = rcc_quaternion.y();
			rcc.pose.orientation.z = rcc_quaternion.z();
			rcc.header.stamp = ros::Time::now();
			rcc_pub.publish(rcc);

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

		
		timer_counter = timer_counter + 1;

		// this ensures update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}


