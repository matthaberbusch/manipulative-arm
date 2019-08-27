//10/31/2018
//Surag Balajepalli
//Scratch pad for cwru_lib_abb (for lack of a better name)
//TODO! FIX BY LOOKING AT NASA DIAGRAM
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>

//global vars for subscribers
//Eigen::VectorXd wrench_spatial_coords_;
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(15,6);
Eigen::MatrixXd joint_states_filter = Eigen::MatrixXd::Zero(15,6);

double dt_ = 0.1;
int dbg;
double MAX_JNT_VEL_NORM = 0.1;
double MAX_JNT_VEL_NORM_WRIST = 1;
bool is_nan;
double K_virt = 1000;
double _int = 0.001;
double K_lag_int = 0.1;
double X_FORCE_THRESH = 1.0;
double has_hit = 0;
double ATTRACTOR_DISTANCE = 0.02;
double theta;
double theta_two;
double dist_from_hit;
double THETA_OFFSET = M_PI /90 ;
double attractor_angle;
double r = 0.01;
double D_THETA = 0.01;
double D_R = 0.01; // 5 mm
bool snapback;
int filter_counter = 0;
int j_filter_counter = 0;

Eigen::VectorXd desired_ee_twist(6);
Eigen::VectorXd desired_twist(6);
Eigen::VectorXd desired_force(6);
Eigen::VectorXd result_force(6);
Eigen::VectorXd result_twist(6);
Eigen::VectorXd desired_twist_sensor(6);
Eigen::VectorXd desired_ee_pos(6);
Eigen::MatrixXd accomodation_gain(6,6);
Eigen::VectorXd current_ee_origin(3);
Eigen::Matrix3d current_ee_rot;
Eigen::Matrix3d desired_ee_rot;
Eigen::VectorXd hit_point(6);




Eigen::Matrix3d vectorHat(Eigen::Vector3d vector) { //for lack of a better name
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


Eigen::VectorXd pose_from_transform(Eigen::Affine3d transform) { //useless function!
	Eigen::VectorXd pose(6);
	pose.head(3) = transform.translation();
	Eigen::Vector3d z_axis;
	z_axis<<0,0,1;
	pose.tail(3) = transform.linear().transpose() * z_axis;
	return pose;
 
}


Eigen::VectorXd transform_wrench(Eigen::VectorXd wrench_wrt_a, Eigen::Affine3d b_wrt_a) { //from eqn 2.66 in MLS book
	//given tf from a to b and wrench in a frame, finds wrench in frame b
	//WRONG APPLICATION
	Eigen::VectorXd wrench_wrt_b;
	Eigen::Vector3d O_b = b_wrt_a.translation();
	Eigen::Matrix3d O_b_hat = vectorHat(O_b);
	Eigen::MatrixXd wrench_transformation_matrix = Eigen::MatrixXd::Zero(6,6);
	wrench_transformation_matrix.block<3,3>(0,0) = b_wrt_a.linear().transpose();
	wrench_transformation_matrix.block<3,3>(3,3) = b_wrt_a.linear().transpose();
	//wrench_transformation_matrix.block<3,3>(3,0) = - b_wrt_a.linear().transpose() * O_b_hat; 
	
	wrench_wrt_b = wrench_transformation_matrix * wrench_wrt_a;
	return wrench_wrt_b;
	
	
}

Eigen::VectorXd transform_twist(Eigen::VectorXd twist_b, Eigen::Affine3d transform) { //from eqn 2.57 in MLS book 
	// twist b = twist in b frame
	// transform - from a to b
	//calculates twist in frame a
	Eigen::VectorXd twist_a;
	Eigen::Vector3d origin = transform.translation();
	Eigen::Matrix3d origin_hat = vectorHat(origin);
	Eigen::Matrix3d rot_mat = transform.linear();
	Eigen::MatrixXd twist_tf_matrix = Eigen::MatrixXd::Zero(6,6);
	twist_tf_matrix.block<3,3>(0,0) = rot_mat;
	twist_tf_matrix.block<3,3>(3,3) = rot_mat;
	twist_tf_matrix.block<3,3>(0,3) = origin_hat * rot_mat;
	twist_a = twist_tf_matrix * twist_b;

	return twist_a;
}

void ftSensorCallback(const geometry_msgs::WrenchStamped& ft_sensor) {
	//subscribe to "robotiq_ft_wrench"
	//implement low pass filter
	
	wrench_body_coords_(0) = std::round(ft_sensor.wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor.wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor.wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor.wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor.wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor.wrench.torque.z * 10) / 10;
	
	if (filter_counter > 14) { //if last 15 readings are recorded
		//low pass filter
		//Block of size (p,q), starting at (i,j); so block(i,j,p,q)
		wrench_filter.block(1,0,14,6) = wrench_filter.block(0,0,14,6); //move everything down
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); // top row is the new coordinates - do I have to transpose it?

		wrench_body_coords_ = wrench_filter.colwise().sum(); // sums up the columns into an array
		wrench_body_coords_ /= 15; // now we have an average
	}
	else {
		filter_counter++;
		wrench_filter.block(1,0,14,6) = wrench_filter.block(0,0,14,6);
		ROS_INFO("HERE");
		wrench_filter.block(0,0,1,6) = wrench_body_coords_.transpose(); 
		// and then wrench_body_coords_ is just exactly what the sensor reads
	}

}

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	//subscribe to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ; //implement low pass filter- TODO
 
	// another low pass filter, enable if desired:
	/*
	if (j_filter_counter > 14) { //if last 15 readings are recorded
		//low pass filter
		//Block of size (p,q), starting at (i,j); so block(i,j,p,q)
		joint_states_filter.block(1,0,14,6) = joint_states_filter.block(0,0,14,6); //move everything down
		joint_states_filter.block(0,0,1,6) = joint_states_; // top row is the new coordinates - do I have to transpose it?

		joint_states_ = joint_states_filter.colwise().sum(); // sums up the columns into an array
		joint_states_ /= 15; // now we have an average
	}
	else {
		j_filter_counter++;
		joint_states_filter.block(1,0,14,6) = joint_states_filter.block(0,0,14,6);
		joint_states_filter.block(0,0,1,6) = joint_states_; 
		// and then wrench_body_coords_ is just exactly what the sensor reads
	}
	*/

}

Eigen::Vector3d ang_vel_from_rot_mats(Eigen::Matrix3d a_wrt_world, Eigen::Matrix3d b_wrt_world) {
	// Math:
	// From eqn 2.48 of MLS book
	//Given b_wrt_a, find ang vel from a to b
	//Here we have a_wrt_world, b_wrt_world
	//b_wrt_a is basically dR? Apparently not!
	Eigen::Vector3d ang_vel;
	Eigen::Matrix3d b_wrt_a = a_wrt_world.inverse() * b_wrt_world;
	ang_vel(0) = b_wrt_a(1,2);
	ang_vel(1) = b_wrt_a(2,0);
	ang_vel(2) = b_wrt_a(0,1);
	return ang_vel;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "control_test");
	ros::NodeHandle nh;
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<sensor_msgs::JointState>("cartesian_logger",1); //Wrong message type, but I can differentiate
	ros::Publisher virt_attr_pub = nh.advertise<geometry_msgs::PoseStamped>("virt_attr_log",1);
	ros::Publisher K_virt_pub = nh.advertise<std_msgs::Float64>("K_virt",1);
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);

	Irb120_fwd_solver irb120_fwd_solver;
	
	geometry_msgs::Wrench transformed_wrench;

	sensor_msgs::JointState desired_joint_state;
	sensor_msgs::JointState cartesian_log;
	geometry_msgs::PoseStamped virt_attr;
	virt_attr.header.frame_id = "map";
	cartesian_log.position.resize(6);
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	std_msgs::Float64 K_virt_log;
	K_virt_log.data = K_virt;
	//desired_twist<<0.1,0,-0.1,0,0,0;
	accomodation_gain<<1,0,0,0,0,0,
						0,1,0,0,0,0,
						0,0,1,0,0,0,
						0,0,0,1,0,0,
						0,0,0,0,1,0,
						0,0,0,0,0,1;

	accomodation_gain *= 0.00001;

	//where to? Make this whole thing a function
	desired_ee_pos<<0.35,0.0,0.72,1.57,0,1.57;

	//static transform for sensor
	Eigen::Affine3d sensor_wrt_flange;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;


	//static Tf for tool
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_wrt_sensor_trans;
	tool_wrt_sensor_trans<<0,0,0.05;
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	double MAX_TWIST_NORM = 0.1;

	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	ros::Rate naptime(50);
	

	while(ros::ok()) {
		//cout<<"----------------";

		ros::spinOnce();
		

		
		//initialize jacobians
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		/*
		Eigen::MatrixXd adj_inv_g(6,6);
		Eigen::Matrix3d tool_rot = tool_wrt_robot.linear();
		adj_inv_g.block<3,3>(0,0) = tool_rot.transpose();
		adj_inv_g.block<3,3>(3,3) = tool_rot.transpose();
		adj_inv_g.block<3,3>(3,0) = -1 * vectorHat(tool_rot.transpose() * tool_wrt_robot.translation()) * tool_rot.transpose();
		jacobian = adj_inv_g * jacobian; 
		*/
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);

		Eigen::MatrixXd jacobian_inv = lu_jac.inverse(); //what to do when matrix is non invertible?
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		
		
		//Current ee transform
		
		//Current ee pose
		Eigen::VectorXd current_ee_pos(6);
		current_ee_pos.head(3) = tool_wrt_robot.translation();
		current_ee_pos.tail(3) = decompose_rot_mat(tool_wrt_robot.linear()); 
		//Update and transform force sensor output
		ros::spinOnce();
		Eigen::Vector3d force_tool_frame =  wrench_body_coords_.head(3);
		
		Eigen::Vector3d moment_tool_frame =  wrench_body_coords_.tail(3);
		

		
		Eigen::VectorXd wrench_wrt_robot(6);
		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * force_tool_frame;
		wrench_wrt_robot.tail(3) = (sensor_wrt_robot.linear() * moment_tool_frame);

		//Eigen::Vector3d force_robot_frame = sensor_wrt_robot * force_tool_frame;
		//Eigen::Vector3d momemnt_robot_frame = sensor_wrt_robot * moment_tool_frame;

		//wrench_wrt_robot.head(3) =  force_robot_frame.head(3);
		//wrench_wrt_robot.tail(3) = momemnt_robot_frame.tail(3);
		//cout<<"current ee pos"<<endl<<current_ee_pos<<endl;
		//cout<<"resulting wrench"<<endl<<wrench_wrt_robot<<endl;


		//find current end effector pose
		current_ee_origin = tool_wrt_robot.translation();
		current_ee_rot = tool_wrt_robot.linear();


		//desired_twist.head(3) =  desired_ee_origin - current_ee_origin;
		//desired_twist.tail(3) = ang_vel_from_rot_mats(current_ee_rot, desired_ee_rot); 
		//desired_twist = desired_twist * K_virt; // this is wrong for now!
		
		
		//ROS_INFO("Virtual attractor X set to %f", current_ee_pos(0));
/*		
		if(abs(wrench_wrt_robot(0)) > X_FORCE_THRESH){
			if(has_hit <= 10) {
				hit_point = current_ee_pos;
				cout<<"Hit point"<<endl<<hit_point<<endl;
			//has_hit = true;
				theta = 0;
				has_hit = has_hit + 1;
				desired_ee_pos = current_ee_pos;
				}
			else {
				theta = theta + D_THETA;
				if(theta >= 2*M_PI){
					r = r + D_R;
					theta = 0;
				}
				desired_ee_pos = current_ee_pos;
				desired_ee_pos(0) = current_ee_pos(0) + 0.05;//current_ee_pos(0); 
				desired_ee_pos(1) = hit_point(1) + r*cos(theta);
				desired_ee_pos(2) = hit_point(2) + r*sin(theta);
				cout<<"desired ee pos"<<endl<<desired_ee_pos<<endl;
				}
			}
			else {
			//ROS_INFO("Force in X is %f , no problem", wrench_wrt_robot(0));
				desired_ee_pos = current_ee_pos;
				desired_ee_pos(0) += 0.05;
			}
*/				
/*				
				dist_from_hit = sqrt(pow((hit_point(1)-current_ee_pos(1)),2) + pow((hit_point(2)-current_ee_pos(2)),2)); 
				theta = atan2(dist_from_hit,ATTRACTOR_DISTANCE);
				theta_two = 90 - theta;
				attractor_angle = theta_two - THETA_OFFSET;
				desired_ee_pos = current_ee_pos;
				desired_ee_pos(0) = current_ee_pos(0); //+ 0.02; // low attraction into the surface, to keep contact
				desired_ee_pos(1) = current_ee_pos(1) + ATTRACTOR_DISTANCE*cos(attractor_angle);
				desired_ee_pos(2) = current_ee_pos(2) + ATTRACTOR_DISTANCE*sin(attractor_angle);
				*/
		/*	
		if((abs(wrench_wrt_robot(0)) > X_FORCE_THRESH)) {
		//ROS_INFO("Virtual attractor X set to %f", current_ee_pos(0));
			snapback = true;
			desired_ee_pos = current_ee_pos;
			desired_ee_pos(0) = current_ee_pos(0) + 0.02; // low attraction into the surface, to keep contact
			desired_ee_pos(2) = current_ee_pos(2) + 0.02; // high(er) attraction to a point on the surface but offset in the y axis by 5cm, a primitive search method, since we already know where the "goal" is
		}
		else if (!snapback){
			//ROS_INFO("Force in X is %f , no problem", wrench_wrt_robot(0));
				desired_ee_pos = current_ee_pos;
				desired_ee_pos(0) += 0.05;
			}
		*/





		desired_twist = K_virt * (desired_ee_pos - current_ee_pos);
		//desired_twist<<0.01,0,0,0,0,0;
		cout<<"------------"<<endl;
		cout<<current_ee_pos<<endl;
		cout<<"------------"<<endl;
		//Method 1 applies force feedback in joint space.
		//Measured force is converted to joint torques, which is converted to joint vels using acc gain
		//Desired twist is converted to joint vels using jacobian inverse
		//Summation of the two joint vels is commanded to the robot
		//Commanding twist directly for now
		//Eigen::VectorXd des_jnt_vel = jacobian_inv * K_lag_int * desired_twist  - accomodation_gain * jacobian_transpose * wrench_wrt_robot;
		//Method 1 ends
		//Method 1 notes: 
		
	
		//Method 1.5
		Eigen::VectorXd result_twist =   accomodation_gain * (desired_force + wrench_wrt_robot);
		
		//twist into joint vels
		if(result_twist.norm() > MAX_TWIST_NORM) result_twist = (result_twist / result_twist.norm()) * MAX_TWIST_NORM;


		Eigen::VectorXd des_jnt_vel = jacobian_inv * result_twist;
		//cout<<"resulting twist is"<<endl<<result_twist<<endl;
		//Eigen::VectorXd des_jnt_vel = jacobian_inv * desired_twist;
	

		/*
		//Method 2 - from admblock_velocity_servo.pptx

		//desired_force = K_virt * (desired_ee_pos - current_ee_pos); //Not in use right now
		
		desired_force = K_virt * desired_twist;
		
		//F = F_d - F_sensor (+ F_bias - not implemented)
		result_force = desired_force - 2.0 * wrench_wrt_robot;
		cout<<"result_force"<<endl<<result_force<<endl;

		//Lag int term - Supposed to be 1/(Ms + B) - Is a constant for now;
		result_twist = _int * result_force;
		//result_twist<<0.1,0,0,0,0,0;
		cout<<"result_twist"<<endl<<result_twist<<endl;


		//Impedance compensator, implement it when you figure it out


		//Use jacobian inverse to convert twist to joint vel cmd
		Eigen::VectorXd des_jnt_vel = jacobian_inv * result_twist;
		*/


		//Method for retracting virtual attractor to surface
		/*
		
				*/

		//Method for a calculation intensive spiral search, CCW
		//if(abs(wrench_wrt_robot(0)) > 0) {

		
		//clip vel command  and remove nan that might have made their way through jacobian inverse
		//nan in jnt vel means Jacobian is losing rank - Fix 1: Stop moving - Make vels 0;
		//cout<<"--------";

		for(int i = 0; i < 6; i++) { 
			if(isnan(des_jnt_vel(i))) {
				
				//ROS_WARN("At SINGU");
				des_jnt_vel<<0,0,0,0,0,0;
				
			}
			
		}

		//prevents from going too fast
		if(des_jnt_vel.head(3).norm() > MAX_JNT_VEL_NORM) des_jnt_vel.head(3) = (des_jnt_vel.head(3) / des_jnt_vel.head(3).norm()) * MAX_JNT_VEL_NORM;
		if(des_jnt_vel.tail(3).norm() > MAX_JNT_VEL_NORM_WRIST) des_jnt_vel.tail(3) = (des_jnt_vel.tail(3) / des_jnt_vel.tail(3).norm()) * MAX_JNT_VEL_NORM_WRIST;

		
		Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		//des_jnt_pos<<0,0,0,0,0,0; //DEBUG ONLY
		
		//stuff it into Jointstate message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;
		arm_publisher.publish(desired_joint_state);
		for(int i = 0; i < 6; i++) cartesian_log.position[i] = std::round(current_ee_pos(i) * 1000) /1000;
		cart_log_pub.publish(cartesian_log);
		
		//Populating this message for logging. 
		virt_attr.header.stamp = ros::Time::now();
		virt_attr.pose.position.x = desired_ee_pos(0);
		virt_attr.pose.position.y = desired_ee_pos(1);
		virt_attr.pose.position.z = desired_ee_pos(2);

		transformed_wrench.force.x = wrench_wrt_robot(0);
		transformed_wrench.force.y = wrench_wrt_robot(1);
		transformed_wrench.force.z = wrench_wrt_robot(2);
		transformed_wrench.torque.x = wrench_wrt_robot(3);
		transformed_wrench.torque.y = wrench_wrt_robot(4);
		transformed_wrench.torque.z = wrench_wrt_robot(5);
		ft_pub.publish(transformed_wrench);
		virt_attr_pub.publish(virt_attr);
		K_virt_pub.publish(K_virt_log);
		naptime.sleep();
		ros::spinOnce();
		

	}
}