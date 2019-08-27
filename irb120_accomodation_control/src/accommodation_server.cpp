//NEEDS MORE WORK. TO DO:
// Figure out where the movement of the virtual attractor should happen. In this node?
//Test with custom server



#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <irb120_accomodation_control/AccommodationAction.h>

class AccommodationActionServer
{
protected: 
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<irb120_accomodation_control::AccommodationAction> as_;
	irb120_accomodation_control::AccommodationGoal goal_;
	irb120_accomodation_control::AccommodationFeedback fdbk_;
	irb120_accomodation_control::AccommodationResult result_;
	Eigen::VectorXd desired_ee_twist;
	Eigen::VectorXd desired_twist;
	
	
	Eigen::VectorXd result_twist;
	
	Eigen::VectorXd desired_ee_pos;
	Eigen::MatrixXd accomodation_gain;
	Eigen::VectorXd current_ee_origin;
	Eigen::Matrix3d current_ee_rot;
	Eigen::Matrix3d desired_ee_rot;

	sensor_msgs::JointState desired_joint_state;
	sensor_msgs::JointState cartesian_log;
	Eigen::Affine3d sensor_wrt_flange;
	Eigen::Matrix3d sensor_rot;
	Eigen::Vector3d sensor_origin;
	
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot;
	Eigen::Vector3d tool_wrt_sensor_trans;
	Irb120_fwd_solver irb120_fwd_solver;
	Eigen::VectorXd joint_states_;
	Eigen::VectorXd wrench_body_coords_;
	double dt_;
	int dbg;
	double MAX_JNT_VEL_NORM;
	double MAX_JNT_VEL_NORM_WRIST;
	bool is_nan;
	double K_virt;
	double _int;
	double K_lag_int;
	ros::Subscriber joint_state_sub;
	ros::Subscriber ft_sub;
	ros::Publisher arm_publisher;
	ros::Publisher log_pub;
	double MIN_POSE_DEVIATION_NORM;
public:
	AccommodationActionServer():as_(nh_,"accommodation_action", boost::bind(&AccommodationActionServer::executeCB,this,_1),false) {
	//what do i need to initialize? Almost everything!
	joint_state_sub = nh_.subscribe("abb120_joint_state",1,&AccommodationActionServer::jointStateCallback,this);
	ft_sub = nh_.subscribe("robotiq_ft_wrench", 1,&AccommodationActionServer::ftSensorCallback,this);
	arm_publisher = nh_.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	log_pub = nh_.advertise<sensor_msgs::JointState>("cartesian_logger",1); //Wrong message type, but I can differentiate
	
	desired_ee_twist = Eigen::VectorXd::Zero(6);
	desired_twist = Eigen::VectorXd::Zero(6);
	result_twist = Eigen::VectorXd::Zero(6);
	desired_ee_pos =  Eigen::VectorXd::Zero(6);
	accomodation_gain = Eigen::MatrixXd::Zero(6,6);
	current_ee_origin = Eigen::VectorXd::Zero(6);
	tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	joint_states_ = Eigen::VectorXd::Zero(6);
	wrench_body_coords_ = Eigen::VectorXd::Zero(6);
	cartesian_log.position.resize(6);
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
	//desired_twist<<0.1,0,-0.1,0,0,0;

	dt_ = 0.1;
	MAX_JNT_VEL_NORM = 0.1;
	MAX_JNT_VEL_NORM_WRIST = 1;
	K_virt = 0.5;
	_int = 0.001;
	K_lag_int = 0.1;
	MIN_POSE_DEVIATION_NORM = 0.001;
	accomodation_gain<<1,0,0,0,0,0,
						0,1,0,0,0,0,
						0,0,1,0,0,0,
						0,0,0,50,0,0,
						0,0,0,0,50,0,
						0,0,0,0,0,50;

	accomodation_gain *= -0.0005;

	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;


	//static Tf for tool
	tool_wrt_sensor_trans<<0,0,0.05;
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	
	//ros::Rate naptime(50);
	

	as_.start();
	}

	~AccommodationActionServer(void) {

	}
	
	
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

	Eigen::Vector3d ang_vel_from_rot_mats(Eigen::Matrix3d a_wrt_world, Eigen::Matrix3d b_wrt_world) {
	// Math:
	// From eqn 2.48 of MLS book
	//Given b_wrt_a, find ang vel from a to b
	//Here we have a_wrt_world, b_wrt_world
	//b_wrt_a is basical dR? Apparently not!
	Eigen::Vector3d ang_vel;
	Eigen::Matrix3d b_wrt_a = a_wrt_world.inverse() * b_wrt_world;
	ang_vel(0) = b_wrt_a(1,2);
	ang_vel(1) = b_wrt_a(2,0);
	ang_vel(2) = b_wrt_a(0,1);
	return ang_vel;
	}

	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state) {
	//subscribe to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state->position[i] ; //implement low pass filter- TODO
	}


	void ftSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& ft_sensor) {
	//subscribe to "robotiq_ft_wrench"
	//implement low pass filter - TODO
	
	
	wrench_body_coords_(0) = std::round(ft_sensor->wrench.force.x * 10) / 10;
	wrench_body_coords_(1) = std::round(ft_sensor->wrench.force.y * 10) / 10;
	wrench_body_coords_(2) = std::round(ft_sensor->wrench.force.z * 10) / 10;
	wrench_body_coords_(3) = std::round(ft_sensor->wrench.torque.x * 10) / 10;
	wrench_body_coords_(4) = std::round(ft_sensor->wrench.torque.y * 10) / 10;
	wrench_body_coords_(5) = std::round(ft_sensor->wrench.torque.z * 10) / 10;
	}

	Eigen::VectorXd populate_vector_from_pose(geometry_msgs::Pose pose) {
		Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(6);
		des_pos(0) = pose.position.x;
		des_pos(1) = pose.position.y;
		des_pos(2) = pose.position.z;
		//method to convert quaternion orientation to rotation matrix
		float a = pose.orientation.w;
		float b = pose.orientation.x;
		float c = pose.orientation.y;
		float d = pose.orientation.z;
		
		//seperately filling out the 3x3 rotation matrix
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix(0,0) = pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2);
		rotation_matrix(0,1) = 2 * (b * c - a * d);
		rotation_matrix(0,2) = 2 * (b * d + a * c);
		rotation_matrix(1,0) = 2 * (b * c + a * d);
		rotation_matrix(1,1) = pow(a,2) - pow(b,2) + pow(c,2) - pow(d,2);
		rotation_matrix(1,2) = 2 * (c * d - a * b);
		rotation_matrix(2,0) = 2 * (b * d - a * c);
		rotation_matrix(2,1) = 2 * (c * d + a * b);
		rotation_matrix(2,2) = pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2); 

		Eigen::Vector3d angles = decompose_rot_mat(rotation_matrix);
		des_pos.tail(3) = angles;

	}

	void executeCB(const actionlib::SimpleActionServer<irb120_accomodation_control::AccommodationAction>::GoalConstPtr& goal) {
	//break down goal into smaller goals
	//create loop that checks force values and changes the postion of the virtual attractor accordingly
		//desired_ee_pos = goal->des_pose;
		desired_ee_pos = populate_vector_from_pose(goal->des_pose);
	//method to break goal down into a trajectory?
		while(!as_.isPreemptRequested()) {
			ros::spinOnce();

			
		//initialize jacobians
			Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
			Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
			Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
			Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
			
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
			if((desired_ee_pos - current_ee_pos).norm() < MIN_POSE_DEVIATION_NORM) {
				result_.done = true;
				as_.setSucceeded(result_);
				return;
			}

			Eigen::Vector3d force_tool_frame =  wrench_body_coords_.head(3);
			
			Eigen::Vector3d moment_tool_frame =  wrench_body_coords_.tail(3);
			

			
			Eigen::VectorXd wrench_wrt_robot(6);
			wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * force_tool_frame;
			wrench_wrt_robot.tail(3) = (sensor_wrt_robot.linear() * moment_tool_frame);

			current_ee_origin = tool_wrt_robot.translation();
			current_ee_rot = tool_wrt_robot.linear();

			desired_twist = K_virt * (desired_ee_pos - current_ee_pos);
			Eigen::VectorXd result_twist =  K_lag_int * desired_twist - accomodation_gain * wrench_wrt_robot;
			Eigen::VectorXd des_jnt_vel = jacobian_inv * result_twist;
			cout<<"resulting twist is"<<endl<<result_twist<<endl;

			//clip vel command  and remove nan that might have made their way through jacobian inverse
			//nan in jnt vel means Jacobian is losing rank - Fix 1: Stop moving - Make vels 0;
			for(int i = 0; i < 6; i++) { 
				if(isnan(des_jnt_vel(i))) {
					
					//ROS_WARN("At SINGU");
					des_jnt_vel<<0,0,0,0,0,0;
					
				}
				
			}

			if(des_jnt_vel.head(3).norm() > MAX_JNT_VEL_NORM) des_jnt_vel.head(3) = (des_jnt_vel.head(3) / des_jnt_vel.head(3).norm()) * MAX_JNT_VEL_NORM;
			if(des_jnt_vel.tail(3).norm() > MAX_JNT_VEL_NORM_WRIST) des_jnt_vel.tail(3) = (des_jnt_vel.tail(3) / des_jnt_vel.tail(3).norm()) * MAX_JNT_VEL_NORM_WRIST;

			
			Eigen::MatrixXd des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);

			//stuff it into Jointstate message and publish
			for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
			for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;
			arm_publisher.publish(desired_joint_state);
			for(int i = 0; i < 6; i++) cartesian_log.position[i] = std::round(current_ee_pos(i) * 1000) /1000;
			log_pub.publish(cartesian_log);
			//need method to populate geometry msgs pose with current pose and publish feedback. TODO

		}

	}
	
};




int main(int argc,char** argv){
	ros::init(argc,argv,"accommodation_as");
	AccommodationActionServer accommodation_as_object;
	ros::spin();
	//return 0;
}