//6/2/2018
//Surag Balajepalli
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
#include <std_msgs/Float64.h>

// TODO: Make the filter loop based
// filter variables 
Eigen::MatrixXd wrench_filter = Eigen::MatrixXd::Zero(10,6);
int filter_counter = 0;

// global variables for subscribers
Eigen::VectorXd wrench_body_coords_ = Eigen::VectorXd::Zero(6);
Eigen::VectorXd joint_states_ = Eigen::VectorXd::Zero(6);
bool jnt_state_update_ = false;




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
	// low pass filter ends here

}

void jointStateCallback(const sensor_msgs::JointState& joint_state) {
	jnt_state_update_ = true;
	//subscribed to "abb120_joint_state"
	for(int i = 0; i < 6; i++) joint_states_(i) = joint_state.position[i] ;

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


// main program
int main(int argc, char **argv) {
	// just ROS things
	ros::init(argc, argv, "acc_controller");
	ros::NodeHandle nh;
	// subscribers
	
	ros::Subscriber ft_sub = nh.subscribe("robotiq_ft_wrench", 1, ftSensorCallback);
	//ros::Subscriber Ka_sub = nh.subscribe("Ka_diagonal",1, acc_gain_Cb);
	ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);
	// publishers
	ros::Publisher arm_publisher = nh.advertise<sensor_msgs::JointState>("abb120_joint_angle_command",1);
	ros::Publisher cart_log_pub = nh.advertise<geometry_msgs::PoseStamped>("cartesian_logger",1); 
	ros::Publisher ft_pub = nh.advertise<geometry_msgs::Wrench>("transformed_ft_wrench",1);
	ros::Publisher virt_attr_after_tf = nh.advertise<geometry_msgs::PoseStamped>("tfd_virt_attr",1);

	// Instantiate an object of the custom FK solver class for the ABB IRB 120 robot
	Irb120_fwd_solver irb120_fwd_solver;
	// declare matricies and vectors
	Eigen::MatrixXd robot_inertia_matrix(6,6);
	Eigen::VectorXd current_ee_pos(3);
	Eigen::MatrixXd current_ee_rot;
	Eigen::VectorXd wrench_wrt_robot(6);
	Eigen::VectorXd virtual_force(6);
	Eigen::VectorXd result_twist(6);
	Eigen::VectorXd des_jnt_vel = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd des_cart_acc(6);
	Eigen::VectorXd des_twist = Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd des_jnt_pos = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd virt_attr_pos(3);
	Eigen::Matrix3d virt_attr_rot;
	// declare constants
	double dt_ = 0.01;
	double MAX_JNT_VEL_NORM = 10;
	double MAX_TWIST_NORM = 0.1;
	double VIRT_ATTR_DIST_MOVE = 0.2, VIRT_ATTR_DIST_CONTACT = 0.1,VIRT_ATTR_ROT_MOVE = 0.8;
    double F_X_THRESHOLD_CONTACT = 1, T_X_THRESHOLD_ROTATE = 0.2;
    Eigen::Matrix3d ROT_MAT; // about 45 degrees
	double ang = -1.55;
    ROT_MAT(0,0) = cos(ang);
    ROT_MAT(0,1) = -sin(ang);
    ROT_MAT(0,2) = 0;
    ROT_MAT(1,0) = sin(ang);
    ROT_MAT(1,1) = cos(ang);
    ROT_MAT(1,2) = 0;
    ROT_MAT(2,0) = 0;
    ROT_MAT(2,1) = 0;
    ROT_MAT(2,2) = 1;
	
	int state = 0;
	// declare gains
	double B_virt = 4000;
	double K_virt = 1000; 
	double K_virt_ang = 1000;
	
	
	// ROS requirement - messages must be in a certain format
	sensor_msgs::JointState desired_joint_state;
	geometry_msgs::PoseStamped cartesian_log, virt_attr_log;
	cartesian_log.header.frame_id = "map";
	virt_attr_log.header.frame_id = "map";
	geometry_msgs::Pose virt_attr;
	geometry_msgs::Wrench transformed_wrench;
	desired_joint_state.position.resize(6);
	desired_joint_state.velocity.resize(6);
		
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
	sensor_origin<<0,0,0.1; //approximately
	sensor_rot<<0,-1,0,
				1,0,0,
				0,0,1;
	sensor_wrt_flange.linear() = sensor_rot;
	sensor_wrt_flange.translation() = sensor_origin;

	// static transform for tool
	Eigen::Affine3d tool_wrt_sensor;
	Eigen::Matrix3d tool_wrt_sensor_rot = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tool_wrt_sensor_trans;
	tool_wrt_sensor_trans<<0,0,0.05;
	tool_wrt_sensor.linear() = tool_wrt_sensor_rot;
	tool_wrt_sensor.translation() = tool_wrt_sensor_trans;
	
	// wait until there are some valid values of joint states from the robot controller
	// ros::spinOnce() allows subscribers to look at their topics
	while(!jnt_state_update_) ros::spinOnce();

	// initialize current end effector position 
	Eigen::Affine3d flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
	Eigen::Affine3d sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
	Eigen::Affine3d tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		

	// rate at which we loop through main program
	ros::Rate naptime(1/dt_);
	
	// main loop
	while(ros::ok()) {
		// subscribers get info
		ros::spinOnce();

		// initialize jacobian
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(joint_states_);
		flange_wrt_robot = irb120_fwd_solver.fwd_kin_solve(joint_states_);
		sensor_wrt_robot = flange_wrt_robot * sensor_wrt_flange;
		tool_wrt_robot = sensor_wrt_robot * tool_wrt_sensor;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		if(!lu_jac.isInvertible()) continue; // Jump to the next iteration in the loop if inverse is not defined
		Eigen::MatrixXd jacobian_inv = lu_jac.inverse();    
		Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
		
		// find current end effector pose
		current_ee_pos = tool_wrt_robot.translation();
		current_ee_rot = tool_wrt_robot.linear(); 
		Eigen::Quaterniond flange_quat(tool_wrt_robot.linear()); // Quaternion is used for logging
		
		// convert forces on the sensor to robot base frame	
		wrench_wrt_robot.head(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.head(3)); // - f_comp.head(3);
		wrench_wrt_robot.tail(3) = sensor_wrt_robot.linear() * (wrench_body_coords_.tail(3)); // - f_comp.tail(3);
        
        virt_attr_pos = current_ee_pos; // This is redundant, TODO
		virt_attr_rot = current_ee_rot;
        switch (state) {
            case 0:
                if(wrench_wrt_robot(0) < F_X_THRESHOLD_CONTACT) {
                    //Until enough force in X is felt, keep pulling the virtual attractor away from the current pose 
                    virt_attr_pos(0) = current_ee_pos(0) + VIRT_ATTR_DIST_MOVE;
                }
                else {
                    state = 1; //Going to state 1
                }
                break;
            case 1:
                if(wrench_wrt_robot(3) < T_X_THRESHOLD_ROTATE) { //  Check if the torque would be positive
                    //Until enough torque in X is felt, keep rotating
					virt_attr_pos = current_ee_pos;
                	virt_attr_rot = current_ee_rot;
					virt_attr_pos(0) = current_ee_pos(0) + VIRT_ATTR_DIST_CONTACT; // Pushing lightly against the surface to keep contact
                    // Do rotation
                    virt_attr_rot = current_ee_rot * ROT_MAT;
                }
				else {
                    state = 2; //Going to state 3
                }
                break;
			case 2:
                //Only move up now
				// TODO: Implement end condition 
                virt_attr_pos = current_ee_pos;
                virt_attr_rot = current_ee_rot;
                virt_attr_pos(0) = current_ee_pos(0) - VIRT_ATTR_DIST_MOVE;
                break;
            default:
                virt_attr_pos = current_ee_pos;
                virt_attr_rot = current_ee_rot;
        }
		
		// CONTROL LAW
		// TODO describe what is happening here
		// effect of virtual attractor
		virtual_force.head(3) = K_virt * (virt_attr_pos - current_ee_pos);
		virtual_force.tail(3) = K_virt_ang * (delta_phi_from_rots(current_ee_rot, virt_attr_rot));
		des_cart_acc = inertia_mat_inv * (-B_virt * des_twist + wrench_wrt_robot + virtual_force);
		des_twist += des_cart_acc * dt_;
		des_jnt_vel = jacobian_inv * des_twist;
							
		//ensure that desired joint vel is within set safe limits
		if(des_jnt_vel.norm() > MAX_JNT_VEL_NORM) des_jnt_vel = (des_jnt_vel / des_jnt_vel.norm()) * MAX_JNT_VEL_NORM;
			
		//euler one step integration to calculate position from velocities
		des_jnt_pos = joint_states_ + (des_jnt_vel * dt_);
		
		// put velocity and position commands into Jointstate message and publish
		for(int i = 0; i < 6; i++) desired_joint_state.position[i] = std::round(des_jnt_pos(i) * 1000) /1000; //implement low pass filter here instead
		for(int i = 0; i < 6; i++) desired_joint_state.velocity[i] = std::round(des_jnt_vel(i) * 1000) /1000;
		arm_publisher.publish(desired_joint_state);
		
		// For logging:
		Eigen::Quaterniond virt_quat(virt_attr_rot);


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
		
		// this ensures update rate is consistent 
		naptime.sleep();
		ros::spinOnce();
	}
}