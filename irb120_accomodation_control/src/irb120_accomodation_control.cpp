/*#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

using namespace std;

class Irb120AccomodationControl {
	public:
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist);
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist, Eigen::MatrixXf accomodation_gain);
	void findJointVelFromCartVel (geometry_msgs::Twist twist, Eigen::MatrixXf jacobian, vector<float> &joint_vel);
	void findJointVelFromCartVel (geometry_msgs::Twist twist, vector<float> &joint_vel);
	void publishJointAngles(vector<float> joint_pos);
	void publishJointAngles(vector<std_msgs::Float64> joint_pos);
	void jointStateCallBack (const sensor_msgs::JointState &joint_state); 
	void ftCallBack (const geometry_msgs::WrenchStamped &wrench_stamped);
	sensor_msgs::JointState getJointState();
	geometry_msgs::Wrench getFTSensorValue();
	Irb120AccomodationControl(ros::NodeHandle &nh);

	private:
	geometry_msgs::Wrench g_ft_value_;
	sensor_msgs::JointState g_joint_state_;
	const Eigen::MatrixXf accomodation_gain = Eigen::MatrixXf::Identity(6,6);
	const Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(6,6); //Need to initialize this
	const Eigen::MatrixXf jacobian_inverse = jacobian.inverse();
	const string joint1_topic_name = "/irb120/joint1_position_controller/command";
	const string joint2_topic_name = "/irb120/joint2_position_controller/command";
	const string joint3_topic_name = "/irb120/joint3_position_controller/command";
	const string joint4_topic_name = "/irb120/joint4_position_controller/command";
	const string joint5_topic_name = "/irb120/joint5_position_controller/command";
	const string joint6_topic_name = "/irb120/joint6_position_controller/command";
	const string joint_state_subscriber_topic = "/irb120/joint_states";
	const string ft_value_subscriber_topic = "/ft_sensor_topic";
	ros::Publisher joint1_pub, joint2_pub, joint3_pub, joint4_pub, joint5_pub, joint6_pub;
	ros::Subscriber joint_state_subscriber, ft_value_subscriber;
	

}; */
//FIX THIS MESSY LIBRARY!!!

#include <irb120_accomodation_control/irb120_accomodation_control.h>
	Irb120AccomodationControl::Irb120AccomodationControl(ros::NodeHandle &nh) {
		
		
		initializePublishers(nh);
		initializeSubscribers(nh);
		tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
		
		//Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(6,6); //Need to initialize this// put it somewhere else?
		warmUp();

		//to prevent massive seg faults in case my subscribers are feeling moody
		
	}

	void Irb120AccomodationControl::initializePublishers(ros::NodeHandle &nh) {
		//setting up all publishers
		joint1_pub = nh.advertise<std_msgs::Float64>(joint1_topic_name, 1);
		joint2_pub = nh.advertise<std_msgs::Float64>(joint2_topic_name, 1);
		joint3_pub = nh.advertise<std_msgs::Float64>(joint3_topic_name, 1);
		joint4_pub = nh.advertise<std_msgs::Float64>(joint4_topic_name, 1);
		joint5_pub = nh.advertise<std_msgs::Float64>(joint5_topic_name, 1);
		joint6_pub = nh.advertise<std_msgs::Float64>(joint6_topic_name, 1);
		
	}

	void Irb120AccomodationControl::initializeSubscribers(ros::NodeHandle &nh) {
		joint_state_subscriber = nh.subscribe(joint_state_subscriber_topic, 1, &Irb120AccomodationControl::jointStateCallBack, this);
		ft_value_subscriber = nh.subscribe(ft_value_subscriber_topic,1,&Irb120AccomodationControl::ftCallBack, this);
	}

	void Irb120AccomodationControl::warmUp() {
		sensor_msgs::JointState joint_state;
		geometry_msgs::Wrench wrench;
		while(!(g_joint_state_.position.size() == 6) && !(g_joint_state_.velocity.size()) ) ros::spinOnce(); //need a way to check F/T sensor value
	}

	void Irb120AccomodationControl::updateFlangeTransform() {
		//need to wait for transform, how?
		//tfBuffer_.waitForTransform(base_frame_,flange_frame_,ros::Time::now(),ros::Duration(3.0)); //this blocks, oh well. This has been deprecated anyway
		flange_transform_ = tfBuffer_.lookupTransform(base_frame_,flange_frame_,ros::Time(0), ros::Duration(1));//This blocks until tfs are found
		//bad code, fix it later on 
		float a = flange_transform_.transform.rotation.w;
		float b = flange_transform_.transform.rotation.x;
		float c = flange_transform_.transform.rotation.y;
		float d = flange_transform_.transform.rotation.z;
		//need to construct a 4x4 transformation matrix from the transformStamped message
		//seperately filling out the 3x3 rotation matrix
		Eigen::Matrix3f rotation_matrix;
		rotation_matrix(0,0) = pow(a,2) + pow(b,2) - pow(c,2) - pow(d,2);
		rotation_matrix(0,1) = 2 * (b * c - a * d);
		rotation_matrix(0,2) = 2 * (b * d + a * c);
		rotation_matrix(1,0) = 2 * (b * c + a * d);
		rotation_matrix(1,1) = pow(a,2) - pow(b,2) + pow(c,2) - pow(d,2);
		rotation_matrix(1,2) = 2 * (c * d - a * b);
		rotation_matrix(2,0) = 2 * (b * d - a * c);
		rotation_matrix(2,1) = 2 * (c * d + a * b);
		rotation_matrix(2,2) = pow(a,2) - pow(b,2) - pow(c,2) + pow(d,2); 
		
		flange_transform_matrix_.block<3,3>(0,0) = rotation_matrix;
		flange_transform_matrix_.row(3)<<0,0,0,1;
		//since rotation matrix element 4,4 is 1
		flange_transform_matrix_(3,3) = 1;
		//last column of the 4x4 matrix contains the origin 
		flange_transform_matrix_.col(3)<<flange_transform_.transform.translation.x,
										flange_transform_.transform.translation.y,
										flange_transform_.transform.translation.z,
										1;
		Eigen::Vector3f translation_for_affine;
		translation_for_affine<<flange_transform_.transform.translation.x,
										flange_transform_.transform.translation.y,
										flange_transform_.transform.translation.z;
		//such overkill, initialized element 4,4 to be 1 at 3 diff places, fix this code! FIX IT!
		//Usable form for transformations
		//flange_transform_affine_.linear() = rotation_matrix;
		//flange_transform_affine_.translation() = translation_for_affine;
		Eigen::Affine3f A(flange_transform_matrix_);
		flange_transform_affine_ = A; //apparently this is a thing
	}

	geometry_msgs::Wrench Irb120AccomodationControl::transformWrench(geometry_msgs::Wrench wrench) {
		//This is a useless func for now, 
		//Can be used well when there is a transform between tool and sensor
		updateFlangeTransform();
		Eigen::VectorXf wrench_vector, transformed_wrench_vector;
		Eigen::Vector3f origin;
		Eigen::Matrix3f origin_hat;
		geometry_msgs::Wrench transformed_wrench, recieved_wrench;
		recieved_wrench = wrench;
		wrench_vector = Eigen::VectorXf::Zero(6);  //why is Eigen library so moody
		wrench_vector<<recieved_wrench.force.x,
						recieved_wrench.force.y,
						recieved_wrench.force.z,
						recieved_wrench.torque.x,
						recieved_wrench.torque.y,
						recieved_wrench.torque.z;
		Eigen::MatrixXf wrench_transformation_matrix = Eigen::MatrixXf::Zero(6,6);
		origin = flange_transform_affine_.translation();
		origin_hat = vectorHat(origin);
		//This is from MLS book, may be implemented wrongly
		Eigen::Matrix3f rotation_matrix = flange_transform_matrix_.block<3,3>(0,0);
		wrench_transformation_matrix.block<3,3>(0,0) = flange_transform_affine_.linear().transpose();
		wrench_transformation_matrix.block<3,3>(3,3) = flange_transform_affine_.linear().transpose();
		wrench_transformation_matrix.block<3,3>(3,0) = flange_transform_affine_.linear().transpose() * origin_hat * -1;
		wrench_transformation_matrix.block<3,3>(0,3)<<0,0,0,0,0,0,0,0,0;
		transformed_wrench_vector = wrench_transformation_matrix.inverse() * wrench_vector;
		transformed_wrench.force.x = transformed_wrench_vector(0);
		transformed_wrench.force.y = transformed_wrench_vector(1);
		transformed_wrench.force.z = transformed_wrench_vector(2);
		transformed_wrench.torque.x = transformed_wrench_vector(3);
		transformed_wrench.torque.y = transformed_wrench_vector(4);
		transformed_wrench.torque.z = transformed_wrench_vector(5);
		return transformed_wrench;
	}

	void Irb120AccomodationControl::initializeJacobian(sensor_msgs::JointState joint_states) {
		
		ros::spinOnce(); //ensure latest joint state values --Not working, joint state message is blank
		//should be a better way to do this.
		//look into KDL for ros
		//hand solved jacobian using symbolic math toolbox in matlab, lot of scope for errors.
		jacobian_(0,0) = 0;
		jacobian_(0,1) = 290*sin(joint_states.position[0]);
		jacobian_(0,2) = 560*sin(joint_states.position[0]); 
		jacobian_(0,3) = - (cos(joint_states.position[0])*sin(joint_states.position[1])*sin(joint_states.position[2]) - cos(joint_states.position[0])*cos(joint_states.position[1])*cos(joint_states.position[2]))*(134*sin(joint_states.position[2]) - 560) - 134*cos(joint_states.position[2])*(cos(joint_states.position[1])*sin(joint_states.position[2]) + cos(joint_states.position[2])*sin(joint_states.position[1]));
		jacobian_(0,4) =  (cos(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) + sin(joint_states.position[2])*sin(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])))*(134*sin(joint_states.position[2]) - 560) + 134*pow((cos(joint_states.position[2])), 2)*sin(joint_states.position[3]);
		jacobian_(0,5) = - (cos(joint_states.position[4])*sin(joint_states.position[2]) + cos(joint_states.position[2])*cos(joint_states.position[3])*sin(joint_states.position[4]))*(134*cos(joint_states.position[2]) + 374*sin(joint_states.position[4])) - (sin(joint_states.position[4])*(sin(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) - cos(joint_states.position[3])*sin(joint_states.position[2])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1]))) + cos(joint_states.position[2])*cos(joint_states.position[4])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])))*(374*cos(joint_states.position[4]) - 134*sin(joint_states.position[2]) + 560);
		jacobian_(1,0) = 0;
		jacobian_(1,1) = -290*cos(joint_states.position[0]); 
		jacobian_(1,2) = -560*cos(joint_states.position[0]);
		jacobian_(1,3) = (cos(joint_states.position[1])*cos(joint_states.position[2])*sin(joint_states.position[0]) - sin(joint_states.position[0])*sin(joint_states.position[1])*sin(joint_states.position[2]))*(134*sin(joint_states.position[2]) - 560);
		jacobian_(1,4) = (cos(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])) - sin(joint_states.position[2])*sin(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])))*(134*sin(joint_states.position[2]) - 560);
		jacobian_(1,5) = -(sin(joint_states.position[4])*(sin(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])) + cos(joint_states.position[3])*sin(joint_states.position[2])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0]))) - cos(joint_states.position[2])*cos(joint_states.position[4])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])))*(374*cos(joint_states.position[4]) - 134*sin(joint_states.position[2]) + 560);

		jacobian_(2,0) = 0;
		jacobian_(2,1) = 0;
		jacobian_(2,2) = 0;
		jacobian_(2,3) = 134*cos(joint_states.position[2])*(cos(joint_states.position[1])*cos(joint_states.position[2])*sin(joint_states.position[0]) - sin(joint_states.position[0])*sin(joint_states.position[1])*sin(joint_states.position[2]));
		jacobian_(2,4) = 134*cos(joint_states.position[2])*(cos(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])) - sin(joint_states.position[2])*sin(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])));
		jacobian_(2,5) = (sin(joint_states.position[4])*(sin(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])) + cos(joint_states.position[3])*sin(joint_states.position[2])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0]))) - cos(joint_states.position[2])*cos(joint_states.position[4])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])))*(134*cos(joint_states.position[2]) + 374*sin(joint_states.position[4]));

		jacobian_(3,0) = 0;
		jacobian_(3,1) = -cos(joint_states.position[0]);
		jacobian_(3,2) = -cos(joint_states.position[0]);
		jacobian_(3,3) = sin(joint_states.position[0])*sin(joint_states.position[1])*sin(joint_states.position[2]) - cos(joint_states.position[1])*cos(joint_states.position[2])*sin(joint_states.position[0]);
		jacobian_(3,4) = sin(joint_states.position[2])*sin(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) - cos(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1]));
		jacobian_(3,5) = cos(joint_states.position[2])*cos(joint_states.position[4])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) - sin(joint_states.position[4])*(sin(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1])) + cos(joint_states.position[3])*sin(joint_states.position[2])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])));

		jacobian_(4,0) = 0;
		jacobian_(4,1) = -sin(joint_states.position[0]);
		jacobian_(4,2) = -sin(joint_states.position[0]);
		jacobian_(4,3) = cos(joint_states.position[0])*cos(joint_states.position[1])*cos(joint_states.position[2]) - cos(joint_states.position[0])*sin(joint_states.position[1])*sin(joint_states.position[2]);
		jacobian_(4,4) = cos(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) + sin(joint_states.position[2])*sin(joint_states.position[3])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1]));
		jacobian_(4,5) = sin(joint_states.position[4])*(sin(joint_states.position[3])*(cos(joint_states.position[0])*sin(joint_states.position[1]) - cos(joint_states.position[1])*sin(joint_states.position[0])) - cos(joint_states.position[3])*sin(joint_states.position[2])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1]))) + cos(joint_states.position[2])*cos(joint_states.position[4])*(cos(joint_states.position[0])*cos(joint_states.position[1]) + sin(joint_states.position[0])*sin(joint_states.position[1]));

		jacobian_(5,0) = 1;
		jacobian_(5,1) = 0;
		jacobian_(5,2) = 0;
		jacobian_(5,3) = - cos(joint_states.position[1])*sin(joint_states.position[2]) - cos(joint_states.position[2])*sin(joint_states.position[1]);
		jacobian_(5,4) = cos(joint_states.position[2])*sin(joint_states.position[3]);
		jacobian_(5,5) = - cos(joint_states.position[4])*sin(joint_states.position[2]) - cos(joint_states.position[2])*cos(joint_states.position[3])*sin(joint_states.position[4]);

		
		jacobian_inverse_ = jacobian_.inverse();

	}

	
	void Irb120AccomodationControl::calculateTwistFromWrench(geometry_msgs::Wrench wrench, sensor_msgs::JointState joint_state, vector<float> desired_point, geometry_msgs::Twist &twist) {
		//calculates accomodation gain from napkin math, need confirmation	
		//Do not use until irb120 fk are implemented
		//DO NOT USE UNTIL CHECKED
		Eigen::VectorXf wrench_matrix(6); //since operations need to be performed
		Eigen::VectorXf twist_matrix(6);
		wrench_matrix<<wrench.force.x, 
						wrench.force.y,
						wrench.force.z,
						wrench.torque.x,
						wrench.torque.y,
						wrench.torque.z;
		Eigen::VectorXf desired_end_effector_pose(6);
		if(desired_point.size() == 6) {
		desired_end_effector_pose<<desired_point[0],
									desired_point[1],
									desired_point[2],
									desired_point[3],
									desired_point[4],
									desired_point[5];
								}
		Eigen::VectorXf current_end_effector_pose(6), current_end_effector_velocity(6);
		current_end_effector_pose<<0,0,0,0,0,0; //replace with fk from joint state
		current_end_effector_velocity<<0,0,0,0,0,0; //ideally to be calculated from Jacobian and joint vel
		//make desired point, joint state.position and joint_state.velocity into eigen
		twist_matrix = (dt *(wrench_matrix + k_virtual_ * (desired_end_effector_pose - current_end_effector_pose) + b_virtual_ * (current_end_effector_velocity))) * m_virtual_.inverse(); 
		twist.linear.x = twist_matrix(0); //rethink the need to populate this message
		twist.linear.y = twist_matrix(1);
		twist.linear.z = twist_matrix(2);
		twist.angular.x = twist_matrix(3);
		twist.angular.y = twist_matrix(4);
		twist.angular.z = twist_matrix(5);
	}


	void Irb120AccomodationControl::findCartVelFromWrench(geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist) {
		//uses accomodation gain described in class
		updateFlangeTransform(); // gets the latest transform value into a member variable called flange_transform_matrix_;
		//geometry_msgs::Wrench transformed_wrench = transformWrench(wrench); //What a dumb thought this was. 
		geometry_msgs::Twist twist_tool_frame;
		Eigen::VectorXf wrench_matrix(6); //since operations need to be performed
		Eigen::VectorXf twist_matrix_tool_frame(6), twist_matrix_world_frame(6);
		wrench_matrix<<wrench.force.x, 
						wrench.force.y,
						wrench.force.z,
						wrench.torque.x,
						wrench.torque.y,
						wrench.torque.z; 
		twist_matrix_tool_frame = accomodation_gain_ * wrench_matrix; 
		twist_tool_frame.linear.x = twist_matrix_tool_frame(0); //rethink the need to populate this message
		twist_tool_frame.linear.y = twist_matrix_tool_frame(1);
		twist_tool_frame.linear.z = twist_matrix_tool_frame(2);
		twist_tool_frame.angular.x = twist_matrix_tool_frame(3);
		twist_tool_frame.angular.y = twist_matrix_tool_frame(4);
		twist_tool_frame.angular.z = twist_matrix_tool_frame(5);
		twist = transformTwist(twist_tool_frame);
	}

	geometry_msgs::Twist Irb120AccomodationControl::transformTwist(geometry_msgs::Twist twist_tool_frame) {
		updateFlangeTransform();
		geometry_msgs::Twist twist_world_frame;
		Eigen::VectorXf twist_vector_tool_frame(6), twist_vector_world_frame(6);
		Eigen::Vector3f origin;
		Eigen::Matrix3f origin_hat;
		Eigen::MatrixXf twist_transformation_matrix = Eigen::MatrixXf::Zero(6,6);
		twist_vector_tool_frame<<twist_tool_frame.linear.x,
								twist_tool_frame.linear.y,
								twist_tool_frame.linear.z,
								twist_tool_frame.angular.x,
								twist_tool_frame.angular.y,
								twist_tool_frame.angular.z;

		origin = flange_transform_affine_.translation();
		origin_hat = vectorHat(origin);
		twist_transformation_matrix.block<3,3>(0,0) = flange_transform_affine_.linear();
		twist_transformation_matrix.block<3,3>(3,3) = flange_transform_affine_.linear();
		twist_transformation_matrix.block<3,3>(0,3) = origin_hat * flange_transform_affine_.linear();
		twist_vector_world_frame = twist_transformation_matrix * twist_vector_tool_frame;
		twist_world_frame.linear.x = twist_vector_world_frame(0);
		twist_world_frame.linear.y = twist_vector_world_frame(1);
		twist_world_frame.linear.z = twist_vector_world_frame(2);
		twist_world_frame.angular.x = twist_vector_world_frame(3);
		twist_world_frame.angular.x = twist_vector_world_frame(4);
		twist_world_frame.angular.x = twist_vector_world_frame(5);
		return twist_world_frame;
	}

	void Irb120AccomodationControl::findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist,
														 Eigen::MatrixXf given_accomodation_gain) {
		//for user defined accomodation gain
		Eigen::VectorXf wrench_matrix(6); //since operations need to be performed
		Eigen::VectorXf twist_matrix(6);
		wrench_matrix<<wrench.force.x, 
						wrench.force.y,
						wrench.force.z,
						wrench.torque.x,
						wrench.torque.y,
						wrench.torque.z; 
		twist_matrix = wrench_matrix * given_accomodation_gain; //maybe the other way round, check it out, makes no difference with I
		twist.linear.x = twist_matrix(0); //rethink the need to populate this message
		twist.linear.y = twist_matrix(1);
		twist.linear.z = twist_matrix(2);
		twist.angular.x = twist_matrix(3);
		twist.angular.y = twist_matrix(4);
		twist.angular.z = twist_matrix(5);	
	}

	void Irb120AccomodationControl::findJointVelFromCartVel (geometry_msgs::Twist twist, sensor_msgs::JointState joint_states, Eigen::MatrixXf given_jacobian, vector<float> &joint_vel) {
		//takes cart vel input in twist form, returns a 6 element vector of joint vel
		//User defined jacobian
		joint_vel.clear();
		Eigen::VectorXf twist_matrix(6);
		Eigen::VectorXf joint_velocities(6);
		twist_matrix<<twist.linear.x,
						twist.linear.y,
						twist.linear.z,
						twist.angular.x,
						twist.angular.y,
						twist.angular.z;
		joint_velocities = given_jacobian.inverse() * twist_matrix;
		joint_vel.push_back(joint_velocities(0));
		joint_vel.push_back(joint_velocities(1));
		joint_vel.push_back(joint_velocities(2));
		joint_vel.push_back(joint_velocities(3));
		joint_vel.push_back(joint_velocities(4));
		joint_vel.push_back(joint_velocities(5));
	}



	void Irb120AccomodationControl::findJointVelFromCartVel (geometry_msgs::Twist twist, sensor_msgs::JointState joint_states, vector<float> &joint_vel) {
		//same as above, except
		//not user defined jacobian
		ros::spinOnce();
		initializeJacobian(joint_states);
		joint_vel.clear();
		Eigen::VectorXf twist_matrix(6);
		Eigen::VectorXf joint_velocities(6);
		twist_matrix<<twist.linear.x,
						twist.linear.y,
						twist.linear.z,
						twist.angular.x,
						twist.angular.y,
						twist.angular.z;
		joint_velocities = jacobian_inverse_ * twist_matrix;
		joint_vel.push_back(joint_velocities(0));
		joint_vel.push_back(joint_velocities(1));
		joint_vel.push_back(joint_velocities(2));
		joint_vel.push_back(joint_velocities(3));
		joint_vel.push_back(joint_velocities(4));
		joint_vel.push_back(joint_velocities(5));
	}

	void Irb120AccomodationControl::commandJointPosFromJointVel(vector<float> joint_vel, sensor_msgs::JointState joint_state) {
		ros::spinOnce();
		vector<float> joint_pos_cmd(6,0);
		for(int i = 0; i <=5; i++) {
			joint_pos_cmd[i] = joint_vel[i] *  dt + joint_state.position[i];
		}
		publishJointAngles(joint_pos_cmd);
	}

	void Irb120AccomodationControl::publishJointAngles(vector<std_msgs::Float64> joint_pos) {
		joint1_pub.publish(joint_pos[0]);
		joint2_pub.publish(joint_pos[1]);
		joint3_pub.publish(joint_pos[2]);
		joint4_pub.publish(joint_pos[3]);
		joint5_pub.publish(joint_pos[4]);
		joint6_pub.publish(joint_pos[5]);
	}

	void Irb120AccomodationControl::publishJointAngles(vector<float> joint_pos) {
		vector<std_msgs::Float64> joint_pos_msg(6);
		for(int i = 0; i < joint_pos.size(); i++) {
			joint_pos_msg[i].data = joint_pos[i];
		} 
		publishJointAngles(joint_pos_msg);

	}

	Eigen::Matrix3f Irb120AccomodationControl::vectorHat(Eigen::Vector3f vector) {
		Eigen::Matrix3f hat_of_vector;
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

	void Irb120AccomodationControl::jointStateCallBack(const sensor_msgs::JointState &joint_state) {
		g_joint_state_ = joint_state;
	}

	void Irb120AccomodationControl::ftCallBack(const geometry_msgs::WrenchStamped &wrench_stamped) {
		g_ft_value_ = wrench_stamped.wrench;
	}

	void Irb120AccomodationControl::setKvirtual(Eigen::MatrixXf k_virtual) {
		k_virtual_ = k_virtual;
	}

	void Irb120AccomodationControl::setMvirtual(Eigen::MatrixXf m_virtual) {
		m_virtual_ = m_virtual;
	}

	void Irb120AccomodationControl::setBvirtual(Eigen::MatrixXf b_virtual) {
		b_virtual_ = b_virtual;
	}

	Eigen::MatrixXf Irb120AccomodationControl::getKvirtual(){
		return k_virtual_;
	}

	Eigen::MatrixXf Irb120AccomodationControl::getMvirtual(){
		return m_virtual_;
	}

	Eigen::MatrixXf Irb120AccomodationControl::getBvirtual(){
		return b_virtual_;
	}
	sensor_msgs::JointState Irb120AccomodationControl::getJointState() {
		
		return g_joint_state_;
	}

	geometry_msgs::Wrench Irb120AccomodationControl::getFTSensorValue() {
		ros::spinOnce();
		return g_ft_value_;
	}

	Eigen::MatrixXf Irb120AccomodationControl::getJacobian() {
		return jacobian_;
	}

	geometry_msgs::TransformStamped Irb120AccomodationControl::getFlangeTransform() {
		updateFlangeTransform();
		return flange_transform_;
	}

	geometry_msgs::Wrench Irb120AccomodationControl::getTransformedWrench() {
		ros::spinOnce();
		updateFlangeTransform();
		ros::spinOnce();
		return transformWrench(g_ft_value_);
	}

	Eigen::Affine3f Irb120AccomodationControl::getAffine_test() {
		ros::spinOnce();
		updateFlangeTransform();
		ros::spinOnce();
		return flange_transform_affine_;
	}
