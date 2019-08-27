#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>

geometry_msgs::Pose g_desired_pose;
bool g_new_pose_exists = false;

void poseCB(const geometry_msgs::Pose &pose) {
	g_desired_pose = pose;
	g_new_pose_exists = true;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "everything_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	geometry_msgs::Wrench ft_sensor_value;
	geometry_msgs::Twist end_effector_twist;
	vector<float> joint_velocities;
	sensor_msgs::JointState joint_state;
	boost::asio::io_service io_service;
	ros::Subscriber pose_subscriber = nh.subscribe("destination_pose",1,poseCB);
	while(ros::ok()) {
		//subscribe to a pose message and populate 2 global vars with it, a bool new_msg and the pose
		ros::spinOnce();
		//ROS_INFO("In accomodation loop");
		//ROS_INFO_STREAM("Recieved FT value"<<ft_sensor_value);
		if(g_new_pose_exists) {
			g_new_pose_exists = false;
			control.gotoPose(g_desired_pose);
			ros::Duration(0.01).sleep();
			
		}
		else{
		control.accomodate();
		}
		/*
		//make all these one function "accomodate" and the other func "move"
		ft_sensor_value = control.getFTSensorValue();  //Remove these and change function definitions, check if it works
		control.findCartVelFromWrench(ft_sensor_value, end_effector_twist);
		//ROS_INFO_STREAM("Calculated twist is: "<<end_effector_twist);
		joint_state = control.getJointState();
		//ROS_INFO_STREAM("Joint state is : "<< joint_state);
		control.findJointVelFromCartVel(end_effector_twist, joint_state, joint_velocities);
		//ROS_INFO_STREAM("Converted");
		//ROS_INFO_STREAM("Calculated joint velocities are: "<<joint_velocities);
		control.commandJointPosFromJointVel(joint_velocities, joint_state);
		*/

		ros::Duration(0.01).sleep();

	}
}
