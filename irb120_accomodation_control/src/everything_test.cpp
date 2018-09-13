#include <irb120_accomodation_control/irb120_accomodation_control.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "everything_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	geometry_msgs::Wrench ft_sensor_value;
	geometry_msgs::Twist end_effector_twist;
	vector<float> joint_velocities;
	sensor_msgs::JointState joint_state;
	
	while(ros::ok()) {
		ros::spinOnce();
		//ROS_INFO("In accomodation loop");
		ft_sensor_value = control.getFTSensorValue();  //Remove these and change function definitions, check if it works
		//ROS_INFO_STREAM("Recieved FT value"<<ft_sensor_value);
		control.findCartVelFromWrench(ft_sensor_value, end_effector_twist);
		//ROS_INFO_STREAM("Calculated twist is: "<<end_effector_twist);
		joint_state = control.getJointState();
		//ROS_INFO_STREAM("Joint state is : "<< joint_state);
		control.findJointVelFromCartVel(end_effector_twist, joint_state, joint_velocities);
		//ROS_INFO_STREAM("Converted");
		//ROS_INFO_STREAM("Calculated joint velocities are: "<<joint_velocities);
		control.commandJointPosFromJointVel(joint_velocities, joint_state);
		ros::Duration(0.01).sleep();

	}
}
