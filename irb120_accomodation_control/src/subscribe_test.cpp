#include <irb120_accomodation_control/irb120_accomodation_control.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "subscribe_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	sensor_msgs::JointState joint_state;
	geometry_msgs::Wrench ft_value;
	ros::Rate sleeptime(1.0);
	while(ros::ok()) {
		ros::spinOnce();
		joint_state = control.getJointState();
		ft_value = control.getFTSensorValue();
		ROS_INFO_STREAM("Joint state is " <<joint_state<<endl);
		ROS_INFO_STREAM("---------------------");
		ROS_INFO_STREAM("ft_value is " <<ft_value<<endl);
		ROS_INFO_STREAM("----------------------");
		sleeptime.sleep();
	}

}