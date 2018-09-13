#include <irb120_accomodation_control/irb120_accomodation_control.h>
int main(int argc, char** argv) {
	ros::init(argc, argv, "transform_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	geometry_msgs::TransformStamped tf;
	geometry_msgs::Wrench ft_sensor_value, ft_sensor_value_transformed;
	Eigen::Affine3f testing;
	Eigen::Vector3f testing_vector = Eigen::Vector3f::Zero();
	Eigen::Vector3f result_vector;
	geometry_msgs::Twist end_effector_twist_body_frame, end_effector_twist_world_frame;
	while(ros::ok()) {
		tf = control.getFlangeTransform();
		ROS_INFO_STREAM(" CURRENT TRANSFORM "<<tf);
		ft_sensor_value = control.getFTSensorValue();
		ROS_INFO_STREAM("Recieved FT value at tool"<<endl<<ft_sensor_value);
		control.findCartVelFromWrench(ft_sensor_value, end_effector_twist_world_frame);
		ROS_INFO_STREAM("Calculated twist is: "<<end_effector_twist_world_frame);  
		ros::Duration(1).sleep();
	}
}