#include<irb120_accomodation_control/irb120_accomodation_control.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "publish_test_with_irb120_class");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	ros::spinOnce();
	std::vector<float> test_vector(6,0);
	while(ros::ok()) {
		control.publishJointAngles(test_vector);
		ros::spinOnce();
	}

}