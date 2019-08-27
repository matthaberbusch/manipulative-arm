//This is how I deal with populating EGM messages for ABB
//But where is everything dealing with UDP protocol? Is that upto us?
//What exactly is this library trying to do
//Where does IP go?
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <abb_libegm/egm_controller_interface.h>

//void populateEGMOutput()

int main(int argc, char** argv) {
	ros::init(argc, argv, "libegm_interface_test");
	ros::NodeHandle nh;
	double vals;
	boost::asio::io_service io_service; //of course, I guess?
	const unsigned short port_number = 55967;
	const abb::egm::BaseConfiguration config = abb::egm::BaseConfiguration();
	vector<double> fdbk_jnt_state;
	abb::egm::EGMBaseInterface baseInterfaceObject(io_service, port_number, config);
	abb::egm::BaseConfiguration current_config = baseInterfaceObject.getConfiguration();

	abb::egm::EGMControllerInterface controller(io_service, port_number, config);
	abb::egm::wrapper::Input *input;
	abb::egm::wrapper::Output *output;
	abb::egm::wrapper::Robot *robot;
	abb::egm::wrapper::Joints *joints;
	abb::egm::wrapper::JointSpace *jsp;

	while(!controller.waitForMessage(100)) 
	{
		ROS_INFO("Waiting for first message");
	}
	while(ros::ok()){
		//joints.set_value(index,value) - Theres obviously gotta be a better way. But for now. 
		//Create function that takes joint vals as vector and returns an output wrapper
		//vals = joints.values(0);
		controller.read(input);
		
		if(!(input->feedback().robot().joints().position().values().size() == 6)){ 
			ROS_WARN("recieved bad message, retrying");
			continue;
		}
		
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(0));
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(1));
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(2));
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(3));
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(4));
		fdbk_jnt_state.push_back(input->feedback().robot().joints().position().values(5));
		/*
		for(int i = 0; i < 6; i++) {
			joints->set_values(i,fdbk_jnt_state[i] + 0.1);
		}
		jsp->set_allocated_position(joints);
		robot->set_allocated_joints(jsp);
		output->set_allocated_robot(robot);
		controller.write(*output); //This is very weird, maybe theres a better way, but I just cant see it.
	*/
	}	
}