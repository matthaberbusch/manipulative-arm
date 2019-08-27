#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <actionlib/server/simple_acttion_server.h>
#include <irb120_accomodation_control/trajectoryAction.h>

class AccActionServer {
private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<irb120_accomodation_control::trajectoryAction> as_;
	irb120_accomodation_control::trajectoryGoal goal_;
	irb120_accomodation_control::trajectoryResult result_;
	irb120_accomodation_control::trajectoryFeedback feedback_;

public:
	AccActionServer();
	~AccActionServer() {

	}
	void executeCB(const actionlib::SimpleActionServer<irb120_accomodation_control::trajectoryAction>::GoalConstPtr &goal);
};

AccActionServer::AccActionServer():
	as_(nh_,"trajectory_action", boost::bind(&AccActionServer::executeCB,this, _1), false)
	{
		as_.start();
	}

void AccActionServer::executeCB(const actionlib::SimpleActionServer<irb120_accomodation_control::trajectoryAction>::GoalConstPtr & goal) {
	ROS_INFO_STREAM("Trajectory recieved is "<<goal->traj);
	//interpolation - optional
	//feedback - current pose
	//result - current pose within some limit of desired pose
	
}
