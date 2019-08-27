//Created 10/31/2018
//Surag Balajepalli


//Simple test for jacobian2 function in irb120_fk_ik
//Chooses random initial joint angles within limits, 
//Adds a disturbance in the order of 10^-4 radians
//Test seems to have passed with the norm of the difference between jacobian calculated and expected cartesian disturbance is less than 0.001




#include <irb120_accomodation_control/irb120_accomodation_control.h>
//#include <Eigen/FullPivLU.h>

Eigen::VectorXd dq,q0,q1,p1,p0;
Eigen::Affine3d p1_affine, p0_affine;
double rand_dist;
int a;

int main(int argc, char **argv) {
	q0 = Eigen::VectorXd::Zero(6);
	q1 = Eigen::VectorXd::Zero(6);
	p1 = Eigen::VectorXd::Zero(6);
	p0 = Eigen::VectorXd::Zero(6);
	
	int i = 0;
	ros::init(argc, argv, "jacobian_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	Irb120_fwd_solver irb120_fwd_solver;
	Irb120_IK_solver ik_solver;
	ROS_INFO_STREAM("before loop");
	while(ros::ok() && i < 1000){
		i++;
		// initializing random joint disturbance 
		dq = Eigen::VectorXd::Zero(6);
		for(int itr = 0; itr < 6;itr++) dq(itr) = (((double)rand() / (double)RAND_MAX)/(double)1000); 
		for(int itr = 0; itr < 6; itr++) {
			//initializing random but valid joint angles. These are limits of joint 3
			// ideally should randomize differently for each joint, between its joint limits
			//q0(itr) = rand()/ ((70 - (-110)) + -110); //Always RADIAN
			q0(itr) = rand() /((double)RAND_MAX + 1) * ((1.22173 - (-1.91986)) + (-1.91986));
		}
		


		//ROS_INFO_STREAM("For testing q0");
		//cout<<q0<<endl;
		//cin>>a;
		q1 = q0 + dq;
		p0_affine = irb120_fwd_solver.fwd_kin_solve(q0);
		p1_affine = irb120_fwd_solver.fwd_kin_solve(q1);
		Eigen::Vector3d rpy0 = p0_affine.linear().eulerAngles(0,1,2);
		Eigen::Vector3d rpy1 = p1_affine.linear().eulerAngles(0,1,2);
		Eigen::Vector3d origin0 = p0_affine.translation();
		Eigen::Vector3d origin1 = p1_affine.translation();
		Eigen::Vector3d dp_origin = origin1 - origin0;
		Eigen::Vector3d dp_rot = rpy1 - rpy0;
		Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
		dp.block<3,1>(0,0) = dp_origin;
		dp.block<3,1>(3,0) = dp_rot; 
		//testing function jacobian2
		q0<<0,-M_PI/3,-M_PI/4,0,M_PI/3,0;
		Eigen::MatrixXd jacobian = irb120_fwd_solver.jacobian2(q0);
		cout<<"jacobian at 0"<<endl<<jacobian<<endl;
		Eigen::FullPivLU<Eigen::MatrixXd> lu_jac(jacobian);
		cin>>a;
		Eigen::VectorXd exp_dp = jacobian * dp;
		
		Eigen::MatrixXd result = jacobian * lu_jac.inverse();
		cout<<"inverse lu jac"<<endl<<lu_jac.inverse()<<endl;
		cin>>a;
		cout<<"inverse"<<endl<<jacobian.inverse()<<endl;
		/* Obsolete
		Eigen::Vector3d exp_dp_origin;
		Eigen::Vector3d exp_dp_rot;
		//for(int itr = 0; itr < 3;itr++) exp_dp_origin(itr) = exp_dp(itr); //theres a better way to do this
		exp_dp_origin = exp_dp.block<3,1>(0,0); // This is the better way
		exp_dp_rot = exp_dp.block<3,1>(3,0);
		ROS_WARN("DEBUG");
		*/
		Eigen::VectorXd diff = dp - exp_dp; //Comparing actual change in end effector position with jacobian calculated change 
		
		if( diff.norm() > 0.001) {
			ROS_WARN("TOO LARGE AN ERROR");
			ROS_INFO_STREAM("Calculated norm is"<<diff.norm());
			
		}		
		else {
			ROS_INFO_STREAM("No error");
		}
		cout<<"diff is";
		cout<<diff<<endl;
		ROS_INFO_STREAM("Dp ");
		cout<<dp<<endl;
		ROS_INFO_STREAM("exp dp ");
		cout<<exp_dp<<endl;
		ROS_INFO_STREAM("jacobian");
		cout<<jacobian<<endl;
			
	}

}