

void ftSensorCallBack(geometry_msgs::WrenchStamped ft_value) {
	/* message description:
	std_msgs/Header header
  		uint32 seq
  		time stamp
  		string frame_id
	geometry_msgs/Wrench wrench
  		geometry_msgs/Vector3 force
    		float64 x
    		float64 y
    		float64 z
  		geometry_msgs/Vector3 torque
    		float64 x
    		float64 y
    		float64 z
*/

	
}

void jointPosCallBack(const sensor_msgs::JointState &joint_state) {
		g_joint_state = joint_state;
	}


void ftCallBack(const geometry_msgs::WrenchStamped &wrench_stamped) {
		g_ft_value = wrench_stamped.wrench;
	}

void jointPosCallBack(sensor_msgs::JointState joint_state) {
	/* message description:
	 std_msgs/Header header
  		uint32 seq
  		time stamp
  		string frame_id
	string[] name
	float64[] position
	float64[] velocity
	float64[] effort
*/


}

void WrenchToCartVel(geometry_msgs/Wrench wrench, geometry_msgs/Twist &twist) {
 /* twist: 
 		geometry_msgs/Vector3 linear
  			float64 x
  			float64 y
  			float64 z
		geometry_msgs/Vector3 angular
		  	float64 x
		  	float64 y
		  	float64 z


1. wrench to eigen 6x1
2. multiply with Ka (6x6), private member variable
3. convert resulting matrix to twist
*/

}

void CartVelToJointVel (geometry_msgs/Twist twist, Eigen::Matrix6f jacobian, vector<float> &joint_vel) {
	Eigen::Matrix6f jacobian_inverse = jacobian.inverse();
	Eigen::Vector6f twist_matrix = //Open up and stuff it
	Eigen::Vector6f joint_velocities = jacobian_inverse * twist_matrix;
	joint_vel = joint_velocities; //Open up and stuff it
}