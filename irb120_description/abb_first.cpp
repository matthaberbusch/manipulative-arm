class Irb120AccomodationControl {
	public:
	void findCartVelFromWrench (geometry_msgs/Wrench wrench, geometry_msgs/Twist &twist);
	void findCartVelFromWrench (geometry_msgs/Wrench wrench, geometry_msgs/Twist &twist, Eigen::MatrixXf accomodation_gain);
	void findJointVelFromCartVel (geometry_msgs/Twist twist, Eigen::MatrixXf jacobian, vector<float> &joint_vel);
	void findJointVelFromCartVel (geometry_msgs/Twist twist, vector<float> &joint_vel);
	void publishJointStates()
	Irb120AccomodationControl();

	private:
	Eigen::MatrixXf accomodation_gain(6) = MaxtrixXf::Identity(6);
	Eigen::MatrixXf jacobian(6) = MaxtrixXf::Zero(6); //Need to initialize this
	Eigen::MatrixXf jacobian_inverse(6) = jacobian.inverse();
	string joint1_topic_name = "/irb120/joint1_position_controller/command";
	string joint2_topic_name = "/irb120/joint2_position_controller/command";
	string joint3_topic_name = "/irb120/joint3_position_controller/command";
	string joint4_topic_name = "/irb120/joint4_position_controller/command";
	string joint5_topic_name = "/irb120/joint5_position_controller/command";
	string joint6_topic_name = "/irb120/joint6_position_controller/command";

	};

	Irb120AccomodationControl::Irb120AccomodationControl(ros::NodeHandle &nh) {
		
		//setting up all publishers
		ros::Publisher joint1_pub = nh.advertise<std::msgs/Float64>(joint1_topic_name, 1);
		ros::Publisher joint2_pub = nh.advertise<std::msgs/Float64>(joint2_topic_name, 1);
		ros::Publisher joint3_pub = nh.advertise<std::msgs/Float64>(joint3_topic_name, 1);
		ros::Publisher joint4_pub = nh.advertise<std::msgs/Float64>(joint4_topic_name, 1);
		ros::Publisher joint5_pub = nh.advertise<std::msgs/Float64>(joint5_topic_name, 1);
		ros::Publisher joint6_pub = nh.advertise<std::msgs/Float64>(joint6_topic_name, 1);
		
		//setting up all subscribers
		ros::Subscriber joint_pos_subscriber = nh.subscribe(joint_pos_subscriber_topic, 1, jointPosCallBack);
		//f/t callback

	}