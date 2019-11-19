// Rotate until torque with Orientation Targeting and Torque Limiting (PTFL)
// Matthew Haberbusch, Surag Balajepalli, and Rahul Pokharna 
// Last updated 10/15/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <behavior_algorithms/status_service.h>
#include <math.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;
Eigen::VectorXd joint_states = Eigen::VectorXd::Zero(6);

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}
void jointStateCallback(const sensor_msgs::JointState& joint_state) {
    // ROS: Get joint states from robot
    for(int i = 0; i < 6; i++) joint_states(i) = joint_state.position[i] ;

}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"otel_y");
    ros::NodeHandle nh;

    // ROS: Define subscribers and publishers used
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback); // subscribe to the topic publishing the cartesian state of the end effector
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);                       // subscribe to the force/torque sensor data
    ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);                 // subscribe to the state of joints to get the orientation of the end effector for rotation
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1); // publish the pose of the virtual attractor for the accomodation controller 

    // ROS: Services used in conjunction with buffer.cpp to have delayed program status sent to operator
    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");

    // ROS: Service status variable for use with buffer.cpp
    behavior_algorithms::status_service srv;
    srv.request.name = "OTEL in y";

    // Declare constants
    /*
    How to tune params:
    ROTATE_ANGLE: By increasing this, the virtual force is greater, and decreasing decreases it, faster or slower rotations
    KEEP_CONTACT_DISTANCE: If the effort threshold is crossed, the virtual attractor is placed this distance below the current pose to keep contact if pushing, and above if pulling to keep a constant force
    FORCE_THRESHOLD: The limit at which the program will stop if the force in the direction which we are moving is crossed
    TORQUE_THRESHOLD: The limit at which the program will stop if the torque threshold is crossed
    RUN_TIME: How long the program will run before timing out and ending

    Params not needed to be tuned: 
    DT: Loop rate, how fast each iteration of the loop is (most likely not needed to be changed)
    TARGET_ORIENTATION: will change with user input, how far and in what direction the end effector moves
    *///                                                                      was  0.1
    double DT = 0.01, TORQUE_THRESHOLD = 0.4, FORCE_THRESHOLD = 15, ROTATE_ANGLE = 0.05, KEEP_CONTACT_ANGLE = 0.1, TARGET_ORIENTATION = 0.05; // Do not increase the virtual attractor angle greater than 1.55 radians
    double KEEP_CONTACT_DISTANCE = 0.015, ROTATION_ERROR_THRESHOLD = 0.03; // was 0.02
    double RUN_TIME = 60;
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // Variable for which set of parameters to use
    string param_set = "Peg";

    // MATH: Define current pose quaternion
    Eigen::Quaterniond current_pose_quat;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // This program is called and parameters are passed in like: 'rosrun behavior_algorithms orientation_targeting_with_torque_limiting _target_orientation:=2' where the number is the number of radians to rotate

    // ROS: get parameter from server, passed by command line (if nothing passed in, results in default, which is the final number)
    nh.param("/otel_y/target_orientation", TARGET_ORIENTATION, 1.57);
    nh.param<std::string>("/otel_y/param_set", param_set, "Peg");

    // clear parameter from server 
    nh.deleteParam("/otel_y/target_orientation");
    nh.deleteParam("/otel_y/param_set"); 

    // Here, the values for PULL_DISTANCE and FORCE_THRESHOLD are changed according to what setting the GUI is on for the appropriate task
    if(!strcmp(param_set.c_str(), "Peg")){
        // set the new values here
        KEEP_CONTACT_DISTANCE = 0.015;
        ROS_INFO("Params set for PEG");
    }
    else if (!strcmp(param_set.c_str(), "Bottle_Cap")){
        // set the other values here
        KEEP_CONTACT_DISTANCE = 0.015;
        ROS_INFO("Params set for BOTTLE_CAP");
    }
    else if (!strcmp(param_set.c_str(), "Tool")){
        // set the other values here
        KEEP_CONTACT_DISTANCE = 0;
        ROS_INFO("Params set for TOOL");
    }
    else if (!strcmp(param_set.c_str(), "Cutting")){
        // set the other values here
        KEEP_CONTACT_DISTANCE = 0;
        ROS_INFO("Params set for CUTTING");
    }
    else if (!strcmp(param_set.c_str(), "Task")){
        // set the other values here
        KEEP_CONTACT_DISTANCE = 0;
        ROS_INFO("Params set for TASK");
    }

    // Output what is received 
    ROS_INFO("Output from parameter for target_orientation; %f", TARGET_ORIENTATION);

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "target_orientation " << TARGET_ORIENTATION << " radians";

    srv.request.status = request_status.str();

    // ROS: Call the client start service, used in buffer.cpp for operator output
    if(client_start.call(srv)){
        // success
        cout<<"Called service_start with name succesfully"<<endl;
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service service_start");
    }

    // Set as unknown in case program somehow progresses past loop without any of the 3 conditions
    srv.request.status = "Unkown";

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.orientation.x == 0) ros::spinOnce();

    // Get starting position and turn it into a quaternion
    virtual_attractor.pose = current_pose;
    current_pose_quat.x() = current_pose.orientation.x;
    current_pose_quat.y() = current_pose.orientation.y;
    current_pose_quat.z() = current_pose.orientation.z;
    current_pose_quat.w() = current_pose.orientation.w;
    
    Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

    if (TARGET_ORIENTATION < 0){
        ROTATE_ANGLE = -ROTATE_ANGLE;
    }

    // MATH: Define the rotation matrix to the destination
    Eigen::Matrix3d TO_DESTINATION_ROTATION_MATRIX;
    TO_DESTINATION_ROTATION_MATRIX(0,0) = cos(TARGET_ORIENTATION);
    TO_DESTINATION_ROTATION_MATRIX(0,1) = 0;
    TO_DESTINATION_ROTATION_MATRIX(0,2) = sin(TARGET_ORIENTATION);
    TO_DESTINATION_ROTATION_MATRIX(1,0) = 0;
    TO_DESTINATION_ROTATION_MATRIX(1,1) = 1; 
    TO_DESTINATION_ROTATION_MATRIX(1,2) = 0;
    TO_DESTINATION_ROTATION_MATRIX(2,0) = -sin(TARGET_ORIENTATION);
    TO_DESTINATION_ROTATION_MATRIX(2,1) = 0;
    TO_DESTINATION_ROTATION_MATRIX(2,2) = cos(TARGET_ORIENTATION);


    // MATH: Define rotation matrix for moving
    Eigen::Matrix3d ROT_MAT_MOVE;
    ROT_MAT_MOVE(0,0) = cos(ROTATE_ANGLE);
    ROT_MAT_MOVE(0,1) = 0;
    ROT_MAT_MOVE(0,2) = sin(ROTATE_ANGLE);
    ROT_MAT_MOVE(1,0) = 0;
    ROT_MAT_MOVE(1,1) = 1; 
    ROT_MAT_MOVE(1,2) = 0;
    ROT_MAT_MOVE(2,0) = -sin(ROTATE_ANGLE);
    ROT_MAT_MOVE(2,1) = 0;
    ROT_MAT_MOVE(2,2) = cos(ROTATE_ANGLE);

    // Goal Rotation Matrix
    Eigen::Matrix3d goal_pose_rot = current_pose_rot * TO_DESTINATION_ROTATION_MATRIX;
    
    // Calculate the angle between the two orientatnions using quaternions
    Eigen::Quaterniond goal_pose_quat(goal_pose_rot);
    Eigen::Quaterniond diff_quat;

    diff_quat = current_pose_quat * goal_pose_quat.inverse();
    double theta = 2 * asin(abs(diff_quat.w()));

    // Print starting and target positions
    cout<<"Starting Rotation Matrix"<<endl<<current_pose_rot<<endl<<endl;
    cout<<"Goal Rotation Matrix"<<endl<<goal_pose_rot<<endl<<endl;
    cout<<"Calculated Rotation Matrix"<<endl<<TO_DESTINATION_ROTATION_MATRIX<<endl<<endl;
    
    cout<<"Angle to goal "<<(abs(theta - M_PI))<<endl<<endl;
    cout<<"W of difference quaternion: "<<diff_quat.w()<<endl<<endl;

    // Text input stop, uncomment if you want to see initial values and calculations
    // int temp;
    // cin >> temp;
    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_THRESHOLD));

    // bool within_orientation_target = ( abs(theta - M_PI) < ROTATION_ERROR_THRESHOLD );
    bool within_orientation_target = current_pose_quat.isApprox(goal_pose_quat, ROTATION_ERROR_THRESHOLD);

    // Begin loop
    /*
    Loop End Conditions:
    1. The operation has timed out (ran the max alloted time)
    2. One of the effort thresholds has been crossed
    3. The target orientation has been reached
    */
    while( (loops_so_far <= total_number_of_loops) && !effort_limit_crossed && !within_orientation_target) { 

        // ROS: for communication between programs
        ros::spinOnce();
        
        virtual_attractor.pose = current_pose;
        current_pose_quat.x() = current_pose.orientation.x;
        current_pose_quat.y() = current_pose.orientation.y;
        current_pose_quat.z() = current_pose.orientation.z;
        current_pose_quat.w() = current_pose.orientation.w;

        // Keep the robot pressing down
        // virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;

        // Convert current pose quaternion to Euler Angles TODO check?
        current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

        // Do rotation
        Eigen::Matrix3d new_virtual_attractor_rot = current_pose_rot * ROT_MAT_MOVE;
                    
        // Convert new orientation to quaternion
        Eigen::Quaterniond new_virtual_attractor_quat(new_virtual_attractor_rot);
        virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
        virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
        virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
        virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();

        // ROS: for communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();

        // Check if the effort threshold has been crossed once again (check each force and toque if they cross the threshold, if any do, update and set to true)
        effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_THRESHOLD));
                
        // Check if we arrived at the target orientation
        diff_quat = goal_pose_quat * current_pose_quat.inverse();
        theta = 2 * asin(abs(diff_quat.w()));

        // within_orientation_target = ( abs(theta - M_PI) < ROTATION_ERROR_THRESHOLD );
        within_orientation_target = current_pose_quat.isApprox(goal_pose_quat, ROTATION_ERROR_THRESHOLD);


        // Print current position
        cout<<"Current Rotation Matrix"<<endl<<current_pose_rot<<endl;
        cout<<"Target Rotation Matrix"<<endl<<goal_pose_rot<<endl<<endl;
        cout<<"Angle to goal "<<(abs(theta - M_PI))<<endl<<endl;
        cout<<"W of difference quaternion: "<<diff_quat.w()<<endl<<endl;

        // Increase counter
        loops_so_far = loops_so_far + 1;
    }
    
    // If we've crossed the effort limits, check which is crossed for the status output
    if(effort_limit_crossed) {

        // Print message
        if(abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD){
            cout<<"X Torque threshold crossed"<<endl;
            srv.request.status = "X Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD){
            cout<<"Y Torque threshold crossed"<<endl;
            srv.request.status = "Y Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD){
            cout<<"Z Torque threshold crossed"<<endl;
            srv.request.status = "Z Torque threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD){
            cout<<"X Force threshold crossed"<<endl;
            srv.request.status = "X Force threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.y) > FORCE_THRESHOLD){
            cout<<"Y Force threshold crossed"<<endl;
            srv.request.status = "Y Force threshold crossed";
        }
        else if(abs(ft_in_robot_frame.force.z) > FORCE_THRESHOLD){
            cout<<"Z Force threshold crossed"<<endl;
            srv.request.status = "Z Force threshold crossed";
        }
        else{
            cout<<"Effort threshold crossed"<<endl;
            srv.request.status = "Effort threshold crossed";
        }

        // ROS: for communication between programs
        ros::spinOnce();

        // Keep the virtual attractor slightly rotated through surface
        virtual_attractor.pose = current_pose;

        current_pose_quat.x() = current_pose.orientation.x;
        current_pose_quat.y() = current_pose.orientation.y;
        current_pose_quat.z() = current_pose.orientation.z;
        current_pose_quat.w() = current_pose.orientation.w;

        // Keep the robot pressing down
        // virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;

        // Convert current pose quaternion to Euler Angles TODO check?
        Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

        // Reverse keep contact rotation if we're rotating the other way
        if (TARGET_ORIENTATION < 0) {
            KEEP_CONTACT_ANGLE = -KEEP_CONTACT_ANGLE;
        }

        // MATH: Define rotation matrix for keeping contact
        Eigen::Matrix3d ROT_MAT_CONTACT;
        ROT_MAT_CONTACT(0,0) = cos(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(0,1) = 0;
        ROT_MAT_CONTACT(0,2) = sin(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(1,0) = 0;
        ROT_MAT_CONTACT(1,1) = 1; 
        ROT_MAT_CONTACT(1,2) = 0;
        ROT_MAT_CONTACT(2,0) = -sin(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(2,1) = 0;
        ROT_MAT_CONTACT(2,2) = cos(KEEP_CONTACT_ANGLE);

        // Do rotation
        Eigen::Matrix3d new_virtual_attractor_rot = current_pose_rot * ROT_MAT_CONTACT;
                    
        // Convert new orientation to quaternion
        Eigen::Quaterniond new_virtual_attractor_quat(new_virtual_attractor_rot);
        virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
        virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
        virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
        virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();
    }

    // If we've reached target position
    if(within_orientation_target) {
        // Print message
        cout<<"Target orientation reached"<<endl;
        srv.request.status = "Target orientation reached";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;

        // Keep the robot pressing down
        // virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;
    }

    //If we've timed out
    if (loops_so_far > total_number_of_loops){
        cout<<"Timed out"<<endl;
        srv.request.status = "Timed out";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;
    }

    // ROS: Call service to send reason for program end to buffer.cpp
    if(client.call(srv)){
        // success
        cout<<"Called service with name succesfully"<<endl;
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service status_service");
    }
         
    // ROS: for communication between programs
    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

    // End of program
}
