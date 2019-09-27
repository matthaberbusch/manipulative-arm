// Rotate until torque with Orientation Targeting and Torque Limiting (PTFL)
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/18/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
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
    ros::init(argc,argv,"orientation_targeting_with_torque_limiting");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);
    ros::Subscriber joint_state_sub = nh.subscribe("abb120_joint_state",1,jointStateCallback);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "OTTL";

    // Declare constants
    // TARGET_ORIENTATION will change with user input
    double DT = 0.01, THRESHOLD_TORQUE = 0.4, ROTATE_ANGLE = 0.5, KEEP_CONTACT_ANGLE = 0.1, TARGET_ORIENTATION = 0.05; // Do not increase the virtual attractor angle greater than 1.55 radians
    double KEEP_CONTACT_DISTANCE = 0.015;
    double RUN_TIME = 10;
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // MATH: Define current pose quaternion
    Eigen::Quaterniond current_pose_quat;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // Get user input
    // cout << "Enter a rotation to move in radians: ";
    // cin >> TARGET_ORIENTATION;

    // Replaced user input with cmd line in
    // ros::NodeHandle n("~"); // refer to the global path to the local parameters

    // called and passed in like: 'rosrun behavior_algorithms orientation_targeting_with_torque_limiting _target_orientation:=2'

    // get parameter from server, passed by command line (if nothing passed in, results in default)
    nh.param("/orientation_targeting_with_torque_limiting/target_orientation", TARGET_ORIENTATION, 1.57);
    
    // clear parameter from server 
    nh.deleteParam("/orientation_targeting_with_torque_limiting/target_orientation"); 

    ROS_INFO("Output from parameter for target_orientation; %f", TARGET_ORIENTATION);

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.orientation.x == 0) ros::spinOnce();

    // Get starting position and turn it into a quaternion
    virtual_attractor.pose = current_pose;
    current_pose_quat.x() = current_pose.orientation.x;
    current_pose_quat.y() = current_pose.orientation.y;
    current_pose_quat.z() = current_pose.orientation.z;
    current_pose_quat.w() = current_pose.orientation.w;
    
    Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();
    double current_orientation = joint_states(5);
    double start_orientation = current_orientation;
    double target_orientation = start_orientation + TARGET_ORIENTATION;

    if (TARGET_ORIENTATION < 0){
        ROTATE_ANGLE = -ROTATE_ANGLE;
    }

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

    // Print starting and target positions
    cout<<"current_orientation: "<<endl<<current_orientation<<endl;
    cout<<"Starting orientation: "<<endl<<start_orientation<<endl;
    cout<<"Target orientation: "<<endl<<target_orientation<<endl;

    // Begin loop
    // While we haven't touched anything and haven't reached our target
    while( (loops_so_far <= total_number_of_loops) && (abs(ft_in_robot_frame.torque.x) < THRESHOLD_TORQUE) && (((current_orientation < target_orientation) && (TARGET_ORIENTATION >= 0)) || ((current_orientation >= target_orientation) && (TARGET_ORIENTATION < 0)))) { 

        // ROS: for communication between programs
        ros::spinOnce();
        
        virtual_attractor.pose = current_pose;
        current_pose_quat.x() = current_pose.orientation.x;
        current_pose_quat.y() = current_pose.orientation.y;
        current_pose_quat.z() = current_pose.orientation.z;
        current_pose_quat.w() = current_pose.orientation.w;

        // Keep the robot pressing down
        virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;

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

        // Print current position
        current_orientation = joint_states(5);
        cout<<"Current orientation: "<<endl<<current_orientation<<endl;

        // Increase counter
        loops_so_far = loops_so_far + 1;
    }
    
    // If we've touched
    if(abs(ft_in_robot_frame.torque.x) >= THRESHOLD_TORQUE) {
        // Print message
        cout<<"Torque threshold crossed"<<endl;
        srv.request.status = "Torque threshold crossed";
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep the virtual attractor slightly rotated through surface
        virtual_attractor.pose = current_pose;

        current_pose_quat.x() = current_pose.orientation.x;
        current_pose_quat.y() = current_pose.orientation.y;
        current_pose_quat.z() = current_pose.orientation.z;
        current_pose_quat.w() = current_pose.orientation.w;

        // Keep the robot pressing down
        virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;

        // Convert current pose quaternion to Euler Angles TODO check?
        Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

        // Reverse keep contact rotation if we're rotating the other way
        if (TARGET_ORIENTATION < 0) {
            KEEP_CONTACT_ANGLE = -KEEP_CONTACT_ANGLE;
        }

        // MATH: Define rotation matrix for keeping contact
        Eigen::Matrix3d ROT_MAT_CONTACT; 
        ROT_MAT_CONTACT(0,0) = cos(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(0,1) = -sin(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(0,2) = 0;
        ROT_MAT_CONTACT(1,0) = sin(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(1,1) = cos(KEEP_CONTACT_ANGLE);
        ROT_MAT_CONTACT(1,2) = 0;
        ROT_MAT_CONTACT(2,0) = 0;
        ROT_MAT_CONTACT(2,1) = 0;
        ROT_MAT_CONTACT(2,2) = 1;

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
    if(((current_orientation >= target_orientation) && (TARGET_ORIENTATION >= 0)) || ((current_orientation < target_orientation) && (TARGET_ORIENTATION < 0)) ) {
        // Print message
        cout<<"Target orientation reached"<<endl;
        srv.request.status = "Target orientation reached";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;

        // Keep the robot pressing down
        virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;
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

    if(client.call(srv)){
        // success
        cout<<"Called service with name succesfully";
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