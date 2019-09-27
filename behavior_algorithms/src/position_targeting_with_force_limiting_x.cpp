// Move Until Touch with Position Targeting and Force Limiting (PTFL)
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
#include <behavior_algorithms/status_service.h>
using namespace std;

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;
//NEW
geometry_msgs::Pose set_attr_pose;
geometry_msgs::Vector3 tool_vector_x;
geometry_msgs::Vector3 tool_vector_y;
geometry_msgs::Vector3 tool_vector_z;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}
// NEW
void set_virt_attr_callback(const geometry_msgs::PoseStamped& set_attr) {
    set_attr_pose = set_attr.pose;
}
void tool_vector_callback(const geometry_msgs::Vector3& tool_vector_msg_z) {
    tool_vector_z = tool_vector_msg_z;
}
void tool_vector_y_callback(const geometry_msgs::Vector3& tool_vector_msg_y) {
    tool_vector_y = tool_vector_msg_y;
}
void tool_vector_x_callback(const geometry_msgs::Vector3& tool_vector_msg_x) {
    tool_vector_x = tool_vector_msg_x;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"ptfl_x");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Subscriber set_virt_attr = nh.subscribe("set_virt_attr",1,set_virt_attr_callback); //NEW
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z",1,tool_vector_callback); //NEW
    ros::Subscriber tool_vector_sub_y = nh.subscribe("tool_vector_y",1,tool_vector_y_callback); //NEW
    ros::Subscriber tool_vector_sub_x = nh.subscribe("tool_vector_x",1,tool_vector_x_callback); //NEW
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);


    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "PTFL_x";
    // Declare constants
    // TARGET_DISTANCE will change with user input

    // Medicine Bottle params: 
    // Peg in hole params: 

    double PULL_DISTANCE = 0.01, KEEP_CONTACT_DISTANCE = 0.0075, DT = 0.01, FORCE_TRESHOLD = 12, TARGET_DISTANCE = 0.05; // Pull distance was 0.01, force_threshold was 13!
    double RUN_TIME = 30;
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // Get user input
    // cout << "Enter a distance to move in meters: ";
    // cin >> TARGET_DISTANCE;
    
    // Get parameter passed in through 
    nh.param("/ptfl_x/target_distance", TARGET_DISTANCE, -0.03);
    
    // clear parameter from server 
    nh.deleteParam("/ptfl_x/target_distance"); 

    ROS_INFO("Output from parameter for target_distance; %f", TARGET_DISTANCE);

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    // Get starting position
    double start_position = abs(current_pose.position.y);
    double target_position = start_position + TARGET_DISTANCE;

    // Print starting and target positions
    cout<<"Starting position: "<<endl<<start_position<<endl;
    cout<<"Target position: "<<endl<<target_position<<endl;

    // Begin loop
    // Assuming we're always going in the positive x direction.
    // While we haven't touched anything and haven't reached our target
    while( (loops_so_far <= total_number_of_loops) && (abs(ft_in_robot_frame.force.y) < FORCE_TRESHOLD) && (((abs(current_pose.position.y) < target_position) && (TARGET_DISTANCE >= 0)) || ((abs(current_pose.position.y) >= target_position) && (TARGET_DISTANCE < 0))) ) {
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep virtual attractor at a distance, to pull the end effector
        virtual_attractor.pose = current_pose;
 
        // Pull down in the direction of the tool
        if(TARGET_DISTANCE > 0){
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_x.x * PULL_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_x.y * PULL_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_x.z * PULL_DISTANCE;
        }
        // Pull up in the direction of the tool
        else {
            virtual_attractor.pose.position.x = current_pose.position.x - tool_vector_x.x * PULL_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y - tool_vector_x.y * PULL_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z - tool_vector_x.z * PULL_DISTANCE;
        }

        // ROS: for communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();

        loops_so_far = loops_so_far + 1;

        // Print current position
        cout<<"Current position: "<<endl<<abs(current_pose.position.y)<<endl;
    }
    
    // If we've touched
    if(abs(ft_in_robot_frame.force.y) >= FORCE_TRESHOLD) {
        // Print message
        cout<<"Force threshold crossed"<<endl;
        srv.request.status = "Force threshold crossed";
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep the virtual attractor slightly below the surface, or above if pulling back
        virtual_attractor.pose = current_pose;
        if (abs(current_pose.position.y) < target_position){
            virtual_attractor.pose.position.y = current_pose.position.y + KEEP_CONTACT_DISTANCE;
        }
        else {
            virtual_attractor.pose.position.y = current_pose.position.y - KEEP_CONTACT_DISTANCE;
        }
    }

    // If we've reached target position
    if((abs(current_pose.position.y) >= target_position && (TARGET_DISTANCE > 0) ) || (abs(current_pose.position.y) < target_position && (TARGET_DISTANCE <= 0) ) ) {
        // Print message
        cout<<"Target position reached"<<endl;
        srv.request.status = "Target position reached";
        // ROS: for communication between programs
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose = current_pose;
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

    // ROS: for communication between programs
    if(client.call(srv)){
        // success
        cout<<"Called service with name succesfully";
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service status_service");
    }

    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

    // End of program
}