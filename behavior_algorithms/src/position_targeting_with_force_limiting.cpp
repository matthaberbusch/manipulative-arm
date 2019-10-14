// Move Until Touch with Position Targeting and Force Limiting (PTFL)
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/18/19
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

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"simple_move_until_touch");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Subscriber set_virt_attr = nh.subscribe("set_virt_attr",1,set_virt_attr_callback); //NEW
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z",1,tool_vector_callback); //NEW
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "PTFL_z";
    // Declare constants
    // TARGET_DISTANCE will change with user input

    // Medicine Bottle params: 
    // Peg in hole params: 
                    // was 0.01                                                                                      0.4 before
    double PULL_DISTANCE = 0.012, KEEP_CONTACT_DISTANCE = 0.0075, DT = 0.01, FORCE_TRESHOLD = 12, THRESHOLD_TORQUE = 0.6, TARGET_DISTANCE = 0.05; // Pull distance was 0.01, force_threshold was 13! ADDED TORQUE LIMIT
    double RUN_TIME = 15;
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // Variable for which set of parameters to use
    string param_set = "Peg";

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
    nh.param("/simple_move_until_touch/target_distance", TARGET_DISTANCE, 0.03);
    nh.param<std::string>("/simple_move_until_touch/param_set", param_set, "Peg");
    // clear parameter from server 
    nh.deleteParam("/simple_move_until_touch/target_distance"); 
    nh.deleteParam("/simple_move_until_touch/param_set");

    if(!strcmp(param_set.c_str(), "Peg")){
        // set the new values here
        PULL_DISTANCE = 0.012;
        FORCE_TRESHOLD = 12;
        ROS_INFO("Params set for PEG");
    }
    else if (!strcmp(param_set.c_str(), "Bottle_Cap")){
        // set the other values here
        PULL_DISTANCE = 0.015;
        FORCE_TRESHOLD = 15;
        ROS_INFO("Params set for BOTTLE_CAP");
    }

    ROS_INFO("Output from parameter for target_distance; %f", TARGET_DISTANCE); 

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "target_distance " << TARGET_DISTANCE << "m";

    srv.request.status = request_status.str();
    // ROS_WARN("Request Status: %s", request_status.str().c_str());

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
    while(current_pose.position.x == 0) ros::spinOnce();

    // Get starting position
    double start_position = abs(current_pose.position.x);
    double target_position = start_position + TARGET_DISTANCE;

    // Print starting and target positions
    cout<<"Starting position: "<<endl<<start_position<<endl;
    cout<<"Target position: "<<endl<<target_position<<endl;

    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > THRESHOLD_TORQUE) || (abs(ft_in_robot_frame.torque.y) > THRESHOLD_TORQUE) || (abs(ft_in_robot_frame.torque.z) > THRESHOLD_TORQUE) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_TRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_TRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_TRESHOLD));

    // Begin loop
    // Assuming we're always going in the positive x direction.
    // While we haven't touched anything and haven't reached our target
    while( (loops_so_far <= total_number_of_loops) && !effort_limit_crossed && (((abs(current_pose.position.x) < target_position) && (TARGET_DISTANCE >= 0)) || ((abs(current_pose.position.x) >= target_position) && (TARGET_DISTANCE < 0))) ) {
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep virtual attractor at a distance, to pull the end effector
        virtual_attractor.pose = current_pose;
 
        // Pull down in the direction of the tool
        if(TARGET_DISTANCE > 0){
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_z.x * PULL_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_z.y * PULL_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_z.z * PULL_DISTANCE;
        }
        // Pull up in the direction of the tool
        else {
            virtual_attractor.pose.position.x = current_pose.position.x - tool_vector_z.x * PULL_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y - tool_vector_z.y * PULL_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z - tool_vector_z.z * PULL_DISTANCE;
        }

        // ROS: for communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();

        // Update the values for the loop condition
        effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > THRESHOLD_TORQUE) || (abs(ft_in_robot_frame.torque.y) > THRESHOLD_TORQUE) || (abs(ft_in_robot_frame.torque.z) > THRESHOLD_TORQUE) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_TRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_TRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_TRESHOLD));

        loops_so_far = loops_so_far + 1;

        // Print current position
        // cout<<"Current position: "<<endl<<abs(current_pose.position.x)<<endl;
    }
    
    // If we've touched
    if(effort_limit_crossed) {
        // Print message
        cout<<"Effort threshold crossed"<<endl;
        srv.request.status = "Effort threshold crossed";
        // ROS: for communication between programs
        ros::spinOnce();

        // Keep the virtual attractor slightly below the surface, or above if pulling back
        virtual_attractor.pose = current_pose;
        if (abs(current_pose.position.x) < target_position){
            virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;
        }
        else {
            virtual_attractor.pose.position.x = current_pose.position.x - KEEP_CONTACT_DISTANCE;
        }
    }

    // If we've reached target position
    if((abs(current_pose.position.x) >= target_position && (TARGET_DISTANCE > 0) ) || (abs(current_pose.position.x) < target_position && (TARGET_DISTANCE <= 0) ) ) {
        // Print message
        cout<<"Target position reached"<<endl;
        srv.request.status = "target position reached";
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

    if(client.call(srv)){
        // success
        cout<<"Called service_exit with name succesfully"<<endl;
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