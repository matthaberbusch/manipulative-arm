// Translational Wiggle
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 7/24/19
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
void tool_vector_x_callback(const geometry_msgs::Vector3& tool_vector_msg_x) {
    tool_vector_x = tool_vector_msg_x;
}
void tool_vector_y_callback(const geometry_msgs::Vector3& tool_vector_msg_y) {
    tool_vector_y = tool_vector_msg_y;
}
void tool_vector_z_callback(const geometry_msgs::Vector3& tool_vector_msg_z) {
    tool_vector_z = tool_vector_msg_z;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"translational_wiggle_push");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Subscriber set_virt_attr = nh.subscribe("set_virt_attr",1,set_virt_attr_callback);
    ros::Subscriber tool_vector_sub_x = nh.subscribe("tool_vector_x",1,tool_vector_x_callback);
    ros::Subscriber tool_vector_sub_y = nh.subscribe("tool_vector_y",1,tool_vector_y_callback);
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z",1,tool_vector_z_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    ros::ServiceClient client = nh.serviceClient<behavior_algorithms::status_service>("status_service");
    ros::ServiceClient client_start = nh.serviceClient<behavior_algorithms::status_service>("start_service");
    behavior_algorithms::status_service srv;
    srv.request.name = "Translational Wiggle Push";

    // Declare constants
    // TARGET_DISTANCE will change with user input
    double KEEP_CONTACT_DISTANCE = 0.01, DT = 0.01;
    double WIGGLE_RADIUS = 0.01, WIGGLE_RATE = 3, WIGGLE_TIME = 5;
    double TORQUE_THRESHOLD = 0.4, FORCE_THRESHOLD = 15;
    double current_loop = 0;
    double current_loop_of_state = 0;
    double current_state = 1;
    double STATES = 4;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // Get user input
    // cout << "Enter an amount of time to wiggle in seconds: ";
    // cin >> WIGGLE_TIME;

    nh.param("/translational_wiggle_push/wiggle_time", WIGGLE_TIME, 5.0); 

    // clear parameter from server 
    nh.deleteParam("/translational_wiggle_push/wiggle_time"); 
    
    ROS_INFO("Output from parameter for runtime; %f", WIGGLE_TIME);

    // With labeled parameter, now call service to send message that program will start
    std::ostringstream request_status; 
    request_status << "runtime " << WIGGLE_TIME << " seconds";

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

    double MAX_LOOPS = WIGGLE_TIME / DT;

    // Updated for frequency
    double LOOPS_BETWEEN_STATES = 1 / (DT * WIGGLE_RATE * STATES); //  MAX_LOOPS / (WIGGLE_RATE * STATES);
    cout<<"MAX_LOOPS"<<endl<<MAX_LOOPS<<endl;
    cout<<"LOOPS_BETWEEN_STATES"<<endl<<LOOPS_BETWEEN_STATES<<endl;

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    // Keep virtual attractor at a distance, to pull the end effector
    virtual_attractor.pose = current_pose;

    // Loop variable to check effort limit condition
    bool effort_limit_crossed = false;
    effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_THRESHOLD));

    // Start main Loop
    while(current_loop <= MAX_LOOPS && !effort_limit_crossed){

        //ROS: To get callback data
        ros::spinOnce();

        // change state
        if(current_loop_of_state >= LOOPS_BETWEEN_STATES){
            if(current_state < STATES){
                current_state = current_state + 1;
                current_loop_of_state = 0;
            }
            else{
                current_state = 1;
                current_loop_of_state = 0;
            }
        }

        // Move to one of four positions
        if (current_state == 1){
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_x.x * 1 * WIGGLE_RADIUS + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_x.y * 1 * WIGGLE_RADIUS + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_x.z * 1 * WIGGLE_RADIUS + tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }
        else if (current_state == 2){
            virtual_attractor.pose.position.x = current_pose.position.x - tool_vector_y.x * 1 * WIGGLE_RADIUS + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y - tool_vector_y.y * 1 * WIGGLE_RADIUS + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z - tool_vector_y.z * 1 * WIGGLE_RADIUS + tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }
        else if (current_state == 3){
            virtual_attractor.pose.position.x = current_pose.position.x - tool_vector_x.x * 1 * WIGGLE_RADIUS + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y - tool_vector_x.y * 1 * WIGGLE_RADIUS + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z - tool_vector_x.z * 1 * WIGGLE_RADIUS + tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }
        else if (current_state == 4){
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_y.x * 1 * WIGGLE_RADIUS + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_y.y * 1 * WIGGLE_RADIUS + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_y.z * 1 * WIGGLE_RADIUS + tool_vector_z.z * KEEP_CONTACT_DISTANCE;
        }

        effort_limit_crossed = ((abs(ft_in_robot_frame.torque.x) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.y) > TORQUE_THRESHOLD) || (abs(ft_in_robot_frame.torque.z) > TORQUE_THRESHOLD) ||
                                 (abs(ft_in_robot_frame.force.x) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.y) > FORCE_THRESHOLD) || (abs(ft_in_robot_frame.force.z) > FORCE_THRESHOLD));

        // Increment counters
        current_loop = current_loop + 1;
        current_loop_of_state = current_loop_of_state + 1;

        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }

    // Set final attractor pose to current ee position
    virtual_attractor.pose = current_pose;
    virtual_attractor_publisher.publish(virtual_attractor);
    naptime.sleep();

    srv.request.status = "Completed run of specified length";

    if(effort_limit_crossed){

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
    }

    if(client.call(srv)){
        // success
        cout<<"Called service with name succesfully"<<endl;
    }
    else{
        // failed to call service
        ROS_ERROR("Failed to call service status_service");
    }
    
    cout<<"End"<<endl;

    // End of program
}