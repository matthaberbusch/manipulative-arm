// Simple Move Until Touch Program
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/11/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>

// Declare variables
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"simple_move_until_touch");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    // Declare constants
    double MOVE_DISTANCE = 0.05, KEEP_CONTACT_DISTANCE = 0.05, DT = 0.01, FORCE_TRESHOLD = 1;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // Start loop
    while(ros::ok()) {
        // ROS: for communication between programs
        ros::spinOnce();

        // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

        // If we haven't touched yet
        if(abs(ft_in_robot_frame.force.x) < FORCE_TRESHOLD) {
            // Keep virtual attractor at a distance, to pull the end effector
            virtual_attractor.pose = current_pose;
            virtual_attractor.pose.position.x = current_pose.position.x + MOVE_DISTANCE;
        }
        // After we've touched 
        else {
            // Keep the virtual attractor slightly below the surface
            virtual_attractor.pose = current_pose;
            virtual_attractor.pose.position.x = current_pose.position.x + KEEP_CONTACT_DISTANCE;
        }   
        // ROS: for communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }
}