// Wiggle
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 6/27/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
using namespace std;

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
    ros::init(argc,argv,"wiggle");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    // Declare constants
    double DT = 0.01, RUN_TIME = 1;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // Calculate the run time
    cout << "Enter a run time in seconds: ";
    cin >> RUN_TIME;
    double total_number_of_loops = RUN_TIME / DT;
    double loops_so_far = 0;

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    double starting_point = current_pose.position.x;
    
    // Begin loop
    while(loops_so_far <= total_number_of_loops) {
        // ROS: For communication
        ros::spinOnce();

        // Put the virtual attractor at the end effector
        virtual_attractor.pose.position.x = starting_point + (sin((loops_so_far * 6.282 * DT * 0.001))); // 1 cm!

        // Add to time counter
        loops_so_far = loops_so_far + 1;
         
        // ROS: For communication between programs
        virtual_attractor_publisher.publish(virtual_attractor);
        naptime.sleep();
    }
    
    // End of program
    cout<<"Done"<<endl;

}