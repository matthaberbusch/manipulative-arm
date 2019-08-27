// Torsional Wiggle
// Matthew Haberbusch and Surag Balajepalli 
// Last updated 8/7/19
// 
// All ROS-specific code labeled with "ROS:" comments

// ROS: include libraries
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>
#include <Eigen/QR>
#include <Eigen/Dense>
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
void set_virt_attr_callback(const geometry_msgs::PoseStamped& set_attr) {
    set_attr_pose = set_attr.pose;
}
void tool_vector_z_callback(const geometry_msgs::Vector3& tool_vector_msg_z) {
    tool_vector_z = tool_vector_msg_z;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"simple_move_until_touch");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Subscriber set_virt_attr = nh.subscribe("set_virt_attr",1,set_virt_attr_callback);
    ros::Subscriber tool_vector_sub_z = nh.subscribe("tool_vector_z",1,tool_vector_z_callback);
    ros::Publisher virtual_attractor_publisher = nh.advertise<geometry_msgs::PoseStamped>("Virt_attr_pose",1);

    // Declare constants
    double KEEP_CONTACT_DISTANCE = 0.01, DT = 0.01;
    double WIGGLE_RADIUS = 0.01, WIGGLE_RATE = 5, WIGGLE_TIME = 5;
    double current_loop = 0;
    double current_loop_of_state = 0;
    double current_state = 1;
    double STATES = 2;
    double ROTATE_ANGLE = 0.5;

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // ROS: for communication between programs
    ros::spinOnce();
    naptime.sleep();

    // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

    // Get user input
    cout << "Enter an amount of time to wiggle in seconds: ";
    cin >> WIGGLE_TIME;

    // Calculate the total loops needed and loops between states
    double MAX_LOOPS = WIGGLE_TIME / DT;
    double LOOPS_BETWEEN_STATES = MAX_LOOPS / (WIGGLE_RATE * STATES); 
    cout<<"MAX_LOOPS"<<endl<<MAX_LOOPS<<endl;
    cout<<"LOOPS_BETWEEN_STATES"<<endl<<LOOPS_BETWEEN_STATES<<endl;

    // ROS: Wait until we have position data. Our default position is 0.
    while(current_pose.position.x == 0) ros::spinOnce();

    // MATH: Define current pose quaternion
    Eigen::Quaterniond current_pose_quat;

    // Get starting position and turn it into a quaternion
    virtual_attractor.pose = current_pose;
    current_pose_quat.x() = current_pose.orientation.x;
    current_pose_quat.y() = current_pose.orientation.y;
    current_pose_quat.z() = current_pose.orientation.z;
    current_pose_quat.w() = current_pose.orientation.w;
    
    Eigen::Matrix3d current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

    // MATH: Define rotation matrix for moving WRT TOOL FRAME
    Eigen::Matrix3d ROT_MAT_MOVE_RIGHT;
    ROT_MAT_MOVE_RIGHT(0,0) = cos(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(0,1) = -sin(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(0,2) = 0;
    ROT_MAT_MOVE_RIGHT(1,0) = sin(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(1,1) = cos(ROTATE_ANGLE);
    ROT_MAT_MOVE_RIGHT(1,2) = 0;
    ROT_MAT_MOVE_RIGHT(2,0) = 0;
    ROT_MAT_MOVE_RIGHT(2,1) = 0;
    ROT_MAT_MOVE_RIGHT(2,2) = 1;

    Eigen::Matrix3d ROT_MAT_MOVE_LEFT;
    ROT_MAT_MOVE_LEFT(0,0) = cos(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(0,1) = -sin(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(0,2) = 0;
    ROT_MAT_MOVE_LEFT(1,0) = sin(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(1,1) = cos(-ROTATE_ANGLE);
    ROT_MAT_MOVE_LEFT(1,2) = 0;
    ROT_MAT_MOVE_LEFT(2,0) = 0;
    ROT_MAT_MOVE_LEFT(2,1) = 0;
    ROT_MAT_MOVE_LEFT(2,2) = 1;

    // Keep virtual attractor at a distance, to push the end effector
    virtual_attractor.pose = current_pose;

    // Start loop
    while(current_loop <= MAX_LOOPS){
        ros::spinOnce();
        
        // If we spent enough time in one state, move to the next
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

        // Rotate right
        if (current_state == 1){
            virtual_attractor.pose = current_pose;
            current_pose_quat.x() = current_pose.orientation.x;
            current_pose_quat.y() = current_pose.orientation.y;
            current_pose_quat.z() = current_pose.orientation.z;
            current_pose_quat.w() = current_pose.orientation.w;

            // Keep the robot pressing down
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_z.z * KEEP_CONTACT_DISTANCE;

            // Convert current pose quaternion to Euler Angles
            current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

            // Do rotation
            Eigen::Matrix3d new_virtual_attractor_rot = current_pose_rot * ROT_MAT_MOVE_RIGHT;
                    
            // Convert new orientation to quaternion
            Eigen::Quaterniond new_virtual_attractor_quat(new_virtual_attractor_rot);
            virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
            virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
            virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
            virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();
        }

        // Rotate left
        else if (current_state == 2){
            virtual_attractor.pose = current_pose;
            current_pose_quat.x() = current_pose.orientation.x;
            current_pose_quat.y() = current_pose.orientation.y;
            current_pose_quat.z() = current_pose.orientation.z;
            current_pose_quat.w() = current_pose.orientation.w;

            // Keep the robot pressing down
            virtual_attractor.pose.position.x = current_pose.position.x + tool_vector_z.x * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.y = current_pose.position.y + tool_vector_z.y * KEEP_CONTACT_DISTANCE;
            virtual_attractor.pose.position.z = current_pose.position.z + tool_vector_z.z * KEEP_CONTACT_DISTANCE;

            // Convert current pose quaternion to Euler Angles
            current_pose_rot = current_pose_quat.normalized().toRotationMatrix();

            // Do rotation
            Eigen::Matrix3d new_virtual_attractor_rot = current_pose_rot * ROT_MAT_MOVE_LEFT;

            // Convert new orientation to quaternion
            Eigen::Quaterniond new_virtual_attractor_quat(new_virtual_attractor_rot);
            virtual_attractor.pose.orientation.x = new_virtual_attractor_quat.x();
            virtual_attractor.pose.orientation.y = new_virtual_attractor_quat.y();
            virtual_attractor.pose.orientation.z = new_virtual_attractor_quat.z();
            virtual_attractor.pose.orientation.w = new_virtual_attractor_quat.w();
        }

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
    cout<<"End"<<endl;

    // End of program
}