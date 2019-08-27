// score_calculator.cpp
// calculates scores from test bags, displays and publishes score
// Matthew Haberbusch
// 6/12/19

//TODO remove the ones I don't need
#include <irb120_accomodation_control/irb120_accomodation_control.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <cmath>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>


// Declare variables
std_msgs::Float64 total_score_msg;
geometry_msgs::Pose current_pose;
geometry_msgs::PoseStamped virtual_attractor;
geometry_msgs::Wrench ft_in_robot_frame;
bool test_running = false;

// ROS: callback functions for how we receive data
void cartesian_state_callback(const geometry_msgs::PoseStamped& cartesian_pose) {
    current_pose = cartesian_pose.pose;
    // If we're getting data, the test bag is running
    // TODO change how we do this for live tests
    test_running = true;
}
void ft_callback(const geometry_msgs::Wrench& ft_values) {
    // These are not values from the sensor. They are f/t values transformed into robot base frame.
    ft_in_robot_frame = ft_values;
    // If we're getting data, the test bag is running
    // TODO change how we do this for live tests
    test_running = true;
}

// ROS: main program
int main(int argc, char** argv) {
    // ROS: for communication between programs
    ros::init(argc,argv,"score_calculator");
    ros::NodeHandle nh;
    ros::Subscriber cartesian_state_subscriber = nh.subscribe("cartesian_logger",1, cartesian_state_callback);
    ros::Subscriber ft_subscriber = nh.subscribe("transformed_ft_wrench",1,ft_callback);
    ros::Publisher score_publisher = nh.advertise<std_msgs::Float64>("score",1); //TODO is Float64 right?

    // Declare constants
    double IMPULSE_GAIN = 1, TIME_GAIN = 1, VELOCITY_GAIN = 1, DT = 0.01;

    double force = 0;
    double impulse = 0;
    double total_force = 0;
    double total_impulse = 0;
    double total_time = 0;
    double distance = 0;
    double velocity = 0;
    double total_velocity = 0;
    double total_score = 0;
    double last_position_x = current_pose.position.x;
    double last_position_y = current_pose.position.y;
    double last_position_z = current_pose.position.z;
    

    // ROS: for loop rate
    ros::Rate naptime(1/DT);

    // Start loop
    while(ros::ok()) {
        // ROS: for communication between programs
        ros::spinOnce();
        if(test_running) {

            // The end effector pose (current_pose) and force torque data (ft_in_robot_frame) are global variables.

            // Add up forces
            force = abs(ft_in_robot_frame.force.x) + abs(ft_in_robot_frame.force.y) + abs(ft_in_robot_frame.force.z);
            impulse = force*DT;
            total_force = total_force + force;
            total_impulse = total_impulse + impulse;
            cout<<"total impulse: "<<endl<<total_impulse<<endl;

            total_time = total_time + DT; //TODO replace DT with a ros thing
            cout<<"total time: "<<endl<<total_time<<endl;

            distance = sqrt(pow((current_pose.position.x - last_position_x),2) + pow((current_pose.position.y - last_position_y),2) + pow((current_pose.position.z - last_position_z),2));
            velocity = distance/DT;
            // If this is the first posistion change, don't include it
            if(last_position_x == 0 && last_position_y == 0 && last_position_z == 0) {
                velocity = 0;
            }
            total_velocity = total_velocity + velocity;
            cout<<"total velocity: "<<endl<<total_velocity<<endl;

            // Then reset previous positions
            last_position_x = current_pose.position.x;
            last_position_y = current_pose.position.y;
            last_position_z = current_pose.position.z;

            total_score = IMPULSE_GAIN*total_impulse + TIME_GAIN*total_time + VELOCITY_GAIN*total_velocity;
            cout<<"total score: "<<endl<<total_score<<endl;

            total_score_msg.data = total_score;
            score_publisher.publish(total_score_msg);
        }
        // Assume the bag isn't running unless a callback happened
        // TODO change this for live tests. Will this skip time?
        test_running = false;
        naptime.sleep();
    }
}