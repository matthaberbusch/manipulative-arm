
#include <cmath> //do i need this?

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/QR>
#include <Eigen/Dense>
//#include <iostream>

//Declare variables
double dt_ = 0.01;
geometry_msgs::Pose myCartPose; //just poses right?
geometry_msgs::Pose myAttrPose;
geometry_msgs::PoseStamped wholeCartPose;

//Methods
void cartIn(const geometry_msgs::PoseStamped& cart_pos) {
    myCartPose = cart_pos.pose;
    wholeCartPose = cart_pos;
}

void attrIn(const geometry_msgs::PoseStamped& attr_pose) {
    //these are not values from the sensor. They are f/t values transformed into robot base frame
    myAttrPose = attr_pose.pose;
}

//Subscriber and Publisher
int main(int argc, char** argv) {
    ros::init(argc,argv,"peg_hole_spiral_search");
    ros::NodeHandle nh;
    ros::Subscriber cart_sub = nh.subscribe("cartesian_logger",1, cartIn); //where I am
    ros::Subscriber attr_sub = nh.subscribe("tfd_virt_attr",1,attrIn); // where do I want to go
    ros::Publisher cart_pub = nh.advertise<std_msgs::Float64>("cart_x_angle",1);
    ros::Publisher attr_pub = nh.advertise<std_msgs::Float64>("attr_x_angle",1);
    ros::Publisher repeater = nh.advertise< geometry_msgs::PoseStamped>("repeater",1);
//Loop

    ros::Rate naptime(1/dt_);
    while(ros::ok()) {
        ros::spinOnce(); // ALWAYS REMEMBER THIS >:(
    	Eigen::Quaterniond myCartPoseQuat;
        myCartPoseQuat.x() = myCartPose.orientation.x;
        myCartPoseQuat.y() = myCartPose.orientation.y;
        myCartPoseQuat.z() = myCartPose.orientation.z;
        myCartPoseQuat.w() = myCartPose.orientation.w;
        // to rotation matrix
        Eigen::Matrix3d myCartRotMatrix = myCartPoseQuat.normalized().toRotationMatrix();
        //to eulerAngles!
        Eigen::Vector3d myCartEulerAngles = myCartRotMatrix.eulerAngles(0,1,2);
        //what does it look like
       // cout<<"myCartEulerAngles"<<endl<<myCartEulerAngles<<endl;

        Eigen::Quaterniond myAttrPoseQuat;
        myAttrPoseQuat.x() = myAttrPose.orientation.x;
        myAttrPoseQuat.y() = myAttrPose.orientation.y;
        myAttrPoseQuat.z() = myAttrPose.orientation.z;
        myAttrPoseQuat.w() = myAttrPose.orientation.w;
        // to rotation matrix
        Eigen::Matrix3d myAttrRotMatrix = myAttrPoseQuat.normalized().toRotationMatrix();
        //to eulerAngles!
        Eigen::Vector3d myAttrEulerAngles = myAttrRotMatrix.eulerAngles(2,1,0); // tried (0,1,2)
        //what does it look like
        // cout<<"myAttrEulerAngles"<<endl<<myAttrEulerAngles<<endl;

        //f
        std_msgs::Float64 cart_x_angle;
        cart_x_angle.data = myCartEulerAngles(0); //but is this a float?
        std_msgs::Float64 attr_x_angle;
        attr_x_angle.data = myAttrEulerAngles(0);


    	cart_pub.publish(cart_x_angle);
        attr_pub.publish(attr_x_angle);
        repeater.publish(wholeCartPose);
        naptime.sleep();
    }
}