#include "../include/controller.hpp"

void Controller::ControlInputCallback()
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("base", "flange", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return;
    }
}

Controller::Controller(ros::NodeHandle& nh)
{
    this->nh = nh;

    // Publisher
    robot_control_input = nh.advertise<std_msgs::Float64MultiArray>("/VelocityControllers_JointGroupVelocityController/command", 100); 

    // Creating the TF listener
    this->pTfListener = new tf2_ros::TransformListener(this->tfBuffer);
 
    this->control_input_timer = this->nh.createTimer(ros::Duration(0.01), std::bind(&Controller::ControlInputCallback, this) );
}

