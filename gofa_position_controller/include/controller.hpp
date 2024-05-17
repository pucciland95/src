#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"

class Controller {
    public:
        Controller(ros::NodeHandle& nh);
    private:

        void ControlInputCallback();

        // ------------------------------------- //
        // ---------------- ROS ---------------- //
        // ------------------------------------- //
        ros::NodeHandle nh;

        // Publishers
        ros::Publisher robot_control_input;

        // Subscribers
        
        // Timers
        ros::Timer control_input_timer;

        // Tf listener
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener* pTfListener;

};

#endif