#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 6> Matrix36d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Controller
{
public:
    Controller(ros::NodeHandle &nh);

private:
    void ControlInputCallback();

    geometry_msgs::Transform GetTransform(std::string parent, std::string child);
    Matrix6d ComputeGeometricalJacobian(std::vector<double>& q);
    Matrix6d ComputeAnalyticalJacobian(Matrix6d Jg, Eigen::Vector3d flange_o);
    Eigen::Matrix4d A(const std::vector<double> &q, const int &j);

    // ------------------------------------- //
    // ---------------- ROS ---------------- //
    // ------------------------------------- //
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher control_input_pub;

    // Subscribers

    // Timers
    ros::Timer control_input_timer;

    // Tf listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *pTfListener;

    // Controller gains
    Eigen::Vector3d Kp;
    Eigen::Vector3d Ko;

    // DH
    const std::vector<double>   a   = {0.0, 0.444, 0.110, 0.0, 0.080, 0.0};
    const std::vector<double>   d   = {0.265, 0.0, 0.0, 0.470, 0.0, 0.101};
    const std::vector<double> alpha = {-M_PI/2.0, 0.0, -M_PI/2.0, M_PI/2.0, -M_PI/2.0, 0.0};
};

#endif