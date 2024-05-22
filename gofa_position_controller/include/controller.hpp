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
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Controller
{
public:
    Controller(ros::NodeHandle &nh);

private:
    void ControlInputCallback();

    geometry_msgs::Transform GetTransform(std::string parent, std::string child);
    Matrix6d ComputeGeometricalJacobian();
    Eigen::Vector3d ComputeOrientationError(Eigen::Matrix3d& R_flange, Eigen::Matrix3d& R_desiredPose);

    // ------------------------------------- //
    // ---------------- ROS ---------------- //
    // ------------------------------------- //
    ros::NodeHandle nh;

    // Publishers
    ros::Publisher joint_velocity_pub;

    // Subscribers

    // Timers
    ros::Timer control_input_timer;

    // Tf listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *pTfListener;

    // ------------------------------------- //
    // --------- Class parameters ---------- //
    // ------------------------------------- //
    // TF names: with respect to which TF to move,\
                 the TF to move (robot's tcp) and \
                 the TF to reach (desired pose)
    std::string tf_base_name;
    std::string tf_tcp_name;
    std::string tf_desired_name;

    // Controller gains
    Eigen::Vector3d Kp;
    Eigen::Vector3d Ko;
};

#endif