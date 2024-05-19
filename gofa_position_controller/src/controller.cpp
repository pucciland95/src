#include "../include/controller.hpp"

Eigen::Matrix4d Controller::A(const std::vector<double> &q, const int &j)
{
    // Subscript j = Index j
    // Subscript i = Index j + 1
    Eigen::Matrix4d Aji;
    Aji << cos(q[j]), -sin(q[j]) * cos(this->alpha[j]), sin(q[j]) * sin(this->alpha[j]), this->a[j] * cos(q[j]),
        sin(q[j]), cos(q[j]) * cos(this->alpha[j]), -cos(q[j]) * sin(this->alpha[j]), this->a[j] * sin(q[j]),
        0, sin(this->alpha[j]), cos(this->alpha[j]), this->d[j],
        0, 0, 0, 1;
    return Aji;
}

geometry_msgs::Transform Controller::GetTransform(std::string parent, std::string child)
{
    geometry_msgs::Transform transform;

    try
    {
        transform = tfBuffer.lookupTransform(parent, child, ros::Time(0), ros::Duration(0.05)).transform;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        std::string error_message = "Cannot find transform from " + parent + " to " + child;
        throw std::runtime_error(error_message);
    }

    return transform;
}

Matrix6d Controller::ComputeGeometricalJacobian(std::vector<double>& q)
{
    std::vector<Eigen::Matrix4d> A;
    std::vector<Eigen::Matrix4d> T;
    std::vector<Eigen::Vector3d> z;
    std::vector<Eigen::Vector3d> p;

    A.push_back( this->A(q, 0) ); // A01
    T.push_back( A[0] );          // T01
    z.push_back( {0, 0, 1} );     // z0
    p.push_back( {0, 0, 0} );     // p0

    for(int k = 1; k < 6; k++)
    {
        A.push_back( this->A(q, k) );    // From A12 to A56
        T.push_back( T[k-1]*A[k] );              // From T02 to T06
        z.push_back( T[k-1].block<3, 1>(0, 2) ); // From z1 to z5
        p.push_back( T[k-1].block<3, 1>(0, 3) ); // From p1 to p5
    }

    Eigen::Matrix4d A6TCP   = Eigen::Matrix4d::Identity();   // The orientation is the same
    A6TCP.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0.1252); // The translation is along Z only
    Eigen::Matrix4d T0TCP   = T[5]*A6TCP;
    Eigen::Vector3d pTCP    = T0TCP.block<3, 1>(0, 3);

    // --- Actual computation of the Geometrical Jacobian according to Theoretical Notation --- //
    Matrix36d Jp = Matrix36d::Zero(); // From Jp1 to Jp6
    Matrix36d Jo = Matrix36d::Zero(); // From Jo1 to Jo6
    Matrix6d J  = Matrix6d::Zero(); // From J1 to J6

    for(int k = 0; k < 6; k++)
    {
        Jp.block<3, 1>(0, k) = z[k].cross(pTCP - p[k]);
        Jo.block<3, 1>(0, k) = z[k];
    }
    J.block<3, 6>(0, 0) = Jp;
    J.block<3, 6>(3, 0) = Jo;

    return J;
}

Matrix6d Controller::ComputeAnalyticalJacobian(Matrix6d Jg, Eigen::Vector3d flange_o)
{
    Eigen::Vector3d euler_xyz = flange_o;
    double psi = euler_xyz.x();
    double theta = euler_xyz.y();
    double phi = euler_xyz.z();
    Eigen::Matrix3d T;
    T << 1, 0, sin(theta),
        0, cos(psi), -sin(psi) * cos(theta),
        0, sin(psi), cos(psi) * cos(theta);
    Matrix6d T_A = Matrix6d::Identity();
    T_A.block<3, 3>(3, 3) = T;

    Matrix6d Ja = T_A.inverse() * Jg;

    return Ja;
}

void Controller::ControlInputCallback()
{
    geometry_msgs::Transform flange_pose;
    geometry_msgs::Transform desired_pose;
    try
    {
        flange_pose = this->GetTransform("base", "flange");
        desired_pose = this->GetTransform("base", "desired_pose");
    }
    catch (std::exception &e)
    {
        ROS_WARN("%s", e.what());
        return;
    }

    // Computing error
    Eigen::Isometry3d flange_eigen = tf2::transformToEigen(flange_pose);
    Eigen::Isometry3d desired_pose_eigen = tf2::transformToEigen(desired_pose);

    Eigen::Vector3d position_error = desired_pose_eigen.translation() - flange_eigen.translation();
    Eigen::Vector3d orientation_error = desired_pose_eigen.rotation().eulerAngles(0, 1, 2) - flange_eigen.rotation().eulerAngles(0, 1, 2);

    // Computing control action
    Eigen::Vector3d control_input_p = Kp.cwiseProduct(position_error);
    Eigen::Vector3d control_input_o = Ko.cwiseProduct(orientation_error);
    Vector6d control_input;
    control_input.block<3, 1>(0, 0) = control_input_p;
    control_input.block<3, 1>(3, 0) = control_input_o;

    ROS_INFO("control input p:  [%lf, %lf, %lf]", control_input_p[0], control_input_p[1], control_input_p[2]);
    ROS_INFO("control input po: [%lf, %lf, %lf]", control_input_o[0], control_input_o[1], control_input_o[2]);

    // Computing Jacobian
    sensor_msgs::JointState joints;
    sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", ros::Duration(10));
        if (msg == NULL)
            ROS_INFO("No joint messages received");
        else
            joints = * msg;

    ROS_INFO("Here");
    Matrix6d Jg = this->ComputeGeometricalJacobian(joints.position);
    Matrix6d Ja = this->ComputeAnalyticalJacobian(Jg, flange_eigen.rotation().eulerAngles(0, 1, 2));

    // Changing datatype
    Vector6d joint_velocities = Ja.inverse() * control_input;
    std_msgs::Float64MultiArray q_dot;
    q_dot.data.reserve(6);

    q_dot.data.push_back(joint_velocities[0]);
    q_dot.data.push_back(joint_velocities[1]);
    q_dot.data.push_back(joint_velocities[2]);
    q_dot.data.push_back(joint_velocities[3]);
    q_dot.data.push_back(joint_velocities[4]);
    q_dot.data.push_back(joint_velocities[5]);

    // ROS_INFO("joint_velocities: [%lf, %lf, %lf, %lf, %lf, %lf]", joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3], joint_velocities[4], joint_velocities[5]);
    // ROS_INFO("q_dot: [%lf, %lf, %lf, %lf, %lf, %lf]", q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3], q_dot.data[4], q_dot.data[5]);

    // Uncomment to die
    control_input_pub.publish(q_dot);
}

Controller::Controller(ros::NodeHandle &nh)
{
    this->nh = nh;

    // Publisher
    control_input_pub = nh.advertise<std_msgs::Float64MultiArray>("/VelocityControllers_JointGroupVelocityController/command", 100);

    // Creating the TF listener
    this->pTfListener = new tf2_ros::TransformListener(this->tfBuffer);

    this->control_input_timer = this->nh.createTimer(ros::Duration(0.01), std::bind(&Controller::ControlInputCallback, this));

    // Getting the controller gains from param file
    this->Kp = Eigen::Vector3d(0.1, 0.1, 0.1); // TODO: Parametrise
    this->Ko = Eigen::Vector3d(0.1, 0.1, 0.1); // TODO: Parametrise
}
