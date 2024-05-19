#include "../include/controller.hpp"

geometry_msgs::Transform Controller::GetTransform(std::string parent, std::string child)
{
    geometry_msgs::Transform transform;

    try
    {
        transform = tfBuffer.lookupTransform(parent, child, ros::Time(0)).transform;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        throw("Cannot find transform from %s to %s", parent.c_str(), child.c_str());
    }

    return transform;
}

Matrix6d Controller::ComputeGeometricalJacobian()
{
    Matrix6d jacobian;

    geometry_msgs::Transform link1 = this->GetTransform("base", "L1");
    geometry_msgs::Transform link2 = this->GetTransform("base", "L2");
    geometry_msgs::Transform link3 = this->GetTransform("base", "L3");
    geometry_msgs::Transform link4 = this->GetTransform("base", "L4");
    geometry_msgs::Transform link5 = this->GetTransform("base", "L5");
    geometry_msgs::Transform flange = this->GetTransform("base", "flange");

    Eigen::Vector3d p = Eigen::Vector3d(flange.translation.x,
                                        flange.translation.y,
                                        flange.translation.x);

    std::vector<geometry_msgs::Transform> links = {link1, link2, link3, link4, link5, flange};

    for (int i = 0; i < 6; i++)
    {
        geometry_msgs::Transform eachLink = links[i];
        Eigen::Vector3d p_i = Eigen::Vector3d(eachLink.translation.x,
                                              eachLink.translation.y,
                                              eachLink.translation.z);
        Eigen::Quaterniond q;
        fromMsg(eachLink.rotation, q);
        Eigen::Vector3d z_i = q.toRotationMatrix().block<3, 1>(0, 2);

        jacobian.block<3, 1>(0, i) = z_i.cross(p - p_i);
        jacobian.block<3, 1>(3, i) = z_i;
    }

    return jacobian;
}

Matrix6d Controller::ComputeAnalyticalJacobian(Matrix6d Jg, Eigen::Vector3d flange_o)
{
    Eigen::Vector3d euler_xyz = flange_o;
    double psi = euler_xyz.x();
    double theta = euler_xyz.y();
    double phi = euler_xyz.z();
    Eigen::Matrix3d T;
    T << 1,        0,           sin(theta),
         0, cos(psi), -sin(psi)*cos(theta),
         0, sin(psi),  cos(psi)*cos(theta);
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
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
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

    // Computing Jacobian
    Matrix6d Jg = this->ComputeGeometricalJacobian();
    Matrix6d Ja = this->ComputeAnalyticalJacobian(Jg, flange_eigen.rotation().eulerAngles(0, 1, 2));

    // Changing datatype
    Vector6d joint_velocities = Ja.inverse() * control_input;
    std_msgs::Float64MultiArray q_dot;
    q_dot.data.insert(q_dot.data.end(), joint_velocities.data()[0], joint_velocities.data()[0] + 6);

    ROS_INFO("joint_velocities: [%lf, %lf, %lf, %lf, %lf, %lf]", joint_velocities[0], joint_velocities[1], joint_velocities[2], joint_velocities[3], joint_velocities[4], joint_velocities[5]);
    ROS_INFO("q_dot: [%lf, %lf, %lf, %lf, %lf, %lf]", q_dot.data[0], q_dot.data[1], q_dot.data[2], q_dot.data[3], q_dot.data[4], q_dot.data[5]);

    // Uncomment to die
    // control_input_pub.publish(q_dot);
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
    this->Kp = Eigen::Vector3d(1.0, 1.0, 1.0); // TODO: Parametrise
    this->Ko = Eigen::Vector3d(1.0, 1.0, 1.0); // TODO: Parametrise
}
