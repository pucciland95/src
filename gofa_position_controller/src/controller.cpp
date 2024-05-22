#include "../include/controller.hpp"

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

Matrix6d Controller::ComputeGeometricalJacobian()
{
    Matrix6d jacobian;

    geometry_msgs::Transform link1 = this->GetTransform("base", "L1");
    geometry_msgs::Transform link2 = this->GetTransform("base", "L2");
    geometry_msgs::Transform link3 = this->GetTransform("base", "L3");
    geometry_msgs::Transform link4 = this->GetTransform("base", "L4");
    geometry_msgs::Transform link5 = this->GetTransform("base", "L5");
    geometry_msgs::Transform link6 = this->GetTransform("base", "L6");
    geometry_msgs::Transform flange = this->GetTransform("base", "flange");

    Eigen::Vector3d p = Eigen::Vector3d(flange.translation.x,
                                        flange.translation.y,
                                        flange.translation.z);

    // Link1
    Eigen::Vector3d p1 = Eigen::Vector3d(link1.translation.x, link1.translation.y, link1.translation.z);
    Eigen::Quaterniond q1;
    fromMsg(link1.rotation, q1);
    Eigen::Vector3d z1 = q1.toRotationMatrix().block<3, 1>(0, 2);
    jacobian.block<3, 1>(0, 0) = z1.cross(p - p1);
    jacobian.block<3, 1>(3, 0) = z1;

    // Link2
    Eigen::Vector3d p2 = Eigen::Vector3d(link2.translation.x, link2.translation.y, link2.translation.z);
    Eigen::Quaterniond q2;
    fromMsg(link2.rotation, q2);
    Eigen::Vector3d z2 = q2.toRotationMatrix().block<3, 1>(0, 1);
    jacobian.block<3, 1>(0, 1) = z2.cross(p - p2);
    jacobian.block<3, 1>(3, 1) = z2;

    // Link3
    Eigen::Vector3d p3 = Eigen::Vector3d(link3.translation.x, link3.translation.y, link3.translation.z);
    Eigen::Quaterniond q3;
    fromMsg(link3.rotation, q3);
    Eigen::Vector3d z3 = q3.toRotationMatrix().block<3, 1>(0, 1);
    jacobian.block<3, 1>(0, 2) = z3.cross(p - p3);
    jacobian.block<3, 1>(3, 2) = z3;

    // Link4
    Eigen::Vector3d p4 = Eigen::Vector3d(link4.translation.x, link4.translation.y, link4.translation.z);
    Eigen::Quaterniond q4;
    fromMsg(link4.rotation, q4);
    Eigen::Vector3d z4 = q4.toRotationMatrix().block<3, 1>(0, 0);
    jacobian.block<3, 1>(0, 3) = z4.cross(p - p4);
    jacobian.block<3, 1>(3, 3) = z4;

    // Link5
    Eigen::Vector3d p5 = Eigen::Vector3d(link5.translation.x, link5.translation.y, link5.translation.z);
    Eigen::Quaterniond q5;
    fromMsg(link5.rotation, q5);
    Eigen::Vector3d z5 = q5.toRotationMatrix().block<3, 1>(0, 1);
    jacobian.block<3, 1>(0, 4) = z5.cross(p - p5);
    jacobian.block<3, 1>(3, 4) = z5;

    // Link6
    Eigen::Vector3d p6 = Eigen::Vector3d(link6.translation.x, link6.translation.y, link6.translation.z);
    Eigen::Quaterniond q6;
    fromMsg(link6.rotation, q6);
    Eigen::Vector3d z6 = q6.toRotationMatrix().block<3, 1>(0, 0);
    jacobian.block<3, 1>(0, 5) = z6.cross(p - p6);
    jacobian.block<3, 1>(3, 5) = z6;

    // std::vector<geometry_msgs::Transform> links = {link1, link2, link3, link4, link5, link6};

    // for (int i = 0; i < 6; i++)
    // {
    //     geometry_msgs::Transform eachLink = links[i];
    //     Eigen::Vector3d p_i = Eigen::Vector3d(eachLink.translation.x,
    //                                           eachLink.translation.y,
    //                                           eachLink.translation.z);
    //     Eigen::Quaterniond q;
    //     fromMsg(eachLink.rotation, q);

    //     Eigen::Vector3d z_i = q.toRotationMatrix().block<3, 1>(0, 2);

    //     jacobian.block<3, 1>(0, i) = z_i.cross(p - p_i);
    //     jacobian.block<3, 1>(3, i) = z_i;
    // }

    return jacobian;
}

Eigen::Vector3d Controller::ComputeOrientationError(Eigen::Matrix3d &R_flange, Eigen::Matrix3d &R_desiredPose)
{
    Eigen::Quaterniond q_e = Eigen::Quaterniond(R_flange);
    Eigen::Quaterniond q_d = Eigen::Quaterniond(R_desiredPose);

    Eigen::Quaterniond delta_q = q_d * q_e.inverse();

    // Which one is better?
    // Eigen::Vector3d orientation_error = Eigen::Vector3d(delta_q.x(), delta_q.y(), delta_q.z()) * delta_q.w();
    Eigen::Vector3d orientation_error = Eigen::Vector3d(delta_q.x(), delta_q.y(), delta_q.z());

    return orientation_error;
}

void Controller::ControlInputCallback()
{
    geometry_msgs::Transform flange_pose;
    geometry_msgs::Transform desired_pose;
    try
    {
        flange_pose = this->GetTransform(this->tf_base_name, this->tf_tcp_name);
        desired_pose = this->GetTransform(this->tf_base_name, this->tf_desired_name);
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
    Eigen::Matrix3d R_flange = flange_eigen.rotation();
    Eigen::Matrix3d R_desiredPose = desired_pose_eigen.rotation();
    Eigen::Vector3d orientation_error = ComputeOrientationError(R_flange, R_desiredPose);

    // Computing control action
    Eigen::Vector3d control_input_p = Kp.cwiseProduct(position_error);
    Eigen::Vector3d control_input_o = Ko.cwiseProduct(orientation_error);
    Vector6d control_input;
    control_input.block<3, 1>(0, 0) = control_input_p;
    control_input.block<3, 1>(3, 0) = control_input_o;

    // Computing Jacobian
    Matrix6d Jg = this->ComputeGeometricalJacobian();
    Vector6d joint_velocities = Jg.inverse() * control_input;

    // Changing datatype
    std_msgs::Float64MultiArray q_dot;
    q_dot.data.reserve(6);

    q_dot.data.push_back(joint_velocities[0]);
    q_dot.data.push_back(joint_velocities[1]);
    q_dot.data.push_back(joint_velocities[2]);
    q_dot.data.push_back(joint_velocities[3]);
    q_dot.data.push_back(joint_velocities[4]);
    q_dot.data.push_back(joint_velocities[5]);

    // Uncomment to die
    joint_velocity_pub.publish(q_dot);
}

Controller::Controller(ros::NodeHandle &nh)
{
    this->nh = nh;

    // Getting parameters

    std::string topic_input_commands_name = "/VelocityControllers_JointGroupVelocityController/command"; // TODO: parametrise
    // Publisher
    this->joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_input_commands_name, 100);

    // Creating the TF listener
    this->pTfListener = new tf2_ros::TransformListener(this->tfBuffer);

    this->control_input_timer = this->nh.createTimer(ros::Duration(0.01), std::bind(&Controller::ControlInputCallback, this));

    // Getting the controller gains from param file
    this->Kp = Eigen::Vector3d(10.0, 10.0, 10.0); // TODO: Parametrise
    this->Ko = Eigen::Vector3d(5.0, 5.0, 5.0); // TODO: Parametrise

    this->tf_base_name = "base"; // TODO: Parametrise
    this->tf_tcp_name = "flange"; // TODO: Parametrise
    this->tf_desired_name = "desired_pose"; // TODO: Parametrise
}
 