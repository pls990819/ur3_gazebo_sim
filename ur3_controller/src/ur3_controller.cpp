#include "ur3_controller/ur3_controller.h"

UR3Controller::UR3Controller(): q(6), v(6) 
{
    _joint_state_sub = nh.subscribe("/ur3/joint_states", 10, &UR3Controller::jointStateCallback, this);
    _shoulder_pan_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/shoulder_pan_joint_controller/command", 10);
    _shoulder_lift_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/shoulder_lift_joint_controller/command", 10);
    _elbow_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/elbow_joint_controller/command", 10);
    _wrist_1_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/wrist_1_joint_controller/command", 10);
    _wrist_2_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/wrist_2_joint_controller/command", 10);
    _wrist_3_torque_pub = nh.advertise<std_msgs::Float64>("/ur3/wrist_3_joint_controller/command", 10);
}

void UR3Controller::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    t = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;
    q(0) = msg->position[2];
    q(1) = msg->position[1];
    q(2) = msg->position[0];
    q(3) = msg->position[3];
    q(4) = msg->position[4];
    q(5) = msg->position[5];
    v(0) = msg->velocity[2];
    v(1) = msg->velocity[1];
    v(2) = msg->velocity[0];
    v(3) = msg->velocity[3];
    v(4) = msg->velocity[4];
    v(5) = msg->velocity[5];
}

void UR3Controller::torquePub(const Eigen::VectorXd &tau)
{
    std_msgs::Float64 msg;
    msg.data = tau(0);
    _shoulder_pan_torque_pub.publish(msg);
    msg.data = tau(1);
    _shoulder_lift_torque_pub.publish(msg);
    msg.data = tau(2);
    _elbow_torque_pub.publish(msg);
    msg.data = tau(3);
    _wrist_1_torque_pub.publish(msg);
    msg.data = tau(4);
    _wrist_2_torque_pub.publish(msg);
    msg.data = tau(5);
    _wrist_3_torque_pub.publish(msg);
}

void UR3Controller::showQV() const
{
    std::cout << "q:" << q << std::endl;
    std::cout << "v:" << v << std::endl;
    // ROS_INFO("time:  %f, q:[%f, %f, %f, %f, %f, %f]", p1, p2, p3, p1*p1 + p2*p2 + p3*p3, t - t0);
}

Eigen::VectorXd UR3Controller::getQ() const
{
    return q;
}

Eigen::VectorXd UR3Controller::getV() const
{
    return v;
}

double UR3Controller::getTime() const
{
    return t;
}