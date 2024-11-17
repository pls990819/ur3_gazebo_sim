#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Eigen>
#include <iostream>

class UR3Controller{
public:
    UR3Controller();
    ~UR3Controller();
    void torquePub(const Eigen::VectorXd &);
    void showQV() const;
    Eigen::VectorXd getQ() const;
    Eigen::VectorXd getV() const;
    double getTime() const;

private:
    ros::NodeHandle nh;
    ros::Subscriber _joint_state_sub;
    ros::Publisher _shoulder_pan_torque_pub;
    ros::Publisher _shoulder_lift_torque_pub;
    ros::Publisher _elbow_torque_pub;
    ros::Publisher _wrist_1_torque_pub;
    ros::Publisher _wrist_2_torque_pub;
    ros::Publisher _wrist_3_torque_pub;
    double t;
    Eigen::VectorXd q,v;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &);
};