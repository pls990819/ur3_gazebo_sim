#include "ur3_controller/ur3_motion_planner.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "ur3_controller/ParaTraj.h"
#include <iostream>
#include <vector>
#include <sstream>


class UR3MotionPlanner{
public:
    UR3MotionPlanner();
    ~UR3MotionPlanner();
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &);
    void run();

private:
    ros::NodeHandle nh;
    ros::Publisher _para_traj_pub;
    ros::Subscriber _joint_state_sub;
    UR3MotionPlanning *MotionPlanning;
    Eigen::VectorXd x;
};