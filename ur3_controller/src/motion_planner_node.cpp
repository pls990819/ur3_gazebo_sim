#include "ur3_controller/motion_planner_node.h"

UR3MotionPlanner::UR3MotionPlanner():x(12)
{
    x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    _para_traj_pub = nh.advertise<ur3_controller::ParaTraj>("/ParaTraj", 10);
    _joint_state_sub = nh.subscribe("/ur3/joint_states", 10, &UR3MotionPlanner::jointStateCallback, this);
    std::string package_name = "ur3_description";
    std::string file_name = "ur3.urdf";
    std::string urdf_path = ros::package::getPath(package_name) + "/urdf/" + file_name;
    // std::string urdf_path = "/home/pls/ur3_ws/src/ur3_description/urdf/" + file_name;
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    std::string integrator = "euler";
    std::string control = "zero";
    MotionPlanning = new UR3MotionPlanning(model,integrator,control,true,9);
}

void UR3MotionPlanner::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    x(0) = msg->position[2];
    x(1) = msg->position[1];
    x(2) = msg->position[0];
    x(3) = msg->position[3];
    x(4) = msg->position[4];
    x(5) = msg->position[5];
    x(6) = msg->velocity[2];
    x(7) = msg->velocity[1];
    x(8) = msg->velocity[0];
    x(9) = msg->velocity[3];
    x(10) = msg->velocity[4];
    x(11) = msg->velocity[5];
}

void UR3MotionPlanner::run() {
    while (ros::ok()) {
        std::string input;
        std::cout << "Enter target joint angles [q1,q2,q3,q4,q5,q6]: ";
        std::getline(std::cin, input);

        Eigen::VectorXd joint_angles(12);
        joint_angles.setZero();
        std::stringstream ss(input);
        for (int i = 0; i < 6; ++i) {
            ss >> joint_angles(i);
            if (ss.peek() == ',' || ss.peek() == ' ') {
                ss.ignore();
            }
        }

        std::cout << "Target joint angles: ";
        for (int i = 0; i < 6; ++i) {
            std::cout << joint_angles(i) << " ";
        }
        std::cout << std::endl;

        double total_duration = 0;
        std::cout << "Enter total duration (seconds): ";
        std::cin >> total_duration;

        MotionPlanning->set_bzrOrder(10);
        std::vector<Eigen::VectorXd> bzrcoef = MotionPlanning->solveProblem(x, joint_angles, total_duration);
        ur3_controller::ParaTraj msg;
        msg.stamp = ros::Time::now();
        msg.totalTime = total_duration;
        msg.nq = MotionPlanning->get_nu();
        msg.bezierOrder = MotionPlanning->get_bzrOrder();

        // 展平bzrcoef并将其存入controlPoints
        for (const auto& vec : bzrcoef) {
            msg.controlPoints.insert(msg.controlPoints.end(), vec.data(), vec.data() + vec.size());
        }


        _para_traj_pub.publish(msg);
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");

    UR3MotionPlanner *MotionPlanner;
    MotionPlanner = new UR3MotionPlanner(); 
    MotionPlanner->run();

    return 0;
}
