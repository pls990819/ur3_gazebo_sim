#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <Eigen/Eigen>
#include "ur3_controller/ur3_controller.h"
#include "ur3_controller/traj_utils.h"
#include "ur3_controller/ParaTraj.h"

class UR3AdaptiveControl{
public:
    UR3AdaptiveControl(const std::string &,const std::string &,const std::string &,const std::string &);
    ~UR3AdaptiveControl();
    void parameterUpdate(const Eigen::VectorXd &, const Eigen::VectorXd &);
    void dynamicUpdate();
    void stateUpdate();
    void run();

private:
    bool trajTracking;
    double t, t0, t_last, totalTime, r13, r23, r33;
    UR3Controller *UR3ControllerNode;
    std::string urdf_path_x, urdf_path_y, urdf_path_z;
    int nq, bzrOrder;
    ros::NodeHandle nh;
    ros::Subscriber _para_traj_sub;
    pinocchio::Model model_x, model_y, model_z;
    pinocchio::Data data_x, data_y, data_z;
    Eigen::VectorXd q_f, v_f, a_f, q, v, a;
    Eigen::VectorXd G_x, G_y, G_z;
    Eigen::MatrixXd K1, K2, Kp, Kd;
    Eigen::MatrixXd C, M, Minv;
    std::vector<Eigen::VectorXd> bzrcoef;
    void paraTrajCallback(const ur3_controller::ParaTraj& msg);
    void loadURDF(const std::string &, const std::string &, pinocchio::Model &, pinocchio::Data &);
};