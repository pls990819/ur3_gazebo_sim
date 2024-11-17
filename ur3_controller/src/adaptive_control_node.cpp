#include "ur3_controller/adaptive_control_node.h"

UR3AdaptiveControl::UR3AdaptiveControl(const std::string &package_name,
                                       const std::string &file_name_x,
                                       const std::string &file_name_y,
                                       const std::string &file_name_z)
    : trajTracking(false),bzrOrder(10),
      t(0), t0(0), t_last(0), totalTime(0), r13(0), r23(0), r33(1),
      q_f(Eigen::VectorXd::Zero(6)),
      v_f(Eigen::VectorXd::Zero(6)),
      a_f(Eigen::VectorXd::Zero(6)),
      q(Eigen::VectorXd::Zero(6)),
      v(Eigen::VectorXd::Zero(6)),
      a(Eigen::VectorXd::Zero(6)),
      G_x(Eigen::VectorXd::Zero(6)),
      G_y(Eigen::VectorXd::Zero(6)),
      G_z(Eigen::VectorXd::Zero(6)),
      K1(5 * Eigen::MatrixXd::Identity(6, 6)),
      K2(2 * Eigen::MatrixXd::Identity(6, 6)),
      Kp(K1 * K2 + Eigen::MatrixXd::Identity(6, 6)),
      Kd(K1 + K2),
      C(Eigen::MatrixXd::Zero(6, 6)),
      M(Eigen::MatrixXd::Zero(6, 6)),
      Minv(Eigen::MatrixXd::Zero(6, 6)),
      UR3ControllerNode(new UR3Controller()),
      bzrcoef(6, Eigen::VectorXd::Zero(bzrOrder+1))
{
    loadURDF(package_name, file_name_x, model_x, data_x);
    loadURDF(package_name, file_name_y, model_y, data_y);
    loadURDF(package_name, file_name_z, model_z, data_z);
    nq = model_x.nq;
    _para_traj_sub = nh.subscribe("/ParaTraj", 10, &UR3AdaptiveControl::paraTrajCallback, this);
    std::cout << "UR3AdaptiveControl_init" << std::endl;

}

void UR3AdaptiveControl::loadURDF(const std::string& package_name, const std::string& file_name, pinocchio::Model& model, pinocchio::Data& data) {
    std::string urdf_path = ros::package::getPath(package_name) + "/urdf/" + file_name;
    if (urdf_path.empty()) {
        std::cerr << "URDF file path is empty: " << urdf_path << std::endl;
        return;
    }
    pinocchio::urdf::buildModel(urdf_path, model);
    data = pinocchio::Data(model);
}

void UR3AdaptiveControl::paraTrajCallback(const ur3_controller::ParaTraj &msg)
{
    if (!trajTracking)
    {
        trajTracking = true;
        t0 = UR3ControllerNode->getTime();
        ROS_INFO("Received ParaTraj message");
        ROS_INFO("Timestamp: %f", msg.stamp.toSec());
        ROS_INFO("Total Time: %f", msg.totalTime);
        ROS_INFO("Number of q: %d", msg.nq);
        ROS_INFO("Bezier Order: %d", msg.bezierOrder);
        totalTime = msg.totalTime;
        for (int i = 0; i < nq; i++) {
            Eigen::VectorXd vec(msg.bezierOrder+1);
            for (int j = 0; j < msg.bezierOrder+1; ++j) {
                vec(j) = msg.controlPoints[i*(msg.bezierOrder+1) + j];
            }
            bzrcoef[i] = vec;
            q_f(i) = vec(msg.bezierOrder);
        }
        t_last = t0;
    }
}

void UR3AdaptiveControl::parameterUpdate(const Eigen::VectorXd &e1, const Eigen::VectorXd &e2)
{
    double dt = t - t_last;
    Eigen::VectorXd e = e2 + K1 * e1;
    r13 += 0.03 * dt * e.dot(Minv * G_x);
    r23 += 0.03 * dt * e.dot(Minv * G_y);
    r33 += 0.03 * dt * e.dot(Minv * G_z);
    ROS_INFO("r13: %f, r23: %f, r33: %f, r13+r23+r33: %f, t: %f", r13, r23, r33, r13 * r13 + r23 * r23 + r33 * r33, t - t0);
    t_last = t;
}

void UR3AdaptiveControl::dynamicUpdate()
{
    G_x = pinocchio::computeGeneralizedGravity(model_x, data_x, q);
    G_y = pinocchio::computeGeneralizedGravity(model_y, data_y, q);
    G_z = pinocchio::computeGeneralizedGravity(model_z, data_z, q);
    crba(model_z, data_z, q);
    Eigen::MatrixXd M_upper = data_z.M.triangularView<Eigen::Upper>();
    M = M_upper + M_upper.transpose().eval();
    Minv = M.inverse();
    C = pinocchio::computeCoriolisMatrix(model_z, data_z, q, v);
}

void UR3AdaptiveControl::stateUpdate()
{
    t = UR3ControllerNode->getTime();
    q = UR3ControllerNode->getQ();
    v = UR3ControllerNode->getV();
}

void UR3AdaptiveControl::run()
{
    ros::Rate rate(250);
    Eigen::VectorXd e1 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd e2 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd a_r = Eigen::VectorXd::Zero(6);
    while (ros::ok())
    {
        stateUpdate();
        dynamicUpdate();
        std::cout << "t:" << t << ";  t0:" << t0 << std::endl;
        if ((t - t0) < totalTime)
        {
            double tt = (t - t0) / totalTime;
            Eigen::VectorXd q_r(6), v_r(6);
            for (int i = 0; i < 6; ++i)
            {
                q_r[i] = computePosition(bzrcoef[i], tt, bzrOrder, totalTime);
                v_r[i] = computeVelocity(bzrcoef[i], tt, bzrOrder, totalTime);
                a_r[i] = computeAcceleration(bzrcoef[i], tt, bzrOrder, totalTime);
            }
            e1 = q_r - q;
            e2 = v_r - v;
            parameterUpdate(e1, e2);
        }
        else
        {
            trajTracking = false;
            e1 = q_f - q;
            e2 = v_f - v;
            a_r = a_f;
        }
        Eigen::VectorXd tau = M * (a_r + Kp * e1 + Kd * e2) + C * v + r13 * G_x + r23 * G_y + r33 * G_z;
        UR3ControllerNode->torquePub(tau);
        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur3_control");
    std::string package_name = "ur3_description";
    std::string file_name_x = "ur3_x.urdf";
    std::string file_name_y = "ur3_y.urdf";
    std::string file_name_z = "ur3_z.urdf";
    UR3AdaptiveControl *UR3AdaptiveControlNode;
    UR3AdaptiveControlNode = new UR3AdaptiveControl(package_name, file_name_x, file_name_y, file_name_z);
    UR3AdaptiveControlNode->run();
    return 0;
}