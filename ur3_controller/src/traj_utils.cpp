#include "ur3_controller/traj_utils.h"

namespace plt = matplotlibcpp;

int nchoosek(int n, int k) {
    if (k == 0 || k == n) {
        return 1;
    } else {
        return nchoosek(n - 1, k - 1) + nchoosek(n - 1, k);
    }
}

double Bernstein(int i, int n, double t) {
    return nchoosek(n, i) * pow(t, i) * pow(1 - t, n - i);
}

double computePosition(const Eigen::VectorXd &B, double t, int order, double totalTime) {
    double res = 0.0;
    for (int j = 0; j <= order; ++j) {
        res += B(j) * Bernstein(j, order, t);
    }
    return res;
}

double computeVelocity(const Eigen::VectorXd &B, double t, int order, double totalTime) {
    double res = 0.0;
    for (int j = 0; j < order; ++j) {
        res += order * (B(j + 1) - B(j)) * Bernstein(j, order - 1, t) / totalTime;
    }
    return res;
}

double computeAcceleration(const Eigen::VectorXd &B, double t, int order, double totalTime) {
    double res = 0.0;
    for (int j = 0; j < order - 1; ++j) {
        res += order * (order - 1) * (B(j + 2) - 2 * B(j + 1) + B(j)) * Bernstein(j, order - 2, t) / (totalTime * totalTime);
    }
    return res;
}


Eigen::VectorXd trajPlanning(double q_0, double v_0, double a_0, double q_f, double v_f, double a_f, double totalTime) {
    double f = 1.0 / totalTime;
    double b = 1.0 / (totalTime * totalTime);
    Eigen::Matrix<double, 6, 6> Aeq;
    Aeq << 1, 0, 0, 0, 0, 0,
          -5 * f, 5 * f, 0, 0, 0, 0,
           20 * b, -40 * b, 20 * b, 0, 0, 0,
           0, 0, 0, 0, 0, 1,
           0, 0, 0, 0, -5 * f, 5 * f,
           0, 0, 0, 20 * b, -40 * b, 20 * b;
    Eigen::VectorXd beq(6);
    beq << q_0, v_0, a_0, q_f, v_f, a_f;
    Eigen::VectorXd x = Aeq.colPivHouseholderQr().solve(beq);
    return x;
}


Eigen::VectorXd trajParameterization(Eigen::VectorXd &trajPos, Eigen::VectorXd &trajVel, int n, double totalTime) {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(n + 1);
    int dataLen = trajPos.size();
    B(0) = trajPos(0);
    B(1) = trajVel(0) * totalTime / n + B(0);
    B(n) = trajPos(dataLen - 1);
    B(n-1) = B(n) - trajVel(dataLen - 1) * totalTime / n;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dataLen, n - 3);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dataLen);
    for (int i = 0; i < dataLen; ++i) {
        double t = static_cast<double>(i) / (dataLen - 1);
        b(i) = trajPos(i) - B(0) * Bernstein(0, n, t) - B(1) * Bernstein(1, n, t) - B(n - 1) * Bernstein(n - 1, n, t) - B(n) * Bernstein(n, n, t);
        for (int j = 2; j < n-1; ++j) {
            A(i, j - 2) = Bernstein(j, n, t);
        }
    }
    // Eigen::LeastSquaresConjugateGradient<MatrixXd> lscg;
    // lscg.compute(A);
    // B.segment(2, n - 3) = lscg.solve(b);
    B.segment(2, n - 3) = A.colPivHouseholderQr().solve(b);
    return B;
}

void plotTrajFit(Eigen::VectorXd & originTraj,int timeN, Eigen::VectorXd &bzrCoef, int bzrOrder, double totalTime)
{
    
    Eigen::VectorXd timeSeq = Eigen::VectorXd::LinSpaced(timeN, 0, totalTime);
    // std::cout << timeSeq << std::endl; 

    std::vector<double> time(timeSeq.data(), timeSeq.data() + timeSeq.size());
    std::vector<double> origin(originTraj.data(), originTraj.data() + originTraj.size());
    plt::plot(time, origin, "r-");

    std::vector<double> bzr(timeN);
    for(int i = 0;i < timeN; i++)
    {
        // std::cout << double(i)/(timeN-1) ;
        bzr[i] = computePosition(bzrCoef, double(i)/(timeN-1), bzrOrder, totalTime);
    }
    plt::plot(time, bzr, "b-");

    plt::title("bzrfit");
    plt::xlabel("time(s)");
    plt::ylabel("angle(rad)");
    plt::show();
}