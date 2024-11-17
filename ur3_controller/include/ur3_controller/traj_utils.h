#ifndef TRAJ_UTILS_H
#define TRAJ_UTILS_H

#include <Eigen/Eigen>
#include "matplotlibcpp.h"

int nchoosek(int n, int k);

double Bernstein(int i, int n, double t);

double computePosition(const Eigen::VectorXd &B, double t, int order, double totalTime);

double computeVelocity(const Eigen::VectorXd &B, double t, int order, double totalTime);

double computeAcceleration(const Eigen::VectorXd &B, double t, int order, double totalTime);

Eigen::VectorXd trajPlanning(double, double, double, double, double, double, double);

Eigen::VectorXd trajParameterization(Eigen::VectorXd &, Eigen::VectorXd &, int, double);

void plotTrajFit(Eigen::VectorXd &, int, Eigen::VectorXd &, int, double);


#endif // TRAJ_UTILS_H
