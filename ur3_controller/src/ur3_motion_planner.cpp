#include "ur3_controller/ur3_motion_planner.h"

UR3MotionPlanning::UR3MotionPlanning(pinocchio::Model &rmodel,
                                     std::string &integrator, 
                                     std::string &control, 
                                     bool fwddyn,
                                     int bzrOrder): _rmodel(rmodel),
                                                    _dt(0.1),
                                                    _T(101),
                                                    _bzrOrder(bzrOrder),
                                                    _rdata(_rmodel),
                                                    _state(boost::make_shared<crocoddyl::StateMultibody>(boost::make_shared<pinocchio::Model>(_rmodel))),
                                                    _actuation(boost::make_shared<crocoddyl::ActuationModelFull>(_state)),
                                                    _nu(_state->get_nv())
{
}

std::vector<Eigen::VectorXd> UR3MotionPlanning::solveProblem(const Eigen::VectorXd &x0, const Eigen::VectorXd &goal, double totalTime)
{
    _T = int(totalTime/_dt)+1;
    // std::cout << _T << std::endl;
    // std::cout << "x0" << x0 << std::endl;
    // std::cout << "goal" << goal << std::endl;
    auto runningCostModel = boost::make_shared<crocoddyl::CostModelSum>(_state);
    auto terminalCostModel = boost::make_shared<crocoddyl::CostModelSum>(_state);

    // Joint angle goal residual
    auto xResidual = boost::make_shared<crocoddyl::ResidualModelState>(_state, goal, _nu);
    auto goalTrackingCost = boost::make_shared<crocoddyl::CostModelResidual>(_state, xResidual);

    auto uResidual = boost::make_shared<crocoddyl::ResidualModelControl>(_state, _nu);
    auto uRegCost = boost::make_shared<crocoddyl::CostModelResidual>(_state, uResidual);
    // Add cost items
    runningCostModel->addCost("jointPose", goalTrackingCost, 10);
    runningCostModel->addCost("uReg", uRegCost, 0.1);
    terminalCostModel->addCost("jointPose", goalTrackingCost, 1000);
    auto runningModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
            _state, _actuation, runningCostModel),
        _dt);

    auto terminalModel = boost::make_shared<crocoddyl::IntegratedActionModelEuler>(
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
            _state, _actuation, terminalCostModel),
        0.0);

    auto runningModels = std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>(_T, runningModel);
    auto problem = boost::make_shared<crocoddyl::ShootingProblem>(x0, runningModels, terminalModel);
    auto solver = boost::make_shared<crocoddyl::SolverFDDP>(problem);
    solver->solve();

    std::vector<Eigen::VectorXd> xs = solver->get_xs();
    int rows = xs[0].size();
    int cols = xs.size();
    Eigen::MatrixXd mat(rows, cols);
    for (int i = 0; i < cols; ++i) {
        mat.col(i) = xs[i];
    }

    std::vector<Eigen::VectorXd> bzrcoef(_nu);
    for (int i = 0; i < _nu; ++i) {
        Eigen::VectorXd pos = mat.row(i);
        Eigen::VectorXd vel = mat.row(i + _nu);
        bzrcoef[i] = trajParameterization(pos,vel,_bzrOrder,totalTime);
        // if(i == 0)
        // {
        //     plotTrajFit(pos, cols, bzrcoef[i], _bzrOrder,totalTime);
        // }
    }

    return bzrcoef;
}


int UR3MotionPlanning::get_nu()
{
    return _nu;
}

int UR3MotionPlanning::get_bzrOrder()
{
    return _bzrOrder;
}

void UR3MotionPlanning::set_bzrOrder(int bzrOrder)
{
    _bzrOrder = bzrOrder;
}
