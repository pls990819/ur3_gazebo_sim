#include <Eigen/Eigen>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/multibody/actions/impulse-fwddyn.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/impulses/impulse-6d.hpp"
#include "crocoddyl/multibody/residuals/com-position.hpp"
#include "crocoddyl/multibody/residuals/frame-translation.hpp"
#include "crocoddyl/multibody/residuals/frame-velocity.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include <crocoddyl/core/solvers/ddp.hpp> // 确保包含了 DDP solver 的头文件
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
// ActivationBounds
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include "crocoddyl/multibody/residuals/contact-control-gravity.hpp"
#include "crocoddyl/multibody/residuals/contact-cop-position.hpp"
#include "crocoddyl/multibody/residuals/contact-force.hpp"
#include "crocoddyl/multibody/residuals/contact-wrench-cone.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"


#include "ur3_controller/traj_utils.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
// #include <crocoddyl/core/activations/quadratic.hpp>
// #include <crocoddyl/core/costs/control.hpp>
// #include <crocoddyl/core/costs/state.hpp>
// #include <crocoddyl/core/diff-action-base.hpp>
// #include <crocoddyl/core/integrator/euler.hpp>
// #include <crocoddyl/core/actuation/full.hpp>
// #include <crocoddyl/core/state-base.hpp>
// #include <crocoddyl/core/utils/exception.hpp>


#include <vector>
#include <string>
class UR3MotionPlanning{
public:
    UR3MotionPlanning(pinocchio::Model & ,std::string &, std::string &, bool, int);
    std::vector<Eigen::VectorXd> solveProblem(const Eigen::VectorXd &, const Eigen::VectorXd &, double);
    int get_nu();
    int get_bzrOrder();
    void set_bzrOrder(int);

private:
    pinocchio::Model _rmodel;
    pinocchio::Data _rdata;
    boost::shared_ptr<crocoddyl::StateMultibody> _state;
    boost::shared_ptr<crocoddyl::ActuationModelFull> _actuation;
    double _dt;
    int _T;
    int _nu;
    int _bzrOrder;
};