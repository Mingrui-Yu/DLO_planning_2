#include "dlo_arm_planning_pkg/mpc/mpc_base.h"


namespace dlo_arm_planning_pkg{

// ------------------------------------------------------------
MPCBase::MPCBase(
    const JacobianModel::Ptr &jaco_model
){
    jaco_model_ = jaco_model;
}



} // end namespace


