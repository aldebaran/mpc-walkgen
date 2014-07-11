////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/trajectory_jerk_minimization_objective.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
TrajectoryJerkMinimizationObjective<Scalar>::TrajectoryJerkMinimizationObjective(
    const NoDynamicModel<Scalar>& model)
:model_(model)
,function_(1)
{
  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
TrajectoryJerkMinimizationObjective<Scalar>::~TrajectoryJerkMinimizationObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& TrajectoryJerkMinimizationObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(model_.getNbSamples()==x0.size());

  return x0;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& TrajectoryJerkMinimizationObjective<Scalar>::getHessian()
{
  return hessian_;
}

template <typename Scalar>
void TrajectoryJerkMinimizationObjective<Scalar>::computeConstantPart()
{
  int N = model_.getNbSamples();
  hessian_ = MatrixX::Identity(N, N);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(TrajectoryJerkMinimizationObjective);
}
