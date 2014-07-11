////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/trajectory_position_tracking_objective.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
PositionTrackingObjective<Scalar>
::PositionTrackingObjective(const NoDynamicModel<Scalar>& model)
:model_(model)
,function_(1)
,tmp_(1)
{
  posRefInWorldFrame_.setZero(model.getNbSamples());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
PositionTrackingObjective<Scalar>::~PositionTrackingObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::MatrixX& PositionTrackingObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(posRefInWorldFrame_.size()==x0.size());
  assert(posRefInWorldFrame_.size()==model_.getNbSamples());

  const LinearDynamic<Scalar>& dyn = model_.getPosLinearDynamic();

  gradient_.noalias() = getHessian()*x0;


  tmp_.noalias() = dyn.S * model_.getState();
  tmp_.noalias() -= posRefInWorldFrame_;
  gradient_.noalias() += dyn.UT*tmp_;
  return gradient_;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& PositionTrackingObjective<Scalar>::getHessian()
{
  return hessian_;
}

template <typename Scalar>
void PositionTrackingObjective<Scalar>::setPosRefInWorldFrame(const VectorX& posRefInWorldFrame)
{
  assert(posRefInWorldFrame.size()==model_.getNbSamples());
  assert(posRefInWorldFrame==posRefInWorldFrame);

  posRefInWorldFrame_ = posRefInWorldFrame;
}


template <typename Scalar>
void PositionTrackingObjective<Scalar>::computeConstantPart()
{
  const LinearDynamic<Scalar>& dyn = model_.getPosLinearDynamic();

  int N = model_.getNbSamples();

  hessian_ = dyn.UT*dyn.U;

  tmp_.resize(N);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(PositionTrackingObjective);
}
