////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/trajectory_velocity_tracking_objective.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
VelocityTrackingObjective<Scalar>
::VelocityTrackingObjective(const NoDynamicModel<Scalar>& model)
:model_(model)
,function_(1)
,tmp_(1)
{
  velRefInWorldFrame_.setZero(model.getNbSamples());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
VelocityTrackingObjective<Scalar>::~VelocityTrackingObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::MatrixX& VelocityTrackingObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(velRefInWorldFrame_.size()==x0.size());
  assert(velRefInWorldFrame_.size()==model_.getNbSamples());

  const LinearDynamic<Scalar>& dyn = model_.getVelLinearDynamic();

  gradient_.noalias() = getHessian()*x0;


  tmp_.noalias() = dyn.S * model_.getState();
  tmp_.noalias() -= velRefInWorldFrame_;
  gradient_.noalias() += dyn.UT*tmp_;


  return gradient_;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& VelocityTrackingObjective<Scalar>::getHessian()
{
  return hessian_;
}

template <typename Scalar>
void VelocityTrackingObjective<Scalar>::setVelRefInWorldFrame(const VectorX& velRefInWorldFrame)
{
  assert(velRefInWorldFrame.size()==model_.getNbSamples());
  assert(velRefInWorldFrame==velRefInWorldFrame);

  velRefInWorldFrame_ = velRefInWorldFrame;
}

template <typename Scalar>
void VelocityTrackingObjective<Scalar>::computeConstantPart()
{
  const LinearDynamic<Scalar>& dyn = model_.getVelLinearDynamic();

  int N = model_.getNbSamples();

  hessian_ = dyn.UT*dyn.U;

  tmp_.resize(N);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(VelocityTrackingObjective);
}
