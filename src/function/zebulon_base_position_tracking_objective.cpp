////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_base_position_tracking_objective.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
BasePositionTrackingObjective<Scalar>
::BasePositionTrackingObjective(const BaseModel<Scalar>& baseModel)
:baseModel_(baseModel)
,function_(1)
,tmp_(1)
{
  posRefInWorldFrame_.setZero(2*baseModel_.getNbSamples());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
BasePositionTrackingObjective<Scalar>::~BasePositionTrackingObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::MatrixX& BasePositionTrackingObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(posRefInWorldFrame_.size()==x0.size());
  assert(posRefInWorldFrame_.size()==baseModel_.getNbSamples()*2);

  const LinearDynamic<Scalar>& dyn = baseModel_.getBasePosLinearDynamic();

  int N = baseModel_.getNbSamples();

  gradient_.noalias() = getHessian()*x0;


  tmp_.noalias() = dyn.S * baseModel_.getStateX();
  tmp_.noalias() -= posRefInWorldFrame_.segment(0, N);
  gradient_.block(0, 0, N, 1).noalias() += dyn.UT*tmp_;

  tmp_.noalias() = dyn.S * baseModel_.getStateY();
  tmp_.noalias()-= posRefInWorldFrame_.segment(N, N);
  gradient_.block(N, 0, N, 1).noalias() += dyn.UT*tmp_;
  return gradient_;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& BasePositionTrackingObjective<Scalar>::getHessian()
{
  return hessian_;
}

template <typename Scalar>
void BasePositionTrackingObjective<Scalar>::setPosRefInWorldFrame(const VectorX& posRefInWorldFrame)
{
  assert(posRefInWorldFrame.size()==baseModel_.getNbSamples()*2);
  assert(posRefInWorldFrame==posRefInWorldFrame);

  posRefInWorldFrame_ = posRefInWorldFrame;
}


template <typename Scalar>
void BasePositionTrackingObjective<Scalar>::computeConstantPart()
{
  const LinearDynamic<Scalar>& dyn = baseModel_.getBasePosLinearDynamic();

  int N = baseModel_.getNbSamples();

  hessian_.resize(2*N, 2*N);
  hessian_.block(0, 0, N, N) = dyn.UT*dyn.U;
  hessian_.block(N, N, N, N) = dyn.UT*dyn.U;
  hessian_.block(0, N, N, N).fill(0.0);
  hessian_.block(N, 0, N, N).fill(0.0);

  tmp_.resize(N);
}

MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(BasePositionTrackingObjective);
