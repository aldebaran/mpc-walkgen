#include "zebulon_base_position_tracking_objective.h"

using namespace MPCWalkgen;

BasePositionTrackingObjective::BasePositionTrackingObjective(const BaseModel& baseModel)
:baseModel_(baseModel)
,function_(1)
{
  posRefInWorldFrame_.setZero(2*baseModel_.getNbSamples());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


BasePositionTrackingObjective::~BasePositionTrackingObjective(){}

const MatrixX& BasePositionTrackingObjective::getGradient(const VectorX& x0)
{
  assert(posRefInWorldFrame_.size()==x0.size());
  assert(posRefInWorldFrame_.size()==baseModel_.getNbSamples()*2);

  const LinearDynamic& dyn = baseModel_.getBasePosLinearDynamic();

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

const MatrixX& BasePositionTrackingObjective::getHessian()
{
  return hessian_;
}

void BasePositionTrackingObjective::setPosRefInWorldFrame(const VectorX& posRefInWorldFrame)
{
  assert(posRefInWorldFrame.size()==baseModel_.getNbSamples()*2);
  assert(posRefInWorldFrame==posRefInWorldFrame);

  posRefInWorldFrame_ = posRefInWorldFrame;
}


void BasePositionTrackingObjective::computeConstantPart()
{
  const LinearDynamic& dyn = baseModel_.getBasePosLinearDynamic();

  int N = baseModel_.getNbSamples();

  hessian_.resize(2*N, 2*N);
  hessian_.block(0, 0, N, N) = dyn.UT*dyn.U;
  hessian_.block(N, N, N, N) = dyn.UT*dyn.U;
  hessian_.block(0, N, N, N).fill(0.0);
  hessian_.block(N, 0, N, N).fill(0.0);

  tmp_.resize(N);
}
