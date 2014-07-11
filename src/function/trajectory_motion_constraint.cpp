////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/trajectory_motion_constraint.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
MotionConstraint<Scalar>::MotionConstraint(const NoDynamicModel<Scalar>& model)
:model_(model)
,functionInf_(1)
,functionSup_(1)
,tmp_(1)
,tmp2_(1)
{
  functionInf_.fill(0);
  functionSup_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
MotionConstraint<Scalar>::~MotionConstraint(){}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& MotionConstraint<Scalar>::getFunctionInf(const VectorX& x0)
{
  assert(model_.getNbSamples()==x0.size());

  computeFunction(x0, functionInf_,
                  -model_.getVelocityLimit(),
                  -model_.getAccelerationLimit());

  return functionInf_;
}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& MotionConstraint<Scalar>::getFunctionSup(const VectorX& x0)
{
  assert(model_.getNbSamples()==x0.size());

  computeFunction(x0, functionSup_,
                  model_.getVelocityLimit(),
                  model_.getAccelerationLimit());

  return functionSup_;
}

template <typename Scalar>
void MotionConstraint<Scalar>::computeFunction(const VectorX& x0, VectorX& func,
                                               Scalar velLimit, Scalar accLimit)
{
  int N = model_.getNbSamples();

  const LinearDynamic<Scalar>& dynBaseVel = model_.getVelLinearDynamic();
  const LinearDynamic<Scalar>& dynBaseAcc = model_.getAccLinearDynamic();

  tmp_.fill(velLimit);
  tmp2_.fill(accLimit);

  func.noalias() = -getGradient()*x0;
  func.segment(0, N).noalias() += tmp_;
  func.segment(0, N).noalias() -= dynBaseVel.S * model_.getState();
  func.segment(N, N).noalias() += tmp2_;
  func.segment(N, N).noalias() -= dynBaseAcc.S * model_.getState();

}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& MotionConstraint<Scalar>::getGradient()
{
  return gradient_;
}

template <typename Scalar>
int MotionConstraint<Scalar>::getNbConstraints()
{
  return model_.getNbSamples()*2;
}

template <typename Scalar>
void MotionConstraint<Scalar>::computeConstantPart()
{
  const LinearDynamic<Scalar>& dynBaseVel = model_.getVelLinearDynamic();
  const LinearDynamic<Scalar>& dynBaseAcc = model_.getAccLinearDynamic();

  int N = model_.getNbSamples();
  int M = getNbConstraints();

  gradient_.setZero(M, N);
  gradient_.block(0, 0, N, N) = dynBaseVel.U;
  gradient_.block(N, 0, N, N) = dynBaseAcc.U;

  tmp_.resize(N);
  tmp2_.resize(N);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(MotionConstraint);
}
