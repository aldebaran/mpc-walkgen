////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_base_motion_constraint.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
BaseMotionConstraint<Scalar>::BaseMotionConstraint(const BaseModel<Scalar>& baseModel)
:baseModel_(baseModel)
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
BaseMotionConstraint<Scalar>::~BaseMotionConstraint(){}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& BaseMotionConstraint<Scalar>::getFunctionInf(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*2==x0.size());

  computeFunction(x0, functionInf_,
                  -baseModel_.getVelocityLimit(),
                  -baseModel_.getAccelerationLimit());

  return functionInf_;
}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& BaseMotionConstraint<Scalar>::getFunctionSup(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*2==x0.size());

  computeFunction(x0, functionSup_,
                  baseModel_.getVelocityLimit(),
                  baseModel_.getAccelerationLimit());

  return functionSup_;
}

template <typename Scalar>
void BaseMotionConstraint<Scalar>::computeFunction(const VectorX& x0, VectorX& func,
                                           Scalar velLimit, Scalar accLimit)
{
  int N = baseModel_.getNbSamples();

  const LinearDynamic<Scalar>& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic<Scalar>& dynBaseAcc = baseModel_.getBaseAccLinearDynamic();

  tmp_.fill(velLimit);
  tmp2_.fill(accLimit);

  func.noalias() = -getGradient()*x0;
  func.segment(0, N).noalias() += tmp_;
  func.segment(0, N).noalias() -= dynBaseVel.S * baseModel_.getStateX();

  func.segment(N, N).noalias() += tmp_;
  func.segment(N, N).noalias() -= dynBaseVel.S * baseModel_.getStateY();

  func.segment(2*N, N).noalias() += tmp2_;
  func.segment(2*N, N).noalias() -= dynBaseAcc.S * baseModel_.getStateX();

  func.segment(3*N, N).noalias() += tmp2_;
  func.segment(3*N, N).noalias() -= dynBaseAcc.S * baseModel_.getStateY();

}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& BaseMotionConstraint<Scalar>::getGradient()
{
  return gradient_;
}

template <typename Scalar>
int BaseMotionConstraint<Scalar>::getNbConstraints()
{
  return baseModel_.getNbSamples()*4;
}

template <typename Scalar>
void BaseMotionConstraint<Scalar>::computeConstantPart()
{
  const LinearDynamic<Scalar>& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic<Scalar>& dynBaseAcc = baseModel_.getBaseVelLinearDynamic();

  int N = baseModel_.getNbSamples();
  int M = getNbConstraints();

  gradient_.setZero(M, 2*N);
  gradient_.block(0, 0, N, N) = dynBaseVel.U;
  gradient_.block(N, N, N, N) = dynBaseVel.U;
  gradient_.block(2*N, 0, N, N) = dynBaseAcc.U;
  gradient_.block(3*N, N, N, N) = dynBaseAcc.U;

  tmp_.resize(N);
  tmp2_.resize(N);
}

MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(BaseMotionConstraint);
