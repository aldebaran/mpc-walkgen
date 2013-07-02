#include "zebulon_base_motion_constraint.h"

using namespace MPCWalkgen;

BaseMotionConstraint::BaseMotionConstraint(const BaseModel& baseModel)
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


BaseMotionConstraint::~BaseMotionConstraint(){}

const VectorX& BaseMotionConstraint::getFunctionInf(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*2==x0.size());

  computeFunction(x0, functionInf_,
                  -baseModel_.getVelocityLimit(),
                  -baseModel_.getAccelerationLimit());

  return functionInf_;
}

const VectorX& BaseMotionConstraint::getFunctionSup(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*2==x0.size());

  computeFunction(x0, functionSup_,
                  baseModel_.getVelocityLimit(),
                  baseModel_.getAccelerationLimit());

  return functionSup_;
}

void BaseMotionConstraint::computeFunction(const VectorX& x0, VectorX& func,
                                           Scalar velLimit, Scalar accLimit)
{
  int N = baseModel_.getNbSamples();

  const LinearDynamic& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic& dynBaseAcc = baseModel_.getBaseAccLinearDynamic();

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

const MatrixX& BaseMotionConstraint::getGradient()
{
  return gradient_;
}

int BaseMotionConstraint::getNbConstraints()
{
  return baseModel_.getNbSamples()*4;
}

void BaseMotionConstraint::computeConstantPart()
{
  const LinearDynamic& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic& dynBaseAcc = baseModel_.getBaseVelLinearDynamic();

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
