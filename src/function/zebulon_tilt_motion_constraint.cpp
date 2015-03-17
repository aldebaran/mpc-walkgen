////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_tilt_motion_constraint.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
TiltMotionConstraint<Scalar>::TiltMotionConstraint(const LIPModel<Scalar>& lipModel,
                                                   const BaseModel<Scalar>& baseModel)
:lipModel_(lipModel)
,baseModel_(baseModel)
,function_(1)
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


template <typename Scalar>
TiltMotionConstraint<Scalar>::~TiltMotionConstraint(){}

template <typename Scalar>
const typename Type<Scalar>::VectorX& TiltMotionConstraint<Scalar>::getFunction(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());


  int N = baseModel_.getNbSamples();
  Scalar theta = baseModel_.getStateYaw()(0);

  const LinearDynamic<Scalar>& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic<Scalar>& dynComVel = lipModel_.getComVelLinearDynamic();


  function_.noalias() = -getGradient()*x0;
  function_.segment(0, N).noalias() -= dynComVel.S * lipModel_.getStateX()*std::sin(theta);
  function_.segment(0, N).noalias() += dynComVel.S * lipModel_.getStateY()*std::cos(theta);
  function_.segment(N, N).noalias() -= dynBaseVel.S * baseModel_.getStateX()*std::sin(theta);
  function_.segment(N, N).noalias() += dynBaseVel.S * baseModel_.getStateY()*std::cos(theta);


  return function_;
}


template <typename Scalar>
const typename Type<Scalar>::MatrixX& TiltMotionConstraint<Scalar>::getGradient()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return gradient_;
}

template <typename Scalar>
int TiltMotionConstraint<Scalar>::getNbConstraints()
{
  return baseModel_.getNbSamples()*2;
}

template <typename Scalar>
void TiltMotionConstraint<Scalar>::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic<Scalar>& dynBaseVel = baseModel_.getBaseVelLinearDynamic();
  const LinearDynamic<Scalar>& dynComVel = lipModel_.getComVelLinearDynamic();

  int N = baseModel_.getNbSamples();
  int M = getNbConstraints();
  Scalar theta = baseModel_.getStateYaw()(0);

  gradient_.setZero(M, 4*N);
  gradient_.block(0, 0, N, N) = dynComVel.U*std::sin(theta);
  gradient_.block(0, N, N, N) = -dynComVel.U*std::cos(theta);
  gradient_.block(N, 2*N, N, N) = dynBaseVel.U*std::sin(theta);
  gradient_.block(N, 3*N, N, N) = -dynBaseVel.U*std::cos(theta);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(TiltMotionConstraint);
}
