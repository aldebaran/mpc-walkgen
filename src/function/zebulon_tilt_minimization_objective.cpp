////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_tilt_minimization_objective.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/constant.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
TiltMinimizationObjective<Scalar>::TiltMinimizationObjective(const LIPModel<Scalar>& lipModel,
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
TiltMinimizationObjective<Scalar>::~TiltMinimizationObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::MatrixX& TiltMinimizationObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  int N = baseModel_.getNbSamples();

  gradient_.noalias() = getHessian()*x0;

  VectorX Kx = dynC_.S*lipModel_.getStateX() + dynB_.S*baseModel_.getStateX()
             + dynPsiX_.S*baseModel_.getStateRoll().segment(0, 2) + dynPsiX_.K;
  VectorX Ky = dynC_.S*lipModel_.getStateY() + dynB_.S*baseModel_.getStateY()
             + dynPsiY_.S*baseModel_.getStatePitch().segment(0, 2) + dynPsiY_.K ;

  gradient_.block(0, 0, N, 1).noalias() += dynC_.UT*Kx;
  gradient_.block(N, 0, N, 1).noalias() += dynC_.UT*Ky;

  gradient_.block(2*N, 0, N, 1).noalias() += dynB_.UT*Kx;
  gradient_.block(3*N, 0, N, 1).noalias() += dynB_.UT*Ky;

  return gradient_;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& TiltMinimizationObjective<Scalar>::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

template <typename Scalar>
void TiltMinimizationObjective<Scalar>::updateTiltContactPoint()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  int N = baseModel_.getNbSamples();
  Scalar dbx = baseModel_.getTiltContactPointX();
  Scalar dby = baseModel_.getTiltContactPointY();
  Scalar M = baseModel_.getMass();
  Scalar g = Constant<Scalar>::GRAVITY_NORM;

  VectorX tmp(N);
  Scalar A3x = M*g*dbx;
  Scalar A3y = M*g*dby;
  tmp.fill(A3x);
  dynPsiX_.K = -uInv_*tmp;

  tmp.fill(A3y);
  dynPsiY_.K = -uInv_*tmp;

}


// The goal is to inverse the equation of the tilt angle psi:
// g ( m h + M L ) psi + m g ( c - d ) =
// M L ddot{d} + m h ddot{c} + ( m h h + M L L ) ddot{psi}
//
// with:
//- m the upper bodies mass (kg)
//- M the base mass (kg)
//- h the upper bodies CoM height (m)
//- L the base CoM height (m)
//- psi the tilt angle (rad)
//- c the upper bodies CoM horizontal position (m)
//- d the base CoM horizontal position (m)
template <typename Scalar>
void TiltMinimizationObjective<Scalar>::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  int N = baseModel_.getNbSamples();
  Scalar L = baseModel_.getComHeight();
  Scalar h = lipModel_.getComHeight();
  Scalar m = lipModel_.getMass();
  Scalar M = baseModel_.getMass();
  Scalar dbx = baseModel_.getTiltContactPointX();
  Scalar dby = baseModel_.getTiltContactPointY();
  Scalar g = Constant<Scalar>::GRAVITY_NORM;

  Scalar A1 = g*(m*h+M*L);
  Scalar A2 = m*g;
  Scalar A3x = M*g*dbx;
  Scalar A3y = M*g*dby;
  Scalar A4 = m*h;
  Scalar A5 = M*L;
  Scalar A6 = m*h*h+M*L*L;

  const LinearDynamic<Scalar>& tiltAngleDyn = baseModel_.getBaseTiltAngleLinearDynamic();
  const LinearDynamic<Scalar>& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic<Scalar>& dynBaseAcc = baseModel_.getBaseAccLinearDynamic();
  const LinearDynamic<Scalar>& dynComPos = lipModel_.getComPosLinearDynamic();
  const LinearDynamic<Scalar>& dynComAcc = lipModel_.getComAccLinearDynamic();

  //Compute the inverse dynamic of the angular acc
  LinearDynamic<Scalar> angularAccRelativeToAngleDyn;

  angularAccRelativeToAngleDyn.U = tiltAngleDyn.Uinv;
  angularAccRelativeToAngleDyn.S = -tiltAngleDyn.Uinv*tiltAngleDyn.S;

  //Invert this matrix
  MatrixX Id = MatrixX::Identity(N, N);
  MatrixX U = (A1*Id - A6*tiltAngleDyn.Uinv);
  const Scalar eps = 1e-15f;
  Tools::inverseLU<Scalar>(U, uInv_, eps);
  //Compute the dynamics
  VectorX tmp(N);
  tmp.fill(A3x);
  dynPsiX_.K = -uInv_*tmp;
  dynPsiX_.S = uInv_*A6*angularAccRelativeToAngleDyn.S;

  tmp.fill(A3y);
  dynPsiY_.K = -uInv_*tmp;
  dynPsiY_.S = uInv_*A6*angularAccRelativeToAngleDyn.S;

  dynC_.U = uInv_*(-A5*dynComAcc.U + A2*dynComPos.U);
  dynC_.UT = dynC_.U.transpose();
  dynC_.S = uInv_*(-A5*dynComAcc.S + A2*dynComPos.S);

  dynB_.U = uInv_*(-A4*dynBaseAcc.U - A2*dynBasePos.U);
  dynB_.UT = dynB_.U.transpose();
  dynB_.S = uInv_*(-A4*dynBaseAcc.S - A2*dynBasePos.S);

  //Compute the hessian
  hessian_.setZero(4*N, 4*N);
  hessian_.block(0, 0, N, N) = dynC_.UT*dynC_.U;
  hessian_.block(N, N, N, N) = dynC_.UT*dynC_.U;

  hessian_.block(2*N, 2*N, N, N) = dynB_.UT*dynB_.U;
  hessian_.block(3*N, 3*N, N, N) = dynB_.UT*dynB_.U;

  hessian_.block(2*N, 0, N, N) = dynB_.UT*dynC_.U;
  hessian_.block(3*N, N, N, N) = dynB_.UT*dynC_.U;

  hessian_.block(0, 2*N, N, N) = dynC_.UT*dynB_.U;
  hessian_.block(N, 3*N, N, N) = dynC_.UT*dynB_.U;
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(TiltMinimizationObjective);
}
