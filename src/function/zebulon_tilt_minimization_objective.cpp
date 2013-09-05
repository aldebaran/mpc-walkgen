#include "zebulon_tilt_minimization_objective.h"

using namespace MPCWalkgen;

TiltMinimizationObjective::TiltMinimizationObjective(const LIPModel& lipModel,
                                                     const BaseModel& baseModel)
:lipModel_(lipModel)
,baseModel_(baseModel)
,function_(1)
,normalizationFactor_(120.0)
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


TiltMinimizationObjective::~TiltMinimizationObjective(){}

const MatrixX& TiltMinimizationObjective::getGradient(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  //Temporary parameters. Need to be changes with an implementation of the new equations
  Scalar factorTheta  = 350.0;
  Scalar factoracc    = 10.0;

  Scalar factorRoll = baseModel_.getStateRoll()(0)*factorTheta
               + baseModel_.getStateRoll()(2)*factoracc;
  Scalar factorPitch = baseModel_.getStatePitch()(0)*factorTheta
               + baseModel_.getStatePitch()(2)*factoracc;

  Scalar alpha5 = partialFactor5_*factorRoll;
  Scalar alpha6 = partialFactor6_*factorPitch;
  int N = lipModel_.getNbSamples();
  VectorX alpha5Vec(N);
  alpha5Vec.fill(alpha5);
  VectorX alpha6Vec(N);
  alpha6Vec.fill(alpha6);

  gradient_.noalias() = getHessian()*x0;
  gradient_.block(0, 0, N, 1).noalias()   += dynC_.UT * ( dynC_.S * lipModel_.getStateX()
                                                        + dynB_.S * baseModel_.getStateX()
                                                        + alpha5Vec);
  gradient_.block(2*N, 0, N, 1).noalias() += dynB_.UT * ( dynC_.S * lipModel_.getStateX()
                                                        + dynB_.S * baseModel_.getStateX()
                                                        + alpha5Vec);

  gradient_.block(N, 0, N, 1).noalias()   += dynC_.UT * ( dynC_.S * lipModel_.getStateY()
                                                        + dynB_.S * baseModel_.getStateY()
                                                        + alpha6Vec);
  gradient_.block(3*N, 0, N, 1).noalias() += dynB_.UT * ( dynC_.S * lipModel_.getStateY()
                                                        + dynB_.S * baseModel_.getStateY()
                                                        + alpha6Vec);
  return gradient_;
}

const MatrixX& TiltMinimizationObjective::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
  return hessian_;
}

void TiltMinimizationObjective::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  int N = baseModel_.getNbSamples();

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic& dynBaseAcc = baseModel_.getBaseAccLinearDynamic();
  const LinearDynamic& dynComPos = lipModel_.getComPosLinearDynamic();
  const LinearDynamic& dynComAcc = lipModel_.getComAccLinearDynamic();

  m_          = lipModel_.getMass();
  M_          = baseModel_.getMass();
  L_          = baseModel_.getWheelToBaseDistance();
  theta0_     = baseModel_.getAngleWheelToBaseCom();
  hc_         = lipModel_.getComHeight();
  sinTheta0_  = std::sin(theta0_);
  cosTheta0_  = std::cos(theta0_);
  sin2Theta0_ = std::sin(2.0*theta0_);
  cos2theta0_ = std::cos(2.0*theta0_);

  Scalar alpha1 = m_*GRAVITY_NORM/normalizationFactor_;
  Scalar alpha2 = -m_*hc_/normalizationFactor_;
  Scalar alpha3 = -m_*GRAVITY_NORM/normalizationFactor_;
  Scalar alpha4 = -M_*L_*(sinTheta0_-theta0_*cosTheta0_)/normalizationFactor_;

  partialFactor5_= (M_*L_*L_*(1.0-cos2theta0_-4.0*theta0_*sin2Theta0_)/2.0)/normalizationFactor_;
  partialFactor6_= (M_*L_*L_*(1.0-cos2theta0_-4.0*theta0_*sin2Theta0_)/2.0)/normalizationFactor_;


  dynC_.U = alpha1 *dynComPos.U + alpha2 * dynComAcc.U;
  dynC_.UT = dynC_.U.transpose();
  dynC_.S = alpha1 *dynComPos.S + alpha2 * dynComAcc.S;

  dynB_.U = alpha3 *dynBasePos.U + alpha4 * dynBaseAcc.U;
  dynB_.UT = dynB_.U.transpose();
  dynB_.S = alpha3 *dynBasePos.S + alpha4 * dynBaseAcc.S;

  hessian_.setZero(4*N, 4*N);
  hessian_.block(0, 0, N, N) = dynC_.UT*dynC_.U;
  hessian_.block(2*N, 2*N, N, N) = dynB_.UT*dynB_.U;
  hessian_.block(2*N, 0, N, N) = dynB_.UT*dynC_.U;
  hessian_.block(0, 2*N, N, N) = dynC_.UT*dynB_.U;

  hessian_.block(N, N, N, N) = dynC_.UT*dynC_.U;
  hessian_.block(3*N, 3*N, N, N) = dynB_.UT*dynB_.U;
  hessian_.block(3*N, N, N, N) = dynB_.UT*dynC_.U;
  hessian_.block(N, 3*N, N, N) = dynC_.UT*dynB_.U;

}
