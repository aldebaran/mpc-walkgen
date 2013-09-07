#include "zebulon_com_centering_objective.h"

using namespace MPCWalkgen;

ComCenteringObjective::ComCenteringObjective(const LIPModel& lipModel,
                                             const BaseModel& baseModel)
:lipModel_(lipModel)
,baseModel_(baseModel)
,function_(1)
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  comRefInLocalFrame_.setZero(2*baseModel_.getNbSamples());
  comShiftInLocalFrame_.setZero(2*baseModel_.getNbSamples());
  gravityShift_.setZero(2*baseModel_.getNbSamples());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


ComCenteringObjective::~ComCenteringObjective(){}

void ComCenteringObjective::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);
  comRefInLocalFrame_.setZero(2*nbSamples);
  comShiftInLocalFrame_.setZero(2*nbSamples);
  gravityShift_.setZero(2*nbSamples);
}

const MatrixX& ComCenteringObjective::getGradient(const VectorX& x0)
{
  assert(comShiftInLocalFrame_.size()==comRefInLocalFrame_.size());
  assert(comShiftInLocalFrame_.size()==gravityShift_.size());
  assert(comRefInLocalFrame_.size()*2==x0.size());
  assert(comRefInLocalFrame_.size()==baseModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic& dynCom = lipModel_.getComPosLinearDynamic();


  int N = lipModel_.getNbSamples();

  gradient_.noalias() = getHessian()*x0;

  tmp_.noalias() = -comShiftInLocalFrame_.segment(0, N);
  tmp_.noalias() += dynCom.S*lipModel_.getStateX();
  tmp_.noalias() -= dynBasePos.S*baseModel_.getStateX();
  gradient_.block(0, 0, N, 1).noalias() += dynCom.UT*tmp_;


  tmp_.noalias() = -comShiftInLocalFrame_.segment(N, N);
  tmp_.noalias() += dynCom.S*lipModel_.getStateY();
  tmp_.noalias() -= dynBasePos.S*baseModel_.getStateY();
  gradient_.block(N, 0, N, 1).noalias() += dynCom.UT*tmp_;

  tmp_.noalias() = comShiftInLocalFrame_.segment(0, N);
  tmp_.noalias() -= dynCom.S*lipModel_.getStateX();
  tmp_.noalias() += dynBasePos.S*baseModel_.getStateX();
  gradient_.block(2*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;


  tmp_.noalias() = comShiftInLocalFrame_.segment(N, N);
  tmp_.noalias() -= dynCom.S*lipModel_.getStateY();
  tmp_.noalias() += dynBasePos.S*baseModel_.getStateY();
  gradient_.block(3*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;

  return gradient_;
}

const MatrixX& ComCenteringObjective::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

void ComCenteringObjective::updateGravityShift()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  Scalar m  = lipModel_.getMass();
  Scalar M  = baseModel_.getMass();
  Scalar cy = lipModel_.getComHeight();
  Scalar by = baseModel_.getComHeight();

  Scalar factor = (m*cy+M*by)/(M+m);

  const Vector3& g = lipModel_.getGravity();

  Scalar shiftX = factor*g(0)/g(2);
  Scalar shiftY = factor*g(1)/g(2);

  int N = baseModel_.getNbSamples();
  gravityShift_.setZero(2*N);
  gravityShift_.segment(0,N).fill(shiftX);
  gravityShift_.segment(N,N).fill(shiftY);
  comShiftInLocalFrame_ = comRefInLocalFrame_ + gravityShift_;
}

void ComCenteringObjective::setComRefInLocalFrame(const VectorX& copRefInWorldFrame)
{
  assert(copRefInWorldFrame.size() == lipModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
  assert(copRefInWorldFrame==copRefInWorldFrame);

  comRefInLocalFrame_ = copRefInWorldFrame;
  comShiftInLocalFrame_ = comRefInLocalFrame_ + gravityShift_;
}

void ComCenteringObjective::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic& dynCom = lipModel_.getComPosLinearDynamic();

  int N = lipModel_.getNbSamples();


  hessian_.setZero(4*N, 4*N);
  hessian_.block(0, 0, N, N) = dynCom.UT*dynCom.U;
  hessian_.block(N, N, N, N) = dynCom.UT*dynCom.U;

  hessian_.block(2*N, 2*N, N, N) = dynBasePos.UT*dynBasePos.U;
  hessian_.block(3*N, 3*N, N, N) = dynBasePos.UT*dynBasePos.U;

  hessian_.block(2*N, 0, N, N) = -dynBasePos.UT*dynCom.U;
  hessian_.block(3*N, N, N, N) = -dynBasePos.UT*dynCom.U;

  hessian_.block(0, 2*N, N, N) = -dynCom.UT*dynBasePos.U;
  hessian_.block(N, 3*N, N, N) = -dynCom.UT*dynBasePos.U;

  tmp_.resize(N);
}
