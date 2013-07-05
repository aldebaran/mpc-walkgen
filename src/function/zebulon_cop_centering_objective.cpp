#include "zebulon_cop_centering_objective.h"

using namespace MPCWalkgen;

CopCenteringObjective::CopCenteringObjective(const LIPModel& lipModel,
                                             const BaseModel& baseModel)
:lipModel_(lipModel)
,baseModel_(baseModel)
,function_(1)
{
  copRefInLocalFrame_.setZero(2*baseModel_.getNbSamples());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);

  computeConstantPart();
}


CopCenteringObjective::~CopCenteringObjective(){}

const MatrixX& CopCenteringObjective::getGradient(const VectorX& x0)
{
  assert(copRefInLocalFrame_.size()*2==x0.size());
  assert(copRefInLocalFrame_.size()==baseModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();


  int N = lipModel_.getNbSamples();

  gradient_.noalias() = getHessian()*x0;

  if (baseModel_.getMass()>EPSILON)
  {
    const LinearDynamic& dynCopXCom = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYCom = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynCopXBase = baseModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYBase = baseModel_.getCopYLinearDynamic();

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopXCom.S*lipModel_.getStateX();
    tmp_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX();
    gradient_.block(0, 0, N, 1).noalias() += dynCopXCom.UT*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopYCom.S*lipModel_.getStateY();
    tmp_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY();
    gradient_.block(N, 0, N, 1).noalias() += dynCopYCom.UT*tmp_;

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopXCom.S*lipModel_.getStateX();
    tmp_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX();
    gradient_.block(2*N, 0, N, 1).noalias() += (dynCopXBase.UT-dynBasePos.UT)*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopYCom.S*lipModel_.getStateY();
    tmp_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY();
    gradient_.block(3*N, 0, N, 1).noalias() += (dynCopYBase.UT-dynBasePos.UT)*tmp_;
  }
  else
  {
    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopX.S*lipModel_.getStateX();
    tmp_.noalias() -= dynBasePos.S*baseModel_.getStateX();
    gradient_.block(0, 0, N, 1).noalias() += dynCopX.UT*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopY.S*lipModel_.getStateY();
    tmp_.noalias() -= dynBasePos.S*baseModel_.getStateY();
    gradient_.block(N, 0, N, 1).noalias() += dynCopY.UT*tmp_;

    tmp_.noalias() = copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() -= dynCopX.S*lipModel_.getStateX();
    tmp_.noalias() += dynBasePos.S*baseModel_.getStateX();
    gradient_.block(2*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;


    tmp_.noalias() = copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() -= dynCopY.S*lipModel_.getStateY();
    tmp_.noalias() += dynBasePos.S*baseModel_.getStateY();
    gradient_.block(3*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;
  }

  return gradient_;
}

const MatrixX& CopCenteringObjective::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

void CopCenteringObjective::setCopRefInLocalFrame(const VectorX& copRefInWorldFrame)
{
  assert(copRefInWorldFrame.size() == lipModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
  assert(copRefInWorldFrame==copRefInWorldFrame);

  copRefInLocalFrame_ = copRefInWorldFrame;
}

void CopCenteringObjective::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  int N = lipModel_.getNbSamples();

  if (baseModel_.getMass()>EPSILON)
  {
    const LinearDynamic& dynCopXCom = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYCom = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynCopXBase = baseModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYBase = baseModel_.getCopYLinearDynamic();

    hessian_.setZero(4*N, 4*N);
    hessian_.block(0, 0, N, N) = dynCopXCom.UT*dynCopXCom.U;
    hessian_.block(N, N, N, N) = dynCopYCom.UT*dynCopYCom.U;

    hessian_.block(2*N, 2*N, N, N) = (dynCopXBase.UT-dynBasePos.UT)*(dynCopXBase.U-dynBasePos.U);
    hessian_.block(3*N, 3*N, N, N) = (dynCopXBase.UT-dynBasePos.UT)*(dynCopYBase.U-dynBasePos.U);

    hessian_.block(2*N, 0, N, N) = (dynCopXBase.UT-dynBasePos.UT)*dynCopXCom.U;
    hessian_.block(3*N, N, N, N) = (dynCopXBase.UT-dynBasePos.UT)*dynCopYCom.U;

    hessian_.block(0, 2*N, N, N) = dynCopXCom.UT*(dynCopXBase.U-dynBasePos.U);
    hessian_.block(N, 3*N, N, N) = dynCopYCom.UT*(dynCopYBase.U-dynBasePos.U);
  }
  else
  {
    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    hessian_.setZero(4*N, 4*N);
    hessian_.block(0, 0, N, N) = dynCopX.UT*dynCopX.U;
    hessian_.block(N, N, N, N) = dynCopY.UT*dynCopY.U;

    hessian_.block(2*N, 2*N, N, N) = dynBasePos.UT*dynBasePos.U;
    hessian_.block(3*N, 3*N, N, N) = dynBasePos.UT*dynBasePos.U;

    hessian_.block(2*N, 0, N, N) = -dynBasePos.UT*dynCopX.U;
    hessian_.block(3*N, N, N, N) = -dynBasePos.UT*dynCopY.U;

    hessian_.block(0, 2*N, N, N) = -dynCopX.UT*dynBasePos.U;
    hessian_.block(N, 3*N, N, N) = -dynCopY.UT*dynBasePos.U;
  }

  tmp_.resize(N);
}
