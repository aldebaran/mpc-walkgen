////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_cop_centering_objective.h>
#include <mpc-walkgen/constant.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
CopCenteringObjective<Scalar>::CopCenteringObjective(const LIPModel<Scalar>& lipModel,
                                                     const BaseModel<Scalar>& baseModel)
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


template <typename Scalar>
CopCenteringObjective<Scalar>::~CopCenteringObjective(){}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& CopCenteringObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(copRefInLocalFrame_.size()*2==x0.size());
  assert(copRefInLocalFrame_.size()==baseModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic<Scalar>& dynBasePos = baseModel_.getBasePosLinearDynamic();


  int N = lipModel_.getNbSamples();

  gradient_.noalias() = getHessian()*x0;

  const LinearDynamic<Scalar>& dynCopXCom = lipModel_.getCopXLinearDynamic();
  const LinearDynamic<Scalar>& dynCopYCom = lipModel_.getCopYLinearDynamic();

  if (baseModel_.getMass()>Constant<Scalar>::EPSILON)
  {
    const LinearDynamic<Scalar>& dynCopXBase = baseModel_.getCopXLinearDynamic();
    const LinearDynamic<Scalar>& dynCopYBase = baseModel_.getCopYLinearDynamic();

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    tmp_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX() + dynCopXBase.K;
    gradient_.block(0, 0, N, 1).noalias() += dynCopXCom.UT*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    tmp_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY() + dynCopYBase.K;
    gradient_.block(N, 0, N, 1).noalias() += dynCopYCom.UT*tmp_;

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    tmp_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX() + dynCopXBase.K;
    gradient_.block(2*N, 0, N, 1).noalias() += (dynCopXBase.UT-dynBasePos.UT)*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    tmp_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY() + dynCopYBase.K;
    gradient_.block(3*N, 0, N, 1).noalias() += (dynCopYBase.UT-dynBasePos.UT)*tmp_;

  }
  else
  {

    tmp_.noalias() = -copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    tmp_.noalias() -= dynBasePos.S*baseModel_.getStateX();
    gradient_.block(0, 0, N, 1).noalias() += dynCopXCom.UT*tmp_;


    tmp_.noalias() = -copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    tmp_.noalias() -= dynBasePos.S*baseModel_.getStateY();
    gradient_.block(N, 0, N, 1).noalias() += dynCopYCom.UT*tmp_;

    tmp_.noalias() = copRefInLocalFrame_.segment(0, N);
    tmp_.noalias() -= dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    tmp_.noalias() += dynBasePos.S*baseModel_.getStateX();
    gradient_.block(2*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;


    tmp_.noalias() = copRefInLocalFrame_.segment(N, N);
    tmp_.noalias() -= dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    tmp_.noalias() += dynBasePos.S*baseModel_.getStateY();
    gradient_.block(3*N, 0, N, 1).noalias() += dynBasePos.UT*tmp_;
  }

  return gradient_;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& CopCenteringObjective<Scalar>::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

template <typename Scalar>
void CopCenteringObjective<Scalar>::setCopRefInLocalFrame(const VectorX& copRefInWorldFrame)
{
  assert(copRefInWorldFrame.size() == lipModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
  assert(copRefInWorldFrame==copRefInWorldFrame);

  copRefInLocalFrame_ = copRefInWorldFrame;
}

template <typename Scalar>
void CopCenteringObjective<Scalar>::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic<Scalar>& dynBasePos = baseModel_.getBasePosLinearDynamic();
  int N = lipModel_.getNbSamples();

  const LinearDynamic<Scalar>& dynCopXCom = lipModel_.getCopXLinearDynamic();
  const LinearDynamic<Scalar>& dynCopYCom = lipModel_.getCopYLinearDynamic();

  hessian_.setZero(4*N, 4*N);

  if (baseModel_.getMass()>Constant<Scalar>::EPSILON)
  {
    const LinearDynamic<Scalar>& dynCopXBase = baseModel_.getCopXLinearDynamic();
    const LinearDynamic<Scalar>& dynCopYBase = baseModel_.getCopYLinearDynamic();

    hessian_.block(0, 0, N, N) = dynCopXCom.UT*dynCopXCom.U;
    hessian_.block(N, N, N, N) = dynCopYCom.UT*dynCopYCom.U;

    hessian_.block(2*N, 2*N, N, N) = (dynCopXBase.UT-dynBasePos.UT)*(dynCopXBase.U-dynBasePos.U);
    hessian_.block(3*N, 3*N, N, N) = (dynCopYBase.UT-dynBasePos.UT)*(dynCopYBase.U-dynBasePos.U);

    hessian_.block(2*N, 0, N, N) = (dynCopXBase.UT-dynBasePos.UT)*dynCopXCom.U;
    hessian_.block(3*N, N, N, N) = (dynCopYBase.UT-dynBasePos.UT)*dynCopYCom.U;

    hessian_.block(0, 2*N, N, N) = dynCopXCom.UT*(dynCopXBase.U-dynBasePos.U);
    hessian_.block(N, 3*N, N, N) = dynCopYCom.UT*(dynCopYBase.U-dynBasePos.U);
  }
  else
  {
    hessian_.block(0, 0, N, N) = dynCopXCom.UT*dynCopXCom.U;
    hessian_.block(N, N, N, N) = dynCopYCom.UT*dynCopYCom.U;

    hessian_.block(2*N, 2*N, N, N) = dynBasePos.UT*dynBasePos.U;
    hessian_.block(3*N, 3*N, N, N) = dynBasePos.UT*dynBasePos.U;

    hessian_.block(2*N, 0, N, N) = -dynBasePos.UT*dynCopXCom.U;
    hessian_.block(3*N, N, N, N) = -dynBasePos.UT*dynCopYCom.U;

    hessian_.block(0, 2*N, N, N) = -dynCopXCom.UT*dynBasePos.U;
    hessian_.block(N, 3*N, N, N) = -dynCopYCom.UT*dynBasePos.U;
  }

  tmp_.resize(N);
}

MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(CopCenteringObjective);
