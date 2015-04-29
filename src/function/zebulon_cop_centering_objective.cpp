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

    copRef_.noalias() = -copRefInLocalFrame_.segment(0, N);
    copRef_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    copRef_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX() +
                         dynCopXBase.K;
    gradient_.block(0, 0, N, 1).noalias() += dynCopXCom.UT*copRef_;


    copRef_.noalias() = -copRefInLocalFrame_.segment(N, N);
    copRef_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    copRef_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY() +
                         dynCopYBase.K;
    gradient_.block(N, 0, N, 1).noalias() += dynCopYCom.UT*copRef_;

    copRef_.noalias() = -copRefInLocalFrame_.segment(0, N);
    copRef_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    copRef_.noalias() += (dynCopXBase.S-dynBasePos.S)*baseModel_.getStateX() +
                         dynCopXBase.K;
    gradient_.block(2*N, 0, N, 1).noalias() +=
        (dynCopXBase.UT-dynBasePos.UT)*copRef_;


    copRef_.noalias() = -copRefInLocalFrame_.segment(N, N);
    copRef_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    copRef_.noalias() += (dynCopYBase.S-dynBasePos.S)*baseModel_.getStateY() +
                         dynCopYBase.K;
    gradient_.block(3*N, 0, N, 1).noalias() +=
        (dynCopYBase.UT-dynBasePos.UT)*copRef_;

  }
  else
  {

    copRef_.noalias() = -copRefInLocalFrame_.segment(0, N);
    copRef_.noalias() += dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    copRef_.noalias() -= dynBasePos.S*baseModel_.getStateX();
    gradient_.block(0, 0, N, 1).noalias() += dynCopXCom.UT*copRef_;


    copRef_.noalias() = -copRefInLocalFrame_.segment(N, N);
    copRef_.noalias() += dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    copRef_.noalias() -= dynBasePos.S*baseModel_.getStateY();
    gradient_.block(N, 0, N, 1).noalias() += dynCopYCom.UT*copRef_;

    copRef_.noalias() = copRefInLocalFrame_.segment(0, N);
    copRef_.noalias() -= dynCopXCom.S*lipModel_.getStateX() + dynCopXCom.K;
    copRef_.noalias() += dynBasePos.S*baseModel_.getStateX();
    gradient_.block(2*N, 0, N, 1).noalias() += dynBasePos.UT*copRef_;


    copRef_.noalias() = copRefInLocalFrame_.segment(N, N);
    copRef_.noalias() -= dynCopYCom.S*lipModel_.getStateY() + dynCopYCom.K;
    copRef_.noalias() += dynBasePos.S*baseModel_.getStateY();
    gradient_.block(3*N, 0, N, 1).noalias() += dynBasePos.UT*copRef_;
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
void CopCenteringObjective<Scalar>::setCopRefInLocalFrame(const VectorX& copRefInLocalFrame)
{
  assert(copRefInLocalFrame.size() == lipModel_.getNbSamples()*2);
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());
  assert(copRefInLocalFrame==copRefInLocalFrame);

  copRefInLocalFrame_ = copRefInLocalFrame;
}

template <typename Scalar>
void CopCenteringObjective<Scalar>::setCopRefInWorldFrame(
    const VectorX& comPosRefInWorldFrame,
    const VectorX& comAccRefInWorldFrame,
    const VectorX& basePosRefInWorldFrame,
    const VectorX& baseAccRefInWorldFrame)
{
  const int N = lipModel_.getNbSamples();
#ifndef NDEBUG
  // Only used in following asserts
  const int refSize = 2 * N;
#endif
  assert(comPosRefInWorldFrame==comPosRefInWorldFrame);
  assert(comAccRefInWorldFrame==comAccRefInWorldFrame);
  assert(basePosRefInWorldFrame==basePosRefInWorldFrame);
  assert(baseAccRefInWorldFrame==baseAccRefInWorldFrame);
  assert(comPosRefInWorldFrame.size() == refSize);
  assert(comAccRefInWorldFrame.size() == refSize);
  assert(basePosRefInWorldFrame.size() == refSize);
  assert(baseAccRefInWorldFrame.size() == refSize);
  assert(baseModel_.getNbSamples() == N);
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  Scalar mb = baseModel_.getMass();
  Scalar mc = lipModel_.getMass();
  Scalar hb = baseModel_.getComHeight();
  Scalar hc = lipModel_.getComHeight();


  const Vector3& g = lipModel_.getGravity();

  constantGravity_.segment(0, N).fill(g[0]);
  constantGravity_.segment(N, N).fill(g[1]);

  //Compute the CoP reference of a double masses model

  // (total mass) x gravity_z is always non null
  assert(std::abs((mb + mc) * g[2]) > 1e-6f);

  copRefInLocalFrame_ = ( mb*g[2]*basePosRefInWorldFrame - mb*hb*baseAccRefInWorldFrame
                        + mc*g[2]*comPosRefInWorldFrame - mc*hc*comAccRefInWorldFrame
                        + (mb*hb+mc*hc)*constantGravity_) / ((mb+mc)*g[2])

                        - basePosRefInWorldFrame; //In local Frame
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

  copRef_.resize(N);
  constantGravity_.resize(2*N);
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(CopCenteringObjective);
}
