#include "zebulon_cop_constraint.h"

using namespace MPCWalkgen;

CopConstraint::CopConstraint(const LIPModel& lipModel,
                             const BaseModel& baseModel)
:lipModel_(lipModel)
,baseModel_(baseModel)
,function_(1)
,b_(1)
,tmp_(1)
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  function_.fill(0);
  gradient_.setZero(1, 1);
  hessian_.setZero(1, 1);
  A_.setZero(1, 1);
  b_.fill(0);

  computeConstantPart();
}


CopConstraint::~CopConstraint(){}

const VectorX& CopConstraint::getFunction(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
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

    tmp_.segment(0, N).noalias() = dynCopXCom.S * lipModel_.getStateX();
    tmp_.segment(0, N).noalias() += (dynCopXBase.S-dynBasePos.S) * baseModel_.getStateX();
    tmp_.segment(N, N).noalias() = dynCopYCom.S * lipModel_.getStateY() ;
    tmp_.segment(N, N).noalias() += (dynCopYBase.S-dynBasePos.S) * baseModel_.getStateY();

    function_.noalias() = -b_;
    function_.noalias() -= getGradient()*x0;
    function_.noalias() -= A_*tmp_;
  }
  else
  {
    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    tmp_.segment(0, N).noalias() = dynCopX.S * lipModel_.getStateX();
    tmp_.segment(0, N).noalias() -= dynBasePos.S * baseModel_.getStateX();
    tmp_.segment(N, N).noalias() = dynCopY.S * lipModel_.getStateY() ;
    tmp_.segment(N, N).noalias() -= dynBasePos.S * baseModel_.getStateY();

    function_.noalias() = -b_;
    function_.noalias() -= getGradient()*x0;
    function_.noalias() -= A_*tmp_;
  }

  return function_;
}

const MatrixX& CopConstraint::getGradient()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return gradient_;
}

int CopConstraint::getNbConstraints()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  return baseModel_.getNbSamples()*baseModel_.getCopSupportHull().p.size();
}

void CopConstraint::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  computeconstraintMatrices();

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();

  int N = lipModel_.getNbSamples();

  if (baseModel_.getMass()>EPSILON)
  {
    const LinearDynamic& dynCopXCom = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYCom = lipModel_.getCopYLinearDynamic();
    const LinearDynamic& dynCopXBase = baseModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopYBase = baseModel_.getCopYLinearDynamic();

    MatrixX tmp(2*N, 4*N);
    tmp.fill(0.0);
    tmp.block(0, 0, N, N) = dynCopXCom.U;
    tmp.block(N, N, N, N) = dynCopYCom.U;
    tmp.block(0, 2*N, N, N) = (dynCopXBase.U-dynBasePos.U);
    tmp.block(N, 3*N, N, N) = (dynCopYBase.U-dynBasePos.U);

    gradient_ = A_ * tmp;
  }
  else
  {
    const LinearDynamic& dynCopX = lipModel_.getCopXLinearDynamic();
    const LinearDynamic& dynCopY = lipModel_.getCopYLinearDynamic();

    MatrixX tmp(2*N, 4*N);
    tmp.fill(0.0);
    tmp.block(0, 0, N, N) = dynCopX.U;
    tmp.block(N, N, N, N) = dynCopY.U;
    tmp.block(0, 2*N, N, N) = -dynBasePos.U;
    tmp.block(N, 3*N, N, N) = -dynBasePos.U;

    gradient_ = A_ * tmp;
  }


  tmp_.resize(2*N);

}

void CopConstraint::computeconstraintMatrices()
{
  int N = lipModel_.getNbSamples();
  const Hull& supportHull = baseModel_.getCopSupportHull();
  int M = supportHull.p.size();

  A_.setZero(M*N, 2*N);
  b_.resize(M*N);

  for(int i=0; i<M; ++i)
  {
    const Vector3& p1 = supportHull.p[i];
    const Vector3& p2 = supportHull.p[(i+1)%M];

    for(int j=0; j<N; ++j)
    {
      A_(i*N+j, j) = p2(1)-p1(1);
      A_(i*N+j, j+N) = -p2(0)+p1(0);
      b_(i*N+j) = -p1(1)*(p2(0)-p1(0)) + p1(0)*(p2(1)-p1(1));
    }
  }

}
