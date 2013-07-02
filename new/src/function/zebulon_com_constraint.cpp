#include "zebulon_com_constraint.h"

using namespace MPCWalkgen;

ComConstraint::ComConstraint(const LIPModel& lipModel,
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


ComConstraint::~ComConstraint(){}

const VectorX& ComConstraint::getFunction(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic& dynCom = lipModel_.getComPosLinearDynamic();

  int N = lipModel_.getNbSamples();

  tmp_.segment(0, N).noalias() = dynCom.S * lipModel_.getStateX();
  tmp_.segment(0, N).noalias() -= dynBasePos.S * baseModel_.getStateX();
  tmp_.segment(N, N).noalias() = dynCom.S * lipModel_.getStateY() ;
  tmp_.segment(N, N).noalias() -= dynBasePos.S * baseModel_.getStateY();

  function_.noalias() = -b_;
  function_.noalias() -= getGradient()*x0;
  function_.noalias() -= A_*tmp_;

  return function_;
}

const MatrixX& ComConstraint::getGradient()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return gradient_;
}

int ComConstraint::getNbConstraints()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  return baseModel_.getNbSamples()*baseModel_.getSupportHull().p.size();
}

void ComConstraint::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  computeconstraintMatrices();

  const LinearDynamic& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic& dynCom = lipModel_.getComPosLinearDynamic();

  int N = lipModel_.getNbSamples();


  MatrixX tmp(2*N, 4*N);
  tmp.fill(0.0);
  tmp.block(0, 0, N, N) = dynCom.U;
  tmp.block(N, N, N, N) = dynCom.U;
  tmp.block(0, 2*N, N, N) = -dynBasePos.U;
  tmp.block(N, 3*N, N, N) = -dynBasePos.U;

  gradient_ = A_ * tmp;

  tmp_.resize(2*N);

}

void ComConstraint::computeconstraintMatrices()
{
  int N = lipModel_.getNbSamples();
  const Hull& supportHull = baseModel_.getSupportHull();
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
