////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_com_constraint.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
ComConstraint<Scalar>::ComConstraint(const LIPModel<Scalar>& lipModel,
                                     const BaseModel<Scalar>& baseModel)
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


template <typename Scalar>
ComConstraint<Scalar>::~ComConstraint(){}

template <typename Scalar>
const typename Type<Scalar>::VectorX& ComConstraint<Scalar>::getFunction(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  const LinearDynamic<Scalar>& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic<Scalar>& dynCom = lipModel_.getComPosLinearDynamic();

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

template <typename Scalar>
const typename Type<Scalar>::MatrixX& ComConstraint<Scalar>::getGradient()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return gradient_;
}

template <typename Scalar>
int ComConstraint<Scalar>::getNbConstraints()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  return baseModel_.getNbSamples()*baseModel_.getComSupportConvexPolygon().getNbVertices();
}

template <typename Scalar>
void ComConstraint<Scalar>::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  computeconstraintMatrices();

  const LinearDynamic<Scalar>& dynBasePos = baseModel_.getBasePosLinearDynamic();
  const LinearDynamic<Scalar>& dynCom = lipModel_.getComPosLinearDynamic();

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

template <typename Scalar>
void ComConstraint<Scalar>::computeconstraintMatrices()
{
  int N = lipModel_.getNbSamples();
  const ConvexPolygon<Scalar>& supportConvexPolygon = baseModel_.getComSupportConvexPolygon();
  int M = supportConvexPolygon.getNbVertices();

  A_.setZero(M*N, 2*N);
  b_.resize(M*N);

  for(int i=0; i<M; ++i)
  {
    const Vector2& p1 = supportConvexPolygon.getVertices()[i];
    const Vector2& p2 = supportConvexPolygon.getVertices()[(i+1)%M];

    for(int j=0; j<N; ++j)
    {
      A_(i*N+j, j) = p2(1)-p1(1);
      A_(i*N+j, j+N) = -p2(0)+p1(0);
      b_(i*N+j) = -p1(1)*(p2(0)-p1(0)) + p1(0)*(p2(1)-p1(1));
    }
  }
}

MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(ComConstraint);
