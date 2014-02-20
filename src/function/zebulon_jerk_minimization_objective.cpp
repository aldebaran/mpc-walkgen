////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/function/zebulon_jerk_minimization_objective.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
JerkMinimizationObjective<Scalar>::JerkMinimizationObjective(const LIPModel<Scalar>& lipModel,
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
JerkMinimizationObjective<Scalar>::~JerkMinimizationObjective(){}

template <typename Scalar>
const typename
Type<Scalar>::VectorX& JerkMinimizationObjective<Scalar>::getGradient(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  return x0;
}

template <typename Scalar>
const typename Type<Scalar>::MatrixX& JerkMinimizationObjective<Scalar>::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

template <typename Scalar>
void JerkMinimizationObjective<Scalar>::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  int N = baseModel_.getNbSamples();
  hessian_ = MatrixX::Identity(N*4, N*4);
}

MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(JerkMinimizationObjective);
