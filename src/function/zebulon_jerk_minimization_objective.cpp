#include "zebulon_jerk_minimization_objective.h"

using namespace MPCWalkgen;

JerkMinimizationObjective::JerkMinimizationObjective(const LIPModel& lipModel,
                                                     const BaseModel& baseModel)
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


JerkMinimizationObjective::~JerkMinimizationObjective(){}

const VectorX& JerkMinimizationObjective::getGradient(const VectorX& x0)
{
  assert(baseModel_.getNbSamples()*4==x0.size());
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());

  return x0;
}

const MatrixX& JerkMinimizationObjective::getHessian()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  return hessian_;
}

void JerkMinimizationObjective::computeConstantPart()
{
  assert(baseModel_.getNbSamples() == lipModel_.getNbSamples());
  assert(baseModel_.getSamplingPeriod() == lipModel_.getSamplingPeriod());

  int N = baseModel_.getNbSamples();
  hessian_ = MatrixX::Identity(N*4, N*4);
}
