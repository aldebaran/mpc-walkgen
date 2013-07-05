#include "zebulon_base_model.h"
#include <cmath>
#include "../tools.h"

using namespace MPCWalkgen;


BaseModel::BaseModel(int nbSamples,
                     Scalar samplingPeriod,
                     Scalar velocityLimit,
                     Scalar accelerationLimit,
                     Scalar jerkLimit,
                     const Hull &supportHull,
                     bool autoCompute)
:autoCompute_(autoCompute)
,nbSamples_(nbSamples)
,samplingPeriod_(samplingPeriod)
,velocityLimit_(velocityLimit)
,accelerationLimit_(accelerationLimit)
,jerkLimit_(jerkLimit)
,supportHull_(supportHull)
{
  assert(samplingPeriod>0);
  assert(nbSamples>0);
  assert(velocityLimit>=0);
  assert(accelerationLimit>=0);
  assert(jerkLimit>=0);
  assert(supportHull.p.size()>=3);

  stateX_.setZero(4);
  stateX_(3)=1.0;
  stateY_.setZero(4);
  stateY_(3)=1.0;
  if (autoCompute_)
  {
    computeDynamics();
  }
}

BaseModel::~BaseModel(){}

void BaseModel::computeDynamics()
{
  computeBasePosDynamic();
  computeBaseVelDynamic();
  computeBaseAccDynamic();
  computeBaseJerkDynamic();
}

void BaseModel::computeBasePosDynamic()
{
  Tools::ConstantJerkDynamic::computePosDynamic(samplingPeriod_, nbSamples_, basePosDynamic_);

}

void BaseModel::computeBaseVelDynamic()
{
  Tools::ConstantJerkDynamic::computeVelDynamic(samplingPeriod_, nbSamples_, baseVelDynamic_);
}

void BaseModel::computeBaseAccDynamic()
{
  Tools::ConstantJerkDynamic::computeAccDynamic(samplingPeriod_, nbSamples_, baseAccDynamic_);
}

void BaseModel::computeBaseJerkDynamic()
{
  Tools::ConstantJerkDynamic::computeJerkDynamic(nbSamples_, baseJerkDynamic_);
}


void BaseModel::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  nbSamples_ = nbSamples;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

void BaseModel::updateStateX(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateX_);
}

void BaseModel::updateStateY(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateY_);
}

void BaseModel::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}
