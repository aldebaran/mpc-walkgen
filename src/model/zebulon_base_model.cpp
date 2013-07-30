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
,comHeight_(0.0)
,gravity_(Vector3(0.0, 0.0, 9.81))
,mass_(0.0)
,totalMass_(1.0)
,velocityLimit_(velocityLimit)
,accelerationLimit_(accelerationLimit)
,jerkLimit_(jerkLimit)
,copSupportHull_(supportHull)
,comSupportHull_(supportHull)
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
  computeCopXDynamic();
  computeCopYDynamic();
}


void BaseModel::computeCopXDynamic()
{
    Tools::ConstantJerkDynamic::computeCopDynamic(samplingPeriod_, nbSamples_,
                                                  copXDynamic_,  comHeight_,
                                                  gravity_(0), gravity_(2),
                                                  mass_, totalMass_);
}

void BaseModel::computeCopYDynamic()
{
    Tools::ConstantJerkDynamic::computeCopDynamic(samplingPeriod_, nbSamples_,
                                                  copYDynamic_,  comHeight_,
                                                  gravity_(1), gravity_(2),
                                                  mass_, totalMass_);
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


void BaseModel::setComHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  comHeight_ = comHeight;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

void BaseModel::setGravity(const Vector3& gravity)
{
  assert(gravity==gravity);
  assert(std::abs(gravity_(2))>EPSILON);

  gravity_ = gravity;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

void BaseModel::setMass(Scalar mass)
{
  assert(mass==mass);

  mass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

void BaseModel::setTotalMass(Scalar mass)
{
  assert(mass==mass);
  assert(mass>=mass_);
  totalMass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}
