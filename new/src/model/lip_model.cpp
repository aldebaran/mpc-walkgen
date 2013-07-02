#include "lip_model.h"
#include <cmath>
#include "../tools.h"

using namespace MPCWalkgen;

LIPModel::LIPModel(int nbSamples,
                   Scalar samplingPeriod,
                   Scalar comHeight,
                   Vector3 gravity,
                   bool autoCompute)
:autoCompute_(autoCompute)
,nbSamples_(nbSamples)
,samplingPeriod_(samplingPeriod)
,comHeight_(comHeight)
,gravity_(gravity)
{
  assert(samplingPeriod>0);
  assert(nbSamples>0);
  assert(std::abs(gravity_(2))>EPSILON);

  stateX_.setZero(4);
  stateX_(3)=1.0;
  stateY_.setZero(4);
  stateY_(3)=1.0;
  if (autoCompute_)
  {
    computeDynamics();
  }
}

LIPModel::~LIPModel(){}

void LIPModel::computeDynamics()
{
  computeCopXDynamic();
  computeCopYDynamic();
  computeComPosDynamic();
  computeComVelDynamic();
  computeComAccDynamic();
  computeComJerkDynamic();
}

void LIPModel::computeCopXDynamic()
{
  assert(std::abs(gravity_(2))>EPSILON);
  assert(samplingPeriod_>0);
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computePosDynamic(samplingPeriod_, nbSamples_,
                                                copXDynamic_,
                                                comHeight_*gravity_(0)/gravity_(2),
                                                comHeight_/gravity_(2));
}

void LIPModel::computeCopYDynamic()
{
  assert(std::abs(gravity_(2))>EPSILON);
  assert(samplingPeriod_>0);
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computePosDynamic(samplingPeriod_, nbSamples_,
                                                copYDynamic_,
                                                comHeight_*gravity_(1)/gravity_(2),
                                                comHeight_/gravity_(2));
}

void LIPModel::computeComPosDynamic()
{
  assert(samplingPeriod_>0);
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computePosDynamic(samplingPeriod_, nbSamples_, comPosDynamic_);
}

void LIPModel::computeComVelDynamic()
{
  assert(samplingPeriod_>0);
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computeVelDynamic(samplingPeriod_, nbSamples_, comVelDynamic_);
}

void LIPModel::computeComAccDynamic()
{
  assert(samplingPeriod_>0);
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computeAccDynamic(samplingPeriod_, nbSamples_, comAccDynamic_);
}

void LIPModel::computeComJerkDynamic()
{
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic::computeJerkDynamic(nbSamples_, comJerkDynamic_);
}

void LIPModel::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  nbSamples_ = nbSamples;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

void LIPModel::updateStateX(Scalar jerk, Scalar feedBackPeriod)
{
  assert(jerk==jerk);
  assert(feedBackPeriod>0);

  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateX_);
}

void LIPModel::updateStateY(Scalar jerk, Scalar feedBackPeriod)
{
  assert(jerk==jerk);
  assert(feedBackPeriod>0);

  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateY_);
}


void LIPModel::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

void LIPModel::setComHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  comHeight_ = comHeight;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

void LIPModel::setGravity(const Vector3& gravity)
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
