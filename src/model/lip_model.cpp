#include "lip_model.h"
#include <cmath>
#include "../tools.h"

using namespace MPCWalkgen;

LIPModel::LIPModel(int nbSamples,
                   Scalar samplingPeriod,
                   bool autoCompute)
:autoCompute_(autoCompute)
,useLipModel2_(false)
,nbSamples_(nbSamples)
,samplingPeriod_(samplingPeriod)
,comHeight_(1.0)
,gravity_(GRAVITY_VECTOR)
,mass_(1.0)
,totalMass_(1.0)
{

  stateX_.setZero(4);
  stateX_(3)=1.0;
  stateY_.setZero(4);
  stateY_(3)=1.0;
  stateZ_.setZero(4);
  stateZ_(0) = comHeight_;
  stateZ_(3) = 1.0;
  if (autoCompute_)
  {
    computeDynamics();
  }
}

LIPModel::LIPModel()
:autoCompute_(true)
,useLipModel2_(false)
,nbSamples_(1)
,samplingPeriod_(1.0)
,comHeight_(1.0)
,gravity_(GRAVITY_VECTOR)
,mass_(1.0)
,totalMass_(1.0)
{

  stateX_.setZero(4);
  stateX_(3)=1.0;
  stateY_.setZero(4);
  stateY_(3)=1.0;
  stateZ_.setZero(4);
  stateZ_(0) = comHeight_;
  stateZ_(3) = 1.0;
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
    Tools::ConstantJerkDynamic::computeCopDynamic(samplingPeriod_, nbSamples_,
                                                  copXDynamic_,  comHeight_,
                                                  gravity_(0), gravity_(2),
                                                  mass_, totalMass_);
}

void LIPModel::computeCopYDynamic()
{
    Tools::ConstantJerkDynamic::computeCopDynamic(samplingPeriod_, nbSamples_,
                                                  copYDynamic_,  comHeight_,
                                                  gravity_(1), gravity_(2),
                                                  mass_, totalMass_);
}

void LIPModel::computeComPosDynamic()
{
  Tools::ConstantJerkDynamic::computePosDynamic(samplingPeriod_, nbSamples_, comPosDynamic_);
}

void LIPModel::computeComVelDynamic()
{
  Tools::ConstantJerkDynamic::computeVelDynamic(samplingPeriod_, nbSamples_, comVelDynamic_);
}

void LIPModel::computeComAccDynamic()
{
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
  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateX_);
}

void LIPModel::updateStateY(Scalar jerk, Scalar feedBackPeriod)
{
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
  stateZ_(0) = comHeight;

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

void LIPModel::setMass(Scalar mass)
{
  assert(mass==mass);

  mass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

void LIPModel::setTotalMass(Scalar mass)
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
