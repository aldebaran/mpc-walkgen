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
  ,feedbackPeriod_(samplingPeriod_)
  ,comHeight_(1.0)
  ,gravity_(GRAVITY_VECTOR)
  ,mass_(1.0)
  ,totalMass_(1.0)
{

  stateX_.setZero(3);
  stateY_.setZero(3);
  stateZ_.setZero(3);
  stateZ_(0) = comHeight_;
  stateYaw_.setZero(3);

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
  ,feedbackPeriod_(samplingPeriod_)
  ,comHeight_(1.0)
  ,gravity_(GRAVITY_VECTOR)
  ,mass_(1.0)
  ,totalMass_(1.0)
{

  stateX_.setZero(3);
  stateY_.setZero(3);
  stateZ_.setZero(3);
  stateZ_(0) = comHeight_;
  stateYaw_.setZero(3);

  if (autoCompute_)
  {
    computeDynamics();
  }
}

LIPModel::~LIPModel(){}

void LIPModel::computeDynamics()
{
  nbFeedbackInOneSample_ = static_cast<int>((samplingPeriod_ + EPSILON)/feedbackPeriod_);

  computeCopXDynamicVec();
  computeCopYDynamicVec();
  computeComPosDynamicVec();
  computeComVelDynamicVec();
  computeComAccDynamicVec();
  computeComJerkDynamic();
}

void LIPModel::computeCopXDynamicVec()
{
  copXDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic::computeCopDynamic(static_cast<int>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  copXDynamicVec_[i],  comHeight_,
                                                  gravity_(0), gravity_(2),
                                                  mass_, totalMass_);
  }
}

void LIPModel::computeCopYDynamicVec()
{
  copYDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic::computeCopDynamic(static_cast<int>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  copYDynamicVec_[i],  comHeight_,
                                                  gravity_(1), gravity_(2),
                                                  mass_, totalMass_);
  }
}

void LIPModel::computeComPosDynamicVec()
{
  comPosDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic::computePosDynamic(static_cast<int>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comPosDynamicVec_[i]);
  }
}

void LIPModel::computeComVelDynamicVec()
{
  comVelDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic::computeVelDynamic(static_cast<int>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comVelDynamicVec_[i]);
  }
}

void LIPModel::computeComAccDynamicVec()
{
  comAccDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic::computeAccDynamic(static_cast<int>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comAccDynamicVec_[i]);
  }
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

void LIPModel::updateStateYaw(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic::updateState(jerk, feedBackPeriod, stateYaw_);
}

void LIPModel::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;
  feedbackPeriod_ = samplingPeriod;

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
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

void LIPModel::setGravity(const Vector3& gravity)
{
  assert(gravity==gravity);
  assert(std::abs(gravity_(2))>EPSILON);

  gravity_ = gravity;

  if (autoCompute_)
  {
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

void LIPModel::setMass(Scalar mass)
{
  assert(mass==mass);

  mass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

void LIPModel::setTotalMass(Scalar mass)
{
  assert(mass==mass);
  assert(mass>=mass_);
  totalMass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

void LIPModel::setFeedbackPeriod(Scalar feedbackPeriod)
{
  assert(feedbackPeriod>=0);
  feedbackPeriod_ = feedbackPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}
