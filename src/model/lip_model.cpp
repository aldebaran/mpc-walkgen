////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/model/lip_model.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/constant.h>
#include <cmath>
#include <cassert>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
LIPModel<Scalar>::LIPModel(int nbSamples,
                   Scalar samplingPeriod,
                   bool autoCompute)
  :autoCompute_(autoCompute)
  ,useLipModel2_(false)
  ,nbSamples_(nbSamples)
  ,samplingPeriod_(samplingPeriod)
  ,feedbackPeriod_(samplingPeriod_)
  ,comHeight_(1.0)
  ,gravity_(Constant<Scalar>::GRAVITY_VECTOR)
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

template <typename Scalar>
LIPModel<Scalar>::LIPModel()
  :autoCompute_(true)
  ,useLipModel2_(false)
  ,nbSamples_(1)
  ,samplingPeriod_(1.0)
  ,feedbackPeriod_(samplingPeriod_)
  ,comHeight_(1.0)
  ,gravity_(Constant<Scalar>::GRAVITY_VECTOR)
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

template <typename Scalar>
LIPModel<Scalar>::~LIPModel(){}

template <typename Scalar>
void LIPModel<Scalar>::computeDynamics()
{
  nbFeedbackInOneSample_ = static_cast<int>(
      (samplingPeriod_ + Constant<Scalar>::EPSILON)/feedbackPeriod_);

  computeCopXDynamicVec();
  computeCopYDynamicVec();
  computeComPosDynamicVec();
  computeComVelDynamicVec();
  computeComAccDynamicVec();
  computeComJerkDynamic();
}

template <typename Scalar>
void LIPModel<Scalar>::computeCopXDynamicVec()
{
  copXDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic<Scalar>::computeCopDynamic(static_cast<Scalar>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  copXDynamicVec_[i],  comHeight_,
                                                  gravity_(0), gravity_(2),
                                                  mass_, totalMass_);
  }
}

template <typename Scalar>
void LIPModel<Scalar>::computeCopYDynamicVec()
{
  copYDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic<Scalar>::computeCopDynamic(static_cast<Scalar>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  copYDynamicVec_[i],  comHeight_,
                                                  gravity_(1), gravity_(2),
                                                  mass_, totalMass_);
  }
}

template <typename Scalar>
void LIPModel<Scalar>::computeComPosDynamicVec()
{
  comPosDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic<Scalar>::computePosDynamic(static_cast<Scalar>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comPosDynamicVec_[i]);
  }
}

template <typename Scalar>
void LIPModel<Scalar>::computeComVelDynamicVec()
{
  comVelDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic<Scalar>::computeVelDynamic(static_cast<Scalar>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comVelDynamicVec_[i]);
  }
}

template <typename Scalar>
void LIPModel<Scalar>::computeComAccDynamicVec()
{
  comAccDynamicVec_.resize(nbFeedbackInOneSample_);

  for (int i=0; i<nbFeedbackInOneSample_; ++i)
  {
    Tools::ConstantJerkDynamic<Scalar>::computeAccDynamic(static_cast<Scalar>(i + 1)*feedbackPeriod_,
                                                  samplingPeriod_, nbSamples_,
                                                  comAccDynamicVec_[i]);
  }
}

template <typename Scalar>
void LIPModel<Scalar>::computeComJerkDynamic()
{
  assert(nbSamples_>0);

  Tools::ConstantJerkDynamic<Scalar>::computeJerkDynamic(nbSamples_, comJerkDynamic_);
}

template <typename Scalar>
void LIPModel<Scalar>::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  nbSamples_ = nbSamples;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
void LIPModel<Scalar>::updateStateX(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, stateX_);
}

template <typename Scalar>
void LIPModel<Scalar>::updateStateY(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, stateY_);
}

template <typename Scalar>
void LIPModel<Scalar>::updateStateYaw(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, stateYaw_);
}

template <typename Scalar>
void LIPModel<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;
  feedbackPeriod_ = samplingPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
void LIPModel<Scalar>::setComHeight(Scalar comHeight)
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

template <typename Scalar>
void LIPModel<Scalar>::setGravity(const Vector3& gravity)
{
  assert(gravity==gravity);
  assert(std::abs(gravity_(2))>Constant<Scalar>::EPSILON);

  gravity_ = gravity;

  if (autoCompute_)
  {
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

template <typename Scalar>
void LIPModel<Scalar>::setMass(Scalar mass)
{
  assert(mass==mass);

  mass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamicVec();
    computeCopYDynamicVec();
  }
}

template <typename Scalar>
void LIPModel<Scalar>::setTotalMass(Scalar mass)
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

template <typename Scalar>
void LIPModel<Scalar>::setFeedbackPeriod(Scalar feedbackPeriod)
{
  assert(feedbackPeriod>=0);
  feedbackPeriod_ = feedbackPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(LIPModel);
}
