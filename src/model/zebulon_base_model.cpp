////////////////////////////////////////////////////////////////////////////////
///
///\author Lafaye Jory
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/model/zebulon_base_model.h>
#include <cmath>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/constant.h>
#include "../macro.h"

using namespace MPCWalkgen;

template <typename Scalar>
BaseModel<Scalar>::BaseModel(int nbSamples,
                     Scalar samplingPeriod,
                     bool autoCompute)
  :autoCompute_(autoCompute)
  ,nbSamples_(nbSamples)
  ,samplingPeriod_(samplingPeriod)
  ,comHeight_(0.0)
  ,gravity_(Constant<Scalar>::GRAVITY_VECTOR)
  ,mass_(0.0)
  ,totalMass_(1.0)
  ,velocityLimit_(1.0)
  ,accelerationLimit_(1.0)
  ,jerkLimit_(1.0)
  ,tiltContactPointX_(0.0)
  ,tiltContactPointY_(0.0)
  ,copSupportConvexPolygon_()
  ,comSupportConvexPolygon_()
{
  assert(samplingPeriod>0);
  assert(nbSamples>0);

  stateX_.setZero(3);
  stateY_.setZero(3);
  stateRoll_.setZero(3);
  statePitch_.setZero(3);
  stateYaw_.setZero(3);
  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
BaseModel<Scalar>::BaseModel()
  :autoCompute_(true)
  ,nbSamples_(1)
  ,samplingPeriod_(1.0)
  ,comHeight_(0.0)
  ,gravity_(Constant<Scalar>::GRAVITY_VECTOR)
  ,mass_(0.0)
  ,totalMass_(1.0)
  ,velocityLimit_(1.0)
  ,accelerationLimit_(1.0)
  ,jerkLimit_(1.0)
  ,copSupportConvexPolygon_()
  ,comSupportConvexPolygon_()
{
  stateX_.setZero(3);
  stateY_.setZero(3);
  stateRoll_.setZero(3);
  statePitch_.setZero(3);
  stateYaw_.setZero(3);
  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
BaseModel<Scalar>::~BaseModel(){}

template <typename Scalar>
void BaseModel<Scalar>::computeDynamics()
{
  computeBasePosDynamic();
  computeBaseVelDynamic();
  computeBaseAccDynamic();
  computeBaseJerkDynamic();
  computeCopXDynamic();
  computeCopYDynamic();
  computeTiltDynamic();
}


template <typename Scalar>
void BaseModel<Scalar>::computeTiltDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeOrder2PosDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, baseTiltAngleDynamic_);
  Tools::ConstantJerkDynamic<Scalar>::computeOrder2VelDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, baseTiltAngularVelDynamic_);
}

template <typename Scalar>
void BaseModel<Scalar>::computeCopXDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeCopDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, copXDynamic_,
                                                comHeight_, gravity_(0),
                                                gravity_(2), mass_,
                                                totalMass_);
}

template <typename Scalar>
void BaseModel<Scalar>::computeCopYDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeCopDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, copYDynamic_,
                                                comHeight_, gravity_(1),
                                                gravity_(2), mass_,
                                                totalMass_);
}

template <typename Scalar>
void BaseModel<Scalar>::computeBasePosDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computePosDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, basePosDynamic_);

}

template <typename Scalar>
void BaseModel<Scalar>::computeBaseVelDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeVelDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, baseVelDynamic_);
}

template <typename Scalar>
void BaseModel<Scalar>::computeBaseAccDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeAccDynamic(samplingPeriod_, samplingPeriod_,
                                                nbSamples_, baseAccDynamic_);
}

template <typename Scalar>
void BaseModel<Scalar>::computeBaseJerkDynamic()
{
  Tools::ConstantJerkDynamic<Scalar>::computeJerkDynamic(nbSamples_, baseJerkDynamic_);
}

template <typename Scalar>
void BaseModel<Scalar>::setNbSamples(int nbSamples)
{
  assert(nbSamples>0);

  nbSamples_ = nbSamples;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
void BaseModel<Scalar>::updateStateX(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, stateX_);
}

template <typename Scalar>
void BaseModel<Scalar>::updateStateY(Scalar jerk, Scalar feedBackPeriod)
{
  Tools::ConstantJerkDynamic<Scalar>::updateState(jerk, feedBackPeriod, stateY_);
}

template <typename Scalar>
void BaseModel<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
{
  assert(samplingPeriod>0);

  samplingPeriod_ = samplingPeriod;

  if (autoCompute_)
  {
    computeDynamics();
  }
}

template <typename Scalar>
void BaseModel<Scalar>::setComHeight(Scalar comHeight)
{
  assert(comHeight==comHeight);

  comHeight_ = comHeight;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

template <typename Scalar>
void BaseModel<Scalar>::setGravity(const Vector3& gravity)
{
  assert(gravity==gravity);
  assert(std::abs(gravity_(2))>Constant<Scalar>::EPSILON);

  gravity_ = gravity;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

template <typename Scalar>
void BaseModel<Scalar>::setMass(Scalar mass)
{
  assert(mass==mass);
  assert(mass>0.);

  mass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

template <typename Scalar>
void BaseModel<Scalar>::setTotalMass(Scalar mass)
{
  assert(mass==mass);
  assert(mass>=mass_);
  assert(mass>0.);
  totalMass_ = mass;

  if (autoCompute_)
  {
    computeCopXDynamic();
    computeCopYDynamic();
  }
}

namespace MPCWalkgen
{
  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(BaseModel);
}
