////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_model.cpp
///\brief Implement of a foot model
///\author de Gourcuff Martin
///\date 11/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_foot_model.h"

namespace MPCWalkgen
{
  HumanoidFootModel::KinematicLimits::KinematicLimits()
    :hipYawUpperBound_(0)
    ,hipYawSpeedUpperBound_(0)
    ,hipYawAccelerationUpperBound_(0)
    ,hipYawLowerBound_(0)
    ,maxHeight_(0)
    ,kinematicConvexPolygon_()
  {}

  HumanoidFootModel::HumanoidFootModel(int nbSamples,
                                       Scalar samplingPeriod,
                                       int nbPreviewedSteps)
    :nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,nbPreviewedSteps_(nbPreviewedSteps)
    ,kinematicLimits_()
    ,CopConvexPolygon_()
  {
    assert(samplingPeriod>0);
    assert(nbSamples>0);

    xInit();
  }

  HumanoidFootModel::HumanoidFootModel()
    :nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,nbPreviewedSteps_(0)
    ,kinematicLimits_()
    ,CopConvexPolygon_()
  {
    xInit();
  }

  HumanoidFootModel::~HumanoidFootModel(){}

  void HumanoidFootModel::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    nbSamples_ = nbSamples;
  }

  void HumanoidFootModel::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>0);

    samplingPeriod_ = samplingPeriod;
  }





  void HumanoidFootModel::setHipYawUpperBound(
      Scalar hipYawUpperBound)
  {kinematicLimits_.hipYawUpperBound_ = hipYawUpperBound;}

  void HumanoidFootModel::setHipYawSpeedUpperBound(
      Scalar hipYawSpeedUpperBound)
  {kinematicLimits_.hipYawSpeedUpperBound_ = hipYawSpeedUpperBound;}

  void HumanoidFootModel::setHipYawAccelerationUpperBound(
      Scalar hipYawAccelerationUpperBound)
  {kinematicLimits_.hipYawAccelerationUpperBound_ = hipYawAccelerationUpperBound;}

  void HumanoidFootModel::setHipYawLowerBound(
      Scalar hipYawLowerBound)
  {kinematicLimits_.hipYawLowerBound_ = hipYawLowerBound;}

  void HumanoidFootModel::setMaxHeight(
      Scalar maxHeight)
  {kinematicLimits_.maxHeight_ = maxHeight;}

  void HumanoidFootModel::setKinematicConvexPolygon(const ConvexPolygon& kinematicConvexPolygon)
  {
    kinematicLimits_.kinematicConvexPolygon_ = kinematicConvexPolygon;
  }

  void HumanoidFootModel::xInit()
  {
    //For now we only need to know the foot position, so the state vector size is 1
    stateX_.setZero(1);
    stateY_.setZero(1);
    stateZ_.setZero(1);
    stateYaw_.setZero(1);

    isInContact_.resize(nbSamples_, true);
    isSupportFoot_.resize(nbPreviewedSteps_, true);
  }

}

