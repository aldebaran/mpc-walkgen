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
    ,kinematicConvexPolygonInWorldFrame_()
  {}

  HumanoidFootModel::HumanoidFootModel(int nbSamples,
                                       Scalar samplingPeriod)
    :nbSamples_(nbSamples)
    ,samplingPeriod_(samplingPeriod)
    ,kinematicLimits_()
    ,CopConvexPolygon_()
    ,CopConvexPolygonInWorldFrame_()
  {
    assert(samplingPeriod>0);
    assert(nbSamples>0);

    init();
  }

  HumanoidFootModel::HumanoidFootModel()
    :nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,kinematicLimits_()
    ,CopConvexPolygon_()
    ,CopConvexPolygonInWorldFrame_()
  {
    init();
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

  void HumanoidFootModel::updateStateX()
  {
    //TODO: Complete
  }

  void HumanoidFootModel::updateStateY()
  {
    //TODO: Complete
  }

  void HumanoidFootModel::updateStateZ()
  {
    //TODO: Complete
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

  void HumanoidFootModel::init()
  {
    //For now we only need to know the foot position, so the state vector size is 2
    stateX_.setZero(2);
    stateX_(1) = 1.0;
    stateY_.setZero(2);
    stateY_(1) = 1.0;
    stateZ_.setZero(2);
    stateZ_(1) = 1.0;
    stateYaw_.setZero(2);
    stateYaw_(1) = 1.0;

    isInContact_.resize(nbSamples_, true);
  }

  void HumanoidFootModel::interpolateFootTrajectory()
  {
    //TODO: Complete
  }

  void HumanoidFootModel::toWorldFrame(ConvexPolygon& convexPolygonInWF,
                                        const ConvexPolygon& convexPolygonInLF)
  {
    //TODO: rotation

    std::vector<Vector2> p(convexPolygonInLF.getNbVertices());

    for(int i=0; i<convexPolygonInLF.getNbVertices(); ++i)
    {
      p[i](0) = convexPolygonInLF.getVertices()[i](0) + stateX_(0);
      p[i](1) = convexPolygonInLF.getVertices()[i](1) + stateY_(0);
    }

    convexPolygonInWF = ConvexPolygon(p);
  }
}

