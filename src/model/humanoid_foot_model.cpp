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
    ,interpolator_()
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

  void HumanoidFootModel::updateStateX(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateX_, obj, T, t);
  }

  void HumanoidFootModel::updateStateY(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateY_, obj, T, t);
  }

  void HumanoidFootModel::updateStateZ(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateZ_, obj, T, t);
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
    stateX_.setZero(4);
    stateX_(3) = 1.0;
    stateY_.setZero(4);
    stateY_(3) = 1.0;
    stateZ_.setZero(4);
    stateZ_(3) = 1.0;
    stateYaw_.setZero(4);
    stateYaw_(3) = 1.0;

    isInContact_.resize(nbSamples_, true);

    int nbOfPolynomCoefficient = 4;
    factor_.setZero(3*nbOfPolynomCoefficient);
    subFactor_.setZero(nbOfPolynomCoefficient);
  }

  void HumanoidFootModel::interpolateTrajectory(VectorX& currentState,
                                                const Vector3& objState,
                                                Scalar T,
                                                Scalar t)
  {
    interpolator_.computePolynomialNormalisedFactors(factor_,
                                                     currentState.head<3>(),
                                                     objState,
                                                     T);

    interpolator_.selectFactors(subFactor_, factor_, t, T);

    // Interpolated values of the state, its derivative and second derivative
    // are computed here.
    // Division by T is a consequence of the polynoms normalization
    currentState(0) = Tools::polynomValue(subFactor_, t/T);
    currentState(1) = Tools::dPolynomValue(subFactor_, t/T)/T;
    currentState(2) = Tools::ddPolynomValue(subFactor_, t/T)/(T*T);
  }
}

