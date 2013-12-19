////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_foot_model.cpp
///\brief Implement of a foot model
///\author de Gourcuff Martin
///\author Barthelemy Sebastien
///
////////////////////////////////////////////////////////////////////////////////
#include <mpc-walkgen/model/humanoid_foot_model.h>
#include "../macro.h"

namespace MPCWalkgen
{
  template <typename Scalar>
  HumanoidFootModel<Scalar>::KinematicLimits::KinematicLimits()
    :hipYawUpperBound_(0)
    ,hipYawSpeedUpperBound_(0)
    ,hipYawAccelerationUpperBound_(0)
    ,hipYawLowerBound_(0)
    ,maxHeight_(0)
    ,kinematicConvexPolygon_()
    ,kinematicConvexPolygonInWorldFrame_()
  {}

  template <typename Scalar>
  HumanoidFootModel<Scalar>::HumanoidFootModel(int nbSamples,
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

  template <typename Scalar>
  HumanoidFootModel<Scalar>::HumanoidFootModel()
    :nbSamples_(1)
    ,samplingPeriod_(1.0)
    ,kinematicLimits_()
    ,CopConvexPolygon_()
    ,CopConvexPolygonInWorldFrame_()
  {
    init();
  }

  template <typename Scalar>
  HumanoidFootModel<Scalar>::~HumanoidFootModel(){}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    nbSamples_ = nbSamples;
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>0);

    samplingPeriod_ = samplingPeriod;
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::updateStateX(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateX_, obj, T, t);
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::updateStateY(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateY_, obj, T, t);
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::updateStateZ(const Vector3& obj,
                                       Scalar T,
                                       Scalar t)
  {
    interpolateTrajectory(stateZ_, obj, T, t);
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setHipYawUpperBound(
      Scalar hipYawUpperBound)
  {kinematicLimits_.hipYawUpperBound_ = hipYawUpperBound;}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setHipYawSpeedUpperBound(
      Scalar hipYawSpeedUpperBound)
  {kinematicLimits_.hipYawSpeedUpperBound_ = hipYawSpeedUpperBound;}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setHipYawAccelerationUpperBound(
      Scalar hipYawAccelerationUpperBound)
  {kinematicLimits_.hipYawAccelerationUpperBound_ = hipYawAccelerationUpperBound;}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setHipYawLowerBound(
      Scalar hipYawLowerBound)
  {kinematicLimits_.hipYawLowerBound_ = hipYawLowerBound;}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setMaxHeight(
      Scalar maxHeight)
  {kinematicLimits_.maxHeight_ = maxHeight;}

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::setKinematicConvexPolygon(
                                                const ConvexPolygon<Scalar>& kinematicConvexPolygon)
  {
    kinematicLimits_.kinematicConvexPolygon_ = kinematicConvexPolygon;
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::init()
  {
    stateX_.setZero(3);
    stateY_.setZero(3);
    stateZ_.setZero(3);
    stateYaw_.setZero(3);

    isInContact_.resize(nbSamples_, true);

    int nbOfPolynomCoefficient = 4;
    factor_.setZero(3*nbOfPolynomCoefficient);
    subFactor_.setZero(nbOfPolynomCoefficient);
  }

  template <typename Scalar>
  void HumanoidFootModel<Scalar>::interpolateTrajectory(VectorX& currentState,
                                                const Vector3& objState,
                                                Scalar T,
                                                Scalar t)
  {
    interpolator_.computePolynomialNormalisedFactors(factor_,
                                                     currentState,
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

  MPC_WALKGEN_INSTANTIATE_CLASS_TEMPLATE(HumanoidFootModel);
}
