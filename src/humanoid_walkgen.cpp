////////////////////////////////////////////////////////////////////////////////
///
///\file humanoid_walkgen.cpp
///\brief Main program for Humanoid
///\author de Gourcuff Martin
///\date 11/07/13
///
////////////////////////////////////////////////////////////////////////////////

#include "humanoid_walkgen.h"

namespace MPCWalkgen
{

  //TODO: MAke leftFootModel and RightFootModel disappear
  HumanoidWalkgen::HumanoidWalkgen()
    :feetSupervisor_(leftFootModel_, rightFootModel_)
    ,velTrackingObj_(lipModel_, feetSupervisor_)
    ,jerkMinObj_(lipModel_, feetSupervisor_)
    ,copCenteringObj_(lipModel_, feetSupervisor_)
    ,copConstraint_(lipModel_, feetSupervisor_)
    ,footConstraint_(lipModel_, feetSupervisor_)
    ,qpoasesSolver_(1, 1)
    ,weighting_()
    ,config_()
  {
    computeConstantPart();
  }

  HumanoidWalkgen::~HumanoidWalkgen(){}

  void HumanoidWalkgen::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    lipModel_.setNbSamples(nbSamples);
    feetSupervisor_.setNbSamples(nbSamples);

    copCenteringObj_.computeConstantPart();

    copConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setSamplingPeriod(Scalar samplingPeriod)
  {
    assert(samplingPeriod>=0);

    lipModel_.setSamplingPeriod(samplingPeriod);
    feetSupervisor_.setSamplingPeriod(samplingPeriod);

    copCenteringObj_.computeConstantPart();

    copConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setStepPeriod(Scalar stepPeriod)
  {
    assert(stepPeriod>0.0);
    feetSupervisor_.setStepPeriod(stepPeriod);
  }

  void HumanoidWalkgen::setLeftFootKinematicConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setLeftFootKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setRightFootKinematicConvexPolygon(const ConvexPolygon &convexPolygon)
  {
    feetSupervisor_.setRightFootKinematicConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setLeftFootCopConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setLeftFootCopConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setRightFootCopConvexPolygon(const ConvexPolygon& convexPolygon)
  {
    feetSupervisor_.setRightFootCopConvexPolygon(convexPolygon);
  }

  void HumanoidWalkgen::setVelRefInWorldFrame(const VectorX& velRef)
  {
    //TODO: complete
  }


  //TODO: for all state setters, add the 2nd or 4th element (=1)
  //Makes the public API more intuitive
  void HumanoidWalkgen::setLeftFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateX(state);
  }

  void HumanoidWalkgen::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateY(state);
  }

  void HumanoidWalkgen::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setLeftFootStateZ(state);
  }

  void HumanoidWalkgen::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateX(state);
  }

  void HumanoidWalkgen::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateY(state);
  }

  void HumanoidWalkgen::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    feetSupervisor_.setRightFootStateZ(state);
  }

  void HumanoidWalkgen::setComStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(3)==1.0);
    lipModel_.setStateX(state);
  }

  void HumanoidWalkgen::setComStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(3)==1.0);
    lipModel_.setStateY(state);
  }

  void HumanoidWalkgen::setComStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==4);
    assert(state(1)==0.0);
    assert(state(2)==0.0);
    assert(state(3)==1.0);
    lipModel_.setComHeight(state(0));
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateX() const
  {
    return leftFootModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateY() const
  {
    return leftFootModel_.getStateY();
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateZ() const
  {
    return leftFootModel_.getStateZ();
  }

  const VectorX& HumanoidWalkgen::getRightFootStateX() const
  {
    return rightFootModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getRightFootStateY() const
  {
    return rightFootModel_.getStateY();
  }
  const VectorX& HumanoidWalkgen::getRightFootStateZ() const
  {
    return rightFootModel_.getStateZ();
  }

  const VectorX& HumanoidWalkgen::getComStateX() const
  {
    return lipModel_.getStateX();
  }

  const VectorX& HumanoidWalkgen::getComStateY() const
  {
    return lipModel_.getStateY();
  }

  const VectorX& HumanoidWalkgen::getComStateZ() const
  {
    return lipModel_.getStateZ();
  }

  void HumanoidWalkgen::setLeftFootMaxHeight(
      Scalar leftFootMaxHeight)
  {
    feetSupervisor_.setLeftFootMaxHeight(
          leftFootMaxHeight);
  }

  void HumanoidWalkgen::setRightFootMaxHeight(
      Scalar rightFootMaxHeight)
  {
        feetSupervisor_.setRightFootMaxHeight(
          rightFootMaxHeight);
  }

  void HumanoidWalkgen::setLeftFootYawUpperBound(Scalar leftFootYawUpperBound)
  {
    feetSupervisor_.setLeftFootYawUpperBound(
          leftFootYawUpperBound);
  }
  void HumanoidWalkgen::setLeftFootYawLowerBound(Scalar leftFootYawLowerBound)
  {
    feetSupervisor_.setLeftFootYawLowerBound(
          leftFootYawLowerBound);
  }
  void HumanoidWalkgen::setRightFootYawUpperBound(Scalar rightFootYawUpperBound)
  {
    feetSupervisor_.setRightFootYawUpperBound(
          rightFootYawUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawLowerBound(Scalar rightFootYawLowerBound)
  {
    feetSupervisor_.setRightFootYawLowerBound(
          rightFootYawLowerBound);
  }

  void HumanoidWalkgen::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    feetSupervisor_.setLeftFootYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    feetSupervisor_.setRightFootYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  void HumanoidWalkgen::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setLeftFootYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    feetSupervisor_.setRightFootYawAccelerationUpperBound(
          rightFootYawAccelerationUpperBound);
  }


  void HumanoidWalkgen::setWeightings(const HumanoidWalkgenImpl::Weighting& weighting){}
  void HumanoidWalkgen::setConfig(const HumanoidWalkgenImpl::Config& config){}

  void HumanoidWalkgen::solve(Scalar feedBackPeriod)
  {


    int N = lipModel_.getNbSamples();
    int NbCtr1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int NbCtr2 = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;

    assert(feedBackPeriod>0);

    qpMatrix_.p.fill(0.0);
    qpMatrix_.bu.fill(std::numeric_limits<Scalar>::max());
    qpMatrix_.bl.fill(-std::numeric_limits<Scalar>::max());
    qpMatrix_.xu.fill(std::numeric_limits<Scalar>::max());
    qpMatrix_.xl.fill(-std::numeric_limits<Scalar>::max());

    //TODO: Complete
  }

  void HumanoidWalkgen::computeConstantPart()
  {
    int N = lipModel_.getNbSamples();
    int NbCtr1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int NbCtr2 = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;
    int NbCtr = NbCtr1+NbCtr2;

    //TODO: Complete
  }
}
