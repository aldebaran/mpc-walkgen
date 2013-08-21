#include "humanoid_walkgen.h"

namespace MPCWalkgen
{
  HumanoidWalkgen::HumanoidWalkgen()
    :velTrackingObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,jerkMinObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,copCenteringObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,copConstraint_(lipModel_, leftFootModel_, rightFootModel_)
    ,footConstraint_(lipModel_, leftFootModel_, rightFootModel_)
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
    leftFootModel_.setNbSamples(nbSamples);
    rightFootModel_.setNbSamples(nbSamples);

    copCenteringObj_.computeConstantPart();
    footConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setSamplingPeriod(Scalar samplingPeriod)
  {
    lipModel_.setSamplingPeriod(samplingPeriod);
    leftFootModel_.setSamplingPeriod(samplingPeriod);
    rightFootModel_.setSamplingPeriod(samplingPeriod);

    copCenteringObj_.computeConstantPart();
    footConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setStepPeriod(Scalar stepPeriod)
  {

  }

  void HumanoidWalkgen::setLeftFootKinematicHull(const Hull& hull)
  {
    assert(hull.p.size()>=3);
    leftFootModel_.setKinematicHull(hull);
  }

  void HumanoidWalkgen::setRightFootKinematicHull(const Hull &hull)
  {
    assert(hull.p.size()>=3);
    rightFootModel_.setKinematicHull(hull);
  }

  void HumanoidWalkgen::setLeftFootCopSSHull(const Hull& hull)
  {
    assert(hull.p.size()==4);
    leftFootModel_.setCopSSHull(hull);

    copConstraint_.computeConstantPart();
  }

  void HumanoidWalkgen::setRightFootCopSSHull(const Hull& hull)
  {
    assert(hull.p.size()==4);
    leftFootModel_.setCopSSHull(hull);

    copConstraint_.computeConstantPart();
  }

  //TODO: delete DS setters
  void HumanoidWalkgen::setLeftFootCopDSHull(const Hull &hull)
  {
    assert(hull.p.size()==4);
    leftFootModel_.setCopDSHull(hull);

    copConstraint_.computeConstantPart();
  }

  void HumanoidWalkgen::setRightFootCopDSHull(const Hull &hull)
  {
    assert(hull.p.size()==4);
    leftFootModel_.setCopDSHull(hull);

    copConstraint_.computeConstantPart();
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
    leftFootModel_.setStateX(state);
  }

  void HumanoidWalkgen::setLeftFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    leftFootModel_.setStateY(state);
  }

  void HumanoidWalkgen::setLeftFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    leftFootModel_.setStateZ(state);
  }

  void HumanoidWalkgen::setRightFootStateX(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateX(state);
  }

  void HumanoidWalkgen::setRightFootStateY(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateY(state);
  }

  void HumanoidWalkgen::setRightFootStateZ(const VectorX& state)
  {
    assert(state==state);
    assert(state.size()==2);
    assert(state(1)==1.0);
    rightFootModel_.setStateZ(state);
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
    leftFootModel_.setMaxHeight(
          leftFootMaxHeight);
  }
  void HumanoidWalkgen::setRightFootMaxHeight(
      Scalar rightFootMaxHeight)
  {
    rightFootModel_.setMaxHeight(
          rightFootMaxHeight);
  }

  void HumanoidWalkgen::setLeftFootYawUpperBound(Scalar leftFootYawUpperBound)
  {
    leftFootModel_.setHipYawUpperBound(
          leftFootYawUpperBound);
  }
  void HumanoidWalkgen::setLeftFootYawLowerBound(Scalar leftFootYawLowerBound)
  {
    leftFootModel_.setHipYawLowerBound(
          leftFootYawLowerBound);
  }
  void HumanoidWalkgen::setRightFootYawUpperBound(Scalar rightFootYawUpperBound)
  {
    rightFootModel_.setHipYawUpperBound(
          rightFootYawUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawLowerBound(Scalar rightFootYawLowerBound)
  {
    rightFootModel_.setHipYawLowerBound(
          rightFootYawLowerBound);
  }

  void HumanoidWalkgen::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    leftFootModel_.setHipYawSpeedUpperBound(
          leftFootYawSpeedUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    rightFootModel_.setHipYawSpeedUpperBound(
          rightFootYawSpeedUpperBound);
  }

  void HumanoidWalkgen::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    leftFootModel_.setHipYawAccelerationUpperBound(
          leftFootYawAccelerationUpperBound);
  }
  void HumanoidWalkgen::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    rightFootModel_.setHipYawAccelerationUpperBound(
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
