#include "humanoid_walkgen.h"

namespace MPCWalkgen
{
  HumanoidWalkgen::HumanoidWalkgen()
    :velTrackingObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,jerkMinObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,copCenteringObj_(lipModel_, leftFootModel_, rightFootModel_)
    ,qpoasesSolver_(1, 1)
    ,weighting_()
    ,config_()
  {
    computeConstantPart();
  }

  HumanoidWalkgen::~HumanoidWalkgen()
  {
  }

  void HumanoidWalkgen::setSamplingPeriod(Scalar samplingPeriod)
  {

  }

  void HumanoidWalkgen::setNbSamples(int nbSamples)
  {
    assert(nbSamples>0);

    lipModel_.setNbSamples(nbSamples);
    leftFootModel_.setNbSamples(nbSamples);
    rightFootModel_.setNbSamples(nbSamples);

    copCenteringObj_.computeConstantPart();
    velTrackingObj_.computeConstantPart();
    jerkMinObj_.computeConstantPart();
    copConstraint_.computeConstantPart();
    footConstraint_.computeConstantPart();

    computeConstantPart();
  }

  void HumanoidWalkgen::setStepPeriod(Scalar stepPeriod)
  {

  }

  void HumanoidWalkgen::setLeftFootKinematicHull(const std::vector<Vector3>& leftFootHull)
  {

  }

  void HumanoidWalkgen::setRightFootKinematicHull(const std::vector<Vector3>& rightFootHull)
  {

  }

  void HumanoidWalkgen::setSSCopHull(const std::vector<Vector3>& SSCopHull)
  {

  }

  void HumanoidWalkgen::setDSCopHull(const std::vector<Vector3>& DSCopHull)
  {

  }

  void HumanoidWalkgen::setVelRefInWorldFrame(const VectorX& velRef)
  {

  }

  void HumanoidWalkgen::setLeftFootStateX(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setLeftFootStateY(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setLeftFootStateZ(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setRightFootStateX(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setRightFootStateY(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setRightFootStateZ(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setComStateX(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setComStateY(const VectorX& state)
  {

  }

  void HumanoidWalkgen::setComStateZ(const VectorX& state)
  {

  }

  const VectorX& HumanoidWalkgen::getLeftFootStateX() const
  {

    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateY() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getLeftFootStateZ() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getRightFootStateX() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getRightFootStateY() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }
  const VectorX& HumanoidWalkgen::getRightFootStateZ() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getComStateX() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getComStateY() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
  }

  const VectorX& HumanoidWalkgen::getComStateZ() const
  {
    VectorX null;
    const VectorX& ret(null);
    return ret;
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
    int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int M2 = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;

    assert(feedBackPeriod>0);

    qpMatrix_.p.fill(0.0);
    qpMatrix_.bu.fill(std::numeric_limits<float>::max());
    qpMatrix_.bl.fill(-std::numeric_limits<float>::max());
    qpMatrix_.xu.fill(std::numeric_limits<float>::max());
    qpMatrix_.xl.fill(-std::numeric_limits<float>::max());


  }


  void HumanoidWalkgen::computeConstantPart()
  {
    int N = lipModel_.getNbSamples();
    int M1 = config_.withCopConstraints? copConstraint_.getNbConstraints() : 0;
    int M2 = config_.withFeetConstraints? footConstraint_.getNbConstraints() : 0;
    int M = M1+M2;

  }
}
