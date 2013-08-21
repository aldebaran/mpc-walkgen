#include <mpc-walkgen/humanoid_walkgen_abstract.h>
#include "humanoid_walkgen.h"

namespace MPCWalkgen
{
  HumanoidWalkgenImpl::Weighting::Weighting()
    :velocityTracking(0.0)
    ,copCentering(0.0)
    ,jerkMinimization(0.0)
  {}

  HumanoidWalkgenImpl::Config::Config()
    :withCopConstraints(true)
    ,withFeetConstraints(true)
  {}

  HumanoidWalkgenImpl::HumanoidWalkgenImpl()
    :walkgen(NULL)
  {
    walkgen = new HumanoidWalkgen();
  }

  HumanoidWalkgenImpl::~HumanoidWalkgenImpl()
  {
    if (walkgen!=NULL)
    {
      delete walkgen;
    }
  }

  void HumanoidWalkgenImpl::setNbSamples(int nbSamples)
  {
    walkgen->setNbSamples(nbSamples);
  }

  void HumanoidWalkgenImpl::setSamplingPeriod(Scalar samplingPeriod)
  {
    walkgen->setSamplingPeriod(samplingPeriod);
  }

  void HumanoidWalkgenImpl::setStepPeriod(Scalar stepPeriod)
  {
    walkgen->setStepPeriod(stepPeriod);
  }

  void HumanoidWalkgenImpl::setLeftFootKinematicHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setLeftFootKinematicHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setRightFootKinematicHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setRightFootKinematicHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setLeftFootCopSSHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setLeftFootCopSSHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setRightFootCopSSHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setRightFootCopSSHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setLeftFootCopDSHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setLeftFootCopDSHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setRightFootCopDSHull(
      const std::vector<Vector3>& hull)
  {
    walkgen->setRightFootCopDSHull(Hull(hull));
  }

  void HumanoidWalkgenImpl::setVelRefInWorldFrame(const VectorX& velRef)
  {
    walkgen->setVelRefInWorldFrame(velRef);
  }

  void HumanoidWalkgenImpl::setLeftFootStateX(const VectorX& state)
  {
    walkgen->setLeftFootStateX(state);
  }

  void HumanoidWalkgenImpl::setLeftFootStateY(const VectorX& state)
  {
    walkgen->setLeftFootStateY(state);
  }

  void HumanoidWalkgenImpl::setLeftFootStateZ(const VectorX& state)
  {
    walkgen->setLeftFootStateZ(state);
  }

  void HumanoidWalkgenImpl::setRightFootStateX(const VectorX& state)
  {
    walkgen->setRightFootStateX(state);
  }

  void HumanoidWalkgenImpl::setRightFootStateY(const VectorX& state)
  {
    walkgen->setRightFootStateY(state);
  }

  void HumanoidWalkgenImpl::setRightFootStateZ(const VectorX& state)
  {
    walkgen->setRightFootStateZ(state);
  }

  void HumanoidWalkgenImpl::setComStateX(const VectorX& state)
  {
    walkgen->setComStateX(state);
  }

  void HumanoidWalkgenImpl::setComStateY(const VectorX& state)
  {
    walkgen->setComStateY(state);
  }

  void HumanoidWalkgenImpl::setComStateZ(const VectorX& state)
  {
    walkgen->setComStateZ(state);
  }

  const VectorX& HumanoidWalkgenImpl::getLeftFootStateX() const
  {
    return walkgen->getLeftFootStateX();
  }
  const VectorX& HumanoidWalkgenImpl::getLeftFootStateY() const
  {
    return walkgen->getLeftFootStateY();
  }
  const VectorX& HumanoidWalkgenImpl::getLeftFootStateZ() const
  {
    return walkgen->getLeftFootStateZ();
  }
  const VectorX& HumanoidWalkgenImpl::getRightFootStateX() const
  {
    return walkgen->getRightFootStateX();
  }
  const VectorX& HumanoidWalkgenImpl::getRightFootStateY() const
  {
    return walkgen->getRightFootStateY();
  }
  const VectorX& HumanoidWalkgenImpl::getRightFootStateZ() const
  {
    return walkgen->getRightFootStateZ();
  }
  const VectorX& HumanoidWalkgenImpl::getComStateX() const
  {
    return walkgen->getComStateX();
  }
  const VectorX& HumanoidWalkgenImpl::getComStateY() const
  {
    return walkgen->getComStateY();
  }
  const VectorX& HumanoidWalkgenImpl::getComStateZ() const
  {
    return walkgen->getComStateZ();
  }

  void HumanoidWalkgenImpl::setLeftFootMaxHeight(Scalar leftFootMaxHeight)
  {
    walkgen->setLeftFootMaxHeight(leftFootMaxHeight);
  }
  void HumanoidWalkgenImpl::setRightFootMaxHeight(Scalar rightFootMaxHeight)
  {
    walkgen->setRightFootMaxHeight(rightFootMaxHeight);
  }

  void HumanoidWalkgenImpl::setLeftFootYawUpperBound(Scalar leftFootYawUpperBound)
  {
    walkgen->setLeftFootYawUpperBound(leftFootYawUpperBound);
  }
  void HumanoidWalkgenImpl::setLeftFootYawLowerBound(Scalar leftFootYawLowerBound)
  {
    walkgen->setLeftFootYawLowerBound(leftFootYawLowerBound);
  }
  void HumanoidWalkgenImpl::setRightFootYawUpperBound(Scalar rightFootYawUpperBound)
  {
    walkgen->setRightFootYawUpperBound(rightFootYawUpperBound);
  }
  void HumanoidWalkgenImpl::setRightFootYawLowerBound(Scalar rightFootYawLowerBound)
  {
    walkgen->setRightFootYawLowerBound(rightFootYawLowerBound);
  }

  void HumanoidWalkgenImpl::setLeftFootYawSpeedUpperBound(
      Scalar leftFootYawSpeedUpperBound)
  {
    walkgen->setLeftFootYawSpeedUpperBound(leftFootYawSpeedUpperBound);
  }
  void HumanoidWalkgenImpl::setRightFootYawSpeedUpperBound(
      Scalar rightFootYawSpeedUpperBound)
  {
    walkgen->setRightFootYawSpeedUpperBound(rightFootYawSpeedUpperBound);
  }

  void HumanoidWalkgenImpl::setLeftFootYawAccelerationUpperBound(
      Scalar leftFootYawAccelerationUpperBound)
  {
    walkgen->setLeftFootYawAccelerationUpperBound(leftFootYawAccelerationUpperBound);
  }
  void HumanoidWalkgenImpl::setRightFootYawAccelerationUpperBound(
      Scalar rightFootYawAccelerationUpperBound)
  {
    walkgen->setRightFootYawAccelerationUpperBound(rightFootYawAccelerationUpperBound);
  }

  void HumanoidWalkgenImpl::setWeightings(const Weighting& weighting)
  {
    walkgen->setWeightings(weighting);
  }

  void HumanoidWalkgenImpl::setConfig(const Config& config)
  {
    walkgen->setConfig(config);
  }


  void HumanoidWalkgenImpl::solve(Scalar feedBackPeriod)
  {
    walkgen->solve(feedBackPeriod);
  }
}
