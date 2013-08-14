#include <mpc-walkgen/zebulon_walkgen_abstract.h>
#include "zebulon_walkgen.h"

using namespace MPCWalkgen;

ZebulonWalkgenImpl::Weighting::Weighting()
  :velocityTracking(0.0)
  ,positionTracking(0.0)
  ,copCentering(0.0)
  ,jerkMinimization(0.0)
{}

ZebulonWalkgenImpl::Config::Config()
  :withCopConstraints(true)
  ,withComConstraints(true)
  ,withBaseMotionConstraints(true)
{}

ZebulonWalkgenImpl::ZebulonWalkgenImpl()
  :walkgen(NULL)
{
  walkgen = new ZebulonWalkgen();
}

ZebulonWalkgenImpl::~ZebulonWalkgenImpl()
{
  if (walkgen!=NULL)
  {
    delete walkgen;
  }
}

void ZebulonWalkgenImpl::setNbSamples(int nbSamples)
{
  walkgen->setNbSamples(nbSamples);
}

void ZebulonWalkgenImpl::setSamplingPeriod(Scalar samplingPeriod)
{
  walkgen->setSamplingPeriod(samplingPeriod);
}

void ZebulonWalkgenImpl::setGravity(const Vector3& gravity)
{
  walkgen->setGravity(gravity);
}

void ZebulonWalkgenImpl::setBaseCopHull(const std::vector<Vector3> p)
{
  walkgen->setBaseCopHull(Hull(p));
}

void ZebulonWalkgenImpl::setBaseComHull(const std::vector<Vector3> p)
{
  walkgen->setBaseComHull(Hull(p));
}

void ZebulonWalkgenImpl::setComBodyHeight(Scalar comHeight)
{
  walkgen->setComBodyHeight(comHeight);
}

void ZebulonWalkgenImpl::setComBaseHeight(Scalar comHeight)
{
  walkgen->setComBaseHeight(comHeight);
}

void ZebulonWalkgenImpl::setBodyMass(Scalar mass)
{
  walkgen->setBodyMass(mass);
}

void ZebulonWalkgenImpl::setBaseMass(Scalar mass)
{
  walkgen->setBaseMass(mass);
}

void ZebulonWalkgenImpl::setWheelToBaseDistance(Scalar dist)
{
  walkgen->setWheelToBaseDistance(dist);
}

void ZebulonWalkgenImpl::setAngleWheelToBaseCom(Scalar angle)
{
  walkgen->setAngleWheelToBaseCom(angle);
}

void ZebulonWalkgenImpl::setVelRefInWorldFrame(const VectorX& velRef)
{
  walkgen->setVelRefInWorldFrame(velRef);
}

void ZebulonWalkgenImpl::setPosRefInWorldFrame(const VectorX& posRef)
{
  walkgen->setPosRefInWorldFrame(posRef);
}
void ZebulonWalkgenImpl::setCopRefInLocalFrame(const VectorX& copRef)
{
  walkgen->setCopRefInLocalFrame(copRef);
}

void ZebulonWalkgenImpl::setBaseVelLimit(Scalar limit)
{
  walkgen->setBaseVelLimit(limit);
}

void ZebulonWalkgenImpl::setBaseAccLimit(Scalar limit)
{
  walkgen->setBaseAccLimit(limit);
}

void ZebulonWalkgenImpl::setBaseJerkLimit(Scalar limit)
{
  walkgen->setBaseJerkLimit(limit);
}

void ZebulonWalkgenImpl::setBaseStateX(const VectorX& state)
{
  walkgen->setBaseStateX(state);
}

void ZebulonWalkgenImpl::setBaseStateY(const VectorX& state)
{
  walkgen->setBaseStateY(state);
}

void ZebulonWalkgenImpl::setBaseStateRoll(const VectorX& state)
{
  walkgen->setBaseStateRoll(state);
}

void ZebulonWalkgenImpl::setBaseStatePitch(const VectorX& state)
{
  walkgen->setBaseStatePitch(state);
}

void ZebulonWalkgenImpl::setComStateX(const VectorX& state)
{
  walkgen->setComStateX(state);
}

void ZebulonWalkgenImpl::setComStateY(const VectorX& state)
{
  walkgen->setComStateY(state);
}

void ZebulonWalkgenImpl::setWeightings(const MPCWalkgen::Weighting& weighting)
{
  walkgen->setWeightings(weighting);
}

void ZebulonWalkgenImpl::setConfig(const MPCWalkgen::Config& config)
{
  walkgen->setConfig(config);
}

bool ZebulonWalkgenImpl::solve(Scalar feedBackPeriod)
{
  return walkgen->solve(feedBackPeriod);
}

const VectorX& ZebulonWalkgenImpl::getBaseStateX() const
{
  return walkgen->getBaseStateX();
}

const VectorX& ZebulonWalkgenImpl::getBaseStateY() const
{
  return walkgen->getBaseStateY();
}

const VectorX& ZebulonWalkgenImpl::getComStateX() const
{
  return walkgen->getComStateX();
}

const VectorX& ZebulonWalkgenImpl::getComStateY() const
{
  return walkgen->getComStateY();
}
