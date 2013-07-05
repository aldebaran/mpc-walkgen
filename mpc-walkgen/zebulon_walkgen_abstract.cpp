#include "zebulon_walkgen_abstract.h"
#include "../src/zebulon_walkgen.h"

using namespace MPCWalkgen;

ZebulonWalkgenAbstract::ZebulonWalkgenAbstract()
:walkgen(NULL)
{
  walkgen = new ZebulonWalkgen();
}

ZebulonWalkgenAbstract::~ZebulonWalkgenAbstract()
{
  if (walkgen!=NULL)
  {
    delete walkgen;
  }
}

void ZebulonWalkgenAbstract::setNbSamples(int nbSamples)
{
  walkgen->setNbSamples(nbSamples);
}

void ZebulonWalkgenAbstract::setSamplingPeriod(Scalar samplingPeriod)
{
  walkgen->setSamplingPeriod(samplingPeriod);
}

void ZebulonWalkgenAbstract::useLipModel2(bool use)
{
  walkgen->useLipModel2(use);
}

void ZebulonWalkgenAbstract::setGravity(const Vector3& gravity)
{
  walkgen->setGravity(gravity);
}

void ZebulonWalkgenAbstract::setBaseHull(const std::vector<Vector3> p)
{
  walkgen->setBaseHull(Hull(p));
}

void ZebulonWalkgenAbstract::setComHeight(Scalar comHeight)
{
  walkgen->setComHeight(comHeight);
}

void ZebulonWalkgenAbstract::setUpperComHeight(Scalar comHeight)
{
  walkgen->setUpperComHeight(comHeight);
}

void ZebulonWalkgenAbstract::setLowerComHeight(Scalar comHeight)
{
  walkgen->setLowerComHeight(comHeight);
}

void ZebulonWalkgenAbstract::setUpperMass(Scalar mass)
{
  walkgen->setUpperMass(mass);
}

void ZebulonWalkgenAbstract::setLowerMass(Scalar mass)
{
  walkgen->setLowerMass(mass);
}

void ZebulonWalkgenAbstract::setVelRefInWorldFrame(const VectorX& velRef)
{
  walkgen->setVelRefInWorldFrame(velRef);
}

void ZebulonWalkgenAbstract::setPosRefInWorldFrame(const VectorX& posRef)
{
  walkgen->setPosRefInWorldFrame(posRef);
}
void ZebulonWalkgenAbstract::setCopRefInLocalFrame(const VectorX& copRef)
{
  walkgen->setCopRefInLocalFrame(copRef);
}

void ZebulonWalkgenAbstract::setBaseVelLimit(Scalar limit)
{
  walkgen->setBaseVelLimit(limit);
}

void ZebulonWalkgenAbstract::setBaseAccLimit(Scalar limit)
{
  walkgen->setBaseAccLimit(limit);
}

void ZebulonWalkgenAbstract::setBaseJerkLimit(Scalar limit)
{
  walkgen->setBaseJerkLimit(limit);
}

void ZebulonWalkgenAbstract::setBaseStateX(const VectorX& state)
{
  walkgen->setBaseStateX(state);
}

void ZebulonWalkgenAbstract::setBaseStateY(const VectorX& state)
{
  walkgen->setBaseStateY(state);
}

void ZebulonWalkgenAbstract::setComStateX(const VectorX& state)
{
  walkgen->setComStateX(state);
}

void ZebulonWalkgenAbstract::setComStateY(const VectorX& state)
{
  walkgen->setComStateY(state);
}

void ZebulonWalkgenAbstract::setWeightings(const Weighting& weighting)
{
  walkgen->setWeightings(weighting);
}

void ZebulonWalkgenAbstract::setConfig(const Config& config)
{
  walkgen->setConfig(config);
}

void ZebulonWalkgenAbstract::solve(Scalar feedBackPeriod)
{
  walkgen->solve(feedBackPeriod);
}

const VectorX& ZebulonWalkgenAbstract::getBaseStateX()
{
  return walkgen->getBaseStateX();
}

const VectorX& ZebulonWalkgenAbstract::getBaseStateY()
{
  return walkgen->getBaseStateY();
}

const VectorX& ZebulonWalkgenAbstract::getComStateX()
{
  return walkgen->getComStateX();
}

const VectorX& ZebulonWalkgenAbstract::getComStateY()
{
  return walkgen->getComStateY();
}

