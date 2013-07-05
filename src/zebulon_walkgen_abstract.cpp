#include <mpc-walkgen/zebulon_walkgen_abstract.h>
#include "zebulon_walkgen.h"

using namespace MPCWalkgen;

ZebulonWalkgenImp::ZebulonWalkgenImp()
:walkgen(NULL)
{
  walkgen = new ZebulonWalkgen();
}

ZebulonWalkgenImp::~ZebulonWalkgenImp()
{
  if (walkgen!=NULL)
  {
    delete walkgen;
  }
}

void ZebulonWalkgenImp::setNbSamples(int nbSamples)
{
  walkgen->setNbSamples(nbSamples);
}

void ZebulonWalkgenImp::setSamplingPeriod(Scalar samplingPeriod)
{
  walkgen->setSamplingPeriod(samplingPeriod);
}

void ZebulonWalkgenImp::setGravity(const Vector3& gravity)
{
  walkgen->setGravity(gravity);
}

void ZebulonWalkgenImp::setBaseHull(const std::vector<Vector3> p)
{
  walkgen->setBaseHull(Hull(p));
}

void ZebulonWalkgenImp::setComHeight(Scalar comHeight)
{
  walkgen->setComHeight(comHeight);
}

void ZebulonWalkgenImp::setVelRefInWorldFrame(const VectorX& velRef)
{
  walkgen->setVelRefInWorldFrame(velRef);
}

void ZebulonWalkgenImp::setPosRefInWorldFrame(const VectorX& posRef)
{
  walkgen->setPosRefInWorldFrame(posRef);
}
void ZebulonWalkgenImp::setCopRefInLocalFrame(const VectorX& copRef)
{
  walkgen->setCopRefInLocalFrame(copRef);
}

void ZebulonWalkgenImp::setBaseVelLimit(Scalar limit)
{
  walkgen->setBaseVelLimit(limit);
}

void ZebulonWalkgenImp::setBaseAccLimit(Scalar limit)
{
  walkgen->setBaseAccLimit(limit);
}

void ZebulonWalkgenImp::setBaseJerkLimit(Scalar limit)
{
  walkgen->setBaseJerkLimit(limit);
}

void ZebulonWalkgenImp::setBaseStateX(const VectorX& state)
{
  walkgen->setBaseStateX(state);
}

void ZebulonWalkgenImp::setBaseStateY(const VectorX& state)
{
  walkgen->setBaseStateY(state);
}

void ZebulonWalkgenImp::setComStateX(const VectorX& state)
{
  walkgen->setComStateX(state);
}

void ZebulonWalkgenImp::setComStateY(const VectorX& state)
{
  walkgen->setComStateY(state);
}

void ZebulonWalkgenImp::setWeightings(const Weighting& weighting)
{
  walkgen->setWeightings(weighting);
}

void ZebulonWalkgenImp::setConfig(const Config& config)
{
  walkgen->setConfig(config);
}

void ZebulonWalkgenImp::solve(Scalar feedBackPeriod)
{
  walkgen->solve(feedBackPeriod);
}

const VectorX& ZebulonWalkgenImp::getBaseStateX()
{
  return walkgen->getBaseStateX();
}

const VectorX& ZebulonWalkgenImp::getBaseStateY()
{
  return walkgen->getBaseStateY();
}

const VectorX& ZebulonWalkgenImp::getComStateX()
{
  return walkgen->getComStateX();
}

const VectorX& ZebulonWalkgenImp::getComStateY()
{
  return walkgen->getComStateY();
}

