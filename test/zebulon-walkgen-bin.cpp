#include <mpc-walkgen/zebulon_walkgen_abstract.h>
#include <iostream>
#include <qi/os.hpp>

using namespace MPCWalkgen;

int main(void)
{

  int nbSamples = 10;

  ZebulonWalkgenImpl walkgen;

  walkgen.setNbSamples(nbSamples);
  walkgen.setSamplingPeriod(0.16);
  walkgen.setComBodyHeight(0.76);
  walkgen.setComBaseHeight(0.13);
  walkgen.setBodyMass(12.5);
  walkgen.setBaseMass(17.5);

  Scalar copLimitMin = 0.01*.2;
  Scalar copLimitMax = 0.015*.2;
  std::vector<Vector3> p(8);
  p[0] = Vector3(copLimitMin, copLimitMin, 0.0);
  p[1] = Vector3(0.0, copLimitMax, 0.0);
  p[2] = Vector3(-copLimitMin, copLimitMin, 0.0);
  p[3] = Vector3(-copLimitMax, 0.0, 0.0);
  p[4] = Vector3(-copLimitMin, -copLimitMin, 0.0);
  p[5] = Vector3(0.0, -copLimitMax, 0.0);
  p[6] = Vector3(copLimitMin, -copLimitMin, 0.0);
  p[7] = Vector3(copLimitMax, 0.0, 0.0);
  walkgen.setBaseCopHull(p);
  walkgen.setBaseComHull(p);

  Weighting weighting;
  weighting.copCentering = 1.0;
  weighting.velocityTracking = 0.01;
  weighting.positionTracking = 0.01;
  weighting.jerkMinimization = 0.00001;
  weighting.tiltMinimization = 100.0;
  walkgen.setWeightings(weighting);

  Config config;
  config.withBaseMotionConstraints = true;
  config.withComConstraints = true;
  config.withCopConstraints = false;
  walkgen.setConfig(config);

  VectorX velRef(2*nbSamples);
  velRef.segment(0, nbSamples).fill(0.0);
  velRef.segment(nbSamples, nbSamples).fill(0.0);
  walkgen.setVelRefInWorldFrame(velRef);

  VectorX posRef(2*nbSamples);
  posRef.fill(0.0);
  walkgen.setPosRefInWorldFrame(posRef);

  VectorX copRef(2*nbSamples);
  copRef.segment(0, nbSamples).fill(0.0);
  copRef.segment(nbSamples, nbSamples).fill(0.0);
  walkgen.setCopRefInLocalFrame(copRef);

  VectorX baseState(4);
  baseState(0) = 0.0;
  baseState(1) = 0.0;
  baseState(2) = 0.0;
  baseState(3) = 1.0;
  walkgen.setBaseStateX(baseState);

  walkgen.setBaseVelLimit(2.0);
  walkgen.setBaseAccLimit(2.0);
  walkgen.setBaseJerkLimit(10.0);

  Scalar samplingFeedback = 0.02;
  Scalar f = 1.0;
  for(Scalar t=0.0; t<6.0*f; t+=samplingFeedback)
  {
    Scalar ddt=0.0;
    if (t<0.5*f)
    {
      ddt = 0.0;
    }
    else if (t<1.0*f)
    {
      ddt = 4.0;
    }
    else if (t<3.0*f)
    {
      ddt = -2.0;
    }
    else if (t<3.5*f)
    {
      ddt = 4.0;
    }
    else
    {
      ddt = 0.0;
    }

    VectorX baseState(4);
    baseState(0) = 0.0;
    baseState(1) = 0.0;
    baseState(2) = ddt;
    baseState(3) = 1.0;
    walkgen.setBaseStateRoll(baseState);

    qi::os::timeval t1, t2;
    qi::os::gettimeofday(&t1);

    walkgen.solve(samplingFeedback);

    qi::os::gettimeofday(&t2);
    double total =
      (static_cast<double>(t2.tv_sec-t1.tv_sec)
       +0.000001*static_cast<double>((t2.tv_usec-t1.tv_usec)))
      /static_cast<double>(1);

    std::cout << ddt << "\t\t" << walkgen.getBaseStateX()(2) << "\t\t" <<walkgen.getComStateX()(2)
              << "\t\t"  << walkgen.getBaseStateX()(0) << "\t\t" <<walkgen.getComStateX()(0)
              << std::endl;
    //std::cout << "Solving time : " << floor(1000000*total)/1000 << " ms" << std::endl;
  }

  return 0;
}
