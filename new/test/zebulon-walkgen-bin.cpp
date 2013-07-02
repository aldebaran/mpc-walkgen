#include <new/mpc-walkgen/zebulon_walkgen_abstract.h>
#include <iostream>
#include <qi/os.hpp>

using namespace MPCWalkgen;

int main(void)
{

  int nbSamples = 10;

  ZebulonWalkgenAbstract walkgen;

  walkgen.setNbSamples(nbSamples);
  walkgen.setSamplingPeriod(0.16) ;
  walkgen.setComHeight(0.45);

  Scalar copLimitMin = 0.01;
  Scalar copLimitMax = 0.015;
  std::vector<Vector3> p(8);
  p[0] = Vector3(copLimitMin, copLimitMin, 0.0);
  p[1] = Vector3(0.0, copLimitMax, 0.0);
  p[2] = Vector3(-copLimitMin, copLimitMin, 0.0);
  p[3] = Vector3(-copLimitMax, 0.0, 0.0);
  p[4] = Vector3(-copLimitMin, -copLimitMin, 0.0);
  p[5] = Vector3(0.0, -copLimitMax, 0.0);
  p[6] = Vector3(copLimitMin, -copLimitMin, 0.0);
  p[7] = Vector3(copLimitMax, 0.0, 0.0);
  walkgen.setBaseHull(p);

  Weighting weighting;
  weighting.copCentering = 10.0;
  weighting.velocityTracking = 1.0;
  weighting.positionTracking = 0.0;
  weighting.jerkMinimization = 0.001;
  walkgen.setWeightings(weighting);

  Config config;
  config.withBaseMotionConstraints = true;
  config.withComConstraints = true;
  config.withCopConstraints = true;
  walkgen.setConfig(config);

  VectorX velRef(2*nbSamples);
  velRef.segment(0, nbSamples).fill(1.5);
  velRef.segment(nbSamples, nbSamples).fill(0.0);
  walkgen.setVelRefInWorldFrame(velRef);

  VectorX posRef(2*nbSamples);
  posRef.fill(0.0);
  walkgen.setPosRefInWorldFrame(posRef);

  VectorX copRef(2*nbSamples);
  copRef.fill(0.0);
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
  for(Scalar t=0.0; t<15.0; t+=samplingFeedback)
  {
    qi::os::timeval t1, t2;
    qi::os::gettimeofday(&t1);

    walkgen.solve(samplingFeedback);

    qi::os::gettimeofday(&t2);
    double total =
      (static_cast<double>(t2.tv_sec-t1.tv_sec)
       +0.000001*static_cast<double>((t2.tv_usec-t1.tv_usec)))
      /static_cast<double>(1);
    std::cout << "Solving time : " << floor(1000000*total)/1000 << " ms" << std::endl;
  }

  return 0;
}
